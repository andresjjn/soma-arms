"""Bench tool logic for v0.3: touch (x, z) points along an arm lane.

PURE module (no ROS): runs under pytest on any machine, no hardware needed.

Plans a sequence of planar IK targets along one arm's lane (y = +-81 mm),
verifies every transition and unwind path at 50 Hz against plate clearance,
elbow curl floor, soft limits, and estimated base torque before moving,
steps one ENTER at a time, logs measured (x_mm, z_mm) into CSV, and
always returns the arm home on q, Ctrl-C, EOF, or any error.
"""
import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

from .kinematics import (
    ELBOW_CURL_FLOOR_RAD,
    JOINTS,
    MAX_BASE_TORQUE_KGCM,
    SIDES,
    check_trajectory_safety,
    load_dimensions,
    solve_pose,
)
from .primitives import HOME, settle_time_s

MIN_Z_MM = 10.0

DEFAULT_POINTS_MM: tuple[tuple[float, float], ...] = (
    (-30.0, 30.0),
    (0.0, 30.0),
    (30.0, 30.0),
)


@dataclass(frozen=True)
class TrialPlan:
    index: int
    x_mm: float
    z_mm: float
    phi_deg: float
    pose: dict[str, float]


@dataclass(frozen=True)
class TrialResult:
    trial: int
    side: str
    requested_x_mm: float
    requested_z_mm: float
    phi_deg: float
    reached_x_mm: float
    reached_z_mm: float
    dx_mm: float
    dz_mm: float
    error_mm: float


def arm_home(side: str) -> dict[str, float]:
    prefix = f'{side}_arm_'
    return {k: v for k, v in HOME.items() if k.startswith(prefix)}


def _q4(side: str, pose: dict[str, float]) -> tuple[float, float, float, float]:
    return tuple(pose[f'{side}_arm_{j}_joint'] for j in JOINTS)


def parse_points(raw: list[str] | None) -> list[tuple[float, float]]:
    """Parse 'x,z' strings in mm into finite [(x_mm, z_mm), ...]."""
    if not raw:
        return list(DEFAULT_POINTS_MM)
    points = []
    for item in raw:
        parts = item.split(',')
        if len(parts) != 2:
            raise ValueError(f'point {item!r} must be x,z in mm (e.g. -30,30)')
        x_mm, z_mm = float(parts[0]), float(parts[1])
        if not (math.isfinite(x_mm) and math.isfinite(z_mm)):
            raise ValueError(f'point {item!r} must be finite')
        points.append((x_mm, z_mm))
    return points


def parse_cli(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        prog='soma_touch_test',
        description=__doc__.split('\n')[0])
    ap.add_argument('--arm', choices=SIDES, default='left',
                    help='which arm lane to test (default: left)')
    ap.add_argument('--points', nargs='+', default=None,
                    help='target points as x,z in mm (default: -30,30 0,30 30,30)')
    ap.add_argument('--tilt', type=float, default=0.0,
                    help='tool tilt from vertical in degrees (default: 0)')
    ap.add_argument('--dwell', type=float, default=None,
                    help='override settle wait in seconds (default: auto from ramp)')
    ap.add_argument('--csv', type=Path, default=None,
                    help='output CSV path (default: calibration/touch_test_<stamp>_<arm>.csv)')
    known_flags = {'-h', '--help', '--arm', '--points', '--tilt', '--dwell', '--csv'}
    raw_points: list[str] = []
    filtered: list[str] = []
    i = 0
    while i < len(argv):
        tok = argv[i]
        if tok == '--points':
            i += 1
            while i < len(argv) and argv[i] not in known_flags:
                raw_points.append(argv[i])
                i += 1
            if not raw_points:
                ap.error('argument --points: expected at least one argument')
            continue
        filtered.append(tok)
        i += 1
    args = ap.parse_args(filtered)
    if args.points is None and raw_points:
        args.points = raw_points
    if not math.isfinite(args.tilt):
        ap.error('--tilt must be finite')
    if args.dwell is not None and (not math.isfinite(args.dwell) or args.dwell <= 0.0):
        ap.error('--dwell must be > 0')
    args.parsed_points = parse_points(args.points)
    return args


def plan_trials(side: str, points_mm: list[tuple[float, float]],
                dims: dict[str, float] | None = None,
                phi_deg: float = 0.0,
                margin: float = 0.1,
                min_z_mm: float = MIN_Z_MM,
                max_torque_kgcm: float = MAX_BASE_TORQUE_KGCM,
                min_elbow_rad: float = ELBOW_CURL_FLOOR_RAD) -> list[TrialPlan]:
    """Solve IK and verify 50 Hz trajectory safety for every step and unwind."""
    if not points_mm:
        raise ValueError('at least one target point is required')
    if not math.isfinite(phi_deg):
        raise ValueError('tilt must be finite')
    if dims is None:
        dims = load_dimensions()
    phi_rad = math.radians(phi_deg)
    min_z_m = min_z_mm / 1000.0
    trials: list[TrialPlan] = []
    bad: list[str] = []
    for idx, (x_mm, z_mm) in enumerate(points_mm, start=1):
        if not (math.isfinite(x_mm) and math.isfinite(z_mm)):
            bad.append(f'({x_mm}, {z_mm}): non-finite coordinate')
            continue
        if z_mm < min_z_mm:
            bad.append(f'({x_mm:g}, {z_mm:g}): below {min_z_mm:g} mm safety floor')
            continue
        pose = solve_pose(
            (x_mm / 1000.0, z_mm / 1000.0), side, dims,
            phi_tool=phi_rad, margin=margin,
            min_z_m=min_z_m, max_torque_kgcm=max_torque_kgcm,
            min_elbow_rad=min_elbow_rad,
        )
        if pose is None:
            bad.append(f'({x_mm:g}, {z_mm:g}): unreachable or unsafe')
        else:
            trials.append(TrialPlan(idx, x_mm, z_mm, phi_deg, pose))
    if bad:
        raise ValueError(f'unsafe or unreachable point(s) on {side} lane: ' + '; '.join(bad))

    # Verify every forward transition AND every abort-unwind path to home.
    home_q = _q4(side, arm_home(side))
    prev_q = home_q
    prev_label = 'home'
    for t in trials:
        curr_q = _q4(side, t.pose)
        label = f'trial {t.index} ({t.x_mm:g}, {t.z_mm:g})'
        fault = check_trajectory_safety(
            prev_q, curr_q, side, dims,
            min_z_m=min_z_m, max_torque_kgcm=max_torque_kgcm,
            min_elbow_rad=min_elbow_rad,
        )
        if fault is not None:
            raise ValueError(f'unsafe transition {prev_label} -> {label}: {fault}')
        unwind_fault = check_trajectory_safety(
            curr_q, home_q, side, dims,
            min_z_m=min_z_m, max_torque_kgcm=max_torque_kgcm,
            min_elbow_rad=min_elbow_rad,
        )
        if unwind_fault is not None:
            raise ValueError(f'unsafe unwind {label} -> home: {unwind_fault}')
        prev_q = curr_q
        prev_label = label
    return trials


def _parse_measurement(text: str) -> tuple[float, float] | str | None:
    s = text.strip().lower()
    if s in ('q', 'quit'):
        return 'quit'
    if s in ('s', 'skip'):
        return 'skip'
    parts = s.replace(',', ' ').split()
    if len(parts) != 2:
        return None
    try:
        x_mm, z_mm = float(parts[0]), float(parts[1])
    except ValueError:
        return None
    if not (math.isfinite(x_mm) and math.isfinite(z_mm)):
        return None
    return x_mm, z_mm


def _unwind_home(home: dict[str, float], walked: list[dict[str, float]],
                 publish, sleep, out, dwell_override: float | None) -> None:
    if len(walked) <= 1:
        return
    current = walked[-1]
    for pose in reversed(walked[:-1]):
        wait = dwell_override if dwell_override is not None else max(
            1.0, settle_time_s(pose, current) + 0.3)
        publish(pose)
        sleep(wait)
        current = pose
    out('arm back at home.')


def run_trials(trials: list[TrialPlan], side: str, publish, sleep, ask, out,
               dwell: float | None = None) -> list[TrialResult]:
    """Step through planned trials, recording measured positions safely."""
    home = arm_home(side)
    walked: list[dict[str, float]] = [home]
    results: list[TrialResult] = []
    total = len(trials)
    try:
        for t in trials:
            ans = ask(f'[{t.index}/{total}] {side} -> ({t.x_mm:+.1f}, {t.z_mm:+.1f}) mm '
                      f'(tilt {t.phi_deg:g} deg). ENTER to move, q to abort: ')
            if ans.strip().lower() in ('q', 'quit'):
                break
            wait = dwell if dwell is not None else max(
                1.0, settle_time_s(t.pose, walked[-1]) + 0.3)
            # Record before publishing so an interrupt during publish/sleep
            # still triggers a full unwind back to home.
            walked.append(t.pose)
            publish(t.pose)
            sleep(wait)

            while True:
                raw = ask('  measured (x_mm z_mm), s to skip, q to abort: ')
                parsed = _parse_measurement(raw)
                if parsed == 'quit':
                    return results
                if parsed == 'skip':
                    break
                if parsed is not None:
                    rx, rz = parsed
                    dx = rx - t.x_mm
                    dz = rz - t.z_mm
                    err = math.hypot(dx, dz)
                    results.append(TrialResult(
                        trial=t.index,
                        side=side,
                        requested_x_mm=t.x_mm,
                        requested_z_mm=t.z_mm,
                        phi_deg=t.phi_deg,
                        reached_x_mm=rx,
                        reached_z_mm=rz,
                        dx_mm=dx,
                        dz_mm=dz,
                        error_mm=err,
                    ))
                    out(f'  error: {err:.2f} mm (dx={dx:+.2f}, dz={dz:+.2f})')
                    break
                out('  enter two finite numbers in mm (e.g. -28.5 31.0), s, or q')
    except (EOFError, KeyboardInterrupt):
        out('\ninterrupted, returning arm home...')
        raise
    finally:
        _unwind_home(home, walked, publish, sleep, out, dwell)
    return results


def write_csv(path: Path, results: list[TrialResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'trial', 'side', 'requested_x_mm', 'requested_z_mm', 'phi_deg',
            'reached_x_mm', 'reached_z_mm', 'dx_mm', 'dz_mm', 'error_mm',
        ])
        for r in results:
            w.writerow([
                r.trial, r.side, f'{r.requested_x_mm:.2f}', f'{r.requested_z_mm:.2f}',
                f'{r.phi_deg:.2f}', f'{r.reached_x_mm:.2f}', f'{r.reached_z_mm:.2f}',
                f'{r.dx_mm:.2f}', f'{r.dz_mm:.2f}', f'{r.error_mm:.2f}',
            ])


def summarize(results: list[TrialResult]) -> str:
    if not results:
        return 'no measurements recorded.'
    errors = [r.error_mm for r in results]
    mean_err = sum(errors) / len(errors)
    max_err = max(errors)
    lines = [
        f'{len(results)} measurement(s): mean error {mean_err:.2f} mm, max {max_err:.2f} mm',
    ]
    by_target: dict[tuple[float, float], list[TrialResult]] = {}
    for r in results:
        by_target.setdefault((r.requested_x_mm, r.requested_z_mm), []).append(r)
    repeats = []
    for (tx, tz), group in by_target.items():
        if len(group) < 2:
            continue
        spread = max(
            math.hypot(a.reached_x_mm - b.reached_x_mm, a.reached_z_mm - b.reached_z_mm)
            for i, a in enumerate(group) for b in group[i + 1:])
        repeats.append(f'({tx:g}, {tz:g}) mm: {spread:.2f} mm ({len(group)} trials)')
    if repeats:
        lines.append('repeatability max spread: ' + '; '.join(repeats))
    return '\n'.join(lines)
