"""Scripted pick-and-place and tending-cell primitives for SOMA (v0.4).

PURE module (no ROS): runs under pytest on any machine, no hardware needed.

Builds safe, 50 Hz-verified pick-and-place sequences along each arm's
planar lane (y = +-81 mm):
  approach -> descend -> grasp -> lift -> move -> descend -> release -> retreat

Key geometric findings from kinematics.py (2026-11-27):
  - Above z = 75 mm a central hole opens under the disc axis, so a
    pick height of 50 mm (e.g. 30 mm deck + 20 mm grasp) with a 70 mm
    lift height keeps both lanes continuous across their pockets.
  - With up to +-15 deg tool tilt, the left lane covers [-70, 0, +70] mm
    and the right lane covers [-130, -60, +10] mm (70 mm pitch).
"""
import math
from dataclasses import dataclass

from .kinematics import (
    ELBOW_CURL_FLOOR_RAD,
    JOINTS,
    MAX_BASE_TORQUE_KGCM,
    MIN_CLEARANCE_Z_M,
    SIDES,
    check_trajectory_safety,
    load_dimensions,
    solve_pose,
)
from .primitives import HOME, settle_time_s

DEFAULT_PICK_Z_MM = 40.0
DEFAULT_LIFT_Z_MM = 60.0
DEFAULT_GRIP_OPEN = 1.0
DEFAULT_GRIP_CLOSED = 0.15
DEFAULT_MAX_TILT_DEG = 15.0

# 3 pockets at 70 mm pitch per lane, verified reachable at both 50 and 70 mm.
DEFAULT_POCKETS_MM: dict[str, tuple[float, float, float]] = {
    'left': (-70.0, 0.0, 70.0),
    'right': (-130.0, -60.0, 10.0),
}


@dataclass(frozen=True)
class TendingStep:
    label: str
    pose: dict[str, float]
    dwell_s: float


def _arm_home(side: str) -> dict[str, float]:
    prefix = f'{side}_arm_'
    return {k: v for k, v in HOME.items() if k.startswith(prefix)}


def _q4(side: str, pose: dict[str, float]) -> tuple[float, float, float, float]:
    return tuple(pose[f'{side}_arm_{j}_joint'] for j in JOINTS)


def _tilt_candidates(max_tilt_deg: float) -> tuple[float, ...]:
    if not math.isfinite(max_tilt_deg) or max_tilt_deg < 0.0:
        raise ValueError(f'max_tilt_deg must be >= 0: {max_tilt_deg}')
    steps = [0.0]
    deg = 5.0
    while deg <= max_tilt_deg + 1e-9:
        steps.extend((-deg, deg))
        deg += 5.0
    return tuple(steps)


def solve_with_tilt(x_mm: float,
                    z_mm: float,
                    side: str,
                    dims: dict[str, float] | None = None,
                    finger: float = 0.0,
                    max_tilt_deg: float = DEFAULT_MAX_TILT_DEG,
                    min_z_m: float = MIN_CLEARANCE_Z_M,
                    max_torque_kgcm: float = MAX_BASE_TORQUE_KGCM,
                    min_elbow_rad: float = ELBOW_CURL_FLOOR_RAD
                    ) -> tuple[float, dict[str, float]]:
    """Solve IK for (x_mm, z_mm) trying smallest tool tilts up to max_tilt_deg."""
    if side not in SIDES:
        raise ValueError(f'unknown side {side!r}: expected one of {SIDES}')
    if not (math.isfinite(x_mm) and math.isfinite(z_mm) and math.isfinite(finger)):
        raise ValueError('coordinates and finger must be finite')
    if not (0.0 <= finger <= 1.0):
        raise ValueError(f'finger must be in [0.0, 1.0], got {finger}')
    if dims is None:
        dims = load_dimensions()

    for tilt_deg in _tilt_candidates(max_tilt_deg):
        pose = solve_pose(
            (x_mm / 1000.0, z_mm / 1000.0),
            side,
            dims=dims,
            phi_tool=math.radians(tilt_deg),
            finger=finger,
            min_z_m=min_z_m,
            max_torque_kgcm=max_torque_kgcm,
            min_elbow_rad=min_elbow_rad,
        )
        if pose is not None:
            return tilt_deg, pose

    raise ValueError(
        f'({x_mm:g}, {z_mm:g}) mm on {side} lane is unreachable or unsafe '
        f'within +-{max_tilt_deg:g} deg tilt')


def _validate_params(pick_z_mm: float, lift_z_mm: float,
                     grip_open: float, grip_closed: float) -> None:
    if not all(math.isfinite(v) for v in (pick_z_mm, lift_z_mm, grip_open, grip_closed)):
        raise ValueError('tending parameters must be finite')
    if pick_z_mm < 10.0:
        raise ValueError(f'pick_z_mm ({pick_z_mm:g}) below 10 mm safety floor')
    if lift_z_mm <= pick_z_mm:
        raise ValueError(f'lift_z_mm ({lift_z_mm:g}) must exceed pick_z_mm ({pick_z_mm:g})')
    if not (0.0 <= grip_closed < grip_open <= 1.0):
        raise ValueError(
            f'require 0.0 <= grip_closed ({grip_closed}) < grip_open ({grip_open}) <= 1.0')


def _verified_transition(prev_pose: dict[str, float],
                         next_pose: dict[str, float],
                         side: str,
                         dims: dict[str, float],
                         label: str,
                         prev_label: str,
                         dwell_margin_s: float = 0.2) -> TendingStep:
    fault = check_trajectory_safety(_q4(side, prev_pose), _q4(side, next_pose), side, dims)
    if fault is not None:
        raise ValueError(f'unsafe transition {prev_label} -> {label}: {fault}')
    dwell = round(max(0.5, settle_time_s(next_pose, prev_pose) + dwell_margin_s), 2)
    return TendingStep(label=label, pose=next_pose, dwell_s=dwell)


def plan_pick_and_place(side: str,
                        src_x_mm: float,
                        dst_x_mm: float,
                        dims: dict[str, float] | None = None,
                        pick_z_mm: float = DEFAULT_PICK_Z_MM,
                        lift_z_mm: float = DEFAULT_LIFT_Z_MM,
                        grip_open: float = DEFAULT_GRIP_OPEN,
                        grip_closed: float = DEFAULT_GRIP_CLOSED,
                        max_tilt_deg: float = DEFAULT_MAX_TILT_DEG,
                        start_pose: dict[str, float] | None = None) -> list[TendingStep]:
    """Plan and 50 Hz-verify an 8-step pick-and-place between two lane pockets."""
    _validate_params(pick_z_mm, lift_z_mm, grip_open, grip_closed)
    if dims is None:
        dims = load_dimensions()

    _, src_high_open = solve_with_tilt(
        src_x_mm, lift_z_mm, side, dims, finger=grip_open, max_tilt_deg=max_tilt_deg)
    _, src_low_open = solve_with_tilt(
        src_x_mm, pick_z_mm, side, dims, finger=grip_open, max_tilt_deg=max_tilt_deg)
    src_low_closed = dict(src_low_open, **{f'{side}_arm_finger_l_joint': float(grip_closed)})
    src_high_closed = dict(src_high_open, **{f'{side}_arm_finger_l_joint': float(grip_closed)})

    _, dst_high_closed = solve_with_tilt(
        dst_x_mm, lift_z_mm, side, dims, finger=grip_closed, max_tilt_deg=max_tilt_deg)
    _, dst_low_closed = solve_with_tilt(
        dst_x_mm, pick_z_mm, side, dims, finger=grip_closed, max_tilt_deg=max_tilt_deg)
    dst_low_open = dict(dst_low_closed, **{f'{side}_arm_finger_l_joint': float(grip_open)})
    dst_high_open = dict(dst_high_closed, **{f'{side}_arm_finger_l_joint': float(grip_open)})

    waypoints = (
        ('approach', src_high_open),
        ('descend_pick', src_low_open),
        ('grasp', src_low_closed),
        ('lift', src_high_closed),
        ('move', dst_high_closed),
        ('descend_place', dst_low_closed),
        ('release', dst_low_open),
        ('retreat', dst_high_open),
    )

    steps: list[TendingStep] = []
    prev_pose = _arm_home(side) if start_pose is None else start_pose
    prev_label = 'start'
    for label, pose in waypoints:
        step = _verified_transition(prev_pose, pose, side, dims, label, prev_label)
        steps.append(step)
        prev_pose = pose
        prev_label = label
    return steps


def plan_tending_cycle(side: str,
                       input_x_mm: float | None = None,
                       machine_x_mm: float | None = None,
                       output_x_mm: float | None = None,
                       dims: dict[str, float] | None = None,
                       pick_z_mm: float = DEFAULT_PICK_Z_MM,
                       lift_z_mm: float = DEFAULT_LIFT_Z_MM,
                       grip_open: float = DEFAULT_GRIP_OPEN,
                       grip_closed: float = DEFAULT_GRIP_CLOSED,
                       max_tilt_deg: float = DEFAULT_MAX_TILT_DEG) -> list[TendingStep]:
    """Plan a full tending cycle: home -> load machine -> unload -> home."""
    if side not in SIDES:
        raise ValueError(f'unknown side {side!r}: expected one of {SIDES}')
    if dims is None:
        dims = load_dimensions()
    d_in, d_mach, d_out = DEFAULT_POCKETS_MM[side]
    in_x = d_in if input_x_mm is None else input_x_mm
    mach_x = d_mach if machine_x_mm is None else machine_x_mm
    out_x = d_out if output_x_mm is None else output_x_mm

    home = _arm_home(side)
    load_steps = plan_pick_and_place(
        side, in_x, mach_x, dims=dims,
        pick_z_mm=pick_z_mm, lift_z_mm=lift_z_mm,
        grip_open=grip_open, grip_closed=grip_closed,
        max_tilt_deg=max_tilt_deg, start_pose=home,
    )
    unload_steps = plan_pick_and_place(
        side, mach_x, out_x, dims=dims,
        pick_z_mm=pick_z_mm, lift_z_mm=lift_z_mm,
        grip_open=grip_open, grip_closed=grip_closed,
        max_tilt_deg=max_tilt_deg, start_pose=load_steps[-1].pose,
    )
    home_step = _verified_transition(
        unload_steps[-1].pose, home, side, dims, 'home', unload_steps[-1].label)
    return [TendingStep('home', home, 0.5)] + load_steps + unload_steps + [home_step]


def interleave_cycles(left_cycle: list[TendingStep],
                      right_cycle: list[TendingStep]) -> list[TendingStep]:
    """Run left cycle then right cycle sequentially so both arms never accelerate together."""
    return list(left_cycle) + list(right_cycle)
