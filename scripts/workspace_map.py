#!/usr/bin/env python3
"""Where can each arm put a vertical gripper? The planar reach map of SOMA.

Both arms hang from the central box with their four big joints (yaw,
shoulder, elbow, wrist pitch) on parallel horizontal axes, so each arm is
a planar 4R chain in its own vertical plane, 62.3 mm outboard of the bench
centerline (mount_ly / 2 + disc_h). The fingertip center sits on the roll
axis and never leaves that plane: the work of each arm is a LANE, a strip
of the deck under its plane, and this script says how long that lane is.

Pure Python, no ROS. Inputs are the measured dimensions
(soma_description/config/dimensions.yaml) and the measured joint limits
(soma_driver SERVO_MAP, calibration of 2026-10-01). Output: a markdown
table of the reachable x intervals per lane for several deck heights, and
an SVG drawing of the deck in millimeters for the laser cutter.

    python3 scripts/workspace_map.py --deck 0 40 60 80 --grasp 20
    python3 scripts/workspace_map.py --deck 40 --svg docs/workspace_map.svg
    python3 scripts/workspace_map.py --flip elbow        # try a sign hypothesis

Frame, per arm plane: x forward (away from the column), z up, plate top at
z = 0, disc axis at (0, mount_h). A joint angle q rotates the hanging
direction by q, POSITIVE = BACKWARD, which is the sense the URDF encodes
on every one of the four parallel axes (axis +Y of the bench frame on
both arms). Only the yaw direction is verified on the metal (2026-10-01:
negative = forward); shoulder, elbow and wrist pitch are hypotheses until
the v0.2 sign check. `--flip <joint>` mirrors one joint's sense and its
limits, so the map can be redone the evening the signs are known.
"""
import argparse
import math
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
DIMENSIONS = REPO / 'soma_description' / 'config' / 'dimensions.yaml'
sys.path.insert(0, str(REPO / 'soma_driver'))
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

JOINTS = ('yaw', 'shoulder', 'elbow', 'wrist_pitch')
SIDES = ('left', 'right')
# Sign hypotheses in the model frame: +1 means the command sign is the
# URDF sign (positive = backward). Only yaw is verified on the metal.
SIGNS = {'yaw': 1, 'shoulder': 1, 'elbow': 1, 'wrist_pitch': 1}
VERIFIED = {'yaw': '2026-10-01, negative = forward, on the metal'}


def load_dimensions(path: Path = DIMENSIONS) -> dict[str, float]:
    """Every arm and bench dimension as {name: meters}."""
    doc = yaml.safe_load(Path(path).read_text())
    dims = {}
    for section in ('arm', 'bench'):
        for name, entry in doc[section]['dimensions'].items():
            dims[name] = float(entry['value'])
    dims['tool_len'] = dims['wrist_len'] + dims['gripper_base_h'] + dims['finger_len']
    return dims


def link_lengths(dims: dict[str, float]) -> tuple[float, float, float, float]:
    return (dims['clavicle_len'], dims['upper_len'], dims['fore_len'], dims['tool_len'])


def lane_y(side: str, dims: dict[str, float]) -> float:
    """Lateral position of the arm's plane in the bench frame, meters."""
    y = dims['mount_ly'] / 2 + dims['disc_h']
    return y if side == 'left' else -y


def limits(side: str, signs: dict[str, int] = SIGNS, margin: float = 0.1
           ) -> dict[str, tuple[float, float]]:
    """Joint limits in the MODEL frame, `margin` radians inside the band."""
    out = {}
    for joint in JOINTS:
        spec = SERVO_MAP[f'{side}_arm_{joint}_joint']
        lo, up = spec.lower, spec.upper
        if signs[joint] < 0:
            lo, up = -up, -lo
        out[joint] = (lo + margin, up - margin)
    return out


def fk(q: tuple[float, float, float, float], dims: dict[str, float]
       ) -> tuple[list[tuple[float, float]], float]:
    """Joint points D, S, E, W and the fingertip T, plus the tool angle."""
    pts = [(0.0, dims['mount_h'])]
    phi = 0.0
    for angle, length in zip(q, link_lengths(dims)):
        phi += angle
        x, z = pts[-1]
        pts.append((x - length * math.sin(phi), z - length * math.cos(phi)))
    return pts, phi


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def ik(target: tuple[float, float], phi_tool: float, q1: float,
       dims: dict[str, float]) -> list[tuple[float, float, float, float]]:
    """Both elbow branches reaching `target` with the tool at `phi_tool`.

    The yaw is given (the chain has one redundant joint); the shoulder
    and elbow solve the 2R problem to the wrist pitch axis and the wrist
    closes the tool angle. Unchecked against limits; empty if out of reach.
    """
    clav, a, b, tool = link_lengths(dims)
    wx = target[0] + tool * math.sin(phi_tool)
    wz = target[1] + tool * math.cos(phi_tool)
    sx = -clav * math.sin(q1)
    sz = dims['mount_h'] - clav * math.cos(q1)
    dx, dz = wx - sx, wz - sz
    c3 = (dx * dx + dz * dz - a * a - b * b) / (2 * a * b)
    if abs(c3) > 1.0:
        return []
    s3 = math.sqrt(max(0.0, 1.0 - c3 * c3))
    out = []
    for q3 in (math.atan2(s3, c3), math.atan2(-s3, c3)):
        k1, k2 = a + b * math.cos(q3), b * math.sin(q3)
        phi2 = math.atan2(-dx, -dz) - math.atan2(k2, k1)
        q2 = wrap(phi2 - q1)
        q4 = wrap(phi_tool - (q1 + q2 + q3))
        out.append((q1, q2, q3, q4))
    return out


def _inside(q, lim) -> bool:
    return all(lim[j][0] <= v <= lim[j][1] for j, v in zip(JOINTS, q))


def solve(target: tuple[float, float], phi_tool: float, side: str,
          dims: dict[str, float], signs: dict[str, int] = SIGNS,
          margin: float = 0.1, yaw_step_deg: float = 0.5
          ) -> tuple[float, float, float, float] | None:
    """A valid configuration in the COMMAND frame, or None.

    Scans the yaw through its band and keeps, among every branch inside
    the limits, the one closest to the hanging pose (smallest total bend).
    """
    lim = limits(side, signs, margin)
    best, best_cost = None, None
    lo, up = lim['yaw']
    steps = max(1, int(round((up - lo) / math.radians(yaw_step_deg))))
    for i in range(steps + 1):
        q1 = lo + (up - lo) * i / steps
        for q in ik(target, phi_tool, q1, dims):
            if not _inside(q, lim):
                continue
            cost = sum(abs(v) for v in q)
            if best is None or cost < best_cost:
                best, best_cost = q, cost
    if best is None:
        return None
    return tuple(v * signs[j] for j, v in zip(JOINTS, best))


def reach_runs(z_tip: float, side: str, dims: dict[str, float],
               signs: dict[str, int] = SIGNS, margin: float = 0.1,
               x_from_mm: int = -250, x_to_mm: int = 250, x_step_mm: int = 1,
               yaw_step_deg: float = 0.5, tilt_deg: float = 0.0
               ) -> list[tuple[int, int]]:
    """Contiguous x intervals (mm, + forward) where the fingertip can be
    placed at height `z_tip` (meters above the plate) with the gripper
    vertical, or within +-tilt_deg of vertical."""
    tilts = [0.0]
    if tilt_deg > 0:
        tilts = [math.radians(t) for t in range(-int(tilt_deg), int(tilt_deg) + 1, 5)]
        if 0.0 not in tilts:
            tilts.append(0.0)
    runs: list[list[int]] = []
    for x_mm in range(x_from_mm, x_to_mm + 1, x_step_mm):
        ok = any(solve((x_mm / 1000.0, z_tip), t, side, dims, signs, margin,
                       yaw_step_deg) is not None for t in tilts)
        if ok:
            if runs and runs[-1][1] == x_mm - x_step_mm:
                runs[-1][1] = x_mm
            else:
                runs.append([x_mm, x_mm])
    return [(a, b) for a, b in runs]


def pockets(run: tuple[int, int], pitch_mm: float, count: int,
            pocket_mm: float) -> list[float] | None:
    """`count` pocket centers at `pitch_mm`, centered on the run, or None
    when a pocket of `pocket_mm` would not fit inside the run."""
    a, b = run
    mid = (a + b) / 2
    centers = [mid + (i - (count - 1) / 2) * pitch_mm for i in range(count)]
    if centers[0] - pocket_mm / 2 < a or centers[-1] + pocket_mm / 2 > b:
        return None
    return centers


def longest(runs: list[tuple[int, int]]) -> tuple[int, int] | None:
    return max(runs, key=lambda r: r[1] - r[0]) if runs else None


def reach_table(decks_mm: list[float], grasp_mm: float, dims, signs, margin,
                x_step_mm=1, yaw_step_deg=0.5, tilt_deg=0.0) -> list[dict]:
    rows = []
    for deck in decks_mm:
        z_tip = (deck + grasp_mm) / 1000.0
        row = {'deck_mm': deck, 'z_tip_mm': deck + grasp_mm}
        for side in SIDES:
            row[side] = reach_runs(z_tip, side, dims, signs, margin,
                                   x_step_mm=x_step_mm, yaw_step_deg=yaw_step_deg,
                                   tilt_deg=tilt_deg)
        rows.append(row)
    return rows


def _fmt_runs(runs) -> str:
    return ', '.join(f'[{a:+d}, {b:+d}]' for a, b in runs) if runs else 'none'


def markdown_table(rows: list[dict]) -> str:
    lines = ['| deck (mm) | fingertip height (mm) | left lane x (mm, + forward) | right lane x (mm, + forward) |',
             '|---|---|---|---|']
    for r in rows:
        lines.append(f"| {r['deck_mm']:g} | {r['z_tip_mm']:g} | {_fmt_runs(r['left'])} | {_fmt_runs(r['right'])} |")
    return '\n'.join(lines)


def render_svg(dims: dict[str, float], runs_by_side: dict[str, list[tuple[int, int]]],
               pockets_by_side: dict[str, list[float] | None], pocket_mm: float,
               deck_mm: float, note: str = '') -> str:
    """Top view of the bench in millimeters: plate, column, box, lanes,
    pockets. Page x = robot forward, page y = robot left (so the drawing
    reads like a plan seen from above the column)."""
    mm = lambda v: v * 1000.0  # noqa: E731
    plate_x0 = mm(-dims['plate_x_back'] - dims['plate_lx'] / 2)
    plate_x1 = mm(-dims['plate_x_back'] + dims['plate_lx'] / 2)
    plate_y = mm(dims['plate_ly'] / 2)
    col_cx = mm(-dims['mount_lx'] / 2 - dims['col_gap_x'] - dims['col_lx'] / 2)
    col_hx, col_hy = mm(dims['col_lx'] / 2), mm(dims['col_ly'] / 2)
    box_hx, box_hy = mm(dims['mount_lx'] / 2), mm(dims['mount_ly'] / 2)
    xs = [plate_x0, plate_x1] + [v for runs in runs_by_side.values() for r in runs for v in r]
    x_min, x_max = min(xs) - 30, max(xs) + 30
    y_half = plate_y + 30
    w, h = x_max - x_min, 2 * y_half
    # page y grows downward; robot +y (left) goes up the page
    px = lambda x: x - x_min  # noqa: E731
    py = lambda y: y_half - y  # noqa: E731
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.1f}mm" height="{h:.1f}mm" '
           f'viewBox="0 0 {w:.1f} {h:.1f}">',
           '<style>.cut{fill:none;stroke:#d00;stroke-width:0.2}'
           '.ref{fill:none;stroke:#555;stroke-width:0.2;stroke-dasharray:2 1}'
           '.lane{stroke:#06c;stroke-width:0.6}.pocket{fill:none;stroke:#d00;stroke-width:0.2}'
           '.axis{stroke:#999;stroke-width:0.2;stroke-dasharray:1 1}'
           'text{font-family:sans-serif;font-size:4px;fill:#333}</style>',
           f'<title>SOMA deck, {deck_mm:g} mm above the plate</title>',
           f'<rect class="ref" id="plate" x="{px(plate_x0):.1f}" y="{py(plate_y):.1f}" '
           f'width="{plate_x1 - plate_x0:.1f}" height="{2 * plate_y:.1f}"/>',
           f'<rect class="ref" id="column" x="{px(col_cx - col_hx):.1f}" y="{py(col_hy):.1f}" '
           f'width="{2 * col_hx:.1f}" height="{2 * col_hy:.1f}"/>',
           f'<rect class="ref" id="central_box" x="{px(-box_hx):.1f}" y="{py(box_hy):.1f}" '
           f'width="{2 * box_hx:.1f}" height="{2 * box_hy:.1f}"/>',
           f'<line class="axis" id="disc_axis" x1="{px(0):.1f}" y1="0" x2="{px(0):.1f}" y2="{h:.1f}"/>']
    for side in SIDES:
        y = mm(lane_y(side, dims))
        for i, (a, b) in enumerate(runs_by_side.get(side, [])):
            out.append(f'<line class="lane" id="{side}_lane_{i}" x1="{px(a):.1f}" y1="{py(y):.1f}" '
                       f'x2="{px(b):.1f}" y2="{py(y):.1f}"/>')
        for i, c in enumerate(pockets_by_side.get(side) or []):
            out.append(f'<rect class="pocket" id="{side}_pocket_{i}" x="{px(c - pocket_mm / 2):.1f}" '
                       f'y="{py(y + pocket_mm / 2):.1f}" width="{pocket_mm:.1f}" height="{pocket_mm:.1f}"/>')
        out.append(f'<text x="{px(x_min + 5):.1f}" y="{py(y) - 2:.1f}">{side} lane, y = {y:+.1f} mm</text>')
    out.append(f'<text x="2" y="{h - 2:.1f}">x forward to the right, mm. {note}</text>')
    out.append('</svg>')
    return '\n'.join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--deck', type=float, nargs='+', default=[0, 40, 60, 80],
                    help='deck top heights above the plate, mm')
    ap.add_argument('--grasp', type=float, default=20.0,
                    help='fingertip height above the deck when grasping, mm')
    ap.add_argument('--flip', action='append', default=[], choices=JOINTS,
                    help='mirror this joint sense (repeatable)')
    ap.add_argument('--margin', type=float, default=0.1, help='radians kept inside every limit')
    ap.add_argument('--tilt', type=float, default=0.0, help='allowed tool tilt from vertical, deg')
    ap.add_argument('--x-step', type=int, default=1, help='x grid, mm')
    ap.add_argument('--yaw-step', type=float, default=0.5, help='yaw scan step, deg')
    ap.add_argument('--pockets', type=int, default=3)
    ap.add_argument('--pitch', type=float, default=70.0, help='pocket pitch, mm')
    ap.add_argument('--pocket', type=float, default=46.0, help='pocket side, mm (40 mm part + play)')
    ap.add_argument('--svg', type=Path, help='write the deck drawing here (uses the first --deck)')
    args = ap.parse_args(argv)

    signs = dict(SIGNS)
    for joint in args.flip:
        signs[joint] *= -1
    dims = load_dimensions()
    rows = reach_table(args.deck, args.grasp, dims, signs, args.margin,
                       args.x_step, args.yaw_step, args.tilt)
    print('Sign hypothesis (model frame, +1 = URDF sense, positive = backward):')
    for joint in JOINTS:
        status = VERIFIED.get(joint, 'UNVERIFIED until the v0.2 sign check')
        print(f'  {joint:12s} {signs[joint]:+d}   {status}')
    print(f'Lanes at y = {lane_y("left", dims) * 1000:+.1f} mm (left) and '
          f'{lane_y("right", dims) * 1000:+.1f} mm (right); margin {args.margin} rad; '
          f'tool tilt up to {args.tilt:g} deg\n')
    print(markdown_table(rows))
    first = rows[0]
    pockets_by_side = {}
    print(f"\nPockets on the deck at {first['deck_mm']:g} mm "
          f"({args.pockets} x {args.pocket:g} mm at {args.pitch:g} mm pitch, longest run):")
    for side in SIDES:
        run = longest(first[side])
        centers = pockets(run, args.pitch, args.pockets, args.pocket) if run else None
        pockets_by_side[side] = centers
        shown = ', '.join(f'{c:+.0f}' for c in centers) if centers else 'do not fit'
        print(f'  {side:5s} run {run}: {shown}')
    if args.svg:
        note = 'signs: ' + ' '.join(f'{j}{signs[j]:+d}' for j in JOINTS)
        args.svg.write_text(render_svg(dims, {s: first[s] for s in SIDES}, pockets_by_side,
                                       args.pocket, first['deck_mm'], note))
        print(f'\nwrote {args.svg}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
