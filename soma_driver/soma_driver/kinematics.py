"""Planar 4R kinematics (FK, IK, and workspace solver) for SOMA's arms.

PURE module (no ROS): runs under pytest on any machine, no hardware needed.

Both arms hang from the central box with their four big joints (yaw,
shoulder, elbow, wrist pitch) on parallel horizontal axes, so each arm is
a planar 4R chain in its own vertical plane, 81 mm outboard of the bench
centerline (tip_lane_y, measured 2026-10-27). The fingertip center sits on
the roll axis and never leaves that plane.

Frame, per arm plane:
  - x: forward (away from the column), meters
  - z: up from the base plate top surface, meters
  - disc axis at (0, mount_h)
  - phi_tool: tool angle from straight down, radians (0.0 = vertical gripper)

Angles are in the hug convention (negative = forward/inward, positive =
backward/up), which matches SERVO_MAP and the verified sign check of
2026-10-12.
"""
import math
from pathlib import Path

import yaml

from .servo_map import SERVO_MAP

JOINTS = ('yaw', 'shoulder', 'elbow', 'wrist_pitch')
SIDES = ('left', 'right')
# Sign convention in the model frame (+1 = command sign matches the
# positive = backward URDF sense). All four planar joints passed
# soma_sign_check on both arms on 2026-10-12.
SIGNS = {'yaw': 1, 'shoulder': 1, 'elbow': 1, 'wrist_pitch': 1}
VERIFIED = {
    'yaw': '2026-10-01 and 2026-10-12, negative = forward, on the metal',
    'shoulder': '2026-10-12, negative = forward, on the metal',
    'elbow': '2026-10-12, negative = forward, on the metal',
    'wrist_pitch': '2026-10-12, negative = forward, on the metal',
}


def default_dimensions_path() -> Path:
    """Locate soma_description/config/dimensions.yaml from repo or install."""
    repo_path = (Path(__file__).resolve().parents[2]
                 / 'soma_description' / 'config' / 'dimensions.yaml')
    if repo_path.exists():
        return repo_path
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('soma_description')) / 'config' / 'dimensions.yaml'


def load_dimensions(path: Path | None = None) -> dict[str, float]:
    """Every arm and bench dimension as {name: meters}."""
    target = default_dimensions_path() if path is None else Path(path)
    doc = yaml.safe_load(target.read_text())
    dims = {}
    for section in ('arm', 'bench'):
        for name, entry in doc[section]['dimensions'].items():
            dims[name] = float(entry['value'])
    # The tool is the one direct reading, closed (2026-10-27): the parts add
    # to 149.1 mm, the caliper across the whole tool reads 150.0.
    dims['tool_len'] = dims['tool_closed_len']
    return dims


def link_lengths(dims: dict[str, float]) -> tuple[float, float, float, float]:
    return (dims['clavicle_len'], dims['upper_len'], dims['fore_len'], dims['tool_len'])


def lane_y(side: str, dims: dict[str, float]) -> float:
    """Lateral position of the closed fingertip's lane, meters (measured)."""
    y = dims['tip_lane_y']
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


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def fk(q: tuple[float, float, float, float], dims: dict[str, float]
       ) -> tuple[list[tuple[float, float]], float]:
    """Joint points D, S, E, W and the fingertip T, plus the tool angle."""
    pts = [(0.0, dims['mount_h'])]
    phi = 0.0
    for i, (angle, length) in enumerate(zip(q, link_lengths(dims))):
        phi += angle
        # The clavicle alone leaves the disc at its mounting angle (a DH
        # theta offset); the shoulder's zero takes it back out.
        seg = phi + dims['clavicle_angle'] if i == 0 else phi
        x, z = pts[-1]
        pts.append((x - length * math.sin(seg), z - length * math.cos(seg)))
    return pts, phi


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
    sx = -clav * math.sin(q1 + dims['clavicle_angle'])
    sz = dims['mount_h'] - clav * math.cos(q1 + dims['clavicle_angle'])
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


def _inside(q: tuple[float, float, float, float],
            lim: dict[str, tuple[float, float]]) -> bool:
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


def solve_pose(target: tuple[float, float], side: str,
               dims: dict[str, float] | None = None,
               phi_tool: float = 0.0,
               wrist_roll: float = 0.0,
               finger: float = 0.0,
               signs: dict[str, int] = SIGNS,
               margin: float = 0.1,
               yaw_step_deg: float = 0.5) -> dict[str, float] | None:
    """Solve planar IK and return full 6-joint targets for `side`, or None."""
    if side not in SIDES:
        raise ValueError(f'unknown side {side!r}: expected one of {SIDES}')
    if dims is None:
        dims = load_dimensions()
    q = solve(target, phi_tool, side, dims, signs=signs,
              margin=margin, yaw_step_deg=yaw_step_deg)
    if q is None:
        return None
    pose = {f'{side}_arm_{joint}_joint': float(val) for joint, val in zip(JOINTS, q)}
    pose[f'{side}_arm_wrist_roll_joint'] = SERVO_MAP[f'{side}_arm_wrist_roll_joint'].clamp(wrist_roll)
    pose[f'{side}_arm_finger_l_joint'] = SERVO_MAP[f'{side}_arm_finger_l_joint'].clamp(finger)
    return pose
