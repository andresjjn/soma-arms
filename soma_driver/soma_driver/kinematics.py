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
from .trajectory import JointMotion

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

# Safety invariants grounded in bench measurements:
# - ARM_MASS_KG: 0.7 kg estimate (docs/plan.md), calibrated so a straight
#   horizontal arm yields ~11.4 kg.cm on the base (primitives.py).
# - MAX_BASE_TORQUE_KGCM: 9.5 kg.cm leaves margin under MG996R ~10 kg.cm stall.
# - MIN_CLEARANCE_Z_M: 5 mm above the base plate surface.
# - ELBOW_CURL_FLOOR_RAD: -1.30 rad (ELBOW_REACH_CLEAN, 2026-10-26).
ARM_MASS_KG = 0.7
MAX_BASE_TORQUE_KGCM = 9.5
MIN_CLEARANCE_Z_M = 0.005
ELBOW_CURL_FLOOR_RAD = -1.30


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


def estimate_base_torque_kgcm(q: tuple[float, float, float, float],
                              dims: dict[str, float],
                              arm_mass_kg: float = ARM_MASS_KG) -> float:
    """Static gravity torque (kg.cm) on the base yaw axis for pose `q`."""
    pts, _ = fk(q, dims)
    # Distribute arm mass over upper arm (S->E), forearm (E->W), tool (W->T).
    segs = [(pts[i], pts[i + 1], math.hypot(
        pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]))
        for i in range(1, 4)]
    total_len = sum(length for _, _, length in segs)
    if total_len <= 0.0:
        return 0.0
    x_disc = pts[0][0]
    torque_kgm = 0.0
    for p0, p1, length in segs:
        mass_i = arm_mass_kg * (length / total_len)
        x_mid = 0.5 * (p0[0] + p1[0])
        torque_kgm += mass_i * abs(x_mid - x_disc)
    return torque_kgm * 100.0


def pose_safety_fault(q: tuple[float, float, float, float],
                      side: str,
                      dims: dict[str, float],
                      min_z_m: float = MIN_CLEARANCE_Z_M,
                      max_torque_kgcm: float = MAX_BASE_TORQUE_KGCM,
                      min_elbow_rad: float = ELBOW_CURL_FLOOR_RAD) -> str | None:
    """Return a human-readable safety fault for `q`, or None if safe."""
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        return 'non-finite joint angle'
    for joint, val in zip(JOINTS, q):
        spec = SERVO_MAP[f'{side}_arm_{joint}_joint']
        if not (spec.lower - 1e-9 <= val <= spec.upper + 1e-9):
            return f'{joint} ({val:.3f} rad) outside soft limits [{spec.lower:.3f}, {spec.upper:.3f}]'
    if q[2] < min_elbow_rad - 1e-9:
        return f'elbow ({q[2]:.3f} rad) curls past measured body clearance ({min_elbow_rad:.2f} rad)'
    pts, _ = fk(q, dims)
    labels = ('shoulder', 'elbow', 'wrist', 'fingertip')
    for label, (_, z) in zip(labels, pts[1:]):
        if z < min_z_m - 1e-9:
            return f'{label} height {z * 1000:.1f} mm below clearance floor ({min_z_m * 1000:.1f} mm)'
    torque = estimate_base_torque_kgcm(q, dims)
    if torque > max_torque_kgcm:
        return f'estimated base torque {torque:.2f} kg.cm exceeds {max_torque_kgcm:.2f} kg.cm limit'
    return None


def check_trajectory_safety(q_start: tuple[float, float, float, float],
                            q_end: tuple[float, float, float, float],
                            side: str,
                            dims: dict[str, float],
                            min_z_m: float = MIN_CLEARANCE_Z_M,
                            max_torque_kgcm: float = MAX_BASE_TORQUE_KGCM,
                            min_elbow_rad: float = ELBOW_CURL_FLOOR_RAD,
                            rate_hz: float = 50.0) -> str | None:
    """Verify every 50 Hz minimum-jerk ramp tick from q_start to q_end."""
    dt = 1.0 / rate_hz
    motions = []
    for joint, start, end in zip(JOINTS, q_start, q_end):
        spec = SERVO_MAP[f'{side}_arm_{joint}_joint']
        m = JointMotion(spec.clamp(start), spec.max_rate)
        m.set_target(spec.clamp(end))
        motions.append(m)

    # Worst-case settle time + 0.5 s safety margin in ticks.
    max_ticks = 0
    for joint, start, end in zip(JOINTS, q_start, q_end):
        spec = SERVO_MAP[f'{side}_arm_{joint}_joint']
        t_s = 1.875 * abs(end - start) / spec.max_rate
        max_ticks = max(max_ticks, int(math.ceil((t_s + 0.5) * rate_hz)))
    max_ticks = max(max_ticks, 10)

    for tick in range(1, max_ticks + 1):
        q_tick = tuple(m.step(dt) for m in motions)
        fault = pose_safety_fault(
            q_tick, side, dims,
            min_z_m=min_z_m,
            max_torque_kgcm=max_torque_kgcm,
            min_elbow_rad=min_elbow_rad,
        )
        if fault is not None:
            return f'tick {tick} ({tick * dt:.2f} s): {fault}'
    return None


def ik(target: tuple[float, float], phi_tool: float, q1: float,
       dims: dict[str, float]) -> list[tuple[float, float, float, float]]:
    """Both elbow branches reaching `target` with the tool at `phi_tool`.

    The yaw is given (the chain has one redundant joint); the shoulder
    and elbow solve the 2R problem to the wrist pitch axis and the wrist
    closes the tool angle. Unchecked against limits; empty if out of reach.
    """
    if not (all(math.isfinite(v) for v in target) and math.isfinite(phi_tool) and math.isfinite(q1)):
        raise ValueError('ik inputs must be finite')
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
               yaw_step_deg: float = 0.5,
               min_z_m: float = MIN_CLEARANCE_Z_M,
               max_torque_kgcm: float = MAX_BASE_TORQUE_KGCM,
               min_elbow_rad: float = ELBOW_CURL_FLOOR_RAD) -> dict[str, float] | None:
    """Solve planar IK with safety checks and return 6-joint targets, or None."""
    if side not in SIDES:
        raise ValueError(f'unknown side {side!r}: expected one of {SIDES}')
    if not (all(math.isfinite(v) for v in target)
            and math.isfinite(phi_tool)
            and math.isfinite(wrist_roll)
            and math.isfinite(finger)):
        raise ValueError('solve_pose inputs must be finite')
    if dims is None:
        dims = load_dimensions()

    lim = limits(side, signs, margin)
    best, best_cost = None, None
    lo, up = lim['yaw']
    steps = max(1, int(round((up - lo) / math.radians(yaw_step_deg))))
    for i in range(steps + 1):
        q1 = lo + (up - lo) * i / steps
        for q_model in ik(target, phi_tool, q1, dims):
            if not _inside(q_model, lim):
                continue
            q_cmd = tuple(v * signs[j] for j, v in zip(JOINTS, q_model))
            if pose_safety_fault(
                q_cmd, side, dims,
                min_z_m=min_z_m,
                max_torque_kgcm=max_torque_kgcm,
                min_elbow_rad=min_elbow_rad,
            ) is not None:
                continue
            cost = sum(abs(v) for v in q_model)
            if best is None or cost < best_cost:
                best, best_cost = q_cmd, cost

    if best is None:
        return None
    pose = {f'{side}_arm_{joint}_joint': float(val) for joint, val in zip(JOINTS, best)}
    pose[f'{side}_arm_wrist_roll_joint'] = SERVO_MAP[f'{side}_arm_wrist_roll_joint'].clamp(wrist_roll)
    pose[f'{side}_arm_finger_l_joint'] = SERVO_MAP[f'{side}_arm_finger_l_joint'].clamp(finger)
    return pose
