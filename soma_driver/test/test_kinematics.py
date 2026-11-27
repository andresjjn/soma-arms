"""Tests for soma_driver.kinematics, safety invariants, and mock round-trip."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soma_driver.kinematics import (  # noqa: E402
    ELBOW_CURL_FLOOR_RAD,
    JOINTS,
    MAX_BASE_TORQUE_KGCM,
    SIDES,
    VERIFIED,
    check_trajectory_safety,
    estimate_base_torque_kgcm,
    fk,
    ik,
    lane_y,
    load_dimensions,
    pose_safety_fault,
    solve_pose,
)
from soma_driver.servo_map import SERVO_MAP  # noqa: E402
from soma_driver.trajectory import JointMotion  # noqa: E402


@pytest.fixture(scope='module')
def dims():
    return load_dimensions()


def test_all_four_planar_joints_are_marked_verified():
    assert set(VERIFIED) == set(JOINTS)


def test_solve_pose_returns_full_six_joint_dict_within_soft_limits(dims):
    pose = solve_pose((0.0, 0.030), 'left', dims)
    assert pose is not None
    expected_keys = {
        'left_arm_yaw_joint',
        'left_arm_shoulder_joint',
        'left_arm_elbow_joint',
        'left_arm_wrist_pitch_joint',
        'left_arm_wrist_roll_joint',
        'left_arm_finger_l_joint',
    }
    assert set(pose) == expected_keys
    for name, value in pose.items():
        spec = SERVO_MAP[name]
        if any(name.endswith(f'_{j}_joint') for j in JOINTS):
            assert spec.lower + 0.1 <= value <= spec.upper - 0.1
        else:
            assert spec.lower <= value <= spec.upper


def test_solve_pose_rejects_non_finite_and_invalid_inputs(dims):
    with pytest.raises(ValueError, match='finite'):
        solve_pose((float('nan'), 0.030), 'left', dims)
    with pytest.raises(ValueError, match='finite'):
        solve_pose((0.0, float('inf')), 'left', dims)
    with pytest.raises(ValueError, match='finite'):
        ik((float('nan'), 0.030), 0.0, 0.0, dims)
    with pytest.raises(ValueError, match='unknown side'):
        solve_pose((0.0, 0.030), 'center', dims)


def test_pose_safety_catches_torque_overload_plate_collision_and_body_curl(dims):
    # 1. Straight horizontal arm exceeds the ~10 kg.cm MG996R stall margin.
    horizontal = (-math.pi / 2, 0.0, 0.0, 0.0)
    torque = estimate_base_torque_kgcm(horizontal, dims)
    assert torque > MAX_BASE_TORQUE_KGCM
    fault = pose_safety_fault(horizontal, 'left', dims)
    assert fault is not None and 'torque' in fault

    # 2. Below-plate collision is caught even if joints are inside SERVO_MAP.
    assert solve_pose((0.0, -0.010), 'left', dims) is None
    assert solve_pose((0.0, 0.002), 'left', dims) is None

    # 3. Elbow curl past measured body clearance (-1.30 rad) is refused.
    curled = (-0.87, 0.0, ELBOW_CURL_FLOOR_RAD - 0.15, 0.0)
    curl_fault = pose_safety_fault(curled, 'right', dims)
    assert curl_fault is not None and 'body clearance' in curl_fault

    # 4. Non-finite and out-of-limit angles are refused.
    assert pose_safety_fault((float('nan'), 0.0, 0.0, 0.0), 'left', dims) is not None
    assert pose_safety_fault((-10.0, 0.0, 0.0, 0.0), 'left', dims) is not None


def test_trajectory_safety_catches_mid_ramp_overload_between_safe_endpoints(dims):
    # Both (-2.0, 0.0, -0.5, 0.0) and home are individually safe, but returning
    # straight home unfolds the elbow before the base clears the horizontal,
    # spiking base torque above 9.50 kg.cm at tick 13 (0.26 s).
    q_high = (-2.0, 0.0, -0.5, 0.0)
    q_home = (0.0, 0.0, 0.0, 0.0)
    assert pose_safety_fault(q_high, 'left', dims) is None
    assert pose_safety_fault(q_home, 'left', dims) is None
    mid_fault = check_trajectory_safety(q_high, q_home, 'left', dims)
    assert mid_fault is not None and 'tick' in mid_fault and 'torque' in mid_fault

    # Safe transition from home to a lane target passes every 50 Hz tick.
    pose = solve_pose((0.0, 0.030), 'left', dims)
    assert pose is not None
    q_safe = tuple(pose[f'left_arm_{j}_joint'] for j in JOINTS)
    assert check_trajectory_safety(q_home, q_safe, 'left', dims) is None


def test_mock_round_trip_ik_then_fk_over_both_lanes(dims):
    """Next-session gate #3: mock round trip ik() then fk() over y = +-81 mm."""
    dt = 1.0 / 50.0  # ArmController RATE_HZ
    lane_points_m = {
        'left': [(-0.070, 0.040), (-0.030, 0.030), (0.0, 0.030),
                 (0.030, 0.030), (0.070, 0.040)],
        'right': [(-0.070, 0.040), (-0.050, 0.030), (-0.030, 0.030),
                  (0.0, 0.030), (0.030, 0.030)],
    }

    for side in SIDES:
        assert abs(lane_y(side, dims)) == pytest.approx(0.081, abs=1e-6)
        arm_joints = [f'{side}_arm_{j}_joint' for j in JOINTS]
        motion = {j: JointMotion(SERVO_MAP[j].clamp(0.0), SERVO_MAP[j].max_rate)
                  for j in arm_joints}

        for tx, tz in lane_points_m[side]:
            pose = solve_pose((tx, tz), side, dims)
            assert pose is not None, (side, tx, tz)
            for j in arm_joints:
                motion[j].set_target(SERVO_MAP[j].clamp(pose[j]))

            # Step the minimum-jerk planner tick-by-tick until settled.
            settled = {}
            for _ in range(250):
                for j in arm_joints:
                    val = motion[j].step(dt)
                    spec = SERVO_MAP[j]
                    assert spec.lower <= val <= spec.upper
                    settled[j] = val

            q_settled = tuple(settled[j] for j in arm_joints)
            assert pose_safety_fault(q_settled, side, dims) is None
            pts, phi = fk(q_settled, dims)
            assert math.dist(pts[-1], (tx, tz)) < 1e-6
            assert abs(phi) < 1e-6
