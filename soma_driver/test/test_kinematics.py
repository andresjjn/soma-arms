"""Tests for soma_driver.kinematics and mock driver round-trip over the lanes."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soma_driver.kinematics import (  # noqa: E402
    JOINTS,
    SIDES,
    VERIFIED,
    fk,
    ik,
    lane_y,
    load_dimensions,
    solve,
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


def test_solve_pose_returns_none_when_unreachable_or_invalid_side(dims):
    assert solve_pose((0.5, 0.030), 'right', dims) is None
    with pytest.raises(ValueError, match='unknown side'):
        solve_pose((0.0, 0.030), 'center', dims)


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
            pts, phi = fk(q_settled, dims)
            assert math.dist(pts[-1], (tx, tz)) < 1e-6
            assert abs(phi) < 1e-6
