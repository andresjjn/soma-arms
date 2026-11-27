"""Tests for soma_driver.tending (v0.4 pick-and-place and 50 Hz safety)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soma_driver.kinematics import (  # noqa: E402
    JOINTS,
    load_dimensions,
    pose_safety_fault,
)
from soma_driver.primitives import HOME  # noqa: E402
from soma_driver.servo_map import SERVO_MAP  # noqa: E402
from soma_driver.tending import (  # noqa: E402
    DEFAULT_POCKETS_MM,
    interleave_cycles,
    plan_pick_and_place,
    plan_tending_cycle,
    solve_with_tilt,
)
from soma_driver.trajectory import JointMotion  # noqa: E402


@pytest.fixture(scope='module')
def dims():
    return load_dimensions()


def test_right_lane_uses_tilt_for_front_pocket_and_rejects_unreachable(dims):
    # Right lane front pocket (+10 mm) at z=60 mm needs negative tool tilt.
    tilt_deg, pose = solve_with_tilt(10.0, 60.0, 'right', dims)
    assert tilt_deg != 0.0
    assert abs(tilt_deg) <= 15.0
    assert pose is not None

    # +70 mm on the right lane at lift height (60 mm) is unreachable even with +-15 deg tilt.
    with pytest.raises(ValueError, match='unreachable or unsafe'):
        solve_with_tilt(70.0, 60.0, 'right', dims)


@pytest.mark.parametrize('side', ('left', 'right'))
def test_full_tending_cycle_is_safe_at_every_50hz_tick(side, dims):
    cycle = plan_tending_cycle(side, dims=dims)
    assert cycle[0].label == 'home'
    assert cycle[-1].label == 'home'
    assert len(cycle) == 18  # home + 8 load + 8 unload + home

    # Every step only touches its own arm's joints.
    prefix = f'{side}_arm_'
    for step in cycle:
        assert all(name.startswith(prefix) for name in step.pose)

    # Simulate the entire cycle tick-by-tick through JointMotion at 50 Hz.
    dt = 1.0 / 50.0
    arm_joints = [f'{side}_arm_{j}_joint' for j in JOINTS]
    motion = {j: JointMotion(SERVO_MAP[j].clamp(0.0), SERVO_MAP[j].max_rate)
              for j in arm_joints}

    for step in cycle:
        for j in arm_joints:
            motion[j].set_target(SERVO_MAP[j].clamp(step.pose[j]))
        ticks = max(1, int(round(step.dwell_s / dt)))
        for _ in range(ticks):
            q_tick = tuple(motion[j].step(dt) for j in arm_joints)
            assert pose_safety_fault(q_tick, side, dims) is None


def test_tending_rejects_unsafe_parameters_and_invalid_inputs(dims):
    with pytest.raises(ValueError, match='safety floor'):
        plan_pick_and_place('left', -70.0, 0.0, dims=dims, pick_z_mm=5.0)

    with pytest.raises(ValueError, match='exceed pick_z_mm'):
        plan_pick_and_place('left', -70.0, 0.0, dims=dims, pick_z_mm=50.0, lift_z_mm=45.0)

    with pytest.raises(ValueError, match='grip_closed'):
        plan_pick_and_place('left', -70.0, 0.0, dims=dims, grip_open=0.5, grip_closed=0.6)

    with pytest.raises(ValueError, match='finite'):
        solve_with_tilt(float('nan'), 50.0, 'left', dims)

    with pytest.raises(ValueError, match='0.0, 1.0'):
        solve_with_tilt(0.0, 50.0, 'left', dims, finger=1.5)


def test_interleave_cycles_keeps_arms_mutually_exclusive(dims):
    left = plan_tending_cycle('left', *DEFAULT_POCKETS_MM['left'], dims=dims)
    right = plan_tending_cycle('right', *DEFAULT_POCKETS_MM['right'], dims=dims)
    combined = interleave_cycles(left, right)
    assert len(combined) == len(left) + len(right)
    # Transition between left and right ends/starts at home.
    assert combined[len(left) - 1].label == 'home'
    assert combined[len(left)].label == 'home'
    assert combined[-1].pose == {k: v for k, v in HOME.items() if k.startswith('right_arm_')}
