"""limp.py: an arm may hang with its signal cut only from home. Pure."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.limp import arm_joints, limp_refusal  # noqa: E402
from soma_driver.servo_map import RELEASE_WHEN_SETTLED, SERVO_MAP  # noqa: E402

HOME = {n: s.clamp(0.0) for n, s in SERVO_MAP.items()}


def test_each_arm_owns_its_six_joints_and_nothing_else():
    for arm in ('left', 'right'):
        joints = arm_joints(arm)
        assert len(joints) == 6 and all(j.startswith(f'{arm}_arm_') for j in joints)
    assert 'torso_lift_joint' not in arm_joints('left') + arm_joints('right')


def test_both_arms_at_home_may_go_limp_and_clearing_is_always_allowed():
    assert limp_refusal('left', HOME, HOME) is None
    assert limp_refusal('right', HOME, HOME) is None
    assert limp_refusal('', {}, {}) is None


@pytest.mark.parametrize('where', ['current', 'target'])
def test_an_arm_away_from_home_is_refused_because_it_would_fall(where):
    moved = dict(HOME, left_arm_yaw_joint=-0.5)
    current, target = (moved, HOME) if where == 'current' else (HOME, moved)
    reason = limp_refusal('left', current, target)
    assert reason and 'left_arm_yaw_joint' in reason and 'falls' in reason


def test_the_other_arm_working_does_not_block_this_one():
    busy_right = dict(HOME, right_arm_yaw_joint=-2.0, right_arm_elbow_joint=-0.9)
    assert limp_refusal('left', busy_right, busy_right) is None


def test_unknown_values_are_refused():
    assert 'must be' in limp_refusal('both', HOME, HOME)


def test_rule_3_holds_no_arm_joint_releases_on_its_own():
    # limp is an explicit parameter, never the automatic settle release.
    assert RELEASE_WHEN_SETTLED == {'torso_lift_joint'}
