"""The primitive catalog is a contract: every pose provably safe on paper.

Pure tests, no ROS. If a pose ever names a joint that does not exist,
exceeds a measured soft limit, or touches hardware that is not on the
bench, the suite goes red before any message can be published.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.primitives import (  # noqa: E402
    COMMANDED_JOINTS, HOME, OFF_BENCH, POSES, SEQUENCES,
    pose_targets, sequence_steps, settle_time_s)
from soma_driver.servo_map import MIMIC_JOINTS, SERVO_MAP  # noqa: E402


class TestCatalogIsSafeOnPaper:
    @pytest.mark.parametrize('pose', sorted(POSES))
    def test_every_joint_exists_and_is_commandable(self, pose):
        for name in POSES[pose]:
            assert name in SERVO_MAP, f'{pose}: unknown joint {name}'
            assert name not in MIMIC_JOINTS, f'{pose}: mimic joints are never commanded'
            assert name not in OFF_BENCH, f'{pose}: {name} is not on this bench'

    @pytest.mark.parametrize('pose', sorted(POSES))
    def test_every_value_is_inside_the_soft_band(self, pose):
        for name, value in POSES[pose].items():
            spec = SERVO_MAP[name]
            assert spec.clamp(value) == pytest.approx(value), (
                f'{pose}.{name}={value} outside [{spec.lower}, {spec.upper}]')

    def test_the_torso_lift_is_absent_from_every_pose(self):
        # The L16 waits in a drawer for the printed torso (task #9).
        for pose, targets in POSES.items():
            assert 'torso_lift_joint' not in targets, pose


class TestHomeIsTheBootPose:
    """Commanding home must be a no-op for a freshly started driver."""

    @pytest.mark.parametrize('name', COMMANDED_JOINTS)
    def test_home_matches_clamped_zero(self, name):
        assert HOME[name] == pytest.approx(SERVO_MAP[name].clamp(0.0))

    def test_home_covers_every_commanded_joint(self):
        assert set(HOME) == set(COMMANDED_JOINTS)

    def test_compact_is_home_on_the_hanging_bench(self):
        # Hanging straight down IS the folded, resting, power-on pose of
        # safety rule 4. If the rig ever changes (printed torso), this
        # test forces the conversation.
        assert POSES['compact'] == POSES['home']


class TestGrippers:
    def test_gripper_poses_touch_only_finger_joints(self):
        for pose in ('grippers_open', 'grippers_closed'):
            for name in POSES[pose]:
                assert 'finger' in name, f'{pose} moved a non-finger: {name}'

    def test_open_is_one_closed_is_zero(self):
        # Measured convention of 2026-08-03: 0.0 rad = fingers closed.
        assert all(v == 1.0 for v in POSES['grippers_open'].values())
        assert all(v == 0.0 for v in POSES['grippers_closed'].values())


class TestSequences:
    @pytest.mark.parametrize('seq', sorted(SEQUENCES))
    def test_steps_reference_defined_poses_with_positive_dwell(self, seq):
        for pose, dwell in SEQUENCES[seq]:
            assert pose in POSES, f'{seq}: unknown pose {pose}'
            assert dwell > 0.0, f'{seq}: non-positive dwell after {pose}'

    def test_demo_starts_and_ends_at_home(self):
        steps = SEQUENCES['demo']
        assert steps[0][0] == 'home'
        assert steps[-1][0] == 'home'

    def test_demo_dwells_cover_the_travel_time(self):
        """Every dwell must outlast the worst-case minimum-jerk travel."""
        current = dict(HOME)
        for pose, dwell in SEQUENCES['demo']:
            targets = pose_targets(pose)
            assert dwell >= settle_time_s(targets, current), (
                f'demo: dwell {dwell}s after {pose} shorter than the ramp')
            current.update(targets)

    def test_demo_moves_gently(self):
        """No step of the demo asks any joint to jump more than 1 rad."""
        current = dict(HOME)
        for pose, _ in SEQUENCES['demo']:
            for name, value in pose_targets(pose).items():
                assert abs(value - current[name]) <= 1.0, (
                    f'demo: {pose} moves {name} too far in one step')
                current[name] = value


class TestAccessors:
    def test_pose_targets_returns_a_copy(self):
        pose_targets('home')['left_arm_elbow_joint'] = 99.0
        assert POSES['home']['left_arm_elbow_joint'] != 99.0

    def test_unknown_names_raise(self):
        with pytest.raises(KeyError):
            pose_targets('backflip')
        with pytest.raises(KeyError):
            sequence_steps('backflip')
