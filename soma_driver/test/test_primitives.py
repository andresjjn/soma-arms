"""The primitive catalog is a contract: every pose provably safe on paper.

Pure tests, no ROS. If a pose ever names a joint that does not exist,
exceeds a measured soft limit, or touches hardware that is not on the
bench, the suite goes red before any message can be published.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.primitives import (
    BLENDED,  # noqa: E402
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

    @pytest.mark.parametrize('seq', sorted(SEQUENCES))
    def test_every_sequence_starts_and_ends_at_home(self, seq):
        steps = SEQUENCES[seq]
        assert steps[0][0] == 'home', seq
        assert steps[-1][0] == 'home', seq

    @pytest.mark.parametrize('seq', sorted(SEQUENCES))
    def test_every_dwell_covers_the_travel_time(self, seq):
        """Every dwell must outlast the worst-case minimum-jerk travel.

        Except in a sequence declared in BLENDED (2026-10-26): there the
        next pose is sent at a declared fraction of the travel, on purpose,
        so the arm flows instead of stopping on every pose. Even then the
        LAST step (home) must be reached; and TestBlendedSequences runs
        the driver's own planner over the whole sequence to prove the
        corner cutting stays inside the soft band and the poses' span."""
        blend = BLENDED.get(seq)
        steps = SEQUENCES[seq]
        current = dict(HOME)
        for i, (pose, dwell) in enumerate(steps):
            targets = pose_targets(pose)
            travel = settle_time_s(targets, current)
            needed = travel if blend is None or i == len(steps) - 1 else travel * blend
            assert dwell >= needed - 0.011, (
                f'{seq}: dwell {dwell}s after {pose} shorter than the ramp')
            current.update(targets)

    @pytest.mark.parametrize('seq', sorted(SEQUENCES))
    def test_every_sequence_moves_gently(self, seq):
        """No step of any sequence asks a joint to jump more than 1 rad."""
        current = dict(HOME)
        for pose, _ in SEQUENCES[seq]:
            for name, value in pose_targets(pose).items():
                assert abs(value - current[name]) <= 1.0, (
                    f'{seq}: {pose} moves {name} too far in one step')
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


class TestBlendedSequences:
    """A flowing sequence, simulated tick by tick with the driver's own
    planner (trajectory.JointMotion, the clamp of servo_map), exactly as
    the player would feed it: publish, sleep the dwell, publish the next.
    Corner cutting must keep every ARM joint inside its soft band and
    within 0.1 rad of the span its own poses cover, and the sequence must
    end settled at home. A finger's replanned curve may overshoot its
    closed stop by a little: the pulse saturates there (command_to_us
    clamps), the same allowance test_player makes for fingers, so a
    finger is judged by the pulse it would send."""

    DT = 0.02

    @pytest.mark.parametrize('seq', sorted(BLENDED))
    def test_the_flow_stays_inside_what_the_poses_describe(self, seq):
        from soma_driver.trajectory import JointMotion
        motion = {n: JointMotion(HOME[n], SERVO_MAP[n].max_rate) for n in HOME}
        span = {n: [HOME[n], HOME[n]] for n in HOME}
        for pose, dwell in SEQUENCES[seq]:
            for name, value in pose_targets(pose).items():
                motion[name].set_target(SERVO_MAP[name].clamp(value))
                span[name] = [min(span[name][0], value), max(span[name][1], value)]
            for _ in range(round(dwell / self.DT)):
                for name, m in motion.items():
                    q = m.step(self.DT)
                    spec = SERVO_MAP[name]
                    if 'finger' in name:
                        lo_us, hi_us = sorted((spec.min_us, spec.max_us))
                        assert lo_us <= spec.command_to_us(q) <= hi_us
                        continue
                    assert spec.lower - 1e-9 <= q <= spec.upper + 1e-9, (seq, pose, name)
                    lo, hi = span[name]
                    assert lo - 0.1 <= q <= hi + 0.1, (seq, pose, name, q)
        for name, m in motion.items():
            for _ in range(200):
                m.step(self.DT)
            assert m.position == pytest.approx(HOME[name], abs=1e-6), name

    @pytest.mark.parametrize('seq', sorted(BLENDED))
    def test_a_blend_is_a_fraction(self, seq):
        assert 0.5 <= BLENDED[seq] < 1.0
