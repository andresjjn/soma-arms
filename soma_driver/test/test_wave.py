"""The wave is a right-arm-only greeting that never surprises the left arm.

Pure tests, no ROS. The generic sequence contract (defined poses, home at
both ends, dwells that cover the ramp, no jump over 1 rad) is already
enforced for every sequence in test_primitives.py; this file pins what is
specific to the wave: which arm moves, what the first move probes, the
margins to the measured limits, and the rhythm of the hand.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.primitives import (  # noqa: E402
    HOME, POSES, SEQUENCES, pose_targets, settle_time_s)
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

WAVE = SEQUENCES['wave']
WAVE_POSES = sorted({pose for pose, _ in WAVE} - {'home'})
RIGHT_JOINTS = tuple(n for n in HOME if n.startswith('right_arm_'))


def _walk():
    """The accumulated commanded state after every step, from HOME."""
    current = dict(HOME)
    for pose, dwell in WAVE:
        targets = pose_targets(pose)
        yield pose, dwell, targets, dict(current)
        current.update(targets)


def test_every_wave_pose_names_only_right_arm_joints():
    for pose in WAVE_POSES:
        assert pose.startswith('wave_'), pose
        assert set(POSES[pose]) == set(RIGHT_JOINTS), (
            f'{pose} must name all six right-arm joints and nothing else')


def test_the_left_arm_stays_at_home_through_the_wave():
    current = dict(HOME)
    for pose, _ in WAVE:
        current.update(pose_targets(pose))
        for name, value in current.items():
            if name.startswith('left_arm_'):
                assert value == HOME[name], f'{pose} moved {name}'


def test_the_probe_is_the_first_move_and_only_swings_the_base_forward():
    # Verified on the metal 2026-10-01: yaw negative = forward. The probe
    # is the one step whose direction a human already knows.
    assert WAVE[1][0] == 'wave_probe'
    probe = POSES['wave_probe']
    assert probe['right_arm_yaw_joint'] == pytest.approx(-0.25)
    for name, value in probe.items():
        if 'yaw' not in name:
            assert value == 0.0, f'the probe must not move {name}'


def test_arm_values_keep_a_tenth_of_a_radian_inside_every_limit():
    for pose in WAVE_POSES:
        for name, value in POSES[pose].items():
            if 'finger' in name:
                continue   # fingers use the measured 0.0 / 1.0 convention
            spec = SERVO_MAP[name]
            assert spec.lower + 0.1 <= value <= spec.upper - 0.1, (
                f'{pose}.{name}={value} is closer than 0.1 rad to a limit')


def test_three_waves_with_the_hand_opening_on_every_out_swing():
    opens = [i for i, (pose, _) in enumerate(WAVE) if pose == 'wave_open']
    assert len(opens) == 3
    for i in opens:
        assert WAVE[i + 1][0] == 'wave_close', 'every open is followed by a close'
    assert POSES['wave_open']['right_arm_finger_l_joint'] == 1.0
    assert POSES['wave_close']['right_arm_finger_l_joint'] == 0.0


def test_the_arm_is_raised_with_the_base_not_the_shoulder():
    for pose in WAVE_POSES:
        assert POSES[pose]['right_arm_shoulder_joint'] == 0.0, pose
    assert POSES['wave_raise_2']['right_arm_yaw_joint'] < 0.0


def test_the_wave_comes_down_the_way_it_went_up():
    names = [pose for pose, _ in WAVE]
    up = names[:names.index('wave_open')]
    down = names[len(names) - len(up):]
    assert down == list(reversed(up))


def test_every_dwell_leaves_a_quarter_second_of_margin():
    for pose, dwell, targets, current in _walk():
        assert dwell - settle_time_s(targets, current) >= 0.25 - 1e-9, (
            f'{pose}: dwell {dwell}s leaves no margin over the ramp')
