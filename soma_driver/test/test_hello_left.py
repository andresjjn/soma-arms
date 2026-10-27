"""The left hello: four poses Andres set by hand in the workbench, played.

Pure tests, no ROS. The generic contract (soft band with margin, home at
both ends, dwells covering the ramp, no joint over 1 rad in a step) runs
for every sequence in test_primitives.py.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.primitives import (  # noqa: E402
    _LEFT_KEYS_US, ELBOW_REACH_CLEAN, HOME, POSES, SEQUENCES)
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

SEQ = SEQUENCES['hello_left']


def test_each_key_pose_lands_on_the_pulses_read_off_the_workbench():
    for pose, pulses in _LEFT_KEYS_US.items():
        for joint, us in pulses.items():
            name = f'left_arm_{joint}_joint'
            assert SERVO_MAP[name].command_to_us(POSES[pose][name]) == pytest.approx(us, abs=2), (pose, joint)


def test_only_the_left_arm_moves():
    for pose, _ in SEQ:
        if pose != 'home':
            assert all(n.startswith('left_arm_') for n in POSES[pose]), pose


def test_five_waves_out_open_in_closed():
    poses = [p for p, _ in SEQ]
    assert poses.count('hello_left_3') == 5 and poses.count('hello_left_4') == 5
    assert POSES['hello_left_3']['left_arm_finger_l_joint'] > 0.5
    assert POSES['hello_left_4']['left_arm_finger_l_joint'] < 0.05


def test_the_elbow_is_folded_while_the_base_passes_the_horizontal():
    rise = POSES['hello_left_rise']
    assert rise['left_arm_yaw_joint'] > -1.3 and rise['left_arm_elbow_joint'] <= -0.9
    for pose, _ in SEQ:
        if pose != 'home':
            assert POSES[pose]['left_arm_elbow_joint'] >= ELBOW_REACH_CLEAN + 0.3 - 1e-9, pose


def test_it_ends_home_closed():
    assert SEQ[-1][0] == 'home' and HOME['left_arm_finger_l_joint'] == 0.0
