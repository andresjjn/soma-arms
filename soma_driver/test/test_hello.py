"""The hello: a right-arm greeting that flows, inside what the bench measured.

Pure tests, no ROS. The generic contract (defined poses, home at both ends,
dwells covering the ramp, no joint moving over 1 rad in a step) is enforced
for every sequence in test_primitives.py; this file pins the hello's own
promises.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.primitives import (  # noqa: E402
    ELBOW_REACH_CLEAN, HOME, POSES, SEQUENCES, settle_time_s)
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

HELLO = SEQUENCES['hello']
POSE_NAMES = sorted({p for p, _ in HELLO} - {'home'})


def test_only_the_right_arm_moves():
    for pose in POSE_NAMES:
        assert all(n.startswith('right_arm_') for n in POSES[pose]), pose


def test_every_arm_value_keeps_a_tenth_inside_the_real_limits():
    for pose in POSE_NAMES:
        for name, value in POSES[pose].items():
            if 'finger' in name:
                continue
            spec = SERVO_MAP[name]
            assert spec.lower + 0.1 <= value <= spec.upper - 0.1, f'{pose}.{name}'


def test_the_fold_stays_well_short_of_where_the_body_was_measured():
    for pose in POSE_NAMES:
        assert POSES[pose]['right_arm_elbow_joint'] >= ELBOW_REACH_CLEAN + 0.3 - 1e-9, pose


def test_the_elbow_is_folded_before_the_base_passes_the_horizontal():
    # The straight arm at 90 deg would ask the base for 11.4 kg.cm.
    assert POSES['hello_rise']['right_arm_yaw_joint'] > -1.3
    assert POSES['hello_rise']['right_arm_elbow_joint'] <= -0.9


def test_five_elbow_swings_with_the_hand_opening_on_every_out():
    # Andres: at the top the elbow waves the forearm, pointing at the
    # ceiling, the wrist stays in line.
    poses = [p for p, _ in HELLO]
    assert poses.count('hello_out') == 5 and poses.count('hello_in') == 5
    top, out, inn = POSES['hello_top'], POSES['hello_out'], POSES['hello_in']
    assert out['right_arm_finger_l_joint'] == 1.0 and inn['right_arm_finger_l_joint'] == 0.0
    elbow = 'right_arm_elbow_joint'
    assert out[elbow] - top[elbow] == pytest.approx(top[elbow] - inn[elbow])
    for pose in (top, out, inn):
        assert pose['right_arm_wrist_roll_joint'] == 0.0
        assert pose['right_arm_wrist_pitch_joint'] == 0.0
        for joint in ('yaw', 'shoulder'):
            assert pose[f'right_arm_{joint}_joint'] == top[f'right_arm_{joint}_joint']


def test_the_forearm_never_crosses_the_vertical_while_it_waves():
    # Base + shoulder + elbow is the forearm's angle from hanging; past pi
    # the elbow's gravity torque flips sign mid-swing (gear play).
    for pose in ('hello_top', 'hello_out', 'hello_in'):
        q = POSES[pose]
        forearm = -(q['right_arm_yaw_joint'] + q['right_arm_shoulder_joint']
                    + q['right_arm_elbow_joint'])
        assert forearm < 3.14159 - 0.03, pose


def test_the_top_is_as_high_as_the_base_and_shoulder_go():
    top = POSES['hello_top']
    for joint in ('yaw', 'shoulder'):
        name = f'right_arm_{joint}_joint'
        assert top[name] == pytest.approx(SERVO_MAP[name].lower + 0.1, abs=0.01)


def test_it_flows_every_dwell_is_its_travel_time_plus_a_tenth():
    state = dict(HOME)
    for pose, dwell in HELLO:
        assert dwell == pytest.approx(settle_time_s(POSES[pose], state) + 0.1, abs=0.01), pose
        state.update(POSES[pose])


def test_it_ends_home_with_the_gripper_closed():
    assert HELLO[-1][0] == 'home'
    assert HOME['right_arm_finger_l_joint'] == 0.0
