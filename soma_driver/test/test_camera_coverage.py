"""scripts/camera_coverage.py: reading /joint_states, naming poses, judging a blob.

Pure tests: the camera, numpy and OpenCV are imported inside the
functions that need them, so this runs anywhere.
"""
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))

import camera_coverage as cc  # noqa: E402

ECHO = """header:
  stamp:
    sec: 1791862151
    nanosec: 0
  frame_id: ''
name:
- right_arm_yaw_joint
- left_arm_yaw_joint
- left_arm_elbow_joint
position:
- -0.52
- 0.0
- -0.0
velocity: []
effort: []
---
"""


def test_importing_pulls_in_no_camera_numpy_or_opencv():
    probe = ('import sys; sys.path.insert(0, sys.argv[1]); import camera_coverage; '
             "print([m for m in ('depthai', 'cv2', 'numpy') if m in sys.modules])")
    out = subprocess.run([sys.executable, '-c', probe, str(REPO / 'scripts')],
                         capture_output=True, text=True, check=True)
    assert out.stdout.strip() == '[]'


def test_joint_states_are_parsed_from_the_echo():
    assert cc.parse_joint_states(ECHO) == {
        'right_arm_yaw_joint': -0.52, 'left_arm_yaw_joint': 0.0, 'left_arm_elbow_joint': 0.0}


@pytest.mark.parametrize('text', ['', 'garbage', 'name:\n- a\nposition:\n- x\n',
                                  'name:\n- a\n- b\nposition:\n- 1.0\n'])
def test_anything_else_parses_to_nothing(text):
    assert cc.parse_joint_states(text) == {}


@pytest.mark.parametrize('state, name', [
    ({}, 'home'),
    ({'right_arm_yaw_joint': -0.01, 'left_arm_yaw_joint': 0.02}, 'home'),
    ({'right_arm_yaw_joint': -0.52}, 'right_30'),
    ({'left_arm_yaw_joint': -0.70}, 'left_40'),
    ({'right_arm_yaw_joint': -0.26, 'left_arm_yaw_joint': -0.26}, 'right_15+left_15'),
])
def test_poses_are_named_by_the_raised_bases(state, name):
    assert cc.pose_name(state) == name


def test_a_blob_away_from_the_borders_is_in_frame():
    assert cc.verdict((100, 100, 50, 80), 640, 1136) == ('in frame', [])


def test_a_blob_touching_borders_is_cut_and_says_where():
    text, edges = cc.verdict((0, 900, 100, 236), 640, 1136)
    assert edges == ['left', 'bottom'] and text == 'cut at left and bottom'


def test_no_blob_is_not_seen():
    assert cc.verdict(None, 640, 1136) == ('not seen', [])


def test_frames_turn_the_same_way_as_the_viewer():
    for script in ('camera_coverage.py', 'oak_view.py'):
        assert 'ROTATE_90_COUNTERCLOCKWISE' in (REPO / 'scripts' / script).read_text()


def test_the_script_commands_nothing():
    source = (REPO / 'scripts' / 'camera_coverage.py').read_text()
    for forbidden in ('soma/command', 'soma/arm', 'SetBool', 'publish', 'topic pub'):
        assert forbidden not in source
