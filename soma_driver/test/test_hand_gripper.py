"""The hand-to-gripper bench experiment: what it reads, what it may send.

Pure tests. The script imports depthai only inside track(), rclpy only
inside bridge() and OpenCV only where it draws, so none of them is needed
here; the drawing is checked against a stand-in for cv2.
"""
import math
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'soma_driver'))
sys.path.insert(0, str(REPO / 'scripts'))

import hand_gripper as hg  # noqa: E402
from soma_driver.servo_map import MIMIC_JOINTS, SERVO_MAP  # noqa: E402


def hand(thumb, tips=((-0.3, -2.0), (-0.1, -2.1), (0.1, -2.05), (0.3, -1.9)),
         scale=1.0, shift=(0.0, 0.0)):
    """21 landmarks: wrist at the origin, middle knuckle one palm up."""
    pts = [(0.0, 0.0)] * 21
    pts[hg.MIDDLE_MCP] = (0.0, -1.0)
    pts[hg.THUMB_TIP] = thumb
    for i, tip in zip(hg.FINGERTIPS, tips):
        pts[i] = tip
    return [(x * scale + shift[0], y * scale + shift[1]) for x, y in pts]


OPEN = hand(thumb=(-1.2, -1.0))      # thumb out to the side
PINCH = hand(thumb=(0.0, -1.95))     # thumb on the fingertips


def test_importing_the_script_pulls_in_no_camera_ros_or_opencv():
    # In a fresh interpreter: in the ROS job other tests import rclpy first,
    # so this process's sys.modules proves nothing about the script.
    import subprocess
    probe = ('import sys; sys.path.insert(0, sys.argv[1]); import hand_gripper; '
             "print([m for m in ('depthai', 'depthai_nodes', 'rclpy', 'cv2') "
             'if m in sys.modules])')
    out = subprocess.run([sys.executable, '-c', probe, str(REPO / 'scripts')],
                         capture_output=True, text=True, check=True)
    assert out.stdout.strip() == '[]'


def test_ratio_reads_open_high_and_pinch_low():
    assert hg.hand_ratio(OPEN) > 1.2
    assert hg.hand_ratio(PINCH) < 0.1


def test_ratio_does_not_change_with_distance_or_position_in_the_frame():
    near = hand(thumb=(-1.2, -1.0), scale=0.4, shift=(0.3, 0.6))
    assert hg.hand_ratio(near) == pytest.approx(hg.hand_ratio(OPEN))


def test_a_palm_tilted_toward_the_lens_does_not_inflate_the_ratio():
    flat = hand(thumb=(-1.2, -1.0))
    flat[hg.INDEX_MCP], flat[hg.PINKY_MCP] = (-0.38, -0.95), (0.38, -0.85)
    tilted = list(flat)
    tilted[hg.MIDDLE_MCP] = (0.0, -0.4)      # the palm's length foreshortened
    plain = math.dist(tilted[hg.THUMB_TIP], (0.0, -2.0125)) / 0.4
    assert plain > 3.0                        # what dividing by length gave
    assert hg.hand_ratio(tilted) == pytest.approx(hg.hand_ratio(flat), rel=0.05)


def test_palm_size_is_the_longer_of_length_and_scaled_width():
    pts = hand(thumb=(-1.2, -1.0))
    pts[hg.INDEX_MCP], pts[hg.PINKY_MCP] = (-0.5, -1.0), (0.5, -1.0)
    assert hg.palm_size(pts) == pytest.approx(1.0 * hg.WIDTH_TO_LENGTH)
    pts[hg.INDEX_MCP], pts[hg.PINKY_MCP] = (-0.1, -1.0), (0.1, -1.0)
    assert hg.palm_size(pts) == pytest.approx(1.0)


def test_degenerate_landmarks_give_no_reading():
    assert hg.hand_ratio(OPEN[:20]) is None
    assert hg.hand_ratio([(0.5, 0.5)] * 21) is None


def test_openness_maps_the_two_ratios_to_the_ends_and_saturates():
    assert hg.openness(hg.CLOSED_RATIO) == 0.0
    assert hg.openness(hg.OPEN_RATIO) == 1.0
    assert hg.openness(0.0) == 0.0
    assert hg.openness(5.0) == 1.0
    mid = (hg.CLOSED_RATIO + hg.OPEN_RATIO) / 2
    assert hg.openness(mid) == pytest.approx(0.5)


def test_follower_refuses_an_open_ratio_not_above_closed():
    with pytest.raises(ValueError):
        hg.Follower(closed=1.0, open_=1.0)


def test_no_hand_never_moves_the_gripper():
    f = hg.Follower()
    assert f.update(None) is None
    assert f.level is None and f.sent is None
    f.update(hg.OPEN_RATIO)
    assert f.update(None) is None
    assert f.sent == 1.0


def test_a_misread_is_skipped_even_as_the_first_reading():
    f = hg.Follower()
    assert f.update(hg.MAX_RATIO + 1.0) is None
    assert f.level is None
    assert f.update(hg.OPEN_RATIO) == 1.0
    assert f.update(3.3) is None
    assert f.sent == 1.0


def at(level):
    """The ratio that reads as this openness."""
    return hg.CLOSED_RATIO + level * (hg.OPEN_RATIO - hg.CLOSED_RATIO)


def raw_follower(**kw):
    """A follower without median or smoothing: targets follow the readings."""
    return hg.Follower(median_of=1, smoothing=1.0, **kw)


def test_deadband_holds_small_changes_between_the_ends():
    f = raw_follower()
    assert f.update(at(0.5)) == 0.5
    assert f.update(at(0.52)) is None
    assert f.update(at(0.6)) == 0.6


def test_smoothing_moves_part_of_the_way():
    f = hg.Follower(median_of=1, smoothing=0.5, deadband=0.0)
    assert f.update(at(0.0)) == 0.0
    assert f.update(at(1.0)) == 0.5


def test_a_closed_hand_that_flutters_keeps_the_gripper_shut():
    f = hg.Follower()
    noise = [0.05, 0.22, 0.0, 0.25, 0.12, 0.24, 0.02, 0.2, 0.18, 0.25] * 5
    sent = [t for t in (f.update(at(v)) for v in noise) if t is not None]
    assert sent == [0.0]


def test_the_closed_end_lets_go_only_past_the_hysteresis():
    f = raw_follower()
    assert f.update(at(0.1)) == 0.0
    assert f.update(at(0.28)) is None         # still inside 0.2 + 0.1
    assert f.update(at(0.31)) == 0.31


def test_the_open_end_holds_the_same_way():
    f = raw_follower()
    assert f.update(at(0.9)) == 1.0
    assert f.update(at(0.78)) is None         # still above 0.85 - 0.1
    assert f.update(at(0.74)) == 0.74


def test_a_one_frame_spike_never_reaches_the_gripper():
    f = hg.Follower(median_of=3, smoothing=1.0)
    assert f.update(at(0.5)) == 0.5
    assert [f.update(at(v)) for v in (0.5, 1.0, 0.5, 0.5)] == [None] * 4


def test_held_ends_that_overlap_are_refused():
    with pytest.raises(ValueError):
        hg.Follower(close_below=0.5, open_above=0.6)


def test_every_target_is_inside_zero_to_one():
    f = hg.Follower(smoothing=1.0, deadband=0.0)
    for ratio in (-1.0, 0.0, 0.2, 0.7, 1.4, 1.99):
        target = f.update(ratio)
        assert target is None or 0.0 <= target <= 1.0


def test_lock_takes_the_most_confident_hand_first():
    lock = hg.HandLock()
    right = hand(thumb=(-1.2, -1.0), scale=0.1, shift=(0.7, 0.5))
    left = hand(thumb=(0.0, -1.95), scale=0.1, shift=(0.2, 0.5))
    assert lock.choose([(0.6, left), (0.9, right)], now=0.0) is right


def test_lock_keeps_its_hand_when_a_more_confident_one_appears():
    lock = hg.HandLock()
    mine = hand(thumb=(-1.2, -1.0), scale=0.1, shift=(0.7, 0.5))
    other = hand(thumb=(0.0, -1.95), scale=0.1, shift=(0.2, 0.5))
    lock.choose([(0.6, mine)], now=0.0)
    moved = hand(thumb=(0.0, -1.95), scale=0.1, shift=(0.72, 0.52))
    assert lock.choose([(0.99, other), (0.55, moved)], now=0.1) is moved


def test_lock_holds_through_a_blink_then_lets_go():
    lock = hg.HandLock(hold_s=1.0)
    mine = hand(thumb=(-1.2, -1.0), scale=0.1, shift=(0.7, 0.5))
    other = hand(thumb=(0.0, -1.95), scale=0.1, shift=(0.2, 0.5))
    lock.choose([(0.9, mine)], now=0.0)
    assert lock.choose([(0.9, other)], now=0.5) is None     # gripper holds
    assert lock.choose([(0.9, other)], now=1.6) is other    # lock released


def test_frame_step_survives_a_first_frame_that_is_a_misread():
    lock, f = hg.HandLock(), hg.Follower()
    wild = hand(thumb=(-3.5, -1.0))          # ratio above MAX_RATIO
    kpts, ratio, target = hg.frame_step([(0.9, wild)], lock, f, 0.0, 0.5)
    assert kpts is wild and ratio > hg.MAX_RATIO and target is None
    assert 'misread' in hg.status_text(ratio, f.level, f.max_ratio)


def test_frame_step_ignores_hands_below_the_confidence_floor():
    lock, f = hg.HandLock(), hg.Follower()
    assert hg.frame_step([(0.3, OPEN)], lock, f, 0.0, 0.5) == (None, None, None)


def test_status_text_never_needs_a_level():
    assert hg.status_text(None, None) == 'no hand'
    assert 'gripper -' in hg.status_text(0.8, None)
    assert '[' in hg.status_text(0.8, 0.5)


def test_drawing_before_any_reading_does_not_crash(monkeypatch):
    calls = []
    fake = types.SimpleNamespace(
        FONT_HERSHEY_SIMPLEX=0,
        circle=lambda *a, **k: calls.append('circle'),
        putText=lambda img, text, *a, **k: calls.append(text))
    monkeypatch.setitem(sys.modules, 'cv2', fake)

    class Img:
        shape = (768, 768, 3)

        def copy(self):
            return self

    wild = hand(thumb=(-3.5, -1.0), scale=0.1, shift=(0.5, 0.5))
    hg._draw(Img(), [(0.9, wild)], wild, None)
    assert 'gripper -' in calls
    hg._draw(Img(), [], None, None)
    assert 'no hand' in calls


@pytest.mark.parametrize('line, value', [
    ('0.42\n', 0.42), ('  1\n', 1.0), ('0', 0.0), ('-0.5', 0.0), ('7', 1.0)])
def test_bridge_parses_and_clamps_targets(line, value):
    assert hg.parse_target(line) == value


@pytest.mark.parametrize('line', ['', '\n', 'open', 'nan', 'inf', '-inf', '0.5 0.6'])
def test_bridge_skips_anything_that_is_not_one_finite_number(line):
    assert hg.parse_target(line) is None


def test_status_lines_never_block_the_camera_loop():
    import threading
    gate = threading.Event()

    class StuckTerminal:
        def write(self, text):
            gate.wait()           # a terminal that stopped reading

        def flush(self):
            pass

    say = hg._Reporter(StuckTerminal(), depth=4)
    started = __import__('time').monotonic()
    for i in range(1000):
        say(f'line {i}')
    assert __import__('time').monotonic() - started < 0.5
    gate.set()


def test_bridge_drives_only_the_driven_finger_of_one_arm():
    for arm in ('right', 'left'):
        joint = hg.joint_for(arm)
        assert joint in SERVO_MAP and joint not in MIMIC_JOINTS
        spec = SERVO_MAP[joint]
        assert (spec.lower, spec.upper) == (0.0, 1.0)   # the 0..1 the pipe carries
    for bad in ('both', 'torso', 'right_arm_elbow_joint', ''):
        with pytest.raises(ValueError):
            hg.joint_for(bad)


@pytest.mark.parametrize('argv', [['--open', '0.4'], ['--smoothing', '0'],
                                  ['--close-below', '0.8']])
def test_bad_tuning_is_refused_before_the_camera_opens(argv):
    assert hg.main(argv) == 2


def test_the_script_cannot_arm_the_driver():
    source = (REPO / 'scripts' / 'hand_gripper.py').read_text()
    assert 'SetBool' not in source
    assert "'soma/arm'" not in source and '"soma/arm"' not in source
    assert "create_publisher(JointState, 'soma/command'" in source


def test_the_web_view_listens_on_localhost_only():
    source = (REPO / 'scripts' / 'hand_gripper.py').read_text()
    assert "ThreadingHTTPServer(('127.0.0.1'" in source
    assert '0.0.0.0' not in source
