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


def test_the_script_can_disarm_but_never_arm_the_driver():
    # Since the release on a lost hand the bridge calls /soma/arm, so the
    # old "never names SetBool" check moved here: one request, built with
    # data False, and no way to write True into it.
    import re
    source = (REPO / 'scripts' / 'hand_gripper.py').read_text()
    assert source.count('SetBool.Request()') == 1
    assert source.count('request.data = False') == 1
    assert not re.search(r'data\s*[=:]\s*(True|true|1)\b', source)
    assert source.count("'soma/arm'") == 1
    assert "create_publisher(JointState, 'soma/command'" in source


def test_the_web_view_listens_on_localhost_only():
    source = (REPO / 'scripts' / 'hand_gripper.py').read_text()
    assert "ThreadingHTTPServer(('127.0.0.1'" in source
    assert '0.0.0.0' not in source


# The present gesture: follow, then on open-then-close lift and show.

def show_run(arm='left', present=True, hand_script=None, end_at=None,
             present_max_s=hg.PRESENT_MAX_S, dt=0.1, until=60.0, lift=hg.LIFT):
    """Drive a Presenter with a fake clock; return [(t, {joint: value})]."""
    brain = hg.Presenter(arm, present, present_max_s, lift=lift)
    script = hand_script if hand_script is not None else [
        (0.5, 0.0), (1.0, 1.0), (2.0, 0.6), (2.3, 0.0)]
    out, t = [], 0.0
    while t <= until and not brain.done:
        for when, value in script:
            if abs(when - t) < dt / 2:
                out.append((t, brain.hand(t, value)))
        if end_at is not None and abs(end_at - t) < dt / 2:
            out.append((t, brain.finish(t)))
        out.append((t, brain.tick(t)))
        t = round(t + dt, 3)
    return brain, [(t, c) for t, c in out if c]


def series(cmds, joint):
    return [(t, c[joint]) for t, c in cmds if joint in c]


def test_without_present_only_the_finger_ever_moves():
    brain, cmds = show_run(present=False, until=10.0)
    assert {j for _, c in cmds for j in c} == {'left_arm_finger_l_joint'}
    assert brain.finish(10.0) == {} and brain.done


def test_open_then_close_lifts_the_arm_shows_and_comes_home_on_its_own():
    brain, cmds = show_run(present_max_s=10.0)
    assert brain.done
    names = {j for _, c in cmds for j in c}
    assert names == {f'left_arm_{j}_joint' for j in ('yaw', 'wrist_roll', 'finger_l')}
    yaw = series(cmds, 'left_arm_yaw_joint')
    assert min(v for _, v in yaw) == pytest.approx(-hg.LIFT)
    for joint in ('yaw', 'wrist_roll', 'finger_l'):
        assert series(cmds, f'left_arm_{joint}_joint')[-1][1] == 0.0   # all home


def test_a_short_open_hand_does_not_trigger():
    brain, cmds = show_run(hand_script=[(1.0, 1.0), (1.2, 0.0)], until=8.0)
    assert brain.phase == 'follow'
    assert not series(cmds, 'left_arm_yaw_joint')


def test_entering_with_a_closed_hand_does_not_trigger():
    brain, _ = show_run(hand_script=[(0.5, 0.0), (1.0, 0.0), (3.0, 0.0)], until=6.0)
    assert brain.phase == 'follow'


def test_after_the_trigger_the_gripper_stays_shut_and_the_hand_is_ignored():
    script = [(0.5, 0.0), (1.0, 1.0), (2.0, 0.0), (5.0, 1.0), (9.0, 1.0)]
    _, cmds = show_run(hand_script=script, present_max_s=5.0)
    finger = series(cmds, 'left_arm_finger_l_joint')
    trigger = next(t for t, v in finger if t >= 2.0)
    assert all(v == 0.0 for t, v in finger if t >= trigger)


def test_the_arm_lifts_straight_the_elbow_and_the_rest_are_never_named():
    # 2026-10-12: a curled elbow jammed the left forearm against the
    # central box where the model saw 45 mm. Only the base lifts now.
    _, cmds = show_run(present_max_s=5.0)
    for joint in ('elbow', 'shoulder', 'wrist_pitch'):
        assert not series(cmds, f'left_arm_{joint}_joint')
    assert not any(name.startswith('right_arm') for _, c in cmds for name in c)


def test_the_wrist_is_level_before_the_base_comes_down():
    _, cmds = show_run(present_max_s=5.0)
    roll = series(cmds, 'left_arm_wrist_roll_joint')
    yaw = series(cmds, 'left_arm_yaw_joint')
    top = min(t for t, v in yaw if v == pytest.approx(-hg.LIFT))
    lowering = min(t for t, v in yaw if t > top and v > -hg.LIFT + 1e-6)
    assert all(v == 0.0 for t, v in roll if t >= lowering)


def test_every_streamed_target_is_slow_and_well_inside_the_limits():
    _, cmds = show_run(present_max_s=8.0)
    for joint in ('yaw', 'wrist_roll'):
        name = f'left_arm_{joint}_joint'
        spec = SERVO_MAP[name]
        values = series(cmds, name)
        for _, v in values:   # 0.3 rad from every stop, or the hang itself
            assert spec.lower + 0.3 <= v <= max(spec.upper - 0.3, 0.0)
        for (t0, a), (t1, b) in zip(values, values[1:]):
            if t1 > t0:
                assert abs(b - a) / (t1 - t0) <= 2.0 < spec.max_rate


def test_the_wrist_rolls_both_ways_by_the_amplitude_only():
    _, cmds = show_run(present_max_s=10.0)
    roll = [v for _, v in series(cmds, 'left_arm_wrist_roll_joint')]
    assert max(roll) == pytest.approx(hg.ROLL_AMPLITUDE, abs=0.01)
    assert min(roll) == pytest.approx(-hg.ROLL_AMPLITUDE, abs=0.01)


def test_ending_mid_raise_comes_home_from_where_it_is_without_a_jump():
    brain, cmds = show_run(end_at=3.5, until=20.0)    # base halfway up
    assert brain.done
    values = [v for _, v in series(cmds, 'left_arm_yaw_joint')]
    assert min(values) > -hg.LIFT + 0.05
    assert all(abs(b - a) <= 0.2 for a, b in zip(values, values[1:]))
    assert values[-1] == 0.0


def test_the_show_has_a_time_limit():
    brain, cmds = show_run(present_max_s=4.0, until=30.0)
    assert brain.done
    last = max(t for t, _ in cmds)
    assert last < 2.3 + hg.YAW_RAMP_S + hg.SHOW_PAUSE_S + 4.0 + hg.ROLL_HOME_S + hg.YAW_RAMP_S + 0.5


def test_the_present_numbers_respect_the_measured_stops_on_both_arms():
    for arm in hg.ARMS:
        for joint, value in (('yaw', -hg.LIFT), ('yaw', -hg.LIFT_MAX),
                             ('wrist_roll', hg.ROLL_AMPLITUDE),
                             ('wrist_roll', -hg.ROLL_AMPLITUDE)):
            spec = SERVO_MAP[f'{arm}_arm_{joint}_joint']
            assert spec.lower + 0.3 <= value <= spec.upper - 0.3


def test_present_refuses_an_arm_away_from_home():
    home = {f'left_arm_{j}_joint': 0.0 for j in
            ('yaw', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll')}
    assert hg.away_from_home(home, 'left') == []
    bent = dict(home, left_arm_elbow_joint=-0.6)
    assert hg.away_from_home(bent, 'left') == ['left_arm_elbow_joint']
    assert hg.away_from_home({}, 'left')            # no state is not home


@pytest.mark.parametrize('argv', [['--present-max', '0'], ['--present-max', '120'],
                                  ['--roll', '0.9'], ['--lift', '0'], ['--lift', '1.2'],
                                  ['--rolls', '0'], ['--rolls', '11']])
def test_bad_show_settings_are_refused(argv):
    assert hg.main(argv) == 2


def test_a_smaller_lift_for_a_first_try():
    _, cmds = show_run(present_max_s=4.0, lift=0.3)
    assert min(v for _, v in series(cmds, 'left_arm_yaw_joint')) == pytest.approx(-0.3)


def test_a_lift_past_the_designed_load_is_refused():
    with pytest.raises(ValueError):
        hg.Presenter('left', True, lift=hg.LIFT_MAX + 0.1)


def test_a_lost_hand_is_reported_once_and_only_after_a_first_hand():
    lost = hg.LostHand(after_s=2.0)
    assert not any(lost.update(t / 10, False) for t in range(50))   # never seen yet
    assert not lost.update(5.0, True)
    assert not lost.update(6.0, False)
    assert not lost.update(7.9, False)
    assert lost.update(8.0, False)
    assert not lost.update(9.0, False)                                # once


def test_a_hand_back_in_view_resets_the_clock():
    lost = hg.LostHand(after_s=2.0)
    lost.update(0.0, True)
    lost.update(1.0, False)
    lost.update(2.5, True)
    assert not lost.update(4.0, False)
    assert lost.update(6.0, False)


def test_release_while_following_stops_at_once_for_the_cut():
    brain = hg.Presenter('left', present=False)
    brain.hand(0.0, 0.6)
    assert brain.release(1.0) == {} and brain.done and brain.released


def finish_ticking(brain, t, limit):
    """Tick a Presenter to done; return (end time, roll values, last targets)."""
    last, rolls = {}, []
    while not brain.done:
        out = brain.tick(t)
        last.update(out)
        if 'left_arm_wrist_roll_joint' in out:
            rolls.append(out['left_arm_wrist_roll_joint'])
        t = round(t + 0.1, 3)
        assert t < limit
    return t, rolls, last


def peaks(values):
    """How many times a series reaches its positive amplitude."""
    top = max(values)
    return sum(1 for a, b in zip(values, values[1:]) if b >= top - 1e-3 > a)


def test_release_during_the_show_waits_for_the_rolls_then_walks_home():
    # 2026-10-12: Andres took his hand away right after the trigger; the
    # release cut the show before the wrist had rolled once.
    brain, cmds = show_run(present_max_s=30.0, until=6.0)
    assert brain.phase == 'show'
    assert brain.release(6.0) == {} and brain.phase == 'show'    # deferred
    end, rolls, last = finish_ticking(brain, 6.0, 40.0)
    all_rolls = [v for _, v in series(cmds, 'left_arm_wrist_roll_joint')] + rolls
    assert peaks(all_rolls) == hg.ROLLS
    assert brain.released
    assert last['left_arm_yaw_joint'] == 0.0 and last['left_arm_wrist_roll_joint'] == 0.0


def test_release_during_the_raise_also_waits_for_the_show():
    brain = hg.Presenter('left', True)
    for t, v in ((0.5, 0.0), (1.0, 1.0), (2.0, 0.0)):
        brain.hand(t, v)
    assert brain.phase == 'raise'
    brain.release(2.5)
    _, rolls, _ = finish_ticking(brain, 2.5, 40.0)
    assert peaks(rolls) == hg.ROLLS and brain.released


def test_end_of_input_still_comes_home_at_once_after_a_release():
    brain, _ = show_run(present_max_s=30.0, until=6.0)
    brain.release(6.0)
    brain.finish(6.5)                                   # Ctrl-C
    assert brain.phase == 'home'
    end, _, _ = finish_ticking(brain, 6.5, 12.0)
    assert end < 6.5 + hg.ROLL_HOME_S + hg.YAW_RAMP_S + 0.5


def test_without_a_release_the_show_does_its_rolls_and_comes_home():
    brain, cmds = show_run(present_max_s=30.0, until=40.0)
    assert brain.done and not brain.released
    assert peaks([v for _, v in series(cmds, 'left_arm_wrist_roll_joint')]) == hg.ROLLS


def test_asking_to_release_twice_changes_nothing():
    brain = hg.Presenter('left', True)
    brain.hand(0.0, 0.5)
    assert brain.release(1.0) == {} and brain.done
    assert brain.release(1.5) == {}


def test_a_heavy_lift_caps_the_rolls():
    brain = hg.Presenter('left', True, 30.0, lift=hg.LIFT_MAX, rolls=8)
    assert brain.show_s == hg.HEAVY_MAX_S


def test_a_heavy_lift_gets_a_short_show():
    assert hg.Presenter('left', True, 30.0, lift=hg.LIFT_MAX).present_max_s == hg.HEAVY_MAX_S
    assert hg.Presenter('left', True, 30.0, lift=hg.LIFT).present_max_s == 30.0


def test_45_degrees_is_the_ceiling():
    assert hg.LIFT_MAX == pytest.approx(0.79)
    hg.Presenter('left', True, lift=0.785)            # 45 deg accepted
    with pytest.raises(ValueError):
        hg.Presenter('left', True, lift=0.8)
