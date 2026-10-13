"""Step mode is a promise about what gets published, so it is tested blind.

Pure tests, no ROS: the player receives fake publish, sleep and ask
callables and the assertions read what it did with them. Every way out
of a walk (the end of input, Ctrl-C at a prompt or mid-move, an error)
is held against the one already trusted, q at a prompt, and the way home
against the rule under all of them: only ground the walk already covered.
"""
import io
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.player import (  # noqa: E402
    PROMPT, SETTLE_MARGIN_S, fresh_input, parse_cli, plan, play, unwind)
from soma_driver.primitives import (  # noqa: E402
    HOME, SEQUENCES, pose_targets, settle_time_s)
from soma_driver.servo_map import SERVO_MAP  # noqa: E402
from soma_driver.trajectory import JointMotion  # noqa: E402

DT = 0.02  # the driver tick (arm_controller_node.RATE_HZ, 50 Hz)


class Recorder:
    def __init__(self, answers=(), ctrl_c_in_sleep=(), ctrl_c_in_publish=()):
        self.published: list[tuple[str, dict]] = []
        self.slept: list[float] = []
        self.events: list[tuple[str, object]] = []   # both of the above, in order
        self.asked = 0
        self.answers = list(answers)
        # Calls, counted from 0, that a Ctrl-C cuts short.
        self.ctrl_c_in_sleep = set(ctrl_c_in_sleep)
        self.ctrl_c_in_publish = set(ctrl_c_in_publish)
        self.publish_calls = 0

    def publish(self, pose, targets):
        self.publish_calls += 1
        if self.publish_calls - 1 in self.ctrl_c_in_publish:
            raise KeyboardInterrupt   # whether it went out is anyone's guess
        self.published.append((pose, dict(targets)))
        self.events.append(('publish', pose))

    def sleep(self, seconds):
        self.slept.append(seconds)
        self.events.append(('sleep', seconds))
        if len(self.slept) - 1 in self.ctrl_c_in_sleep:
            raise KeyboardInterrupt

    def ask(self, prompt):
        assert prompt == PROMPT
        self.asked += 1
        answer = self.answers.pop(0)
        if isinstance(answer, BaseException):
            raise answer
        return answer


def never_ask(prompt):
    raise AssertionError('play must not prompt without step mode')


def q_after(k):
    """The reference way out: q at the prompt after step k of the wave."""
    rec = Recorder(answers=[''] * k + ['q'])
    assert play('wave', rec.publish, rec.sleep, ask=rec.ask, log=lambda _: None) is False
    return rec


def ctrl_c_in_dwell(k, name='wave', step_mode=True):
    """Play `name` with a Ctrl-C in the middle of step k, the arm moving."""
    rec = Recorder(answers=[''] * k if step_mode else (), ctrl_c_in_sleep={k})
    ask = rec.ask if step_mode else None
    assert play(name, rec.publish, rec.sleep, ask=ask, log=lambda _: None) is False
    return rec


def moves(states):
    """Every move between consecutive states, in both directions."""
    pairs = list(zip(states, states[1:]))
    return pairs + [(b, a) for a, b in pairs]


def reverse_in_flight(start, target, max_rate, cut_s):
    """Ramp start -> target the way the driver does, cut it after cut_s
    and command start again: the positions of the way back."""
    motion = JointMotion(start, max_rate)
    motion.set_target(target)
    for _ in range(round(cut_s / DT)):
        motion.step(DT)
    motion.set_target(start)
    back = []
    while not motion.done:
        back.append(motion.step(DT))
        assert len(back) < 1000, 'the reverse never settled'
    return back


@pytest.mark.parametrize('name', sorted(SEQUENCES))
def test_plan_follows_the_catalog_and_ends_at_home(name):
    steps = plan(name)
    assert [(s.pose, s.dwell) for s in steps] == list(SEQUENCES[name])
    assert [s.index for s in steps] == list(range(len(steps)))
    assert all(s.count == len(steps) for s in steps)
    assert steps[-1].state == HOME
    assert all(set(s.state) == set(HOME) for s in steps)


@pytest.mark.parametrize('name', sorted(SEQUENCES))
def test_play_publishes_every_step_and_sleeps_every_dwell(name):
    rec = Recorder()
    assert play(name, rec.publish, rec.sleep, ask=None, log=lambda _: None) is True
    assert rec.published == [(pose, pose_targets(pose)) for pose, _ in SEQUENCES[name]]
    assert rec.slept == [dwell for _, dwell in SEQUENCES[name]]


def test_no_prompt_without_step_mode():
    rec = Recorder()
    play('wave', rec.publish, rec.sleep, ask=None, log=lambda _: None)
    # ask=None is the contract; a prompting fake would have raised.
    play('wave', rec.publish, rec.sleep, log=lambda _: None)


def test_step_mode_prompts_after_every_step_but_the_last():
    count = len(SEQUENCES['wave'])
    rec = Recorder(answers=[''] * (count - 1))
    assert play('wave', rec.publish, rec.sleep, ask=rec.ask, log=lambda _: None) is True
    assert rec.asked == count - 1
    assert len(rec.published) == count


def test_q_unwinds_along_the_walked_path_to_home():
    rec = Recorder(answers=['', '', '', 'q'])   # abort after wave_raise_2
    logs = []
    assert play('wave', rec.publish, rec.sleep, ask=rec.ask, log=logs.append) is False
    walked = [pose for pose, _ in SEQUENCES['wave'][:4]]
    assert [p for p, _ in rec.published[:4]] == walked
    unwound = rec.published[4:]
    # Back through the proven states, newest first, ending at home.
    assert [p for p, _ in unwound] == ['wave_raise_1', 'wave_probe', 'home']
    assert unwound[-1][1] == HOME
    previous = plan('wave')[3].state
    for (_, state), wait in zip(unwound, rec.slept[4:]):
        for name, value in state.items():
            assert abs(value - previous[name]) <= 1.0, name
        assert wait >= settle_time_s(state, previous) + SETTLE_MARGIN_S - 1e-9
        previous = state
    assert any('unwinding' in line for line in logs)


def test_unwind_right_after_a_home_step_publishes_nothing():
    rec = Recorder(answers=['q'])
    assert play('wave', rec.publish, rec.sleep, ask=rec.ask, log=lambda _: None) is False
    assert [p for p, _ in rec.published] == ['home']


def test_end_of_input_counts_as_abort():
    rec = Recorder(answers=['', EOFError()])
    assert play('wave', rec.publish, rec.sleep, ask=rec.ask, log=lambda _: None) is False
    assert rec.published[-1][1] == HOME


@pytest.mark.parametrize('stop, said', [(EOFError(), 'end of input'),
                                        (KeyboardInterrupt(), 'interrupted')],
                         ids=['end_of_input', 'ctrl_c'])
def test_end_of_input_or_ctrl_c_at_a_prompt_unwinds_exactly_like_q(stop, said):
    rec = Recorder(answers=['', '', '', stop])   # after wave_raise_2
    logs = []
    assert play('wave', rec.publish, rec.sleep, ask=rec.ask, log=logs.append) is False
    assert rec.events == q_after(3).events
    assert f'wave: {said} at wave_raise_2, unwinding to home' in logs


def test_ctrl_c_moving_to_a_new_pose_undoes_the_move_exactly_like_q_after_it():
    # The way up is what step mode probes: every move there leads to a
    # pose never held before. Cut in the middle, it is undone, and from
    # the pose before it the way home is the one q after it would take.
    steps = plan('wave')
    new = [k for k, s in enumerate(steps)
           if s.state != HOME and all(s.state != t.state for t in steps[:k])]
    assert [steps[k].pose for k in new] == [
        'wave_probe', 'wave_raise_1', 'wave_raise_2', 'wave_open', 'wave_close']
    for k in new:
        assert ctrl_c_in_dwell(k).events == q_after(k).events, steps[k].pose


@pytest.mark.parametrize('k, way_home', [
    (6, ['wave_raise_2', 'wave_raise_1', 'wave_probe', 'home']),   # a swing again
    (11, ['wave_probe', 'home']),                                  # the way down
    (12, ['home']),
    (13, []),                                                      # the last move
])
def test_ctrl_c_on_covered_ground_lets_the_move_finish_then_goes_like_q(k, way_home):
    # A move that retraces one already made is not turned around: on the
    # way down, turning back would mean climbing again. It runs on for
    # the whole move's time with nothing published, and then the arm goes
    # home exactly as q after it would.
    steps = plan('wave')
    rec = ctrl_c_in_dwell(k)
    finish = settle_time_s(steps[k - 1].state, steps[k].state) + SETTLE_MARGIN_S
    assert rec.events[2 * k + 1:2 * k + 3] == [('sleep', steps[k].dwell),
                                                ('sleep', finish)]
    assert [p for p, _ in rec.published[k + 1:]] == way_home
    if k + 1 < len(steps):   # no prompt follows the last step
        assert rec.published == q_after(k).published


def test_ctrl_c_on_a_first_move_toward_a_pose_held_before_still_undoes_it():
    # wave_close -> wave_raise_2 is a move never made before, even though
    # wave_raise_2 was held: undone, then home over covered ground only,
    # which from wave_close runs back through the first swing.
    rec = ctrl_c_in_dwell(10)
    assert [p for p, _ in rec.published[11:]] == [
        'wave_close', 'wave_open', 'wave_raise_2', 'wave_raise_1',
        'wave_probe', 'home']


def test_ctrl_c_inside_a_publish_undoes_the_move_even_on_covered_ground():
    # Cut inside the command of the descending wave_probe: whether it went
    # out is unknown, so nothing is let finish. Undoing it asks for
    # wave_raise_1, which is a no-op if the command never left.
    rec = Recorder(answers=[''] * 12, ctrl_c_in_publish={12})
    assert play('wave', rec.publish, rec.sleep, ask=rec.ask, log=lambda _: None) is False
    assert [p for p, _ in rec.published[12:]] == ['wave_raise_1', 'wave_probe', 'home']


@pytest.mark.parametrize('name', sorted(SEQUENCES))
def test_ctrl_c_mid_move_does_the_same_with_or_without_step_mode(name):
    for k in range(len(SEQUENCES[name])):
        stepped = ctrl_c_in_dwell(k, name, step_mode=True)
        straight = ctrl_c_in_dwell(k, name, step_mode=False)
        assert straight.events == stepped.events, f'Ctrl-C in step {k}'


def test_q_late_in_a_sequence_goes_home_without_replaying_it():
    # From the descending wave_probe, one move from home, a literal
    # retrace would climb back through all three swings: twelve moves.
    wave = plan('wave')
    assert [p for p, _, _ in unwind(wave[:13])] == ['home']
    assert [p for p, _, _ in unwind(wave[:10])] == [
        'wave_open', 'wave_raise_2', 'wave_raise_1', 'wave_probe', 'home']
    # And the demo does not open the grippers again on its way home.
    assert [p for p, _, _ in unwind(plan('demo')[:4])] == ['home']


@pytest.mark.parametrize('name', sorted(SEQUENCES))
def test_every_way_home_crosses_covered_ground_once_and_ends_at_home(name):
    # The rule under every way out, from every point of the walk, settled
    # or in flight: each move is one already made (either direction) and
    # waited for as such, no pose is commanded twice, and it ends home.
    steps = plan(name)
    for n in range(1, len(steps) + 1):
        trail = [dict(HOME)] + [s.state for s in steps[:n]]
        for moving in (False, True):
            path = unwind(steps[:n], moving)
            here, ground = trail[-1], trail
            if moving:   # undone first, then home off the ground before it
                here, ground = trail[-2], trail[:-1]
                if trail[-2] != trail[-1]:
                    assert path[0][1:] == (here, settle_time_s(
                        trail[-2], trail[-1]) + SETTLE_MARGIN_S), n
                    path = path[1:]
            seen = [here]
            for pose, state, wait in path:
                where = f'{steps[n - 1].pose} (moving={moving}) -> {pose}'
                assert (here, state) in moves(ground), where
                assert wait == settle_time_s(state, here) + SETTLE_MARGIN_S, where
                assert state not in seen, where
                seen.append(state)
                here = state
            assert here == HOME


def test_a_second_ctrl_c_during_the_unwind_is_let_through():
    # Cut in the move to wave_raise_2, cut again while undoing it: the
    # arm is left at wave_raise_1, a pose it has already held, and the
    # interrupt reaches the CLI, which tells the human where it stopped.
    rec = Recorder(answers=['', '', ''], ctrl_c_in_sleep={3, 4})
    with pytest.raises(KeyboardInterrupt):
        play('wave', rec.publish, rec.sleep, ask=rec.ask, log=lambda _: None)
    assert [p for p, _ in rec.published] == [
        'home', 'wave_probe', 'wave_raise_1', 'wave_raise_2', 'wave_raise_1']
    assert rec.published[-1][1] == plan('wave')[2].state


def test_an_error_unwinds_home_first_and_is_raised_afterwards():
    rec = Recorder(answers=['', RuntimeError('terminal gone')])
    with pytest.raises(RuntimeError, match='terminal gone'):
        play('wave', rec.publish, rec.sleep, ask=rec.ask, log=lambda _: None)
    assert rec.published[-1] == ('home', HOME)


@pytest.mark.parametrize('name', sorted(SEQUENCES))
def test_reversing_a_move_in_flight_stays_on_the_walked_path(name):
    """Ctrl-C mid-move on new ground makes the unwind's first command the
    reverse of a ramp still in flight (so does an interrupt before a
    command is known to have gone out, on any step), which the driver
    replans from the joint's current position and velocity
    (trajectory.py). Cut at every tenth of every step, the reverse must
    settle inside the wait the unwind sleeps and stay within 0.1 rad of
    the span already walked: the margin every wave pose keeps to its arm
    limits (test_wave). A finger commands its own stops, where the pulse
    saturates."""
    steps = plan(name)
    trail = [dict(HOME)] + [s.state for s in steps]
    for step, before, after in zip(steps, trail, trail[1:]):
        wait = settle_time_s(before, after) + SETTLE_MARGIN_S
        ramp = settle_time_s(after, before)
        for joint in after:
            low, high = sorted((before[joint], after[joint]))
            if low == high:
                continue
            for tenth in range(1, 11):
                back = reverse_in_flight(before[joint], after[joint],
                                         SERVO_MAP[joint].max_rate,
                                         tenth * ramp / 10)
                where = f'{step.pose}: {joint} cut at {tenth}/10 of the ramp'
                assert len(back) * DT <= wait, where
                assert low - 0.1 < min(back) and max(back) < high + 0.1, where


def test_unwind_skips_repeated_states():
    steps = plan('demo')[:2]          # home, elbows_bent
    path = unwind(steps)
    assert [p for p, _, _ in path] == ['home']
    assert path[0][1] == HOME
    assert unwind([]) == []


@pytest.mark.parametrize('argv, expected', [
    (['wave', '--step', '--ros-args', '-p', 'a:=1'], ('wave', True)),
    (['--ros-args', '-r', 'n:=x', '--', 'demo'], ('demo', False)),
    (['--step', 'wave'], ('wave', True)),
    ([], (None, False)),
    (['list'], (None, False)),
    (['--help'], (None, False)),
    (['relax'], ('relax', False)),
])
def test_parse_cli(argv, expected):
    assert parse_cli(argv) == expected


def test_fresh_input_discards_typeahead_on_a_terminal(monkeypatch):
    # An ENTER pressed while the arm moved must not answer the next
    # question: the pending input is flushed before the prompt appears.
    import termios
    calls = []

    class Terminal:
        def isatty(self):
            return True

        def fileno(self):
            return 0

    monkeypatch.setattr(sys, 'stdin', Terminal())
    monkeypatch.setattr(termios, 'tcflush', lambda fd, queue: calls.append((fd, queue)))
    monkeypatch.setattr('builtins.input', lambda prompt: calls.append(prompt) or 'x')
    assert fresh_input('go? ') == 'x'
    assert calls == [(0, termios.TCIFLUSH), 'go? ']


def test_fresh_input_reads_plainly_off_a_terminal(monkeypatch):
    monkeypatch.setattr(sys, 'stdin', io.StringIO('typed\n'))
    assert fresh_input('') == 'typed'
