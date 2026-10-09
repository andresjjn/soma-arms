"""Step mode is a promise about what gets published, so it is tested blind.

Pure tests, no ROS: the player receives fake publish, sleep and ask
callables and the assertions read what it did with them.
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


class Recorder:
    def __init__(self, answers=()):
        self.published: list[tuple[str, dict]] = []
        self.slept: list[float] = []
        self.asked = 0
        self.answers = list(answers)

    def publish(self, pose, targets):
        self.published.append((pose, dict(targets)))

    def sleep(self, seconds):
        self.slept.append(seconds)

    def ask(self, prompt):
        assert prompt == PROMPT
        self.asked += 1
        answer = self.answers.pop(0)
        if isinstance(answer, BaseException):
            raise answer
        return answer


def never_ask(prompt):
    raise AssertionError('play must not prompt without step mode')


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
