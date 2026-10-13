"""The sequence player behind `soma_primitives`, pure and testable without ROS.

The CLI hands this module two callables, publish and sleep, and in step
mode a third one that asks a human before every step. Nothing here can
arm the driver: it only produces joint targets in the order the catalog
defines. When the human aborts, it walks the arm back home along the path
it has just proved, one step at a time, never with a single jump: from the
top of the wave, "go home" would be a 2 rad elbow move through territory
whose sign may be wrong, which is exactly what step mode exists to probe.

Every way out of a walk takes that road home: `q` at a prompt, the end
of input, Ctrl-C at a prompt or in the middle of a move (with or without
step mode), and any error, which is raised again once the arm is home.
The road crosses only ground the walk already covered and skips every
detour that came back to a pose already held: from the way down, home
is down, not back through the whole sequence. A move cut short is
undone first, unless it was itself retracing covered ground (the way
down), in which case it is let finish. A second Ctrl-C, during the
unwind itself, is let through: the arm keeps the last pose commanded,
held by the driver.

The two command-line helpers at the bottom (strip_ros_segment and
fresh_input) are shared with the sign check, which asks a human the same
way.
"""
import sys
from dataclasses import dataclass
from typing import Callable

from .primitives import HOME, pose_targets, sequence_steps, settle_time_s

SETTLE_MARGIN_S = 0.5
ABORT_ANSWERS = ('q', 'quit')
PROMPT = 'ENTER = next step, q = unwind to home: '
HELP_WORDS = ('-h', '--help', 'list')


@dataclass(frozen=True)
class Step:
    index: int                   # position in the sequence, from 0
    count: int                   # steps in the sequence
    pose: str
    targets: dict[str, float]    # what this step publishes
    state: dict[str, float]      # every commanded joint after this step
    dwell: float


def plan(name: str) -> list[Step]:
    """The steps of a sequence with the commanded state after each one.

    Joints a pose omits keep their previous target, exactly as on the
    driver and in the workbench's plan_sequence.
    """
    steps = sequence_steps(name)
    current = dict(HOME)
    out = []
    for i, (pose, dwell) in enumerate(steps):
        targets = pose_targets(pose)
        current.update(targets)
        out.append(Step(i, len(steps), pose, targets, dict(current), dwell))
    return out


def unwind(done: list[Step],
           moving: bool = False) -> list[tuple[str, dict[str, float], float]]:
    """The way home from the end of `done`: (pose, full state, wait), in order.

    Only ground the walk already covered: every move undoes a forward
    move already made, so it is at most 1 rad per joint and takes at most
    settle + margin. A detour that came back to a pose already held is
    skipped, and a repeated pose (home after home) is never a move. With
    `moving`, the newest step is still in flight and is undone first:
    the arm is not at its pose yet.
    """
    trail = [('home', dict(HOME))] + [(s.pose, s.state) for s in done]
    states = [state for _, state in trail]
    i, path = len(trail) - 1, []
    if moving and i > 0:
        i -= 1
        if states[i] != states[i + 1]:
            path.append((trail[i][0], states[i], settle_time_s(
                states[i], states[i + 1]) + SETTLE_MARGIN_S))
    # From the FIRST time the arm held the pose it is in, one move back:
    # whatever happened after that first time was a loop.
    while (first := states.index(states[i])) > 0:
        pose, state = trail[first - 1]
        path.append((pose, state,
                     settle_time_s(state, states[first]) + SETTLE_MARGIN_S))
        i = first - 1
    return path


def _finish_s(done: list[Step]) -> float | None:
    """The wait that lets the newest step finish when it retraces ground
    the walk already covered (the same two poses were consecutive before,
    in either order), as the way down of the wave does; None when it is
    new ground, which the unwind undoes instead."""
    if not done:
        return None
    states = [dict(HOME)] + [s.state for s in done]
    move = (states[-2], states[-1])
    covered = list(zip(states, states[1:-1]))
    if move in covered or move[::-1] in covered:
        return settle_time_s(*move) + SETTLE_MARGIN_S
    return None


class _Quit(Exception):
    """The human typed q at a prompt."""


# The ways a walk ends early on purpose, as the log names them. Any other
# exception is an error: it unwinds the same way and is raised afterwards.
_STOPS = ((_Quit, 'aborted'), (EOFError, 'end of input'),
          (KeyboardInterrupt, 'interrupted'))


def play(name: str,
         publish: Callable[[str, dict[str, float]], None],
         sleep: Callable[[float], None],
         ask: Callable[[str], str] | None = None,
         log: Callable[[str], None] = print) -> bool:
    """Play a sequence; True when it ran to the end, False when unwound.

    With `ask`, a human is prompted after every step but the last: an
    empty answer continues, `q` aborts. Whatever ends the walk early (q,
    the end of input, Ctrl-C at a prompt or during a dwell, any error)
    unwinds home first; an error is raised again once the unwind is
    done. A move cut short is undone, or let finish when it retraces
    covered ground. A Ctrl-C during the unwind is not caught.
    """
    steps = plan(name)
    done: list[Step] = []
    moving = sent = False
    try:
        for step in steps:
            # The flags are set before the step joins `done`, and `moving`
            # clears only once its dwell is over: whatever the instant of
            # an interrupt, the arm never reads as settled too early.
            moving, sent = True, False
            done.append(step)
            publish(step.pose, step.targets)
            sent = True
            log(f'{name}: step {step.index + 1}/{step.count} {step.pose}, '
                f'dwell {step.dwell:.1f}s')
            sleep(step.dwell)
            moving = False
            if ask is None or step.index + 1 == step.count:
                continue
            if ask(PROMPT).strip().lower() in ABORT_ANSWERS:
                raise _Quit
    except BaseException as exc:
        # First thing, whatever happened: the walk back home. The sleeps
        # keep the process alive while each command goes out.
        why = next((text for kind, text in _STOPS if isinstance(exc, kind)),
                   None)
        at = done[-1].pose if done else 'the start'
        log(f'{name}: {why or type(exc).__name__} at {at}, unwinding to home')
        # A move whose command went out and that retraces covered ground
        # (the way down) runs on undisturbed instead of turning around.
        # Any other move in flight is undone: if its command never left,
        # undoing it asks for the pose the arm is already in.
        finish = _finish_s(done) if moving and sent else None
        if finish is not None:
            log(f'{name}: letting {at} finish, it retraces a move already '
                f'made, {finish:.1f}s')
            sleep(finish)
            moving = False
        for pose, state, wait in unwind(done, moving):
            publish(pose, state)
            log(f'{name}: unwind to {pose}, settling {wait:.1f}s')
            sleep(wait)
        if why is None:
            raise
        return False
    return True


def fresh_input(prompt: str) -> str:
    """input(), after throwing away whatever was typed during the motion.

    Keys pressed while a joint moves wait in the terminal buffer and
    answer the NEXT prompt: a nervous double ENTER would accept one step
    and start the next before anyone read it. Flushing pending input
    first means every answer was typed after its question was on screen.
    """
    if sys.stdin.isatty():
        import termios   # POSIX only, and only needed on a real terminal
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    return input(prompt)


def strip_ros_segment(argv: list[str]) -> list[str]:
    """Drop every `--ros-args ... [--]` segment, which belongs to rclpy."""
    own, i = [], 0
    while i < len(argv):
        if argv[i] == '--ros-args':
            i += 1
            while i < len(argv) and argv[i] != '--':
                i += 1
            i += 1    # the `--` terminator, when present
            continue
        own.append(argv[i])
        i += 1
    return own


def parse_cli(argv: list[str]) -> tuple[str | None, bool]:
    """(target, step) from the command line; target None means 'print help'."""
    own = strip_ros_segment(argv)
    step = '--step' in own
    rest = [a for a in own if a != '--step']
    if not rest or rest[0] in HELP_WORDS:
        return None, step
    return rest[0], step
