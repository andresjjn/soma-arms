"""The sequence player behind `soma_primitives`, pure and testable without ROS.

The CLI hands this module two callables, publish and sleep, and in step
mode a third one that asks a human before every step. Nothing here can
arm the driver: it only produces joint targets in the order the catalog
defines. When the human aborts, it walks the arm back home along the path
it has just proved, one step at a time, never with a single jump: from the
top of the wave, "go home" would be a 2 rad elbow move through territory
whose sign may be wrong, which is exactly what step mode exists to probe.

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


def unwind(done: list[Step]) -> list[tuple[str, dict[str, float], float]]:
    """Retrace `done` back to HOME: (pose, full state, wait) newest first.

    Every reverse move is a forward move already made, so it is at most
    1 rad per joint and takes at most settle + margin. States that repeat
    (home after home) are published once.
    """
    trail = [('home', dict(HOME))] + [(s.pose, s.state) for s in done]
    previous = trail[-1][1]
    path = []
    for pose, state in reversed(trail[:-1]):
        if state == previous:
            continue
        path.append((pose, state, settle_time_s(state, previous) + SETTLE_MARGIN_S))
        previous = state
    return path


def play(name: str,
         publish: Callable[[str, dict[str, float]], None],
         sleep: Callable[[float], None],
         ask: Callable[[str], str] | None = None,
         log: Callable[[str], None] = print) -> bool:
    """Play a sequence; True when it ran to the end, False when unwound.

    With `ask`, a human is prompted after every step but the last: an
    empty answer continues, `q` (or end of input) aborts and unwinds.
    """
    done: list[Step] = []
    for step in plan(name):
        publish(step.pose, step.targets)
        log(f'{name}: step {step.index + 1}/{step.count} {step.pose}, '
            f'dwell {step.dwell:.1f}s')
        sleep(step.dwell)
        done.append(step)
        if ask is None or step.index + 1 == step.count:
            continue
        try:
            answer = ask(PROMPT)
        except EOFError:
            answer = ABORT_ANSWERS[0]
        if answer.strip().lower() in ABORT_ANSWERS:
            log(f'{name}: aborted after {step.pose}, unwinding to home')
            for pose, state, wait in unwind(done):
                publish(pose, state)
                log(f'{name}: unwind to {pose}, settling {wait:.1f}s')
                sleep(wait)
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
