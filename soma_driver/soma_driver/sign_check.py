"""The v0.2 sign check, pure and testable without ROS.

The question, joint by joint: does a NEGATIVE command move the METAL the
hug way? The calibration of 2026-10-01 labelled one end of every joint
MIN (the negative, hug side) and the other MAX, and SERVO_MAP maps the
commanded sign onto those labels. Only the yaw was later confirmed on the
metal (negative = forward, away from the column). This module walks the
joints of one arm, one at a time: a small step toward the hug side, a
hold, back to 0.0, and a verdict typed by the human who watched it.

Verdicts, and the fix each one means:
  ok        the metal moved as expected.
  reversed  the metal moved the OTHER way. A DRIVER fix: mirror the
            joint's SERVO_MAP row (mirror() below) and sync the limits
            in the xacro. Not an axis flip: every planar <axis> line in
            soma_arm.xacro uses ${side}, so a flip there would change
            both arms and leave the driver commanding the wrong way.
  model     the metal was right and RViz drew it the other way. A model
            fix only (the joint's <axis> line); the driver is untouched.
            It needs no hardware: RViz can be held against these same
            expectations with the mock driver, at a desk.
  unclear   could not tell; the joint repeats.

The CLI (sign_check_cli.py) hands run() publish, sleep and ask callables,
the same split as player.py. Nothing here imports ROS and nothing here
can arm the driver: with the driver disarmed the whole walk is a mock
rehearsal.

Every excursion starts from the hang and leaves the vertical, so it can
only lift the fingertips (a pendulum leaving the vertical rises), and a
lane sits outboard of the column: whichever way the metal turns, a
0.25 rad step from home touches nothing.
"""
import argparse
from dataclasses import asdict, dataclass, replace
from typing import Callable

from .player import SETTLE_MARGIN_S, strip_ros_segment
from .primitives import POSES, SEQUENCES, settle_time_s
from .servo_map import SERVO_MAP, ServoSpec

ARMS = ('right', 'left')
ARM_CHOICES = ARMS + ('both',)
# Confidence order. The yaw goes first because its direction is already
# verified on the metal: if it moves as expected, the topic, the driver,
# the board and the channel are proven before any unverified joint moves.
# Then from the least loaded joint to the most loaded, the shoulder last:
# the biggest load, and the limit cycle at the hanging zero (2026-10-01).
ORDER = ('yaw', 'finger_l', 'wrist_roll', 'wrist_pitch', 'elbow', 'shoulder')
VERIFIED = {'yaw': '2026-10-01'}

DELTA = 0.25       # rad, about 14 deg: plainly visible, far from any stop
MARGIN = 0.1       # rad, never closer than this to a measured stop
MAX_DELTA = 0.5    # rad. A sign check needs a visible move, not a big one,
                   # and the cap refuses a degrees-for-radians typo (15).
DWELL_S = 2.0      # s held at the target once the ramp has arrived
MAX_DWELL_S = 10.0
ZERO_TOL = 1e-3    # rad, how close to 0.0 counts as "at home"

VERDICTS = ('ok', 'reversed', 'model', 'unclear')
JUDGED = ('ok', 'reversed', 'model')
MOVE_PROMPT = '    ENTER = move, s = skip, q = quit: '
VERDICT_PROMPT = ('    verdict: ENTER = ok, r = reversed, m = model, '
                  'u = unclear (repeat): ')
_MOVE_ANSWERS = {'': 'move', 's': 'skip', 'q': 'quit'}
_VERDICT_ANSWERS = {'': 'ok', 'r': 'reversed', 'm': 'model', 'u': 'unclear'}

# What the METAL should do for a (negative, positive) command, in bench
# terms: forward = away from the column. Negative is the hug side.
_EXPECT = {
    'yaw': ('the whole arm swings FORWARD, away from the column',
            'the whole arm swings BACKWARD, toward the column'),
    'shoulder': ('the upper arm rises to the FRONT',
                 'the upper arm rises to the BACK, toward the column'),
    'elbow': ('the forearm folds FORWARD (the hug bend)',
              'the forearm folds BACKWARD, toward the column'),
    'wrist_pitch': ('the gripper pitches FORWARD',
                    'the gripper pitches BACKWARD, toward the column'),
    # The roll has no forward or backward: its hug side is the end that
    # was captured as MIN. CW or CCW seen from above is what the desk
    # check of the model needs, so it gets written down.
    'wrist_roll': ('the gripper rolls INWARD, toward the end captured as '
                   'MIN (note CW or CCW seen from above)',
                   'the gripper rolls OUTWARD, toward the end captured as '
                   'MAX (note CW or CCW seen from above)'),
    'finger_l': ('the gripper CLOSES', 'the gripper OPENS'),
}

# Each joint's <axis> line in soma_arm.xacro, and the same line flipped.
_PLANAR_AXIS = ('<axis xyz="0 0 ${side}"/>', '<axis xyz="0 0 ${-side}"/>')
_AXIS = {
    'yaw': _PLANAR_AXIS,
    'shoulder': _PLANAR_AXIS,
    'elbow': _PLANAR_AXIS,
    'wrist_pitch': _PLANAR_AXIS,
    'wrist_roll': ('<axis xyz="${side} 0 0"/>', '<axis xyz="${-side} 0 0"/>'),
    'finger_l': ('<axis xyz="0 1 0"/>', '<axis xyz="0 -1 0"/>'),
}


@dataclass(frozen=True)
class Finding:
    joint: str
    target: float      # the excursion that was commanded, rad
    verdict: str       # one of VERDICTS


@dataclass(frozen=True)
class Options:
    arm: str = 'right'
    delta: float = DELTA
    dwell: float = DWELL_S
    report: str | None = None


def arm_of(name: str) -> str:
    """'right_arm_elbow_joint' -> 'right'."""
    for arm in ARMS:
        if name.startswith(f'{arm}_arm_') and name.endswith('_joint'):
            return arm
    raise ValueError(f'{name} is not an arm joint')


def joint_kind(name: str) -> str:
    """'right_arm_elbow_joint' -> 'elbow'."""
    prefix = f'{arm_of(name)}_arm_'
    return name[len(prefix):-len('_joint')]


def joints_for(arm: str) -> tuple[str, ...]:
    """The joints to check, in confidence order; 'both' = right, then left."""
    if arm == 'both':
        return joints_for('right') + joints_for('left')
    if arm not in ARMS:
        raise ValueError(f'arm must be one of {ARM_CHOICES}, got {arm!r}')
    return tuple(f'{arm}_arm_{kind}_joint' for kind in ORDER)


def check_delta(delta: float) -> float:
    if not 0.0 < delta <= MAX_DELTA:
        raise ValueError(
            f'delta must be above 0 and at most {MAX_DELTA} rad, got {delta}')
    return delta


def check_dwell(dwell: float) -> float:
    if not 0.0 < dwell <= MAX_DWELL_S:
        raise ValueError(
            f'dwell must be above 0 and at most {MAX_DWELL_S} s, got {dwell}')
    return dwell


def excursion(name: str, delta: float = DELTA, margin: float = MARGIN,
              servo_map: dict[str, ServoSpec] = SERVO_MAP) -> float:
    """The test target: the hug (negative) side whenever it has room.

    Negative when `lower <= -(delta + margin)`, else positive when
    `upper >= delta + margin`, else the larger travel minus the margin.
    Never within `margin` of a stop: the right yaw, whose zero sits
    0.2749 rad from its backward stop, goes to -0.25 (the old tool sent
    it to +0.25, 1.4 deg from that stop); the fingers, whose zero is the
    closed stop, open to +0.25.
    """
    check_delta(delta)
    spec = servo_map[name]
    if spec.lower <= -(delta + margin):
        return -delta
    if spec.upper >= delta + margin:
        return delta
    room_neg, room_pos = -spec.lower, spec.upper
    room = max(room_neg, room_pos) - margin
    if room <= 0.0:
        raise ValueError(
            f'{name}: no side has more than {margin} rad of travel, '
            'so there is no step that stays clear of both stops')
    return -room if room_neg >= room_pos else room


def plan(arm: str, delta: float = DELTA,
         margin: float = MARGIN) -> list[tuple[str, float]]:
    """(joint, target) for every joint of the walk, computed before any
    motion, so a joint without a safe excursion fails the whole run up
    front instead of halfway through it."""
    return [(name, excursion(name, delta, margin)) for name in joints_for(arm)]


def expectation(name: str, target: float) -> str:
    """What the metal should do for this command, in bench terms."""
    kind = joint_kind(name)
    negative, positive = _EXPECT[kind]
    text = negative if target < 0.0 else positive
    if kind == 'finger_l':
        spec = SERVO_MAP[name]
        text += f' {abs(target) / (spec.upper - spec.lower):.0%} of its travel'
    if kind in VERIFIED:
        text += f' (verified on the metal {VERIFIED[kind]})'
    return text


def expectation_table(arm: str, delta: float = DELTA,
                      margin: float = MARGIN) -> str:
    """The printable table of the runbook, generated so it cannot drift."""
    rows = ['| # | joint | command (rad) | the metal should | metal | RViz | notes |',
            '|---|---|---|---|---|---|---|']
    for i, (name, target) in enumerate(plan(arm, delta, margin), 1):
        rows.append(f'| {i} | `{name}` | {target:+.2f} | '
                    f'{expectation(name, target)} | '
                    '[ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |')
    return '\n'.join(rows)


def mirror(spec: ServoSpec) -> ServoSpec:
    """The same physical motion under the opposite command sign.

    Swaps the pulse ends and negates the band, so
    mirror(spec).command_to_us(q) == spec.command_to_us(-q) for every q:
    the zero pulse stays where it is, and mirroring twice is the
    identity. `0.0 - x` instead of `-x` keeps a 0.0 limit from turning
    into -0.0 in a printed row.
    """
    return replace(spec, min_us=spec.max_us, max_us=spec.min_us,
                   lower=0.0 - spec.upper, upper=0.0 - spec.lower)


def _num(value: float) -> str:
    """A limit the way servo_map.py and the xacro write it: 4 decimals,
    except the round finger limits (0.0, 1.0)."""
    return repr(value) if value == int(value) else f'{value:.4f}'


def servo_map_line(name: str, spec: ServoSpec) -> str:
    """The SERVO_MAP row as servo_map.py writes it, aligned like its block."""
    prefix = name.split('_arm_')[0] + '_arm_' if '_arm_' in name else name
    block = [n for n in SERVO_MAP if n.startswith(prefix)] or [name]
    key = f"'{name}':".ljust(max(len(n) for n in block) + 4)
    args = [str(spec.channel), repr(spec.min_us), repr(spec.max_us),
            _num(spec.lower), _num(spec.upper), repr(spec.max_rate)]
    if spec.address != 0x40:
        args.append(f'address=0x{spec.address:02x}')
    return f"    {key}ServoSpec({', '.join(args)}),"


def xacro_limits(name: str, spec: ServoSpec) -> tuple[str, str]:
    """Where this joint's limits live in the model, and what to write."""
    kind = joint_kind(name)
    lower, upper = _num(spec.lower), _num(spec.upper)
    if arm_of(name) == 'right':
        return ('soma_description/urdf/soma_arm.xacro, the macro defaults '
                '(they are the right arm)',
                f'{kind}_lower:={lower} {kind}_upper:={upper}')
    return ('soma_description/urdf/soma_bench.urdf.xacro, the left arm call',
            f'{kind}_lower="{lower}" {kind}_upper="{upper}"')


def poses_outside(name: str, spec: ServoSpec,
                  margin: float = MARGIN) -> list[str]:
    """Catalog poses that command `name` away from zero, out of the band
    of `spec` or within `margin` of its ends. Zero is exempt: it is the
    calibrated pulse, and mirror() keeps it."""
    out = []
    for pose, targets in POSES.items():
        value = targets.get(name)
        if value is None or value == 0.0:
            continue
        if not spec.lower + margin <= value <= spec.upper - margin:
            out.append(pose)
    return sorted(out)


def _reversed_fix(finding: Finding) -> list[str]:
    name = finding.joint
    if joint_kind(name) == 'finger_l':
        return [
            f'REVERSED {name}: {finding.target:+.2f} should have opened the '
            'gripper and it did not.',
            '  Do NOT mirror a finger: 0.0 closed and 1.0 open is the '
            'convention every pose uses.',
            '  Re-capture closed and open with the workbench, then check '
            'this joint again.',
        ]
    spec = SERVO_MAP[name]
    fixed = mirror(spec)
    where, limits = xacro_limits(name, fixed)
    lines = [
        f'REVERSED {name}: the metal moved opposite to the hug convention.',
        '  This is a driver fix, NOT an axis flip. The new row for '
        'soma_driver/soma_driver/servo_map.py:',
        servo_map_line(name, fixed),
        f'  and the limits in {where}:',
        f'      {limits}',
        f'  The zero pulse stays at {spec.command_to_us(0.0):.0f} us. The '
        'rows pinned in soma_driver/test/test_servo_map.py (EXACT_SERVO_MAP, '
        'INVERTED_CHANNELS) change with it; the test says why (this sign '
        'check, on the metal).',
    ]
    broken = poses_outside(name, fixed)
    if broken:
        sequences = sorted(s for s, steps in SEQUENCES.items()
                           if any(pose in broken for pose, _ in steps))
        lines.append(
            f'  Poses outside the new band [{_num(fixed.lower)}, '
            f'{_num(fixed.upper)}] or within {MARGIN} rad of it: '
            f'{", ".join(broken)}.')
        lines.append(
            f'  Redesign them before anything plays them (sequences: '
            f'{", ".join(sequences) or "none"}).')
    return lines


def _model_fix(finding: Finding) -> list[str]:
    name = finding.joint
    kind = joint_kind(name)
    line, flipped = _AXIS[kind]
    lines = [
        f'MODEL {name}: the metal was right and RViz drew it the other way.',
        '  A model fix only; the SERVO_MAP row stays. The joint line in '
        'soma_description/urdf/soma_arm.xacro:',
        f'      {line}  ->  {flipped}',
        '  That line serves BOTH arms (one macro). If only this arm '
        'disagrees, add a per-joint sign parameter to the macro and pass '
        'it for this side; never flip the shared line for one arm.',
    ]
    if kind == 'finger_l':
        lines.append('  finger_r mimics finger_l with the same axis line: '
                     'flip both, or the pair stops mirroring.')
    return lines


def summary(findings: list[Finding], planned: tuple[str, ...] = ()) -> str:
    """What the walk found, and the exact change each finding asks for."""
    lines = ['=== sign check summary']
    judged = {f.joint for f in findings if f.verdict in JUDGED}
    if planned:
        lines.append(f'{len(judged & set(planned))} of {len(planned)} '
                     'joints judged')
    ok = [f.joint for f in findings if f.verdict == 'ok']
    if ok:
        lines.append('ok: ' + ', '.join(ok))
    problems = False
    for finding in findings:
        if finding.verdict == 'reversed':
            lines += _reversed_fix(finding)
            problems = True
        elif finding.verdict == 'model':
            lines += _model_fix(finding)
            problems = True
        elif finding.verdict == 'unclear':
            lines.append(f'UNCLEAR {finding.joint}: check it again before '
                         'trusting it.')
    missing = [name for name in planned if name not in judged]
    if missing:
        lines.append('NOT judged (skipped, unclear or not reached): '
                     + ', '.join(missing))
    if problems:
        lines.append('Make these changes with the driver disarmed and the '
                     'ATX off, and play nothing that uses the old values '
                     'until the tests are green again.')
    elif judged:
        lines.append('no reversals; the hug convention survived contact '
                     'with power.')
    else:
        lines.append('no joint was judged.')
    return '\n'.join(lines)


def report(options: Options, findings: list[Finding],
           planned: tuple[str, ...]) -> dict:
    """The JSON record of a walk, for calibration/ and the log."""
    return {
        'arm': options.arm,
        'delta': options.delta,
        'dwell': options.dwell,
        'planned': list(planned),
        'findings': [asdict(f) for f in findings],
    }


def not_at_zero(state: dict[str, float], joints: tuple[str, ...],
                tol: float = ZERO_TOL) -> list[str]:
    """Joints that are not at 0.0, or not reported at all.

    The walk must start from home: a joint resting at -0.5 that is sent
    to -0.25 moves BACKWARD and would read as a reversed sign.
    """
    return [name for name in joints
            if name not in state or abs(state[name]) > tol]


def _answer(ask: Callable[[str], str], prompt: str,
            choices: dict[str, str], log: Callable[[str], None]) -> str:
    """Ask until the answer is one of `choices`; never guess what a typo
    meant. Only an empty answer can mean "move"."""
    keys = ', '.join(repr(k) if k else 'ENTER' for k in choices)
    while True:
        answer = ask(prompt).strip().lower()
        if answer in choices:
            return choices[answer]
        log(f'    {answer!r} is not an answer here: {keys}')


def run(arm: str,
        publish: Callable[[str, float], None],
        sleep: Callable[[float], None],
        ask: Callable[[str], str],
        log: Callable[[str], None] = print,
        delta: float = DELTA,
        dwell: float = DWELL_S) -> list[Finding]:
    """Walk the joints of `arm`; one Finding per joint that got a verdict.

    Per joint: the command and the expectation are shown, ENTER moves it
    (s skips, q quits), it is held `dwell` seconds after the ramp
    arrives, commanded back to 0.0 and given time to settle, and then
    the human answers. `unclear` repeats the joint; if it is left
    unclear it is recorded as such. Whatever ends the walk early (q,
    Ctrl-C, end of input, any error) commands the current joint to 0.0
    first, so the last thing published for it is always 0.0.
    """
    steps = plan(arm, delta)
    check_dwell(dwell)
    log(f'Sign check, {arm}: {len(steps)} joints, one at a time, each '
        f'{delta:g} rad toward the hug side, held {dwell:g} s, back to 0.0.')
    log('Judge the METAL: ok, reversed (a driver fix) or unclear. '
        'Mark model only when RViz is open beside it and draws the other '
        'way. This tool never arms the driver.')
    for i, (name, target) in enumerate(steps, 1):
        log(f'  {i:2d}. {name:28s} {target:+.2f}  {expectation(name, target)}')

    findings: list[Finding] = []
    for i, (name, target) in enumerate(steps, 1):
        out = settle_time_s({name: target}, {name: 0.0}) + dwell
        back = settle_time_s({name: 0.0}, {name: target}) + SETTLE_MARGIN_S
        log(f'\n=== {i}/{len(steps)} {name}')
        log(f'    command {target:+.2f} rad, hold {dwell:g} s, back to 0.0')
        log(f'    expect: {expectation(name, target)}')
        unclear = False
        try:
            while True:
                action = _answer(ask, MOVE_PROMPT, _MOVE_ANSWERS, log)
                if action == 'quit':
                    raise _Quit
                if action == 'skip':
                    break
                publish(name, target)
                sleep(out)
                publish(name, 0.0)
                sleep(back)
                verdict = _answer(ask, VERDICT_PROMPT, _VERDICT_ANSWERS, log)
                if verdict != 'unclear':
                    findings.append(Finding(name, target, verdict))
                    unclear = False
                    break
                unclear = True
                log('    unclear: the same joint again')
        except BaseException as exc:
            # First thing, whatever happened: the joint goes home, and the
            # sleep keeps the process alive while the command goes out.
            publish(name, 0.0)
            sleep(back)
            if unclear:
                findings.append(Finding(name, target, 'unclear'))
            if not isinstance(exc, (_Quit, KeyboardInterrupt, EOFError)):
                raise
            log(f'    stopped: {name} commanded back to 0.0')
            return findings
        if unclear:
            findings.append(Finding(name, target, 'unclear'))
    return findings


class _Quit(Exception):
    """The human typed q."""


def _arm_choice(text: str) -> str:
    if text not in ARM_CHOICES:
        raise argparse.ArgumentTypeError(
            f'choose from {", ".join(ARM_CHOICES)}')
    return text


def _checked(check: Callable[[float], float]) -> Callable[[str], float]:
    def parse(text: str) -> float:
        try:
            return check(float(text))
        except ValueError as exc:
            raise argparse.ArgumentTypeError(str(exc)) from None
    return parse


def parse_cli(argv: list[str]) -> Options:
    """Options from the command line, rclpy's --ros-args segment dropped."""
    parser = argparse.ArgumentParser(
        prog='soma_sign_check',
        description='Check, joint by joint, that a negative command moves '
                    'the metal the hug way. Never arms the driver.')
    parser.add_argument('--arm', type=_arm_choice, default='right',
                        help='right (default), left, or both: right, then left')
    parser.add_argument('--delta', type=_checked(check_delta), default=DELTA,
                        help=f'excursion in rad (default {DELTA}, at most {MAX_DELTA})')
    parser.add_argument('--dwell', type=_checked(check_dwell), default=DWELL_S,
                        help=f'seconds held at the target (default {DWELL_S:g})')
    parser.add_argument('--report', default=None,
                        help='write the findings as JSON to this path')
    args = parser.parse_args(strip_ros_segment(argv))
    return Options(args.arm, args.delta, args.dwell, args.report)
