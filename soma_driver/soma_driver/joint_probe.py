"""The joint probe: how far a joint really turns per radian commanded. Pure.

The radians in SERVO_MAP come from a nominal 2000 us = 180 deg
(scripts/apply_calibration.py), never measured. The sign check of
2026-10-12 proved directions at 0.25 rad, not magnitudes, and on
2026-10-12 the left forearm jammed against the central box where the
model saw 45 mm of room: the first evidence that the two differ.

One planar joint at a time (base, shoulder, elbow, wrist pitch), every
other joint at 0.0. With the arm hanging and only that joint moved, the
inclination of the link it drives, measured from the vertical with a
phone level, IS the joint's real angle. The probe walks up a staircase
toward the hug side (forward), asks for the reading at every step,
including 0.0 first (the zero error), and walks back down the same
steps. A straight line through (commanded, measured) gives the real
scale and zero; 1.0 and 0 would mean the nominal map is right.

Every way out of a walk takes the staircase home: q, x (it came close
to something, or touched it), the end of input, Ctrl-C and any error,
which is raised again once the joint is back at 0.0. The CLI that
publishes lives in joint_probe_cli.py; this module never arms anything.
"""
import math
from dataclasses import dataclass, field
from typing import Callable

from .primitives import settle_time_s
from .servo_map import SERVO_MAP

PLANAR = ('yaw', 'shoulder', 'elbow', 'wrist_pitch')
STEP = 0.2                 # rad per stair
MARGIN = 0.1               # rad kept from every captured stop
SETTLE_MARGIN_S = 0.8      # extra wait after the ramp before asking
# The most each joint may be walked, rad forward. The base at 0.70 holds
# about 7.3 kg.cm by the model (the designed wave's load); the elbow and
# the wrist stop at 1.0 until their scale is known (the elbow jammed
# against the box on 2026-10-12); the shoulder's forward travel is short.
CEILING = {'yaw': 0.70, 'shoulder': 0.4, 'elbow': 1.0, 'wrist_pitch': 1.0}

# What to measure, per joint: the link that the joint alone tilts.
LINK = {
    'yaw': 'any link of the arm (the whole arm swings)',
    'shoulder': 'the upper arm (between shoulder and elbow)',
    'elbow': 'the forearm (between elbow and wrist)',
    'wrist_pitch': 'the gripper body',
}
PROMPT = ('    degrees forward from vertical on the phone (ENTER = same as asked '
          'for, s = skip, x = close to something: stop, q = stop): ')


@dataclass
class Walk:
    arm: str
    joint: str
    targets: list[float]                    # commanded, rad, hug side
    points: list[tuple[float, float]] = field(default_factory=list)
    stopped: str = ''                       # why it ended early, if it did

    @property
    def name(self) -> str:
        return f'{self.arm}_arm_{self.joint}_joint'


class _Stop(Exception):
    def __init__(self, why: str) -> None:
        super().__init__(why)
        self.why = why


def plan(arm: str, joint: str, step: float = STEP, most: float | None = None) -> list[float]:
    """0.0, then negative stairs of `step` up to the joint's safe most."""
    if arm not in ('right', 'left'):
        raise ValueError(f'arm must be right or left, not {arm!r}')
    if joint not in PLANAR:
        raise ValueError(f'only planar joints can be read with a level: {PLANAR}')
    if not 0.05 <= step <= 0.3:
        raise ValueError('step must be in [0.05, 0.3] rad')
    spec = SERVO_MAP[f'{arm}_arm_{joint}_joint']
    reach = min(CEILING[joint], -(spec.lower + MARGIN))
    if most is not None:
        if most <= 0:
            raise ValueError('most must be positive (rad forward)')
        reach = min(reach, most)
    targets, k = [0.0], 1
    while k * step <= reach + 1e-9:
        targets.append(-round(k * step, 4))
        k += 1
    return targets


def fit(points: list[tuple[float, float]]) -> dict | None:
    """Least squares: measured deg = scale * commanded deg forward + offset."""
    if len(points) < 3:
        return None
    xs = [-math.degrees(c) for c, _ in points]
    ys = [m for _, m in points]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-9:
        return None
    scale = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    offset = my - scale * mx
    residual = max(abs(y - (scale * x + offset)) for x, y in zip(xs, ys))
    return {'scale': round(scale, 4), 'offset_deg': round(offset, 2),
            'max_residual_deg': round(residual, 2), 'points': n,
            'us_per_rad_real': round(2000.0 / math.pi / scale, 1) if scale else None}


def parse_reading(text: str, expected_deg: float):
    """A reading in degrees, or 'skip' / 'stop' / 'close', or None if unclear."""
    t = text.strip().lower()
    if t == '':
        return expected_deg
    if t in ('s', 'skip'):
        return 'skip'
    if t in ('q', 'quit'):
        return 'stop'
    if t in ('x', 'close'):
        return 'close'
    try:
        value = float(t.replace(',', '.'))
    except ValueError:
        return None
    return value if math.isfinite(value) and abs(value) <= 180 else None


def run(walk: Walk,
        publish: Callable[[str, float], None],
        sleep: Callable[[float], None],
        ask: Callable[[str], str],
        log: Callable[[str], None] = print) -> Walk:
    """Walk the staircase up, read every step, walk back down to 0.0."""
    name = walk.name
    log(f'Joint probe, {name}: {len(walk.targets) - 1} stairs of up to '
        f'{abs(walk.targets[1]) if len(walk.targets) > 1 else 0:g} rad forward, '
        f'to {walk.targets[-1]:+.2f} rad.')
    log(f'Put the phone level on {LINK[walk.joint]}; read degrees FORWARD from '
        'vertical (negative if backward). This tool never arms the driver.')
    at = 0.0
    try:
        for target in walk.targets:
            if target != at:
                publish(name, target)
                sleep(settle_time_s({name: target}, {name: at}) + SETTLE_MARGIN_S)
                at = target
            expected = -math.degrees(target)
            log(f'\n  commanded {target:+.2f} rad: nominal {expected:+.1f} deg forward')
            while True:
                answer = parse_reading(ask(PROMPT), expected)
                if answer is None:
                    log('    a number in degrees, or s, x, q')
                    continue
                break
            if answer in ('stop', 'close'):
                raise _Stop('quit' if answer == 'stop' else
                            f'close to something at {target:+.2f} rad')
            if answer != 'skip':
                walk.points.append((target, float(answer)))
    except BaseException as exc:
        if isinstance(exc, _Stop):
            walk.stopped = exc.why
        elif isinstance(exc, (KeyboardInterrupt, EOFError)):
            walk.stopped = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'end of input'
        else:
            walk.stopped = type(exc).__name__
        _home(walk, at, publish, sleep, log)
        if not isinstance(exc, (_Stop, KeyboardInterrupt, EOFError)):
            raise
        return walk
    _home(walk, at, publish, sleep, log)
    return walk


def _home(walk: Walk, at: float, publish, sleep, log) -> None:
    """Down the same stairs, one at a time, to 0.0."""
    name = walk.name
    below = [t for t in reversed(walk.targets) if abs(t) < abs(at)]
    for target in below:
        publish(name, target)
        sleep(settle_time_s({name: target}, {name: at}) + 0.3)
        at = target
    if at != 0.0:
        publish(name, 0.0)
        sleep(settle_time_s({name: 0.0}, {name: at}) + 0.3)
    log(f'  {name} back at 0.0')


def summary(walk: Walk) -> str:
    lines = [f'{walk.name}: {len(walk.points)} readings'
             + (f', stopped: {walk.stopped}' if walk.stopped else '')]
    for cmd, deg in walk.points:
        lines.append(f'  {cmd:+.2f} rad  nominal {-math.degrees(cmd):+6.1f}  measured {deg:+6.1f} deg')
    f = fit(walk.points)
    if f is None:
        lines.append('  not enough readings for a fit (3 or more, not all at one angle)')
    else:
        lines.append(f"  scale {f['scale']} (1.0 = nominal), zero error {f['offset_deg']:+.1f} deg, "
                     f"worst residual {f['max_residual_deg']:.1f} deg, about "
                     f"{f['us_per_rad_real']} us/rad (nominal 636.6)")
    return '\n'.join(lines)


def report(walk: Walk) -> dict:
    return {'arm': walk.arm, 'joint': walk.joint, 'targets': walk.targets,
            'points': [{'commanded_rad': c, 'measured_deg': m} for c, m in walk.points],
            'stopped': walk.stopped, 'fit': fit(walk.points)}
