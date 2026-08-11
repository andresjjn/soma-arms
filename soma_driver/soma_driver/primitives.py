"""SOMA motion primitives: the named poses and sequences of v0.2.

Pure data plus validation helpers, no ROS imports: the fast test suite
proves every pose against SERVO_MAP before anything can be published.
The CLI that actually publishes lives in primitives_cli.py; from v0.4 on
the Gemini Robotics-ER 2 agent calls these same primitives, always from
BEHIND the two arming gates (the reasoner proposes, the armed driver
disposes).

Conventions encoded here:
  - Angles are radians in the hug convention (negative = inward, as if
    closing a hug; positive = outward/up). Fingers: 0.0 closed, 1.0 open.
  - `home` is the hanging rest: every joint at its clamped zero, which is
    exactly the pose the driver assumes at boot. Commanding home can
    never surprise the hardware.
  - On the hanging bench, `compact` IS `home`: arms hanging straight down
    are already the folded, resting, power-on-safe pose that safety rule
    4 demands. The alias exists so the safety vocabulary stays explicit.
  - The torso lift is deliberately absent from every pose: the L16 is
    not on this bench (it waits for the printed torso, task #9).
"""
from .servo_map import RELEASE_WHEN_SETTLED, SERVO_MAP

# Joints the primitives are allowed to command on the current bench.
OFF_BENCH = frozenset(RELEASE_WHEN_SETTLED)  # today: the torso lift
COMMANDED_JOINTS = tuple(
    name for name in SERVO_MAP if name not in OFF_BENCH)

_FINGERS = tuple(n for n in COMMANDED_JOINTS if 'finger' in n)
_ELBOWS = tuple(n for n in COMMANDED_JOINTS if 'elbow' in n)

HOME = {name: SERVO_MAP[name].clamp(0.0) for name in COMMANDED_JOINTS}

POSES: dict[str, dict[str, float]] = {
    'home': dict(HOME),
    # On the hanging bench the resting fold IS the hang. Same numbers on
    # purpose; see the module docstring.
    'compact': dict(HOME),
    'grippers_open': {name: 1.0 for name in _FINGERS},
    'grippers_closed': {name: 0.0 for name in _FINGERS},
    # Both elbows bend hug-inward (forward) by a gentle, visible amount.
    'elbows_bent': {name: -0.6 for name in _ELBOWS},
}

# Sequences: (pose name, dwell seconds after commanding it). Dwells leave
# room for the minimum-jerk profile to arrive and visibly settle.
SEQUENCES: dict[str, tuple[tuple[str, float], ...]] = {
    # The v0.2 demo: wake up, bend, talk with the hands, rest.
    'demo': (
        ('home', 1.5),
        ('elbows_bent', 2.5),
        ('grippers_open', 1.5),
        ('grippers_closed', 1.5),
        ('grippers_open', 1.5),
        ('home', 2.5),
    ),
}


def pose_targets(name: str) -> dict[str, float]:
    """The joint targets of a named pose. KeyError on unknown names."""
    return dict(POSES[name])


def sequence_steps(name: str) -> tuple[tuple[str, float], ...]:
    """The (pose, dwell) steps of a named sequence."""
    return SEQUENCES[name]


def settle_time_s(targets: dict[str, float],
                  current: dict[str, float] | None = None) -> float:
    """Worst-case minimum-jerk travel time to reach `targets`.

    From `current` if given, else from the farthest soft limit: an upper
    bound the CLI can sleep on without tracking state. The 1.875 factor
    is the minimum-jerk peak ratio: the profile whose PEAK touches the
    rate limit takes 1.875 * |distance| / rate.
    """
    worst = 0.0
    for name, target in targets.items():
        spec = SERVO_MAP[name]
        if current is not None:
            dist = abs(target - current.get(name, 0.0))
        else:
            dist = max(abs(target - spec.lower), abs(target - spec.upper))
        worst = max(worst, 1.875 * dist / spec.max_rate)
    return worst
