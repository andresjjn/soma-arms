"""SOMA motion primitives: the named poses and sequences of v0.2.

Pure data plus validation helpers, no ROS imports: the fast test suite
proves every pose against SERVO_MAP before anything can be published.
The CLI that actually publishes lives in primitives_cli.py; from v0.5 on
the Gemini Robotics ER 2 supervisor calls these same primitives, always
from BEHIND the two arming gates (the reasoner proposes, the armed driver
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

The wave (designed 2026-10-03, not yet run on the metal):
  - Right arm only; the left stays at home. The arm is raised with the
    BASE (yaw), never the shoulder: yaw negative = forward is the one
    direction verified on the metal (2026-10-01). If the same rule holds
    for the shoulder, its forward travel is only 25 deg, so it cannot
    raise the arm to the front anyway.
  - The elbow curls the forearm up and does the waving; the gripper opens
    on every out-swing. The elbow direction is NOT verified yet: if it is
    reversed, nothing collides (the forearm sweeps backward beside the
    column instead of up), but the first elbow step is a real probe. Play
    it with `soma_primitives wave --step` the first time.
  - No loaded link is held vertical: at yaw -0.87 the shoulder sits 50 deg
    off the gravitational zero where the backlash limit cycle of
    2026-10-01 lives, and the raised forearm stays about 25 deg off
    vertical. Holds are short.
"""
from .servo_map import RELEASE_WHEN_SETTLED, SERVO_MAP

# Joints the primitives are allowed to command on the current bench.
OFF_BENCH = frozenset(RELEASE_WHEN_SETTLED)  # today: the torso lift
COMMANDED_JOINTS = tuple(
    name for name in SERVO_MAP if name not in OFF_BENCH)

_FINGERS = tuple(n for n in COMMANDED_JOINTS if 'finger' in n)
_ELBOWS = tuple(n for n in COMMANDED_JOINTS if 'elbow' in n)

HOME = {name: SERVO_MAP[name].clamp(0.0) for name in COMMANDED_JOINTS}


def _right(**joints: float) -> dict[str, float]:
    """Targets for the right arm only; the left keeps its last target."""
    return {f'right_arm_{j}_joint': float(v) for j, v in joints.items()}


def _wave_pose(yaw: float, elbow: float, finger: float = 0.0) -> dict[str, float]:
    """A wave pose names all six right-arm joints, nothing is implied."""
    return _right(yaw=yaw, shoulder=0.0, elbow=elbow, wrist_pitch=0.0,
                  wrist_roll=0.0, finger_l=finger)


# The wave numbers (radians, hug convention), chosen against the measured
# right-arm limits of 2026-10-01 with at least 0.3 rad to spare.
_WAVE_PROBE_YAW = -0.25     # first move of the base: it MUST swing forward
_WAVE_YAW = -0.87           # 50 deg forward, the arm raised with the base
_WAVE_ELBOW_HALF = -0.90    # halfway curl: no step may move a joint > 1 rad
_WAVE_ELBOW_UP = -1.83      # forearm up, about 25 deg off vertical
_WAVE_ELBOW_OPEN = -1.57    # out-swing, hand opens
_WAVE_ELBOW_CLOSE = -2.09   # in-swing, hand closes

POSES: dict[str, dict[str, float]] = {
    'home': dict(HOME),
    # On the hanging bench the resting fold IS the hang. Same numbers on
    # purpose; see the module docstring.
    'compact': dict(HOME),
    'grippers_open': {name: 1.0 for name in _FINGERS},
    'grippers_closed': {name: 0.0 for name in _FINGERS},
    # Both elbows bend hug-inward (forward) by a gentle, visible amount.
    'elbows_bent': {name: -0.6 for name in _ELBOWS},
    # The right-arm wave, step by step. See the module docstring.
    'wave_probe': _wave_pose(yaw=_WAVE_PROBE_YAW, elbow=0.0),
    'wave_raise_1': _wave_pose(yaw=_WAVE_YAW, elbow=_WAVE_ELBOW_HALF),
    'wave_raise_2': _wave_pose(yaw=_WAVE_YAW, elbow=_WAVE_ELBOW_UP),
    'wave_open': _wave_pose(yaw=_WAVE_YAW, elbow=_WAVE_ELBOW_OPEN, finger=1.0),
    'wave_close': _wave_pose(yaw=_WAVE_YAW, elbow=_WAVE_ELBOW_CLOSE, finger=0.0),
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
    # The v0.2 greeting: right arm up with the base, three waves of the
    # forearm with the hand opening on each out-swing, back down the same
    # way. Dwells cover the minimum-jerk travel with at least 0.25 s to
    # spare; the probe dwells are long so a human can see the direction.
    'wave': (
        ('home', 1.5),
        ('wave_probe', 2.0),
        ('wave_raise_1', 1.5),
        ('wave_raise_2', 1.5),
        ('wave_open', 1.0),
        ('wave_close', 1.0),
        ('wave_open', 1.0),
        ('wave_close', 1.0),
        ('wave_open', 1.0),
        ('wave_close', 1.0),
        ('wave_raise_2', 1.0),
        ('wave_raise_1', 1.5),
        ('wave_probe', 1.5),
        ('home', 2.0),
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
