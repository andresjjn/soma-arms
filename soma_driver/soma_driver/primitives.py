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
    2026-10-01 lives, and the forearm swings between about horizontal and
    30 deg above it: the body sets that ceiling (elbow_reach, 2026-10-26).
    Holds are short.
  - Every number is a real angle since 2026-10-26: the joint probe's
    scales are in SERVO_MAP.
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


# The wave numbers (radians, hug convention). Designed 2026-10-03 on
# nominal radians; redrawn 2026-10-26 in real radians, after the joint
# probe found the right elbow at 0.81 of nominal: it reaches -1.94 rad, so
# the old in-swing (-2.09) no longer fitted. Same base angle (no extra
# load on the base), same swing of about 28 deg. That first redraw
# (in-swing -1.84, up -1.60) folded the forearm into the body at -1.60;
# `elbow_reach` then found -1.30 the last clean stair in this posture
# (ELBOW_REACH_CLEAN), so the in-swing sits 0.1 rad short of it.
ELBOW_REACH_CLEAN = -1.30   # measured 2026-10-26, base at _WAVE_YAW
_WAVE_PROBE_YAW = -0.25     # first move of the base: it MUST swing forward
_WAVE_YAW = -0.87           # 50 deg forward, the arm raised with the base
_WAVE_ELBOW_HALF = -0.50    # halfway curl
_WAVE_ELBOW_UP = -0.95      # forearm about 14 deg above horizontal
_WAVE_ELBOW_OPEN = -0.70    # out-swing, hand opens
_WAVE_ELBOW_CLOSE = -1.20   # in-swing, hand closes (forearm about 30 deg up)

def _straight(side: str, yaw: float) -> dict[str, float]:
    """One arm swung forward by the base alone, every other joint of that
    arm at zero: a straight arm, nothing implied."""
    return {f'{side}_arm_{j}_joint': float(v) for j, v in (
        ('yaw', yaw), ('shoulder', 0.0), ('elbow', 0.0), ('wrist_pitch', 0.0),
        ('wrist_roll', 0.0), ('finger_l', 0.0))}


# The camera coverage check (2026-10-13, the OAK on its mast): one arm at a
# time, straight, swung forward by the base only, so the camera can be
# checked against poses whose geometry is not in doubt. The base is the
# joint whose direction the metal proved and whose large angles the hand
# show ran; the elbow stays out (it jammed against the central box at a
# large angle the model did not predict). 40 deg is the ceiling: about
# 7.3 kg.cm on the base by the model, the designed wave's load, because in
# step mode a pose is held for as long as the operator reads the screen.
_CAM_LIFTS = ((15, -0.26), (30, -0.52), (40, -0.70))

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
POSES.update({f'cam_{side}_{deg}': _straight(side, yaw)
              for side in ('right', 'left') for deg, yaw in _CAM_LIFTS})


# Where does the right elbow touch? (2026-10-26) The redrawn wave's
# wave_raise_2 (elbow -1.60 real, 92 deg of curl, base at -0.87) folded
# the forearm into the robot's body: the map's elbow end is the servo's,
# not the metal's. Same posture as the wave, the elbow curled in stairs
# of 0.1 rad that stop short of the -1.60 that touched; in step mode the
# operator measures the gap before every ENTER and quits when it is
# small. The last clean stair, minus a margin, is the wave's new floor.
# Result, 2026-10-26: -1.30 (ELBOW_REACH_CLEAN).
_REACH_STAIRS = (-1.0, -1.1, -1.2, -1.3, -1.4, -1.5)
POSES.update({f'elbow_reach_{round(-100 * e)}': _wave_pose(yaw=_WAVE_YAW, elbow=e)
              for e in _REACH_STAIRS})


# The hello (2026-10-26, Andres's idea): the right arm rises with the
# elbow folding on the way, opens the elbow at the top and waves the
# forearm with it five times while the gripper opens and closes, and
# comes down. Designed against the bench, not against the eye:
#  - The base at the top of its reach (-2.08, 0.1 inside the real -2.18).
#    Gravity still loads one side of the gear play at the top, so the
#    limit cycle of the hanging zero has nowhere to live.
#  - Third form. First: base -2.0, elbow -0.5, rolling the wrist (clean).
#    Second: the palm waving on the wrist pitch, elbow -0.35 (clean).
#    Third: the elbow nearly straight, waving +0.15 .. -0.45 (clean,
#    looked better, the forearm still leaning forward). Fourth, Andres's
#    call: the forearm pointing at the ceiling while it waves. The elbow
#    swings -0.25 .. -0.75, the forearm about 150 .. 178 deg: it stops
#    just short of vertical, because past it the gravity torque on the
#    elbow changes sign mid-swing, the same zero crossing that drives the
#    gear-play limit cycle. Fifth, Andres's call: the SHOULDER opened
#    outward (it sat folded at -0.28) so the wave stands farther from the
#    body, upper arm about 110 deg (shoulder +0.16), the forearm still
#    about 150 .. 176 deg. The more the shoulder opens, the more the
#    elbow must fold to keep the forearm up, and the fold has a measured
#    end (clean at -1.30, touched at -1.60, likely the forearm on its own
#    arm: the forearm was moving away from the box). A horizontal upper
#    arm would need -1.54; 110 deg needs -1.15, 0.15 short of clean.
#  - The fold carries the load: by the URDF masses a straight arm at 90
#    deg asks the base for 11.4 kg.cm, above the 10 a MG996R gives. The
#    rise curls the elbow to -1.0 by the time the base is at -1.0, the
#    worst point of the path, about 9.4 kg.cm, under the 9.7 the wave asks
#    and has held. (A point-mass
#    model fitted to that 11.4; relative, not a measurement.)
#  - The fold stays at least 0.15 rad short of ELBOW_REACH_CLEAN; no step
#    moves a joint more than 1 rad.
#  - Flowing: every dwell is the travel time plus 0.1 s, so the motion
#    never stops to wait, and each step is one minimum-jerk move of
#    several joints at once.
_HELLO_TOP = dict(yaw=-2.08, shoulder=0.16, wrist_pitch=0.0, wrist_roll=0.0)
_HELLO_ELBOW = -0.925       # the forearm about 163 deg
_HELLO_SWING = 0.225        # +-13 deg: out -0.70 (about 150), in -1.15 (about 176)
POSES.update({
    'hello_rise': _right(yaw=-1.0, shoulder=0.0, elbow=-1.0, wrist_pitch=0.0,
                         wrist_roll=0.0, finger_l=0.0),
    'hello_lift': _right(yaw=-1.6, shoulder=-0.15, elbow=-0.9, wrist_pitch=-0.2,
                         wrist_roll=0.0, finger_l=0.0),
    'hello_top': _right(**_HELLO_TOP, elbow=_HELLO_ELBOW, finger_l=0.0),
    'hello_out': _right(**_HELLO_TOP, elbow=_HELLO_ELBOW + _HELLO_SWING, finger_l=1.0),
    'hello_in': _right(**_HELLO_TOP, elbow=_HELLO_ELBOW - _HELLO_SWING, finger_l=0.0),
})


def _flowing(poses: tuple[str, ...], hold_s: float = 0.1) -> tuple[tuple[str, float], ...]:
    """Each pose with a dwell of exactly its travel time plus `hold_s`."""
    state, out = dict(HOME), []
    for pose in poses:
        targets = POSES[pose]
        out.append((pose, round(settle_time_s(targets, state) + hold_s, 2)))
        state.update(targets)
    return tuple(out)


def _camera_sweep(side: str) -> tuple[tuple[str, float], ...]:
    """Up 15, 30, 40 deg and back down the same way; 3 s at each pose up
    (the coverage capture needs 2 s of still frames)."""
    up = tuple((f'cam_{side}_{deg}', 3.0) for deg, _ in _CAM_LIFTS)
    down = tuple((f'cam_{side}_{deg}', 2.0) for deg, _ in reversed(_CAM_LIFTS[:-1]))
    return up + down + (('home', 2.5),)

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
    # Camera coverage: the right arm, home, then the left arm. Pair it with
    # scripts/camera_coverage.py --watch, which captures each pose.
    'camera_check': (('home', 1.5),) + _camera_sweep('right') + _camera_sweep('left'),
    # Diagnostic, step mode only in practice: see _REACH_STAIRS.
    'elbow_reach': (('home', 1.5), ('wave_probe', 2.0), ('wave_raise_1', 2.0))
    + tuple((f'elbow_reach_{round(-100 * e)}', 2.0) for e in _REACH_STAIRS)
    + tuple((f'elbow_reach_{round(-100 * e)}', 1.5) for e in reversed(_REACH_STAIRS[:-1]))
    + (('wave_raise_1', 1.5), ('wave_probe', 1.5), ('home', 2.0)),
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


# The hello flows: its dwells come from the travel times (_flowing), so it
# is defined once settle_time_s exists.
SEQUENCES['hello'] = _flowing(('home', 'hello_rise', 'hello_lift', 'hello_top')
                              + ('hello_out', 'hello_in') * 5
                              + ('hello_top', 'hello_lift', 'hello_rise', 'home'))

# How high does the right shoulder take the arm from the hello's top?
# (2026-10-26, Andres: the workbench seemed to raise the arm further.)
# The hello's top, then the shoulder up in stairs of 0.1 rad to 0.1
# inside its map end (the 710 us captured on 2026-10-01); the operator
# reads the upper arm above the horizontal at each stair and quits when
# it stops rising or strains. Never past the map end: that is a stop.
_SHOULDER_STAIRS = (0.06, -0.04, -0.14, -0.24, -0.28)
POSES.update({f'shoulder_reach_{i}': _right(**dict(_HELLO_TOP, shoulder=sh),
                                            elbow=_HELLO_ELBOW, finger_l=0.0)
              for i, sh in enumerate(_SHOULDER_STAIRS, 1)})
SEQUENCES['shoulder_reach'] = (
    (('home', 1.5), ('hello_rise', 2.0), ('hello_lift', 2.0), ('hello_top', 2.0))
    + tuple((f'shoulder_reach_{i}', 2.0) for i in range(1, len(_SHOULDER_STAIRS) + 1))
    + tuple((f'shoulder_reach_{i}', 1.5) for i in range(len(_SHOULDER_STAIRS) - 1, 0, -1))
    + (('hello_top', 1.5), ('hello_lift', 1.5), ('hello_rise', 1.5), ('home', 2.0)))
