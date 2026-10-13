#!/usr/bin/env python3
"""One gripper follows one hand: open the hand and the gripper opens with it.

A bench experiment decided by Andres on 2026-10-12, not part of the
library. The OAK-D Lite runs the MediaPipe palm detector and hand
landmarker on the camera itself (RVC2, about 8 fps), and how open the
hand is sets how open ONE gripper is, in proportion. It is two processes
joined by a pipe, because the camera lives on the Jetson host (the
depthai 3 venv) and ROS lives in the driver's container:

    # measure only, no robot: prints how open the hand reads, live, and
    # with --show draws it in a browser at http://localhost:8090 on the
    # Jetson (from the Mac: ssh -N -L 8090:localhost:8090 jetson)
    ~/venvs/depthai3/bin/python scripts/hand_gripper.py [--show]

    # drive the right gripper (arming stays a separate, explicit step)
    ~/venvs/depthai3/bin/python scripts/hand_gripper.py --emit \\
      | docker exec -i soma_driver bash -c '. /opt/ros/humble/setup.bash \\
          && . /ros2_ws/install/setup.bash \\
          && python3 /ros2_ws/src/soma-arms/scripts/hand_gripper.py --bridge right'

How open the hand is: the distance from the thumb tip to the mean of the
four fingertips, divided by the palm's size (its length, wrist to the
middle knuckle, or its width scaled to a length if that is longer, so a
hand tilted toward the lens does not inflate it). The ratio does not
change with the hand's distance to the camera, and it reads a pinch, a
claw and a fist alike as closed. Best seen with the palm to the camera
and the fingers up, as if waving at the robot. It is
mapped linearly from --closed (gripper 0.0) to --open (gripper 1.0) and
smoothed, and a new target goes out only when it moved by a deadband.

What keeps this safe:
  - The bridge publishes one joint only, finger_l of the arm it is given,
    clamped to 0..1. The driver clamps again to the soft limits and ramps
    every target (minimum jerk under the joint's max rate).
  - Nothing here can arm the driver: /soma/arm stays a manual call. It
    can only DISARM: once the hand has been gone for --release-after
    seconds, the bridge walks the arm home if it was raised, lets it
    settle, and then cuts every servo (an arm released raised would
    fall).
  - No hand in view, no command: the gripper holds where it is.
  - Ending either side (q, Ctrl-C, the pipe closing) sends nothing more;
    the gripper holds its last target until the driver is disarmed.
  - No force sensing: closing the hand on an object stalls the servo
    against it. Hold briefly, and open a little at a sustained hum.

Only the pure helpers sit at module level: the container has no depthai
and the host has no rclpy, so each side imports its own library inside
its own function, and the tests import neither.
"""
import argparse
import math
import sys
import time

# MediaPipe hand landmark indices.
WRIST, THUMB_TIP, INDEX_MCP, MIDDLE_MCP, PINKY_MCP = 0, 4, 5, 9, 17
FINGERTIPS = (8, 12, 16, 20)
# Palm length (wrist to middle knuckle) over palm width (index knuckle to
# little finger knuckle) on a flat adult hand, about 9 cm over 7 cm.
WIDTH_TO_LENGTH = 1.3

# Calibrated on Andres's hand, 2026-10-12, 270 s at 8 fps: a pinch read
# 0.4 to 0.5, an open hand 1.5 to 1.7; readings above 2.5 were not a hand.
CLOSED_RATIO = 0.5      # thumb on the fingertips: gripper 0.0
OPEN_RATIO = 1.5        # hand open, thumb out: gripper 1.0
MEDIAN_OF = 3           # readings in the median that drops one-frame spikes
SMOOTHING = 0.3         # weight of the newest reading in the running average
DEADBAND = 0.05         # smallest change worth a new target
MAX_RATIO = 2.5         # above this no real hand: a misread, skipped
# The two ends hold. At or below CLOSE_BELOW the gripper goes to 0.0 and
# stays shut until the hand opens past CLOSE_BELOW + HYSTERESIS; the open
# end mirrors it. Without this, a closed hand's noise (0.0 to 0.25) sent
# 89 small targets in 107 s on the metal, 2026-10-12: the gripper fluttered.
CLOSE_BELOW = 0.2
OPEN_ABOVE = 0.85
HYSTERESIS = 0.1
ARMS = ('right', 'left')


def palm_size(kpts) -> float:
    """The palm's length, or its width scaled to a length if that is longer.

    The camera sees the hand flat, in 2D: a hand tilted toward the lens
    shows a short palm and every ratio over it inflates (readings of 2 to
    5 for an open hand on 2026-10-12). Tilting about one axis shortens
    the length or the width, rarely both, so the longer of the two is the
    better estimate of the hand's real size.
    """
    length = math.dist(kpts[WRIST], kpts[MIDDLE_MCP])
    width = math.dist(kpts[INDEX_MCP], kpts[PINKY_MCP]) * WIDTH_TO_LENGTH
    return max(length, width)


def hand_ratio(kpts) -> float | None:
    """Thumb tip to the fingertips' mean, in palm sizes; None if degenerate.

    `kpts` is the 21 MediaPipe landmarks as (x, y) in an isotropic frame
    (pixels, or normalized coordinates of a square image).
    """
    if len(kpts) < 21:
        return None
    palm = palm_size(kpts)
    if palm < 1e-6:
        return None
    tips = [kpts[i] for i in FINGERTIPS]
    mean = (sum(p[0] for p in tips) / len(tips), sum(p[1] for p in tips) / len(tips))
    return math.dist(kpts[THUMB_TIP], mean) / palm


def openness(ratio: float, closed: float = CLOSED_RATIO, open_: float = OPEN_RATIO) -> float:
    """The ratio mapped linearly onto 0.0 (closed) .. 1.0 (open), saturated."""
    return min(max((ratio - closed) / (open_ - closed), 0.0), 1.0)


class Follower:
    """From one reading per frame to the targets worth sending.

    update() takes the hand's ratio, or None when no hand is in view, and
    returns a gripper target or None. A lost hand never moves the gripper,
    and neither does a ratio above max_ratio: an open hand reads about
    1.7 at most, so 3 is the landmarker guessing, not a hand.

    Each usable reading goes through three layers: the median of the last
    `median_of` readings (a one-frame spike never reaches the gripper),
    a running average (`smoothing` is the newest reading's weight), and
    the two held ends (see CLOSE_BELOW). Between the ends the target is
    the smoothed level, sent only when it moved by the deadband; reaching
    an end always goes out, so a closed hand really reaches 0.0.
    """

    def __init__(self, closed: float = CLOSED_RATIO, open_: float = OPEN_RATIO,
                 smoothing: float = SMOOTHING, deadband: float = DEADBAND,
                 max_ratio: float = MAX_RATIO, median_of: int = MEDIAN_OF,
                 close_below: float = CLOSE_BELOW, open_above: float = OPEN_ABOVE,
                 hysteresis: float = HYSTERESIS) -> None:
        if not open_ > closed:
            raise ValueError(f'--open ({open_}) must be above --closed ({closed})')
        if not 0.0 <= close_below < close_below + hysteresis < open_above - hysteresis:
            raise ValueError('the held ends overlap: need close_below + hysteresis '
                             'below open_above - hysteresis')
        self.closed, self.open_ = closed, open_
        self.smoothing, self.deadband = smoothing, deadband
        self.max_ratio = max_ratio
        self.median_of = median_of
        self.close_below, self.open_above = close_below, open_above
        self.hysteresis = hysteresis
        self.recent: list[float] = []      # last raw readings, for the median
        self.level: float | None = None    # smoothed openness
        self.held: float | None = None     # 0.0 or 1.0 while an end holds
        self.sent: float | None = None     # last target that went out

    def update(self, ratio: float | None) -> float | None:
        if ratio is None or ratio > self.max_ratio:
            return None
        self.recent = (self.recent + [openness(ratio, self.closed, self.open_)]
                       )[-self.median_of:]
        ordered = sorted(self.recent)
        mid = len(ordered) // 2
        median = (ordered[mid] if len(ordered) % 2
                  else (ordered[mid - 1] + ordered[mid]) / 2)
        self.level = median if self.level is None else (
            self.smoothing * median + (1.0 - self.smoothing) * self.level)
        target = self._hold(self.level)
        if self.sent is not None:
            moved = abs(target - self.sent)
            at_end = target in (0.0, 1.0) and target != self.sent
            if moved < self.deadband and not at_end:
                return None
        self.sent = target
        return target

    def _hold(self, level: float) -> float:
        """The target for this level, with the ends held (hysteresis)."""
        if self.held == 0.0 and level < self.close_below + self.hysteresis:
            return 0.0
        if self.held == 1.0 and level > self.open_above - self.hysteresis:
            return 1.0
        self.held = None
        if level <= self.close_below:
            self.held = 0.0
        elif level >= self.open_above:
            self.held = 1.0
        return self.held if self.held is not None else round(level, 2)


def _fmt(value: float | None) -> str:
    return '-' if value is None else f'{value:.2f}'


def _bar(level: float | None, width: int = 20) -> str:
    filled = 0 if level is None else round(level * width)
    return '[' + '#' * filled + '.' * (width - filled) + ']'


def status_text(ratio: float | None, level: float | None,
                max_ratio: float = MAX_RATIO) -> str:
    """One status line: what the hand read and where the gripper would go.

    `level` is None until a first usable reading, which a misread is not.
    """
    if ratio is None:
        return 'no hand'
    if ratio > max_ratio:
        return f'ratio {ratio:4.2f}  misread, skipped  gripper {_fmt(level)}'
    return f'ratio {ratio:4.2f}  gripper {_fmt(level)}  {_bar(level)}'


def parse_target(line: str) -> float | None:
    """One line from the pipe to a gripper target in 0..1, or None to skip it."""
    try:
        value = float(line.strip())
    except ValueError:
        return None
    if not math.isfinite(value):
        return None
    return min(max(value, 0.0), 1.0)


def joint_for(arm: str) -> str:
    """The one joint the bridge may command: that arm's driven finger."""
    if arm not in ARMS:
        raise ValueError(f'arm must be one of {ARMS}, not {arm!r}')
    return f'{arm}_arm_finger_l_joint'


# The present gesture (Andres's video, 2026-10-12): the hand opens and
# closes, then the whole arm swings to the front with the base, straight,
# gripper shut, and the wrist rolls a little each way until the operator
# ends it. Base and roll only: the elbow is never commanded.
#
# Why straight: the first version curled the elbow to -1.65 before lifting.
# On the metal the left forearm folded in and jammed against the central
# box, from the start of the curl, where the model has 45 mm to spare: at
# large angles the elbow does not move as the model believes (the sign
# check only proved its direction, at 0.25 rad). Until the elbow is
# measured, the show leaves it alone. The base's direction is verified,
# and by the URDF's estimated masses a straight arm 30 deg forward asks it
# for about 5.7 kg.cm (6.0 with a 20 g object), 45 mm from the body.
LIFT = 0.52                # base, rad forward (30 deg); --lift
LIFT_MAX = 0.79            # 45 deg: about 8.0 kg.cm, 80% of the measured torque
HEAVY_LIFT = 0.7           # above this (40 deg, about 7.3 kg.cm) the hold is short
HEAVY_MAX_S = 15.0
YAW_RAMP_S = 2.5
ROLL_HOME_S = 0.8
SHOW_PAUSE_S = 0.4         # still, before the wrist starts to roll
ROLL_AMPLITUDE = 0.25      # rad each way, about 14 deg
ROLL_PERIOD_S = 3.0
PRESENT_MAX_S = 30.0       # then home on its own; HEAVY_MAX_S above HEAVY_LIFT
TRIGGER_OPEN_S = 0.5       # the hand must stay open this long before a close counts
STREAM_HZ = 10
RELEASE = 'release'        # the tracker's word for "the hand is gone"
RELEASE_AFTER_S = 2.0      # hand gone this long: walk home, then disarm
RELEASE_SETTLE_S = 1.0     # after the last home target, before the signal is cut


class LostHand:
    """True once, when a hand seen before has been gone for `after_s`.

    Never before a first hand: the operator starts the program and only
    then brings the hand in. A hand back in view resets the clock.
    """

    def __init__(self, after_s: float = RELEASE_AFTER_S) -> None:
        self.after_s = after_s
        self.seen_once = False
        self.gone_since: float | None = None
        self.fired = False

    def update(self, now: float, seen: bool) -> bool:
        if seen:
            self.seen_once, self.gone_since = True, None
            return False
        if not self.seen_once or self.fired:
            return False
        if self.gone_since is None:
            self.gone_since = now
        if now - self.gone_since >= self.after_s:
            self.fired = True
            return True
        return False


def away_from_home(state: dict[str, float], arm: str, tolerance: float = 0.02):
    """The arm's joints (fingers aside) not at 0.0 on /joint_states, by name.

    The Presenter plans its raise from 0.0 and leaves the other joints
    where they are: started anywhere else, its first command would be a
    jump, or the arm would lift bent.
    """
    names = [f'{arm}_arm_{j}_joint' for j in
             ('yaw', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll')]
    return [n for n in names if n not in state or abs(state[n]) > tolerance]


def min_jerk(a: float, b: float, tau: float) -> float:
    """From a to b at fraction tau of the time, on a minimum-jerk profile."""
    tau = min(max(tau, 0.0), 1.0)
    return a + (b - a) * tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau ** 2)


class Presenter:
    """The bridge's brain: follow the hand, and with `present`, show off.

    Phases: follow (finger targets pass through), raise (the base swings
    the straight arm forward), show (the wrist rolls), home (wrist, then
    base), done. hand() takes a finger target from the tracker, tick()
    the clock, finish() the end of input; each returns the commands to
    publish now, as {joint: position}. Only this arm's base, wrist roll
    and driven finger are ever named; from the trigger on, the finger is
    held at 0.0 and the hand is ignored.
    """

    def __init__(self, arm: str, present: bool = False,
                 present_max_s: float = PRESENT_MAX_S,
                 roll: float = ROLL_AMPLITUDE, period_s: float = ROLL_PERIOD_S,
                 lift: float = LIFT) -> None:
        joint_for(arm)   # validates the arm
        if not 0.0 < lift <= LIFT_MAX:
            raise ValueError(f'lift must be in (0, {LIFT_MAX}] rad')
        self.joint = {k: f'{arm}_arm_{k}_joint'
                      for k in ('yaw', 'wrist_roll', 'finger_l')}
        if lift > HEAVY_LIFT:
            present_max_s = min(present_max_s, HEAVY_MAX_S)
        self.present, self.present_max_s = present, present_max_s
        self.roll, self.period_s, self.lift = roll, period_s, lift
        self.released = False
        self.phase = 'follow'
        self.last = {'yaw': 0.0, 'wrist_roll': 0.0, 'finger_l': None}
        self._open_since: float | None = None
        self._was_open = False
        self._plan: list[tuple[float, float, str, float, float]] = []
        self._show_from = 0.0

    @property
    def done(self) -> bool:
        return self.phase == 'done'

    def hand(self, now: float, value: float) -> dict[str, float]:
        if self.phase != 'follow':
            return {}
        if value >= 1.0:
            if self._open_since is None:
                self._open_since = now
        elif self._open_since is not None:
            self._was_open = now - self._open_since >= TRIGGER_OPEN_S
            self._open_since = None
        out = self._set({'finger_l': value})
        if self.present and value <= 0.0 and self._was_open:
            self.phase = 'raise'
            self._plan = self._ramps(now, [('yaw', -self.lift, YAW_RAMP_S)])
        elif value <= 0.0:
            self._was_open = False
        return out

    def tick(self, now: float) -> dict[str, float]:
        if self.phase in ('raise', 'home'):
            out = self._sample(now)
            if now >= self._plan[-1][1]:
                if self.phase == 'raise':
                    self.phase, self._show_from = 'show', now + SHOW_PAUSE_S
                else:
                    self.phase = 'done'
            return out
        if self.phase == 'show':
            if now - self._show_from >= self.present_max_s:
                return self.finish(now)
            if now < self._show_from:
                return {}
            angle = 2.0 * math.pi * (now - self._show_from) / self.period_s
            return self._set({'wrist_roll': round(self.roll * math.sin(angle), 3)})
        return {}

    def release(self, now: float) -> dict[str, float]:
        """The hand is gone: walk home like finish(), and mark that the
        servos are to be cut once home. Only the bridge can cut them."""
        self.released = True
        return self.finish(now)

    def finish(self, now: float) -> dict[str, float]:
        """End of input (or the show's time is up): walk home, or stop."""
        if self.phase == 'follow':
            self.phase = 'done'
            return {}
        if self.phase in ('home', 'done'):
            return {}
        self.phase = 'home'
        self._plan = self._ramps(now, [('wrist_roll', 0.0, ROLL_HOME_S),
                                       ('yaw', 0.0, YAW_RAMP_S)],
                                 full={'wrist_roll': self.roll, 'yaw': self.lift})
        return self._sample(now)

    def _ramps(self, now, legs, full=None):
        """Back-to-back ramps from where each joint was last commanded.

        With `full`, a leg's time shrinks with its distance (a raise cut
        short comes down in less time); a leg with nothing to do is
        skipped. Always ends with a hold of the finger at 0.0.
        """
        plan, t = [], now
        for name, goal, duration in legs:
            start = self.last[name]
            if full is not None:
                span = abs(full[name]) or 1.0
                duration = max(0.3, duration * abs(goal - start) / span)
                if start == goal:
                    continue
            plan.append((t, t + duration, name, start, goal))
            t += duration
        if not plan:
            plan.append((now, now, 'finger_l', 0.0, 0.0))
        return plan

    def _sample(self, now: float) -> dict[str, float]:
        targets = {'finger_l': 0.0}
        for t0, t1, name, a, b in self._plan:
            if now >= t0:
                tau = 1.0 if t1 <= t0 else (now - t0) / (t1 - t0)
                targets[name] = round(min_jerk(a, b, tau), 3)
        return self._set(targets)

    def _set(self, targets: dict[str, float]) -> dict[str, float]:
        self.last.update(targets)
        return {self.joint[k]: v for k, v in targets.items()}


def bridge(arm: str, present: bool = False, present_max_s: float = PRESENT_MAX_S,
           roll: float = ROLL_AMPLITUDE, lift: float = LIFT, wait_s: float = 5.0) -> int:
    """Inside the driver's container: targets from stdin to /soma/command.

    Without `present` only the finger follows the hand. With it, the
    Presenter adds the show (see LIFT). The end of input, which is
    what Ctrl-C on the operator's side becomes in here, walks the arm home
    before the bridge exits: it never leaves a raised arm to whoever comes
    next, and it never disarms (an unpowered raised arm falls).
    """
    import os
    import select

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool

    brain = Presenter(arm, present, present_max_s, roll, lift=lift)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('soma_hand_gripper')
    pub = node.create_publisher(JointState, 'soma/command', 10)
    seen: dict[str, float] = {}
    sub = node.create_subscription(
        JointState, 'joint_states',
        lambda m: seen.update(zip(m.name, m.position)), 10)

    def say(text: str) -> None:
        try:   # the operator's side may be gone (Ctrl-C) while we walk home
            print(text, file=sys.stderr, flush=True)
        except OSError:
            pass

    def cut_signal() -> None:
        """Disarm the driver: every servo goes limp. The request is built
        here with data False and nowhere else: this script cannot arm."""
        client = node.create_client(SetBool, 'soma/arm')
        if not client.wait_for_service(timeout_sec=2.0):
            say('bridge: /soma/arm did not answer, the servos stay as they are')
            return
        request = SetBool.Request()
        request.data = False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        answer = future.result()
        say(f'bridge: released: {answer.message if answer else "no answer"}')

    def send(targets: dict[str, float]) -> None:
        if not targets:
            return
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name, msg.position = list(targets), [float(v) for v in targets.values()]
        pub.publish(msg)

    try:
        deadline = time.monotonic() + wait_s
        while ((pub.get_subscription_count() == 0 or not seen)
               and time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)
        if pub.get_subscription_count() == 0:
            say('REFUSED: nobody listens on /soma/command. Is the driver up?')
            return 2
        off = away_from_home(seen, arm) if present else []
        if off:
            say('REFUSED, nothing was commanded: not at 0.0 on /joint_states: '
                + ', '.join(off) + '. Bring the arm home first.')
            return 2
        say(f'bridge up: stdin -> {arm} arm' + (', present armed: open then close'
                                                 if present else ', gripper only'))
        fd = sys.stdin.fileno()
        state = {'pending': b'', 'open': True, 'phase': brain.phase}

        def step() -> None:
            """One pass: read what the tracker sent, then let the clock act."""
            started = time.monotonic()
            if state['open']:
                ready, _, _ = select.select([fd], [], [], 1.0 / STREAM_HZ)
                if ready:
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        state['open'] = False
                        say('bridge: end of input')
                        send(brain.finish(time.monotonic()))
                    *lines, state['pending'] = (state['pending'] + chunk).split(b'\n')
                    for line in lines:
                        if line.strip() == RELEASE.encode():
                            say('bridge: the hand is gone, walking home, then release')
                            state['open'] = False
                            send(brain.release(time.monotonic()))
                            return
                        value = parse_target(line.decode(errors='replace'))
                        if value is not None:
                            targets = brain.hand(time.monotonic(), value)
                            send(targets)
                            if targets:
                                say(f'gripper -> {value:.2f}')
            else:
                time.sleep(max(0.0, 1.0 / STREAM_HZ - (time.monotonic() - started)))
            send(brain.tick(time.monotonic()))
            if brain.phase != state['phase']:
                state['phase'] = brain.phase
                say(f'bridge: {brain.phase}')

        interrupts = 0
        while not brain.done:
            try:
                step()
            except KeyboardInterrupt:
                # The first one ends the input and walks home, like the end
                # of the pipe; a second one gives up: the arm holds.
                interrupts += 1
                if interrupts > 1:
                    say('bridge: interrupted again, the arm holds its last targets')
                    return 130
                state['open'] = False
                send(brain.finish(time.monotonic()))
        if brain.released:
            time.sleep(RELEASE_SETTLE_S)    # the driver's ramp lands on 0.0
            cut_signal()
            return 0
        say('bridge: done, the arm holds its last targets')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def home_check(wait_s: float = 5.0) -> int:
    """Inside the container: 0 when every arm joint of both arms reads 0.0.

    For scripts/hand_show.sh, before it offers to arm: the first armed tick
    sends the believed pose with no ramp, so it must be the hang. Fingers
    are reported, not required (a limp gripper keeps its last position).
    Reads /joint_states only; it commands nothing.
    """
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('soma_home_check')
    seen: dict[str, float] = {}
    node.create_subscription(JointState, 'joint_states',
                             lambda m: seen.update(zip(m.name, m.position)), 10)
    try:
        deadline = time.monotonic() + wait_s
        while not seen and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not seen:
            print('no /joint_states: is the driver up?', file=sys.stderr)
            return 2
        fingers = ', '.join(f'{n} {v:.2f}' for n, v in seen.items() if n.endswith('finger_l_joint'))
        print(f'fingers believed at: {fingers} (arming sends them there)')
        off = [n for arm in ARMS for n in away_from_home(seen, arm)]
        if off:
            print('not at 0.0: ' + ', '.join(off), file=sys.stderr)
            return 2
        print('every arm joint of both arms reads 0.0')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def track(args) -> int:
    """On the Jetson host: the hand on the camera to targets on stdout."""
    import depthai as dai
    from depthai_nodes.node import FrameCropper, GatherData, ParsingNeuralNetwork

    side, fps, pad = 768, args.fps, 0.1

    class CropConfigs(dai.node.HostNode):
        """Palm detections to one crop per hand for the landmarker, as in
        the oak-examples hand-pose example."""

        def __init__(self):
            super().__init__()
            self.configs = self.createOutput()

        def build(self, detections, size):
            self.size = size
            self.link_args(detections)
            return self

        def process(self, dets: dai.Buffer) -> None:
            group = dai.MessageGroup()
            for i, det in enumerate(dets.detections):
                box = det.getBoundingBox()
                rect = dai.RotatedRect()
                rect.center.x, rect.center.y = box.center.x, box.center.y
                rect.size.width = box.size.width + 2 * pad
                rect.size.height = box.size.height + 2 * pad
                rect.angle = 0
                cfg = dai.ImageManipConfig()
                cfg.addCropRotatedRect(rect, normalizedCoords=True)
                cfg.setOutputSize(*self.size, dai.ImageManipConfig.ResizeMode.STRETCH)
                cfg.setReusePreviousImage(False)
                cfg.setTimestamp(dets.getTimestamp())
                cfg.setSequenceNum(dets.getSequenceNum())
                group[f'cfg_{i}'] = cfg
            group.setTimestamp(dets.getTimestamp())
            group.setSequenceNum(dets.getSequenceNum())
            self.configs.send(group)

    def archive(model):
        desc = dai.NNModelDescription(model=model, platform='RVC2')
        return dai.NNArchive(dai.getModelFromZoo(desc))

    follower = Follower(args.closed, args.open, smoothing=args.smoothing,
                        close_below=args.close_below)
    out = sys.stderr if args.emit else sys.stdout
    with dai.Pipeline() as pipeline:
        palm = archive('luxonis/mediapipe-palm-detection:192x192')
        marks = archive('luxonis/mediapipe-hand-landmarker:224x224')
        cam = pipeline.create(dai.node.Camera).build()
        if not args.no_rotate:   # the camera hangs upside down (2026-10-12)
            cam.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)
        video = cam.requestOutput((side, side), dai.ImgFrame.Type.BGR888p, fps=fps)
        resize = pipeline.create(dai.node.ImageManip)
        resize.setMaxOutputFrameSize(palm.getInputWidth() * palm.getInputHeight() * 3)
        resize.initialConfig.setOutputSize(palm.getInputWidth(), palm.getInputHeight(),
                                           mode=dai.ImageManipConfig.ResizeMode.STRETCH)
        resize.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        video.link(resize.inputImage)
        det_nn = pipeline.create(ParsingNeuralNetwork).build(resize.out, palm)
        crops = pipeline.create(CropConfigs).build(
            det_nn.out, (marks.getInputWidth(), marks.getInputHeight()))
        cropper = pipeline.create(FrameCropper).fromManipConfigs(
            inputManipConfigs=crops.configs,
            maxOutputFrameSize=marks.getInputWidth() * marks.getInputHeight() * 3,
            waitForConfig=True).build(det_nn.passthrough)
        pose_nn = pipeline.create(ParsingNeuralNetwork).build(cropper.out, marks)
        gather = pipeline.create(GatherData).build(
            cameraFps=fps, inputData=pose_nn.outputs, inputReference=det_nn.out)
        q_hands = gather.out.createOutputQueue(maxSize=4, blocking=False)
        want_video = args.show or args.save
        q_video = video.createOutputQueue(maxSize=2, blocking=False) if want_video else None
        publish = _web_view(args.port) if args.show else None
        pipeline.start()

        say = _Reporter(out)
        log = open(args.log, 'w', buffering=1) if args.log else None
        say(f'tracking at {fps} fps; closed {args.closed:.2f}, open {args.open:.2f} '
            'palm sizes' + (f'; view on http://localhost:{args.port}' if args.show else '')
            + '; Ctrl-C quits')
        lock = HandLock()
        lost = LostHand(args.release_after) if args.emit and args.release_after > 0 else None
        frames, started = 0, time.monotonic()
        last_print = last_save = 0.0
        hands, kpts, img = [], None, None
        try:
            while pipeline.isRunning():
                if q_video is not None:
                    msg = q_video.tryGet()
                    if msg is not None:
                        img = msg.getCvFrame()
                data = q_hands.tryGet()
                if data is None:
                    time.sleep(0.005)
                    continue
                frames += 1
                now = time.monotonic()
                hands = _hands(data, pad)
                kpts, ratio, target = frame_step(hands, lock, follower, now,
                                                 args.min_confidence)
                if args.emit and target is not None:
                    print(f'{target:.2f}', flush=True)
                if lost is not None and lost.update(now, kpts is not None):
                    print(RELEASE, flush=True)
                    say(f'hand gone for {args.release_after:g} s: release sent, tracker done')
                    time.sleep(0.5)     # let the line reach the bridge
                    return 0
                palms = ' '.join(
                    f'{c:.2f}/{_fmt(hand_ratio(k))}@{k[WRIST][0]:.2f},{k[WRIST][1]:.2f}'
                    f'/L{math.dist(k[WRIST], k[MIDDLE_MCP]):.3f}'
                    f'W{math.dist(k[INDEX_MCP], k[PINKY_MCP]):.3f}' for c, k in hands)
                line = (f'{status_text(ratio, follower.level, follower.max_ratio)}'
                        f'  | palms {palms or "-"}')
                if log is not None:
                    log.write(f'{now - started:7.2f} {line}\n')
                if args.verbose or now - last_print >= 0.5:
                    rate = frames / max(now - started, 1e-6)
                    say(f'{rate:4.1f} fps  {line}')
                    last_print = now
                if args.save and img is not None and now - last_save >= 2.0:
                    import cv2
                    path = f'{args.save}/hand_{frames:05d}.jpg'
                    cv2.imwrite(path, _draw(img, hands, kpts, follower.level))
                    say(f'saved {path}')
                    last_save = now
                if publish is not None and img is not None:
                    publish(_draw(img, hands, kpts, follower.level))
        except BrokenPipeError:   # the bridge ended: stop quietly
            return 0
        except KeyboardInterrupt:
            return 130
        finally:
            if log is not None:
                log.close()
    return 0


class _Reporter:
    """Status lines through a thread that drops them rather than wait.

    A terminal that stops reading (a hidden panel, an SSH channel that
    stalls) fills the pipe, and a blocking print would then freeze the
    camera loop and the view with it, as it did on 2026-10-12. The loop
    only queues; when the queue is full the line is dropped. Targets for
    the bridge never go through here: they must not be dropped.
    """

    def __init__(self, stream, depth: int = 64) -> None:
        import queue
        import threading
        self._queue = queue.Queue(maxsize=depth)
        self._stream = stream
        threading.Thread(target=self._drain, daemon=True).start()

    def __call__(self, line: str) -> None:
        import queue
        try:
            self._queue.put_nowait(line)
        except queue.Full:
            pass

    def _drain(self) -> None:
        while True:
            line = self._queue.get()
            try:
                print(line, file=self._stream, flush=True)
            except (BrokenPipeError, ValueError):
                return


LOCK_RADIUS = 0.2     # how far the wrist may jump between frames, frame widths
LOCK_HOLD_S = 1.0     # how long a lost hand keeps the lock


class HandLock:
    """Follow the same hand while more than one is in view.

    choose() takes (confidence, landmarks) per hand, already filtered by
    confidence, and returns the landmarks to follow or None. Locked on a
    hand, it takes the hand whose wrist is nearest to where the locked
    wrist was, within LOCK_RADIUS. If the locked hand is missing it
    returns None (the gripper holds) and keeps the lock for LOCK_HOLD_S,
    so the other hand cannot take over by a blink of the detector; after
    that it locks onto the most confident hand in view.
    """

    def __init__(self, radius: float = LOCK_RADIUS, hold_s: float = LOCK_HOLD_S) -> None:
        self.radius, self.hold_s = radius, hold_s
        self.wrist = None
        self.lost_at: float | None = None

    def choose(self, hands, now: float):
        if self.wrist is not None:
            near = [(math.dist(k[WRIST], self.wrist), k) for _, k in hands]
            near = [nk for nk in near if nk[0] <= self.radius]
            if near:
                kpts = min(near, key=lambda nk: nk[0])[1]
                self.wrist, self.lost_at = kpts[WRIST], None
                return kpts
            if self.lost_at is None:
                self.lost_at = now
            if now - self.lost_at < self.hold_s:
                return None
            self.wrist = self.lost_at = None
        if not hands:
            return None
        kpts = max(hands, key=lambda h: h[0])[1]
        self.wrist = kpts[WRIST]
        return kpts


def frame_step(hands, lock: HandLock, follower: Follower, now: float,
               min_confidence: float):
    """One frame: (landmarks followed, their ratio, target to send or None)."""
    kpts = lock.choose([h for h in hands if h[0] >= min_confidence], now)
    ratio = hand_ratio(kpts) if kpts else None
    return kpts, ratio, follower.update(ratio)


def _hands(data, pad: float):
    """(confidence, landmarks) for every palm, landmarks in frame coords."""
    out = []
    for i, det in enumerate(data.reference_data.detections):
        conf = data.items[i]['1'].getFirstPrediction()
        box = det.getBoundingBox()
        x0 = box.center.x - box.size.width / 2 - pad
        y0 = box.center.y - box.size.height / 2 - pad
        w, h = box.size.width + 2 * pad, box.size.height + 2 * pad
        out.append((conf, [(x0 + w * kp.imageCoordinates.x, y0 + h * kp.imageCoordinates.y)
                           for kp in data.items[i]['0'].getKeypoints()]))
    return out




def _draw(img, hands, chosen, level):
    """The frame with every palm on it: the followed hand green, others red."""
    import cv2
    img = img.copy()
    h, w = img.shape[:2]
    for conf, kpts in hands:
        color = (0, 255, 0) if kpts is chosen else (0, 0, 255)
        for x, y in kpts:
            cv2.circle(img, (round(x * w), round(y * h)), 4, color, -1)
        x, y = kpts[WRIST]
        cv2.putText(img, f'{conf:.2f}', (round(x * w), round(y * h) + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(img, 'no hand' if chosen is None else f'gripper {_fmt(level)}',
                (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return img


def _web_view(port: int):
    """Serve the annotated frames as an MJPEG stream; returns publish(img).

    The depthai 3 venv carries headless OpenCV (no windows), so the view
    goes to a browser, as the Luxonis examples do on 8082. It listens on
    localhost only: the camera looks at the operator, so the stream is not
    put on the LAN. From the Mac, tunnel it:
    `ssh -N -L 8090:localhost:8090 jetson`, then open http://localhost:8090.
    """
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    import cv2
    latest = {'jpg': None}
    fresh = threading.Condition()

    page = (b'<!doctype html><meta charset="utf-8"><title>SOMA hand</title>'
            b'<body style="margin:0;background:#111">'
            b'<img src="/stream" style="display:block;margin:auto;max-width:100%;'
            b'max-height:100vh">')

    class Stream(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(page)))
                self.end_headers()
                self.wfile.write(page)
                return
            if self.path != '/stream':
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with fresh:
                        fresh.wait(timeout=1.0)
                        jpg = latest['jpg']
                    if jpg is not None:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: ' + str(len(jpg)).encode()
                                         + b'\r\n\r\n' + jpg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Stream)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()

    def publish(img) -> None:
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with fresh:
                latest['jpg'] = buf.tobytes()
                fresh.notify_all()
    return publish


def parse_cli(argv: list[str]):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--home-check', action='store_true',
                    help='inside the container: exit 0 only if every arm joint reads 0.0')
    ap.add_argument('--bridge', choices=ARMS, metavar='ARM',
                    help='inside the container: forward stdin targets to that arm')
    ap.add_argument('--present', action='store_true',
                    help='with --bridge: open then close the hand and the straight arm '
                         'swings to the front and shows, until the end of input')
    ap.add_argument('--lift', type=float, default=LIFT,
                    help=f'how far the base swings the arm forward, rad (default '
                         f'%(default)s = 30 deg, at most {LIFT_MAX} = 45 deg; above '
                         f'{HEAVY_LIFT} the show lasts at most {HEAVY_MAX_S:g} s)')
    ap.add_argument('--present-max', type=float, default=PRESENT_MAX_S,
                    help='seconds the show may last before the arm goes home on its own '
                         '(default %(default)s, at most 60)')
    ap.add_argument('--roll', type=float, default=ROLL_AMPLITUDE,
                    help='wrist roll each way during the show, rad (default %(default)s, '
                         'at most 0.5)')
    ap.add_argument('--release-after', type=float, default=RELEASE_AFTER_S,
                    help='with --emit: after the hand has been gone this long, send '
                         'release (the bridge walks home, then disarms); 0 never '
                         '(default %(default)s)')
    ap.add_argument('--emit', action='store_true',
                    help='print gripper targets on stdout, for the bridge')
    ap.add_argument('--show', action='store_true',
                    help='serve the frames with the hands drawn to a browser')
    ap.add_argument('--port', type=int, default=8090, help='port of --show (default 8090)')
    ap.add_argument('--closed', type=float, default=CLOSED_RATIO,
                    help='ratio read as a closed hand (default %(default)s)')
    ap.add_argument('--open', type=float, default=OPEN_RATIO,
                    help='ratio read as an open hand (default %(default)s)')
    ap.add_argument('--smoothing', type=float, default=SMOOTHING,
                    help='weight of the newest reading, lower is smoother and slower '
                         '(default %(default)s)')
    ap.add_argument('--close-below', type=float, default=CLOSE_BELOW,
                    help='at or below this the gripper shuts and holds (default %(default)s)')
    ap.add_argument('--fps', type=int, default=8, help='camera fps (default 8, RVC2)')
    ap.add_argument('--min-confidence', type=float, default=0.5)
    ap.add_argument('--no-rotate', action='store_true', help='frames as they come')
    ap.add_argument('--verbose', action='store_true', help='one status line per frame')
    ap.add_argument('--save', metavar='DIR', help='every 2 s, save a frame with the hands drawn')
    ap.add_argument('--log', metavar='PATH', help='every frame, one line to this file')
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_cli(sys.argv[1:] if argv is None else argv)
    if (not 0.0 < args.present_max <= 60.0 or not 0.0 <= args.roll <= 0.5
            or not 0.0 < args.lift <= LIFT_MAX or args.release_after < 0.0):
        print(f'--present-max must be in (0, 60] s, --roll in [0, 0.5] rad and '
              f'--lift in (0, {LIFT_MAX}] rad', file=sys.stderr)
        return 2
    if args.home_check:
        return home_check()
    if args.bridge:
        try:
            return bridge(args.bridge, args.present, args.present_max, args.roll,
                          args.lift)
        except KeyboardInterrupt:   # before the loop: nothing was commanded
            return 130
    try:
        Follower(args.closed, args.open, smoothing=args.smoothing,
                 close_below=args.close_below)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2
    if not 0.0 < args.smoothing <= 1.0:
        print('--smoothing must be in (0, 1]', file=sys.stderr)
        return 2
    return track(args)


if __name__ == '__main__':
    raise SystemExit(main())
