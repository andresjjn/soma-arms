"""Bench tool for v0.2: does every joint's metal move the hug way?

    ros2 run soma_driver soma_sign_check                  # the right arm
    ros2 run soma_driver soma_sign_check --arm left
    ros2 run soma_driver soma_sign_check --arm both --report PATH.json
    options: --delta RAD (default 0.25, at most 0.5), --dwell S (default 2)

Order per arm, most trusted first: yaw (already verified on the metal, so
it proves the whole pipeline), finger, wrist roll, wrist pitch, elbow,
shoulder. For each joint the command and what the metal should do are
printed; ENTER moves it 0.25 rad toward the hug side (never within 0.1
rad of a measured stop), holds it, commands it back to 0.0, and asks:

    ENTER  ok        the metal moved as expected
    r      reversed  the metal moved the OTHER way. A DRIVER fix: mirror
                     the joint's SERVO_MAP row (the summary prints the
                     new row and the xacro limits). Never an axis flip.
    m      model     the metal was right and RViz drew it the other way.
                     A model fix: the joint's <axis> line. Driver untouched.
    u      unclear   the same joint again

`s` skips a joint. `q`, Ctrl-C or the end of input command the current
joint back to 0.0 before leaving, and the summary is printed either way.
Keys pressed while a joint moves are discarded: every answer is typed
after its question is on screen.

This tool NEVER arms the driver and holds no safety logic of its own: the
two gates (allow_real + /soma/arm) live in the driver and in Andres's
hands. With the driver disarmed (its boot state) nothing physical moves,
so the walk can be rehearsed end to end on the mock. It refuses to start
unless the driver is listening and /joint_states reads 0.0 on every joint
it will move: a walk that starts elsewhere can show the right motion as
the wrong direction. The pure logic lives in sign_check.py and is tested
without ROS; the bench procedure is docs/session_v02.md.
"""
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from .player import fresh_input
from .sign_check import joints_for, not_at_zero, parse_cli, report, run, summary

READY_TIMEOUT_S = 5.0


def init_ros() -> None:
    """rclpy WITHOUT its signal handlers.

    On Humble, rclpy's own SIGINT handler shuts the context down, and a
    shut-down context cannot publish: a Ctrl-C during a hold would leave
    the joint at its target. With no rclpy handler, Ctrl-C is a plain
    KeyboardInterrupt and run() can still command the joint back to 0.0.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)


def _as_interrupt(signum, frame):
    raise KeyboardInterrupt


def command_msg(name: str, position: float) -> JointState:
    msg = JointState()
    msg.name = [name]
    msg.position = [float(position)]
    return msg


class SignCheck(Node):
    def __init__(self) -> None:
        super().__init__('soma_sign_check')
        self.pub = self.create_publisher(JointState, 'soma/command', 10)
        self.state: dict[str, float] | None = None
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._on_state, 10)

    def _on_state(self, msg: JointState) -> None:
        self.state = dict(zip(msg.name, msg.position))

    def publish(self, name: str, position: float) -> None:
        msg = command_msg(name, position)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def wait_ready(self, timeout: float = READY_TIMEOUT_S) -> str | None:
        """None once the driver listens and /joint_states arrived, else why not.

        Also the beat a late-joining publisher needs before its first
        message is seen by the driver's subscription.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pub.get_subscription_count() > 0 and self.state is not None:
                self.destroy_subscription(self._sub)
                return None
        if self.pub.get_subscription_count() == 0:
            return 'nobody listens on /soma/command: is the driver running?'
        return f'no /joint_states within {timeout:g} s: is the driver running?'


def main() -> None:
    options = parse_cli(sys.argv[1:])
    planned = joints_for(options.arm)
    # A hang-up or a kill gets the same way out as Ctrl-C: joint to 0.0.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _as_interrupt)
    init_ros()
    node = SignCheck()
    walking = False
    try:
        problem = node.wait_ready()
        if problem is None:
            off = not_at_zero(node.state, planned)
            if off:
                problem = ('not at 0.0 on /joint_states: ' + ', '.join(off)
                           + '. Bring the arm home (soma_primitives home), '
                           'then run this again.')
        if problem is not None:
            print(f'REFUSED, nothing was commanded: {problem}')
            sys.exit(2)
        # From here run() owns every exit: q, Ctrl-C, end of input and
        # errors all command the current joint back to 0.0 first.
        walking = True
        findings = run(options.arm, node.publish, time.sleep, fresh_input,
                       print, options.delta, options.dwell)
    except KeyboardInterrupt:
        if walking:
            print('\ninterrupted again: every joint the walk moved was '
                  'already commanded back to 0.0. No summary this time; '
                  'read /joint_states.')
        else:
            print('\ninterrupted before the walk began: nothing was commanded.')
        sys.exit(130)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print()
    print(summary(findings, planned))
    if options.report:
        record = report(options, findings, planned)
        record['date'] = datetime.now().isoformat(timespec='seconds')
        Path(options.report).write_text(json.dumps(record, indent=2) + '\n')
        print(f'report written to {options.report}')


if __name__ == '__main__':
    main()
