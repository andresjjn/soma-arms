"""Command-line runner for SOMA primitives.

    ros2 run soma_driver soma_primitives list
    ros2 run soma_driver soma_primitives home
    ros2 run soma_driver soma_primitives demo
    ros2 run soma_driver soma_primitives wave --step
    ros2 run soma_driver soma_primitives relax

Publishes named poses (or sequences of them) to /soma/command and, for
`relax`, disarms through /soma/arm once the arms have settled at home.

Step mode (`--step`, sequences only) publishes one pose, waits for ENTER,
and on `q` unwinds to home along the path already walked, one proven step
at a time, never with a single jump. It is how a sequence meets the metal
for the first time; the logic lives in player.py and is tested without
ROS. Keys pressed during a step are discarded, so ENTER always answers
a prompt that is already on screen.

Ctrl-C during a sequence unwinds exactly like q, with or without
--step, at a prompt or in the middle of a move (one on new ground is
reversed first, one retracing a move already made is let finish); so do
the end of input, a hang-up and a kill. A second Ctrl-C, during the
unwind, stops it there: the arm keeps the last pose commanded, held by
the driver. During a single pose Ctrl-C stops nothing, the pose is
already commanded; during relax it can leave the driver ARMED, and says
so.

This tool NEVER arms the driver and holds no safety logic of its own:
the two gates (allow_real parameter + /soma/arm service) live in the
driver and are exercised by a human. If the driver is disarmed, running
this moves nothing real, which is exactly the point.
"""
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .player import SETTLE_MARGIN_S, fresh_input, parse_cli, play
from .primitives import POSES, SEQUENCES, pose_targets, settle_time_s


def init_ros() -> None:
    """rclpy WITHOUT its signal handlers, as in sign_check_cli.

    On Humble, rclpy's own SIGINT handler shuts the context down, and a
    shut-down context cannot publish: a Ctrl-C in the middle of the wave
    would leave the arm raised. With no rclpy handler, Ctrl-C is a plain
    KeyboardInterrupt and play() can still unwind home. The price is
    that Ctrl-C no longer wakes an rclpy wait; the only one here is the
    disarm call of relax, bounded at 5 s.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)


def _as_interrupt(signum, frame):
    raise KeyboardInterrupt


class PrimitiveRunner(Node):
    def __init__(self) -> None:
        super().__init__('soma_primitives')
        self.pub = self.create_publisher(JointState, 'soma/command', 10)
        # A late-joining publisher needs a beat before the first message
        # is seen by the driver's subscription.
        time.sleep(0.3)

    def _publish(self, pose: str, targets: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(targets)
        msg.position = [float(v) for v in targets.values()]
        self.pub.publish(msg)

    def send_pose(self, name: str) -> None:
        targets = pose_targets(name)
        self._publish(name, targets)
        wait = settle_time_s(targets) + SETTLE_MARGIN_S
        self.get_logger().info(f'pose {name}: commanded, settling {wait:.1f}s')
        time.sleep(wait)

    def run_sequence(self, name: str, step: bool = False) -> bool:
        """Play a sequence; False when it was stopped and unwound home."""
        return play(name, self._publish, time.sleep,
                    ask=fresh_input if step else None,
                    log=self.get_logger().info)

    def relax(self) -> None:
        """Home, settle, then cut the signal: rest before silence."""
        self.send_pose('home')
        client = self.create_client(SetBool, 'soma/arm')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'soma/arm service not up: driver not running? Signal NOT cut.')
            return
        req = SetBool.Request()
        req.data = False
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is not None:
            self.get_logger().info(f'disarm: {result.message}')
        else:
            self.get_logger().warn('disarm call timed out')


def _interrupted(target: str) -> str:
    """What a Ctrl-C that reaches main() has left behind."""
    if target in SEQUENCES:
        # play() catches the first one, so this one cut the unwind short.
        return ('interrupted during the unwind: the arm holds the last pose '
                'commanded (read /joint_states). If that is not home, walk '
                'it home one pose at a time: docs/session_v02.md step 7.')
    if target == 'relax':
        return ('relax interrupted: home was commanded, but the driver may '
                'still be ARMED. Disarm with /soma/arm false (harmless if it '
                'already is).')
    return (f'interrupted: {target} was already commanded; the driver '
            'finishes the move and holds it.')


def main() -> None:
    target, step = parse_cli(sys.argv[1:])
    if target is None:
        print(__doc__)
        print('poses:     ' + ', '.join(sorted(POSES)))
        print('sequences: ' + ', '.join(sorted(SEQUENCES)))
        print('behaviors: relax')
        print('flags:     --step (sequences only)')
        return

    # A hang-up or a kill gets the same way out as Ctrl-C: the unwind.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _as_interrupt)
    init_ros()
    node = PrimitiveRunner()
    completed = True
    try:
        if target == 'relax':
            node.relax()
        elif target in SEQUENCES:
            # play() owns every exit from here: q, Ctrl-C, the end of
            # input and errors all unwind home first.
            completed = node.run_sequence(target, step=step)
        elif target in POSES:
            if step:
                print('--step applies to sequences only; sending the pose once')
            node.send_pose(target)
        else:
            print(f'unknown primitive: {target}')
            print('poses:     ' + ', '.join(sorted(POSES)))
            print('sequences: ' + ', '.join(sorted(SEQUENCES)))
            sys.exit(2)
    except KeyboardInterrupt:
        print('\n' + _interrupted(target))
        sys.exit(130)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not completed:
        sys.exit(3)


if __name__ == '__main__':
    main()
