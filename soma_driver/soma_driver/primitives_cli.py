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
a prompt that is already on screen. Abort with q: Ctrl-C ends the
program and leaves the arm where it is, held by the driver.

This tool NEVER arms the driver and holds no safety logic of its own:
the two gates (allow_real parameter + /soma/arm service) live in the
driver and are exercised by a human. If the driver is disarmed, running
this moves nothing real, which is exactly the point.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .player import SETTLE_MARGIN_S, fresh_input, parse_cli, play
from .primitives import POSES, SEQUENCES, pose_targets, settle_time_s


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
        """Play a sequence; False when a human unwound it in step mode."""
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


def main() -> None:
    target, step = parse_cli(sys.argv[1:])
    if target is None:
        print(__doc__)
        print('poses:     ' + ', '.join(sorted(POSES)))
        print('sequences: ' + ', '.join(sorted(SEQUENCES)))
        print('behaviors: relax')
        print('flags:     --step (sequences only)')
        return

    rclpy.init()
    node = PrimitiveRunner()
    completed = True
    try:
        if target == 'relax':
            node.relax()
        elif target in SEQUENCES:
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
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not completed:
        sys.exit(3)


if __name__ == '__main__':
    main()
