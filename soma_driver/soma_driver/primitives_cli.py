"""Command-line runner for SOMA primitives.

    ros2 run soma_driver soma_primitives list
    ros2 run soma_driver soma_primitives home
    ros2 run soma_driver soma_primitives demo
    ros2 run soma_driver soma_primitives relax

Publishes named poses (or sequences of them) to /soma/command and, for
`relax`, disarms through /soma/arm once the arms have settled at home.

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

from .primitives import POSES, SEQUENCES, pose_targets, sequence_steps, settle_time_s

SETTLE_MARGIN_S = 0.5


class PrimitiveRunner(Node):
    def __init__(self) -> None:
        super().__init__('soma_primitives')
        self.pub = self.create_publisher(JointState, 'soma/command', 10)
        # A late-joining publisher needs a beat before the first message
        # is seen by the driver's subscription.
        time.sleep(0.3)

    def send_pose(self, name: str) -> None:
        targets = pose_targets(name)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(targets)
        msg.position = [float(v) for v in targets.values()]
        self.pub.publish(msg)
        wait = settle_time_s(targets) + SETTLE_MARGIN_S
        self.get_logger().info(f'pose {name}: commanded, settling {wait:.1f}s')
        time.sleep(wait)

    def run_sequence(self, name: str) -> None:
        for pose, dwell in sequence_steps(name):
            targets = pose_targets(pose)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(targets)
            msg.position = [float(v) for v in targets.values()]
            self.pub.publish(msg)
            self.get_logger().info(f'{name}: pose {pose}, dwell {dwell:.1f}s')
            time.sleep(dwell)

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
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    if not args or args[0] in ('-h', '--help', 'list'):
        print(__doc__)
        print('poses:     ' + ', '.join(sorted(POSES)))
        print('sequences: ' + ', '.join(sorted(SEQUENCES)))
        print('behaviors: relax')
        return

    target = args[0]
    rclpy.init()
    node = PrimitiveRunner()
    try:
        if target == 'relax':
            node.relax()
        elif target in SEQUENCES:
            node.run_sequence(target)
        elif target in POSES:
            node.send_pose(target)
        else:
            print(f'unknown primitive: {target}')
            print('poses:     ' + ', '.join(sorted(POSES)))
            print('sequences: ' + ', '.join(sorted(SEQUENCES)))
            sys.exit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
