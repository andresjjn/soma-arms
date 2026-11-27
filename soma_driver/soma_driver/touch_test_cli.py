"""Bench CLI for v0.3: step through planar IK points and log error in mm.

    ros2 run soma_driver soma_touch_test --arm left
    ros2 run soma_driver soma_touch_test --arm left --points -30,30 0,30 30,30

This tool NEVER arms the driver. It checks that /joint_states reads 0.0
on every joint of the selected arm before moving, steps one point per
ENTER, and returns the arm home on q, Ctrl-C, EOF, or any error.
"""
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
from .sign_check import not_at_zero
from .touch_test import arm_home, parse_cli, plan_trials, run_trials, summarize, write_csv

READY_TIMEOUT_S = 5.0


def _as_interrupt(signum, frame):
    raise KeyboardInterrupt


class TouchTestNode(Node):
    def __init__(self) -> None:
        super().__init__('soma_touch_test')
        self.pub = self.create_publisher(JointState, 'soma/command', 10)
        self.state: dict[str, float] | None = None
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._on_state, 10)

    def _on_state(self, msg: JointState) -> None:
        self.state = dict(zip(msg.name, msg.position))

    def publish_pose(self, pose: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(pose.keys())
        msg.position = [float(v) for v in pose.values()]
        self.pub.publish(msg)

    def wait_ready(self, timeout: float = READY_TIMEOUT_S) -> str | None:
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
    try:
        trials = plan_trials(options.arm, options.parsed_points, phi_deg=options.tilt)
    except ValueError as exc:
        print(f'REFUSED, nothing was commanded: {exc}')
        sys.exit(2)

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _as_interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TouchTestNode()
    planned_joints = tuple(arm_home(options.arm).keys())
    walking = False
    results = []
    try:
        problem = node.wait_ready()
        if problem is None:
            off = not_at_zero(node.state, planned_joints)
            if off:
                problem = ('not at 0.0 on /joint_states: ' + ', '.join(off)
                           + '. Bring the arm home (soma_primitives home), '
                           'then run this again.')
        if problem is not None:
            print(f'REFUSED, nothing was commanded: {problem}')
            sys.exit(2)
        walking = True
        results = run_trials(
            trials, options.arm, node.publish_pose, time.sleep,
            fresh_input, print, dwell=options.dwell)
    except KeyboardInterrupt:
        if walking:
            print('\ninterrupted: arm returned home.')
        else:
            print('\ninterrupted before the walk began: nothing was commanded.')
        sys.exit(130)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print()
    print(summarize(results))
    if results:
        stamp = datetime.now().strftime('%Y-%m-%d_%H%M')
        out_path = options.csv or Path(f'calibration/touch_test_{stamp}_{options.arm}.csv')
        write_csv(out_path, results)
        print(f'csv written to {out_path}')


if __name__ == '__main__':
    main()
