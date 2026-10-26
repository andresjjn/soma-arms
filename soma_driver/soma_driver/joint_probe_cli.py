"""ros2 run soma_driver soma_joint_probe --arm left --joint elbow [--step 0.2] [--most 0.6]

Measures how far one planar joint really turns per radian commanded (see
joint_probe.py). Run it ARMED (Andres's call, rule 1), both arms hanging
at home: it refuses unless every joint of that arm reads 0.0, because the
reading is the link's inclination from the vertical and only means the
joint's angle when every other joint is straight. It never arms anything.

    soma ros2 run soma_driver soma_joint_probe --arm left --joint elbow \\
        --most 0.6 --report /ros2_ws/src/soma-arms/calibration/probe.json
"""
import argparse
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy

from .joint_probe import PLANAR, STEP, Walk, plan, report, run, summary
from .player import fresh_input, strip_ros_segment
from .sign_check import not_at_zero
from .sign_check_cli import SignCheck, _as_interrupt, init_ros

ARM_JOINTS = PLANAR + ('wrist_roll',)


def parse_cli(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog='soma_joint_probe',
        description='Measure a planar joint\'s real angle per radian, with a '
                    'phone level. Never arms the driver.')
    parser.add_argument('--arm', choices=('right', 'left'), required=True)
    parser.add_argument('--joint', choices=PLANAR, required=True)
    parser.add_argument('--step', type=float, default=STEP,
                        help=f'rad per stair (default {STEP}, 0.05 to 0.3)')
    parser.add_argument('--most', type=float, default=None,
                        help='stop the staircase here, rad forward (default: the '
                             "joint's ceiling, 0.1 rad inside its stop)")
    parser.add_argument('--report', default=None, help='write the readings as JSON here')
    args = parser.parse_args(strip_ros_segment(argv))
    try:
        args.targets = plan(args.arm, args.joint, args.step, args.most)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def main() -> None:
    args = parse_cli(sys.argv[1:])
    walk = Walk(args.arm, args.joint, args.targets)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _as_interrupt)
    init_ros()
    node = SignCheck()
    walking = False
    try:
        problem = node.wait_ready()
        if problem is None:
            off = not_at_zero(node.state, tuple(f'{args.arm}_arm_{j}_joint' for j in ARM_JOINTS))
            if off:
                problem = ('not at 0.0 on /joint_states: ' + ', '.join(off)
                           + '. Bring the arm home first.')
        if problem is not None:
            print(f'REFUSED, nothing was commanded: {problem}')
            sys.exit(2)
        walking = True
        run(walk, node.publish, time.sleep, fresh_input, print)
    except KeyboardInterrupt:
        print('\ninterrupted again: the joint was already on its way down; '
              'read /joint_states.' if walking else
              '\ninterrupted before the walk began: nothing was commanded.')
        sys.exit(130)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print()
    print(summary(walk))
    if args.report:
        record = report(walk)
        record['date'] = datetime.now().isoformat(timespec='seconds')
        Path(args.report).write_text(json.dumps(record, indent=2) + '\n')
        print(f'report written to {args.report}')


if __name__ == '__main__':
    main()
