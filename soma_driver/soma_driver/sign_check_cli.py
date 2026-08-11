"""Interactive bench tool: verify the SIGN of every arm joint, one by one.

    ros2 run soma_driver soma_sign_check

The v0.1 model froze measured lengths, parallel axes and per-side limits,
but the world-direction of each joint's positive sense could not be read
with a caliper. This walks the 12 arm joints; for each one it commands a
small excursion toward the roomier side of its band, waits, and returns
it to zero. A human watches the metal (or RViz side by side) and writes
down every joint that moves OPPOSITE to the hug expectation. Each finding
is a one-line axis flip in soma_arm.xacro.

Safety, unchanged: this tool never arms anything. With the driver
DISARMED (its boot state) nothing physical moves and the walk can be
rehearsed end to end in mock. Moving real metal requires the human
ritual: compact resting arms, V+ up, allow_real:=true at launch, and an
explicit /soma/arm call. One joint at a time, small angles, hands clear.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .primitives import COMMANDED_JOINTS
from .servo_map import SERVO_MAP

DELTA = 0.25          # rad, about 14 degrees
DWELL_S = 2.0

HUG_HINT = {
    'yaw': 'positive = swing BACKWARD/outward (hug says negative is forward)',
    'shoulder': 'positive = up/outward per the hug rule',
    'elbow': 'positive = open outward (negative is the hug bend)',
    'wrist_pitch': 'positive = pitch outward/up',
    'wrist_roll': 'positive = roll outward (grasp features away from front)',
    'finger': 'positive = OPEN (0.0 is closed, measured)',
}


def excursion(name: str) -> float:
    """Small test target toward whichever side of the band has room."""
    spec = SERVO_MAP[name]
    if spec.upper >= DELTA:
        return DELTA
    return max(spec.lower, -DELTA)


def hint_for(name: str) -> str:
    for key, text in HUG_HINT.items():
        if key in name:
            return text
    return 'direction per the hug convention'


def main() -> None:
    rclpy.init()
    node = Node('soma_sign_check')
    pub = node.create_publisher(JointState, 'soma/command', 10)
    time.sleep(0.3)

    def command(name: str, position: float) -> None:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = [name]
        msg.position = [float(position)]
        pub.publish(msg)

    print(__doc__)
    findings: list[str] = []
    try:
        for name in COMMANDED_JOINTS:
            target = excursion(name)
            print(f'\n=== {name}')
            print(f'    will command {target:+.2f} rad, hold {DWELL_S:.0f}s, return to zero')
            print(f'    expectation: {hint_for(name)}')
            answer = input('    ENTER to move, s to skip, q to quit: ').strip().lower()
            if answer == 'q':
                break
            if answer == 's':
                continue
            command(name, target)
            time.sleep(DWELL_S)
            command(name, 0.0)
            time.sleep(1.0)
            verdict = input(
                '    did it match the expectation? ENTER = yes, f = FLIP needed: ').strip().lower()
            if verdict == 'f':
                findings.append(name)
                print('    noted for an axis flip.')
    except (KeyboardInterrupt, EOFError):
        print('\ninterrupted, returning nothing else to zero implicitly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('\n=== sign check summary')
    if findings:
        print('joints needing an axis flip in soma_arm.xacro:')
        for name in findings:
            print(f'  - {name}')
        print('flip = the joint\'s <axis> sign for that side; re-run to confirm.')
    else:
        print('no flips noted. The hug convention survived contact with power.')


if __name__ == '__main__':
    main()
