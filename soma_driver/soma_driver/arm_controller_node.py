"""SOMA arm controller node: takes joint targets and executes them on a safe ramp.

Interface:
  Sub  /soma/command    (sensor_msgs/JointState: name + position, the target)
  Pub  /joint_states    (ramped current position, feeds RViz and TF)
  Srv  /soma/arm        (std_srvs/SetBool: arm or disarm the real output)

Safety rules of the project, encoded here:
  - The node ALWAYS starts on the MOCK backend and DISARMED. Arming is an
    explicit service call, and it is refused unless the node was also
    started with allow_real:=true. Two independent gates, on purpose.
  - Per joint minimum-jerk profile (trajectory.py): velocity follows a
    bell curve, so motion starts and stops softly instead of at constant
    speed with hard corners. The rate limit from servo_map remains a HARD
    CEILING enforced on every tick: no snap moves, ever.
  - Incoming targets are clamped to the soft limits, so /joint_states never
    reports a pose the hardware is not allowed to reach.
  - Self locking joints (the L16) drop their signal once settled, so a
    command is never held against a mechanical stop.
  - Mimic fingers are computed here (physical gear), never commanded.

Open loop caveat: RC servos give no position feedback. On startup the node
assumes every joint sits at zero. It does not know the true pose, which is
why the ramp, the soft limits and the arming gates all exist.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .pca9685_backend import MockPca9685, RealPca9685
from .servo_map import (
    MIMIC_JOINTS, RELEASE_WHEN_SETTLED, SERVO_MAP, SETTLE_S)
from .trajectory import JointMotion

RATE_HZ = 50.0


class ArmController(Node):
    def __init__(self) -> None:
        super().__init__('soma_driver')

        # Gate 1: the node must have been launched with permission to even
        # consider talking to the I2C bus. Default is no.
        self.declare_parameter('allow_real', False)
        self.declare_parameter('i2c_address', 0x40)

        # Gate 2: armed state, flipped only by the arm service. Starts off.
        self.armed = False

        # The node ALWAYS boots on the mock. No exceptions.
        self.backend = MockPca9685()
        self.get_logger().info(
            'SOMA driver up: MOCK backend, DISARMED. Pulses are logged, '
            'nothing moves.')
        if self.allow_real:
            self.get_logger().warn(
                'allow_real:=true. Calling /soma/arm with data:=true WILL '
                'energize the servos.')

        # State: current and target position per joint. Zero, clamped into
        # the soft band, so the published state is never outside the URDF
        # limits. For the torso that means 5 mm, which is also exactly the
        # pulse the driver emits on its first tick.
        self.current = {name: spec.clamp(0.0) for name, spec in SERVO_MAP.items()}
        self.target = dict(self.current)
        # One minimum-jerk planner per joint, seeded at the initial pose.
        self.motion = {name: JointMotion(self.current[name], spec.max_rate)
                       for name, spec in SERVO_MAP.items()}
        # Time settled on target, used to release self locking joints.
        self.settled_s = {name: 0.0 for name in RELEASE_WHEN_SETTLED}

        self.create_subscription(JointState, 'soma/command', self._on_command, 10)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.create_service(SetBool, 'soma/arm', self._on_arm)
        self.create_timer(1.0 / RATE_HZ, self._tick)

    @property
    def allow_real(self) -> bool:
        return bool(self.get_parameter('allow_real').value)

    def _on_command(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            spec = SERVO_MAP.get(name)
            if spec is not None:
                # Clamp on arrival: the published joint state must never
                # claim a pose outside the soft limits.
                clamped = spec.clamp(pos)
                self.target[name] = clamped
                # Replans from the CURRENT position and velocity, so a new
                # command mid-motion never causes a discontinuity.
                self.motion[name].set_target(clamped)
            elif name not in MIMIC_JOINTS:
                self.get_logger().warn(f'unknown joint: {name}')

    def _on_arm(self, req: SetBool.Request, res: SetBool.Response):
        if req.data:
            res.success, res.message = self._arm()
        else:
            res.success, res.message = self._disarm()
        self.get_logger().warn(f'SOMA: {res.message}')
        return res

    def _arm(self) -> tuple[bool, str]:
        if self.armed:
            return True, 'already ARMED'
        if not self.allow_real:
            return False, (
                'REFUSED: node was started with allow_real:=false. Restart '
                'with allow_real:=true to enable the real backend.')
        try:
            addr = int(self.get_parameter('i2c_address').value)
            self.backend = RealPca9685(i2c_address=addr, armed=True)
        except Exception as exc:  # hardware missing, bus down, no library
            self.backend = MockPca9685()
            return False, f'REFUSED: real backend failed ({exc}). Staying on MOCK.'
        # Nothing moves on the arming call itself: hold the current pose
        # until a fresh command arrives. Any in-flight trajectory is
        # retargeted to where the joint is right now, so it decelerates
        # smoothly instead of continuing toward a stale goal.
        self.target = dict(self.current)
        for name, m in self.motion.items():
            m.set_target(self.current[name])
        self.armed = True
        return True, 'ARMED: real PCA9685 output is live'

    def _disarm(self) -> tuple[bool, str]:
        self.backend.disable_all()
        self.backend = MockPca9685()
        self.armed = False
        return True, 'DISARMED: signal cut, back on MOCK'

    def _tick(self) -> None:
        dt = 1.0 / RATE_HZ
        names, positions = [], []
        for name, spec in SERVO_MAP.items():
            self.current[name] = self.motion[name].step(dt)
            if name in RELEASE_WHEN_SETTLED:
                # L16: self locking lead screw. Once settled, signal off.
                # Holding PWM against a stop wedges it (2026-07-22).
                if self.current[name] == self.target[name]:
                    self.settled_s[name] += dt
                else:
                    self.settled_s[name] = 0.0
                if self.settled_s[name] >= SETTLE_S:
                    self.backend.release(spec.channel)
                else:
                    self.backend.write(spec, self.current[name])
            else:
                self.backend.write(spec, self.current[name])
            names.append(name)
            positions.append(self.current[name])
        # Mirrored fingers (physical gear pair)
        for mimic, (master, mult) in MIMIC_JOINTS.items():
            names.append(mimic)
            positions.append(self.current[master] * mult)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.pub_js.publish(msg)


def main() -> None:
    rclpy.init()
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.backend.disable_all()
        node.destroy_node()


if __name__ == '__main__':
    main()
