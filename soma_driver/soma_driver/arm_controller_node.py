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
  - While ARMED, a 1 Hz watchdog reads back every board's config. A board
    that reset to power-on defaults (asleep, 200 Hz) or stopped answering
    disarms the node. It never re-arms on its own: /soma/arm again, which
    re-runs the board init.

Open loop caveat: RC servos give no position feedback. On startup the node
assumes every joint sits at zero. It does not know the true pose, which is
why the ramp, the soft limits and the arming gates all exist.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from std_srvs.srv import SetBool

from .ina3221 import BATTERY_PROFILES, Ina3221, MockIna3221
from .pca9685_backend import mock_fleet, real_fleet
from .servo_map import (
    MIMIC_JOINTS, RELEASE_WHEN_SETTLED, SERVO_MAP, SETTLE_S)
from .trajectory import JointMotion

RATE_HZ = 50.0
# Board config read-back while ARMED. Not every 50 Hz tick: when a board
# resets its outputs are already dead, so the check only bounds how long
# the node keeps claiming otherwise. One second does that (same period as
# the workbench watchdog) without adding reads to every tick on the bus.
WATCHDOG_HZ = 1.0


class ArmController(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__('soma_driver',
                         parameter_overrides=parameter_overrides or [])

        # Gate 1: the node must have been launched with permission to even
        # consider talking to the I2C bus. Default is no.
        self.declare_parameter('allow_real', False)
        self.declare_parameter('i2c_address', 0x40)
        # -1 = autodetect: probe /dev/i2c-* for the PCA9685 (bus 1 on a
        # Pi, bus 7 on a Jetson Orin Nano header).
        self.declare_parameter('i2c_bus', -1)

        # Optional THIRD veto: the INA3221 battery monitor. 'none'
        # (default) keeps the exact pre-monitor behaviour. A named
        # profile publishes soma/power and refuses arming below that
        # chemistry's floor. A floor can VETO arming; nothing here can
        # arm, so the two gates above stay the only way in.
        self.declare_parameter('battery_source', 'none')
        source = str(self.get_parameter('battery_source').value)
        if source != 'none' and source not in BATTERY_PROFILES:
            raise ValueError(
                f'unknown battery_source {source!r}: pick one of '
                f"{sorted(BATTERY_PROFILES)} or 'none'")
        self.battery_floor_v = BATTERY_PROFILES.get(source)
        self.power_monitor = None
        if source != 'none':
            try:
                self.power_monitor = Ina3221()
            except Exception as exc:
                # Fail safe: supervision was requested, so without a
                # readable monitor the node still runs and simulates,
                # but arming stays refused (the mock reads 0 V, below
                # any floor) until the INA3221 answers.
                self.power_monitor = MockIna3221()
                self.get_logger().warn(
                    f'battery_source={source} but no INA3221 answered '
                    f'({exc}): arming will be refused until it does.')
            self.pub_power = self.create_publisher(
                BatteryState, 'soma/power', 10)
            self.create_timer(1.0, self._power_tick)

        # Gate 2: armed state, flipped only by the arm service. Starts off.
        self.armed = False

        # The node ALWAYS boots on the mock. No exceptions. Since the
        # two-board bench (2026-08-11) "the mock" is a fleet of mocks,
        # one per board address in the map: same spirit, more boards.
        self.backend = mock_fleet(SERVO_MAP)
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
        self.create_timer(1.0 / WATCHDOG_HZ, self._board_watchdog)

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
        if self.power_monitor is not None:
            volts = self.power_monitor.bus_voltage_v(1)
            if volts < self.battery_floor_v:
                return False, (
                    f'REFUSED: battery at {volts:.2f} V, below the '
                    f'{self.battery_floor_v:.1f} V floor. Charge or swap '
                    'the pack; the driver stays on MOCK.')
        addr_param = int(self.get_parameter('i2c_address').value)
        if addr_param != 0x40:
            self.get_logger().warn(
                'i2c_address parameter is deprecated and IGNORED: board '
                'addresses live per channel in servo_map.py now.')
        try:
            bus = int(self.get_parameter('i2c_bus').value)
            # All or none: real_fleet arms every board in the map or
            # raises, so a half armed bench cannot exist.
            self.backend = real_fleet(SERVO_MAP, armed=True,
                                      bus=None if bus < 0 else bus)
        except Exception as exc:  # hardware missing, bus down, no library
            self.backend = mock_fleet(SERVO_MAP)
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
        try:
            self.backend.disable_all()
        except Exception as exc:  # a board off the bus cannot be told
            self.backend = mock_fleet(SERVO_MAP)
            self.armed = False
            return True, (
                f'DISARMED, back on MOCK, but the signal cut failed ({exc}): '
                'a board may still be driving. Cut V+ if anything holds.')
        self.backend = mock_fleet(SERVO_MAP)
        self.armed = False
        return True, 'DISARMED: signal cut, back on MOCK'

    def _board_watchdog(self) -> str | None:
        """1 Hz while ARMED: is every board still running our config?

        Seen on the bench 2026-10-01: both boards lost their shared 3.3 V
        logic supply for an instant and came back at power-on defaults
        (MODE1 0x11, oscillator asleep; prescale 0x1E, 200 Hz). Every
        output went dead while the driver kept writing LED registers and
        reporting ARMED. A driver that says ARMED over sleeping boards is
        lying, so a reset disarms through the normal path and says which
        board. Re-arming stays the explicit /soma/arm call: the fault
        (a supply, a connector) has to be fixed by a person first.

        Returns the error logged, or None if nothing happened (tests).
        """
        if not self.armed:
            return None
        faults = self.backend.board_faults()
        if not faults:
            return None
        boards = '; '.join(f'0x{a:02x} {f}' for a, f in faults.items())
        _, outcome = self._disarm()
        message = (f'BOARD FAULT: {boards}. {outcome}. Check the 3.3 V '
                   'logic supply and the I2C harness, then /soma/arm again.')
        self.get_logger().error(message)
        return message

    def _power_tick(self) -> None:
        """Publish pack voltage and current from INA3221 channel 1 at 1 Hz."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(self.power_monitor.bus_voltage_v(1))
        msg.current = float(self.power_monitor.shunt_current_a(1))
        msg.present = not isinstance(self.power_monitor, MockIna3221)
        self.pub_power.publish(msg)

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
                    self.backend.release(spec)
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
