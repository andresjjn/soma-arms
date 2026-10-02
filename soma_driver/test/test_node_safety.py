"""Node level tests of the two arming gates. Needs ROS 2 (rclpy).

Skipped automatically on a plain laptop, and run in the ROS job of CI.
These are the tests that actually encode "the node boots MOCK and DISARMED,
and arming is an explicit service".
"""
import sys
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy', reason='ROS 2 not available')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from std_srvs.srv import SetBool  # noqa: E402

from soma_driver.arm_controller_node import ArmController  # noqa: E402
from soma_driver.pca9685_backend import MockPca9685, Pca9685Fleet  # noqa: E402
from soma_driver.servo_map import SERVO_MAP  # noqa: E402


def _is_mock_fleet(backend) -> bool:
    """Boot contract since the two-board bench (2026-08-11): the node
    boots a FLEET of mocks, one per board address in the map. Same
    spirit as before, more boards: it can simulate, never move metal."""
    return (isinstance(backend, Pca9685Fleet)
            and backend.is_real is False
            and all(isinstance(b, MockPca9685)
                    for b in backend.boards.values()))


@pytest.fixture
def node():
    rclpy.init()
    n = ArmController()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _arm(node, value: bool):
    req, res = SetBool.Request(), SetBool.Response()
    req.data = value
    return node._on_arm(req, res)


class TestBootsSafe:
    def test_boots_on_mock_and_disarmed(self, node):
        assert _is_mock_fleet(node.backend)
        assert node.armed is False

    def test_allow_real_defaults_to_false(self, node):
        assert node.allow_real is False

    def test_initial_state_is_inside_the_soft_limits(self, node):
        """The published pose must be valid from the very first tick,
        otherwise RViz and MoveIt start out of bounds."""
        for name, spec in SERVO_MAP.items():
            assert spec.lower <= node.current[name] <= spec.upper, name
        assert node.current['torso_lift_joint'] == pytest.approx(0.005)


class TestArmingGates:
    def test_arming_is_refused_without_allow_real(self, node):
        res = _arm(node, True)
        assert res.success is False
        assert node.armed is False
        assert _is_mock_fleet(node.backend)

    def test_disarm_always_succeeds_and_returns_to_mock(self, node):
        res = _arm(node, False)
        assert res.success is True
        assert node.armed is False
        assert _is_mock_fleet(node.backend)


class TestBatteryVeto:
    """battery_source adds a THIRD veto: it can refuse arming, never arm.

    On the CI machine there is no INA3221, so a node started with a
    battery profile falls back to the mock monitor (0 V): that IS the
    fail-safe contract, and the first test rides on it.
    """

    def _node_with(self, **params):
        from rclpy.parameter import Parameter
        overrides = [Parameter(k, value=v) for k, v in params.items()]
        return ArmController(parameter_overrides=overrides)

    def test_dead_battery_refuses_arming_before_any_backend(self):
        rclpy.init()
        n = None
        try:
            n = self._node_with(allow_real=True, battery_source='lipo_2s')
            res = _arm(n, True)
            assert res.success is False
            assert 'battery' in res.message.lower()
            assert n.armed is False
            assert _is_mock_fleet(n.backend)
        finally:
            if n is not None:
                n.destroy_node()
            rclpy.shutdown()

    def test_healthy_battery_lets_arming_proceed_past_the_veto(self):
        rclpy.init()
        n = None
        try:
            n = self._node_with(allow_real=True, battery_source='lipo_2s')
            n.power_monitor.set_reading(1, voltage_v=8.0, current_a=0.4)
            res = _arm(n, True)
            # The veto passed; arming then fails at the real fleet
            # (no PCA9685 on this machine), which is exactly the point:
            # a healthy pack must never be blocked by the monitor.
            assert res.success is False
            assert 'battery' not in res.message.lower()
        finally:
            if n is not None:
                n.destroy_node()
            rclpy.shutdown()

    def test_unknown_profile_refuses_to_boot(self):
        rclpy.init()
        try:
            with pytest.raises(ValueError):
                self._node_with(battery_source='potato_9v')
        finally:
            rclpy.shutdown()


class TestCommandClamping:
    def test_target_is_clamped_to_soft_limits(self, node):
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.name = ['torso_lift_joint', 'left_arm_elbow_joint']
        msg.position = [0.14, 99.0]
        node._on_command(msg)
        assert node.target['torso_lift_joint'] == pytest.approx(0.135)
        assert node.target['left_arm_elbow_joint'] == pytest.approx(
            SERVO_MAP['left_arm_elbow_joint'].upper)

    def test_unknown_joint_is_ignored(self, node):
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.name = ['not_a_joint']
        msg.position = [1.0]
        node._on_command(msg)
        assert 'not_a_joint' not in node.target


class TestBoardWatchdog:
    """1 Hz config read-back while ARMED (bench incident 2026-10-01).

    Both boards lost their 3.3 V logic supply for an instant and came
    back asleep at 200 Hz while the node kept reporting ARMED. Here the
    node is armed for real against the fake smbus2 of conftest.py (no
    hardware reachable), so the full _arm / watchdog / _disarm path runs.
    """

    def _armed_node(self):
        from rclpy.parameter import Parameter
        n = ArmController(parameter_overrides=[
            Parameter('allow_real', value=True),
            Parameter('i2c_bus', value=7)])
        res = _arm(n, True)
        assert res.success is True, res.message
        assert n.armed is True and n.backend.is_real is True
        return n

    def test_watchdog_is_idle_while_disarmed(self, node):
        for board in node.backend.boards.values():
            board.simulate_reset()
        assert node._board_watchdog() is None
        assert node.armed is False

    def test_healthy_boards_stay_armed(self, chips):
        rclpy.init()
        n = None
        try:
            n = self._armed_node()
            assert n._board_watchdog() is None
            assert n.armed is True
        finally:
            if n is not None:
                n.destroy_node()
            rclpy.shutdown()

    def test_reset_disarms_names_the_board_and_never_rearms(self, chips):
        rclpy.init()
        n = None
        try:
            n = self._armed_node()
            chips.regs[0x40][0xFD] = 0x00   # 0x40 still driving
            chips.power_glitch(0x43)
            message = n._board_watchdog()
            assert message is not None and '0x43' in message
            assert n.armed is False
            assert _is_mock_fleet(n.backend)
            # Same path as _disarm: the cut reached the board still live.
            assert chips.regs[0x40][0xFD] == 0x10
            # The fault is gone from the bus view, yet nothing re-arms.
            assert n._board_watchdog() is None
            assert n.armed is False
        finally:
            if n is not None:
                n.destroy_node()
            rclpy.shutdown()

    def test_rearming_reruns_the_board_init(self, chips):
        rclpy.init()
        n = None
        try:
            n = self._armed_node()
            chips.power_glitch(0x40)
            chips.power_glitch(0x43)
            n._board_watchdog()
            assert n.armed is False
            res = _arm(n, True)
            assert res.success is True
            assert n.backend.board_faults() == {}
            assert all(regs[0xFE] == 121 for regs in chips.regs.values())
        finally:
            if n is not None:
                n.destroy_node()
            rclpy.shutdown()

    def test_a_silent_board_still_disarms(self, chips):
        rclpy.init()
        n = None
        try:
            n = self._armed_node()
            chips.regs[0x43][0xFD] = 0x00   # 0x43 still driving
            chips.silent.add(0x40)
            message = n._board_watchdog()
            assert message is not None and 'not answering' in message
            assert n.armed is False
            assert _is_mock_fleet(n.backend)
            # 0x40 refused its cut; 0x43 must have had its own anyway.
            assert chips.regs[0x43][0xFD] == 0x10
        finally:
            if n is not None:
                n.destroy_node()
            rclpy.shutdown()
