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
from soma_driver.pca9685_backend import MockPca9685  # noqa: E402
from soma_driver.servo_map import SERVO_MAP  # noqa: E402


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
        assert isinstance(node.backend, MockPca9685)
        assert node.backend.is_real is False
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
        assert isinstance(node.backend, MockPca9685)

    def test_disarm_always_succeeds_and_returns_to_mock(self, node):
        res = _arm(node, False)
        assert res.success is True
        assert node.armed is False
        assert isinstance(node.backend, MockPca9685)


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
