"""The ROS adapter of the sign check, on a fake rclpy and on the real one.

The fake-rclpy tests run everywhere: they load sign_check_cli.py against
stand-in modules and drive main() end to end, so the refusals (no driver
listening, an arm away from home) and the wiring are pinned on any
laptop. The real-rclpy tests run only in the ROS Humble job; their skip
is raised inside each test on purpose, because a module-level
importorskip aborts collection on the old pytest of the ROS container.
"""
import importlib.util
import io
import json
import signal
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.sign_check import joints_for  # noqa: E402

CLI = Path(__file__).resolve().parents[1] / 'soma_driver' / 'sign_check_cli.py'
HOME = {name: 0.0 for name in joints_for('both')}


class FakeROS:
    """Just enough rclpy and sensor_msgs for main() to run."""

    def __init__(self, listeners=1, state=None):
        self.listeners = listeners
        self.state = state
        self.published: list[tuple[str, float]] = []
        self.init_kwargs = None
        self.shut_down = False

    def modules(self):
        ros = self

        class SignalHandlerOptions:
            NO, ALL = 'NO', 'ALL'

        class JointState:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None)
                self.name, self.position = [], []

        class Publisher:
            def publish(self, msg):
                ros.published.append((msg.name[0], msg.position[0]))

            def get_subscription_count(self):
                return ros.listeners

        class Node:
            def __init__(self, name):
                self.callback = None

            def create_publisher(self, msg_type, topic, depth):
                assert topic == 'soma/command'
                return Publisher()

            def create_subscription(self, msg_type, topic, callback, depth):
                assert topic == 'joint_states'
                self.callback = callback
                return object()

            def destroy_subscription(self, sub):
                self.callback = None

            def get_clock(self):
                now = types.SimpleNamespace(to_msg=lambda: 0)
                return types.SimpleNamespace(now=lambda: now)

            def destroy_node(self):
                pass

        def spin_once(node, timeout_sec=None):
            if ros.state is not None and node.callback is not None:
                msg = JointState()
                msg.name, msg.position = list(ros.state), list(ros.state.values())
                node.callback(msg)

        rclpy = types.ModuleType('rclpy')
        rclpy.init = lambda **kwargs: setattr(ros, 'init_kwargs', kwargs)
        rclpy.spin_once = spin_once
        rclpy.try_shutdown = lambda: setattr(ros, 'shut_down', True)
        node = types.ModuleType('rclpy.node')
        node.Node = Node
        signals = types.ModuleType('rclpy.signals')
        signals.SignalHandlerOptions = SignalHandlerOptions
        msgs = types.ModuleType('sensor_msgs.msg')
        msgs.JointState = JointState
        return {'rclpy': rclpy, 'rclpy.node': node, 'rclpy.signals': signals,
                'sensor_msgs': types.ModuleType('sensor_msgs'),
                'sensor_msgs.msg': msgs}


def run_main(monkeypatch, ros, argv, answers=()):
    """Load the CLI against `ros` under a private name and run main()."""
    for name, module in ros.modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        'soma_driver.sign_check_cli_on_fakes', CLI)
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)

    clock = iter(range(10_000))
    monkeypatch.setattr(cli, 'time', types.SimpleNamespace(
        sleep=lambda s: None, monotonic=lambda: next(clock) * 0.1))
    handlers = {}
    monkeypatch.setattr(signal, 'signal', lambda sig, h: handlers.update({sig: h}))
    pending = list(answers)
    monkeypatch.setattr('builtins.input', lambda prompt='': pending.pop(0))
    monkeypatch.setattr(sys, 'stdin', io.StringIO(''))
    monkeypatch.setattr(sys, 'argv', ['soma_sign_check'] + argv)
    cli.main()
    return handlers


def test_refuses_when_nobody_listens(monkeypatch, capsys):
    ros = FakeROS(listeners=0, state=HOME)
    with pytest.raises(SystemExit) as exc:
        run_main(monkeypatch, ros, [])
    assert exc.value.code == 2
    assert ros.published == []
    assert 'nobody listens on /soma/command' in capsys.readouterr().out
    assert ros.shut_down


def test_refuses_when_the_arm_is_not_home(monkeypatch, capsys):
    ros = FakeROS(state=dict(HOME, right_arm_elbow_joint=-0.5))
    with pytest.raises(SystemExit) as exc:
        run_main(monkeypatch, ros, ['--arm', 'right'])
    assert exc.value.code == 2
    assert ros.published == []
    out = capsys.readouterr().out
    assert 'REFUSED, nothing was commanded' in out
    assert 'right_arm_elbow_joint' in out


def test_the_other_arm_away_from_home_does_not_block_this_one(monkeypatch):
    ros = FakeROS(state=dict(HOME, left_arm_elbow_joint=-0.5))
    run_main(monkeypatch, ros, ['--arm', 'right'], answers=['q'])
    assert ros.published == [('right_arm_yaw_joint', 0.0)]


def test_walks_summarizes_and_writes_the_report(monkeypatch, capsys, tmp_path):
    ros = FakeROS(state=HOME)
    path = tmp_path / 'sign_check.json'
    handlers = run_main(monkeypatch, ros, ['--report', str(path)],
                        answers=['', 'r', 'q'])
    assert ros.init_kwargs == {'signal_handler_options': 'NO'}
    assert set(handlers) == {signal.SIGTERM, signal.SIGHUP}
    assert ros.published == [('right_arm_yaw_joint', -0.25),
                             ('right_arm_yaw_joint', 0.0),
                             ('right_arm_finger_l_joint', 0.0)]
    assert ros.shut_down
    out = capsys.readouterr().out
    assert 'REVERSED right_arm_yaw_joint' in out
    record = json.loads(path.read_text())
    assert record['findings'] == [
        {'joint': 'right_arm_yaw_joint', 'target': -0.25, 'verdict': 'reversed'}]
    assert 'date' in record


def test_real_cli_imports_and_never_arms():
    pytest.importorskip('rclpy')
    from soma_driver import sign_check_cli
    assert callable(sign_check_cli.main)
    assert 'NEVER arms' in sign_check_cli.__doc__
    assert sign_check_cli.parse_cli(['--arm', 'both']).arm == 'both'


def test_real_rclpy_starts_without_its_signal_handlers(monkeypatch):
    """On Humble, rclpy's SIGINT handler shuts the context down; with it,
    a Ctrl-C during a hold could not command the joint back to 0.0."""
    pytest.importorskip('rclpy')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from soma_driver import sign_check_cli
    seen = {}
    monkeypatch.setattr(rclpy, 'init', lambda **kwargs: seen.update(kwargs))
    sign_check_cli.init_ros()
    assert seen == {'signal_handler_options': SignalHandlerOptions.NO}
    msg = sign_check_cli.command_msg('right_arm_yaw_joint', -0.25)
    assert list(msg.name) == ['right_arm_yaw_joint']
    assert list(msg.position) == [-0.25]
