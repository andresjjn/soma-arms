"""The ROS adapter of the v0.3 touch test, on a fake rclpy and the real one.

The fake-rclpy tests run everywhere: they load touch_test_cli.py against
stand-in modules and drive main() end to end, so the refusals (no driver
listening, arm away from home, unsafe points) and the home unwind + CSV
output are pinned on any laptop. The real-rclpy test runs only in the
ROS Humble job; its skip is raised inside the test on purpose.
"""
import csv
import importlib.util
import io
import signal
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.touch_test import arm_home  # noqa: E402

CLI = Path(__file__).resolve().parents[1] / 'soma_driver' / 'touch_test_cli.py'
HOME = {**arm_home('left'), **arm_home('right')}


class FakeROS:
    """Just enough rclpy and sensor_msgs for touch_test_cli.main()."""

    def __init__(self, listeners=1, state=None):
        self.listeners = listeners
        self.state = state
        self.published: list[dict[str, float]] = []
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
                ros.published.append(dict(zip(msg.name, msg.position)))

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
        return {
            'rclpy': rclpy,
            'rclpy.node': node,
            'rclpy.signals': signals,
            'sensor_msgs': types.ModuleType('sensor_msgs'),
            'sensor_msgs.msg': msgs,
        }


def run_main(monkeypatch, ros, argv, answers=()):
    for name, module in ros.modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        'soma_driver.touch_test_cli_on_fakes', CLI)
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
    monkeypatch.setattr(sys, 'argv', ['soma_touch_test'] + argv)
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
    ros = FakeROS(state=dict(HOME, left_arm_elbow_joint=-0.5))
    with pytest.raises(SystemExit) as exc:
        run_main(monkeypatch, ros, ['--arm', 'left'])
    assert exc.value.code == 2
    assert ros.published == []
    out = capsys.readouterr().out
    assert 'REFUSED, nothing was commanded' in out
    assert 'left_arm_elbow_joint' in out


def test_refuses_unsafe_points_before_moving(monkeypatch, capsys):
    ros = FakeROS(state=HOME)
    with pytest.raises(SystemExit) as exc:
        run_main(monkeypatch, ros, ['--points', '0,5'])
    assert exc.value.code == 2
    assert ros.published == []
    assert 'safety floor' in capsys.readouterr().out


def test_the_other_arm_away_from_home_does_not_block_this_one(monkeypatch):
    ros = FakeROS(state=dict(HOME, right_arm_elbow_joint=-0.5))
    run_main(monkeypatch, ros, ['--arm', 'left'], answers=['q'])
    assert ros.published == []


def test_walks_writes_csv_and_returns_home(monkeypatch, capsys, tmp_path):
    ros = FakeROS(state=HOME)
    csv_path = tmp_path / 'touch.csv'
    handlers = run_main(
        monkeypatch, ros,
        ['--arm', 'left', '--points', '-30,30', '--csv', str(csv_path)],
        answers=['', '-28.5 31.0'],
    )
    assert ros.init_kwargs == {'signal_handler_options': 'NO'}
    assert set(handlers) == {signal.SIGTERM, signal.SIGHUP}
    assert len(ros.published) == 2
    assert ros.published[-1] == arm_home('left')
    assert ros.shut_down

    rows = list(csv.DictReader(csv_path.open()))
    assert len(rows) == 1
    assert rows[0]['requested_x_mm'] == '-30.00'
    assert rows[0]['reached_x_mm'] == '-28.50'
    assert 'mean error' in capsys.readouterr().out


def test_real_cli_imports_and_never_arms():
    pytest.importorskip('rclpy')
    from soma_driver import touch_test_cli
    assert callable(touch_test_cli.main)
    assert 'NEVER arms' in touch_test_cli.__doc__
