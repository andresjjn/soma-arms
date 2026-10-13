"""The ROS adapter of the player, on a fake rclpy and on the real one.

The fake-rclpy tests run everywhere: they load primitives_cli.py against
stand-in modules and drive main() end to end, so the way a Ctrl-C reaches
the player (no rclpy signal handlers, a hang-up or a kill taken as Ctrl-C,
the exit codes and what the human is told) is pinned on any laptop. The
pure logic is covered by test_player.py. The real-rclpy tests run only in
the ROS Humble job; their skip is raised inside each test on purpose,
because a module-level importorskip aborts collection on the old pytest
of the ROS container.
"""
import importlib.util
import io
import signal
import sys
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.player import plan  # noqa: E402
from soma_driver.primitives import HOME  # noqa: E402

CLI = Path(__file__).resolve().parents[1] / 'soma_driver' / 'primitives_cli.py'


class FakeROS:
    """Just enough rclpy, sensor_msgs and std_srvs for main() to run."""

    def __init__(self):
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

        class Node:
            def __init__(self, name):
                pass

            def create_publisher(self, msg_type, topic, depth):
                assert topic == 'soma/command'
                return Publisher()

            def get_clock(self):
                now = types.SimpleNamespace(to_msg=lambda: 0)
                return types.SimpleNamespace(now=lambda: now)

            def get_logger(self):
                return types.SimpleNamespace(info=print, warn=print)

            def destroy_node(self):
                pass

        rclpy = types.ModuleType('rclpy')
        rclpy.init = lambda **kwargs: setattr(ros, 'init_kwargs', kwargs)
        rclpy.try_shutdown = lambda: setattr(ros, 'shut_down', True)
        node = types.ModuleType('rclpy.node')
        node.Node = Node
        signals = types.ModuleType('rclpy.signals')
        signals.SignalHandlerOptions = SignalHandlerOptions
        msgs = types.ModuleType('sensor_msgs.msg')
        msgs.JointState = JointState
        srvs = types.ModuleType('std_srvs.srv')
        srvs.SetBool = object   # relax is not driven here
        return {'rclpy': rclpy, 'rclpy.node': node, 'rclpy.signals': signals,
                'sensor_msgs': types.ModuleType('sensor_msgs'),
                'sensor_msgs.msg': msgs,
                'std_srvs': types.ModuleType('std_srvs'),
                'std_srvs.srv': srvs}


def run_main(monkeypatch, ros, argv, ctrl_c_in_sleep=(), answers=()):
    """Load the CLI against `ros` under a private name and run main().

    Sleep calls are counted from 0, and call 0 is the beat the publisher
    waits after it is created. Returns (exit code, signal handlers set).
    """
    for name, module in ros.modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        'soma_driver.primitives_cli_on_fakes', CLI)
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)

    slept = []

    def sleep(seconds):
        slept.append(seconds)
        if len(slept) - 1 in ctrl_c_in_sleep:
            raise KeyboardInterrupt

    monkeypatch.setattr(cli, 'time', types.SimpleNamespace(sleep=sleep))
    handlers = {}
    monkeypatch.setattr(signal, 'signal', lambda sig, h: handlers.update({sig: h}))
    pending = list(answers)
    monkeypatch.setattr('builtins.input', lambda prompt='': pending.pop(0))
    monkeypatch.setattr(sys, 'stdin', io.StringIO(''))
    monkeypatch.setattr(sys, 'argv', ['soma_primitives'] + argv)
    try:
        cli.main()
    except SystemExit as exc:
        return exc.code, handlers
    return 0, handlers


def test_ctrl_c_mid_wave_still_publishes_the_way_home(monkeypatch):
    # Sleep 4 is the move to wave_raise_2, the top of the raise: without
    # rclpy's handlers the context survives the Ctrl-C, so the undo and
    # the way down still go out, and the run ends as an unwound one.
    ros = FakeROS()
    code, handlers = run_main(monkeypatch, ros, ['wave'], ctrl_c_in_sleep={4})
    assert code == 3
    assert ros.init_kwargs == {'signal_handler_options': 'NO'}
    steps = plan('wave')
    assert ros.published[4:] == [steps[2].state, steps[1].state, HOME]
    assert ros.shut_down
    # A hang-up (the SSH session dropping) or a kill takes the same road.
    assert set(handlers) == {signal.SIGTERM, signal.SIGHUP}
    for sig, handler in handlers.items():
        with pytest.raises(KeyboardInterrupt):
            handler(sig, None)


def test_a_second_ctrl_c_stops_the_unwind_and_says_so(monkeypatch, capsys):
    # Cut in the move to wave_raise_2 and again while it is undone.
    ros = FakeROS()
    code, _ = run_main(monkeypatch, ros, ['wave', '--step'],
                       ctrl_c_in_sleep={4, 5}, answers=['', '', ''])
    assert code == 130
    assert ros.published[-1] == plan('wave')[2].state   # wave_raise_1
    assert ros.shut_down
    out = capsys.readouterr().out
    assert 'interrupted during the unwind' in out
    assert 'docs/session_v02.md step 7' in out


def test_ctrl_c_during_a_single_pose_says_the_pose_was_already_sent(monkeypatch, capsys):
    ros = FakeROS()
    code, _ = run_main(monkeypatch, ros, ['wave_probe'], ctrl_c_in_sleep={1})
    assert code == 130
    assert len(ros.published) == 1   # nothing to unwind: one pose, commanded
    assert 'wave_probe was already commanded' in capsys.readouterr().out
    assert ros.shut_down


def test_cli_imports_and_parses_the_step_flag():
    pytest.importorskip('rclpy')
    from soma_driver import primitives_cli
    assert primitives_cli.parse_cli(['wave', '--step']) == ('wave', True)
    assert callable(primitives_cli.main)
    assert 'never arms' in primitives_cli.__doc__.lower() or 'NEVER arms' in primitives_cli.__doc__


def test_real_rclpy_starts_without_its_signal_handlers(monkeypatch):
    """On Humble, rclpy's SIGINT handler shuts the context down; with it,
    a Ctrl-C in the middle of the wave could not publish the way home."""
    pytest.importorskip('rclpy')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from soma_driver import primitives_cli
    seen = {}
    monkeypatch.setattr(rclpy, 'init', lambda **kwargs: seen.update(kwargs))
    primitives_cli.init_ros()
    assert seen == {'signal_handler_options': SignalHandlerOptions.NO}


def test_real_rclpy_still_publishes_after_a_ctrl_c():
    """What the unwind stands on, on the real Humble rclpy: after a SIGINT
    the context is alive and a publisher still publishes. With rclpy's
    own handler, a thread shuts the context down right after the signal
    and that publish raises. The topic is private: no driver hears it."""
    pytest.importorskip('rclpy')
    import rclpy
    from sensor_msgs.msg import JointState
    from soma_driver import primitives_cli
    if signal.getsignal(signal.SIGINT) is not signal.default_int_handler:
        pytest.skip('this runner does not leave SIGINT to Python')
    primitives_cli.init_ros()
    try:
        node = rclpy.create_node('soma_primitives_sigint_check')
        pub = node.create_publisher(JointState, 'soma_test/sigint_check', 10)
        with pytest.raises(KeyboardInterrupt):
            signal.raise_signal(signal.SIGINT)
        time.sleep(0.2)   # long enough for rclpy's shutdown thread, if any
        assert rclpy.ok()
        pub.publish(JointState())
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
