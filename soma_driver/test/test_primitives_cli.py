"""The ROS adapter of the player imports and wires the step flag.

Runs only where rclpy exists (the ROS Humble job); the pure logic is
covered by test_player.py everywhere. The skip is raised inside the test
on purpose: a module-level importorskip aborts collection on the old
pytest of the ROS container.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_cli_imports_and_parses_the_step_flag():
    pytest.importorskip('rclpy')
    from soma_driver import primitives_cli
    assert primitives_cli.parse_cli(['wave', '--step']) == ('wave', True)
    assert callable(primitives_cli.main)
    assert 'never arms' in primitives_cli.__doc__.lower() or 'NEVER arms' in primitives_cli.__doc__
