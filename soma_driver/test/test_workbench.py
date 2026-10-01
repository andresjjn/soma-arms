"""The servo workbench reads its joint list from SERVO_MAP, never a copy.

Its old hand-copied channel table went stale when the harness changed on
2026-10-01; these tests pin that the page is derived from the map.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))
sys.path.insert(0, str(REPO / 'soma_driver'))

from servo_workbench import joint_rows  # noqa: E402
from soma_driver.servo_map import SERVO_MAP  # noqa: E402


def test_one_row_per_arm_joint_and_no_torso():
    rows = joint_rows()
    assert len(rows) == 12
    assert 'torso_lift_joint' not in {r['name'] for r in rows}


def test_rows_mirror_the_map_board_and_channel():
    for r in joint_rows():
        spec = SERVO_MAP[r['name']]
        assert (r['addr'], r['ch']) == (spec.address, spec.channel)


def test_output_keys_are_unique():
    keys = [r['key'] for r in joint_rows()]
    assert len(keys) == len(set(keys))


def test_slider_starts_at_the_calibrated_zero_inside_the_band():
    # The first pulse snaps the servo, so it must be the pose a hanging arm
    # already holds: the map zero, never the 1500 us servo center.
    for r in joint_rows():
        spec = SERVO_MAP[r['name']]
        assert r['zero_us'] == round(spec.command_to_us(spec.clamp(0.0)))
        assert r['band'][0] <= r['zero_us'] <= r['band'][1]
