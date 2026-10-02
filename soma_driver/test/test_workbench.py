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


class TestMasterSlider:
    """group_targets: one value drives every joint toward its own limits."""

    ROWS = [
        {'key': '40:15', 'ch': 15, 'name': 'right_arm_finger_l_joint',
         'zero_us': 850, 'band': [850, 2340]},
        {'key': '43:6', 'ch': 6, 'name': 'left_arm_elbow_joint',
         'zero_us': 1300, 'band': [800, 2300]},
    ]
    CAL = {'15': {'zero': 860, 'min': 860, 'max': 2185},
           '6': {'zero': 1205, 'min': 2455, 'max': 630}}

    def test_center_is_every_zero(self):
        from servo_workbench import group_targets
        t = group_targets(self.ROWS, self.CAL, 0.0, 1.0, {'right', 'left'})
        assert t == {'40:15': (860, 860), '43:6': (1205, 1205)}

    def test_full_right_reaches_each_captured_max(self):
        from servo_workbench import group_targets
        t = group_targets(self.ROWS, self.CAL, 1.0, 1.0, {'right', 'left'})
        assert t['40:15'][1] == 2185 and t['43:6'][1] == 630

    def test_full_left_reaches_each_captured_min_even_if_min_us_is_higher(self):
        # The elbow's MIN is the larger pulse (inverted joint): direction
        # follows the capture labels, not the pulse arithmetic.
        from servo_workbench import group_targets
        t = group_targets(self.ROWS, self.CAL, -1.0, 1.0, {'right', 'left'})
        assert t['43:6'][1] == 2455
        assert t['40:15'][1] == 860   # gripper: zero == min, nothing to do

    def test_cap_limits_the_travel(self):
        from servo_workbench import group_targets
        t = group_targets(self.ROWS, self.CAL, 1.0, 0.25, {'right', 'left'})
        assert t['40:15'][1] == round(860 + 0.25 * (2185 - 860))

    def test_unchecked_arm_is_left_alone(self):
        from servo_workbench import group_targets
        t = group_targets(self.ROWS, self.CAL, 0.5, 1.0, {'left'})
        assert set(t) == {'43:6'}

    def test_missing_capture_falls_back_to_the_map(self):
        from servo_workbench import group_targets
        t = group_targets(self.ROWS, {}, 1.0, 1.0, {'right'})
        assert t['40:15'] == (850, 2340)
