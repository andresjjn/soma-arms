"""The calibration converter: workbench microseconds in, joint limits out."""
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))
sys.path.insert(0, str(REPO / 'soma_driver'))

from apply_calibration import US_PER_RAD, l16_m_to_us, l16_us_to_m, propose  # noqa: E402


def cal(ch, zero, lo, hi):
    return {str(ch): {'name': 'x', 'zero': zero, 'min': lo, 'max': hi}}


class TestArmJoints:
    def test_asymmetric_range_around_the_measured_zero(self):
        # elbow (ch 12): zero at 1400, travel 900 to 2300 -> more travel up
        rows = propose(cal(12, 1400, 900, 2300))
        r = rows['right_arm_elbow_joint']
        assert r['lower'] == pytest.approx((900 - 1400) / US_PER_RAD, abs=1e-3)
        assert r['upper'] == pytest.approx((2300 - 1400) / US_PER_RAD, abs=1e-3)
        assert r['lower_deg'] == pytest.approx(-45.0, abs=0.1)
        assert r['upper_deg'] == pytest.approx(81.0, abs=0.1)

    def test_pulse_anchors_follow_the_default_sign(self):
        rows = propose(cal(12, 1400, 900, 2300))
        r = rows['right_arm_elbow_joint']
        assert r['min_us'] == 900.0 and r['max_us'] == 2300.0

    def test_missing_capture_becomes_an_error_not_a_guess(self):
        rows = propose({'12': {'name': 'x', 'zero': 1400}})
        assert 'missing' in rows['right_arm_elbow_joint']['error']

    def test_unknown_channel_is_reported(self):
        rows = propose(cal(0, 1500, 1000, 2000))
        assert 'not in SERVO_MAP' in rows['channel 0']['error']


class TestGrippers:
    def test_zero_is_closed_and_far_limit_is_open(self):
        rows = propose(cal(15, 1450, 1400, 2400))
        r = rows['right_arm_finger_l_joint']
        assert (r['lower'], r['upper']) == (0.0, 1.0)
        assert r['min_us'] == 1450.0      # closed anchor = the zero
        assert r['max_us'] == 2400.0      # open = farther capture


class TestL16:
    def test_roundtrip_us_meters(self):
        assert l16_us_to_m(2000.0) == pytest.approx(0.0)
        assert l16_us_to_m(1000.0) == pytest.approx(0.140)
        assert l16_m_to_us(l16_us_to_m(1234.0)) == pytest.approx(1234.0)

    def test_measured_stops_get_the_5mm_margin(self):
        # stops measured at 1980 us (2.8 mm) and 1020 us (137.2 mm)
        rows = propose(cal(3, 1500, 1020, 1980))
        r = rows['torso_lift_joint']
        assert r['lower'] == pytest.approx(0.0028 + 0.005, abs=1e-4)
        assert r['upper'] == pytest.approx(0.1372 - 0.005, abs=1e-4)
        # anchors stay inverted: retracted end has MORE microseconds
        assert r['min_us'] > r['max_us']

    def test_l16_rate_is_preserved(self):
        rows = propose(cal(3, 1500, 1020, 1980))
        assert rows['torso_lift_joint']['max_rate'] == pytest.approx(0.020)
