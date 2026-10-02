"""Tests for the joint to pulse map and the safety rules. No ROS required.

These began as the 24 tests carried over from the first bench driver. On
2026-08-03 the hardware facts CHANGED on purpose: every horn was re-splined
to a natural mechanical zero and the real travel limits were captured per
channel, so the tests that pinned the old assumed map (symmetric +/- 90 deg
around 1500 us) were rewritten to pin the measured one. That edit is
allowed by the project rule precisely because the hardware fact changed,
and this docstring is the required explanation.

Sign convention under test (the "hug rule"): negative = inward, as if the
robot were closing a hug; positive = outward/up. Mirrored by construction.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soma_driver.pca9685_backend import (  # noqa: E402
    MockPca9685, RealPca9685, retry_i2c)
from soma_driver.servo_map import (  # noqa: E402
    MIMIC_JOINTS, RELEASE_WHEN_SETTLED, SERVO_MAP, rate_limit)

HALF_PI = math.pi / 2


#: Mechanical zero of every joint, in microseconds, as re-captured on
#: 2026-10-01 (fingers: the zero is CLOSED; the right yaw zero was not
#: re-captured and keeps its 2026-08-03 value). Commanding 0.0 must land
#: the pulse exactly here. Edited 2026-10-01 because the physical fact
#: changed: the August zeros were superseded by a full re-capture.
MEASURED_ZERO_US = {
    'right_arm_finger_l_joint': 860, 'right_arm_wrist_roll_joint': 1705,
    'right_arm_wrist_pitch_joint': 1960, 'right_arm_elbow_joint': 935,
    'right_arm_shoulder_joint': 990, 'right_arm_yaw_joint': 720,
    'left_arm_finger_l_joint': 1080, 'left_arm_wrist_roll_joint': 1345,
    'left_arm_wrist_pitch_joint': 1605, 'left_arm_elbow_joint': 1205,
    'left_arm_shoulder_joint': 1075, 'left_arm_yaw_joint': 2370,
}


class TestMeasuredCalibration:
    """The calibration capture (2026-10-01, superseding 2026-08-03), as
    executable spec."""

    @pytest.mark.parametrize('joint', sorted(MEASURED_ZERO_US))
    def test_commanding_zero_lands_on_the_measured_zero(self, joint):
        us = SERVO_MAP[joint].command_to_us(0.0)
        assert us == pytest.approx(MEASURED_ZERO_US[joint], abs=0.5), joint

    def test_saturates_at_the_measured_stops(self):
        """Safety rule number one, with real numbers: the left elbow
        physically stops at 630 us outward and 2455 us inward (captured
        2026-10-01; it was 800/2300 in August)."""
        spec = SERVO_MAP['left_arm_elbow_joint']
        assert spec.command_to_us(math.pi) == pytest.approx(630.0)
        assert spec.command_to_us(-10.0) == pytest.approx(2455.0)

    def test_hug_convention_every_arm_joint_can_hug(self):
        """Negative = inward. Every arm joint must have inward travel."""
        for joint in MEASURED_ZERO_US:
            if 'finger' in joint:
                continue
            assert SERVO_MAP[joint].lower < 0.0, joint

    def test_shoulders_are_biased_up_on_both_arms(self):
        """The design decision of the re-splining session: a shoulder needs
        far more travel up than down toward the table."""
        for side in ('left', 'right'):
            spec = SERVO_MAP[f'{side}_arm_shoulder_joint']
            assert spec.upper > 4.0 * abs(spec.lower), side

    def test_left_yaw_recovered_outward_travel(self):
        """Replaces test_left_yaw_zero_sits_at_its_end_stop, as that test
        demanded when the flag was fixed: in August the left yaw zero sat
        AT its 2500 us stop (upper == 0.0, no outward travel). The
        2026-10-01 capture puts the zero at 2370 us, so the joint can
        rotate outward again (about 9 deg)."""
        spec = SERVO_MAP['left_arm_yaw_joint']
        assert spec.upper > 0.1
        assert spec.command_to_us(0.0) == pytest.approx(2370.0, abs=0.5)


class TestL16Torso:
    """Actuonix L16-140-63-6-R: 0 to 140 mm stroke. This unit runs INVERTED
    (verified with power on, 2026-07-22): 2.0 ms retracted, 1.0 ms extended.
    Soft limits sit 5 mm short of each stop: holding a command against a
    stop wedged the lead screw and it had to be freed by hand."""

    def test_retracted_saturates_at_soft_limit(self):
        # asking for 0.0 must stop 5 mm short (1964 us), never reach 2000
        spec = SERVO_MAP['torso_lift_joint']
        assert spec.command_to_us(0.0) == pytest.approx(1964.3, abs=0.1)

    def test_extended_saturates_at_soft_limit(self):
        spec = SERVO_MAP['torso_lift_joint']
        assert spec.command_to_us(0.14) == pytest.approx(1035.7, abs=0.1)

    def test_mid_stroke(self):
        spec = SERVO_MAP['torso_lift_joint']
        assert spec.command_to_us(0.07) == pytest.approx(1500.0)

    def test_max_rate_is_the_l16_datasheet_speed(self):
        # 20 mm/s from the datasheet (63:1 gearbox)
        assert SERVO_MAP['torso_lift_joint'].max_rate == pytest.approx(0.020)


class TestChannels:
    def test_no_duplicate_outputs(self):
        # Since the two-board bench an output is (board, channel), not a
        # bare channel number.
        outputs = [(s.address, s.channel) for s in SERVO_MAP.values()]
        assert len(outputs) == len(set(outputs))

    def test_13_pca9685_channels(self):
        """12 arm servos plus the L16 = 13 channels, they fit in 16."""
        assert len(SERVO_MAP) == 13
        assert all(0 <= s.channel <= 15 for s in SERVO_MAP.values())

    def test_harness_proven_2026_10_01(self):
        """Contract with the physical wiring, proven with a spare servo
        output by output on 2026-10-01 (after a header counted from the
        wrong end briefly put 1-12 in this map and killed the right arm).
        Gripper first, descending: right arm 15 to 10 on 0x40, left arm
        9 to 4 on 0x43. L16 on 3; channel 0 stays empty, under suspicion."""
        order = ('finger_l', 'wrist_roll', 'wrist_pitch', 'elbow',
                 'shoulder', 'yaw')
        for arm, first, board in (('right', 15, 0x40), ('left', 9, 0x43)):
            for i, joint in enumerate(order):
                spec = SERVO_MAP[f'{arm}_arm_{joint}_joint']
                assert (spec.address, spec.channel) == (board, first - i), joint
        assert (SERVO_MAP['torso_lift_joint'].address,
                SERVO_MAP['torso_lift_joint'].channel) == (0x40, 3)
        assert all(s.channel != 0 for s in SERVO_MAP.values())

    def test_mimic_joints_have_no_channel(self):
        """The right hand fingers are geared: they must own no PWM channel."""
        assert not (set(MIMIC_JOINTS) & set(SERVO_MAP))

    def test_duty_12_bits(self):
        spec = SERVO_MAP['left_arm_shoulder_joint']
        # 1500 us out of 20000 us maps to about 307 counts of 4095
        assert spec.us_to_duty12(1500.0) == 307


class TestSafetyRamp:
    def test_never_jumps_more_than_one_step(self):
        # from 0 toward 1 rad at 2.5 rad/s with dt=0.02, max 0.05 per tick
        assert rate_limit(0.0, 1.0, 2.5, 0.02) == pytest.approx(0.05)

    def test_lands_exactly_when_close(self):
        assert rate_limit(0.99, 1.0, 2.5, 0.02) == pytest.approx(1.0)

    def test_works_in_reverse(self):
        assert rate_limit(1.0, 0.0, 2.5, 0.02) == pytest.approx(0.95)

    def test_l16_takes_7s_for_full_stroke(self):
        """Integrate the whole ramp: 140 mm at 20 mm/s = 7.0 s (datasheet)."""
        pos, t, dt = 0.0, 0.0, 0.02
        spec = SERVO_MAP['torso_lift_joint']
        while pos < 0.14:
            pos = rate_limit(pos, 0.14, spec.max_rate, dt)
            t += dt
        assert t == pytest.approx(7.0, abs=0.05)


class TestGoldenRule:
    def test_real_backend_without_arming_is_impossible(self):
        """NEVER move a motor without explicit confirmation."""
        with pytest.raises(PermissionError):
            RealPca9685(armed=False)

    def test_mock_records_without_moving_anything(self):
        mock = MockPca9685()
        spec = SERVO_MAP['left_arm_shoulder_joint']
        us = mock.write(spec, 0.0)
        assert us == pytest.approx(1075.0, abs=0.5)   # its measured zero
        assert mock.last_us[spec.channel] == pytest.approx(us)

    def test_disable_all_clears_everything(self):
        mock = MockPca9685()
        mock.write(SERVO_MAP['torso_lift_joint'], 0.07)
        mock.disable_all()
        assert mock.last_us == {} and mock.enabled is False


class TestSelfLockingRelease:
    """The L16 drops its signal once settled; arm servos NEVER do."""

    def test_only_the_torso_releases(self):
        assert RELEASE_WHEN_SETTLED == {'torso_lift_joint'}

    def test_release_cuts_the_channel_and_write_revives_it(self):
        mock = MockPca9685()
        spec = SERVO_MAP['torso_lift_joint']
        mock.write(spec, 0.07)
        # release takes the spec since the two-board bench (2026-08-11):
        # a bare channel no longer names an output, (address, channel) does.
        mock.release(spec)
        assert spec.channel in mock.released
        assert spec.channel not in mock.last_us
        mock.write(spec, 0.10)
        assert spec.channel not in mock.released


class TestI2CRetry:
    """A transient Errno 121 (seen 2026-07-22) must not take the node down."""

    def test_recovers_after_a_transient_failure(self):
        attempts = []

        def tx():
            attempts.append(1)
            if len(attempts) < 2:
                raise OSError(121, 'Remote I/O error')
            return 'ok'

        assert retry_i2c(tx, wait_s=0.0) == 'ok'
        assert len(attempts) == 2

    def test_reraises_if_the_failure_persists(self):
        def tx():
            raise OSError(121, 'Remote I/O error')

        with pytest.raises(OSError):
            retry_i2c(tx, tries=3, wait_s=0.0)


#: The complete channel table, transcribed field by field from the capture
#: (re-captured 2026-10-01; generated from calibration/servo_calibration_
#: 2026-10-01.json with each joint's direction checked against August)
#: of 2026-08-03 (calibration/servo_calibration_2026-08-03.json). This is
#: the contract: a single wrong field here is a servo driven to the wrong
#: place. min_us > max_us means more microseconds moves the joint INWARD
#: on that channel: measured, mirrored, and intentional.
#:
#: joint -> (channel, min_us, max_us, lower, upper, max_rate, address)
#: The address column joined on 2026-08-12: the left arm switched to
#: board #2 (0x43) keeping its channel numbers, so a wrong board here
#: is now as much of a mis-drive as a wrong channel. (On 2026-10-01 the
#: numbers briefly read 1-12 after a header was counted from the wrong
#: end; a spare-servo test, output by output, restored these.)
EXACT_SERVO_MAP = {
    'right_arm_finger_l_joint':    (15, 860.0, 2185.0, 0.0, 1.0, 2.5, 0x40),
    'right_arm_wrist_roll_joint':  (14, 615.0, 2410.0, -1.7122, 1.1074, 2.5, 0x40),
    'right_arm_wrist_pitch_joint': (13, 760.0, 2270.0, -1.8850, 0.4869, 2.5, 0x40),
    'right_arm_elbow_joint':       (12, 2460.0, 535.0, -2.3955, 0.6283, 2.5, 0x40),
    'right_arm_shoulder_joint':    (11, 710.0, 2480.0, -0.4398, 2.3405, 2.5, 0x40),
    'right_arm_yaw_joint':         (10, 2425.0, 545.0, -2.6782, 0.2749, 2.5, 0x40),
    'left_arm_finger_l_joint':     (9, 1080.0, 2130.0, 0.0, 1.0, 2.5, 0x43),
    'left_arm_wrist_roll_joint':   (8, 2280.0, 580.0, -1.4687, 1.2017, 2.5, 0x43),
    'left_arm_wrist_pitch_joint':  (7, 585.0, 2430.0, -1.6022, 1.2959, 2.5, 0x43),
    'left_arm_elbow_joint':        (6, 2455.0, 630.0, -1.9635, 0.9032, 2.5, 0x43),
    'left_arm_shoulder_joint':     (5, 770.0, 2460.0, -0.4791, 2.1756, 2.5, 0x43),
    'left_arm_yaw_joint':          (4, 530.0, 2475.0, -2.8903, 0.1649, 2.5, 0x43),
    # The L16 keeps its 2026-07-22 anchors: its stops have not been
    # re-measured (that capture comes with the torso build).
    'torso_lift_joint':            (3, 1964.3, 1035.7, 0.005, 0.135, 0.020, 0x40),
}

#: Channels where more microseconds moves the joint inward (min_us >
#: max_us). The pattern is the mirror geometry itself: rolls and yaws
#: invert on opposite arms, elbows invert on both.
INVERTED_CHANNELS = {
    'torso_lift_joint',
    'right_arm_elbow_joint', 'right_arm_yaw_joint',
    'left_arm_wrist_roll_joint', 'left_arm_elbow_joint',
}


class TestExactServoMapTable:
    """Field by field check against the transcribed hardware table."""

    @pytest.mark.parametrize('joint', sorted(EXACT_SERVO_MAP))
    def test_row_matches(self, joint):
        (channel, min_us, max_us, lower, upper, max_rate,
         address) = EXACT_SERVO_MAP[joint]
        spec = SERVO_MAP[joint]
        assert spec.address == address, f'{joint}: board address'
        assert spec.channel == channel, f'{joint}: channel'
        assert spec.min_us == pytest.approx(min_us), f'{joint}: min_us'
        assert spec.max_us == pytest.approx(max_us), f'{joint}: max_us'
        assert spec.lower == pytest.approx(lower), f'{joint}: lower'
        assert spec.upper == pytest.approx(upper), f'{joint}: upper'
        assert spec.max_rate == pytest.approx(max_rate), f'{joint}: max_rate'

    def test_no_joint_was_added_or_dropped(self):
        assert set(SERVO_MAP) == set(EXACT_SERVO_MAP)

    def test_the_inversion_pattern_is_exactly_the_measured_one(self):
        """Guard against someone "fixing" min_us > max_us anywhere: the
        set of inverted channels is a measurement, and it is mirrored
        between arms exactly as the hug convention predicts."""
        for joint, spec in SERVO_MAP.items():
            if joint in INVERTED_CHANNELS:
                assert spec.min_us > spec.max_us, f'{joint} must be inverted'
            else:
                assert spec.min_us < spec.max_us, f'{joint} must be normal'


class TestSoftLimitClamp:
    """Added during the SOMA migration: the reported pose must never claim
    a position the driver is not allowed to command."""

    def test_clamp_holds_the_torso_inside_the_soft_band(self):
        spec = SERVO_MAP['torso_lift_joint']
        assert spec.clamp(0.0) == pytest.approx(0.005)
        assert spec.clamp(0.14) == pytest.approx(0.135)
        assert spec.clamp(0.07) == pytest.approx(0.07)

    def test_clamp_holds_arm_servos_inside_the_measured_range(self):
        # Limits re-captured 2026-10-01 (August: -1.5708 .. 0.7854).
        spec = SERVO_MAP['left_arm_elbow_joint']
        assert spec.clamp(math.pi) == pytest.approx(0.9032)
        assert spec.clamp(-math.pi) == pytest.approx(-1.9635)

    def test_clamp_and_command_to_us_agree(self):
        for name, spec in SERVO_MAP.items():
            for probe in (-10.0, 0.0, 10.0):
                assert (spec.command_to_us(probe)
                        == pytest.approx(spec.command_to_us(spec.clamp(probe)))), name
