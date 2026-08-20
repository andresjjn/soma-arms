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


#: Mechanical zero of every joint, in microseconds, as captured on
#: 2026-08-03 (fingers: the zero is CLOSED). Commanding 0.0 must land the
#: pulse exactly here: this is what the re-splining session bought.
MEASURED_ZERO_US = {
    'right_arm_finger_l_joint': 850, 'right_arm_wrist_roll_joint': 1680,
    'right_arm_wrist_pitch_joint': 1960, 'right_arm_elbow_joint': 1140,
    'right_arm_shoulder_joint': 1000, 'right_arm_yaw_joint': 720,
    'left_arm_finger_l_joint': 1160, 'left_arm_wrist_roll_joint': 1540,
    'left_arm_wrist_pitch_joint': 1800, 'left_arm_elbow_joint': 1300,
    'left_arm_shoulder_joint': 1150, 'left_arm_yaw_joint': 2500,
}


class TestMeasuredCalibration:
    """The 2026-08-03 capture session, as executable spec."""

    @pytest.mark.parametrize('joint', sorted(MEASURED_ZERO_US))
    def test_commanding_zero_lands_on_the_measured_zero(self, joint):
        us = SERVO_MAP[joint].command_to_us(0.0)
        assert us == pytest.approx(MEASURED_ZERO_US[joint], abs=0.5), joint

    def test_saturates_at_the_measured_stops(self):
        """Safety rule number one, now with real numbers: the left elbow
        physically stops at 800 us outward and 2300 us inward."""
        spec = SERVO_MAP['left_arm_elbow_joint']
        assert spec.command_to_us(math.pi) == pytest.approx(800.0)
        assert spec.command_to_us(-10.0) == pytest.approx(2300.0)

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

    def test_left_yaw_zero_sits_at_its_end_stop(self):
        """Known flag, encoded so it cannot be forgotten: the left yaw
        cannot rotate outward at all (upper == 0.0). Re-spline one tooth
        pending; when that happens, THIS test is the one to update."""
        spec = SERVO_MAP['left_arm_yaw_joint']
        assert spec.upper == 0.0
        assert spec.command_to_us(0.0) == pytest.approx(2500.0)


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
    def test_no_duplicate_channels(self):
        channels = [s.channel for s in SERVO_MAP.values()]
        assert len(channels) == len(set(channels))

    def test_13_pca9685_channels(self):
        """12 arm servos plus the L16 = 13 channels, they fit in 16."""
        assert len(SERVO_MAP) == 13
        assert all(0 <= s.channel <= 15 for s in SERVO_MAP.values())

    def test_measured_wiring_2026_07_22(self):
        """Contract with the physical wiring: right arm 15 down to 10
        (gripper first), left arm 9 down to 4, L16 on channel 3 (verified
        with power on; channel 0 is under suspicion and left spare)."""
        assert SERVO_MAP['right_arm_finger_l_joint'].channel == 15
        assert SERVO_MAP['right_arm_yaw_joint'].channel == 10
        assert SERVO_MAP['left_arm_finger_l_joint'].channel == 9
        assert SERVO_MAP['left_arm_yaw_joint'].channel == 4
        assert SERVO_MAP['torso_lift_joint'].channel == 3
        # "elbow 2" in the wiring is wrist_pitch in the URDF, next to the elbow
        assert (SERVO_MAP['right_arm_wrist_pitch_joint'].channel
                == SERVO_MAP['right_arm_elbow_joint'].channel + 1)

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
        assert us == pytest.approx(1150.0, abs=0.5)   # its measured zero
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
#: of 2026-08-03 (calibration/servo_calibration_2026-08-03.json). This is
#: the contract: a single wrong field here is a servo driven to the wrong
#: place. min_us > max_us means more microseconds moves the joint INWARD
#: on that channel: measured, mirrored, and intentional.
#:
#: joint -> (channel, min_us, max_us, lower, upper, max_rate, address)
#: The address column joined on 2026-08-12: the left arm switched to
#: board #2 (0x43) keeping its channel numbers, so a wrong board here
#: is now as much of a mis-drive as a wrong channel.
EXACT_SERVO_MAP = {
    'right_arm_finger_l_joint':    (15, 850.0, 2340.0, 0.0, 1.0, 2.5, 0x40),
    'right_arm_wrist_roll_joint':  (14, 520.0, 2490.0, -1.8222, 1.2724, 2.5, 0x40),
    'right_arm_wrist_pitch_joint': (13, 660.0, 2500.0, -2.0420, 0.8482, 2.5, 0x40),
    'right_arm_elbow_joint':       (12, 2500.0, 520.0, -2.1363, 0.9739, 2.5, 0x40),
    'right_arm_shoulder_joint':    (11, 700.0, 2500.0, -0.4712, 2.3562, 2.5, 0x40),
    'right_arm_yaw_joint':         (10, 2500.0, 500.0, -2.7960, 0.3456, 2.5, 0x40),
    'left_arm_finger_l_joint':     (9, 1160.0, 2190.0, 0.0, 1.0, 2.5, 0x43),
    'left_arm_wrist_roll_joint':   (8, 2500.0, 680.0, -1.5080, 1.3509, 2.5, 0x43),
    'left_arm_wrist_pitch_joint':  (7, 770.0, 2500.0, -1.6179, 1.0996, 2.5, 0x43),
    'left_arm_elbow_joint':        (6, 2300.0, 800.0, -1.5708, 0.7854, 2.5, 0x43),
    'left_arm_shoulder_joint':     (5, 900.0, 2500.0, -0.3927, 2.1206, 2.5, 0x43),
    'left_arm_yaw_joint':          (4, 540.0, 2500.0, -3.0788, 0.0, 2.5, 0x43),
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
        spec = SERVO_MAP['left_arm_elbow_joint']
        assert spec.clamp(math.pi) == pytest.approx(0.7854)
        assert spec.clamp(-math.pi) == pytest.approx(-1.5708)

    def test_clamp_and_command_to_us_agree(self):
        for name, spec in SERVO_MAP.items():
            for probe in (-10.0, 0.0, 10.0):
                assert (spec.command_to_us(probe)
                        == pytest.approx(spec.command_to_us(spec.clamp(probe)))), name
