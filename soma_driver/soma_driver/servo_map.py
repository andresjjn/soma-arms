"""Joint to PCA9685 channel map, and joint position to PWM pulse conversion.

PURE module (no ROS): runs under pytest on any machine, no hardware needed.

Sources of truth:
- Arm kit manual (Red Sun Global, 30 pages): servos are "25 kg, 180 deg,
  PWM 0.5 to 2.5 ms". We fit MG996R servos, same 25T spline and same
  0.5 to 2.5 ms band. Manual page 28 labels the six joints A to F.
- Actuonix L16-140-63-6-R: linear RC servo, nominally 1.0 ms retracted and
  2.0 ms extended over a 140 mm stroke. Electrically identical to a servo.
- MEASURED WIRING (2026-07-22, verified with the 6 V rail live): channels
  descend from 15. Right arm first, outermost joint first: gripper F,
  wrist roll E, "elbow 2" = wrist pitch D, "elbow 1" = elbow C,
  lift = shoulder B, rotation = base yaw A. Then the left arm in the same
  order (9 down to 4). Six independent servos per arm, no mirrored pairs.
  The board carries no silkscreen; the map below was confirmed channel by
  channel with power on, not read off a label.
- 2026-08-12: the left arm moved to board #2 (0x43) keeping its channel
  numbers, so the physical move was six connectors plus the address
  column below. Board #2 has no silkscreen either: the first armed
  session re-confirms channel by channel, same ritual as 2026-07-22.
- 2026-10-01, a lesson worth keeping: the harness was reported as "1 to
  12", the map followed, and the right arm went dead. Counted from the
  wrong end of an unlabeled header. A pulse sweep plus a spare servo,
  output by output, proved the physical truth: still 15 down to 10 on
  0x40 and 9 down to 4 on 0x43. Never trust a channel number that was
  not proven by moving the servo on it.

See docs/wiring.md for the full channel table and docs/hardware.md for
part numbers and datasheets.
"""
from dataclasses import dataclass

PCA9685_FREQ_HZ = 50.0  # servo standard: 20 ms period
PERIOD_US = 1_000_000.0 / PCA9685_FREQ_HZ


@dataclass(frozen=True)
class ServoSpec:
    """Specification of one servo channel."""

    channel: int
    min_us: float          # pulse at the joint lower limit
    max_us: float          # pulse at the joint upper limit
    lower: float           # joint lower limit (rad or m)
    upper: float           # joint upper limit (rad or m)
    max_rate: float        # max allowed rate (rad/s or m/s), safety ramp
    address: int = 0x40    # I2C board this channel lives on. Two boards
                           # since 2026-08-11; the left arm switched to
                           # 0x43 on 2026-08-12 (same channel numbers,
                           # different board). The default stays 0x40:
                           # the right arm and the L16 live there.

    def clamp(self, value: float) -> float:
        """Saturate a joint position to the soft limits of this channel."""
        return min(max(value, self.lower), self.upper)

    def command_to_us(self, value: float) -> float:
        """Joint position to pulse width, SATURATING at the soft limits."""
        v = self.clamp(value)
        frac = (v - self.lower) / (self.upper - self.lower)
        return self.min_us + frac * (self.max_us - self.min_us)

    def us_to_duty12(self, us: float) -> int:
        """Pulse width to PCA9685 12 bit count (0 to 4095)."""
        return round(us / PERIOD_US * 4095.0)


HALF_PI = 1.5707963267948966

# MEASURED CALIBRATION, 2026-10-01 (supersedes the 2026-08-03 capture).
# Andres re-captured, with scripts/servo_workbench.py on the two-board
# bench, the mechanical zero and both travel limits of every arm joint.
# Raw captures: calibration/servo_calibration_2026-10-01.json (the August
# ones stay next to it, calibration/servo_calibration_2026-08-03.json).
# Each MIN capture sits on the negative (hug) side and each MAX on the
# positive side; the inversion pattern (min_us > max_us) matches the
# August measurement joint by joint, checked when these rows were made.
#
# THE HUG CONVENTION (Andres's rule, and it is the sign convention of the
# whole project): negative = inward, as if the robot were closing a hug,
# grasping features facing forward; positive = outward/up. It is mirrored
# by construction, so "inward" is physically opposite between arms, which
# is exactly why some channels below have min_us > max_us: on those, more
# microseconds moves the joint inward. That inversion is measured, not a
# typo, same as the L16.
#
# Limits are the captured travel ends, as in August. Zeros are asymmetric
# on purpose (a shoulder needs far more travel up than down); the 25T
# spline only lands every 14.4 deg, so the electrical zero absorbs the
# residue. MG996R does roughly 6 rad/s; we cap at 2.5 rad/s by project
# rule: smooth motion, never snap moves.
#
# Changes worth knowing against August: the left yaw zero moved off its
# stop (2500 -> 2370 us): it can now rotate outward about 9 deg. The
# right yaw zero was not re-captured; it keeps the August 720 us.

SERVO_MAP: dict[str, ServoSpec] = {
    # Right arm on board 0x40: channels 15 down to 10, gripper first
    # (measured wiring, re-proven 2026-10-01).
    'right_arm_finger_l_joint':    ServoSpec(15, 860.0, 2185.0, 0.0, 1.0, 2.5),   # F: 860 closed, 2185 open
    'right_arm_wrist_roll_joint':  ServoSpec(14, 615.0, 2410.0, -1.7122, 1.1074, 2.5),   # E, zero 1705
    'right_arm_wrist_pitch_joint': ServoSpec(13, 760.0, 2270.0, -1.8850, 0.4869, 2.5),   # D, zero 1960
    'right_arm_elbow_joint':       ServoSpec(12, 2460.0, 535.0, -2.3955, 0.6283, 2.5),   # C, zero 935, inverted
    'right_arm_shoulder_joint':    ServoSpec(11, 710.0, 2480.0, -0.4398, 2.3405, 2.5),   # B, zero 990
    'right_arm_yaw_joint':         ServoSpec(10, 2425.0, 545.0, -2.6782, 0.2749, 2.5),   # A, zero 720, inverted (zero from 2026-08-03, not re-captured)
    # Left arm on board 0x43 (since 2026-08-12): channels 9 down to 4,
    # same order, mirrored signs. Pulses and limits are untouched on
    # purpose: they belong to the servos and their horns, not the board.
    'left_arm_finger_l_joint':    ServoSpec(9, 1080.0, 2130.0, 0.0, 1.0, 2.5, address=0x43),   # F: 1080 closed, 2130 open
    'left_arm_wrist_roll_joint':  ServoSpec(8, 2280.0, 580.0, -1.4687, 1.2017, 2.5, address=0x43),   # E, zero 1345, inverted
    'left_arm_wrist_pitch_joint': ServoSpec(7, 585.0, 2430.0, -1.6022, 1.2959, 2.5, address=0x43),   # D, zero 1605
    'left_arm_elbow_joint':       ServoSpec(6, 2455.0, 630.0, -1.9635, 0.9032, 2.5, address=0x43),   # C, zero 1205, inverted
    'left_arm_shoulder_joint':    ServoSpec(5, 770.0, 2460.0, -0.4791, 2.1756, 2.5, address=0x43),   # B, zero 1075
    'left_arm_yaw_joint':         ServoSpec(4, 530.0, 2475.0, -2.8903, 0.1649, 2.5, address=0x43),   # A, zero 2370
    # Torso: L16-140 on channel 3 (VERIFIED with power on, 2026-07-22).
    # Not on this bench: re-verify with power on when the torso returns.
    #
    # This unit has an INVERTED convention (measured, not from the
    # datasheet): 2000 us = retracted, 1000 us = extended. min_us > max_us
    # below is intentional and correct for this actuator.
    #
    # SOFT LIMITS 5 mm short of each mechanical stop. Holding a command
    # against a stop WEDGES the lead screw: it happened on 2026-07-22 and
    # the actuator had to be freed by hand. The physical mapping is still
    # 2000/1000 us = 0/140 mm; the anchors here are the pulse widths at
    # 5 mm and 135 mm so that saturation can never reach a hard stop.
    #
    # 0x40 spares: 1 and 2, and 0 (under suspicion, unconfirmed).
    'torso_lift_joint': ServoSpec(3, 1964.3, 1035.7, 0.005, 0.135, 0.020),
}

# Joints that RELEASE the PWM signal once settled on target (0.5 s).
# The L16 lead screw is self locking (it holds 46 N with no power), so
# holding PWM only heats the motor and can wedge it against a stop
# (this happened on 2026-07-22). Arm servos NEVER belong here: they need
# active torque or the arm falls under its own weight.
RELEASE_WHEN_SETTLED = {'torso_lift_joint'}
SETTLE_S = 0.5

# The right hand fingers are mimic joints (physical gear pair): they have
# no channel of their own and are never commanded.
MIMIC_JOINTS = {
    'left_arm_finger_r_joint':  ('left_arm_finger_l_joint', -1.0),
    'right_arm_finger_r_joint': ('right_arm_finger_l_joint', -1.0),
}


def rate_limit(current: float, target: float, max_rate: float, dt: float) -> float:
    """Move current toward target without exceeding max_rate (safety ramp)."""
    step = max_rate * dt
    delta = target - current
    if delta > step:
        return current + step
    if delta < -step:
        return current - step
    return target
