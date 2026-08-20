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

# MEASURED CALIBRATION, 2026-08-03. Every horn was re-splined with its
# servo holding the electrical center, then the mechanical zero and both
# travel limits were captured per channel with scripts/servo_workbench.py.
# Raw captures: calibration/servo_calibration_2026-08-03.json.
#
# THE HUG CONVENTION (Andres's rule, and it is the sign convention of the
# whole project): negative = inward, as if the robot were closing a hug,
# grasping features facing forward; positive = outward/up. It is mirrored
# by construction, so "inward" is physically opposite between arms, which
# is exactly why some channels below have min_us > max_us: on those, more
# microseconds moves the joint inward. That inversion is measured, not a
# typo, same as the L16.
#
# Zeros are asymmetric on purpose (a shoulder needs far more travel up
# than down); the 25T spline only lands every 14.4 deg, so the electrical
# zero absorbs the residue. MG996R does roughly 6 rad/s; we cap at
# 2.5 rad/s by project rule: smooth motion, never snap moves.
#
# Open flags from the capture session (docs/ESTADO.md in the Waver repo):
#   - left yaw's zero sits AT its 2500 us end stop: it can hug 176 deg
#     but cannot rotate outward at all (upper = 0.0). Re-spline one tooth
#     pending; until then the model carries the truth.
#   - the grippers open different amounts (1490 us right vs 1030 us
#     left); physical check pending.

SERVO_MAP: dict[str, ServoSpec] = {
    # Right arm: channels 15 down to 10, gripper first (measured wiring).
    'right_arm_finger_l_joint':    ServoSpec(15, 850.0, 2340.0, 0.0, 1.0, 2.5),   # F: 850 closed, 2340 open
    'right_arm_wrist_roll_joint':  ServoSpec(14, 520.0, 2490.0, -1.8222, 1.2724, 2.5),   # E, zero 1680
    'right_arm_wrist_pitch_joint': ServoSpec(13, 660.0, 2500.0, -2.0420, 0.8482, 2.5),   # D, zero 1960
    'right_arm_elbow_joint':       ServoSpec(12, 2500.0, 520.0, -2.1363, 0.9739, 2.5),   # C, zero 1140, inverted
    'right_arm_shoulder_joint':    ServoSpec(11, 700.0, 2500.0, -0.4712, 2.3562, 2.5),   # B, zero 1000
    'right_arm_yaw_joint':         ServoSpec(10, 2500.0, 500.0, -2.7960, 0.3456, 2.5),   # A, zero 720, inverted
    # Left arm: channels 9 down to 4, same order, mirrored signs.
    # SWITCHED to board #2 (0x43) on 2026-08-12: same channel numbers,
    # different board. Pulses and limits are untouched on purpose: they
    # belong to the servos and their horns, not to the driver board.
    'left_arm_finger_l_joint':    ServoSpec(9, 1160.0, 2190.0, 0.0, 1.0, 2.5, address=0x43),    # F: 1160 closed, 2190 open
    'left_arm_wrist_roll_joint':  ServoSpec(8, 2500.0, 680.0, -1.5080, 1.3509, 2.5, address=0x43),    # E, zero 1540, inverted
    'left_arm_wrist_pitch_joint': ServoSpec(7, 770.0, 2500.0, -1.6179, 1.0996, 2.5, address=0x43),    # D, zero 1800
    'left_arm_elbow_joint':       ServoSpec(6, 2300.0, 800.0, -1.5708, 0.7854, 2.5, address=0x43),    # C, zero 1300, inverted
    'left_arm_shoulder_joint':    ServoSpec(5, 900.0, 2500.0, -0.3927, 2.1206, 2.5, address=0x43),    # B, zero 1150
    'left_arm_yaw_joint':         ServoSpec(4, 540.0, 2500.0, -3.0788, 0.0, 2.5, address=0x43),       # A, zero AT the stop
    # Torso: L16-140 on channel 3 (VERIFIED with power on, 2026-07-22).
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
    # Channels 0 to 2 are spare (channel 0 is under suspicion, unconfirmed).
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
