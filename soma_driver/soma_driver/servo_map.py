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


def _arm_servo(ch: int) -> ServoSpec:
    """180 deg servo: 500 to 2500 us over +/- 90 deg.

    MG996R does roughly 0.17 s/60 deg, about 6 rad/s. We cap at 2.5 rad/s
    on purpose: the project rule is smooth motion, never snap moves.
    """
    return ServoSpec(ch, 500.0, 2500.0, -HALF_PI, HALF_PI, 2.5)


def _gripper_servo(ch: int) -> ServoSpec:
    """Gripper: joint 0 to 1 rad mapped onto the upper half of the band.

    [calibrate] against the real gear gripper once it is assembled.
    """
    return ServoSpec(ch, 1500.0, 2500.0, 0.0, 1.0, 2.5)


SERVO_MAP: dict[str, ServoSpec] = {
    # Right arm: channels 15 down to 10, gripper first (measured wiring).
    'right_arm_finger_l_joint':    _gripper_servo(15),   # F, gripper
    'right_arm_wrist_roll_joint':  _arm_servo(14),       # E
    'right_arm_wrist_pitch_joint': _arm_servo(13),       # D, "elbow 2"
    'right_arm_elbow_joint':       _arm_servo(12),       # C, "elbow 1"
    'right_arm_shoulder_joint':    _arm_servo(11),       # B
    'right_arm_yaw_joint':         _arm_servo(10),       # A, base yaw
    # Left arm: channels 9 down to 4, same order.
    'left_arm_finger_l_joint':    _gripper_servo(9),     # F, gripper
    'left_arm_wrist_roll_joint':  _arm_servo(8),         # E
    'left_arm_wrist_pitch_joint': _arm_servo(7),         # D, "elbow 2"
    'left_arm_elbow_joint':       _arm_servo(6),         # C, "elbow 1"
    'left_arm_shoulder_joint':    _arm_servo(5),         # B
    'left_arm_yaw_joint':         _arm_servo(4),         # A, base yaw
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
