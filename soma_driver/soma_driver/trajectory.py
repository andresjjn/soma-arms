"""Minimum-jerk joint motion: organic profiles on top of the safety ramp.

PURE module (no ROS): runs under pytest on any machine.

Why this exists: the original driver walked every joint toward its target
at constant velocity. That is safe but it looks robotic, because velocity
has hard corners: infinite jerk at the start and at the stop, and a load
spike into the gearbox both times. Biological motion follows a bell shaped
velocity curve instead. The classic minimum-jerk polynomial (Flash and
Hogan, 1985) gives exactly that bell with a closed form quintic.

Safety model, unchanged from the linear ramp it replaces:
  - The per joint rate limit from SERVO_MAP is a HARD CEILING, enforced on
    every step no matter what the polynomial asks for. The trajectory is
    planned so its peak velocity equals that ceiling, so in normal motion
    the clamp never engages; it exists for the abnormal cases.
  - Positions are clamped by ServoSpec at the pulse conversion as before.
  - Retargeting mid-motion replans FROM the current position and velocity,
    so a new command never causes a discontinuity.

The peak velocity of a rest-to-rest minimum-jerk segment is
15/8 * |delta| / T, so choosing T = 15/8 * |delta| / v_max makes the peak
exactly v_max. With a non zero initial velocity the true peak can exceed
that slightly; the ceiling clamp absorbs the difference.
"""
from dataclasses import dataclass, field

# Peak velocity factor of a rest-to-rest minimum-jerk segment.
PEAK_FACTOR = 15.0 / 8.0
# Segments shorter than this are stretched to it: prevents degenerate
# sub-tick trajectories for tiny corrections.
MIN_DURATION_S = 0.2


def _quintic_coeffs(p0: float, v0: float, pf: float, duration: float):
    """Quintic with x(0)=p0, x'(0)=v0, x''(0)=0, x(T)=pf, x'(T)=0, x''(T)=0."""
    t = duration
    d = pf - p0
    a3 = (20.0 * d - 12.0 * v0 * t) / (2.0 * t ** 3)
    a4 = (-30.0 * d + 16.0 * v0 * t) / (2.0 * t ** 4)
    a5 = (12.0 * d - 6.0 * v0 * t) / (2.0 * t ** 5)
    return p0, v0, a3, a4, a5


@dataclass
class JointMotion:
    """One joint's motion state: plan on set_target, sample on step."""

    position: float
    max_rate: float
    velocity: float = 0.0
    _target: float = field(init=False)
    _coeffs: tuple = field(default=None, init=False)
    _t: float = field(default=0.0, init=False)
    _duration: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        self._target = self.position

    @property
    def target(self) -> float:
        return self._target

    @property
    def done(self) -> bool:
        return self._coeffs is None and self.position == self._target

    def set_target(self, target: float) -> None:
        """Plan a minimum-jerk segment from the CURRENT state to target."""
        self._target = target
        delta = target - self.position
        if delta == 0.0 and self.velocity == 0.0:
            self._coeffs = None
            return
        duration = max(MIN_DURATION_S,
                       PEAK_FACTOR * abs(delta) / self.max_rate)
        self._coeffs = _quintic_coeffs(
            self.position, self.velocity, target, duration)
        self._duration = duration
        self._t = 0.0

    def step(self, dt: float) -> float:
        """Advance dt seconds. Returns the new position, ceiling-clamped."""
        if self._coeffs is None:
            return self.position

        self._t += dt
        if self._t >= self._duration:
            desired = self._target
        else:
            p0, v0, a3, a4, a5 = self._coeffs
            t = self._t
            desired = (p0 + v0 * t + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5)

        # THE SAFETY CEILING: never move faster than max_rate, no matter
        # what the polynomial (or a bug in it) asks for.
        max_step = self.max_rate * dt
        step = desired - self.position
        if step > max_step:
            step = max_step
        elif step < -max_step:
            step = -max_step

        self.position += step
        self.velocity = step / dt if dt > 0.0 else 0.0

        if self._t >= self._duration and self.position == self._target:
            self._coeffs = None
            self.velocity = 0.0
        return self.position
