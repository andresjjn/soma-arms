"""PCA9685 backends: mock (simulation and logging) and real (I2C).

THE GOLDEN RULE OF THIS PROJECT, encoded here:
  "NEVER move a motor without explicit prior confirmation from Andres."

The real backend refuses to exist unless it was explicitly armed. The mock
is always safe: it only records what would have been sent. The node starts
on the mock and swaps to the real backend only through the arm service.
"""
import time
from abc import ABC, abstractmethod

from .servo_map import PCA9685_FREQ_HZ, ServoSpec


def retry_i2c(fn, tries: int = 3, wait_s: float = 0.005):
    """Retry an I2C transaction on a transient OSError.

    Observed on hardware (2026-07-22): the inrush current of a servo can
    bounce the ground rail and corrupt ONE transaction (Errno 121) without
    resetting the chip. Retrying a few milliseconds later succeeds.
    """
    for attempt in range(tries):
        try:
            return fn()
        except OSError:
            if attempt == tries - 1:
                raise
            time.sleep(wait_s)


class Pca9685Backend(ABC):
    @abstractmethod
    def write(self, spec: ServoSpec, position: float) -> float:
        """Write a position (rad or m). Returns the pulse width applied, in us."""

    @abstractmethod
    def disable_all(self) -> None:
        """Cut the signal on ALL channels (servos go limp)."""

    @abstractmethod
    def release(self, channel: int) -> None:
        """Cut the signal on ONE channel (for self locking joints: the L16)."""


class MockPca9685(Pca9685Backend):
    """Simulator: records the pulses that WOULD have been sent."""

    is_real = False

    def __init__(self) -> None:
        self.last_us: dict[int, float] = {}
        self.write_count = 0
        self.enabled = True
        self.released: set[int] = set()

    def write(self, spec: ServoSpec, position: float) -> float:
        us = spec.command_to_us(position)
        self.last_us[spec.channel] = us
        self.write_count += 1
        self.released.discard(spec.channel)
        return us

    def disable_all(self) -> None:
        self.enabled = False
        self.last_us.clear()

    def release(self, channel: int) -> None:
        self.released.add(channel)
        self.last_us.pop(channel, None)


class RealPca9685(Pca9685Backend):
    """Real hardware over I2C (adafruit-circuitpython-pca9685).

    Only ever instantiated with explicit arming and hardware present.
    The import is lazy: simulating does not require the library, so the
    unit tests and CI run on any laptop.
    """

    is_real = True

    def __init__(self, i2c_address: int = 0x40, armed: bool = False) -> None:
        if not armed:
            raise PermissionError(
                'GOLDEN RULE: the real backend requires explicit arming '
                '(the arm service). Use the mock to simulate.')
        from adafruit_pca9685 import PCA9685  # lazy import
        import board
        import busio
        self._pca = PCA9685(busio.I2C(board.SCL, board.SDA), address=i2c_address)
        self._pca.frequency = int(PCA9685_FREQ_HZ)

    def write(self, spec: ServoSpec, position: float) -> float:
        us = spec.command_to_us(position)
        # the adafruit driver takes a 16 bit duty cycle
        duty16 = int(us / (1_000_000.0 / PCA9685_FREQ_HZ) * 0xFFFF)

        def _tx():
            self._pca.channels[spec.channel].duty_cycle = duty16

        retry_i2c(_tx)
        return us

    def disable_all(self) -> None:
        for ch in self._pca.channels:
            retry_i2c(lambda c=ch: setattr(c, 'duty_cycle', 0))

    def release(self, channel: int) -> None:
        retry_i2c(lambda: setattr(self._pca.channels[channel], 'duty_cycle', 0))
