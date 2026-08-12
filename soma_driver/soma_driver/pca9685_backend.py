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
    def release(self, spec: ServoSpec) -> None:
        """Cut the signal on ONE output (for self locking joints: the L16).

        Takes the full spec, not a bare channel: with two boards on the
        bus (bench fact 2026-08-11) a channel number alone no longer
        names an output; the pair (address, channel) does.
        """


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

    def release(self, spec: ServoSpec) -> None:
        self.released.add(spec.channel)
        self.last_us.pop(spec.channel, None)


# PCA9685 registers (datasheet). Same register-level protocol the bench
# tools (scripts/servo_workbench.py, scripts/pi/*) already validated on
# the real chip, on both a Raspberry Pi (bus 1) and a Jetson Orin Nano
# (bus 7).
_MODE1, _PRESCALE = 0x00, 0xFE
_LED0_ON_L, _ALL_LED_OFF_H = 0x06, 0xFD
_PRESCALE_50HZ = 121          # 25 MHz / (4096 * (121+1)) = exactly 50.0 Hz
_FULL_OFF = 0x10


class RealPca9685(Pca9685Backend):
    """Real hardware over I2C, driven at register level with smbus2.

    Deliberately NOT the Adafruit/Blinka stack: smbus2 is a plain kernel
    ioctl wrapper that works identically on the Pi and on the Jetson
    (JetPack 7.2 included), with no platform detection layer to break.
    The import is lazy AND happens only after the arming check, so the
    unit tests and CI run on any laptop, and the golden rule fires before
    any hardware library is even loaded.

    If no bus number is given, every /dev/i2c-* is probed for a device
    answering at the address: the header pins are bus 1 on a Pi and bus 7
    on an Orin Nano, and autodetection removes that footgun.
    """

    is_real = True

    def __init__(self, i2c_address: int = 0x40, armed: bool = False,
                 bus: int | None = None) -> None:
        if not armed:
            raise PermissionError(
                'GOLDEN RULE: the real backend requires explicit arming '
                '(the arm service). Use the mock to simulate.')
        from smbus2 import SMBus  # lazy import, after the arming check
        self._addr = i2c_address
        self.bus_number = self._find_bus(SMBus, bus)
        self._bus = SMBus(self.bus_number)
        # 50 Hz: sleep, set prescale, wake with auto-increment, all off.
        self._bus.write_byte_data(self._addr, _MODE1, 0x10)
        self._bus.write_byte_data(self._addr, _PRESCALE, _PRESCALE_50HZ)
        self._bus.write_byte_data(self._addr, _MODE1, 0x20)
        time.sleep(0.01)
        self.disable_all()

    def _find_bus(self, smbus_cls, forced: int | None) -> int:
        import glob
        if forced is not None:
            return forced
        for path in sorted(glob.glob('/dev/i2c-*')):
            n = int(path.rsplit('-', 1)[1])
            try:
                with smbus_cls(n) as probe:
                    probe.read_byte_data(self._addr, _MODE1)
                return n
            except OSError:
                continue
        raise OSError(f'no PCA9685 at 0x{self._addr:02x} on any I2C bus')

    def write(self, spec: ServoSpec, position: float) -> float:
        us = spec.command_to_us(position)
        counts = round(us / (1_000_000.0 / PCA9685_FREQ_HZ) * 4096.0)
        base = _LED0_ON_L + 4 * spec.channel
        retry_i2c(lambda: self._bus.write_i2c_block_data(
            self._addr, base, [0, 0, counts & 0xFF, counts >> 8]))
        return us

    def disable_all(self) -> None:
        retry_i2c(lambda: self._bus.write_byte_data(
            self._addr, _ALL_LED_OFF_H, _FULL_OFF))

    def release(self, spec: ServoSpec) -> None:
        base = _LED0_ON_L + 4 * spec.channel
        retry_i2c(lambda: self._bus.write_i2c_block_data(
            self._addr, base, [0, 0, 0, _FULL_OFF]))


class Pca9685Fleet(Pca9685Backend):
    """One backend per PCA9685 board, routed by ServoSpec.address.

    Since 2026-08-11 the bench carries two boards on the same bus (0x40
    in service with all 12 servos, 0x43 standing by for the left arm
    switchover), so the fleet keeps the node's view unchanged: it looks
    like one backend and routes every write to the board its spec names.
    disable_all reaches every board; that is the point of a kill switch.
    """

    def __init__(self, boards: dict[int, Pca9685Backend]) -> None:
        if not boards:
            raise ValueError('a fleet needs at least one board')
        self.boards = boards

    @property
    def is_real(self) -> bool:
        return all(board.is_real for board in self.boards.values())

    def _board(self, spec: ServoSpec) -> Pca9685Backend:
        try:
            return self.boards[spec.address]
        except KeyError:
            raise KeyError(
                f'channel {spec.channel} maps to board '
                f'0x{spec.address:02x} but the fleet only carries '
                f'{[hex(a) for a in sorted(self.boards)]}') from None

    def write(self, spec: ServoSpec, position: float) -> float:
        return self._board(spec).write(spec, position)

    def disable_all(self) -> None:
        for board in self.boards.values():
            board.disable_all()

    def release(self, spec: ServoSpec) -> None:
        self._board(spec).release(spec)


def addresses_in(servo_map: dict[str, ServoSpec]) -> tuple[int, ...]:
    """Distinct board addresses used by a map, in a stable order."""
    return tuple(sorted({spec.address for spec in servo_map.values()}))


def fleet_from_map(servo_map: dict[str, ServoSpec], factory) -> Pca9685Fleet:
    """Build one board per distinct address in the map, all or none.

    If the factory fails on any board, every board already built gets
    disable_all() and the error propagates: a fleet is never half armed.
    """
    boards: dict[int, Pca9685Backend] = {}
    for address in addresses_in(servo_map):
        try:
            boards[address] = factory(address)
        except BaseException:
            for built in boards.values():
                built.disable_all()
            raise
    return Pca9685Fleet(boards)


def mock_fleet(servo_map: dict[str, ServoSpec]) -> Pca9685Fleet:
    """The boot backend: one MockPca9685 per board address in the map."""
    return fleet_from_map(servo_map, lambda address: MockPca9685())


def real_fleet(servo_map: dict[str, ServoSpec], armed: bool = False,
               bus: int | None = None) -> Pca9685Fleet:
    """Arm every board in the map, reusing the bus the first one found.

    armed=False reaches RealPca9685 and raises PermissionError before
    any hardware library loads: the golden rule holds board by board.
    """
    found_bus = bus

    def factory(address: int) -> Pca9685Backend:
        nonlocal found_bus
        board = RealPca9685(i2c_address=address, armed=armed, bus=found_bus)
        found_bus = board.bus_number
        return board

    return fleet_from_map(servo_map, factory)
