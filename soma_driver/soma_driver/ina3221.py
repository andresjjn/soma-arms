"""INA3221 triple-channel power monitor, register level over smbus2.

The eyes of the power system: three channels of bus voltage and shunt
current on the same I2C bus as the PCA9685. Planned wiring (see
INVENTARIO.md in the Waver repo, verified 2026-08-10):

  channel 1  battery side, upstream of the UBEC (pack voltage, total A)
  channel 2  reserved: 6 V servo rail (needs a lower-value shunt first)
  channel 3  reserved: 12 V Jetson rail

Unlike the PCA9685 this device is READ ONLY: no register here can move
metal, so it carries no arming gates. What it does carry is the battery
floor of the drill packs (DCB203, 5S: 3.0 V per cell = 15.0 V) as a
queryable fact, so the driver can refuse to arm on a dying battery
instead of browning out mid-motion.

Stock shunts are 0.1 ohm (full scale about +/-1.6 A per channel). On the
battery side of the UBEC that covers the whole normal envelope; sustained
readings pinned at full scale mean the real current is beyond range, not
equal to it.
"""
from __future__ import annotations

import glob

# Registers (datasheet SBOS576)
_REG_CONFIG = 0x00
_REG_SHUNT_BASE = 0x01   # ch1 0x01, ch2 0x03, ch3 0x05
_REG_BUS_BASE = 0x02     # ch1 0x02, ch2 0x04, ch3 0x06
_REG_MANUFACTURER_ID = 0xFE
_REG_DIE_ID = 0xFF

_MANUFACTURER_TI = 0x5449   # 'TI'
_DIE_INA3221 = 0x3220

# Conversion constants, straight from the datasheet: both registers are
# left-justified 13-bit two's complement values (shift right by 3).
_BUS_LSB_V = 0.008       # 8 mV per count
_SHUNT_LSB_V = 0.000040  # 40 uV per count

SHUNT_OHMS = 0.1
DEFAULT_ADDRESS = 0x41   # BENCH FACT 2026-08-11: the A0 pin offers only
                         # 0x40-0x43 (datasheet SBOS576); the 0x44/45
                         # idea belonged to the rover's INA219, a
                         # different chip. Pad adjacency on the breakout
                         # set the final map: INA at 0x41 (A0 bridged to
                         # VS), PCA #2 moved to 0x43 (A0+A1 closed).

# Chemistry floor for a DCB203 5S pack is 3.0 V/cell = 15.0 V. The 12 V
# dock converter (Amazon B0FP1B1F86) cuts at 15.2 V, so software adopts
# the strictest guard in the system and every branch behaves the same.
BATTERY_FLOOR_V = 15.2

# Arming floors per battery chemistry, keyed by the node's
# battery_source parameter. 'dewalt_5s' mirrors the dock's UVLO so
# software fires before the hardware cut; 'lipo_2s' is a conservative
# 3.2 V per cell for the bench pack. The node treats a floor as a VETO
# on arming, never as something that can arm ('none' means no monitor
# and is deliberately not a key here).
BATTERY_PROFILES = {
    'dewalt_5s': BATTERY_FLOOR_V,   # 15.2 V, the strictest guard
    'lipo_2s': 6.4,                 # 2 cells x 3.2 V
}

CHANNELS = (1, 2, 3)


def _twos13(raw16: int) -> int:
    """Left-justified 13-bit two's complement register to signed counts."""
    value = raw16 >> 3
    if value & 0x1000:
        value -= 0x2000
    return value


class Ina3221:
    """Real INA3221 over smbus2. Read-only by nature, no gates needed."""

    def __init__(self, address: int = DEFAULT_ADDRESS,
                 bus: int | None = None) -> None:
        from smbus2 import SMBus  # lazy: keeps pure tests import-clean
        self.address = address
        if bus is None:
            bus = self._find_bus(address)
        self._bus = SMBus(bus)
        self.bus_number = bus
        ident = self._read_word(_REG_MANUFACTURER_ID)
        if ident != _MANUFACTURER_TI:
            raise RuntimeError(
                f'device at 0x{address:02x} on i2c-{bus} answered '
                f'0x{ident:04x}, not an INA3221 (expected 0x5449)')

    @staticmethod
    def _find_bus(address: int) -> int:
        """Probe /dev/i2c-* for a chip that answers the TI signature."""
        from smbus2 import SMBus
        candidates = sorted(
            int(path.rsplit('-', 1)[1]) for path in glob.glob('/dev/i2c-*'))
        for number in candidates:
            try:
                with SMBus(number) as bus:
                    raw = bus.read_word_data(address, _REG_MANUFACTURER_ID)
                    swapped = ((raw & 0xFF) << 8) | (raw >> 8)
                    if swapped == _MANUFACTURER_TI:
                        return number
            except OSError:
                continue
        raise RuntimeError(
            f'no INA3221 found at 0x{address:02x} on any /dev/i2c-* bus')

    def _read_word(self, register: int) -> int:
        # SMBus reads words little-endian; the INA3221 talks big-endian.
        raw = self._bus.read_word_data(self.address, register)
        return ((raw & 0xFF) << 8) | (raw >> 8)

    def bus_voltage_v(self, channel: int) -> float:
        self._check_channel(channel)
        raw = self._read_word(_REG_BUS_BASE + 2 * (channel - 1))
        return _twos13(raw) * _BUS_LSB_V

    def shunt_current_a(self, channel: int) -> float:
        self._check_channel(channel)
        raw = self._read_word(_REG_SHUNT_BASE + 2 * (channel - 1))
        return _twos13(raw) * _SHUNT_LSB_V / SHUNT_OHMS

    def read_all(self) -> dict[int, dict[str, float]]:
        return {
            ch: {'voltage_v': self.bus_voltage_v(ch),
                 'current_a': self.shunt_current_a(ch)}
            for ch in CHANNELS
        }

    def battery_ok(self, channel: int = 1,
                   floor_v: float = BATTERY_FLOOR_V) -> bool:
        """True while the pack on `channel` sits above the floor."""
        return self.bus_voltage_v(channel) >= floor_v

    @staticmethod
    def _check_channel(channel: int) -> None:
        if channel not in CHANNELS:
            raise ValueError(f'INA3221 channels are 1..3, got {channel}')

    def close(self) -> None:
        self._bus.close()


class MockIna3221:
    """Test double with settable readings, mirroring the real interface."""

    def __init__(self, address: int = DEFAULT_ADDRESS,
                 bus: int | None = None) -> None:
        self.address = address
        self.bus_number = bus if bus is not None else 7
        self.readings = {ch: {'voltage_v': 0.0, 'current_a': 0.0}
                         for ch in CHANNELS}

    def set_reading(self, channel: int, voltage_v: float,
                    current_a: float) -> None:
        Ina3221._check_channel(channel)
        self.readings[channel] = {'voltage_v': voltage_v,
                                  'current_a': current_a}

    def bus_voltage_v(self, channel: int) -> float:
        Ina3221._check_channel(channel)
        return self.readings[channel]['voltage_v']

    def shunt_current_a(self, channel: int) -> float:
        Ina3221._check_channel(channel)
        return self.readings[channel]['current_a']

    def read_all(self) -> dict[int, dict[str, float]]:
        return {ch: dict(values) for ch, values in self.readings.items()}

    def battery_ok(self, channel: int = 1,
                   floor_v: float = BATTERY_FLOOR_V) -> bool:
        return self.bus_voltage_v(channel) >= floor_v

    def close(self) -> None:
        pass
