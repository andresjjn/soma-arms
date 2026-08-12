"""The INA3221 driver's paper contract: conversions, addresses, the floor.

Pure tests against the register math and the mock; no I2C, no hardware.
The interesting failure modes of a power monitor are silent unit errors
(mV vs V, sign of discharge current) and a wrong default address that
would collide with the PCA9685: all pinned here.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.ina3221 import (  # noqa: E402
    BATTERY_FLOOR_V, CHANNELS, DEFAULT_ADDRESS, SHUNT_OHMS,
    MockIna3221, _twos13)


class TestRegisterMath:
    def test_positive_bus_voltage_conversion(self):
        # 20.0 V = 2500 counts of 8 mV, left-justified by 3 bits.
        raw = 2500 << 3
        assert _twos13(raw) * 0.008 == pytest.approx(20.0)

    def test_negative_shunt_reading_is_signed(self):
        # Current flowing "backwards" (charge direction) must come out
        # negative, not as a huge positive number.
        raw = (-100 & 0x1FFF) << 3
        assert _twos13(raw) == -100

    def test_full_scale_current_with_stock_shunt(self):
        # 12-bit + sign at 40 uV over 0.1 ohm: about +/-1.638 A. This is
        # the honest range limit recorded in INVENTARIO.md.
        max_counts = 0x0FFF
        amps = max_counts * 0.000040 / SHUNT_OHMS
        assert amps == pytest.approx(1.638, abs=0.001)


class TestAddressPlan:
    def test_default_address_avoids_both_pca9685_boards(self):
        # Edited 2026-08-11 because the physical fact changed: the
        # INA3221 A0 pin offers only 0x40-0x43 (datasheet SBOS576);
        # the 0x44/45 options recorded earlier belonged to the rover's
        # INA219, a different chip. Verified live on Jetson i2c-7:
        # INA at 0x41 answers 0x5449 ('TI') and die ID 0x3220; PCA #2
        # sits at 0x43 (A0+A1 bridged); PCA #1 keeps 0x40.
        assert DEFAULT_ADDRESS == 0x41
        assert DEFAULT_ADDRESS not in (0x40, 0x43)


class TestBatteryFloor:
    def test_floor_matches_the_strictest_guard_in_the_system(self):
        # Chemistry floor for the 5S DCB203 is 15.0 V; the 12 V dock
        # converter (B0FP1B1F86) cuts at 15.2 V. Software mirrors the
        # strictest guard so every power branch behaves identically.
        assert BATTERY_FLOOR_V == pytest.approx(15.2)
        assert BATTERY_FLOOR_V >= 5 * 3.0

    def test_mock_battery_ok_above_floor(self):
        ina = MockIna3221()
        ina.set_reading(1, voltage_v=19.8, current_a=0.9)
        assert ina.battery_ok()

    def test_mock_battery_not_ok_below_floor(self):
        ina = MockIna3221()
        ina.set_reading(1, voltage_v=14.9, current_a=0.4)
        assert not ina.battery_ok()

    def test_floor_boundary_is_inclusive(self):
        ina = MockIna3221()
        ina.set_reading(1, voltage_v=BATTERY_FLOOR_V, current_a=0.0)
        assert ina.battery_ok()


class TestMockMirrorsTheInterface:
    def test_read_all_covers_three_channels(self):
        ina = MockIna3221()
        data = ina.read_all()
        assert set(data) == set(CHANNELS)
        for values in data.values():
            assert set(values) == {'voltage_v', 'current_a'}

    def test_read_all_returns_copies(self):
        ina = MockIna3221()
        ina.read_all()[1]['voltage_v'] = 99.0
        assert ina.bus_voltage_v(1) == 0.0

    def test_invalid_channel_raises(self):
        ina = MockIna3221()
        with pytest.raises(ValueError):
            ina.bus_voltage_v(4)
        with pytest.raises(ValueError):
            ina.set_reading(0, 1.0, 1.0)

    def test_real_class_stays_import_clean_without_smbus2(self):
        # The lazy import pattern of the PCA9685 backend applies here
        # too: importing the module must never require smbus2.
        from soma_driver import ina3221
        assert hasattr(ina3221, 'Ina3221')
