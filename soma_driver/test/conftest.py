"""Shared fixtures: a register-level fake of smbus2 and the PCA9685s on it.

Lets the suite build a RealPca9685 (armed=True on purpose, it is a fake)
and watch what it writes, with no hardware and no smbus2 installed.
Nothing here can reach a real bus: the fixture replaces the smbus2 module
itself for the duration of one test.
"""
import sys
import types

import pytest

MODE1, PRESCALE, ALL_LED_OFF_H = 0x00, 0xFE, 0xFD
# What both boards read back after the 3.3 V glitch of 2026-10-01, and
# the datasheet power-on defaults: asleep, prescale 0x1E (200 Hz).
POWER_ON = {MODE1: 0x11, PRESCALE: 0x1E}


class FakeChips:
    """Every PCA9685 on one fake bus, modelled at register level."""

    def __init__(self, addresses=(0x40, 0x43)):
        self.regs = {a: dict(POWER_ON) for a in addresses}
        self.silent: set[int] = set()

    def power_glitch(self, address):
        """The logic supply dropped for an instant: power-on defaults."""
        self.regs[address] = dict(POWER_ON)


class FakeSMBus:
    chips: FakeChips = None   # bound per test by the fixture

    def __init__(self, bus_number):
        self.bus_number = bus_number

    def _chip(self, addr):
        if addr in self.chips.silent or addr not in self.chips.regs:
            raise OSError(121, 'Remote I/O error')
        return self.chips.regs[addr]

    def read_byte_data(self, addr, reg):
        return self._chip(addr).get(reg, 0)

    def write_byte_data(self, addr, reg, value):
        regs = self._chip(addr)
        # Datasheet: PRESCALE only takes a write while the chip sleeps.
        if reg == PRESCALE and not regs[MODE1] & 0x10:
            return
        regs[reg] = value

    def write_i2c_block_data(self, addr, reg, data):
        regs = self._chip(addr)
        for i, value in enumerate(data):
            regs[reg + i] = value


@pytest.fixture
def chips(monkeypatch):
    """Two fake PCA9685 (0x40, 0x43) at power-on defaults, smbus2 faked."""
    fake = FakeChips()
    monkeypatch.setattr(FakeSMBus, 'chips', fake)
    monkeypatch.setitem(sys.modules, 'smbus2',
                        types.SimpleNamespace(SMBus=FakeSMBus))
    return fake
