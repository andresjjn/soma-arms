"""Board reset detection: a PCA9685 that silently came back asleep.

Born with the bench incident of 2026-10-01: both boards lost their shared
3.3 V logic supply for an instant and came back at power-on defaults
(MODE1 0x11, oscillator asleep; prescale 0x1E, 200 Hz instead of 121 for
50.0 Hz). Every output went dead while software believed it was driving
pulses. These tests pin four promises:

  1. The config check reads MODE1 and PRESCALE and judges exactly the
     SLEEP bit and the prescale, nothing else.
  2. The fleet names every faulty board, and a board that does not
     answer counts as faulty.
  3. A kill switch reaches every board even when one of them fails.
  4. Building a RealPca9685 again (what /soma/arm does) re-runs the init
     and clears a reset, and the golden rule still fires before smbus2.

No hardware, no rclpy: smbus2 is replaced by the register-level fake
in conftest.py.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.pca9685_backend import (  # noqa: E402
    MockPca9685, Pca9685Fleet, RealPca9685, describe_config, mock_fleet,
    real_fleet)
from soma_driver.servo_map import SERVO_MAP, ServoSpec  # noqa: E402

MODE1, PRESCALE = 0x00, 0xFE   # the fake bus lives in conftest.py

TWO_BOARDS = {
    'right_gripper': ServoSpec(15, 850.0, 2340.0, 0.0, 1.0, 2.5),
    'left_elbow':    ServoSpec(6, 2300.0, 800.0, -1.5708, 0.7854, 2.5,
                               address=0x43),
}


class TestConfigCheck:
    def test_armed_config_is_clean(self):
        assert describe_config(0x20, 121) is None

    def test_restart_and_allcall_bits_are_not_faults(self):
        # Only SLEEP stops the oscillator; RESTART and ALLCALL can read
        # back either way on a healthy, running chip.
        assert describe_config(0xA1, 121) is None

    def test_the_bench_power_on_defaults_are_a_fault(self):
        fault = describe_config(0x11, 0x1E)
        assert fault is not None
        assert 'SLEEP' in fault and '0x1e' in fault

    def test_sleep_alone_is_a_fault(self):
        assert describe_config(0x30, 121) is not None

    def test_wrong_prescale_alone_is_a_fault(self):
        assert describe_config(0x20, 0x1E) is not None


class TestMockReset:
    def test_a_fresh_mock_is_configured(self):
        assert MockPca9685().is_configured() is True

    def test_a_simulated_reset_shows_the_bench_registers(self):
        board = MockPca9685()
        board.simulate_reset()
        assert board.is_configured() is False
        assert board.config_fault() == describe_config(0x11, 0x1E)

    def test_a_simulated_silent_board_raises_like_the_bus(self):
        board = MockPca9685()
        board.simulate_reset(answering=False)
        with pytest.raises(OSError):
            board.config_fault()


class TestFleetCheck:
    def test_a_healthy_fleet_has_no_faults(self):
        fleet = mock_fleet(TWO_BOARDS)
        assert fleet.board_faults() == {}
        assert fleet.is_configured() is True

    def test_the_boot_fleet_of_the_real_map_is_configured(self):
        assert mock_fleet(SERVO_MAP).is_configured() is True

    def test_a_reset_board_is_named(self):
        fleet = mock_fleet(TWO_BOARDS)
        fleet.boards[0x43].simulate_reset()
        faults = fleet.board_faults()
        assert set(faults) == {0x43}
        assert fleet.is_configured() is False
        assert '0x43' in fleet.config_fault()

    def test_both_boards_reset_together_are_both_named(self):
        # The 2026-10-01 case: one shared logic supply, both boards hit.
        fleet = mock_fleet(TWO_BOARDS)
        for board in fleet.boards.values():
            board.simulate_reset()
        assert set(fleet.board_faults()) == {0x40, 0x43}

    def test_a_silent_board_is_a_fault_not_a_crash(self):
        fleet = mock_fleet(TWO_BOARDS)
        fleet.boards[0x40].simulate_reset(answering=False)
        faults = fleet.board_faults()
        assert set(faults) == {0x40}
        assert 'not answering' in faults[0x40]


class _FailingBoard(MockPca9685):
    def disable_all(self):
        raise OSError(121, 'Remote I/O error')


class TestKillSwitchReachesEveryBoard:
    def test_a_failing_board_does_not_shield_the_others(self):
        # Dict order puts 0x40 first: if it is the one off the bus, 0x43
        # must still get its cut, and the caller must still hear about it.
        live = MockPca9685()
        fleet = Pca9685Fleet({0x40: _FailingBoard(), 0x43: live})
        with pytest.raises(OSError):
            fleet.disable_all()
        assert live.enabled is False


class TestRealBoardOnAFakeBus:
    def test_init_leaves_the_board_configured(self, chips):
        board = RealPca9685(0x40, armed=True, bus=7)
        assert chips.regs[0x40][PRESCALE] == 121
        assert not chips.regs[0x40][MODE1] & 0x10
        assert board.is_configured() is True

    def test_a_power_glitch_is_detected(self, chips):
        board = RealPca9685(0x40, armed=True, bus=7)
        chips.power_glitch(0x40)
        assert board.is_configured() is False
        assert board.config_fault() == describe_config(0x11, 0x1E)

    def test_a_silent_board_raises(self, chips):
        board = RealPca9685(0x40, armed=True, bus=7)
        chips.silent.add(0x40)
        with pytest.raises(OSError):
            board.is_configured()

    def test_the_fleet_names_the_board_that_reset(self, chips):
        fleet = real_fleet(SERVO_MAP, armed=True, bus=7)
        assert fleet.is_real is True
        assert fleet.board_faults() == {}
        chips.power_glitch(0x43)
        assert set(fleet.board_faults()) == {0x43}

    def test_rearming_reruns_the_init_and_clears_the_reset(self, chips):
        # What /soma/arm does after a disarm: build the boards again.
        real_fleet(SERVO_MAP, armed=True, bus=7)
        chips.power_glitch(0x40)
        chips.power_glitch(0x43)
        fleet = real_fleet(SERVO_MAP, armed=True, bus=7)
        assert fleet.board_faults() == {}

    def test_golden_rule_fires_before_smbus2_even_with_a_bus(self, chips,
                                                            monkeypatch):
        # A module that explodes on import proves the order: the
        # PermissionError comes first, the hardware library never loads.
        monkeypatch.delitem(sys.modules, 'smbus2')
        monkeypatch.setattr('builtins.__import__', _no_smbus2(__import__))
        with pytest.raises(PermissionError):
            RealPca9685(0x40, armed=False, bus=7)
        with pytest.raises(PermissionError):
            real_fleet(SERVO_MAP, armed=False, bus=7)


def _no_smbus2(real_import):
    def guarded(name, *args, **kwargs):
        if name == 'smbus2':
            raise AssertionError('smbus2 imported before the arming check')
        return real_import(name, *args, **kwargs)
    return guarded
