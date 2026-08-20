"""The two-board fleet: routing, atomic arming, and the no-change pin.

Born with the bench fact of 2026-08-11: two PCA9685 boards live on the
same bus (0x40 in service, 0x43 standing by), so a channel number alone
no longer names an output; the pair (address, channel) does. These tests
pin three promises:

  1. THE MAP MATCHES THE BENCH: the switchover happened on 2026-08-12
     (left arm on 0x43 with its channel numbers unchanged; right arm
     and L16 stay on 0x40), and a spec built without an address still
     lands on 0x40.
  2. Routing is by spec.address, and boards never bleed into each other.
  3. Arming is all or none, and the golden rule holds board by board.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.pca9685_backend import (  # noqa: E402
    MockPca9685, Pca9685Fleet, addresses_in, fleet_from_map, mock_fleet,
    real_fleet)
from soma_driver.servo_map import SERVO_MAP, ServoSpec  # noqa: E402

# A synthetic two-board map: same joint shapes, one channel number on
# purpose repeated across boards to prove (address, channel) is the key.
TWO_BOARDS = {
    'right_gripper': ServoSpec(15, 850.0, 2340.0, 0.0, 1.0, 2.5),
    'left_gripper':  ServoSpec(15, 1160.0, 2190.0, 0.0, 1.0, 2.5,
                               address=0x43),
    'left_elbow':    ServoSpec(6, 2300.0, 800.0, -1.5708, 0.7854, 2.5,
                               address=0x43),
}


class TestMapMatchesTheBench:
    def test_spec_without_address_lands_on_0x40(self):
        spec = ServoSpec(9, 1000.0, 2000.0, 0.0, 1.0, 2.5)
        assert spec.address == 0x40

    def test_the_map_spans_exactly_the_two_bench_boards(self):
        # Edited 2026-08-12 because the physical fact changed, exactly
        # as the old pin demanded: the switchover happened. The left arm
        # rows moved to 0x43 (same channel numbers, six connectors
        # replugged); the right arm and the L16 stay on 0x40.
        assert addresses_in(SERVO_MAP) == (0x40, 0x43)
        for joint, spec in SERVO_MAP.items():
            expected = 0x43 if joint.startswith('left_') else 0x40
            assert spec.address == expected, joint

    def test_boot_fleet_of_the_real_map_has_both_boards(self):
        fleet = mock_fleet(SERVO_MAP)
        assert set(fleet.boards) == {0x40, 0x43}


class TestRouting:
    def test_one_mock_board_per_distinct_address(self):
        fleet = mock_fleet(TWO_BOARDS)
        assert set(fleet.boards) == {0x40, 0x43}
        assert all(isinstance(b, MockPca9685)
                   for b in fleet.boards.values())

    def test_write_reaches_only_the_board_the_spec_names(self):
        fleet = mock_fleet(TWO_BOARDS)
        fleet.write(TWO_BOARDS['left_elbow'], 0.0)
        assert 6 in fleet.boards[0x43].last_us
        assert 6 not in fleet.boards[0x40].last_us

    def test_same_channel_number_on_two_boards_does_not_collide(self):
        fleet = mock_fleet(TWO_BOARDS)
        us_right = fleet.write(TWO_BOARDS['right_gripper'], 0.0)
        us_left = fleet.write(TWO_BOARDS['left_gripper'], 0.0)
        assert fleet.boards[0x40].last_us[15] == pytest.approx(us_right)
        assert fleet.boards[0x43].last_us[15] == pytest.approx(us_left)
        assert us_right != us_left   # measured pulses differ per arm

    def test_release_routes_by_address(self):
        fleet = mock_fleet(TWO_BOARDS)
        fleet.write(TWO_BOARDS['left_gripper'], 0.5)
        fleet.release(TWO_BOARDS['left_gripper'])
        assert 15 in fleet.boards[0x43].released
        assert 15 not in fleet.boards[0x40].released

    def test_disable_all_reaches_every_board(self):
        fleet = mock_fleet(TWO_BOARDS)
        fleet.write(TWO_BOARDS['right_gripper'], 0.2)
        fleet.write(TWO_BOARDS['left_elbow'], 0.2)
        fleet.disable_all()
        assert all(board.enabled is False and board.last_us == {}
                   for board in fleet.boards.values())

    def test_unknown_address_fails_loudly(self):
        fleet = mock_fleet(TWO_BOARDS)
        stray = ServoSpec(3, 1000.0, 2000.0, 0.0, 1.0, 2.5, address=0x41)
        with pytest.raises(KeyError):
            fleet.write(stray, 0.0)

    def test_a_fleet_needs_at_least_one_board(self):
        with pytest.raises(ValueError):
            Pca9685Fleet({})
        with pytest.raises(ValueError):
            mock_fleet({})


class TestAtomicArming:
    def test_factory_failure_disables_the_boards_already_built(self):
        built = []

        def factory(address):
            if address == 0x43:
                raise RuntimeError('second board refused to arm')
            board = MockPca9685()
            built.append(board)
            return board

        with pytest.raises(RuntimeError):
            fleet_from_map(TWO_BOARDS, factory)
        # All or none: the board that DID come up was shut back down.
        assert len(built) == 1
        assert built[0].enabled is False

    def test_real_fleet_without_arming_is_impossible(self):
        # The golden rule fires per board, before any hardware library
        # is even imported, so this runs on any laptop.
        with pytest.raises(PermissionError):
            real_fleet(SERVO_MAP, armed=False)

    def test_mixed_fleet_does_not_claim_to_be_real(self):
        fleet = mock_fleet(TWO_BOARDS)
        assert fleet.is_real is False
