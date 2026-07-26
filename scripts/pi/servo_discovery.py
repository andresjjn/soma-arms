#!/usr/bin/env python3
"""Channel discovery/verification — MINIMUM MOTION version.

60 ms burst (3 PWM pulses): the servo twitches a few degrees and goes
silent. Repeatable with Enter. One channel at a time, the operator decides
everything. Tip: rest a finger on the servo body to feel which one vibrates.

Measured map 2026-07-22 (verified with power): right arm 15 down to 10
(gripper first), left arm 9 down to 4, L16 on channel 3 INVERTED
(2000 us = retracted). Mirrors the canonical copy in the Waver repo at
ROS2_Docker_twin/scripts/pi/.
"""
import time
from smbus2 import SMBus

ADDR, MODE1, PRESCALE, LED0, ALL_OFF_H = 0x40, 0x00, 0xFE, 0x06, 0xFD
BURST_S = 0.06

# (channel, name, target_us, burst_duration_s)
# Servos: center 1500 us, 60 ms -> a few degrees of twitch.
# L16 (linear, 20 mm/s, measured INVERTED): 2000 us = RETRACTED. If already
# retracted it does not move; otherwise it creeps at most 6 mm in 0.3 s.
# Listen for the gear motor.
EXPECTED = [
    (15, "right GRIPPER", 1500, 0.06), (14, "right WRIST ROLL", 1500, 0.06),
    (13, "right WRIST PITCH (elbow 2)", 1500, 0.06), (12, "right ELBOW (elbow 1)", 1500, 0.06),
    (11, "right SHOULDER (lift)", 1500, 0.06), (10, "right YAW (shoulder rotation)", 1500, 0.06),
    (9, "left GRIPPER", 1500, 0.06), (8, "left WRIST ROLL", 1500, 0.06),
    (7, "left WRIST PITCH (elbow 2)", 1500, 0.06), (6, "left ELBOW (elbow 1)", 1500, 0.06),
    (5, "left SHOULDER (lift)", 1500, 0.06), (4, "left YAW (shoulder rotation)", 1500, 0.06),
    (3, "L16 TORSO (linear, listen for the gear motor)", 2000, 0.30),
]


def burst(bus, ch, us=1500, secs=BURST_S):
    c = round(us / 20000.0 * 4096.0)
    base = LED0 + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, c & 0xFF, c >> 8])
    time.sleep(secs)
    bus.write_i2c_block_data(ADDR, base, [0, 0, 0, 0x10])


with SMBus(1) as bus:
    bus.write_byte_data(ADDR, MODE1, 0x10)
    bus.write_byte_data(ADDR, PRESCALE, 121)
    bus.write_byte_data(ADDR, MODE1, 0x20)
    time.sleep(0.01)
    bus.write_byte_data(ADDR, ALL_OFF_H, 0x10)
    print("PCA9685 at 50 Hz, everything off.")
    print("Short burst: a twitch, then the servo goes LOOSE.\n")

    results = []
    for ch, name, us, secs in EXPECTED:
        print(f"\nch{ch:2d} — expected: {name}")
        res = None
        while res is None:
            r = input(f"  [Enter]=burst {int(secs*1000)}ms  y=matched  n=was another  x=skip  q=quit > ").strip().lower()
            if r == "":
                burst(bus, ch, us, secs)
            elif r == "y":
                res = "OK"
            elif r == "n":
                res = "REAL: " + input("  which one moved? > ").strip()
            elif r == "x":
                res = "skipped"
            elif r == "q":
                res = "quit"
        if res == "quit":
            break
        results.append((ch, name, res))

    bus.write_byte_data(ADDR, ALL_OFF_H, 0x10)
    print("\n=== SUMMARY (everything off again) ===")
    bad = 0
    for ch, name, res in results:
        mark = "  OK " if res == "OK" else " ?? "
        if res not in ("OK", "skipped"):
            bad += 1
        print(f"{mark} ch{ch:2d} {name} -> {res}")
    print(f"\n{bad} discrepancies." + (" The map is correct." if bad == 0 else " Fix servo_map."))
