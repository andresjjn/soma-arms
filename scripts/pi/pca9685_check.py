#!/usr/bin/env python3
"""PCA9685 validation WITHOUT power (runs on the Pi 5, native smbus2).

Programs 50 Hz, writes the safe pulse for every channel of the measured
map, reads it back from the chip, and leaves ALL outputs off (FULL_OFF).
With V+ disconnected no servo can move; when V+ is connected afterwards,
nothing jumps because every output stays off.

Measured map 2026-07-22 (verified with power): right arm 15 down to 10
(gripper first), left arm 9 down to 4, L16 on channel 3 with the INVERTED
convention (2000 us = retracted). Mirrors the canonical copy in the Waver
repo at ROS2_Docker_twin/scripts/pi/; history in Waver/cad/MEDIDAS.md.
"""
import time
from smbus2 import SMBus

ADDR = 0x40
MODE1, PRESCALE = 0x00, 0xFE
LED0_ON_L = 0x06
ALL_LED_OFF_H = 0xFD

# (channel, name, safe pulse: center for servos, RETRACTED for the L16)
MAP = [
    (15, "right gripper",      1500), (14, "right wrist roll",  1500),
    (13, "right wrist pitch",  1500), (12, "right elbow",       1500),
    (11, "right shoulder",     1500), (10, "right yaw",         1500),
    (9,  "left gripper",       1500), (8,  "left wrist roll",   1500),
    (7,  "left wrist pitch",   1500), (6,  "left elbow",        1500),
    (5,  "left shoulder",      1500), (4,  "left yaw",          1500),
    (3,  "L16 torso (2000=retracted)", 2000),
]


def us_to_counts(us):
    return round(us / 20000.0 * 4096.0)


with SMBus(1) as bus:
    bus.write_byte_data(ADDR, MODE1, 0x10)
    bus.write_byte_data(ADDR, PRESCALE, 121)
    bus.write_byte_data(ADDR, MODE1, 0x20)
    time.sleep(0.01)
    pre = bus.read_byte_data(ADDR, PRESCALE)
    freq = 25_000_000 / (4096 * (pre + 1))
    print(f"prescale={pre} -> {freq:.1f} Hz " + ("OK" if pre == 121 else "ERROR"))

    failures = 0
    for ch, name, us in MAP:
        counts = us_to_counts(us)
        base = LED0_ON_L + 4 * ch
        bus.write_i2c_block_data(ADDR, base, [0, 0, counts & 0xFF, counts >> 8])
        echo_raw = bus.read_i2c_block_data(ADDR, base, 4)
        echo = echo_raw[2] | (echo_raw[3] << 8)
        ok = echo == counts
        failures += 0 if ok else 1
        print(f"  ch{ch:2d} {name:26s} {us}us = {counts:3d} -> echo {echo:3d} " + ("OK" if ok else "ERROR"))

    bus.write_byte_data(ADDR, ALL_LED_OFF_H, 0x10)
    echo_off = bus.read_i2c_block_data(ADDR, LED0_ON_L + 4 * 15, 4)
    all_off = bool(echo_off[3] & 0x10)
    print("outputs off (FULL_OFF): " + ("OK" if all_off else "ERROR"))
    print("\nRESULT: " + ("ALL OK - safe to connect power"
          if failures == 0 and pre == 121 and all_off else f"{failures} failures - do NOT connect"))
