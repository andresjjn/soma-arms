# SOMA wiring

## The channel map

**Proven output by output with a spare servo on 2026-10-01.** The
PCA9685 board has no silkscreen numbering, so this table is the only
authority. It is also locked in by
`test_harness_proven_2026_10_01` in `soma_driver/test/test_servo_map.py`:
change the wiring and that test goes red, which is exactly what should happen.

The pattern is: **gripper first, channels descending from 15**, one board
per arm. "Right" is the robot's own right.

| Board | Channel | Joint | Manual label | Actuator |
|---|---|---|---|---|
| 0x40 | 15 | `right_arm_finger_l_joint` | F | MG996R, gripper |
| 0x40 | 14 | `right_arm_wrist_roll_joint` | E | MG996R |
| 0x40 | 13 | `right_arm_wrist_pitch_joint` | D | MG996R |
| 0x40 | 12 | `right_arm_elbow_joint` | C | MG996R |
| 0x40 | 11 | `right_arm_shoulder_joint` | B | MG996R |
| 0x40 | 10 | `right_arm_yaw_joint` | A | MG996R, base |
| 0x43 | 9 | `left_arm_finger_l_joint` | F | MG996R, gripper |
| 0x43 | 8 | `left_arm_wrist_roll_joint` | E | MG996R |
| 0x43 | 7 | `left_arm_wrist_pitch_joint` | D | MG996R |
| 0x43 | 6 | `left_arm_elbow_joint` | C | MG996R |
| 0x43 | 5 | `left_arm_shoulder_joint` | B | MG996R |
| 0x43 | 4 | `left_arm_yaw_joint` | A | MG996R, base |
| 0x40 | 3 | `torso_lift_joint` | n/a | Actuonix L16-140-63-6-R, not on this bench |
| 0x40 | 0 | spare, **under suspicion** | | see below |

**Neither board has a channel silkscreen.** On 2026-10-01 a header counted
from the wrong end put "1 to 12" in the map and the right arm went dead with
power and signal both present at the header. The cure was empirical: a
spare servo moved output by output (`servo_workbench.py --raw`). Prove a
channel number by moving a servo on it, never by counting pins.

**Channel 0 is left empty on purpose.** The L16 was originally wired there.
When it went deaf, channel 0 was one of the suspects. The real cause turned
out to be a wedged lead screw, not the channel, but by then the actuator had
been moved to channel 3 and everything was verified there. Channel 0 was never
cleared, so it stays out of service until someone proves it good.

The right hand fingers (`*_finger_r_joint`) appear nowhere in this table.
They are a mechanical gear pair driven by the left finger servo, so they have
no channel. `test_mimic_joints_have_no_channel` enforces that.

## Power

Today, as assembled on 2026-09-01 and in use since:

```
  mains --> ATX supply (MaxiTech 300U, 10 A on +12 V)   <- rear switch = EMERGENCY STOP
              P4 +12 V --> 10 A fuse --> XT60 --> Y splitter --+--> UBEC 6 V --> V+ block of 0x40 --> right arm (6 servos)
                                                               +--> UBEC 6 V --> V+ block of 0x43 --> left arm (6 servos)

  Jetson Orin Nano Super: its OWN wall adapter (since 2026-09-01)
  PCA9685 logic (VCC 3.3 V) from the Jetson header, pin 1
  Jetson and servo rails share GROUND and nothing else
  INA3221 (0x41): off the bus since 2026-10-01
```

A star from the ATX, one UBEC per arm, separate returns, so a stall on
one arm cannot brown out the other. The L16 is off this bench; when the
torso returns it hangs off whichever rail is less loaded.

Measured, 2026-09-01: the ATX gives 11.6 V with nothing connected,
inside the ATX +-5 % band but below the 11.8 to 12.3 V we would have
liked. Watch it under load; below 11.4 V sustained, a load fan on 5 V
goes on. 6.00 V on both terminal blocks.

Non negotiable:

- **The ATX rear switch is the emergency stop**, and it stays in reach
  whenever V+ is up. It drops the 12 V, both UBECs and the 6 V rail and
  leaves the Jetson running. Never stop the arms by pulling the XT60 under
  load: see [safety.md](safety.md), "What the safety model does not cover".
- **Nothing but a UBEC feeds a servo.** The MG996R is rated to 7.2 V: not
  the 12 V side, not a battery straight on V+ (a full 2S LiPo, the July
  bench supply, is 8.4 V). Feed each PCA9685 green V+ terminal block from
  its UBEC with wire of **16 AWG or thicker**, and a capacitor across V+.
- **The Jetson never shares the servo supply.** On 2026-09-01 hot plugging
  the servo branch (the input capacitors of both UBECs plus twelve servos)
  sank the shared 12 V and hard-shut the Jetson. Its own adapter since,
  and **nothing is ever hot plugged onto the servo branch.**
- **Energise V+ only with both arms hanging at rest**, nothing under or
  around them: on this bench that IS the compact pose. MG996R clones
  twitch at power-up.
- The Jetson and the servos share **ground** and nothing else.

## Connectors, which is where the real failures came from

Every unexplained fault in the first session traced back to a connector, not
to code:

- A **dupont jumper broken inside its insulation** took the I2C bus down four
  times, the last time for good. Four new short cables fixed it.
- Loose servo connectors produced **"ghost" servos** twitching at power-up.

The permanent harness, and it is not optional once the arms leave the bench:

| Line | Connector |
|---|---|
| I2C | **JST with a positive lock.** No dupont jumpers |
| Servo signal | silicone retainer on the connector |
| V+ | **16 AWG or thicker**, screwed into the terminal block |

Diagnostic: run `i2cdetect` in a loop and tap the cables. If `0x40` blinks,
the fault is mechanical and no amount of retry logic will fix it.

## I2C

| | |
|---|---|
| Bus | Jetson Orin Nano, bus i2c-7, header pins 1 (3V3), 3 (SDA), 5 (SCL), 6 (GND). Pi era: bus 1 |
| Addresses | `0x40` PCA9685 #1 (right arm + L16) · `0x43` PCA9685 #2 (A0+A1 bridged, left arm since 2026-08-12) · `0x41` INA3221 (A0 to VS), **off the bus since 2026-10-01** · `0x70` PCA all-call, always present |
| Frequency | 50 Hz, prescale 121 gives exactly 50.0 Hz |

Address facts, verified live on 2026-08-11: the INA3221 A0 pin offers only
0x40 to 0x43 (datasheet SBOS576), so the old 0x44 plan was impossible (that
option belonged to the rover's INA219). Pad adjacency on the breakout chose
the final map: INA bridged A0 to VS (0x41), PCA #2 closed A0+A1 (0x43).

First check of any bench session, before anything is energised:

```bash
i2cdetect -y -r 7
```

`0x40` and `0x43` must appear (`0x41` only when the INA3221 is back on the
bus). If either is missing, stop: nothing below this line will work.

### Retries handle glitches, not broken wires

`OSError 121` (Remote I/O error) is retried three times, 5 ms apart, by
`retry_i2c()`. That keeps an isolated glitch from taking the node down mid
motion, and persistent failures still raise, as they should.

Do not read that as "the bus is unreliable and the software copes". In the one
session where it mattered, the last drop did not recover after 20 retries,
because the cause was a broken wire. **Retries are a net under a good harness,
never a substitute for one.**

## Bring-up order

The order for the Jetson bench. Steps 1 to 3 cannot move anything, which
is the whole point. The session runbook,
[session_v02.md](session_v02.md), walks it with checkboxes.

1. **Jetson and container first**, servo rail OFF. The Jetson boots on its
   own adapter; `docker compose -f docker-compose.jetson.yml up -d`; the log
   says "MOCK backend, DISARMED".
2. `i2cdetect -y -r 7` shows `0x40` and `0x43`. The boards' logic runs on
   the Jetson's 3.3 V, so they answer with the servo rail off.
3. **Silence both boards** before the rail comes up, in case a previous
   session left them driving pulses (a killed process does not clean up):
   `i2cset -y 7 0x40 0xfd 0x10` and `i2cset -y 7 0x43 0xfd 0x10`. That is
   ALL_LED_OFF_H with the full-off bit, the same write the driver's
   `disable_all()` makes. With V+ off it cannot move anything.
4. **Arms hanging at rest**, hands and tools clear, the ATX rear switch in
   reach. ATX on. Expect the power-on twitch.
5. Multimeter: **6.00 V on both V+ terminal blocks.**
6. Only then the gates of [safety.md](safety.md) rule 2, and only when
   Andres says so out loud: gate 1 by relaunch, gate 2 by `/soma/arm`.

Shutting down is the same list backwards: `relax` (home, then disarm),
ATX off, close gate 1 with a plain relaunch; the Jetson can stay up.

Channel by channel identification, when the wiring is unknown or has been
touched: drive **one channel at a time** with **60 ms bursts**, which is the
smallest motion still visible on an uncalibrated servo. Never sweep, never
hold, never aim at an end stop. See [bench.md](bench.md).
