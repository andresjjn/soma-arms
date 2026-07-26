# SOMA wiring

## The channel map

**Verified channel by channel on 2026-07-22 with the 6 V rail live.** The
PCA9685 board has no silkscreen numbering, so this table is the only
authority. It is also locked in by
`test_measured_wiring_2026_07_22` in `soma_driver/test/test_servo_map.py`:
change the wiring and that test goes red, which is exactly what should happen.

The pattern is: **channels descend from 15, right arm first, outermost joint
first**. Gripper, then wrist roll, then down the arm to the base.

| Channel | Joint | Manual label | Actuator |
|---|---|---|---|
| 15 | `right_arm_finger_l_joint` | F | MG996R, gripper |
| 14 | `right_arm_wrist_roll_joint` | E | MG996R |
| 13 | `right_arm_wrist_pitch_joint` | D | MG996R, called "elbow 2" while wiring |
| 12 | `right_arm_elbow_joint` | C | MG996R, "elbow 1" |
| 11 | `right_arm_shoulder_joint` | B | MG996R |
| 10 | `right_arm_yaw_joint` | A | MG996R, base |
| 9 | `left_arm_finger_l_joint` | F | MG996R, gripper |
| 8 | `left_arm_wrist_roll_joint` | E | MG996R |
| 7 | `left_arm_wrist_pitch_joint` | D | MG996R |
| 6 | `left_arm_elbow_joint` | C | MG996R |
| 5 | `left_arm_shoulder_joint` | B | MG996R |
| 4 | `left_arm_yaw_joint` | A | MG996R, base |
| 3 | `torso_lift_joint` | n/a | Actuonix L16-140-63-6-R |
| 2, 1 | spare | | |
| 0 | spare, **under suspicion** | | see below |

Thirteen actuators on a sixteen channel board, three channels to spare.

**Channel 0 is left empty on purpose.** The L16 was originally wired there.
When it went deaf, channel 0 was one of the suspects. The real cause turned
out to be a wedged lead screw, not the channel, but by then the actuator had
been moved to channel 3 and everything was verified there. Channel 0 was never
cleared, so it stays out of service until someone proves it good.

The right hand fingers (`*_finger_r_joint`) appear nowhere in this table.
They are a mechanical gear pair driven by the left finger servo, so they have
no channel. `test_mimic_joints_have_no_channel` enforces that.

## Power

Today, as validated on the bench:

```
  LiPo 2S ---> switch + fuse ---> UBEC 6 V ---> PCA9685 V+ terminal
                                                  |
                                                  +--> 12 arm servos
                                                  +--> L16 lift
  Raspberry Pi 5 (its own supply, shares GROUND only)
```

Planned, and not fitted yet: a **second UBEC, one per arm**, so a stall on one
arm cannot brown out the other. The L16 then hangs off whichever rail is less
loaded.

Non negotiable:

- **NEVER connect the LiPo directly to the servos.** A full 2S pack is 8.4 V
  and the MG996R is rated to 7.2 V. The UBEC is the only thing preventing
  twelve dead servos.
- **The servo rail never comes from USB or from the Pi's 5 V pin.** Feed the
  PCA9685 green V+ terminal block from the UBEC, with wire of **16 AWG or
  thicker**, and a capacitor across V+.
- **Energise V+ only with the arms in a compact pose, resting on something.**
  MG996R clones twitch at power-up.
- The Pi and the servos share **ground** and nothing else.

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
| Bus | Raspberry Pi 5, bus 1, header pins 1 (3V3), 3 (SDA), 5 (SCL), 6 (GND) |
| Address | `0x40` |
| Frequency | 50 Hz, prescale 121 gives exactly 50.0 Hz |

First check of any bench session, before anything is energised:

```bash
i2cdetect -y 1
```

`0x40` must appear. If it does not, stop: nothing below this line will work.

### Retries handle glitches, not broken wires

`OSError 121` (Remote I/O error) is retried three times, 5 ms apart, by
`retry_i2c()`. That keeps an isolated glitch from taking the node down mid
motion, and persistent failures still raise, as they should.

Do not read that as "the bus is unreliable and the software copes". In the one
session where it mattered, the last drop did not recover after 20 retries,
because the cause was a broken wire. **Retries are a net under a good harness,
never a substitute for one.**

## Bring-up order

This is the order the 2026-07-22 session actually followed, and it is the one
to repeat. Steps 1 and 2 cannot move anything, which is the whole point.

1. `i2cdetect -y 1` shows `0x40`. **Servo rail still off.**
2. Validate the chain **with no power to the servos**: set 50 Hz, write all 13
   channels, read the registers back, and leave every output in **FULL_OFF**
   before the battery goes anywhere near it.
3. Put both arms in a **compact pose, resting on the bench**. Then energise
   the 6 V rail. Expect a twitch.
4. Only then run the driver, and only then consider arming it. Arming needs
   Andres to say so out loud first: see [safety.md](safety.md).

Channel by channel identification, when the wiring is unknown or has been
touched: drive **one channel at a time** with **60 ms bursts**, which is the
smallest motion still visible on an uncalibrated servo. Never sweep, never
hold, never aim at an end stop. See [bench.md](bench.md).
