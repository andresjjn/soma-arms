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

```
  battery / bench supply
        |
        +-- UBEC 6 V 8-10 A  ---> PCA9685 V+ terminal ---> left arm servos
        |                                              \-> L16 (whichever
        |                                                  UBEC is less loaded)
        +-- UBEC 6 V 8-10 A  ---> right arm servos
        |
        +-- Raspberry Pi 5 (its own supply, NOT the servo rail)
```

Non negotiable:

- **The servo rail never comes from USB or from the Pi's 5 V pin.** Feed the
  PCA9685 green V+ terminal block from the UBEC, with wire of **16 AWG or
  thicker**, and a capacitor across V+.
- **One UBEC per arm.** A stall on one arm must not brown out the other.
- The Pi and the servos share **ground** and nothing else.

## I2C

| | |
|---|---|
| Bus | Raspberry Pi 5, 40 pin header |
| Address | `0x40` |
| Frequency set on the PCA9685 | 50 Hz |

First check of any bench session, before anything is energised:

```bash
i2cdetect -y 1
```

`0x40` must appear. If it does not, stop: nothing below this line will work.

### Transient failures are normal, and handled

`OSError 121` (Remote I/O error) appeared four times during the 2026-07-22
session. A servo's inrush current bounces the ground rail and corrupts one
transaction without resetting the chip. `retry_i2c()` retries three times,
5 ms apart. Persistent failures still raise, as they should.

## Bring-up order

The order below is the safe one, and it is the order the 2026-07-22 session
actually followed.

1. `i2cdetect -y 1` shows `0x40`. **Servo rail still off.**
2. Validate the chain with **no power to the servos**: set 50 Hz, write
   channels, read the registers back. Nothing can move, so nothing can go
   wrong.
3. Energise the 6 V rail with the arms in a safe, unloaded pose.
4. Only then run the driver, and only then consider arming it. Arming needs
   Andres to say so out loud first: see [safety.md](safety.md).
