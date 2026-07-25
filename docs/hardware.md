# SOMA hardware

Every part SOMA is built from, why it was chosen, and which numbers in the
code come from it. If a value in the URDF or in `servo_map.py` looks
arbitrary, it is probably explained here.

Two kinds of numbers appear below and they are never mixed:

- **Datasheet**: what the manufacturer claims.
- **MEASURED**: what we observed on the bench, with dates. When the two
  disagree, the measured value wins and the code follows the measured one.

---

## 1. Bill of materials

| Qty | Part | Role |
|---|---|---|
| 2 | 6DOF aluminum robot arm, "Arm Only" variant (ThanksBuyer / Red Sun Global) | the arms |
| 13 | Tower Pro MG996R servo | 12 in the arms, 1 spare |
| 1 | Actuonix L16-140-63-6-R linear actuator | torso lift |
| 1 | PCA9685 16 channel PWM driver | one I2C bus drives all 13 actuators |
| 2 | UBEC 6 V, 8 to 10 A | 6 V servo rail, one per arm |
| 1 | Raspberry Pi 5 | runs ROS 2 Humble in Docker |
| 1 | Luxonis OAK-D Lite | head camera, on-camera pose estimation for v0.5 |

---

## 2. The arm

**Product**: 6DOF metal robot arm, AliExpress item `1005007215923987`, sold by
ThanksBuyer. Bought as the **"Arm Only"** variant, without servos, so our own
MG996R units go in. Two complete arms, not the short version, because the goal
is a torso with two full arms rather than one desk arm.

**Manual**: 30 page illustrated assembly guide branded "Red Sun Global",
Chinese with English subtitles. Two pages matter for this repository:

- **Page 1**: the parts list (below).
- **Page 28**: the wiring diagram, which is where the **A to F joint labels**
  come from. Those labels are the naming authority for the whole project. The
  page shows the vendor's own Arduino Nano wiring (`D4`=A through `D9`=F)
  pointing at each physical joint on a photo of the finished arm.

**We do not use the vendor electronics.** The kit ships an Arduino Nano, an
expansion board, a Bluetooth module and an LM2596 step down module. All of it
stays in the box: SOMA drives the servos from a Raspberry Pi 5 over I2C
through a PCA9685. The manual's page 28 is kept only as the source of the
A to F labelling.

### Kit contents (manual, page 1)

| Item | Qty | Note |
|---|---|---|
| NANO motherboard, NANO expansion board, Bluetooth module | 1 each | unused, see above |
| Servo | 6 | not included in the "Arm Only" variant |
| 5 V step down module (LM2596) | 1 | unused |
| Metal servo horn | 4 | 25T spline |
| M4x12 bearing | 3 | including the large base slew bearing |
| Round power female connector, Dupont lines, heat shrink | assorted | unused |
| M3 nuts | 22 | plus 2 anti slip nuts |
| M3x6 / M3x8 / M3x12 / M3x10 / M3x16 screws | 23 / 12 / 8 / 4 / 4 | |
| M4x6 / M4x8 screws | 20 / 3 | |
| M5x8 / M5x25 screws | 3 / 3 | M5x8 fastens the base plate, page 29 |
| M3x2 / M3x4 / M3x8 / M5x15 black pillars | 6+6 / 4 / 2+2 / 3 | |
| M5x25 steel columns, M3x11 copper pillars | 3 / 6 | |
| M2.5x5 self tapping, M3x5 flat head screws | 8 / 8 | |
| M3x8 / M4x8 shims | 8 / 3 | |

### Kinematic chain

The manual labels the joints A to F. The URDF uses the same order, so a
photograph of the arm can be read straight onto the model:

| Manual | URDF joint | Axis | Description |
|---|---|---|---|
| A | `<prefix>yaw_joint` | Z | base yaw on the large slew bearing |
| B | `<prefix>shoulder_joint` | Y | shoulder pitch |
| C | `<prefix>elbow_joint` | Y | elbow pitch, twin servo block |
| D | `<prefix>wrist_pitch_joint` | Y | wrist pitch, called "elbow 2" while wiring |
| E | `<prefix>wrist_roll_joint` | local Z | wrist roll |
| F | `<prefix>finger_l_joint` (+ mimic `finger_r_joint`) | local X | geared two finger gripper |

The gripper is a gear pair: one servo drives the left finger and the right
finger is mechanically forced to mirror it. In the URDF the right finger is a
`<mimic>` joint with multiplier -1, and in the driver it lives in
`MIMIC_JOINTS` with no PWM channel of its own. Commanding it would be a bug.

### Frame and servo compatibility

| | Frame designed for | MG996R | Fits |
|---|---|---|---|
| Servo body | 40 x 20 class, 25 kg servo | 40.7 x 19.7 x 42.9 mm | yes |
| Spline | 25T | 25T | yes, our 25T horns bolt straight on |

**Torque caveat**: the frame is dimensioned for 25 kg.cm servos and the MG996R
delivers roughly 10 kg.cm. Base (A) and shoulder (B) are therefore the joints
to watch under load. This is why the SRDF ships a `compact` named pose:
a folded arm has a much shorter moment arm than an extended one. Upgrading is
non destructive, since only those two servos would need to be swapped for a
stronger 25 to 40 kg.cm unit in the same 40x20 / 25T format.

### Link lengths

Every length in `soma_description/urdf/soma_arm.xacro` is marked
`[calibrate]`. They are estimates taken from the manual photographs and from
the standard dimensions of this frame family. Measuring the assembled arm with
calipers and correcting those properties **once** is the deliverable of
milestone v0.1: the rest of the model, the FK and the planning follow
automatically.

---

## 3. Servos: Tower Pro MG996R

| Property | Value | Source |
|---|---|---|
| Body | 40.7 x 19.7 x 42.9 mm, flange 54.5 mm long | measured |
| Output shaft | offset 10.3 mm toward the front of the body | measured |
| Spline | 25T | datasheet |
| Mounting | M3, 14 mm hole pattern; M3x8 into the internal thread of the output shaft | measured |
| Torque | about 10 kg.cm at 6 V, roughly 0.98 N.m | datasheet |
| Speed | about 0.17 s / 60 degrees, roughly 6 rad/s | datasheet |
| Control | PWM 500 to 2500 us over 180 degrees, 50 Hz | kit manual |
| Supply | 4.8 to 7.2 V, driven here from a 6 V rail | datasheet |

**The driver deliberately does not use the full speed.** `servo_map.py` caps
every arm joint at **2.5 rad/s**, well under the roughly 6 rad/s the servo can
do. Smooth motion is a project rule, not a performance compromise.

Joint range is mapped as +/- 90 degrees around center: 500 us at -90, 1500 us
at 0, 2500 us at +90. The gripper uses the upper half of the band only,
1500 to 2500 us over 0 to 1 rad, and that mapping is `[calibrate]` against the
real gear gripper.

---

## 4. Torso lift: Actuonix L16-140-63-6-R

The part number decodes as: **L16** series, **140** mm stroke, **63:1**
gearbox, **6** V, **R** = RC servo interface.

| Property | Value | Source |
|---|---|---|
| Stroke | 140 mm | datasheet |
| Gearbox | 63:1 | datasheet |
| Speed | 20 mm/s | datasheet, used as the driver rate limit |
| Max force | 100 N | datasheet |
| Holding force with power removed | 46 N | datasheet, and the reason auto release is safe |
| Current draw | about 650 mA | datasheet |
| Interface | RC PWM, 1 to 2 ms, same as a servo | datasheet |
| Lead screw | self locking | datasheet |

Because the interface is a plain RC servo signal, the L16 is simply the
thirteenth channel on the PCA9685. No extra electronics, no motor driver.

### MEASURED: this unit runs inverted

Verified on 2026-07-22 with the 6 V rail live:

> **2000 us = retracted, 1000 us = extended.**

That is the opposite of the nominal convention. The code encodes it directly:
in `SERVO_MAP['torso_lift_joint']` the field `min_us` (1964.3) is **larger**
than `max_us` (1035.7). That inversion is intentional and correct, and the
tests lock it in.

### MEASURED: soft limits, 5 mm from each stop

Also on 2026-07-22: a retract command held for about 8 seconds while the rod
was **already fully retracted** wedged the lead screw against its stop. The
actuator went deaf and had to be freed by hand.

Two independent mitigations came out of that, and both are load bearing:

1. **Soft limits.** The commandable range is **5 to 135 mm**, not 0 to 140.
   The pulse anchors in `SERVO_MAP` are the pulse widths at 5 mm and 135 mm,
   so saturation can never land on a mechanical stop. The URDF limits match,
   so a planner cannot aim at one either.
2. **Auto release.** Once the lift has been settled on target for 0.5 s, the
   driver **cuts the PWM signal on that channel**. The lead screw is self
   locking and holds 46 N unpowered, so nothing sags, and there is no signal
   left to grind against a stop. See `RELEASE_WHEN_SETTLED`.

Arm servos must **never** be added to `RELEASE_WHEN_SETTLED`. They are not
self locking: cut their signal and the arm falls under its own weight.

Usable travel is therefore **130 mm**, and a full sweep takes 6.5 s at
20 mm/s. The end to end smoke test asserts exactly that.

---

## 5. PCA9685 servo driver

| Property | Value |
|---|---|
| Channels | 16, 12 bit PWM each |
| Interface | I2C, address `0x40` |
| Frequency | 50 Hz (20 ms period), the servo standard |
| Library | `adafruit-circuitpython-pca9685`, imported lazily |

Channel allocation and the power wiring live in [wiring.md](wiring.md).

Two rules that are not optional:

- **Never power the servo rail from USB or from the Pi's 5 V.** The V+
  terminal block takes 6 V from the UBEC with wire of 16 AWG or thicker, and
  a capacitor across V+.
- **The board carries no silkscreen channel numbers.** The map in `wiring.md`
  was confirmed channel by channel with power on. Do not infer it from a
  photo.

### MEASURED: transient I2C failures

Also on 2026-07-22: the bus dropped four times during the session with
`OSError 121` (Remote I/O error). Root cause is the inrush current of a servo
bouncing the ground rail and corrupting a **single** transaction, without
resetting the chip. Retrying a few milliseconds later works.

`retry_i2c()` wraps every real transaction with three attempts, 5 ms apart. A
single glitch must never take the node down mid motion.

---

## 6. Compute and camera

**Raspberry Pi 5.** Runs ROS 2 Humble in Docker. I2C to the PCA9685 comes off
the 40 pin header; `i2cdetect` showing `0x40` is the first check of any bench
session.

**Luxonis OAK-D Lite.** 91 x 28 x 17.5 mm, 61 g, mounted as the head on the
torso plate so it rises with the arms. It carries its own VPU, which is the
whole point: from v0.5 the BlazePose pose estimation runs **on the camera**,
not on the Pi, so the Pi keeps its cycles for control.

---

## 7. Power

- **6 V rail** from **two UBECs, 8 to 10 A, one per arm.** Splitting them is
  deliberate: a stall on one arm must not brown out the other.
- The **L16** (about 650 mA) hangs off whichever UBEC is less loaded.
- Twelve MG996R servos are the wild card for the power budget. In a compact
  pose or de-energised they draw almost nothing; stalled they draw a lot. The
  uncertainty is large until measured under real load, which is one of the
  things the v1.0 two hour run will finally answer.

---

## 8. Bench session log

The dated facts above come from real sessions. The short version:

**2026-07-22, first power-on of the full chain.**

- I2C from the Pi to the PCA9685 verified (`i2cdetect` shows `0x40`), and the
  whole chain validated **with no power to the servos first**: 50 Hz exactly,
  register echo confirmed.
- Channel map confirmed channel by channel, with power. See `wiring.md`.
- L16 inverted convention measured, contradicting the nominal datasheet
  convention.
- The bus glitched four times with `OSError 121`. Cause found, retry added.
- The L16 stopped responding after its first successful cycle. Root cause
  found later that evening: it was **wedged against its stop** by a held
  command. Freed by hand. Soft limits and auto release both date from here.
