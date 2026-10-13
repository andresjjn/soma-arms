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

The bench as it runs since 2026-09-01:

| Qty | Part | Role |
|---|---|---|
| 2 | 6DOF aluminum robot arm, "Arm Only" variant (ThanksBuyer / Red Sun Global) | the arms |
| 13 | Tower Pro MG996R servo | 12 in the arms, 1 spare |
| 1 | Actuonix L16-140-63-6-R linear actuator | torso lift. Off the hanging bench until the printed torso |
| 2 | PCA9685 16 channel PWM driver | one per arm, both on one I2C bus: `0x40` right arm and L16, `0x43` left arm |
| 1 | ATX supply, MaxiTech 300U, rated 10 A on +12 V | the bench supply, behind a 10 A fuse. Its rear switch is the emergency stop |
| 2 | UBEC 6 V, 8 to 10 A | the 6 V servo rails, one per board, so a stall on one arm cannot brown out the other |
| 1 | NVIDIA Jetson Orin Nano Super 8 GB | the computer: JetPack 7.2, the driver in a ROS 2 Humble container, I2C on bus `i2c-7`. Its own wall adapter, never the servo supply |
| 1 | INA3221 three channel power monitor | meant for the rails, read only, at `0x41`. Off the bus since 2026-10-01 |
| 1 | Luxonis OAK-D Lite | the eyes, verified on the Jetson on 2026-10-01. Since 2026-10-13 fixed on a mast above the central box, tilted forward over the arms' arc and rolled a quarter turn: software turns frames 90 deg counterclockwise (`oak_view.ROTATION`, `hand_gripper.CAMERA_ROTATE_DEG`) |

**History, not the current bench.** The first power-up, on 2026-07-22, ran
on a **Raspberry Pi 5** (ROS 2 Humble in Docker, I2C bus 1) and a
**LiPo 2S** with switch and fuse feeding a single UBEC, shared by both arms
and the L16. The Jetson replaced the Pi on 2026-07-30, and the ATX chain
with one UBEC per arm replaced the LiPo on 2026-09-01. The dated July
material below (sections 5, 6, 7 and 8) says which hardware it ran on.

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
stays in the box: SOMA drives the servos from a Jetson Orin Nano Super over
I2C through two PCA9685 boards (in July 2026, from a Raspberry Pi 5 through
one). The manual's page 28 is kept only as the source of the A to F
labelling.

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

**Torque caveat, and it is the defining constraint of this robot**: the frame
is advertised for 25 kg.cm servos, and the MG996R delivers roughly 10 kg.cm.
The manual's "25KG" label describes what the frame can take, not what is
bolted into it. Honest number: **at about 30 cm of reach the useful payload is
around 330 g.**

Base (A) and shoulder (B) are the joints that feel it first. The derived rule
is **always work in compact poses**, because a folded arm has a far shorter
moment arm than an extended one. That is why the SRDF ships a `compact` named
pose next to `candle`, and why pick and place trajectories should stay folded.

Upgrading is non destructive: only those two servos would need to be swapped
for a stronger 25 to 40 kg.cm unit in the same 40x20 body with a 25T spline.

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
thirteenth channel, channel 3 of the `0x40` board. No extra electronics, no
motor driver.

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

## 5. PCA9685 servo drivers

Two boards on one I2C bus, one per arm.

| Property | Value |
|---|---|
| Channels | 16 per board, 12 bit PWM each |
| Interface | I2C: `0x40` (right arm and L16) and `0x43` (A0+A1 bridged, left arm since 2026-08-12), both on the Jetson's bus `i2c-7` |
| Frequency | 50 Hz (20 ms period), the servo standard. Prescale 121 gives exactly 50.0 Hz, verified on the chip |
| Duty conversion | `duty12 = us / 20000 * 4095`, so 1500 us is 307 counts |
| Library | `smbus2`, register level, imported lazily after the arming check. Same protocol the bench tools validated on the chip; works identically on the Jetson Orin Nano (bus 7, today) and on the Pi (bus 1, July 2026), with autodetection |

Channel allocation and the power wiring live in [wiring.md](wiring.md).

Two rules that are not optional:

- **Never power a servo rail from USB or from the computer's 5 V** (the
  Jetson's today, the Pi's in July). Each board's V+ terminal block takes
  6 V from its own UBEC with wire of 16 AWG or thicker, and a capacitor
  across V+.
- **Neither board carries silkscreen channel numbers.** The map in
  `wiring.md` was proven output by output with a spare servo on 2026-10-01.
  Prove a channel number by moving a servo on it; never infer it from a
  photo or by counting pins.

### MEASURED: I2C failures, and what actually caused them

On 2026-07-22, on the Pi (bus 1) with a single board, the bus dropped four
times in one session with `OSError 121` (Remote I/O error). The last drop
did not recover at all, even after 20 retries.

The failures correlated with **handling and vibration**, not with motion or
current draw. Root cause: a **dupont jumper broken inside its insulation**, or
a cold solder joint on the clone board's header. The fix was physical, four
new short cables, and the bus stayed stable for the rest of the session.

Two conclusions, and it matters that they are kept separate:

- **`retry_i2c()` is a safety net, not a fix.** Three attempts 5 ms apart stop
  an isolated glitch from taking the node down mid motion. It did not, and
  could not, save the session: a broken wire is not a transient.
- **The real fix is the harness.** See the connector rules in
  [wiring.md](wiring.md). Loose connectors also produced "ghost" servos
  twitching at power-up.

Diagnostic that works: watch `i2cdetect` in a loop and tap the cables. If
`0x40` or `0x43` blinks, the problem is mechanical.

---

## 6. Compute and camera

**NVIDIA Jetson Orin Nano Super 8 GB**, the bench computer since
2026-07-30. JetPack 7.2 ships no ROS 2 Humble, so the driver runs in the
`soma_driver` container, made by `scripts/jetson_driver.sh` (the Jetson
has no `docker compose`; `docker-compose.jetson.yml` is its readable
twin). The repository lives at `/home/jetson/Projects/soma/soma-arms`,
and the address comes from DHCP: reach it as `jetson.local`. I2C to both PCA9685
boards goes over **bus `i2c-7`**, on header pins 1 (3V3), 3 (SDA), 5 (SCL)
and 6 (GND); pin 1 also feeds the boards' 3.3 V logic. `i2cdetect -y -r 7`
showing `0x40` and `0x43` is the first check of any bench session. Since
2026-09-01 the Jetson runs on its own wall adapter, never on the servo
supply (section 7). How to operate it: [bench.md](bench.md), section 6.

**History: Raspberry Pi 5**, the computer of the first power-up on
2026-07-22. It ran ROS 2 Humble in Docker with the single PCA9685 of the
time on **bus 1** (same header pins, `i2cdetect -y 1`), and `smbus2`
natively, outside Docker, for the bare metal bring-up scripts that still
live in `scripts/pi/`.

**Luxonis OAK-D Lite.** 91 x 28 x 17.5 mm, 61 g (datasheet). Stereo
baseline 75 mm, mono HFOV 73 deg, RGB HFOV 69 deg (Luxonis); minimum
depth about 34 cm in standard mode and 17 cm with extended disparity
(computed from those numbers at 640 px, matching the Luxonis figures),
and a stated absolute depth error under 2 % below 4 m, which is 7 to
11 mm at 35 to 55 cm. That is why a point from the supervisor becomes a
deck coordinate through the plane of the deck (a homography from
markers), with depth kept for confirming a part and its height, not for
placing the gripper. From v0.3 it is fixed to the column of the hanging
bench, looking down at the deck, at least 35 cm from the work area, and
out of the arms' reach. As found on 2026-10-12 it is mounted but looks
forward at the operator, a few centimeters from where the raised gripper
reaches, upside down (180 deg, corrected in software): it has to move.
On the printed torso it becomes the head; it carries its own VPU for
on-camera inference.

MEASURED on the Jetson, 2026-10-01, on its USB-C port with a 40 Gbps cable:
USB SUPER (5 Gbps), RGB 640x400 at 29.7 fps, depth 640x480 at 30.0 fps with
LR-check and subpixel, depthai 2.32 on the system Python. Against a plain
wall at 196 cm: 0.93 cm of temporal noise at the center, 37 % valid pixels.
The 7.5 cm stereo baseline is read from its factory calibration, not
measured.

---

## 7. Power

The bench chain, as assembled on 2026-09-01 and in use since (the wiring
rules that go with it are in [wiring.md](wiring.md)):

```
mains -> ATX supply (MaxiTech 300U, rated 10 A on +12 V)   <- rear switch = EMERGENCY STOP
           P4 +12 V -> 10 A fuse -> XT60 -> Y splitter -+-> UBEC 6 V -> V+ of 0x40 -> right arm
                                                        +-> UBEC 6 V -> V+ of 0x43 -> left arm

Jetson Orin Nano Super: its OWN wall adapter
```

**MEASURED, 2026-09-01:** 11.6 V at the ATX with nothing connected, inside
the ATX +-5 % band (11.4 to 12.6 V) but below the 11.8 to 12.3 V we would
have liked; 6.00 V on both V+ terminal blocks. Watch the 12 V under load:
below 11.4 V sustained, add a load fan on 5 V.

**Nothing but a UBEC feeds a servo.** The MG996R is rated to 7.2 V: not the
12 V side, and not a battery straight on V+. The UBEC is not a convenience,
it is the only thing between the supply and twelve dead servos.

- **One UBEC per arm**, in a star from the ATX with separate returns, so a
  stall on one arm cannot brown out the other. The ATX is the common root of
  that star, and a star does not protect against the root collapsing (next
  point).
- **The Jetson never shares the servo supply.** On 2026-09-01 hot plugging
  the servo branch (the input capacitors of both UBECs plus twelve servos)
  sank the shared 12 V and hard-shut the Jetson. It has run on its own wall
  adapter since, and nothing is ever hot plugged onto the servo branch.
- **The ATX rear switch is the emergency stop.** It drops the 12 V, both
  UBECs and their 6 V rails, and leaves the Jetson running. The driver
  cannot see the rails drop (the boards' logic runs on the Jetson's 3.3 V),
  so after an ATX stop it is still ARMED: disarm before the ATX comes back
  on. See [safety.md](safety.md).
- The Jetson and the servo rails share **ground** and nothing else.
- The **L16** (about 650 mA) is off this bench. When the torso returns it
  hangs off whichever UBEC is less loaded.
- The **INA3221** (`0x41`), meant to monitor the rails, is off the bus since
  2026-10-01: nothing worth measuring yet.
- Twelve MG996R servos are the wild card in the power budget. In a compact
  pose, or de-energised, they draw almost nothing; stalled they draw a lot.
  The uncertainty stays large until measured under real load, and with the
  INA3221 off the bus nothing on the bench measures current today.

**Power-on twitch.** MG996R clones kick when V+ comes up. Always energise the
rails with the arms at rest in a **compact pose**: on the hanging bench that
is both arms hanging at rest, nothing under or around them. This is a safety
rule, not a preference: see [safety.md](safety.md).

### History: the July 2026 battery chain

The bench chain from the first power-up, exactly as validated on
2026-07-22, until the ATX replaced it on 2026-09-01:

```
LiPo 2S  ->  switch + fuse  ->  UBEC 6 V  ->  PCA9685 V+ terminal  ->  servos
```

A single UBEC fed both arms and the L16, a weak point known at the time,
and the Pi had its own supply. A full 2S pack sits at 8.4 V, above the
7.2 V servo rating: that is why the rule above names batteries, and why the
LiPo never went straight onto V+.

---

## 8. Bench session log

The dated facts above come from real sessions. Below is the short version
of the first two; every session from 2026-09-01 on has its own entry in
[log.md](log.md).

**2026-07-22, first power-on of the full chain** (the Pi and the LiPo of
section 1, one board at `0x40`).

- I2C from the Pi to the PCA9685 verified (`i2cdetect -y 1` shows `0x40`), and
  the whole chain validated **with no power to the servos first**: exactly
  50 Hz, register echo on all 13 channels, every output left in FULL_OFF
  before the battery was connected.
- Channel by channel discovery with **60 ms bursts**, the smallest motion that
  is still visible on an uncalibrated servo. All 12 arm channels matched the
  expected map: right 15 down to 10, left 9 down to 4. See
  [wiring.md](wiring.md).
- **L16 inverted convention measured**, contradicting the datasheet. Found the
  hard way: an 8 second "retract" command extended it fully.
- The bus dropped four times with `OSError 121`, the last one unrecoverable.
  Cause was mechanical, fixed with new cables. Retry logic added as a net.
- The L16 then stopped responding after its first successful cycle. Root cause
  found that evening: it was **wedged against its internal stop** by that same
  held command. Symptoms: retract gave silence, extend gave a twitch with no
  travel. Neither a fresh battery nor a channel change helped.
- **Freed by hand**: an extend command plus gentle manual traction on the rod.
  After that it took absolute positions again, ending exactly at 70 mm.
- Soft limits, auto release and the move to channel 3 all date from here.

Session close: 13 channels mapped and verified with power, both arms and the
spine responding, and four hardware lessons the digital twin could never have
taught us.

**2026-08-05, caliper session: the geometry stops being an estimate.**

- Every arm-chain length measured on the right arm with a caliper; bench
  anchors with a tape measure. All values and exact endpoints live in
  `soma_description/config/dimensions.yaml` with per-entry provenance.
  The manual-photo estimates were off by up to 40 mm (upper arm: 79.3 mm
  real vs 120 mm estimated). Measured beats estimated, again.
- **The real bench configuration was established**: both arms HANG from a
  shared central box (the two base boxes bolted back to back, 109.8 mm
  disc seat to disc seat) on a monitor-stand column, disc axes horizontal,
  colinear, pointing outboard, 343 mm above the base plate.
- **The first four joint axes of each arm are PARALLEL** (disc, shoulder,
  elbow, wrist pitch): each arm is a planar 4R chain in its own vertical
  plane; the wrist roll provides the out-of-plane motion. A 33 mm link
  (the "clavicle") joins the disc center to the shoulder axis. This
  contradicts the manual's upright yaw-first chain and the model now
  follows the hardware.
- Cross-check that closed the loop: with every joint at zero the model
  puts the fingertips 5.7 mm above the base plate, which is what the
  photos of the rig show.
- Still pending: arm weight (arms never left the bench), joint axis SIGNS
  versus the hug convention (needs the first powered RViz-vs-reality
  comparison, v0.2), and the left yaw re-spline.
