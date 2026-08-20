# CLAUDE.md

Operating manual for AI agents working in this repository. Read it before
touching anything.

## What SOMA is

SOMA (Skilled Operator via Mimicry and Autonomy) is an embodied AI library for
two 6DOF aluminum arms on a torso with a powered lift. Real hardware, not a
simulation exercise: MG996R servos and an Actuonix L16 linear actuator on a
PCA9685, driven from a Raspberry Pi 5 running ROS 2 Humble in Docker.

The repository is **standalone**. It knows nothing about any mobile platform,
and it must stay that way. Integration happens in the integrating project, by
calling the `soma_torso` xacro macro with its own parent link.

**The project has an end.** At v1.0 it closes and the arms move on. Anything
still missing at that point becomes an issue, not a reason to keep the project
open. Do not propose scope that pushes v1.0 further away.

---

## INVIOLABLE RULES

These override everything else, including a plausible reading of a task that
seems to require breaking them. If a rule blocks the work, stop and ask.

### 1. NEVER move a motor without explicit prior confirmation from Andres

Not "it follows from the task". Not "it is only a small move". Not "the last
message implied it". Explicit, prior, for that specific motion. This includes
arming the driver, running anything with `allow_real:=true`, and any command
that could reach a real servo, including over SSH on the Pi.

### 2. The node boots MOCK and DISARMED. Arming is an explicit service

Never add a configuration, a default or a shortcut that starts the driver on
the real backend. `ArmController` constructs the mock backend first, always
(since the two-board bench of 2026-08-11, a fleet of `MockPca9685`, one per
board address in the map).
Arming requires both the `allow_real` parameter and a `/soma/arm` service
call. Do not collapse those two gates into one.
`RealPca9685(armed=False)` raising `PermissionError` is load bearing.

### 3. Never hold a command against a physical stop

The L16 wedges: it happened on 2026-07-22 after an 8 second held command, and
it had to be freed by hand. Soft limits (5 to 135 mm), clamping on arrival and
auto release after settling are all load bearing. Do not widen the limits to
"use the full stroke". Do not remove the release. Do not add arm servos to
`RELEASE_WHEN_SETTLED`, they are not self locking and the arm will fall.

When probing real end stops, move in small steps and never sustain a command.

### 4. Energise V+ only with the arms compact and resting

MG996R clones twitch when the rail comes up. Both arms folded, resting on the
bench, nothing fragile underneath, every time.

No parameter or test can enforce this one. That is why it is written here.

### 5. The harness is part of the safety system

Every unexplained fault in the first real session was a connector: a dupont
jumper broken inside its insulation dropped the I2C bus four times, and loose
servo connectors produced servos twitching at power-up. `retry_i2c` is a net
under isolated glitches, never a substitute for good wiring. Never present it
as one.

Permanent harness: JST with a positive lock for I2C, silicone retainers on
servo connectors, 16 AWG or thicker for V+.

### 6. Language and public content

- **Talk to Andres in Spanish, always.** Every reply, explanation and question.
- **Explain every technical decision as it is made**: what was done, why
  that option, what the alternative would have cost. Andres is here to
  learn the system end to end and ride this wave as high as it goes, not
  to delegate it. Every change is also a lesson; write it like one.
- **Everything committed here is in English**: code, comments, docstrings,
  test names, docs, commit messages, issues, README.
- **No em dash characters in public content.** Use commas, colons,
  parentheses or a plain hyphen.
- **Honest numbers, always.** Measured beats manufacturer claims, and when
  they disagree, say so and say which one the code follows.
- **Never mention clients, employers or work code.** NDA applies.
- **No job search strategy in a public repository.**

---

## Repository layout

```
soma_description/     URDF/xacro, SRDF, RViz, MoveIt kinematics
  urdf/soma_arm.xacro       the 6DOF arm, one reusable macro
  urdf/soma_torso.xacro     bay + L16 lift + plate + 2 arms + camera
  urdf/soma_bench.urdf.xacro     the reference model
  urdf/single_arm.urdf.xacro     one arm on the bench, for calibration
  urdf/soma_bench_sim.urdf.xacro bench model plus ros2_control
soma_driver/          PCA9685 driver
  soma_driver/servo_map.py         channel map and pulse conversion, pure Python
  soma_driver/pca9685_backend.py   mock/real backends + per-board fleet
  soma_driver/arm_controller_node.py  the ROS node
  soma_driver/primitives.py        named poses and sequences, pure and tested
                                   (the surface the v0.4 agent will call)
  soma_driver/primitives_cli.py    ros2 run soma_driver soma_primitives <name>
  soma_driver/sign_check_cli.py    bench tool: verify joint axis signs (v0.2)
  soma_driver/ina3221.py           power monitor, read-only, no gates needed
  test/                            the safety test suite
docs/                 hardware, wiring, safety, bench, migration
scripts/              smoke_test.sh, check_model_driver_sync.py
```

Packages that arrive with their milestone, and not before:
`soma_moveit_config` (v0.3), `soma_agent` (v0.4, the Gemini Robotics-ER 2
orchestration layer), `soma_operator` (v1.0). Mimicry teleop is post-1.0
backlog.

### The cloud reasoner never touches the trigger

From v0.4 on, an external model (Gemini Robotics-ER 2) points, plans and
verifies. It does so exclusively by calling SOMA primitives that sit BEHIND
the two arming gates. No API response, function call or agent step may arm
the driver, widen a limit, or bypass the ramp. The reasoner proposes, the
armed driver disposes. If a design makes the cloud a safety dependency, the
design is wrong.

## Facts that are measured, not assumed

Do not "clean up" any of these. They came off the bench and the tests lock
them in. Full context in `docs/hardware.md`.

| Fact | Value |
|---|---|
| Bench configuration | both arms HANG from a central box on a monitor-stand column (measured 2026-08-05). First joint axis horizontal, outboard; J1 to J4 axes PARALLEL per arm (planar 4R chain plus wrist roll). The L16 torso is NOT on this bench |
| Channel map | right arm 15 down to 10 (gripper first) on board `0x40`, left arm 9 down to 4 on board `0x43` (switched 2026-08-12, channel numbers unchanged), L16 on 3 (`0x40`) |
| Channel 0 | out of service, suspected during a fault and never cleared |
| L16 convention | **INVERTED**: 2000 us retracted, 1000 us extended. `min_us > max_us` is correct |
| L16 soft limits | 5 to 135 mm, never 0 to 140 |
| Servos | MG996R at about 10 kg.cm, **not** the "25KG" the manual advertises |
| Payload | about 330 g at 30 cm of reach. Work in compact poses |
| Power | LiPo 2S to switch and fuse to UBEC 6 V to V+. **Never the LiPo directly**, 8.4 V exceeds the 7.2 V servo rating |
| I2C | Jetson Orin bus i2c-7, header pins 1/3/5/6. PCA #1 at `0x40` (12 servos), PCA #2 at `0x43` (A0+A1 bridged, left arm since 2026-08-12), INA3221 at `0x41` (A0 to VS). Verified live 2026-08-11. Prescale 121 for exactly 50.0 Hz. Pi era: bus 1 |
| Gripper | geared pair, the right finger is mimic and owns no channel |

`servo_map.py` and the URDF hold the same physical facts. If you change one,
change the other. `scripts/check_model_driver_sync.py`, run by CI, will tell
you if you forgot.

## The tests are the specification

24 tests came over from the bench driver, and the suite has grown since
(151 passing plus 1 skipped as of 2026-08-12; the skipped one needs `rclpy`
and runs in the ROS job of CI). They encode every hardware contract. If a
change breaks one, the change is wrong until proven otherwise. Never edit a
test to make a change pass without saying so explicitly and explaining why
the hardware fact changed.

If a test count is ever quoted publicly, run the suite and read the number
off the terminal first. The honest-numbers rule applies to our own metrics
before it applies to anyone else's datasheet.

Run the fast suite before and after any driver change, no ROS or hardware
needed:

```bash
python -m pytest soma_driver/test -q
```

Full validation inside a ROS 2 Humble environment:

```bash
bash scripts/smoke_test.sh
```

Build, unit tests, all three xacro models through `check_urdf`, and an end to
end run where the mock driver raises the torso and TF confirms 130 mm.

When adding a safety behaviour, add the test with it. A rule with no test is a
rule that will be refactored away by someone in a hurry.

## Roadmap discipline

Each version is a git tag, a video in the README and a short. Do not skip
ahead: v0.2 does not start until v0.1 is tagged with its calibrated URDF, and
calibration needs calipers on real parts, which is not something an agent can
do. The roadmap lives in the README.

## Waver ecosystem

SOMA is independent code, but not independent history. The arms were brought
up as part of a rover project, and the record of every hardware decision lives
there. This section is context only: nothing in this repository depends on
those files, and nothing from them should be copied in wholesale.

| File | What it holds |
|---|---|
| `Waver/HANDOFF_SOMA.md` | the handoff that defined this project: roadmap, inviolable rules, the exact `SERVO_MAP` table, node interfaces, the 24 tests, validated commands |
| `Waver/cad/MEDIDAS.md` | the master log of hardware decisions and bench sessions. **Read this first for any question about history** |
| `Waver/Manual de ensamble ^ DOF arm.pdf` | the arm kit manual. Page 1 is the parts list, page 28 is where the A to F joint labels come from |

When SOMA reaches v1.0, the platform project drops its own copies of these
packages and consumes SOMA as a dependency instead.
