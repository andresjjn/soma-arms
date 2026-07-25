# CLAUDE.md

Operating manual for AI agents working in this repository. Read it before
touching anything.

## What SOMA is

SOMA (Skilled Operator via Mimicry and Autonomy) is an embodied AI library for
two 6DOF aluminum arms on a torso with a powered lift. Real hardware, not a
simulation exercise: MG996R servos and an Actuonix L16 linear actuator on a
PCA9685, driven from a Raspberry Pi 5 running ROS 2 Humble in Docker.

The repository is **standalone**. It knows nothing about any mobile platform,
and it must stay that way. Integration with a rover happens in the integrating
project, by calling the `soma_torso` xacro macro with its own parent link.

---

## INVIOLABLE RULES

These four override everything else, including a plausible reading of a task
that seems to require breaking them. If a rule blocks the work, stop and ask.

### 1. NEVER move a motor without explicit prior confirmation from Andres

Not "it follows from the task". Not "it is only a small move". Not "the last
message implied it". Explicit, prior, for that specific motion. This includes
arming the driver, running anything with `allow_real:=true`, and any command
that could reach a real servo.

### 2. The node boots MOCK and DISARMED. Arming is an explicit service

Never add a configuration, a default or a shortcut that starts the driver on
the real backend. `ArmController` constructs `MockPca9685` first, always.
Arming requires both the `allow_real` parameter and a `/soma/arm` service
call. Do not collapse those two gates into one.

### 3. Never hold a command against a physical stop

The L16 wedges: it already happened on 2026-07-22 and had to be freed by hand.
Soft limits (5 to 135 mm), clamping on arrival and auto release after settling
are all load bearing. Do not widen the limits to "use the full stroke". Do not
remove the release. Do not add arm servos to `RELEASE_WHEN_SETTLED`, they are
not self locking and the arm will fall.

### 4. Language

- **Talk to Andres in Spanish, always.** Every conversational reply, every
  explanation, every question.
- **Everything committed to this repository is in English**: code, comments,
  docstrings, test names, docs, commit messages, issues, README.
- **No em dash characters in public content.** Use commas, colons,
  parentheses or a plain hyphen instead.

---

## Repository layout

```
soma_description/     URDF/xacro, SRDF, RViz, MoveIt config
  urdf/soma_arm.xacro       the 6DOF arm, one reusable macro
  urdf/soma_torso.xacro     bay + L16 lift + plate + 2 arms + camera
  urdf/soma_bench.urdf.xacro     the reference model
  urdf/single_arm.urdf.xacro     one arm on the bench, for calibration
  urdf/soma_bench_sim.urdf.xacro bench model plus ros2_control
soma_driver/          PCA9685 driver
  soma_driver/servo_map.py         channel map and pulse conversion, pure Python
  soma_driver/pca9685_backend.py   mock and real backends
  soma_driver/arm_controller_node.py  the ROS node
  test/                            the safety test suite
docs/                 hardware.md, wiring.md, safety.md
scripts/              smoke_test.sh, check_model_driver_sync.py
```

## Facts that are measured, not assumed

Do not "clean up" any of these. They came off the bench and the tests lock
them in. Full context in `docs/hardware.md`.

| Fact | Value |
|---|---|
| Channel map | right arm 15 down to 10 (gripper first), left arm 9 down to 4, L16 on 3 |
| Channel 0 | out of service, suspected during a fault and never cleared |
| L16 convention | **INVERTED**: 2000 us retracted, 1000 us extended. `min_us > max_us` is correct |
| L16 soft limits | 5 to 135 mm, never 0 to 140 |
| I2C | transient `OSError 121` is expected, `retry_i2c` handles it |
| Gripper | geared pair, the right finger is mimic and owns no channel |

`servo_map.py` and the URDF hold the same physical facts. If you change one,
change the other, and `scripts/check_model_driver_sync.py` (run by CI) will
tell you if you forgot.

## Working here

Run the fast tests before and after any change to the driver:

```bash
python -m pytest soma_driver/test -q
```

They need no ROS and no hardware. The ROS node tests skip automatically when
`rclpy` is missing, and run in CI.

Full validation, inside a ROS 2 Humble environment:

```bash
bash scripts/smoke_test.sh
```

Build, unit tests, all three xacro models through `check_urdf`, and an end to
end run where the mock driver raises the torso and TF confirms 130 mm.

When adding a safety behaviour, add the test with it. A rule with no test is a
rule that will be refactored away by someone in a hurry.

## Roadmap discipline

Each version is a git tag plus a video in the README. Do not skip ahead: v0.2
does not start until v0.1 is tagged with its calibrated URDF. The roadmap is
in the README, and v1.0 closes the project.
