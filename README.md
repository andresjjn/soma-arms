# SOMA

**S**killed **O**perator via **M**imicry and **A**utonomy.

An embodied AI library for two 6DOF aluminum arms with a powered torso lift.
Real hardware: MG996R servos and an Actuonix L16 linear actuator on a PCA9685,
driven from a Raspberry Pi 5 running ROS 2 Humble in Docker.

[![CI](https://github.com/andresjjn/soma-arms/actions/workflows/ci.yml/badge.svg)](https://github.com/andresjjn/soma-arms/actions/workflows/ci.yml)

SOMA is standalone. It describes and drives the arms and the torso, and
nothing else: no chassis, no wheels, no navigation. Putting it on a mobile
base is the integrator's job, done by calling the `soma_torso` xacro macro
with their own parent link.

---

## The robot

| | |
|---|---|
| Arms | 2 x 6DOF aluminum, ThanksBuyer / Red Sun Global frame, "Arm Only" variant |
| Joints per arm | base yaw, shoulder, elbow, wrist pitch, wrist roll, geared gripper |
| Arm servos | 12 x Tower Pro MG996R, 25T spline, 500 to 2500 us over 180 degrees |
| Torso lift | Actuonix L16-140-63-6-R, 140 mm stroke, 20 mm/s, self locking |
| Driver board | PCA9685, 16 channels, I2C at `0x40`, 50 Hz |
| Compute | Raspberry Pi 5, ROS 2 Humble in Docker |
| Head | Luxonis OAK-D Lite, on-camera pose estimation |

Thirteen actuators on one I2C bus. Full part numbers, datasheets, the kit
parts list and every measured value: **[docs/hardware.md](docs/hardware.md)**
and **[docs/wiring.md](docs/wiring.md)**.

Honest numbers, because they shape everything else: the frame is advertised
for 25 kg.cm servos and the MG996R delivers about 10, so **useful payload is
around 330 g at 30 cm of reach**. SOMA is built to work in compact poses.

---

## Safety first

SOMA can hurt itself and you. Four things are worth knowing before anything
else, and the full model is in **[docs/safety.md](docs/safety.md)**:

1. **The driver boots on a mock backend and disarmed.** Going live needs both
   a launch parameter and an explicit service call. Two independent gates, on
   purpose.
2. **The lift is limited to 5 to 135 mm of its 140 mm stroke.** Holding a
   command against a stop wedged the lead screw once and it had to be freed by
   hand. It will not happen twice.
3. **Energise the 6 V rail only with the arms folded and resting.** These
   servos twitch at power-up.
4. **The real emergency stop is the switch on the 6 V rail.** Keep it in
   reach.

---

## Quick start

Nothing below touches hardware.

```bash
# in a ROS 2 Humble workspace
git clone https://github.com/andresjjn/soma-arms.git src/soma-arms
colcon build --symlink-install && source install/setup.bash
```

See the model in RViz with a slider per joint:

```bash
ros2 launch soma_description display.launch.py
```

One arm on its own, which is the model to use with calipers in hand:

```bash
ros2 launch soma_description display.launch.py model:=single_arm.urdf.xacro
```

Run the driver in mock mode and watch the torso rise:

```bash
ros2 launch soma_driver driver.launch.py
ros2 topic pub --once /soma/command sensor_msgs/msg/JointState "{name: [torso_lift_joint], position: [0.135]}"
```

Run the tests, no ROS and no hardware needed:

```bash
python -m pytest soma_driver/test -q
```

---

## Interface

| Kind | Name | Type |
|---|---|---|
| Subscriber | `/soma/command` | `sensor_msgs/JointState`, target positions |
| Publisher | `/joint_states` | `sensor_msgs/JointState`, ramped current pose |
| Service | `/soma/arm` | `std_srvs/SetBool`, arm or disarm the real output |

---

## Roadmap

Every version is a git tag, a video in this README, and a short. The project
has an end: at v1.0 it closes and the arms move on to their next life as the
manipulator of a mobile platform. Anything still missing at v1.0 becomes an
issue, not a reason to keep the project open.

| Version | Milestone | What has to be true | Status |
|---|---|---|---|
| **v0.1** | Calibrated URDF | Every `[calibrate]` length measured with calipers and corrected, servo zeros and ranges captured, model right in RViz, CI green | **TAGGED 2026-08-10.** Caliper session 2026-08-05: real hanging-bench configuration modeled (parallel J1-J4 axes, 33 mm clavicle); FK predicted the fingertips 5.7 mm above the plate and the photos agreed. Axis signs pinned down at the first powered session |
| **v0.2** | Real driver | Sliders move metal through the ROS driver: mock to hardware, arming procedure exercised end to end | bench first: all 13 actuators verified live from a Jetson Orin Nano through `scripts/servo_workbench.py` (2026-07-30) |
| **v0.3** | Eye-hand | Both arms bolted to the bench rig with the OAK-D fixed above. Pixel + depth deprojects to 3D in each arm's frame, IK reaches it. Acceptance: **click a point in the image and the arm touches it** | |
| **v0.4** | Language-directed manipulation | Gemini Robotics-ER 2 points, plans and verifies; SOMA executes through its safety gates (the reasoner proposes, the armed driver disposes; the cloud can never arm the robot). Bimanual pick and place directed in natural language | |
| **v1.0** | The operator | Two hours of continuous pick and place cycles on the bench, with a public cycle counter **verified by the ER model's progress classification**. Then the project closes | |

Packages arrive with their milestone: `soma_moveit_config` at v0.3,
`soma_agent` (the ER 2 orchestration layer) at v0.4, `soma_operator` at v1.0.

Vision teleop by human mimicry (BlazePose on the OAK-D VPU) moves to the
post-1.0 backlog: it is not a prerequisite for the operator and the
language-directed path ships sooner.

A **learning track** runs in parallel on the same model: the arm is
identified against MuJoCo (`L1`), then policies are trained on a MacBook
through MJX and deployed to the Jetson (`L2` reach, `L3` grasp, `L4`
bimanual handoff). Every physical quantity is measured and randomizable
precisely so those policies can transfer. See **[docs/plan.md](docs/plan.md)**.

Videos land here as each tag ships.

---

## Layout

```
soma_description/   URDF/xacro, SRDF, RViz config, MoveIt kinematics
soma_driver/        PCA9685 driver (mock and real) plus the safety test suite
docs/               see below
scripts/            smoke_test.sh, check_model_driver_sync.py
docker/             headless ROS 2 Humble image for building and validating
```

| Document | What it covers |
|---|---|
| **[docs/plan.md](docs/plan.md)** | **the master plan: hardware roles, the model pipeline, the simulation stack, the actuator model, the RL tasks, and the acceptance criteria of every phase** |
| [docs/hardware.md](docs/hardware.md) | every part, its datasheet numbers, and the measured values that override them |
| [docs/wiring.md](docs/wiring.md) | the verified channel map, power chain, connectors, bring-up order |
| [docs/safety.md](docs/safety.md) | the rules, how each is enforced in code, and the incident log |
| [docs/bench.md](docs/bench.md) | validated commands, RViz in a browser, the calibration procedures |
| [docs/migration.md](docs/migration.md) | what carried over from the bench driver and what was deliberately changed |

## Development

CI runs on every push: the pure Python test suite, a full ROS 2 Humble build,
`check_urdf` on all three models, a consistency check between the URDF and the
driver, and an end to end run where the mock driver raises the torso and TF
confirms 130 mm of travel.

```bash
bash scripts/smoke_test.sh   # the same thing, locally, inside a ROS environment
```

## License

MIT. See [LICENSE](LICENSE).
