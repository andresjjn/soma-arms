# SOMA master plan

How this robot gets built, in order, with what hardware, and what has to be
true before each step counts as done. The README holds the short roadmap
table; this file holds the engineering behind it.

Two tracks run in parallel and share one model and one driver:

- **Demo track**: calibrated model, real driver, eye-hand, then a bimanual
  pick and place directed in natural language by Gemini Robotics-ER 2.
  This is what ships publicly and what closes the project at v1.0.
- **Learning track**: the same robot in MuJoCo, system identified against
  the real arm, used to train policies that transfer back. This is the
  deeper work, it feeds a write-up, and it is why every model decision
  below is made "RL ready" instead of "good enough for RViz".

The tracks are deliberately coupled at exactly one place: **the model**.
One `dimensions.yaml` produces the URDF that RViz, MoveIt, MuJoCo and the
driver all agree on. If they ever disagree, the model is wrong, not the
robot.

---

## 1. Hardware, and what each piece is for

| Device | Role | Why it and not something else |
|---|---|---|
| 2 x 6DOF aluminum arm, 12 x MG996R | The robot | Already built and wired |
| Actuonix L16-140-63-6-R | Torso lift, one prismatic DOF | Self locking, so it holds with no power |
| PCA9685 at `0x40` | 13 PWM channels on one I2C bus | One bus, no motor drivers needed |
| UBEC 6 V (a second one planned) | Servo rail | The only thing between a 2S LiPo and twelve dead servos |
| **Jetson Orin Nano Super 8 GB** | Robot brain: ROS 2, driver, OAK pipeline, ER 2 client, policy inference | 67 TOPS at the edge. Runs the deployed policy, never trains it |
| **OAK-D Lite** | Eyes: RGB, stereo depth, on-VPU inference | Depth is what turns a 2D point from ER 2 into an XYZ the arm can reach |
| **MacBook Pro M3 Pro 18 GB** | Development and **RL training** | MJX runs on Apple Silicon through XLA. This is the training box |
| Raspberry Pi 5 | Waver rover, separate project | Not part of SOMA. SOMA is integrated into Waver after v1.0 |
| Cardboard cubes, 10 cm faces, ArUco ids 7 to 10 | Manipulation targets and ground truth pose | Light enough for a 330 g payload, and their pose is measurable to the millimeter |

**Compute topology, and the rule that holds it together:**

```
MacBook M3 Pro           Jetson Orin Nano              PCA9685 -> 13 actuators
  MuJoCo + MJX             ROS 2 Humble                       ^
  train policies    --->   soma_driver  (armed) ---------------
  export weights           soma_agent   (ER 2 client)
                           depthai pipeline <--- OAK-D Lite
                                  |
                                  v
                        Gemini Robotics-ER 2 (cloud, reasoning only)
```

Latency decides the split. Anything inside the control loop runs on the
Jetson. Anything that thinks in seconds (task planning, cycle
verification) may live in the cloud. Anything that needs a GPU for days
(training) runs on the Mac. **The cloud is never in the safety path**, and
no remote response can arm the driver.

---

## 2. What "RL ready URDF" actually means

A model that looks right in RViz is not a model you can learn in. Seven
concrete requirements, each with the artifact that satisfies it:

| # | Requirement | How SOMA satisfies it | Phase |
|---|---|---|---|
| 1 | **Kinematics measured, not guessed** | `dimensions.yaml` with `status: measured` on every arm entry, caliper session, sync test against the xacro | v0.1 |
| 2 | **Real joint limits per servo** | Zeros and mechanical min/max captured with `servo_workbench.py`, written back to the URDF and to `SERVO_MAP` | v0.1 |
| 3 | **Mass, center of mass and inertia** | Each subassembly weighed on a kitchen scale; inertias recomputed from primitives with real masses, not from guessed ones | v0.1 |
| 4 | **An actuator model that matches the hardware** | MG996R is a **position controlled** RC servo with an internal loop. The sim actuator must be a saturated position servo (`kp`, `forcerange`, `armature`, `damping`, `frictionloss`), never a torque source. See section 4 | v0.5 |
| 5 | **Cheap collision geometry** | Collisions stay primitives (boxes, cylinders). No mesh collisions. Thousands of parallel envs cannot afford mesh contacts | already true |
| 6 | **Parameterization for domain randomization** | Every physical quantity comes from `dimensions.yaml`, so a randomizer can perturb masses, friction, gains and latencies without editing XML by hand | v0.5 |
| 7 | **One source of truth across sim, planner and hardware** | `check_model_driver_sync.py` and `test_dimensions_sync.py` in CI, extended to cover the MJCF export | v0.5 |

Points 1 to 3 are what the caliper session buys. Point 4 is what most
hobby RL projects get wrong and why their policies never transfer.

### Model pipeline

```
dimensions.yaml  (measured values, with provenance)
      |
      v
  *.xacro  ------> URDF  ------> RViz / MoveIt / ros2_control  (demo track)
      |                 \
      |                  ------> MJCF  ------> MuJoCo + MJX     (learning track)
      v
 SERVO_MAP (soma_driver)  ------> PCA9685 -> real arms
```

The URDF to MJCF step is not automatic and must be scripted and versioned
(`scripts/urdf_to_mjcf.py`), because three things need hand authoring on
the MuJoCo side:

- **Actuators**: URDF has no notion of an RC servo. The MJCF gets one
  `<position>` actuator per joint with identified gains.
- **The geared gripper**: URDF `<mimic>` does not survive conversion.
  MJCF expresses it as an `<equality joint>` constraint.
- **Sensors and sites**: the tool frame, the camera site, and the contact
  sensors the reward needs.

---

## 3. Simulation stack, decided by the hardware we own

| Option | Verdict | Reason |
|---|---|---|
| **MuJoCo + MJX** | **Primary, for learning** | MJX compiles through XLA and runs on Apple Silicon GPUs. MuJoCo Playground exists exactly for arm sim to real. Contact model is the best available for grasping |
| Gazebo + ros2_control | Secondary, optional | Useful to rehearse the ROS pipeline (MoveIt trajectories, controllers) without hardware. `soma_bench_sim.urdf.xacro` already carries the plumbing. Not used for RL: too slow for parallel envs |
| Isaac Sim / Isaac Lab | **Ruled out for now** | Requires x86 plus RTX. Isaac Sim is not supported on Jetson at all. Revisit only if a rented cloud GPU becomes worth it for a final large training run |

Honest consequence: an M3 Pro is not an RTX 4090. Expect thousands of
parallel environments, not tens of thousands, and expect a reach policy to
train in hours rather than minutes. That is fine for the tasks in section
5, and it keeps the entire project on hardware already owned.

---

## 4. The actuator model, which is where sim to real is won or lost

An MG996R takes a pulse width and closes its own position loop internally.
It is not a torque source. Modeling it as one produces policies that look
brilliant in sim and thrash the real arm.

The sim actuator is a **saturated PD position servo** whose parameters are
identified from the real hardware:

| Parameter | Meaning | How it gets identified |
|---|---|---|
| `kp`, `kv` | Stiffness of the internal loop | Step response: command a 20 degree step, film at 240 fps, fit the rise time and overshoot |
| `forcerange` | Stall torque ceiling, about 0.98 N.m at 6 V | Datasheet, then verified with a lever and a scale |
| `armature` | Reflected gearbox inertia | Fit from the same step response. Non zero armature is what stops a geared servo from behaving like a free joint |
| `damping`, `frictionloss` | Losses in the gear train | Fit from decay of a small oscillation |
| Rate limit | 2.5 rad/s, imposed by the driver by project rule | Applied in sim too, so the policy never learns motions the driver will refuse |
| **Latency** | Command to motion delay: ROS tick plus I2C plus servo | Measured with a high speed video against a logged timestamp. Injected in sim as an action delay buffer |
| **Backlash** | Aluminum joints plus 25T spline slop, easily a degree | Measured per joint by hand at the tool tip. Randomized in sim, never assumed zero |

The last two are the ones hobby projects skip, and they are the two that
break transfer on this class of hardware.

**Domain randomization list** (applied at env reset): link masses +/- 15 %,
`kp` +/- 20 %, friction and damping +/- 30 %, latency 0 to 60 ms,
backlash 0 to 1.5 degrees, camera extrinsics +/- 5 mm and 1 degree, target
object mass and friction, and lighting when pixels are used.

**No force feedback and no joint encoders.** The real robot cannot measure
what it is doing. Every policy must therefore be robust to open loop
execution, and the observation space in section 5 reflects that honestly.

---

## 5. The learning track, task by task

Tasks are ordered so each one validates a piece of the pipeline before the
next depends on it.

### L1. Model and system identification (unblocks everything)

Deliverable: `soma_description/mjcf/soma_arm.xml` plus
`scripts/urdf_to_mjcf.py`, plus an identification report comparing sim and
real step responses.

Acceptance: for a commanded 30 degree step on the shoulder, sim and real
agree on rise time within 15 % and on final position within 2 degrees.
Until that holds, no policy trained in this sim means anything.

### L2. Reach (the pipeline test)

- **Observation**: joint positions (commanded, since there are no
  encoders), previous action, target XYZ in the arm base frame.
- **Action**: joint position deltas for 5 joints, clipped to the driver
  rate limit.
- **Reward**: negative distance from tool0 to target, action smoothness
  penalty, self collision penalty, plus a bonus inside 2 cm.
- **Why first**: it exercises the whole chain (MJCF, randomization,
  training on the Mac, export, inference on the Jetson, driver, real arm)
  on a task where failure costs nothing.
- **Acceptance**: policy trained only in sim reaches a real target within
  3 cm, ten times out of ten, without touching anything it should not.

### L3. Grasp and place a cube (the real task)

- **Observation**: L2 plus cube pose (from ArUco during training and
  evaluation, so ground truth is available), and gripper state.
- **Action**: L2 plus gripper command.
- **Reward**: staged. Approach, then contact with both fingers, then lift
  above a height threshold, then place inside the box footprint, with
  penalties for dropping and for pressing into the table.
- **Honest expectation**: contact rich grasping with no force sensing and
  about 330 g of payload is genuinely hard. Plan for a **residual policy**
  on top of a scripted approach primitive, rather than pure end to end.
  That is also the more interesting result to write up.

### L4. Bimanual handoff (stretch, only if L3 lands)

One arm picks, both arms meet at a fixed pose, the other receives. This is
the task that justifies two arms and the one nobody expects from a home
built robot.

### L5. Write-up

Sim to real transfer of position controlled hobby servos, with measured
backlash and latency, on a sub 400 g payload arm. The negative results are
publishable too, and the Alpha 1S thesis already established the format.

---

## 6. The demo track, phase by phase

Each phase ends with a git tag, a short video, and CI green.

### v0.1 Calibrated model

Blocked only by physical measurement, which is Andres and a caliper.

1. Capture zeros and mechanical limits for the 13 actuators with
   `scripts/servo_workbench.py`. Output: `servo_calibration.json`.
2. Re-center the horns: with the servo held at 1500 us, unbolt the horn
   and re-spline it so the mechanical zero of each joint matches the
   electrical center. The 25T spline gives 14.4 degree steps; whatever is
   left over becomes a software offset.
3. Caliper the 14 arm dimensions into `dimensions.yaml`, status
   `measured`.
4. Weigh each subassembly, recompute inertias, re-verify the lifted mass
   budget (4 kg rule) against real numbers.
5. Fold the measured zeros and ranges into `SERVO_MAP` as per joint
   offsets and limits, with tests.

**Acceptance**: RViz pose matches a photograph of the real arm in the same
pose, `check_model_driver_sync.py` passes, CI green, tag `v0.1`.

### v0.2 Real driver

The code already exists and is tested. This phase is the hardware run.

1. Bring-up order from `docs/wiring.md`, arms compact and resting.
2. `allow_real:=true`, then an explicit `/soma/arm` call, then a single
   joint moved through `/soma/command`.
3. Add motion primitives to the driver: `go_pose(joint targets)`,
   `open_gripper()`, `close_gripper()`, `home()`, `relax()`. These become
   the functions the ER agent is allowed to call in v0.4.
4. A `demo_pose_sequence.py` that plays a safe choreography, which is also
   the video for this tag.

**Acceptance**: both arms execute a scripted sequence end to end under the
ramp, with the emergency switch never needed. Tag `v0.2`.

### v0.3 Eye-hand (the hard one)

This is the phase that makes everything after it possible.

1. **Rigid rig**: both arms bolted to one board at a measured separation,
   OAK-D on a fixed mast looking down at the work area. New model
   `soma_rig.urdf.xacro` describing exactly that geometry. Without a rigid
   rig, extrinsics die every time something shifts.
2. **Deprojection**: pixel plus depth to XYZ in the camera frame, using
   the factory intrinsics from `device.readCalibration()`.
3. **Extrinsics camera to arm base**: an ArUco marker on the gripper,
   moved to N known joint configurations; solve the transform. Store it in
   `dimensions.yaml` with provenance, publish it as a static TF.
4. **IK**: MoveIt with the existing SRDF, KDL first, TRAC-IK if KDL
   struggles near singularities.
5. **Acceptance, and it is deliberately brutal**: click a point in the
   camera image and the tool tip touches that point within 1 cm, from ten
   different points across the work area. Repeat after a power cycle to
   prove the calibration persists.

Tag `v0.3`. Video: the click to touch loop. That alone is a good short.

### v0.4 Language directed manipulation

1. `soma_agent` package: a client for `gemini-robotics-er-2-preview`.
   Pointing queries return normalized `[y, x]` in a 0 to 1000 space with a
   label; combine with depth to get XYZ, then reuse the v0.3 pipeline.
2. Function calling: expose exactly the v0.2 primitives, nothing more. The
   model plans, decomposes and calls them. **It cannot arm the driver,
   change a limit, or bypass the ramp**, and there is a test asserting the
   tool schema contains no such capability.
3. Cycle verification: use the model's progress classification to decide
   whether a cycle succeeded, and log it.
4. Demo: both arms clearing the ArUco cubes into a box from a spoken or
   typed instruction, with the plan visible on screen.

Tag `v0.4`. This is the loud one.

### v1.0 The operator

Two hours of continuous cycles, a public counter verified by the model,
thermal and current behavior logged, failures counted honestly. Then the
project closes and the arms move to Waver. Anything missing becomes an
issue, not a reason to keep it open.

---

## 7. Sequencing, and what unblocks what

```
v0.1 calibrated model ──► v0.2 driver ──► v0.3 eye-hand ──► v0.4 ER 2 ──► v1.0
        │                                      ▲
        │                                      │
        └──► L1 MJCF + system ID ──► L2 reach ─┘ (policy runs on the same rig)
                                        │
                                        └──► L3 grasp ──► L4 bimanual ──► L5 write-up
```

- The rigid rig (a board, bolts and a mast) can be built any evening and
  is a hard prerequisite for both v0.3 and any policy evaluation on real
  hardware. Build it early.
- L1 needs only v0.1 (a measured model) plus a couple of step response
  videos, so the learning track can start while the demo track is doing
  hardware runs.
- L2 evaluation on the real robot needs v0.2 and the rig, not v0.3.

Rough effort, in evening sessions of two to three hours: v0.1 two to
three, v0.2 one to two, rig one, v0.3 four to six, v0.4 three to four,
L1 two to three, L2 three to five, L3 open ended.

---

## 8. Invariants that survive every phase

These do not bend for a demo, a deadline or an agent.

1. Motors move only after explicit prior confirmation, per session.
2. The driver boots on the mock backend and disarmed. Two independent
   gates, both closed by default.
3. No command is ever held against a mechanical stop. Soft limits,
   clamping on arrival, and auto release on the self locking joint.
4. Every motion is ramped, in sim and on hardware, with the same numbers.
5. The 6 V rail is energised only with the arms compact and resting, and
   the switch on that rail is the real emergency stop.
6. The cloud reasoner proposes; the armed driver disposes.
7. A safety rule without a test is a rule that will be refactored away.

---

## 9. Risks, named before they bite

| Risk | Impact | Mitigation |
|---|---|---|
| Payload is about 330 g at 30 cm | Limits every task | Light targets (cardboard cubes), compact poses, shoulder and base are the first servos to upgrade if needed |
| No encoders, no force sensing | Open loop everything; policies cannot correct | Position based policies, generous randomization, vision as the only feedback |
| Backlash in aluminum joints | Repeatability worse than the model suggests | Measure it, randomize it, and report the real repeatability number instead of hiding it |
| MG996R clones vary unit to unit | One servo behaves unlike its twin | Per channel calibration already in the workflow; identify gains per joint, not per model |
| Single UBEC shared by twelve servos | Brownouts under simultaneous load | Second UBEC planned, one per arm; sequence motions rather than moving everything at once |
| ER 2 is a preview API | Interface may change | Keep the client behind a thin adapter with its own tests; the robot must work without it |
| Cloud dependency in a demo | A network hiccup ruins the video | Cache plans, and keep a scripted fallback sequence for the recording |
| Training time on an M3 Pro | Slow iteration | Start with state based tasks and small networks; consider a rented GPU only for a final run |
| Scope creep past v1.0 | The project never closes | The closure rule is in the README and in CLAUDE.md: at v1.0 it ends |

---

## 10. Immediate next actions

1. Finish zeros and mechanical limits with the workbench (in progress).
2. Re-center the horns at 1500 us, one servo at a time.
3. Caliper session when the tool arrives, fill `dimensions.yaml`, tag
   `v0.1`.
4. Build the rigid rig (board, bolts, camera mast) and measure the arm
   separation for `soma_rig.urdf.xacro`.
5. Record the step response videos for L1 while the arms are already
   powered and instrumented.
