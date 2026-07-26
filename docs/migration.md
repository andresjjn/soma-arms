# Migration notes

SOMA was not written from scratch. Both packages were migrated from the bench
driver and digital twin that came out of the 2026-07-22 session, where the
hardware facts were established. This file records exactly what carried over
unchanged, what was renamed, and the handful of places where the migrated code
was deliberately changed.

The rule that governed the migration: **the tests are the specification.** If
something broke during the move, the migration was wrong, not the test.

## Renames

| Original | Here |
|---|---|
| `waver_arm_description` package | `soma_description` |
| `waver_arm` package | `soma_driver` |
| `waver_arm` xacro macro | `soma_arm` |
| node `waver_arm` | node `soma_driver` |
| topic `waver_arm/command` | `/soma/command` |
| service `waver_arm/arm` | `/soma/arm` |
| `arm_6dof.xacro` | `soma_arm.xacro` |
| `control.xacro` | `soma_control.xacro` |
| Spanish material names | English (`aluminio` to `aluminum`, and so on) |

Joint and link names are **unchanged**: `left_arm_*`, `right_arm_*`,
`torso_lift_joint`, `*_tool0`. Anything already written against those names
still works.

Comments, docstrings and test names were translated to English, because
everything committed here is public content in English. No assertion changed
in translation.

## Carried over unchanged

- The **channel map**, verified with power on: right arm 15 down to 10,
  left arm 9 down to 4, L16 on channel 3, channels 0 to 2 spare with 0 out of
  service.
- The **inverted L16 convention**: `min_us` 1964.3 greater than `max_us`
  1035.7.
- The **soft limits** 5 to 135 mm and their pulse anchors.
- **Auto release** after 0.5 s settled, for the lift only.
- **`retry_i2c`**: three attempts, 5 ms apart.
- Per joint **rate limits**: 2.5 rad/s for arm servos, 0.020 m/s for the lift.
- The **mimic gripper**: right fingers driven by gears, with no PWM channel.
- All **24 tests**, assertion for assertion.

## Deliberate changes

Five, each with a reason. Nothing else was touched.

### 1. Arming actually works now

**Before**: the node took a `use_mock` parameter. With `use_mock:=false` it
constructed `RealPca9685(armed=False)` in its own constructor, which raises
`PermissionError` by design. The node died at startup, so the real backend was
unreachable in practice.

**Now**: the node **always** boots on the mock. Arming is a transition made by
the `/soma/arm` service, and it is refused unless the node was also started
with `allow_real:=true`. Two independent gates.

This implements the stated rule ("boots MOCK and DISARMED, arming is an
explicit service") rather than approximating it. `PermissionError` on an
unarmed `RealPca9685` is untouched and still tested.

### 2. Targets are clamped when they arrive

**Before**: saturation happened only in `command_to_us`, so commanding the
lift to 0.140 m published `0.140` on `/joint_states` while the hardware got
the pulse for 0.135. The twin reported a pose the robot could not reach.

**Now**: incoming targets are clamped on arrival, and the startup state is
clamped too, so the lift starts at 0.005 rather than 0.0.

Consequence to be aware of: **published travel is 130 mm, not 140 mm.** That
is the honest number, and the smoke test asserts it.

### 3. URDF lift limits are the soft band

The `torso_lift_joint` limit in the URDF is **0.005 to 0.135**, not 0 to 0.140.
The physical stroke is documented in the file right above the limit.

Reason: MoveIt2 arrives at v0.3 and plans against URDF limits. With the
physical stroke in the URDF, a planner would happily aim at a mechanical stop
and the driver would silently clamp, producing a permanent tracking error at
exactly the position that wedged the actuator once already.

`scripts/check_model_driver_sync.py` runs in CI and fails if the URDF and
`servo_map.py` ever disagree about a limit, a lift speed or a mimic joint.

### 4. The rover assembly did not come along

The original description package contained the full mobile robot: chassis,
four wheels, LiDAR and sub-chassis, with the torso and arms on top.

SOMA ships the **arms and the torso only**. `soma_torso.xacro` is a macro
parameterised by parent link and origin, so a platform mounts it by calling
the macro with its own link. In its place there are two bench models:

- `single_arm.urdf.xacro`: one arm on the world frame, the model to use with
  calipers,
- `soma_bench.urdf.xacro`: bay, lift, two arms and the head, on the world
  frame. This is the reference model.

The SRDF covers SOMA links only. Collision pairs involving a platform belong
to whoever owns that platform.

FK moved accordingly. `world` to `left_arm_tool0` is **0.662 m** with the lift
down and **0.792 m** with it up. In the original full assembly, measured from
the rover's `base_footprint`, the same frame sat at 0.737 m and 0.877 m.

### 5. Tests added, none removed

24 carried over, plus:

- 3 for the soft limit clamp in `test_servo_map.py`, so the suite that needs
  no ROS now has **27**,
- 7 in `test_node_safety.py` covering both arming gates, the disarm path, the
  startup state and command clamping. These need `rclpy` and skip on a
  laptop.

CI runs 27 on plain Python and all 34 in the ROS job.

## Still living upstream

The mobile platform integration is not in this repository and should not be.
When SOMA is consumed as a dependency, the platform repository removes its own
copies of these packages and pulls SOMA in instead, then calls `soma_torso`
with its own parent link.
