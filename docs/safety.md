# SOMA safety model

SOMA is about 1.8 kg of aluminum on twelve servos with no position feedback,
plus a linear actuator that can push 100 N. The rules below are not
aspirations. Each one is enforced by code, and each one is covered by a test.

## The rules

### 1. Never move a motor without explicit prior confirmation

No script, no agent and no automation energises a servo because it seemed
like the next reasonable step. Motion happens when the operator has said so,
in advance, for that specific action.

**Enforced by two independent gates**, and both must be open:

| Gate | What it is | Default |
|---|---|---|
| `allow_real` | a launch parameter, decided when the node starts | `false` |
| `/soma/arm` | a service call, made deliberately while it runs | disarmed |

`RealPca9685.__init__` raises `PermissionError` unless it is constructed with
`armed=True`, so even importing and instantiating it by accident cannot emit a
pulse. Tested by `test_real_backend_without_arming_is_impossible`.

### 2. The node boots on the mock backend and disarmed

`ArmController` always constructs `MockPca9685` first. There is no
configuration that starts it live. The mock records the pulses that would have
been sent, publishes `/joint_states` exactly as the real backend would, and
touches no hardware, so RViz, MoveIt and the whole pipeline can be developed
and demonstrated with zero risk.

Arming is a transition, never an initial state:

```bash
# 1. start it (safe, mock, disarmed) but permitted to arm later
ros2 launch soma_driver driver.launch.py allow_real:=true

# 2. only after Andres has confirmed, out loud, for this specific run:
ros2 service call /soma/arm std_srvs/srv/SetBool "{data: true}"

# 3. cut the signal at any time
ros2 service call /soma/arm std_srvs/srv/SetBool "{data: false}"
```

Calling `/soma/arm` on a node started without `allow_real:=true` is **refused**
and logged. Disarming cuts every channel, drops back to the mock, and always
succeeds. Arming holds the current pose: the arming call itself never causes
motion, since a fresh command has to arrive first.

Tested by `test_boots_on_mock_and_disarmed`,
`test_arming_is_refused_without_allow_real` and
`test_disarm_always_succeeds_and_returns_to_mock`.

### 3. Never hold a command against a physical stop

This rule was written in blood, or at least in a wedged lead screw. On
2026-07-22 a retract command held for about 8 seconds against an already
retracted L16 jammed the screw against its stop. It had to be freed by hand.

Three mechanisms, all active at once:

- **Soft limits.** The lift is commandable over **5 to 135 mm**, never
  0 to 140. Saturation happens in `ServoSpec.command_to_us`, so it applies to
  every path into the hardware. The URDF carries the same limits, so planners
  cannot aim at a stop either, and CI compares the two
  (`scripts/check_model_driver_sync.py`).
- **Clamping on arrival.** Incoming targets are clamped when they are
  received, so `/joint_states` never reports a pose the hardware is not
  allowed to reach.
- **Auto release.** After 0.5 s settled on target, the driver cuts the PWM on
  the lift channel. The L16 lead screw is self locking and holds 46 N with no
  power, so the load stays put and there is no signal left to grind.

**Auto release applies only to self locking joints.** Arm servos hold their
position with active torque; cut their signal and the arm falls. That is why
`RELEASE_WHEN_SETTLED` contains exactly one joint, and why a test asserts
exactly that.

### 4. Every motion is ramped

`rate_limit` moves each joint toward its target at no more than its own
maximum rate: 2.5 rad/s for arm servos, well below what an MG996R could do,
and 0.020 m/s for the lift, which is what the L16 actually does. There is no
code path that jumps a servo straight to a new position.

## What the safety model does not cover

Being honest about this matters more than the list above.

- **No position feedback.** RC servos report nothing. On startup the driver
  assumes every joint is at zero, and it can be wrong. The ramp, the soft
  limits and the arming gates exist precisely because the true pose is
  unknown.
- **No force sensing.** Nothing detects a collision or a stalled servo. A
  stalled MG996R draws heavy current and gets hot.
- **No emergency stop in software worth trusting.** `/soma/arm false` cuts the
  signal, but it needs a working node, a working bus and a working Pi. **The
  real emergency stop is the switch on the 6 V rail.** Keep it in reach, and
  keep a hand near it whenever the arms are armed.

## Incident log

| Date | What happened | What changed |
|---|---|---|
| 2026-07-22 | L16 wedged against its stop by a held retract command | soft limits 5 to 135 mm, auto release after 0.5 s settled |
| 2026-07-22 | I2C dropped four times, `OSError 121` | `retry_i2c`, three attempts 5 ms apart |
| 2026-07-22 | Channel 0 suspected during the fault, never cleared | L16 moved to channel 3, channel 0 out of service |

New incidents belong in this table, with the code change that came out of
them. An incident with no code change is an incident that will happen again.
