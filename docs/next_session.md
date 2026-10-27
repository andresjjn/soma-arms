# Next session: where 2026-10-27 left off

The record is `docs/log.md`. Delete items here once done and logged.

## State

- `main` green. SERVO_MAP is in real radians (joint probe scales,
  2026-10-26); dimensions re-measured with a caliper on 2026-10-27.
- `v0.2` TAGGED 2026-10-27 (filmed with `hello_left`). v0.3, the IK to
  points, is open.
- Jetson: driver may be left with gate 1 open; close it when no motion
  is planned (`bash scripts/jetson_driver.sh closed`).

## Pending, in order

1. Disc center to closed fingertip, arm hanging limp (`limp_arm` or ATX
   off): about 339 mm means the arm is straight and `mount_h` changed
   (re-measure it with a square); about 296 means joints are off zero,
   then the per-screw heights of the 2026-10-27 plan.
2. `fk()` at home must predict the measured tip height within 2 to 3 mm.
3. MOCK round trip `ik()` then `fk()` over the lanes (y = +-81 mm).
4. Real robot: tape crosses on the lanes, gripper closed, error in mm
   logged per trial (requested, reached, error) to a timestamped CSV;
   accuracy and repeatability apart. Recording set up first.
5. Grasp: approach from above, lower, close, lift, with the TCP measured
   at the grasp opening.
6. Debts: the right base stops about 119 deg (left 142); the PCA9685
   clock check (`scripts/pwm_clock_check.py`); `workspace_map.py` still
   prints the joint signs as unverified though the sign check passed.
