# Next session: where 2026-10-13 left off

The record is `docs/log.md` (2026-10-12 and 2026-10-13 entries). This is
the short list to pick up from; delete it when its items are done and
logged.

## State

- `main` green in CI. Servos kept (MG996R, decided 2026-10-13): preload
  first; a servo is bought only if the shoulder cycle survives the preload
  or the base passes 60 C in the soak.
- Jetson: driver container boots with gate 1 OPEN (`allow_real:=true`),
  DISARMED. Close it when no motion is planned:
  `bash scripts/jetson_driver.sh closed`.
- Camera: on the mast above the central box, frames turned 90 deg in
  software. The coverage check showed the arms' forward arc leaving the
  frame on the right: the camera must be turned toward the robot.
- Every joint's radians are still the nominal 2000 us = 180 deg; the
  elbow jam of 2026-10-12 is the first evidence they differ.

## Pending, in order

1. Camera: validated on a tripod on 2026-10-26 with four lane marks
   (`calibration/camera_marks_2026-10-26.json`). Re-aim against
   `docs/media/camera/2026-10-26/reference_tripod.jpg`. Open: measure with
   a ruler: gripper tips 180 mm apart (2026-10-26), the model says
   124.6 mm. Likely the model's arm plane is on the disc face instead of
   the arm sandwich's mid-plane (+26.4 mm a side). Confirm with calipers,
   then correct the URDF, `workspace_map.py` and the gripper clearance.
   Also check which way the arms' forward points relative to the tripod:
   the raised arms were not in view.
2. Measure magnitudes with `soma_joint_probe` (written 2026-10-26, not
   yet run on the metal): elbows first with `--most 0.6` watching the
   central box, then bases, wrist pitches, shoulders; `--report` into
   `calibration/`. Then feed the measured scales into
   `apply_calibration.py` and the xacro.
3. Shoulder preload: measure the oscillation threshold, fit a one-sign
   band or spring (never a perfect balance).
4. Re-verify the wave poses with measured magnitudes; wave `--step`, then
   full, filmed: tag `v0.2`.
5. Desk: fix `grippers_open` / `demo` (both grippers open would collide
   in the model) with a clearance test; apply the 2026-10-12 review to
   `docs/plan.md`.
6. Cleanup, after asking: integrated worktrees, the old remote branch,
   container `soma_driver_old_8d38831`, two loose `servo_calibration*.json`
   in the Jetson's repository.
