#!/usr/bin/env python3
"""Export a SOMA URDF to an MJCF skeleton through MuJoCo's own parser.

Usage:
  python3 scripts/export_mjcf.py /tmp/soma_bench.urdf sim/build/soma_bench_mjcf.xml

MuJoCo loads standard URDF and converts it internally; saving the result
gives the MJCF starting point for the L1 track. Two things are KNOWN to
be missing from this automatic pass, by design of the plan:

  - actuators: URDF has none; the position servos (with the measured
    rate and effort of servo_map) are hand-authored on top.
  - the gripper gear: MuJoCo's URDF parser ignores <mimic>, so both
    fingers come out independent; the physical gear pair becomes an MJCF
    equality constraint, also hand-authored.

Until those layers land, the exported file is a passive but
geometrically faithful twin: measured lengths, parallel axes, soft
limits, masses.
"""
import sys
from pathlib import Path

import mujoco


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    urdf, out = sys.argv[1], sys.argv[2]
    model = mujoco.MjModel.from_xml_path(urdf)
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveLastXML(out, model)
    print(f'{out}: {model.njnt} joints, {model.nbody} bodies, '
          f'{model.nu} actuators (expected 0 before hand-authoring)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
