#!/usr/bin/env python3
"""Measure the probe-to-hammer offset, and re-apply it to the calibrated points.

    python3 hammer_offset.py probe    # 1. probe tip right over a mark on the plate
    python3 hammer_offset.py hammer   # 2. jog until the hammer is over the same mark

Jog between the steps with jog.py (it never turns the tool, which this relies on).
The result goes to hammer_offset.json, which calibrate.py uses from then on, turned
to match whatever tool orientation corner 1 was saved with. Re-applying shifts the
already-probed points sideways without probing again.
"""
import json
import sys

import numpy as np

from urcommon import OFFSET_FILE, Arm, fmt_mm, hammer_offset, heading, load_state, save_state

PROBE_MARK = OFFSET_FILE + ".probe"   # step 1's position, until step 2 reads it


def apply(old):
    """Swap offset `old` for the current one in the calibrated points."""
    state = load_state()
    if not state.get("calibrated"):
        print("No calibrated points yet; calibrate.py will use the new offset.")
        return
    new = hammer_offset(state["rotation"])
    for p in state["calibrated"]:
        if p is not None:
            p[0] += new[0] - old[0]
            p[1] += new[1] - old[1]
    save_state(state)
    print(f"Calibrated points moved by x {(new[0] - old[0]) * 1000:+.1f} mm, "
          f"y {(new[1] - old[1]) * 1000:+.1f} mm. Check with: python3 goto.py 1 --hammer")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("probe", "hammer"):
        raise SystemExit(__doc__)
    cmd = sys.argv[1]

    arm = Arm("hammer_offset")
    try:
        pose = arm.pose()
    finally:
        arm.close()
    xyz, rot = pose[:, 3].tolist(), pose[:, :3]

    if cmd == "probe":
        with open(PROBE_MARK, "w") as f:
            json.dump({"xyz": xyz, "heading": heading(rot)}, f)
        print("probe position saved at", fmt_mm(xyz))
        print("Now: +z, jog until the HAMMER is over the same mark, then: python3 hammer_offset.py hammer")
        return

    try:
        with open(PROBE_MARK) as f:
            mark = json.load(f)
    except FileNotFoundError:
        raise SystemExit("Run 'python3 hammer_offset.py probe' first.")
    if abs(heading(rot) - mark["heading"]) > 2:
        raise SystemExit("The tool turned between the two steps; redo both with jog.py only.")

    state = load_state()
    old = hammer_offset(state["rotation"]) if "rotation" in state else None
    offset = [xyz[0] - mark["xyz"][0], xyz[1] - mark["xyz"][1]]
    with open(OFFSET_FILE, "w") as f:
        json.dump({"offset": offset, "heading": heading(rot)}, f, indent=2)
    print(f"hammer offset: x {offset[0] * 1000:+.1f} mm, y {offset[1] * 1000:+.1f} mm "
          f"(tool heading {heading(rot):.1f} deg); saved to hammer_offset.json")
    if old is not None:
        if abs(heading(rot) - heading(np.asarray(state["rotation"]))) > 2:
            print("note: tool heading differs from corner 1's; the offset is turned to match.")
        apply(old)


if __name__ == "__main__":
    main()
