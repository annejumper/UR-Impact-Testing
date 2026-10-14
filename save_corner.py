#!/usr/bin/env python3
"""Record the current tool position as corner 1 or corner 3 of the impact grid.

    python3 save_corner.py 1     # probe extended, ~1 mm above the FIRST impact point
    python3 save_corner.py 3     # probe over the LAST impact point (diagonal; height ignored)

Corner 1 also stores the tool orientation used for the whole test.
Saving a corner clears any planned grid and calibration.
"""
import sys

from urcommon import Arm, fmt_mm, load_state, save_state


def save_corner(arm, which):
    pose = arm.pose()
    state = load_state()
    state[f"corner{which}"] = pose[:, 3].tolist()
    if which == 1:
        state["rotation"] = pose[:, :3].tolist()
    for key in ("grid", "calibrated", "calibration_mode"):
        state.pop(key, None)
    save_state(state)
    print(f"corner {which} saved at", fmt_mm(state[f'corner{which}']))
    if "corner1" in state and "corner3" in state:
        print("Both corners set. Next: python3 plan_grid.py N")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("1", "3"):
        raise SystemExit(__doc__)
    arm = Arm("save_corner")
    try:
        save_corner(arm, int(sys.argv[1]))
    finally:
        arm.close()


if __name__ == "__main__":
    main()
