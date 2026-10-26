#!/usr/bin/env python3
"""Record the current tool position as corner 1, 2 or 3 of the impact grid.

    python3 save_corner.py 1     # probe extended, ~1 mm above the FIRST impact point
    python3 save_corner.py 2     # probe ~1 mm above the LAST point of the first row
    python3 save_corner.py 3     # probe ~1 mm above the LAST impact point (diagonal)

Corner 2 is only needed when the plate is not square (e.g. a strip). With corner 2
saved, all three heights are used, so the grid can follow a tilted or cantilevered plate.
Without it, corner 3's height is ignored.
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
    for key in ("grid", "calibrated", "calibration_mode", "interpolated"):
        state.pop(key, None)
    save_state(state)
    print(f"corner {which} saved at", fmt_mm(state[f'corner{which}']))
    if "corner1" in state and "corner3" in state:
        if "corner2" in state:
            print("Corners 1, 2, 3 set (rectangle). Next: python3 plan_grid.py NX NY")
        else:
            print("Corners 1 and 3 set (square). Next: python3 plan_grid.py N "
                  "(save corner 2 for a non-square plate)")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("1", "2", "3"):
        raise SystemExit(__doc__)
    arm = Arm("save_corner")
    try:
        save_corner(arm, int(sys.argv[1]))
    finally:
        arm.close()


if __name__ == "__main__":
    main()
