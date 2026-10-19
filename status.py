#!/usr/bin/env python3
"""Show the arm position and how far setup has got. Does not move the arm.

    python3 status.py
    python3 status.py --reset    # delete state.json (corners, grid, calibration)
"""
import os
import sys

from urcommon import STATE_FILE, Arm, fmt_mm, load_state


def main():
    if "--reset" in sys.argv:
        if os.path.exists(STATE_FILE):
            os.remove(STATE_FILE)
        print("state cleared")
        return

    arm = Arm("status")
    try:
        print("joints (rad):", [round(q, 4) for q in arm.joints()])
        print("tool:        ", fmt_mm(arm.xyz()))
    finally:
        arm.close()

    state = load_state()
    for key in ("corner1", "corner3"):
        print(f"{key}:     ", fmt_mm(state[key]) if key in state else "not saved")
    grid = state.get("grid")
    print("grid:        ", f"{grid['size'][0]} x {grid['size'][1]}" if grid else "not planned")
    cal = state.get("calibrated")
    if cal:
        print(f"calibrated:   {sum(p is not None for p in cal)}/{len(cal)} points "
              f"({state.get('calibration_mode', 'probed')})")
    else:
        print("calibrated:   no")


if __name__ == "__main__":
    main()
