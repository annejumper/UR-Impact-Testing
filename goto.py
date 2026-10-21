#!/usr/bin/env python3
"""Move to a grid point to check it, without probing or hitting.

    python3 goto.py 5              # probe over planned point 5, LIFT (10 mm) above it
    python3 goto.py 5 --hammer     # hammer over calibrated point 5, LIFT above it
    python3 goto.py 5 --hammer --touch   # hammer at the calibrated hit height
    python3 goto.py c1             # back to corner 1
"""
import argparse

from urcommon import LIFT, Arm, fmt_mm, load_state


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("point", help="point number, or c1 / c2 / c3")
    parser.add_argument("--hammer", action="store_true", help="use calibrated hammer positions")
    parser.add_argument("--touch", action="store_true", help="go all the way down (no LIFT)")
    args = parser.parse_args()

    state = load_state()
    if args.point in ("c1", "c2", "c3"):
        key = "corner" + args.point[1]
        if key not in state:
            raise SystemExit(f"{key} not saved")
        target = list(state[key])
    else:
        key = "calibrated" if args.hammer else "grid"
        if not state.get(key):
            raise SystemExit("Run calibrate.py first." if args.hammer else "Run plan_grid.py first.")
        points = state[key] if args.hammer else state["grid"]["points"]
        n = int(args.point)
        if not 1 <= n <= len(points) or points[n - 1] is None:
            raise SystemExit(f"point {n} not available")
        target = list(points[n - 1])
    if not args.touch:
        target[2] += LIFT

    print("target", fmt_mm(target))
    input("Slider low, area clear, hand near e-stop. Enter to move, Ctrl-C to cancel: ")
    arm = Arm("goto")
    try:
        arm.travel(target, rot=state.get("rotation"))
        print("now at", fmt_mm(arm.xyz()))
    except KeyboardInterrupt:
        print()
    finally:
        arm.close()


if __name__ == "__main__":
    main()
