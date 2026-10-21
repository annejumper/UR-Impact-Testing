#!/usr/bin/env python3
"""Run the impact test: go to each calibrated point and fire the hammer.

    python3 run_test.py                  # 1 hit per point, press Enter before each hit
    python3 run_test.py --hits 3         # 3 hits per point
    python3 run_test.py --auto 4         # no prompts: hit every 4 s
    python3 run_test.py --start 7        # resume at point 7
    python3 run_test.py --points 1 5 9   # only these points

At the prompt: Enter = hit, r = redo the last hit, n = skip to next point,
b = go back to the previous point (its hits start over), q = quit.
After a point's last hit it waits for Enter before moving on, so that hit can be redone.
To redo a point afterwards: python3 run_test.py --points N
Each hit is appended to hits_log.csv so it can be matched with the recorded data.
A redo is logged again with the same point and hit number; the later row is the one to keep.

move_to_point() and hit() hold the test logic; this file's main() only does the
pacing. To drive the test from Obie later, call those two functions from a
channel listener instead of from input().
"""
import argparse
import csv
import os
import time
from datetime import datetime

from urcommon import HERE, LIFT, Arm, Tool, fmt_mm, load_state

LOG_FILE = os.path.join(HERE, "hits_log.csv")


def move_to_point(arm, state, number):
    """Move the hammer over calibrated point `number` (1-based)."""
    target = state["calibrated"][number - 1]
    if target is None:
        raise SystemExit(f"Point {number} was not calibrated. Run calibrate.py --start {number}")
    arm.travel(target, rot=state["rotation"])
    return target


def hit(tool, number, hit_index, target):
    """Fire one hit and log it."""
    ok = tool.hit()
    new_file = not os.path.exists(LOG_FILE)
    with open(LOG_FILE, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["time", "point", "hit", "x_mm", "y_mm", "z_mm", "confirmed"])
        w.writerow([datetime.now().isoformat(timespec="milliseconds"), number, hit_index,
                    *(round(v * 1000, 2) for v in target), ok])
    return ok


def run_point(tool, number, target, args, first):
    """Do the hits at one point. Returns +1 to go to the next point, -1 to go back one."""
    h = 1  # next hit to fire
    while True:
        if args.auto:
            if h > args.hits:
                return 1
            time.sleep(args.auto)
            ans = ""
        elif h <= args.hits:
            ans = input(f"  hit {h}/{args.hits} - Enter=hit, r=redo last hit, n=next point, "
                        "b=back a point, q=quit: ").strip().lower()
        else:
            ans = input(f"  point {number} done - Enter=next point, r=redo last hit, "
                        "b=back a point, q=quit: ").strip().lower()
        if ans == "q":
            raise KeyboardInterrupt
        if ans == "n" or (ans == "" and h > args.hits):
            return 1
        if ans == "b":
            if first:
                print("  already at the first point")
                continue
            return -1
        if ans == "r":
            if h == 1:
                print("  no hit to redo at this point yet")
                continue
            print(f"  hit {h - 1} (redo)" + ("" if hit(tool, number, h - 1, target) else " (unconfirmed)"))
        elif ans == "":
            print(f"  hit {h}" + ("" if hit(tool, number, h, target) else " (unconfirmed)"))
            h += 1
        else:
            print("  ?")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hits", type=int, default=1, help="hits per point")
    parser.add_argument("--auto", type=float, metavar="SECONDS",
                        help="fire automatically with this delay instead of prompting")
    parser.add_argument("--start", type=int, default=1, help="first point (1-based)")
    parser.add_argument("--points", type=int, nargs="+", help="run only these points")
    args = parser.parse_args()

    state = load_state()
    if not state.get("calibrated"):
        raise SystemExit("No calibration yet. Run calibrate.py first.")
    n = len(state["calibrated"])
    numbers = args.points or list(range(args.start, n + 1))
    bad = [p for p in numbers if not 1 <= p <= n]
    if bad:
        raise SystemExit(f"points must be 1-{n}, got {bad}")

    mode = state.get("calibration_mode", "probed")
    if mode != "probed":
        print(f"NOTE: heights were not probed ({mode}).")
    print(f"Testing points {numbers}, {args.hits} hit(s) each"
          + (f", auto every {args.auto:g} s" if args.auto else "") + ".")
    input("Slider low, area clear, hand near e-stop, control panel connected. Enter to start, Ctrl-C to cancel: ")

    tool = Tool()
    arm = Arm("run_test")
    number = numbers[0]
    i = 0
    try:
        while i < len(numbers):
            number = numbers[i]
            target = move_to_point(arm, state, number)
            print(f"point {number}/{n} at {fmt_mm(target)}")
            i += run_point(tool, number, target, args, first=(i == 0))
        arm.move_by(dz=LIFT)
        print("Test done.")
    except KeyboardInterrupt:
        print(f"\nStopped at point {number}. Resume with: python3 run_test.py --start {number}")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
