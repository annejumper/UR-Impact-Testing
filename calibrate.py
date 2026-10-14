#!/usr/bin/env python3
"""Probe the plate at every grid point to find its real height (port of callibrate_heights).

    python3 calibrate.py              # all points
    python3 calibrate.py --start 5    # resume from point 5 (keeps earlier results)

At each point: a coarse search (1 mm steps), then a fine search (0.2 mm steps).
Each point's result is saved to state.json as soon as it is measured, so you
can Ctrl-C and resume. Needs the Robot Control Panel open with the tool connected.

Without a working probe:
    python3 calibrate.py --no-probe               # hammer 10 mm above corner 1's height
    python3 calibrate.py --no-probe --height 2    # hammer 2 mm above corner 1's height
This does not move the arm or use the tool. It assumes the plate is flat and
level at corner 1's height and just applies the hammer offset to the planned points.
"""
import argparse
import time

from urcommon import (HAMMER_OFFSET, HIT_Z_OFFSET, Arm, Tool, fmt_mm, load_state,
                      save_state)

COARSE_STEP = 0.001    # m
COARSE_MAX = 0.015     # m, give up if no contact after descending this far
FINE_STEP = 0.0002     # m
FINE_MAX = 0.004       # m
SETTLE = 0.5           # s, wait after each step for the probe reply to arrive over the channel


def deploy_probe(arm, tool, raise_by, wait, lower_by):
    """Raise, extend the probe, lower again, and wait until the probe reports 'sensing'."""
    arm.move_by(dz=raise_by)
    tool.send("probe")
    time.sleep(wait)
    arm.move_by(dz=-lower_by)
    # 'sensing' may have arrived during the moves; it is still in the queue.
    if not tool.wait_for("sensor_status", "sensing", 10.0):
        raise SystemExit("Probe never reported 'sensing'. Is the control panel connected to the tool?")


def descend_until_contact(arm, tool, step, limit):
    """Step down until the probe reports 'sensed'. Returns the z at contact."""
    travelled = 0.0
    while travelled < limit:
        arm.move_by(dz=-step)
        travelled += step
        if tool.wait_for("sensor_status", "sensed", SETTLE):
            return arm.xyz()[2]
    raise SystemExit(f"No contact after descending {limit * 1000:.1f} mm; stopping.")


def probe_point(arm, tool, point, rot):
    arm.travel(point, rot=rot)
    deploy_probe(arm, tool, raise_by=0.001, wait=1.0, lower_by=0.001)
    descend_until_contact(arm, tool, COARSE_STEP, COARSE_MAX)
    deploy_probe(arm, tool, raise_by=0.003, wait=2.0, lower_by=0.0015)
    return descend_until_contact(arm, tool, FINE_STEP, FINE_MAX)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--start", type=int, default=1, help="first point to probe (1-based)")
    parser.add_argument("--no-probe", action="store_true",
                        help="skip probing; use corner 1's height for every point")
    parser.add_argument("--height", type=float, default=10.0,
                        help="with --no-probe: hammer height above corner 1, in mm (default 10)")
    args = parser.parse_args()

    state = load_state()
    if "grid" not in state:
        raise SystemExit("No grid yet. Run plan_grid.py first.")
    points = state["grid"]["points"]
    rot = state["rotation"]
    if not 1 <= args.start <= len(points):
        raise SystemExit(f"--start must be 1-{len(points)}")
    calibrated = state.get("calibrated") or [None] * len(points)
    if state.get("calibration_mode") != "probed":
        calibrated = [None] * len(points)  # don't mix probed and --no-probe points

    if args.no_probe:
        state["calibrated"] = [[p[0] + HAMMER_OFFSET[0], p[1] + HAMMER_OFFSET[1],
                                p[2] + args.height / 1000] for p in points]
        state["calibration_mode"] = f"no-probe, {args.height:g} mm above corner 1"
        save_state(state)
        for i, p in enumerate(state["calibrated"], 1):
            print(f"  point {i:2d} hammer at {fmt_mm(p)}")
        print(f"Saved {len(points)} points without probing, hammer {args.height:g} mm above "
              "corner 1's height. Check one with: python3 goto.py 1 --hammer --touch")
        return

    print(f"Probing points {args.start}-{len(points)}. The arm will move to each point and "
          "step down until the probe touches.")
    input("Slider low, area clear, hand near e-stop, control panel connected. Enter to start, Ctrl-C to cancel: ")

    tool = Tool()
    arm = Arm("calibrate")
    i = args.start - 1
    try:
        for i in range(args.start - 1, len(points)):
            p = points[i]
            print(f"point {i + 1}/{len(points)}: planned {fmt_mm(p)}")
            z = probe_point(arm, tool, p, rot)
            print(f"  surface z {z * 1000:.2f} mm ({(z - p[2]) * 1000:+.2f} mm from plan)")
            calibrated[i] = [p[0] + HAMMER_OFFSET[0], p[1] + HAMMER_OFFSET[1], z + HIT_Z_OFFSET]
            state["calibrated"] = calibrated
            state["calibration_mode"] = "probed"
            save_state(state)
        arm.move_by(dz=0.010)
        print("Calibration done. Next: python3 run_test.py")
    except KeyboardInterrupt:
        print(f"\nStopped. Resume with: python3 calibrate.py --start {i + 1}")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
