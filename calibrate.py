#!/usr/bin/env python3
"""Probe the plate at the grid points to find its real height (port of callibrate_heights).

    python3 calibrate.py                    # probe every point (e.g. a sagging cantilevered beam)
    python3 calibrate.py --probe edges      # probe the outer ring, interpolate the inside
    python3 calibrate.py --probe corners    # probe the 4 corners, interpolate the rest
                                            #   (flat plate; tilted or twisted is fine)
    python3 calibrate.py --start 5          # resume from point 5 (keeps earlier results)

Unprobed points get their height from the probed ones: straight lines between the
probed corners/edge points, blended across the plate (a Coons patch). With only the
corners that is a flat-or-twisted surface; a plate that bends between them needs
edges or all.

At each point: start START_ABOVE (6 mm) over the planned height, then a coarse
search (1 mm steps), then a fine search (0.2 mm steps). Planned heights follow the
corners (see plan_grid.py), so a tilted or cantilevered plate works if all corner
heights were saved ~1 mm above the surface.
Each point's result is saved to state.json as soon as it is measured, so you
can Ctrl-C and resume. Needs the Robot Control Panel open with the tool connected.

Without a working probe:
    python3 calibrate.py --no-probe               # hammer 10 mm above the planned heights
    python3 calibrate.py --no-probe --height 2    # hammer 2 mm above the planned heights
This does not move the arm or use the tool. It trusts the planned heights (from
the corners, see plan_grid.py) and just applies the hammer offset to the planned points.
"""
import argparse
import time

from urcommon import (HIT_Z_OFFSET, Arm, Tool, fmt_mm, hammer_offset, load_state,
                      save_state)

COARSE_STEP = 0.001    # m
COARSE_MAX = 0.015     # m, give up if no contact after descending this far
START_ABOVE = 0.006    # m, start each search this far above the planned height
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
    """Step down until the probe reports 'sensed'. Returns the z at contact.

    Every search starts above the surface (START_ABOVE, or 1.5 mm above the coarse
    contact), so 'sensed' on the first step means the probe triggered in the air.
    """
    travelled = 0.0
    while travelled < limit:
        arm.move_by(dz=-step)
        travelled += step
        if tool.wait_for("sensor_status", "sensed", SETTLE):
            if travelled <= step:
                arm.move_by(dz=0.005)
                raise SystemExit("Probe reported 'sensed' on the first step, before it could reach the "
                                 "plate. It is triggering by itself (alarm?): check the probe, re-arm "
                                 "it with tool.py probe, and resume with --start.")
            return arm.xyz()[2]
    raise SystemExit(f"No contact after descending {limit * 1000:.1f} mm; stopping.")


def probe_point(arm, tool, point, rot):
    arm.travel([point[0], point[1], point[2] + START_ABOVE], rot=rot)
    deploy_probe(arm, tool, raise_by=0.001, wait=1.0, lower_by=0.001)
    descend_until_contact(arm, tool, COARSE_STEP, COARSE_MAX)
    deploy_probe(arm, tool, raise_by=0.003, wait=2.0, lower_by=0.0015)
    return descend_until_contact(arm, tool, FINE_STEP, FINE_MAX)


def grid_index(k, nx, zigzag):
    """(column, row) of point index k (0-based), undoing --zigzag's reversed rows."""
    j, i = divmod(k, nx)
    if zigzag and j % 2:
        i = nx - 1 - i
    return i, j


def points_to_probe(which, nx, ny, zigzag):
    """0-based indices of the points to probe for --probe corners / edges / all."""
    keep = []
    for k in range(nx * ny):
        i, j = grid_index(k, nx, zigzag)
        on_x, on_y = i in (0, nx - 1), j in (0, ny - 1)
        if which == "all" or (which == "edges" and (on_x or on_y)) or (on_x and on_y):
            keep.append(k)
    return keep


def fill_heights(surface, nx, ny, zigzag):
    """Fill missing surface heights from the probed edge (or corner) points.

    Coons patch: blends the four edges across the plate; with only the corners
    probed, the edges are straight lines between them, so this is bilinear.
    """
    z = {grid_index(k, nx, zigzag): s for k, s in enumerate(surface)}

    def edge(i, j):
        """Height on the edge, straight between its two corners where not probed."""
        if z.get((i, j)) is not None:
            return z[(i, j)]
        if i in (0, nx - 1):
            v = j / (ny - 1)
            return (1 - v) * z[(i, 0)] + v * z[(i, ny - 1)]
        u = i / (nx - 1)
        return (1 - u) * z[(0, j)] + u * z[(nx - 1, j)]

    filled = []
    for k, s in enumerate(surface):
        if s is not None:
            filled.append(s)
            continue
        i, j = grid_index(k, nx, zigzag)
        u, v = i / (nx - 1), j / (ny - 1)
        corners = ((1 - u) * (1 - v) * z[(0, 0)] + u * (1 - v) * z[(nx - 1, 0)]
                   + (1 - u) * v * z[(0, ny - 1)] + u * v * z[(nx - 1, ny - 1)])
        filled.append((1 - u) * edge(0, j) + u * edge(nx - 1, j)
                      + (1 - v) * edge(i, 0) + v * edge(i, ny - 1) - corners)
    return filled


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--probe", choices=("all", "edges", "corners"), default="all",
                        help="which points to probe; the rest are interpolated (default all)")
    parser.add_argument("--start", type=int, default=1, help="first point to probe (1-based)")
    parser.add_argument("--no-probe", action="store_true",
                        help="skip probing; use the planned heights")
    parser.add_argument("--height", type=float, default=10.0,
                        help="with --no-probe: hammer height above the planned heights, in mm (default 10)")
    args = parser.parse_args()

    state = load_state()
    if "grid" not in state:
        raise SystemExit("No grid yet. Run plan_grid.py first.")
    points = state["grid"]["points"]
    rot = state["rotation"]
    ox, oy = hammer_offset(rot)
    print(f"hammer offset from probe for this tool orientation: x {ox * 1000:+.1f} mm, y {oy * 1000:+.1f} mm")
    if not 1 <= args.start <= len(points):
        raise SystemExit(f"--start must be 1-{len(points)}")
    calibrated = state.get("calibrated") or [None] * len(points)
    if not state.get("calibration_mode", "").startswith("probed"):
        calibrated = [None] * len(points)  # don't mix probed and --no-probe points
    for k in state.get("interpolated", []):
        calibrated[k - 1] = None  # only real probes carry over between runs

    if args.no_probe:
        state["calibrated"] = [[p[0] + ox, p[1] + oy,
                                p[2] + args.height / 1000] for p in points]
        state["calibration_mode"] = f"no-probe, {args.height:g} mm above planned heights"
        state.pop("interpolated", None)
        save_state(state)
        for i, p in enumerate(state["calibrated"], 1):
            print(f"  point {i:2d} hammer at {fmt_mm(p)}")
        print(f"Saved {len(points)} points without probing, hammer {args.height:g} mm above "
              "the planned heights. Check one with: python3 goto.py 1 --hammer --touch")
        return

    nx, ny = state["grid"]["size"]
    zigzag = state["grid"].get("zigzag", False)
    todo = [k for k in points_to_probe(args.probe, nx, ny, zigzag) if k >= args.start - 1]
    if not todo:
        raise SystemExit(f"No {args.probe} points at or after point {args.start}.")
    print(f"Probing {args.probe} points {[k + 1 for k in todo]}"
          + ("" if args.probe == "all" else "; the rest will be interpolated")
          + ". The arm will move to each point and step down until the probe touches.")
    input("Slider low, area clear, hand near e-stop, control panel connected. Enter to start, Ctrl-C to cancel: ")

    tool = Tool()
    arm = Arm("calibrate")
    i = todo[0]
    try:
        for i in todo:
            p = points[i]
            print(f"point {i + 1}/{len(points)}: planned {fmt_mm(p)}")
            z = probe_point(arm, tool, p, rot)
            print(f"  surface z {z * 1000:.2f} mm ({(z - p[2]) * 1000:+.2f} mm from plan)")
            calibrated[i] = [p[0] + ox, p[1] + oy, z + HIT_Z_OFFSET]
            state["calibrated"] = calibrated
            state["calibration_mode"] = "probed"
            state.pop("interpolated", None)
            save_state(state)
        arm.move_by(dz=0.010)

        missing = [k for k in range(len(points)) if calibrated[k] is None]
        needed = points_to_probe(args.probe, nx, ny, zigzag)
        if missing and all(calibrated[k] is not None for k in needed):
            surface = [None if c is None else c[2] - HIT_Z_OFFSET for c in calibrated]
            filled = fill_heights(surface, nx, ny, zigzag)
            for k in missing:
                p = points[k]
                calibrated[k] = [p[0] + ox, p[1] + oy, filled[k] + HIT_Z_OFFSET]
                print(f"  point {k + 1:2d} interpolated surface z {filled[k] * 1000:.2f} mm "
                      f"({(filled[k] - p[2]) * 1000:+.2f} mm from plan)")
            state["calibrated"] = calibrated
            state["calibration_mode"] = f"probed {args.probe}, rest interpolated"
            state["interpolated"] = [k + 1 for k in missing]
            save_state(state)
        elif missing:
            print(f"Points {[k + 1 for k in missing]} are still not calibrated; "
                  f"probe them with --start, or use --probe edges / corners.")
            return
        print("Calibration done. Next: python3 run_test.py")
    except KeyboardInterrupt:
        print(f"\nStopped. Resume with: python3 calibrate.py --probe {args.probe} --start {i + 1}")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
