#!/usr/bin/env python3
"""Compute the grid of impact points from corner 1 and corner 3. Does not move the arm.

    python3 plan_grid.py 4            # 4 x 4 grid
    python3 plan_grid.py 4 3          # 4 points along the bottom edge, 3 rows
    python3 plan_grid.py 4 --zigzag   # reverse every other row for shorter moves

Point 1 is corner 1. Corners 1 and 3 are diagonal, and the plate is assumed square.
"""
import argparse
import math

from urcommon import fmt_mm, load_state, save_state


def corners_from_diagonal(first, last):
    """All 4 corners of a square from two diagonal corners (from ImpactTrajectory)."""
    dx, dy = last[0] - first[0], last[1] - first[1]
    cx, cy = first[0] + dx / 2, first[1] + dy / 2
    vx, vy = dx / 2, dy / 2
    p1 = [cx - vy, cy + vx, first[2]]
    p2 = [cx + vy, cy - vx, first[2]]
    return [first, p1, [last[0], last[1], first[2]], p2]


def grid_points(corners, nx, ny, zigzag=False):
    """Evenly spaced points; corners = (bottom left, bottom right, top right, top left)."""
    c0 = corners[0]
    bx, by = (corners[1][0] - c0[0]) / (nx - 1), (corners[1][1] - c0[1]) / (nx - 1)
    sx, sy = (corners[3][0] - c0[0]) / (ny - 1), (corners[3][1] - c0[1]) / (ny - 1)
    points = []
    for j in range(ny):
        row = [[c0[0] + bx * i + sx * j, c0[1] + by * i + sy * j, c0[2]] for i in range(nx)]
        if zigzag and j % 2 == 1:
            row.reverse()
        points.extend(row)
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("nx", type=int)
    parser.add_argument("ny", type=int, nargs="?")
    parser.add_argument("--zigzag", action="store_true")
    args = parser.parse_args()
    ny = args.ny or args.nx
    if args.nx < 2 or ny < 2:
        raise SystemExit("need at least 2 points per side")

    state = load_state()
    if "corner1" not in state or "corner3" not in state:
        raise SystemExit("Save both corners first (save_corner.py 1 / 3).")
    c1, c3 = state["corner1"], state["corner3"]
    diag = math.dist(c1[:2], c3[:2])
    print(f"diagonal {diag * 1000:.1f} mm, side {diag / math.sqrt(2) * 1000:.1f} mm")
    if abs(c3[2] - c1[2]) > 0.005:
        print(f"note: corner 3 is {(c3[2] - c1[2]) * 1000:+.1f} mm from corner 1 in z; "
              "corner 1's height is used for all points")

    points = grid_points(corners_from_diagonal(c1, c3), args.nx, ny, args.zigzag)
    for i, p in enumerate(points, 1):
        print(f"  point {i:2d}: {fmt_mm(p)}")

    state["grid"] = {"size": [args.nx, ny], "zigzag": args.zigzag, "points": points}
    state.pop("calibrated", None)
    state.pop("calibration_mode", None)
    save_state(state)
    print(f"{len(points)} points saved. Next: python3 calibrate.py")


if __name__ == "__main__":
    main()
