#!/usr/bin/env python3
"""Compute the grid of impact points from the saved corners. Does not move the arm.

    python3 plan_grid.py 4            # 4 x 4 grid
    python3 plan_grid.py 15 2         # 15 points from corner 1 to corner 2, 2 rows
    python3 plan_grid.py 4 --zigzag   # reverse every other row for shorter moves
    python3 plan_grid.py 4 --square   # ignore corner 2 and assume a square plate

Point 1 is corner 1. Corners 1 and 3 are diagonal. If corner 2 (the other end of
corner 1's row) is saved, the plate can be any rectangle and tilted: point heights
follow the plane through the three corners, so save all three ~1 mm above the surface.
Otherwise the plate is assumed square and flat at corner 1's height.
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


def corners_from_three(first, second, last):
    """All 4 corners of a rectangle from corner 1, the end of its row, and the diagonal corner.

    Heights are kept, so the grid follows the plane through the three corners.
    """
    p4 = [f + l - s for f, s, l in zip(first, second, last)]
    return [list(first), list(second), list(last), p4]


def corner_angle(first, second, last):
    """Angle at corner 2 in degrees (90 for a rectangle)."""
    ax, ay = first[0] - second[0], first[1] - second[1]
    bx, by = last[0] - second[0], last[1] - second[1]
    return math.degrees(math.atan2(abs(ax * by - ay * bx), ax * bx + ay * by))


def grid_points(corners, nx, ny, zigzag=False):
    """Evenly spaced points; corners = (bottom left, bottom right, top right, top left)."""
    c0 = corners[0]
    b = [(c1 - c) / (nx - 1) for c, c1 in zip(c0, corners[1])]
    s = [(c3 - c) / (ny - 1) for c, c3 in zip(c0, corners[3])]
    points = []
    for j in range(ny):
        row = [[c + bk * i + sk * j for c, bk, sk in zip(c0, b, s)] for i in range(nx)]
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
    parser.add_argument("--square", action="store_true",
                        help="ignore corner 2 and assume a square plate")
    args = parser.parse_args()
    ny = args.ny or args.nx
    if args.nx < 2 or ny < 2:
        raise SystemExit("need at least 2 points per side")

    state = load_state()
    if "corner1" not in state or "corner3" not in state:
        raise SystemExit("Save corners 1 and 3 first (save_corner.py 1 / 3).")
    c1, c3 = state["corner1"], state["corner3"]
    c2 = None if args.square else state.get("corner2")
    if c2:
        angle = corner_angle(c1, c2, c3)
        print(f"rectangle: {math.dist(c1[:2], c2[:2]) * 1000:.1f} mm (corner 1 to 2, {args.nx} points) "
              f"x {math.dist(c2[:2], c3[:2]) * 1000:.1f} mm (corner 2 to 3, {ny} rows)")
        if abs(angle - 90) > 5:
            print(f"WARNING: angle at corner 2 is {angle:.1f} deg, not 90. "
                  "Is corner 2 from this plate? Re-save it, or use --square.")
        corners = corners_from_three(c1, c2, c3)
    else:
        diag = math.dist(c1[:2], c3[:2])
        print(f"square: diagonal {diag * 1000:.1f} mm, side {diag / math.sqrt(2) * 1000:.1f} mm")
        corners = corners_from_diagonal(c1, c3)
    if c2:
        print(f"heights: corner 2 {(c2[2] - c1[2]) * 1000:+.1f} mm, corner 3 "
              f"{(c3[2] - c1[2]) * 1000:+.1f} mm from corner 1; points follow this slope")
    elif abs(c3[2] - c1[2]) > 0.005:
        print(f"note: corner 3 is {(c3[2] - c1[2]) * 1000:+.1f} mm from corner 1 in z; "
              "corner 1's height is used for all points")

    points = grid_points(corners, args.nx, ny, args.zigzag)
    for i, p in enumerate(points, 1):
        print(f"  point {i:2d}: {fmt_mm(p)}")

    state["grid"] = {"size": [args.nx, ny], "zigzag": args.zigzag,
                     "shape": "rectangle" if c2 else "square", "points": points}
    state.pop("calibrated", None)
    state.pop("calibration_mode", None)
    save_state(state)
    print(f"{len(points)} points saved. Next: python3 calibrate.py")


if __name__ == "__main__":
    main()
