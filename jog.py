#!/usr/bin/env python3
"""Jog the tool in x/y/z, like the Robot Control Panel's move buttons.

One move:      python3 jog.py +x 5        (mm; also -x, +y, -y, +z, -z)
Interactive:   python3 jog.py             (type "+x 5", "-z", "step 1", "pos",
                                           "c1", "c2", "c3", "open", "close", "q")

Orientation is held fixed. Each jog is limited to MAX_JOG (40 mm).
"""
import sys

from urcommon import GRIPPER_CLOSE, GRIPPER_OPEN, MAX_JOG, Arm, fmt_mm
from save_corner import save_corner

AXES = {"x": 0, "y": 1, "z": 2}
HELP = """commands:
  +x 5 / -y 2 / +z     jog (mm; uses the current step if no number given)
  step 1               set the default step in mm
  pos                  print the current position
  c1 / c2 / c3         save corner 1 / 2 / 3 (same as save_corner.py; c2 only for non-square plates)
  open / close         gripper
  q                    quit"""


def parse_jog(words, default_mm):
    """'+x 5' -> (axis_index, meters). Returns None if not a jog command."""
    if not words or len(words[0]) != 2 or words[0][0] not in "+-" or words[0][1] not in AXES:
        return None
    sign = 1 if words[0][0] == "+" else -1
    mm = float(words[1]) if len(words) > 1 else default_mm
    return AXES[words[0][1]], sign * abs(mm) / 1000


def jog(arm, axis, meters):
    if abs(meters) > MAX_JOG:
        print(f"ERROR: maximum jog is {MAX_JOG * 1000:.0f} mm")
        return
    delta = [0.0, 0.0, 0.0]
    delta[axis] = meters
    arm.move_by(*delta)
    print("now at", fmt_mm(arm.xyz()))


def interactive(arm):
    step = 1.0
    print(HELP)
    print("now at", fmt_mm(arm.xyz()))
    while True:
        try:
            words = input(f"jog [{step:g} mm]> ").split()
        except EOFError:
            break
        if not words:
            continue
        cmd = words[0].lower()
        try:
            parsed = parse_jog([cmd] + words[1:], step)
            if parsed:
                jog(arm, *parsed)
            elif cmd == "step" and len(words) == 2:
                step = float(words[1])
            elif cmd == "pos":
                print("now at", fmt_mm(arm.xyz()))
            elif cmd in ("c1", "c2", "c3"):
                save_corner(arm, int(cmd[1]))
            elif cmd == "open":
                arm.gripper(GRIPPER_OPEN)
            elif cmd == "close":
                arm.gripper(GRIPPER_CLOSE)
            elif cmd in ("q", "quit", "exit"):
                break
            else:
                print(HELP)
        except ValueError:
            print("bad number")
        except SystemExit as e:  # keep the session alive after a refused move
            print(e)


def main():
    args = sys.argv[1:]
    arm = Arm("jog")
    try:
        if not args:
            interactive(arm)
        else:
            parsed = parse_jog([args[0].lower()] + args[1:], 1.0) if len(args) <= 2 else None
            if not parsed:
                raise SystemExit(__doc__)
            jog(arm, *parsed)
    except KeyboardInterrupt:
        print()
    finally:
        arm.close()


if __name__ == "__main__":
    main()
