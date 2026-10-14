#!/usr/bin/env python3
"""Open or close the gripper.

    python3 gripper.py open      # sized for the impact tool (GRIPPER_OPEN %)
    python3 gripper.py close     # GRIPPER_CLOSE %
    python3 gripper.py 40        # any position, 0-100 %

Needs the gripper node running (see README).
"""
import sys

from urcommon import GRIPPER_CLOSE, GRIPPER_OPEN, Arm


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    arg = sys.argv[1].lower()
    percent = {"open": GRIPPER_OPEN, "close": GRIPPER_CLOSE}.get(arg)
    if percent is None:
        try:
            percent = float(arg)
        except ValueError:
            raise SystemExit(__doc__)
    if not 0 <= percent <= 100:
        raise SystemExit("position must be 0-100")

    arm = Arm("gripper_cmd")
    try:
        arm.gripper(percent)
        print(f"gripper -> {percent:g}%")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
