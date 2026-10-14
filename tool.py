#!/usr/bin/env python3
"""Test the hammer/probe and the channel link, without moving the arm.

    python3 tool.py hit       # fire the hammer once
    python3 tool.py probe     # extend the probe (touch it to see 'sensed')
    python3 tool.py listen    # print every channel message until Ctrl-C
"""
import sys
import time

from urcommon import Tool


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("hit", "probe", "listen"):
        raise SystemExit(__doc__)
    cmd = sys.argv[1]
    tool = Tool()
    print("connected to channel")
    if cmd != "listen":
        tool.send(cmd)
        print(f"sent '{cmd}', showing tool replies for 10 s (Ctrl-C to stop)")
    end = time.monotonic() + 10 if cmd != "listen" else float("inf")
    try:
        while time.monotonic() < end:
            item = tool.channel.get(timeout=0.2)
            if item is None:
                continue
            topic, value = item
            if cmd == "listen" or Tool.parse_status(topic, value):
                print(f"{topic}: {value}")
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
