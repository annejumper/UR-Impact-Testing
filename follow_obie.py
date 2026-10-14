#!/usr/bin/env python3
"""Run an impact test driven by ObieApp: go to the node Acquire is on, hit, wait, repeat.

    python3 follow_obie.py --latest ~/obie_runs/Violin7      # newest run in that folder
    python3 follow_obie.py "~/obie_runs/Violin7/Test 3"       # a specific run
    python3 follow_obie.py --latest ~/obie_runs/Violin7 --confirm   # Enter before every hit
    python3 follow_obie.py RUN --dry-run          # print moves and hits; no arm, no hammer
    python3 follow_obie.py RUN --dry-run --simulate   # ...and record fake hits (fake_obie_run.py runs)

Before: points planned (plan_stencil.py, or corners + plan_grid.py) and calibrated
(calibrate.py), sync_to_arm.sh running on the Mac, recording started in Acquire.

For each node: move the hammer over it, fire, then wait until that position's hit
count in the TRF file goes up (Acquire recorded the hit) before firing again. When
Acquire has enough hits it moves on, and so does the arm. If the count doesn't change
within --timeout seconds, it stops and asks instead of firing again.

Node id N is point N+1 (ObieApp position N+1). Points from plan_stencil.py must come
from the same stencil as the run; points from plan_grid.py must match the node count,
with corner 1 over the stencil's node 1 and no --zigzag.
"""
import argparse
import os
import time
from pathlib import Path

from obiewatch import RunWatcher, is_run, latest_run, read_template, stencil_fingerprint, stencil_nodes
from urcommon import LIFT, fmt_mm, load_state

POLL = 0.5          # s between file checks
GAP = 1.0           # s, default wait after a hit is recorded before the next one
TIMEOUT = 10.0      # s, default wait for Acquire to record a hit


def point_lookup(state, run, total, quiet=False):
    """Map node id -> (point number, target) after checking the plan fits this run."""
    grid = state.get("grid") or {}
    targets = state.get("calibrated")
    if not targets:
        raise SystemExit("No calibration yet. Run calibrate.py first.")
    if len(targets) != total:
        raise SystemExit(f"The run has {total} positions but {len(targets)} points are planned.")
    nodes = stencil_nodes(read_template(run))
    planned = grid.get("stencil")
    if planned:
        if not nodes:
            raise SystemExit("Points were planned from a stencil, but this run has no stencil.")
        if stencil_fingerprint(nodes) != planned["fingerprint"]:
            raise SystemExit("This run's stencil is not the one the points were planned from "
                             "(nodes moved, or a different stencil). Re-run plan_stencil.py.")
        ids = planned["ids"]
    else:
        if not quiet:
            print("NOTE: points come from plan_grid.py, matched by index. Corner 1 must be over "
                  "the stencil's node 1, rows in ObieApp's order, no --zigzag.")
        if grid.get("zigzag"):
            raise SystemExit("The grid was planned with --zigzag, which doesn't match ObieApp's order.")
        ids = list(range(total))
    missing = [i + 1 for i, t in enumerate(targets) if t is None]
    if missing:
        raise SystemExit(f"Points {missing} were not calibrated. Run calibrate.py --start {missing[0]}")
    return {node_id: (i + 1, targets[i]) for i, node_id in enumerate(ids)}


class Rig:
    """The arm and hammer, or a stand-in that only prints (dry run)."""

    def __init__(self, dry_run, simulate_run=None):
        self.dry = dry_run
        self.fake = None
        if simulate_run:
            from fake_obie_run import FakeAcquire
            self.fake = FakeAcquire(simulate_run)
        if dry_run:
            self.arm = self.tool = None
        else:
            from urcommon import Arm, Tool
            self.tool = Tool()
            self.arm = Arm("follow_obie")

    def move(self, number, target, rot):
        if self.dry:
            print(f"  [dry run] travel to point {number} at {fmt_mm(target)}")
            return
        self.arm.travel(target, rot=rot)

    def hit(self, number, hit_index, target):
        if self.dry:
            print(f"  [dry run] hit {hit_index}")
            if self.fake:
                self.fake.hit()
            return True
        from run_test import hit
        return hit(self.tool, number, hit_index, target)

    def lift(self):
        if not self.dry:
            self.arm.move_by(dz=LIFT)

    def close(self):
        if self.arm:
            self.arm.close()


def ask(prompt):
    """Prompt; returns the lower-cased answer. q quits."""
    ans = input(prompt).strip().lower()
    if ans == "q":
        raise KeyboardInterrupt
    return ans


def follow(run, rig, state, args):
    watcher = RunWatcher(run, taps=args.taps)
    points = point_lookup(state, run, watcher.total, quiet=True)
    rot = state.get("rotation")
    template_mtime = os.path.getmtime(Path(run) / "template.json")
    print(f"Following {run}: {watcher.total} positions, {watcher.taps} hits each.")

    at_node = None          # node the hammer is over
    baseline = 0            # n_hits when we last fired (or arrived)
    awaiting = False        # fired, waiting for Acquire to record it
    fired_at = fire_at = 0.0
    last_run_check = time.monotonic()
    while True:
        now = time.monotonic()
        mtime = os.path.getmtime(Path(run) / "template.json")
        if mtime != template_mtime:     # e.g. stencil edited during the run
            template_mtime = mtime
            point_lookup(state, run, watcher.total, quiet=True)

        if args.latest_dir and now - last_run_check > 3:
            last_run_check = now
            newest = latest_run(args.latest_dir)
            if newest and newest != run and is_run(newest):
                print(f"A newer run appeared: {newest} (Start Over in Acquire?)")
                rig.lift()
                ask("Enter to switch to it, q to quit: ")
                return newest

        st = watcher.poll()
        if st.node is None:
            rig.lift()
            print("Run finished.")
            return None

        if st.node != at_node:
            if st.node not in points:
                raise SystemExit(f"Acquire is on node {st.node}, which has no planned point.")
            number, target = points[st.node]
            print(f"node {st.node} (point {number}, {len(st.done)}/{st.total} done): "
                  f"{st.n_hits}/{watcher.taps} hits so far")
            rig.move(number, target, rot)
            at_node, baseline, awaiting = st.node, st.n_hits, False
            fire_at = time.monotonic() + args.gap    # let the arm settle

        elif awaiting and st.n_hits != baseline:
            if st.n_hits < baseline:
                print(f"  hit count dropped to {st.n_hits} (Repeat in Acquire?); hitting again")
            awaiting, baseline = False, st.n_hits
            fire_at = now + args.gap

        elif awaiting and now - fired_at > args.timeout:
            print(f"  Acquire hasn't recorded a hit in {args.timeout:g} s "
                  f"(still {st.n_hits}/{watcher.taps}). Is it recording?")
            ans = ask("  Enter = hit again, w = keep waiting, q = quit: ")
            if ans == "w":
                fired_at = time.monotonic()
            else:
                awaiting, fire_at = False, 0.0

        if not awaiting and time.monotonic() >= fire_at:
            number, target = points[at_node]
            if args.confirm:
                ask(f"  hit {st.n_hits + 1}/{watcher.taps} at point {number} - Enter=hit, q=quit: ")
            ok = rig.hit(number, st.n_hits + 1, target)
            if not ok:
                print("  (hammer did not confirm; waiting for Acquire anyway)")
            awaiting, fired_at = True, time.monotonic()

        time.sleep(POLL)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="run folder, or with --latest a folder of runs")
    parser.add_argument("--latest", action="store_true",
                        help="use the newest run in RUN, and offer to switch if a newer one appears")
    parser.add_argument("--confirm", action="store_true", help="press Enter before every hit")
    parser.add_argument("--gap", type=float, default=GAP,
                        help=f"seconds between a recorded hit and the next one (default {GAP:g})")
    parser.add_argument("--timeout", type=float, default=TIMEOUT,
                        help=f"seconds to wait for Acquire to record a hit (default {TIMEOUT:g})")
    parser.add_argument("--taps", type=int, help="hits per position, if template.json lacks it")
    parser.add_argument("--dry-run", action="store_true", help="don't use the arm or the hammer")
    parser.add_argument("--simulate", action="store_true",
                        help="with --dry-run: record each hit into the run like Acquire would")
    args = parser.parse_args()
    if args.simulate and not args.dry_run:
        raise SystemExit("--simulate only works with --dry-run")

    args.latest_dir = Path(args.run).expanduser() if args.latest else None
    run = latest_run(args.run) if args.latest else Path(args.run).expanduser()
    if run is None or not is_run(run):
        raise SystemExit(f"No run folder (with TRF/ and template.json) at {args.run}")

    state = load_state()
    if state.get("calibration_mode", "probed") != "probed":
        print(f"NOTE: heights were not probed ({state['calibration_mode']}).")
    point_lookup(state, run, RunWatcher(run, taps=args.taps).total)   # check before moving
    if not args.dry_run:
        input("Slider low, area clear, hand near e-stop, control panel connected, "
              "Acquire recording. Enter to start, Ctrl-C to cancel: ")

    rig = Rig(args.dry_run, run if args.simulate else None)
    try:
        while run:
            run = follow(run, rig, state, args)
            if run and rig.fake:
                rig.fake = type(rig.fake)(run)
    except KeyboardInterrupt:
        print("\nStopped. Run follow_obie.py again to pick up where Acquire is.")
    finally:
        rig.close()


if __name__ == "__main__":
    main()
