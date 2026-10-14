#!/usr/bin/env python3
"""Make fake ObieApp runs for testing obiewatch.py / follow_obie.py without ObieApp.

    python3 fake_obie_run.py make /tmp/obie --rows 3 --cols 4 --taps 3
    python3 fake_obie_run.py make /tmp/obie --prefix H,V --rows 2 --cols 3   # H and V groups
    python3 fake_obie_run.py make /tmp/obie --custom 7                       # 7 nodes on a circle
    python3 fake_obie_run.py hit "/tmp/obie/Fake/Test 1"          # record one hit, like Acquire
    python3 fake_obie_run.py auto "/tmp/obie/Fake/Test 1" --every 2   # a hit every 2 s
    python3 fake_obie_run.py repeat "/tmp/obie/Fake/Test 1" 2     # Repeat: clear position 2
    python3 fake_obie_run.py partial "/tmp/obie/Fake/Test 1" 3    # half-written file for position 3

Files are written the way Acquire writes them: one "<Test> H01.trf" per position,
rewritten after each hit, with a text block holding n_hits after the binary data.
"""
import argparse
import json
import math
import time
from pathlib import Path

from obiewatch import MARKER, RunWatcher

BINARY = bytes(range(256)) * 8   # stands in for the FRF data


def trf_name(test, prefixes, per_group, position):
    group, nn = divmod(position - 1, per_group)
    return f"{test} {prefixes[group]}{nn + 1:02d}.trf"


def write_trf(path, n_hits, partial=False):
    meta = (f"sample_rate: 48000\nbit_depth: 16\nn_hits: {n_hits}\nthreshold: 0.05\n"
            "ham_cutoff: 0.050 s\nmic_cutoff: 0.300 s\ndevice: fake\n").encode()
    Path(path).write_bytes(BINARY[:len(BINARY) // 2] if partial else BINARY + MARKER + meta)


def make_run(data_dir, instrument, test, rows, cols, taps, prefix, x_spacing, y_spacing, custom):
    run = Path(data_dir).expanduser() / instrument / test
    (run / "TRF").mkdir(parents=True, exist_ok=True)
    prefixes = [p.strip().upper() for p in prefix.split(",")]
    if custom:
        # irregular layout: nodes on a circle of radius 60 mm, plus one in the middle
        nodes = [{"id": i, "label": str(i + 1),
                  "xMm": round(60 * math.cos(2 * math.pi * i / (custom - 1)), 2),
                  "yMm": round(60 * math.sin(2 * math.pi * i / (custom - 1)), 2)}
                 for i in range(custom - 1)]
        nodes.append({"id": custom - 1, "label": str(custom), "xMm": 0.0, "yMm": 0.0})
        grid = None
    else:
        # row-major from the top-left, origin at the grid centre (as the Stencil Builder does)
        nodes = [{"id": r * cols + c, "label": str(r * cols + c + 1), "row": r, "col": c,
                  "xMm": (c - (cols - 1) / 2) * x_spacing, "yMm": (r - (rows - 1) / 2) * y_spacing}
                 for r in range(rows) for c in range(cols)]
        grid = {"rows": rows, "cols": cols, "xSpacing": x_spacing, "ySpacing": y_spacing}
    if len(nodes) % len(prefixes):
        raise SystemExit("node count must divide evenly between the prefixes")
    template = {
        "taps": taps, "positions": len(nodes) // len(prefixes), "prefix": prefix,
        "sample_rate": 48000,
        "stencil": {"name": "fake", "type": "node-stencil", "grid": grid, "nodes": nodes},
    }
    (run / "template.json").write_text(json.dumps(template, indent=2))
    return run


class FakeAcquire:
    """Records hits into a run the way Acquire does."""

    def __init__(self, run):
        self.run = Path(run).expanduser()
        self.test = self.run.name
        self.watcher = RunWatcher(self.run)

    def path(self, position):
        w = self.watcher
        return self.run / "TRF" / trf_name(self.test, w.prefixes, w.per_group, position)

    def hit(self):
        """Add one hit at the current position. Returns (position, n_hits) or None if finished."""
        st = RunWatcher(self.run).poll()   # fresh watcher: no cache, so Repeat is seen
        if st.node is None:
            return None
        write_trf(self.path(st.position), st.n_hits + 1)
        return st.position, st.n_hits + 1

    def repeat(self, position):
        write_trf(self.path(position), 0)

    def partial(self, position):
        write_trf(self.path(position), 0, partial=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("make")
    m.add_argument("data_dir")
    m.add_argument("--instrument", default="Fake")
    m.add_argument("--test", default="Test 1")
    m.add_argument("--rows", type=int, default=3)
    m.add_argument("--cols", type=int, default=4)
    m.add_argument("--taps", type=int, default=3)
    m.add_argument("--prefix", default="H")
    m.add_argument("--xspacing", type=float, default=25.0)
    m.add_argument("--yspacing", type=float, default=20.0)
    m.add_argument("--custom", type=int, metavar="N", help="N nodes in a circle instead of a grid")
    for name in ("hit", "auto", "repeat", "partial"):
        s = sub.add_parser(name)
        s.add_argument("run")
        if name == "auto":
            s.add_argument("--every", type=float, default=2.0)
        if name in ("repeat", "partial"):
            s.add_argument("position", type=int)
    args = parser.parse_args()

    if args.cmd == "make":
        run = make_run(args.data_dir, args.instrument, args.test, args.rows, args.cols, args.taps,
                       args.prefix, args.xspacing, args.yspacing, args.custom)
        print("made", run)
        return
    acq = FakeAcquire(args.run)
    if args.cmd == "hit":
        print(acq.hit() or "run finished")
    elif args.cmd == "repeat":
        acq.repeat(args.position)
    elif args.cmd == "partial":
        acq.partial(args.position)
    elif args.cmd == "auto":
        try:
            while (r := acq.hit()):
                print("position %d: %d hits" % r)
                time.sleep(args.every)
            print("run finished")
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
