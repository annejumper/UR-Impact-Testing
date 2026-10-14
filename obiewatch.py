#!/usr/bin/env python3
"""Follow an ObieApp Acquire run by reading its files (port of the Stencil Builder's _pollTRF).

    python3 obiewatch.py "~/obie_runs/Violin7/Test 3"    # print progress once a second
    python3 obiewatch.py --latest ~/obie_runs/Violin7     # newest run in that folder

Does not move the arm. The run folder is the copy that sync_to_arm.sh keeps up to
date from the Mac. Nothing here ever writes to it.

Run folder layout (written by ObieApp):
    <run>/template.json      taps, positions, prefix, and the stencil (node layout)
    <run>/TRF/<Test> H01.trf one file per position; rewritten after every hit, and its
                             text block at the end holds the running hit count (n_hits)
"""
import argparse
import hashlib
import json
import os
import re
import time
from collections import namedtuple
from pathlib import Path

MARKER = b"\x00OBIE_META\n"
NAME_RE = re.compile(r"\s([A-Z]{1,3})(\d+)\.trf$", re.I)   # "Test 3 H07.trf" -> ("H", "07")

# node: current node id (0-based), or None when the run is finished
# position: current position (1-based, node + 1); n_hits: hits recorded there so far
# done: set of finished node ids; total: number of positions in the run
Status = namedtuple("Status", "node position n_hits done total")


def read_meta(path):
    """The key: value block after MARKER, or {} if the file is missing or mid-write."""
    try:
        data = Path(path).read_bytes()
    except OSError:
        return {}
    i = data.find(MARKER)
    if i < 0:
        return {}
    meta = {}
    for line in data[i + len(MARKER):].decode("utf-8", "ignore").splitlines():
        key, sep, value = line.partition(":")
        if sep:
            meta[key.strip()] = value.strip()
    return meta


def read_template(run):
    with open(Path(run) / "template.json") as f:
        return json.load(f)


def stencil_nodes(template):
    """The stencil's nodes sorted by id, or [] if no stencil is attached to the run."""
    nodes = (template.get("stencil") or {}).get("nodes") or []
    return sorted(nodes, key=lambda n: int(n["id"]))


def stencil_fingerprint(nodes):
    """Short hash of node ids and positions. Labels and names don't affect it."""
    key = [[int(n["id"]), round(float(n["xMm"]), 2), round(float(n["yMm"]), 2)] for n in nodes]
    return hashlib.sha1(json.dumps(key).encode()).hexdigest()[:12]


def is_run(path):
    return (Path(path) / "TRF").is_dir() and (Path(path) / "template.json").is_file()


def latest_run(folder):
    """The run folder under `folder` (an instrument folder or the whole Data Folder)
    whose TRF folder or template changed most recently."""
    folder = Path(folder).expanduser()
    if is_run(folder):
        return folder
    candidates = [p for p in folder.glob("*") if is_run(p)] + \
                 [p for p in folder.glob("*/*") if is_run(p)]
    if not candidates:
        return None

    def changed(run):
        files = [run / "TRF", run / "template.json"] + list((run / "TRF").glob("*.trf"))
        return max(os.path.getmtime(f) for f in files)
    return max(candidates, key=changed)


class RunWatcher:
    """Works out which node Acquire is on from the run's TRF files.

    taps: hits per position; overrides template.json (needed for old runs without it).
    """

    def __init__(self, run_dir, taps=None):
        self.run = Path(run_dir).expanduser()
        tpl = read_template(self.run)
        if taps is None and str(tpl.get("taps", "")).isdigit():
            taps = int(tpl["taps"])
        if not taps:
            raise ValueError(f"{self.run}/template.json has no 'taps'; pass the hits per position")
        self.taps = taps
        self.per_group = max(1, int(tpl.get("positions", 1)))
        self.prefixes = [p.strip()[:3].upper() for p in str(tpl.get("prefix", "H")).split(",")
                         if p.strip()] or ["H"]
        self.total = self.per_group * len(self.prefixes)
        self.nodes = stencil_nodes(tpl)
        self.confirmed = set()   # positions already finished; never re-read

    def position(self, filename):
        """Global 1-based position for a TRF filename, or None if it isn't one of ours."""
        m = NAME_RE.search(filename)
        if not m or m.group(1).upper() not in self.prefixes:
            return None
        nn = int(m.group(2))
        if not 1 <= nn <= self.per_group:
            return None
        return self.prefixes.index(m.group(1).upper()) * self.per_group + nn

    def files(self):
        by_pos = {}
        for f in (self.run / "TRF").glob("*.trf"):
            p = self.position(f.name)
            if p is not None:
                by_pos[p] = f
        return by_pos

    def poll(self):
        """Current Status. Safe to call while Acquire (or rsync) is rewriting files."""
        by_pos = self.files()
        hits = {}
        for p, f in by_pos.items():
            if p in self.confirmed:
                continue
            n = read_meta(f).get("n_hits", "")
            if n.isdigit():
                hits[p] = int(n)
                if hits[p] >= self.taps:
                    self.confirmed.add(p)
        done = {p - 1 for p in by_pos if p in self.confirmed}
        pending = sorted(p for p in by_pos if p not in self.confirmed)
        if pending:
            cur = pending[0]
        elif by_pos:
            # Every recorded position is done; Acquire records in order, so the next
            # one is after the highest. (Same as ObieApp's count + 1 when there are no gaps.)
            cur = max(by_pos) + 1
        else:
            cur = 1
        if cur > self.total:
            return Status(None, None, None, done, self.total)
        return Status(cur - 1, cur, hits.get(cur, 0), done, self.total)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="run folder, or with --latest a folder of runs")
    parser.add_argument("--latest", action="store_true", help="follow the newest run in RUN")
    parser.add_argument("--taps", type=int, help="hits per position, if template.json lacks it")
    args = parser.parse_args()

    run = latest_run(args.run) if args.latest else Path(args.run).expanduser()
    if run is None or not is_run(run):
        raise SystemExit(f"No run folder (with TRF/ and template.json) at {args.run}")
    w = RunWatcher(run, taps=args.taps)
    nodes = {int(n["id"]): n for n in w.nodes}
    print(f"run {run}: {w.total} positions {','.join(w.prefixes)}, {w.taps} taps each, "
          + (f"stencil with {len(nodes)} nodes" if nodes else "no stencil"))
    last = None
    try:
        while True:
            st = w.poll()
            if st.node is None:
                print("run finished")
                break
            if (st.node, st.n_hits) != last:
                last = (st.node, st.n_hits)
                n = nodes.get(st.node)
                where = f" label {n['label']} at ({n['xMm']}, {n['yMm']}) mm" if n else ""
                print(f"node {st.node} (position {st.position}){where}: "
                      f"{st.n_hits}/{w.taps} hits, {len(st.done)}/{st.total} done")
            time.sleep(1)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
