#!/usr/bin/env python3
"""Plan impact points from an ObieApp stencil (any node layout) instead of corners + plan_grid.py.

    python3 plan_stencil.py RUN --show       # list the nodes and suggest alignment nodes (no arm)
    python3 plan_stencil.py RUN              # interactive: jog, save alignment nodes, fit, save
    python3 plan_stencil.py RUN --fit-only   # refit from alignment nodes already saved (no arm)

RUN is a run folder synced from the Mac (one with template.json and TRF/), or with
--latest a folder of runs. The stencil must be attached to the run (apply it in Acquire).

Alignment: jog the probe, extended, about 1 mm above a node (the same as corner 1),
then type "a <label>". Do this for at least 3 nodes spread out in a triangle;
4 or more lets the fit report how well they agree. "save" then fits the stencil to the
robot, checks every node is reachable and writes the points to state.json in node-id
order, where calibrate.py, goto.py, run_test.py and follow_obie.py pick them up.
Point N is node id N-1, i.e. ObieApp position N.

Interactive commands (jog commands are the same as jog.py):
  +x 5 / -y 2 / +z     jog (mm)          step 1     default jog step
  pos                  current position  a LABEL    save alignment node here
  rm LABEL             forget one        list       alignment nodes + suggestions
  fit                  show the fit      g LABEL    go 10 mm above a node (needs a fit)
  save                 fit and save      q          quit
"""
import argparse
import math

import numpy as np

from jog import jog, parse_jog
from obiewatch import is_run, latest_run, read_template, stencil_fingerprint, stencil_nodes
from urcommon import HAMMER_OFFSET, LIFT, MAX_JOINT_STEP, fmt_mm, load_state, save_state

MAX_ERROR_MM = 1.0       # refuse to save if an alignment node is further off than this
MAX_TILT_DEG = 10.0      # warn if the fitted stencil plane is tilted more than this


def node_xyz(node):
    """Stencil node position in metres, in the stencil plane (z = 0)."""
    return np.array([float(node["xMm"]) / 1000, float(node["yMm"]) / 1000, 0.0])


def suggest(nodes, k=4):
    """Pick k well spread nodes: two far apart, the one making the biggest triangle, then spread."""
    if len(nodes) <= k:
        return list(nodes)
    pts = {int(n["id"]): node_xyz(n)[:2] for n in nodes}
    centre = np.mean(list(pts.values()), axis=0)
    a = max(pts, key=lambda i: np.linalg.norm(pts[i] - centre))
    b = max(pts, key=lambda i: np.linalg.norm(pts[i] - pts[a]))

    def area(i):
        u, v = pts[b] - pts[a], pts[i] - pts[a]
        return abs(u[0] * v[1] - u[1] * v[0])
    chosen = [a, b, max(pts, key=area)]
    while len(chosen) < k:
        chosen.append(max((i for i in pts if i not in chosen),
                          key=lambda i: min(np.linalg.norm(pts[i] - pts[j]) for j in chosen)))
    by_id = {int(n["id"]): n for n in nodes}
    return [by_id[i] for i in chosen]


def fit(pairs):
    """Rigid fit (rotation R, translation t) taking stencil points onto robot points.

    pairs: list of (stencil_xyz, robot_xyz). Returns (R, t, info) where info has the
    per-pair errors in mm, the scale the measurements suggest, and the plane's tilt.
    """
    P = np.array([p for p, _ in pairs])
    Q = np.array([q for _, q in pairs])
    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    P0, Q0 = P - pc, Q - qc
    spread = np.linalg.svd(P0[:, :2], compute_uv=False)
    if len(pairs) < 3 or spread[1] < 0.15 * spread[0]:
        raise ValueError("need at least 3 alignment nodes that form a triangle, not a line")
    U, S, Vt = np.linalg.svd(P0.T @ Q0)
    d = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = qc - R @ pc
    errors = [float(np.linalg.norm(R @ p + t - q)) * 1000 for p, q in pairs]
    scale = float((S[0] + S[1] + d * S[2]) / np.sum(P0 ** 2))   # best similarity scale
    normal = R @ [0, 0, 1]
    info = {
        "errors_mm": errors,
        "scale": scale,
        "tilt_deg": math.degrees(math.acos(min(1.0, abs(normal[2])))),
        "mirrored": bool(normal[2] < 0),
        "turn_deg": math.degrees(math.atan2(R[1, 0], R[0, 0])),
    }
    return R, t, info


def describe_fit(info, labels):
    for label, e in zip(labels, info["errors_mm"]):
        print(f"  node {label:>4}: off by {e:.2f} mm")
    print(f"  stencil turned {info['turn_deg']:.1f} deg, plane tilt {info['tilt_deg']:.1f} deg"
          + (", y axis flipped (normal for screen-style stencil coordinates)" if info["mirrored"] else ""))
    if abs(info["scale"] - 1) > 0.01:
        print(f"  NOTE: distances on the table are {info['scale'] * 100:.1f}% of the stencil's mm. "
              "Check the stencil's spacing or the projector calibration.")
    if info["tilt_deg"] > MAX_TILT_DEG:
        print(f"  NOTE: plane tilted {info['tilt_deg']:.1f} deg. Check the alignment heights.")


def reach_check(points, rot, q_start):
    """IK for each point's travel height and hammer position, chaining joint guesses.
    Returns a list of problems (empty if every node looks reachable)."""
    from myur.ik_solver.ur_kinematics import URKinematics
    ik = URKinematics("ur3e")
    q_prev = list(q_start)
    problems = []
    for i, p in enumerate(points, 1):
        for name, xyz in (("travel", [p[0], p[1], p[2] + LIFT]),
                          ("hammer", [p[0] + HAMMER_OFFSET[0], p[1] + HAMMER_OFFSET[1], p[2]])):
            pose = np.hstack([np.asarray(rot), np.reshape(xyz, (3, 1))])
            q = ik.inverse(pose.reshape(-1).tolist(), q_guess=np.asarray(q_prev))
            if q is None:
                problems.append(f"point {i} ({name} {fmt_mm(xyz)}): no IK solution")
                continue
            step = max(abs(a - b) for a, b in zip(q, q_prev))
            if step > MAX_JOINT_STEP:
                problems.append(f"point {i} ({name}): a joint turns {step:.2f} rad from the "
                                "previous point; the move would be refused")
            q_prev = list(q)
    return problems


class Planner:
    def __init__(self, run, max_error=MAX_ERROR_MM):
        self.run = run
        self.max_error = max_error
        tpl = read_template(run)
        self.nodes = stencil_nodes(tpl)
        if not self.nodes:
            raise SystemExit(f"{run}/template.json has no stencil. Apply the stencil in Acquire "
                             "(or start Watch in the Stencil Builder) and let it sync.")
        self.name = (tpl.get("stencil") or {}).get("name", "")
        self.fingerprint = stencil_fingerprint(self.nodes)
        self.by_label = {str(n["label"]): n for n in self.nodes}
        if len(self.by_label) != len(self.nodes):           # labels not unique: use ids
            self.by_label = {str(int(n["id"]) + 1): n for n in self.nodes}
        state = load_state()
        align = state.get("stencil_align") or {}
        if align.get("fingerprint") != self.fingerprint:
            if align.get("points"):
                print("Saved alignment nodes are for a different stencil; starting fresh.")
            align = {"fingerprint": self.fingerprint, "points": {}}
        self.align = align

    def save_align(self):
        state = load_state()
        state["stencil_align"] = self.align
        save_state(state)

    def node(self, label):
        n = self.by_label.get(str(label))
        if n is None:
            raise SystemExit(f"no node labelled {label!r}")
        return n

    def show(self):
        print(f"stencil {self.name!r}: {len(self.nodes)} nodes, fingerprint {self.fingerprint}")
        for n in self.nodes:
            print(f"  id {int(n['id']):3d}  label {str(n['label']):>4}  "
                  f"({float(n['xMm']):7.1f}, {float(n['yMm']):7.1f}) mm")
        self.list_align()

    def list_align(self):
        saved = self.align["points"]
        if saved:
            print("alignment nodes saved:", ", ".join(saved))
        rest = [str(n["label"]) for n in suggest(self.nodes) if str(n["label"]) not in saved]
        if rest:
            print("suggested alignment nodes:", ", ".join(rest))

    def add(self, arm, label):
        n = self.node(label)
        joints = arm.joints()
        pose = arm.pose(joints)
        self.align["points"][str(label)] = {
            "id": int(n["id"]), "xyz": pose[:, 3].tolist(), "rot": pose[:, :3].tolist(),
            "joints": joints,
        }
        self.save_align()
        print(f"node {label} saved at {fmt_mm(pose[:, 3])} ({len(self.align['points'])} saved)")

    def fit(self, verbose=True):
        labels = list(self.align["points"])
        pairs = [(node_xyz(self.node(lb)), np.array(self.align["points"][lb]["xyz"]))
                 for lb in labels]
        R, t, info = fit(pairs)
        if verbose:
            describe_fit(info, labels)
        return R, t, info

    def points(self, R, t):
        return [(R @ node_xyz(n) + t).tolist() for n in self.nodes]

    def save(self):
        R, t, info = self.fit()
        worst = max(info["errors_mm"])
        if worst > self.max_error:
            raise SystemExit(f"Not saved: an alignment node is {worst:.2f} mm off "
                             f"(limit {self.max_error} mm). "
                             "Re-save that node, or remove it with rm.")
        first = next(iter(self.align["points"].values()))
        points = self.points(R, t)
        problems = reach_check(points, first["rot"], first["joints"])
        if problems:
            print("\n".join("  " + p for p in problems))
            raise SystemExit("Not saved: some nodes can't be reached from this setup.")

        state = load_state()
        state["rotation"] = first["rot"]
        state["grid"] = {
            "size": [len(points), 1], "zigzag": False, "points": points, "source": "stencil",
            "stencil": {"name": self.name, "fingerprint": self.fingerprint, "run": str(self.run),
                        "ids": [int(n["id"]) for n in self.nodes],
                        "labels": [str(n["label"]) for n in self.nodes]},
        }
        state["stencil_align"] = self.align
        state.pop("calibrated", None)
        state.pop("calibration_mode", None)
        save_state(state)
        for n, p in zip(self.nodes, points):
            print(f"  point {int(n['id']) + 1:2d} (node {n['label']}): {fmt_mm(p)}")
        print(f"{len(points)} points saved from stencil {self.fingerprint}. Next: python3 calibrate.py")

    def go(self, arm, label):
        R, t, _ = self.fit(verbose=False)
        target = (R @ node_xyz(self.node(label)) + t).tolist()
        target[2] += LIFT
        first = next(iter(self.align["points"].values()))
        if input(f"move 10 mm above node {label} at {fmt_mm(target)}? Enter=yes ").strip():
            return
        arm.travel(target, rot=first["rot"])
        print("now at", fmt_mm(arm.xyz()))


def interactive(planner):
    from urcommon import Arm
    planner.list_align()
    arm = Arm("plan_stencil")
    step = 1.0
    try:
        print("now at", fmt_mm(arm.xyz()))
        while True:
            try:
                words = input(f"stencil [{step:g} mm]> ").split()
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
                elif cmd == "a" and len(words) == 2:
                    planner.add(arm, words[1])
                elif cmd == "rm" and len(words) == 2:
                    if planner.align["points"].pop(words[1], None) is not None:
                        planner.save_align()
                        print(f"node {words[1]} removed")
                elif cmd == "list":
                    planner.list_align()
                elif cmd == "fit":
                    planner.fit()
                elif cmd == "g" and len(words) == 2:
                    planner.go(arm, words[1])
                elif cmd == "save":
                    planner.save()
                elif cmd in ("q", "quit", "exit"):
                    break
                else:
                    print(__doc__.split("Interactive commands")[1])
            except ValueError as e:
                print(e if str(e) else "bad number")
            except SystemExit as e:   # keep the session alive after a refused move or save
                print(e)
    except KeyboardInterrupt:
        print()
    finally:
        arm.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="run folder, or with --latest a folder of runs")
    parser.add_argument("--latest", action="store_true", help="use the newest run in RUN")
    parser.add_argument("--show", action="store_true", help="list nodes and suggestions only")
    parser.add_argument("--fit-only", action="store_true",
                        help="fit and save from saved alignment nodes, without the arm")
    parser.add_argument("--max-error", type=float, default=MAX_ERROR_MM,
                        help=f"largest alignment error allowed, mm (default {MAX_ERROR_MM})")
    args = parser.parse_args()

    run = latest_run(args.run) if args.latest else args.run
    if run is None or not is_run(run):
        raise SystemExit(f"No run folder (with TRF/ and template.json) at {args.run}")
    planner = Planner(run, args.max_error)
    print(f"run {run}")
    if args.show:
        planner.show()
    elif args.fit_only:
        try:
            planner.save()
        except ValueError as e:
            raise SystemExit(e)
    else:
        interactive(planner)


if __name__ == "__main__":
    main()
