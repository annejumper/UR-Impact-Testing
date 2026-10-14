# Impact testing over ssh

Terminal version of the `Impact_Testing.ipynb` / `impact_commands.py` flow from
`UR_summer2025`. Each step is a small script, and the scripts share progress through `state.json`.

The arm is driven directly the way `wrist_nudge.py` does it. The hammer and probe
are still run by the **Robot Control Panel** web page, and the scripts reach it
over the same `hackathon` channel the notebook used.

This folder does not depend on `UR_summer2025`. `myur/` holds copies of the two pieces it needs:
`myur/ik_solver` (the compiled UR3e IK solver, built for Python 3.10, with its sources and `setup.py` for rebuilding)
and `myur/robotiq_gripper.py` (used by `Gripper_Node.py`).

## One-time setup

```bash
pip install --user websockets      # already done on this machine
```

## Each session

1. UR driver running, and **External Control** program playing on the pendant (same as for wrist_nudge).
2. Gripper node in its own ssh window (only needed for `gripper.py` / `open` / `close`):
   ```bash
   source /opt/ros/humble/setup.bash
   cd ~/anne/impact_testing && python3 Gripper_Node.py --ip <ROBOT_IP> -sc   # drop -sc to recalibrate
   ```
3. Robot Control Panel open in a browser: **connect** the channel, connect the tool (serial or BLE), and **Start Code**.
4. In your ssh window:
   ```bash
   source /opt/ros/humble/setup.bash
   cd ~/anne/impact_testing
   ```

## Running a test

| Step | Command | Replaces |
|---|---|---|
| Check the tool link | `python3 tool.py probe` / `python3 tool.py hit` | panel buttons |
| Grip the tool | `python3 gripper.py open`, then `close` | Gripper Control |
| Jog to the first point | `python3 jog.py` (interactive) or `python3 jog.py -z 2` | move buttons |
| Save corners | `c1` / `c3` inside `jog.py`, or `python3 save_corner.py 1` / `3` | Corner Placement |
| Plan the grid | `python3 plan_grid.py 4` (4×4) | Points + Start |
| Probe heights | `python3 calibrate.py` | `callibrate_heights` |
| Hit | `python3 run_test.py --hits 3` | `run_test` + Obie |

Corner 1: probe **extended**, about 1 mm above the **first** impact point.
Corner 3: probe over the **last** impact point (diagonal). Its height is ignored.

Useful extras:
- `python3 status.py` shows the current position and which steps are done. `--reset` clears `state.json`.
- `python3 goto.py 5` moves the probe 10 mm above planned point 5. `python3 goto.py 5 --hammer` uses the calibrated hammer position.
- `python3 tool.py listen` prints all channel traffic, for debugging the panel link.
- `calibrate.py --no-probe --height 10` skips probing (for when the probe isn't working). It puts the hammer 10 mm above corner 1's height at every point. It assumes the plate is flat; `status.py` and `run_test.py` show when heights weren't probed.
- `calibrate.py --start N` and `run_test.py --start N` / `--points 3 7` resume or redo points.
- `run_test.py --auto 4` fires every 4 s with no prompts.
- Every hit is logged to `hits_log.csv` with a timestamp, point number and position.

**Ctrl-C during a move cancels the trajectory, so the arm stops.** The e-stop is still the real safety.

## Differences from the notebook version

- **Calibrated heights are actually used.** The old `flat_plate_impact_test` computed `new_traj` but ran `run_test(traj)`, so the probe heights and the hammer offset of −15 mm / +12 mm were never applied.
- **Moves between points lift 10 mm first** (`LIFT`), so the tool doesn't drag across the plate.
- **The probe search has limits.** It stops after 15 mm (coarse) or 4 mm (fine) without contact, where the old loop lowered forever.
- **Orientation:** the old code forced every point to Euler `[0, 0, 0]`. These scripts keep the orientation the tool had when corner 1 was saved.
- **Move times scale with distance** (20 mm/s, min 0.5 s) instead of 5 s for every move, including 0.2 mm probe steps.
- **Both panel tool modes work.** In serial mode the panel replies on `/modal/tool/request/response`, which the old websocket bridge dropped because it only passed `/URarm` topics.
- **No `Websocket_Startup` notebook is needed.** Scripts that use the tool open their own channel connection.

The x/y/z directions are the same as the old panel's, because both use the same ikfast solver frame. They may not match the pendant's base-frame display.

## Settings

All tunable values are at the top of `urcommon.py`: speed, jog limit, `LIFT`, `HAMMER_OFFSET`,
`HIT_Z_OFFSET`, gripper open/close %, and the channel URL. The probe step sizes and limits are at the top of `calibrate.py`.

## Running from ObieApp (stencils)

The arm can follow an ObieApp Acquire run on your Mac: it goes to whichever node Acquire is on,
hits, waits for Acquire to record the hit, and moves on when Acquire does. ObieApp is not changed.
The arm reads the same run files ObieApp's Stencil Builder reads (see
`obieapp-node-tracking-handoff.md`). Any stencil layout works, not just rectangles.

| Step | Where | Command |
|---|---|---|
| Copy runs to the arm | **Mac** | `./sync_to_arm.sh ~/path/to/DataFolder croger@<arm-host>` (leave running) |
| Set up the run, stencil applied | Mac | Acquire |
| Align the stencil | arm | `python3 plan_stencil.py --latest ~/obie_runs/<Instrument>` |
| Probe heights | arm | `python3 calibrate.py` (unchanged) |
| Start recording | Mac | Acquire |
| Hit | arm | `python3 follow_obie.py --latest ~/obie_runs/<Instrument>` |

- **Getting the sync script onto the Mac:** `scp croger@<arm-host>:anne/impact_testing/sync_to_arm.sh ~/`.
  It copies `template.json` and `TRF/` for each run once a second over ssh, skipping the WAVs.
- **Aligning (`plan_stencil.py`)** replaces corners + `plan_grid.py`. Run it with `--show` first to list
  the nodes and suggested alignment nodes. Then, in the interactive session, jog the extended probe about
  1 mm above a node (as for corner 1) and type `a <label>`. Do this for 3 or more nodes spread out in a
  triangle (4 is better: the fit then shows how well they agree), then `save`. `fit` shows the
  fit so far, and `g <label>` moves 10 mm above any node to check it. Saving is refused if an alignment
  node is more than 1 mm off, or if any node is out of reach. Alignment nodes are kept in `state.json`
  as you go, so `--fit-only` can refit later without the arm.
- **`follow_obie.py`** checks that the run's stencil is the one you aligned to (it refuses if nodes
  were moved since), then follows Acquire. If Acquire doesn't record a hit within 10 s it stops and asks,
  rather than hitting again. Repeat in Acquire makes it hit that position again. With `--latest` it
  offers to switch when a new run appears (Start Over). Every hit is logged to `hits_log.csv`.
  `--confirm` asks before every hit; use it for the first live runs. `--gap` sets the pause after a
  recorded hit (default 1 s).
- **Rectangles still work the old way:** with points from `plan_grid.py` instead of `plan_stencil.py`,
  node N is grid point N+1. Corner 1 must be over the stencil's node 1, and no `--zigzag`.
- **Testing without the arm:** `obiewatch.py RUN` prints Acquire's progress, and
  `follow_obie.py RUN --dry-run` prints moves and hits without touching the arm or hammer.
  `fake_obie_run.py` makes fake runs (grid, custom shapes, H,V prefixes) and can play Acquire, so
  `follow_obie.py FAKE_RUN --dry-run --simulate` runs a whole test on paper.

Under the hood, `run_test.py`'s `move_to_point()` and `hit()` do the work. `follow_obie.py` only decides
when to call them, from the run's files instead of from terminal prompts.
