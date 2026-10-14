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

## Moving to Obie later

`run_test.py` keeps the test logic in `move_to_point()` and `hit()`. Only `main()` decides when to call them
(terminal prompts right now). An Obie version would be a small listener built on `urcommon.Channel` that calls the same
two functions when Obie asks for a point or a hit. Corners, grid and calibration would stay as they are.
