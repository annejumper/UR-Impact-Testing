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
| Save corners | `c1` / `c3` (and `c2` for a non-square plate) inside `jog.py`, or `python3 save_corner.py 1` / `2` / `3` | Corner Placement |
| Plan the grid | `python3 plan_grid.py 4` (4×4), or `python3 plan_grid.py 15 2` (15 points × 2 rows) | Points + Start |
| Probe heights | `python3 calibrate.py` | `callibrate_heights` |
| Hit | `python3 run_test.py --hits 3` | `run_test` + Obie |

Corner 1: probe **extended**, about 1 mm above the **first** impact point.
Corner 3: probe over the **last** impact point (diagonal).
Corner 2 (only for non-square plates, e.g. a strip): probe over the **last point of the first row**.
With corner 2 saved, save corners 2 and 3 the same way as corner 1 (probe extended, ~1 mm above the surface): their heights are used.

With only corners 1 and 3, the plate is assumed square and flat at corner 1's height (corner 3's height is ignored). With corner 2 saved, the plate can be any rectangle, tilted or cantilevered: `plan_grid.py NX NY` puts NX points from corner 1 to corner 2 and NY rows toward corner 3, with heights on the plane through the three corners. `calibrate.py` starts each probe search 6 mm above that height, so a sagging plate that bulges a little above the plane is still found. `plan_grid.py` warns if the angle at corner 2 isn't close to 90° (e.g. corner 2 left over from a different plate); `--square` ignores corner 2.

Useful extras:
- `python3 status.py` shows the current position and which steps are done. `--reset` clears `state.json`.
- `python3 goto.py 5` moves the probe 10 mm above planned point 5. `python3 goto.py 5 --hammer` uses the calibrated hammer position.
- `python3 tool.py listen` prints all channel traffic, for debugging the panel link.
- `calibrate.py --no-probe --height 10` skips probing (for when the probe isn't working). It puts the hammer 10 mm above each point's planned height. It trusts the corner heights; `status.py` and `run_test.py` show when heights weren't probed.
- `calibrate.py --probe corners` probes only the 4 corners and `--probe edges` only the outer ring; the other points' heights are interpolated (fine for a flat plate, even tilted or twisted). Use the default `--probe all` when the plate bends between points, like a long cantilevered beam.
- `calibrate.py --start N` and `run_test.py --start N` / `--points 3 7` resume or redo points.
- At the `run_test.py` prompt: Enter = hit, `r` = redo the last hit, `n` = next point, `b` = back to the previous point, `q` = quit. A redo is logged again under the same point and hit number; keep the later row.
- `run_test.py --auto 4` fires every 4 s with no prompts.
- Every hit is logged to `hits_log.csv` with a timestamp, point number and position.
- `python3 hammer_offset.py probe`, jog the hammer over the same mark, then `python3 hammer_offset.py hammer` measures the probe-to-hammer offset (saved in `hammer_offset.json`, kept across `--reset`) and shifts the calibrated points to match, without re-probing.

**Ctrl-C during a move cancels the trajectory, so the arm stops.** The e-stop is still the real safety.

## Differences from the notebook version

- **Calibrated heights are actually used.** The old `flat_plate_impact_test` computed `new_traj` but ran `run_test(traj)`, so the probe heights and the hammer offset of −15 mm / +12 mm were never applied.
- **Moves between points lift 10 mm first** (`LIFT`), so the tool doesn't drag across the plate.
- **Moves are straight lines**, split into 20 mm segments (`MAX_SEGMENT`), so a long move doesn't swing the tool through an arc or trip the joint-step check.
- **The probe search has limits.** It stops after 15 mm (coarse) or 4 mm (fine) without contact, where the old loop lowered forever.
- **Orientation:** the old code forced every point to Euler `[0, 0, 0]`. These scripts keep the orientation the tool had when corner 1 was saved.
- **Move times scale with distance** (20 mm/s, min 0.5 s) instead of 5 s for every move, including 0.2 mm probe steps.
- **Both panel tool modes work.** In serial mode the panel replies on `/modal/tool/request/response`, which the old websocket bridge dropped because it only passed `/URarm` topics.
- **No `Websocket_Startup` notebook is needed.** Scripts that use the tool open their own channel connection.

The x/y/z directions are the same as the old panel's, because both use the same ikfast solver frame. They may not match the pendant's base-frame display.

## Settings

All tunable values are at the top of `urcommon.py`: speed, jog limit, `LIFT`, `HAMMER_OFFSET`
(measured with the tool heading `HAMMER_OFFSET_YAW`; calibrate turns it to match corner 1's orientation),
`HIT_Z_OFFSET`, gripper open/close %, and the channel URL. The probe step sizes and limits are at the top of `calibrate.py`.

## Moving to Obie later

`run_test.py` keeps the test logic in `move_to_point()` and `hit()`. Only `main()` decides when to call them
(terminal prompts right now). An Obie version would be a small listener built on `urcommon.Channel` that calls the same
two functions when Obie asks for a point or a hit. Corners, grid and calibration would stay as they are.
