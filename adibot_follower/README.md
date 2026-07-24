# adibot_follower

A differential-drive robot follows a moving target through a cluttered world.
Pipeline: **constant-velocity target prediction → A\* on a static occupancy grid
→ pure-pursuit tracking**, replanning at a fixed rate. Target pose comes from
Gazebo ground truth (bridged), so perception can be swapped in later behind the
same topic.

## Pipeline

```
/target/odom ──▶ target_predictor ──▶ /target/predicted_pose ─┐
                                                              ▼
   /map (map_loader, latched) ──▶ global_planner ──▶ /plan ──▶ pure_pursuit ──▶ /cmd_vel
                                        ▲
                                   tf: map → base_link
```

- **target_driver** — drives the target model around a waypoint circuit
  (`/target/cmd_vel`); ground-truth odometry on `/target/odom`.
- **target_predictor** — extrapolates the target `prediction_horizon` seconds
  ahead at constant velocity → `/target/predicted_pose`.
- **map_loader** — loads the static grid (`config/cluttered.map.yaml/.pgm`) and
  latches it on `/map` (`transient_local`).
- **global_planner** — A\* (8-connected, octile) on the inflated grid from the
  robot's tf pose to the predicted target, at `replan_rate` → `/plan`. Snaps an
  occupied start/goal cell to the nearest free cell.
- **pure_pursuit** — turns `/plan` into `/cmd_vel`, slowing into sharp turns and
  stopping within `standoff_distance` of the target so it trails rather than rams.

Pure logic (A\* and pure-pursuit geometry) lives in `include/adibot_follower/`
and is unit-tested (`test/`, gtest).

## Build

```bash
cd ~/dev_ws
colcon build --packages-select adibot adibot_follower
source install/setup.bash
```

## Run

```bash
ros2 launch adibot_follower follow.launch.py
```

Launch args: `headless:=true` (gz server only, no GUI/RViz), `rviz:=false`,
`paused:=true` (start Gazebo paused — press play to begin, handy for recording a
demo from t=0), `robot_x`/`robot_y`/`robot_yaw` (follower spawn), `world:=<path>`.

Record a demo from the first frame:

```bash
ros2 launch adibot_follower follow.launch.py paused:=true
# set up your recording / camera, then press the play button in Gazebo
```

> **GUI note (VS Code integrated terminal).** `gz sim` and `rviz2` may crash with
> a `libpthread` symbol-lookup error because `GTK_PATH` is inherited from the
> VS Code snap. Run `unset GTK_PATH GTK_MODULES` before launching any GUI app
> (or add it to `~/.bashrc`). Use `headless:=true` for CLI/CI runs.

## Parameters

All parameters live in `config/follower_params.yaml` (one block per node,
declared with defaults and logged at startup). Key ones: `prediction_horizon`,
`replan_rate`, `inflation_radius`, `lookahead_distance`, `standoff_distance`,
`max_linear`, `max_angular`, target `speed` + `waypoints`. Topic names are
remapped in the launch file, never hard-coded.

## Reproducibility

`scripts/gen_world.py` reads `config/obstacles.yaml` (explicit `seed`) and emits
both `worlds/cluttered.sdf` and the matching `config/cluttered.map.pgm/.yaml`
from the same placements, so the sim world and planner grid can never disagree.
Spec, seed, and generated files are all committed. Regenerate with:

```bash
python3 scripts/gen_world.py
```

## Sim-only assumptions

- **Ground-truth odometry** ⇒ `map ≡ odom` and `map ≡ target/odom` (identity
  static transforms in launch). On hardware a localizer would own `map → odom`.
- **Kinematic locomotion.** The follower executes `cmd_vel` via `VelocityControl`
  (a "sliding box"), not wheeled diff-drive dynamics — see
  [docs/known_issues.md](docs/known_issues.md) for why. A real base controller
  replaces this on hardware.

## Known issues 

See [docs/known_issues.md](docs/known_issues.md) for the debugging trail and the
reasoning behind the inflation / clearance / locomotion choices (obstacle
collisions, tf clock races, the GUI `GTK_PATH` crash, RViz "No map received").
