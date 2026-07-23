# adibot_follower — known issues & workarounds

Issues found while bringing up the follow pipeline, with the mechanism, the
fix/workaround applied, and any residual limitation. Most were diagnosed by
instrumented headless runs (logging the robot's real `map -> base_link`
trajectory and its clearance to every obstacle) rather than by eye, because
several presented as one symptom with a different root cause underneath.

---

## Follower collides with obstacles / wedges

**Symptom.** In Gazebo the robot drives into an obstacle and stops, even though
the A* path shown in RViz cleanly avoids it.

**Root cause (two layers).** Not a map or planner bug: an interior-cell check
found 0/25 obstacles miscovered, and the planned path never routed through an
occupied cell. The A* path is collision-free. The robot collides because it
**deviates off that safe path**, and there is **no reactive/local collision
layer** to catch it, so on contact it just keeps pushing and wedges. Two things
drove the deviation:

1. `global_planner` plans for a **point robot** and leans entirely on
   `inflation_radius` for the body. At `inflation_radius = 0.4 m` the path was
   allowed within 0.4 m of an obstacle surface — only ~0.10 m of body margin
   for the ~0.25–0.30 m footprint.
2. The bigger one: the **diff-drive wheel dynamics** did not execute
   `pure_pursuit`'s `(v, omega)` faithfully. Spherical-wheel contact, slip, and
   a minimum turning radius (`max_linear/max_angular = 0.53 m` at full speed)
   made the real motion understeer and bow off the path — enough to cross the
   thin margin.

**Why it was so hard to pin down.** The failure is **real-time-timing
sensitive**. Un-recorded GUI runs at real-time factor ~1.0 collided reliably,
but headless runs and `ros2 bag record -a` (which dropped RTF to ~0.65 under
the I/O load) **never reproduced it** — two full bags captured only clean laps.
So the collision could not be captured or a fix verified from a bag; the
real-time GUI run was the only reliable signal.

**Margin measures (necessary but not sufficient).** These raised the clearance
floor and were kept, but did **not** eliminate the collision on their own —
bumping `inflation_radius` 0.4 -> 0.5 -> 0.6 only lowered its probability:

- `obstacles.yaml` `min_clearance` 1.0 -> **1.4 m** (regenerate with
  `python3 scripts/gen_world.py`; all 25 obstacles still fit at seed 42). Needed
  because inflation grows a gap `g` down to a `g - 2*inflation_radius` corridor,
  so at `min_clearance = 1.0` an `inflation_radius = 0.6` sealed passages and
  stranded the robot.
- `global_planner` `inflation_radius` -> **0.6 m** (0.2 m free corridor).
- `global_planner` **snaps an occupied start cell** to the nearest free cell
  (mirrors the goal snap), so the robot can plan an escape if it does end up in
  the inflated margin instead of deadlocking.
- `pure_pursuit` **slows into sharp turns** (`v <= max_angular/|curvature|`) and
  uses a smaller `lookahead_distance` (0.3 m) to track tight paths and cut
  corners less.

**Resolution.** The deviation was fundamentally the **wheel dynamics**, so the
follower was switched to **kinematic locomotion**: `adibot/description/`
`gazebo_control.xacro` now uses `VelocityControl` + `OdometryPublisher` (a
"sliding box") instead of `DiffDrive`, mirroring the already-kinematic target.
The robot now executes `cmd_vel` almost exactly (measured mean tracking error
0.06 m/s / 0.07 rad/s vs. the diff-drive's slip), so it tracks the
collision-free A* path faithfully and stops clipping obstacles. **Confirmed at
real-time in the GUI.** The margin measures above remain as complementary
robustness. This is a **sim-only simplification** — on hardware a real base
controller replaces it; that is also where a reactive layer using `/scan` would
belong (future work), which is out of scope for the current
predict -> A* -> pursuit pipeline.

---

## Robot receives cmd_vel but doesn't move, or only drives dead-straight

**Symptom.** `/cmd_vel` shows nonzero linear+angular, but the robot sits still,
or moves forward but never turns (traces a straight line at its spawn heading).

**Root cause.** Wheel joint limits in `adibot/description/robot_core.xacro`
were too low for this task:

- `effort="0.05"` (N·m) — too little torque to push through any contact
  friction; `/joint_states` showed wheel velocity ~0 under full command.
- `velocity="10.0"` (rad/s) — `max_linear = 0.8 m/s` alone needs
  `0.8 / 0.05 (wheel radius) = 16 rad/s`, already over the cap, so both wheels
  saturated identically and no differential was left for `angular.z` → the
  robot could only go straight.

**Workaround (now superseded).** The wheel `effort`/`velocity` limits were
raised at the time, but this whole failure mode is moot since the follower moved
to kinematic `VelocityControl` (see "Follower collides with obstacles / wedges"
above) — the wheels are no longer actuated. The bumped limits were reverted to
avoid dead tuning. Kept here because it's the same wheel-dynamics root cause that
ultimately drove the kinematic switch.

---

## tf intermittently "jumps back in time" / pipeline destabilizes

**Symptom.** Floods of `Detected jump back in time. Clearing TF buffer` from
every node, followed by `no path found` and `map -> base_link` becoming
unavailable.

**Root cause.** Two independent `parameter_bridge` processes each bridged
`/clock` (adibot's `gz_bridge.yaml` and the target's `target_bridge.yaml`),
publishing conflicting sim-time streams.

**Workaround applied.** Only one clock bridge runs. `spawn_target.launch.py`
has a `bridge_clock` arg (default true for standalone use); `follow.launch.py`
includes it with `bridge_clock:=false` since adibot's bridge already provides
`/clock`.

---

## Gazebo GUI / RViz crash on launch (`libpthread` symbol lookup error)

**Symptom.** `gz sim` GUI or `rviz2` dies immediately with
`symbol lookup error: .../libpthread.so.0: undefined symbol: __libc_pthread_init`.

**Root cause.** `GTK_PATH` (and `GTK_MODULES`) are inherited from the VS Code
snap wrapper into the integrated terminal, pointing GTK apps at the snap's
bundled, incompatible libraries.

**Workaround.** `unset GTK_PATH GTK_MODULES` before launching any GUI app.
(Not a code change; add to `~/.bashrc` to make it permanent.) `follow.launch.py`
also has a `headless:=true` mode that skips all GUI for CLI/CI verification.

---

## RViz shows "No map received"

**Symptom.** The Map display never populates even though `/map` is publishing.

**Root cause.** `map_loader` publishes the grid once, latched with
`transient_local` durability; RViz's Map display defaults its subscription to
`volatile`, which only receives messages published *after* it subscribes, so it
misses the one-time latch.

**Workaround.** In the RViz Map display, set the topic's **Durability Policy**
to **Transient Local**. (The saved `rviz/follower.rviz` config bakes this in.)
