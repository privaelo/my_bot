# adibot workspace

ROS 2 packages for [adibot](adibot/), a small mobile robot that follows a moving
object through a cluttered world: constant-velocity target prediction, A* global
planning on an occupancy grid, pure-pursuit tracking, and fixed-rate replanning.

### Lidar visualization
<img width="800" height="450" alt="Lidar_Gif" src="https://github.com/user-attachments/assets/62b1fc09-2b01-4429-9da6-3b672b1ee57f" />

## Packages

- [`adibot/`](adibot/) - robot description, launch files, and base Gazebo simulation. See its [README](adibot/README.md).
- [`adibot_follower/`](adibot_follower/) - the moving-target follow pipeline (prediction, A* planner, pure-pursuit) and the cluttered-world demo. See its [README](adibot_follower/README.md) for run instructions, parameters, and [known issues](adibot_follower/docs/known_issues.md).

## Setup

This repo is the `src/` folder of a ROS 2 workspace. Clone it there:

```bash
mkdir -p ~/dev_ws
git clone https://github.com/privaelo/adibot.git ~/dev_ws/src
```

Then build from the workspace root (`~/dev_ws`, not this folder):

```bash
cd ~/dev_ws
colcon build
source install/setup.bash
```

## Run the follow demo

```bash
ros2 launch adibot_follower follow.launch.py
```

Add `paused:=true` to start Gazebo paused (press play to begin — handy for
recording), or `headless:=true` for a GUI-less run. See the
[adibot_follower README](adibot_follower/README.md) for all launch args and the
`GTK_PATH` note if `gz sim`/`rviz2` crash in a VS Code terminal.
