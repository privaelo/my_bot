# adibot workspace

ROS 2 packages for [adibot](adibot/), a small autonomous mobile robot,
plus a trained RL policy for autonomous navigation.

### Lidar visualization
<img width="800" height="450" alt="Lidar_Gif" src="https://github.com/user-attachments/assets/62b1fc09-2b01-4429-9da6-3b672b1ee57f" />

## Packages

- [`adibot/`](adibot/) - robot description, launch files, and Gazebo simulation. See its [README](adibot/README.md) for build/run instructions and troubleshooting notes.
- [`adibot_rl/`](adibot_rl/) - RL navigation policy for adibot (training, evaluation, and a policy node).

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
