# my_bot

A small ROS 2 autonomous mobile robot project following the
[Articulated Robotics - Build a Mobile Robot with ROS](https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros/) tutorial series.

<!--## Current State

This package currently contains a basic differential-drive robot description:

- `base_link` with a rectangular chassis
- left and right continuous wheel joints
- a fixed caster wheel
- simple visual, collision, and inertial properties using Xacro macros
- a `robot_state_publisher` launch file
- starter RViz, config, and empty Gazebo world files
-->

## Package Layout

- `description/` - URDF/Xacro robot model files
- `launch/` - ROS 2 launch files
- `rviz/` - RViz configuration
- `worlds/` - Gazebo world files
- `config/` - placeholder configuration files

## Build

From the workspace root:

```bash
colcon build --packages-select my_bot
source install/setup.bash
```

## Run

Start the robot state publisher:

```bash
ros2 launch my_bot rsp.launch.py
```

To use simulated time:

```bash
ros2 launch my_bot rsp.launch.py use_sim_time:=true
```

To visualize wheels, you need to simulate with the joint state publisher GUI
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
