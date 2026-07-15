# adibot

A small ROS 2 autonomous mobile robot project following the
[Articulated Robotics - Build a Mobile Robot with ROS](https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros/) tutorial series.

### Lidar visualization
#### Gazebo
<img width="800" height="450" alt="Lidar_Gif" src="https://github.com/user-attachments/assets/62b1fc09-2b01-4429-9da6-3b672b1ee57f" />

#### Rviz
<img width="1649" height="943" alt="Lidar-scan-Rviz" src="https://github.com/user-attachments/assets/6bdc69db-2914-4ebe-9eed-83c4a39e29aa" />

### Coming next:
- Adding a camera
- Object detection and tracking

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

From the workspace root (`~/dev_ws`, not this folder):

```bash
colcon build --packages-select adibot
source install/setup.bash
```

## Run

Start the simulation on Gazebo:

```bash
ros2 launch adibot launch_sim.launch.py world:=path/to/your/world.sdf
```

Start the robot state publisher:

```bash
ros2 launch adibot rsp.launch.py
```

To use simulated time:

```bash
ros2 launch adibot rsp.launch.py use_sim_time:=true
```

To visualize wheels, you need to simulate with the joint state publisher GUI
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### Good to know
Use commands below to check the data from topics
```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic info /cmd_vel
```

## Troubleshooting (Some issues I ran into)

**Lidar rays not visible / no data on `/scan`:** the tutorial's classic
`libgazebo_ros_ray_sensor.so` plugin doesn't exist in the new Gazebo (gz-sim), so
the sensor silently never activates. In gz-sim use `type="gpu_lidar"` with a
`<topic>` and no plugin (the world's `Sensors` system handles it), then bridge the
topic via `ros_gz_bridge`. See `description/lidar.xacro`.

**Still no data:** the world file must load the sensors system — check the launch
log for `Loaded system [gz::sim::systems::Sensors]`. Worlds saved from the Gazebo
GUI don't include it; add
`<plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>` next
to the other world plugins.

**Scan publishes but RViz shows 0 points:** gz-sim stamps the scan with a scoped
frame like `adibot/laser_frame/laser` that isn't in the ROS TF tree, so RViz drops
every point. Set `<gz_frame_id>laser_frame</gz_frame_id>` in the sensor. Verify
with `ros2 topic echo /scan --field header.frame_id --once`.
