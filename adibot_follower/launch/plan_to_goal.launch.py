import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    follower_pkg = get_package_share_directory('adibot_follower')
    adibot_pkg = get_package_share_directory('adibot')
    params_file = os.path.join(follower_pkg, 'config', 'follower_params.yaml')

    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    headless = LaunchConfiguration('headless')
    args = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(follower_pkg, 'worlds', 'cluttered.sdf'),
            description='World to load'),
        DeclareLaunchArgument('x', default_value='-8.0',
                              description='Robot spawn x (map frame)'),
        DeclareLaunchArgument('y', default_value='-8.0',
                              description='Robot spawn y (map frame)'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Robot spawn yaw (map frame)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gz sim server-only, no GUI'),
    ]

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless))

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 -s ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless))

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            adibot_pkg, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'adibot',
                   '-x', x, '-y', y, '-z', '0.1', '-Y', yaw],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f"config_file:={os.path.join(adibot_pkg, 'config', 'gz_bridge.yaml')}"],
        parameters=[{'use_sim_time': True}])

    # adibot's DiffDrive plugin publishes dead-reckoning odometry that starts at
    # (0,0,0) wherever the robot spawns -- unlike the target's OdometryPublisher
    # plugin, which reports world-frame ground truth directly (see
    # spawn_target.launch.py). So map -> odom has to be anchored at the robot's
    # spawn pose, not identity, for map -> base_link to reflect the true pose.
    # Verified empirically: with an identity anchor, map -> base_link stayed at
    # (0,0,0) even after spawning at (-8,-8).
    odom_anchor = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', x, '--y', y, '--yaw', yaw,
                   '--frame-id', 'map', '--child-frame-id', 'odom'])

    map_loader = Node(
        package='adibot_follower', executable='map_loader',
        parameters=[params_file,
                    {'map_yaml': os.path.join(follower_pkg, 'config', 'cluttered.map.yaml'),
                     'use_sim_time': True}],
        output='screen')

    global_planner = Node(
        package='adibot_follower', executable='global_planner',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen')

    return LaunchDescription(
        args + [gazebo_gui, gazebo_headless, rsp, spawn, bridge, odom_anchor, map_loader,
                global_planner])
