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
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')
    headless = LaunchConfiguration('headless')
    args = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(follower_pkg, 'worlds', 'cluttered.sdf'),
            description='World to load'),
        DeclareLaunchArgument('robot_x', default_value='-8.0',
                              description='Robot spawn x (map frame)'),
        DeclareLaunchArgument('robot_y', default_value='-8.0',
                              description='Robot spawn y (map frame)'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0',
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
                   '-x', robot_x, '-y', robot_y, '-z', '0.1', '-Y', robot_yaw],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f"config_file:={os.path.join(adibot_pkg, 'config', 'gz_bridge.yaml')}"],
        parameters=[{'use_sim_time': True}])

    # See plan_to_goal.launch.py: adibot's DiffDrive odom is dead-reckoning
    # from spawn, so map -> odom must be anchored at the spawn pose.
    odom_anchor = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', robot_x, '--y', robot_y, '--yaw', robot_yaw,
                   '--frame-id', 'map', '--child-frame-id', 'odom'])

    map_loader = Node(
        package='adibot_follower', executable='map_loader',
        parameters=[params_file,
                    {'map_yaml': os.path.join(follower_pkg, 'config', 'cluttered.map.yaml'),
                     'use_sim_time': True}],
        output='screen')

    # Target model + bridge + waypoint driver, reused as-is (Phase 2).
    target = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            follower_pkg, 'launch', 'spawn_target.launch.py')]))

    target_predictor = Node(
        package='adibot_follower', executable='target_predictor',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('odom', '/target/odom'), ('predicted_pose', '/target/predicted_pose')],
        output='screen')

    global_planner = Node(
        package='adibot_follower', executable='global_planner',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('goal_pose', '/target/predicted_pose')],
        output='screen')

    return LaunchDescription(
        args + [gazebo_gui, gazebo_headless, rsp, spawn, bridge, odom_anchor, map_loader,
                target, target_predictor, global_planner])
