import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('adibot_follower')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    bridge_clock = LaunchConfiguration('bridge_clock')
    args = [
        DeclareLaunchArgument('x', default_value='6.0',
                              description='Target spawn x (map frame)'),
        DeclareLaunchArgument('y', default_value='-6.0',
                              description='Target spawn y (map frame)'),
        DeclareLaunchArgument('yaw', default_value='1.5708',
                              description='Target spawn yaw (map frame)'),
        DeclareLaunchArgument(
            'bridge_clock', default_value='true',
            description='Bridge /clock -- disable when another bridge in the same '
                        'launch (e.g. adibot gz_bridge.yaml) already provides it, '
                        'since two independent clock bridges cause tf to intermittently '
                        'jump back in time'),
    ]

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-file', os.path.join(pkg, 'models', 'target', 'model.sdf'),
                   '-name', 'target',
                   '-x', x, '-y', y, '-z', '0.25', '-Y', yaw],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f"config_file:={os.path.join(pkg, 'config', 'target_bridge.yaml')}"],
        parameters=[{'use_sim_time': True}])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        condition=IfCondition(bridge_clock),
        output='screen')

    # Gazebo's OdometryPublisher reports the target's pose in world
    # coordinates (verified empirically: first message equals the spawn pose),
    # and the world origin coincides with the map origin, so target/odom is
    # anchored to map with an identity transform. Sim-only assumption; on
    # hardware a localizer would own this transform.
    odom_anchor = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'target/odom'])

    driver = Node(
        package='adibot_follower', executable='target_driver',
        parameters=[os.path.join(pkg, 'config', 'follower_params.yaml'),
                    {'use_sim_time': True}],
        remappings=[('cmd_vel', '/target/cmd_vel'), ('odom', '/target/odom')],
        output='screen')

    return LaunchDescription(args + [spawn, bridge, clock_bridge, odom_anchor, driver])
