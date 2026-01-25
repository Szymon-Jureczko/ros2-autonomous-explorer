"""
auto_explore.launch.py — Gazebo + bridge + static TFs + SLAM Toolbox.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_autonomous_explorer = get_package_share_directory('autonomous_explorer')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    world_file = os.path.join(pkg_autonomous_explorer, 'worlds', 'robot_world.sdf')
    slam_params_file = os.path.join(pkg_autonomous_explorer, 'config', 'slam_params.yaml')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
                ],
                output='screen'
            )
        ]
    )

    tf_lidar = Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['0.21', '0', '0.3', '0', '0', '0', 'chassis', 'lidar'],
                    parameters=[{'use_sim_time': True}])
    tf_base = Node(package='tf2_ros', executable='static_transform_publisher',
                   arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
                   parameters=[{'use_sim_time': True}])
    tf_footprint = Node(package='tf2_ros', executable='static_transform_publisher',
                        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
                        parameters=[{'use_sim_time': True}])

    # SLAM Toolbox (online async) — wait for /scan + /tf
    slam = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': slam_params_file,
                }.items()
            )
        ]
    )

    return LaunchDescription([gz_sim, bridge, tf_lidar, tf_base, tf_footprint, slam])
