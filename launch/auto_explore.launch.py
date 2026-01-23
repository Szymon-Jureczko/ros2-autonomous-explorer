"""
auto_explore.launch.py — Gazebo + bridge + static TF publishers.
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

    world_file = os.path.join(pkg_autonomous_explorer, 'worlds', 'robot_world.sdf')

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

    # Static TF: chassis -> lidar (sensor mounted on chassis)
    tf_lidar = Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['0.21', '0', '0.3', '0', '0', '0', 'chassis', 'lidar'],
                    parameters=[{'use_sim_time': True}])
    # Static TF: base_link -> chassis
    tf_base = Node(package='tf2_ros', executable='static_transform_publisher',
                   arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
                   parameters=[{'use_sim_time': True}])
    # Static TF: base_link -> base_footprint
    tf_footprint = Node(package='tf2_ros', executable='static_transform_publisher',
                        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
                        parameters=[{'use_sim_time': True}])

    return LaunchDescription([gz_sim, bridge, tf_lidar, tf_base, tf_footprint])
