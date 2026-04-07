"""
auto_explore.launch.py — Full autonomous exploration stack.

Launches Gazebo, ros_gz_bridge, SLAM Toolbox, Nav2 (minimal),
frontier explorer, and optionally RViz2. Timed delays ensure
each component's dependencies are ready before it starts.

Usage:
  ros2 launch autonomous_explorer auto_explore.launch.py
  ros2 launch autonomous_explorer auto_explore.launch.py headless:=true rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    TimerAction, ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # --- Package directories ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_autonomous_explorer = get_package_share_directory('autonomous_explorer')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # --- File paths ---
    world_file = os.path.join(pkg_autonomous_explorer, 'worlds', 'robot_world.sdf')
    nav2_params_file = os.path.join(pkg_autonomous_explorer, 'config', 'nav2_explore_params.yaml')
    slam_params_file = os.path.join(pkg_autonomous_explorer, 'config', 'slam_params.yaml')
    # RViz config with Map, costmaps, LaserScan, paths
    rviz_config_file = os.path.join(pkg_autonomous_explorer, 'config', 'explore_rviz.rviz')

    # --- Launch arguments ---
    headless_arg = DeclareLaunchArgument('headless', default_value='false',
                                         description='Run Gazebo without GUI')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='Launch RViz2')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')

    # =====================================================================
    #  1. GAZEBO SIMULATION
    # =====================================================================
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s {world_file}'}.items(),
        condition=IfCondition(headless)
    )
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=UnlessCondition(headless)
    )

    # =====================================================================
    #  2. GAZEBO ↔ ROS BRIDGE (delay 5s for GZ transport readiness)
    # =====================================================================
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

    # =====================================================================
    #  3. STATIC TF PUBLISHERS
    # =====================================================================
    tf_lidar = Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['0.21', '0', '0.3', '0', '0', '0', 'chassis', 'lidar'],
                    parameters=[{'use_sim_time': True}])
    tf_base = Node(package='tf2_ros', executable='static_transform_publisher',
                   arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
                   parameters=[{'use_sim_time': True}])
    tf_footprint = Node(package='tf2_ros', executable='static_transform_publisher',
                        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
                        parameters=[{'use_sim_time': True}])

    # =====================================================================
    #  4. SLAM TOOLBOX (delay 10s for /scan and /tf)
    # =====================================================================
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

    # =====================================================================
    #  5. NAV2 — minimal set (delay 20s for SLAM map→odom TF)
    #     Only 7 nodes: no route_server, waypoint_follower, docking_server.
    # =====================================================================
    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nav2 = TimerAction(
        period=20.0,
        actions=[
            GroupAction(actions=[
                SetParameter('use_sim_time', True),
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    output='screen',
                    parameters=[nav2_params_file],
                    remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
                ),
                Node(
                    package='nav2_smoother',
                    executable='smoother_server',
                    name='smoother_server',
                    output='screen',
                    parameters=[nav2_params_file],
                    remappings=nav2_remappings,
                ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params_file],
                    remappings=nav2_remappings,
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params_file],
                    remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params_file],
                    remappings=nav2_remappings,
                ),
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    parameters=[nav2_params_file],
                    remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
                ),
                Node(
                    package='nav2_collision_monitor',
                    executable='collision_monitor',
                    name='collision_monitor',
                    output='screen',
                    parameters=[nav2_params_file],
                    remappings=nav2_remappings,
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'autostart': True,
                        'node_names': [
                            'controller_server',
                            'smoother_server',
                            'planner_server',
                            'behavior_server',
                            'velocity_smoother',
                            'collision_monitor',
                            'bt_navigator',
                        ],
                    }],
                ),
            ])
        ]
    )

    # =====================================================================
    #  6. FRONTIER EXPLORER (delay 40s for Nav2 lifecycle activation)
    # =====================================================================
    frontier_explorer = TimerAction(
        period=40.0,
        actions=[
            Node(
                package='autonomous_explorer',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'min_frontier_size': 3,
                    'replan_interval': 4.0,
                    'blacklist_radius': 0.8,
                }]
            )
        ]
    )

    # =====================================================================
    #  7. RVIZ2 (delay 25s, Mesa software rendering env vars)
    # =====================================================================
    rviz = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config_file,
                     '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen',
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    'MESA_GL_VERSION_OVERRIDE': '3.3',
                    'QT_QUICK_BACKEND': 'software',
                    'OGRE_RTT_MODE': 'Copy',
                },
                condition=IfCondition(use_rviz),
            )
        ]
    )

    # =====================================================================
    #  ASSEMBLE LAUNCH
    # =====================================================================
    return LaunchDescription([
        # OpenGL env vars set ONLY on RViz2 — global override breaks Gazebo ogre2
        headless_arg,
        rviz_arg,
        gz_sim_headless,
        gz_sim_gui,
        bridge,
        tf_lidar,
        tf_base,
        tf_footprint,
        slam,
        nav2,
        frontier_explorer,
        rviz,
    ])
