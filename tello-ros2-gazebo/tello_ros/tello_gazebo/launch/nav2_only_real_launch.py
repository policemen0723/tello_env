"""Run Nav2 explicitly for real Tello (no sim time, no namespace by default)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_dir = get_package_share_directory('tello_gazebo')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_rtabmap_real.yaml'),
        description='Full path to the ROS2 parameters file to use')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')

    # Nav2(controller) -> cmd_vel_nav -> Smoother -> cmd_vel -> Drone
    cmd_vel_remappings = [
        ('cmd_vel', 'cmd_vel_nav'),
        ('cmd_vel_smoothed', 'cmd_vel')
    ]

    nav2_nodes = GroupAction([
        PushRosNamespace(namespace),

        # 1. Depth -> Scan conversion
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'scan_height': 10,
                'scan_time': 0.033,
                'range_min': 0.3,
                'range_max': 5.0,
                'output_frame': 'base_link'
            }],
            remappings=[
                ('depth', 'depth/image_raw'),
                ('depth_camera_info', 'depth/camera_info'),
                ('scan', 'scan')
            ]
        ),

        # 2. Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        # 3. Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=cmd_vel_remappings
        ),

        # 4. Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # 5. Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),

        # 6. BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # 7. Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),

        # 8. Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator',
                                        'waypoint_follower',
                                        'velocity_smoother']}]
        ),
    ])

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        nav2_nodes
    ])
