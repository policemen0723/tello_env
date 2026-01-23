"""Run Nav2 explicitly (Bypassing nav2_bringup defaults)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_dir = get_package_share_directory('tello_gazebo')

    # 引数の取得
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # 引数定義
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='drone1', description='Top-level namespace')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_rtabmap.yaml'),
        description='Full path to the ROS2 parameters file to use')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')

    # リマッピング設定 (cmd_velの流れを整理)
    # Nav2(controller) -> cmd_vel_nav -> Smoother -> cmd_vel -> Drone
    cmd_vel_remappings = [
        ('cmd_vel', 'cmd_vel_nav'),
        ('cmd_vel_smoothed', 'cmd_vel')
    ]

    # ノードの定義
    nav2_nodes = GroupAction([
        PushRosNamespace(namespace),

        # depthimage_to_laserscan は削除（temporal/points を使用するため不要）

        # 1. Controller Server (制御)
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        # 2. Smoother Server (速度滑らか化)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=cmd_vel_remappings
        ),

        # 4. Planner Server (経路計画)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # 5. Behavior Server (リカバリー動作)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),

        # 6. BT Navigator (行動決定)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # 7. Waypoint Follower (経由地)
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),

        # 7. Lifecycle Manager (起動管理)
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
                                        'velocity_smoother'
                                        ]}]
        ),
    ])

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        nav2_nodes
    ])