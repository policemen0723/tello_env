"""Run Nav2 only (expects /<namespace>/map and /<namespace>/odom)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_dir = get_package_share_directory('tello_gazebo')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # 設定値の取得
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # 引数の定義
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Top-level namespace for Nav2',
    )
    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Use namespace for Nav2 nodes',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_rtabmap.yaml'),
        description='Nav2 parameters file',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart Nav2 lifecycle nodes',
    )

    # ---------------------------------------------------------
    # 修正ポイント: GroupActionを使って強制的にNamespaceに入れる
    # ---------------------------------------------------------
    nav2_group = GroupAction([
        PushRosNamespace(namespace), # これで中のノードは全て /drone1/... になる

        # 1. Depth -> Scan 変換ノード (これもグループ内に入れる)
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'scan_height': 10,
                'scan_time': 0.033,
                'range_min': 0.3,
                'range_max': 5.0,
                'output_frame': 'base_link_1'
            }],
            remappings=[
                ('depth', 'depth/image_raw'),
                ('depth_camera_info', 'depth/camera_info'),
                ('scan', 'scan')
            ]
        ),

        # 2. Nav2の起動 (引数からnamespaceは外し、外側のPushRosNamespaceに任せる)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': 'False',
                # ここでの 'namespace' 指定は削除し、親のGroupActionに任せることで重複を防ぐ
                'use_namespace': 'False', 
            }.items(),
        ),
    ])

    return LaunchDescription([
        declare_namespace,
        declare_use_namespace,
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        nav2_group, # グループ化したものを起動
    ])