"""Run RTAB-Map only (expects RGB/Depth/CameraInfo already available)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    publish_optical_tf = LaunchConfiguration('publish_optical_tf')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Namespace for RTAB-Map topics',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz with RTAB-Map config',
    )
    declare_publish_optical_tf = DeclareLaunchArgument(
        'publish_optical_tf',
        default_value='false',
        description='Publish static TF camera_link_1 -> camera_optical_link_1 (keep false if DepthAnything publishes it)',
    )
    declare_publish_odom_tf = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false',
        description='Let RTAB-Map publish odom->base_link TF (disable when EKF publishes it)',
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link_1',
            'odom_frame_id': 'odom',  # EKFと同じodomフレームを使用
            'map_frame_id': 'map',
            'publish_tf_map': 'true',  # map -> odom は RTAB-Map が担当
            'publish_tf': 'false',     # 【重要】odom -> base_link は EKF に任せるため false
            'publish_tf_odom': 'false', # 【追加】rgbd_odometryのTF発行も無効化
            'visual_odometry': 'true',
            'rgbd_sync': 'true',
            'approx_sync': 'true',     # DepthAnythingの遅延対策で true に
            'approx_sync_max_interval': '0.1', # 0.02は厳しすぎるので 0.1(100ms) 程度に緩和
            'odom_topic': 'rgbd_odom', # トピック名を明示して EKF が拾いやすくする
            
            # 入力トピックのremapping（実際のトピック名に合わせる）
            # 【改善】時間平滑化された深度画像を使用（depth_temporal_filterの出力）
            'rgb_topic': 'depth/filtered_rgb',
            'depth_topic': 'depth/filtered',
            'camera_info_topic': 'depth/filtered_camera_info',
            
            # 深度の乖離に強い設定
            'rtabmap_args': (
                '--delete_db_on_start '
                '--Grid/FromDepth true '
                '--Grid/RangeMax 4.0 '             # 【要望1】4m以上先は地図に書かない
                '--Grid/MinGroundHeight -0.11 '    # 【要望2】機体から11cm下より低いものは地面（無視）
                '--Grid/MaxObstacleHeight 0.5 '    # 【要望3】機体から50cm上より高いものは無視
                '--RGBD/OptimizeMaxError 0 ' # ループクローズ時の不自然なジャンプを抑制
            ),
            'odom_args': (
                '--Odom/Strategy 0 '       # Frame-to-Map (履歴を使い安定させる)
                '--Vis/EstimationType 1 '  # 3D-to-2D (PnP法: 深度の乖離に強い)
                '--Odom/MinInliers 5 '
                '--Reg/Force3DoF true '    # Z移動なしのため true
                '--Odom/PublishTF false'   # 念のためここでもTF発行をオフ
            ),
        }.items(),
    )

    tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'camera_link_1', 'camera_optical_link_1'],
        condition=IfCondition(publish_optical_tf),
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_rviz,
        declare_publish_optical_tf,
        declare_publish_odom_tf,
        rtabmap_launch,
        tf_optical,
    ])
