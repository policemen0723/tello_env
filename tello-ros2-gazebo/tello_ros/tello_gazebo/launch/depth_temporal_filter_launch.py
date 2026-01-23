"""
Depth Temporal Filter Launch

深度画像の時間平滑化フィルターを起動する。

データフロー:
  Depth Anything → /depth/image_raw
                          ↓
               depth_temporal_filter
                          ↓
                   /depth/filtered → RTAB-Map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Namespace for the node',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    depth_temporal_filter_node = Node(
        package='tello_gazebo',
        executable='depth_temporal_filter.py',
        name='depth_temporal_filter',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            
            # 入力トピック (Depth Anythingの出力)
            'input_depth_topic': 'depth/image_raw',
            'input_rgb_topic': 'depth/rgb',
            'input_camera_info_topic': 'depth/camera_info',
            
            # 出力トピック (RTAB-Mapへの入力)
            'output_depth_topic': 'depth/filtered',
            'output_rgb_topic': 'depth/filtered_rgb',
            'output_camera_info_topic': 'depth/filtered_camera_info',
            
            # 平滑化パラメータ
            'ema_alpha': 0.7,           # 新しい値の重み (高いほど追従速い)
            'change_threshold': 0.3,    # これ以上の変化は新しい値を採用 (m)
            'min_depth': 0.3,
            'max_depth': 5.0,
            'confidence_decay': 0.95,   # 信頼度の減衰率
            'min_confidence': 0.3,      # 最小信頼度（これ以下はNaN）
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        depth_temporal_filter_node,
    ])
