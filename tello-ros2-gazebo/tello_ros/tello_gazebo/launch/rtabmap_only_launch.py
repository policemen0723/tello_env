"""Run RTAB-Map only (expects RGB/Depth/CameraInfo already available)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

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

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link_1',
            'map_frame_id': 'map',
            'publish_tf_map': 'true',
            'visual_odometry': 'true',
            'icp_odometry': 'false',
            'rgbd_sync': 'true',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'approx_sync': 'false',
            'approx_sync_max_interval': '0.02',
            'topic_queue_size': '10',
            'sync_queue_size': '20',
            'qos': '2',
            'rgb_topic': 'depth/rgb',
            'depth_topic': 'depth/image_raw',
            'camera_info_topic': 'depth/camera_info',
            'rviz': rviz,
            'rtabmap_viz': 'false',
            # 修正後の rtabmap_args
            'rtabmap_args': (
                '--delete_db_on_start '
                '--Grid/FromDepth true '
                '--Grid/RangeMax 4.5 '             # 【要望1】4m以上先は地図に書かない
                '--Grid/MinGroundHeight -0.11 '    # 【要望2】機体から11cm下より低いものは地面（無視）
                '--Grid/MaxObstacleHeight 0.5 '    # 【要望3】機体から50cm上より高いものは無視
                '--Grid/RayTracing true '
                '--Grid/NoiseFilteringRadius 0.2 '
                '--Grid/NoiseFilteringMinNeighbors 5'
            ),
            # 高速化のために特徴点数を制限
            'odom_args': '--Odom/MinInliers 5 --Vis/MinInliers 5 --Reg/Force3DoF true --Odom/Strategy 1 --GFTT/MinDistance 10 --Vis/MaxFeatures 500',
        }.items(),
    )

    tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'camera_link_1', 'camera_optical_link_1'],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_rviz,
        rtabmap_launch,
        tf_optical,
    ])
