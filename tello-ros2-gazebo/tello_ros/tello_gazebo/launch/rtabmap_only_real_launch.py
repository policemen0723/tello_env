"""Run RTAB-Map only for real Tello (expects RGB/Depth/CameraInfo already available)."""

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
        default_value='',
        description='Namespace for RTAB-Map topics',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
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
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf_map': 'true',
            'visual_odometry': 'true',
            'icp_odometry': 'false',
            'rgbd_sync': 'true',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.1',
            'topic_queue_size': '10',
            'sync_queue_size': '20',
            'qos': '2',
            'rgb_topic': 'depth/rgb',
            'depth_topic': 'depth/image_raw',
            'camera_info_topic': 'depth/camera_info',
            'rviz': rviz,
            'rtabmap_viz': 'false',
            'rtabmap_args': '--delete_db_on_start --Grid/FromDepth true',
            'odom_args': '--Odom/MinInliers 5 --Vis/MinInliers 5 --Reg/Force3DoF true --Odom/Strategy 0 --Odom/ResetCountdown 1 --Vis/EstimationType 1',
        }.items(),
    )

    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.035', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
    )
    tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'camera_link', 'camera_optical_link'],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_rviz,
        rtabmap_launch,
        tf_camera,
        tf_optical,
    ])
