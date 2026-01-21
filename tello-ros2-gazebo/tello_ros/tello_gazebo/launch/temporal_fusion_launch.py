"""Temporal fusion: /<ns>/depth/image_raw -> /<ns>/temporal/points"""

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
        description='Namespace for topics',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    fusion_node = Node(
        package='tello_gazebo',
        executable='temporal_fusion_node.py',
        name='temporal_fusion_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'depth_topic': 'depth/image_raw',
            'camera_info_topic': 'depth/camera_info',
            'odom_topic': 'odometry/filtered',
            'output_topic': 'temporal/points',
            'output_frame': 'odom',
            'depth_frame': 'camera_link_1',
            'stride': 4,
            'min_depth': 0.3,
            'max_depth': 5.0,
            'voxel_size': 0.2,
            'ema_alpha': 0.6,
            'prune_after': 1.0,
            'publish_rate': 5.0,
            'min_weight': 0.2,
            'max_points': 20000,
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        fusion_node,
    ])
