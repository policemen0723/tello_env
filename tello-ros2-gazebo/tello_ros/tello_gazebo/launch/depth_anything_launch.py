"""DepthAnything only: /<ns>/image_raw + /<ns>/camera_info -> /<ns>/depth/*"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_optical_tf = LaunchConfiguration('publish_optical_tf')

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
    declare_publish_optical_tf = DeclareLaunchArgument(
        'publish_optical_tf',
        default_value='true',
        description='Publish static TF camera_link_1 -> camera_optical_link_1',
    )

    depth_node = Node(
        package='tello_gazebo',
        executable='depth_anything_v3_node.py',
        name='depth_anything_node',
        output='screen',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,

            # Inputs (inside namespace)
            'rgb_topic': 'image_raw',
            'camera_info_topic': 'camera_info',

            # Outputs (inside namespace)
            'depth_topic': 'depth/image_raw',
            'depth_rgb_topic': 'depth/rgb',
            'depth_camera_info_topic': 'depth/camera_info',

            # TF optical frame used by DepthAnything / downstream
            'optical_frame_id': 'camera_optical_link_1',
        }],
    )

    # Optional static TF (enable only if your TF tree doesn't already have it)
    # camera_link (X前方) → camera_optical_link (Z前方, X右, Y下)
    # 正しい変換: roll=-π/2, yaw=-π/2
    tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link_1', 'camera_optical_link_1'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(publish_optical_tf),
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_publish_optical_tf,
        depth_node,
        tf_optical,
    ])
