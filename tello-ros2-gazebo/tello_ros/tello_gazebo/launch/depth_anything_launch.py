"""DepthAnything only: /<ns>/image_raw + /<ns>/camera_info -> /<ns>/depth/*"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        default_value='false',
        description='Publish static TF camera_link_1 -> camera_optical_link_1',
    )

    depth_node = Node(
        package='tello_gazebo',
        executable='depth_anything_node.py',
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
    tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'camera_link_1', 'camera_optical_link_1'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=None,  # set below (LaunchCondition無しでシンプルにしたいので別案を提示)
    )

    # LaunchCondition を使わず最小にしたい場合：
    # publish_optical_tf を true/falseで切り替えたいなら `IfCondition` が必要。
    # いったん最小構成として「TF出すならこのノード行をコメント解除」で運用が楽です。
    #
    # ↓最小運用：TFが必要なときだけこのノードを追加して起動してください。
    #
    # return LaunchDescription([declare_namespace, declare_use_sim_time, depth_node, tf_optical])

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        depth_node,
        # tf_optical,  # TFが無いならコメント外す
    ])
