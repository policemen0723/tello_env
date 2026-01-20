"""DepthAnything Real-world Offline Launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 【追加】最強のオフライン設定
    # これを入れると、HuggingFaceライブラリがネット接続を一切試みなくなります
    # タイムアウト待ちがなくなり、即座にローカルキャッシュを読みに行きます
    set_offline_mode = SetEnvironmentVariable('HF_HUB_OFFLINE', '1')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for topics',
    )
    # 実機なのでデフォルトは false
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    depth_node = Node(
        package='tello_gazebo',
        executable='depth_anything_node_real.py',
        name='depth_anything_node_real',
        output='screen',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'rgb_topic': 'image_raw',
            'camera_info_topic': 'camera_info',
            'depth_topic': 'depth/image_raw',
            'depth_rgb_topic': 'depth/rgb',
            'depth_camera_info_topic': 'depth/camera_info',
            'optical_frame_id': 'camera_optical_link',
        }],
    )

    # Static TFs for real Tello: base_link -> camera_link -> camera_optical_link
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
        set_offline_mode,  # ← これが重要
        declare_namespace,
        declare_use_sim_time,
        depth_node,
        tf_camera,
        tf_optical,
    ])
