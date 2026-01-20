"""UniDepth only: /<ns>/image_raw + /<ns>/camera_info -> /<ns>/depth/*"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_type = LaunchConfiguration('model_type')

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
    declare_model_type = DeclareLaunchArgument(
        'model_type',
        default_value='vits14',
        description='UniDepth V2 model type: vitl14, vitb14, vits14',
    )

    depth_node = Node(
        package='tello_gazebo',
        executable='unidepth_node.py',
        name='unidepth_node',
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

            # TF optical frame used by UniDepth / downstream
            'optical_frame_id': 'camera_optical_link_1',
            'model_type': model_type,
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_model_type,
        depth_node,
    ])
