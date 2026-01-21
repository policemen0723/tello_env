"""EKF fusion: /<ns>/imu + /<ns>/flight_data -> /<ns>/odometry/filtered"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

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
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('tello_gazebo'),
            'config',
            'ekf_flightdata.yaml',
        ),
        description='EKF parameter file',
    )

    flight_twist = Node(
        package='tello_gazebo',
        executable='flight_data_to_twist.py',
        name='flight_data_to_twist',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'flight_data_topic': 'flight_data',
            'twist_topic': 'flight_twist_cov',
            'frame_id': 'base_link_1',
            'velocity_scale': 0.01,
            'invert_y': True,
            'linear_covariance': 0.05,
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('flight_twist', 'flight_twist_cov'),
        ],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        flight_twist,
        ekf_node,
    ])
