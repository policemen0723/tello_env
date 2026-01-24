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
    cmd_vel_twist = Node(
        package='tello_gazebo',
        executable='cmd_vel_to_twist_cov.py',
        name='cmd_vel_to_twist_cov',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': 'cmd_vel',
            'twist_topic': 'twist',
            'frame_id': 'base_link_1',
            'scale_xy': 2.0,
            'scale_z': 4.0,
            'scale_yaw': 3.141592653589793,
            'zero_y': True,
            'linear_covariance': 0.6,
            'angular_covariance': 1.5,
        }],
    )

    imu_sanitizer = Node(
        package='tello_gazebo',
        executable='imu_sanitizer.py',
        name='imu_sanitizer',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'imu_in_topic': 'imu',
            'imu_out_topic': 'imu_sanitized',
            'frame_id': 'base_link_1',
            # Down-weight IMU heavily to reduce its influence in EKF.
            'orientation_covariance': [6.0, 6.0, 24.0],
            'angular_velocity_covariance': [1.5, 1.5, 3.0],
            'linear_acceleration_covariance': [12.0, 12.0, 24.0],
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        cmd_vel_twist,
        imu_sanitizer,
        ekf_node,
    ])
