"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ns = 'drone1'
    tello_gazebo_path = get_package_share_directory('tello_gazebo')
    world_path = os.path.join(tello_gazebo_path, 'worlds', 'takahashi.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    gazebo_models = os.path.join(tello_gazebo_path, 'models')
    gazebo_model_path = os.pathsep.join([
        gazebo_models,
        os.environ.get('GAZEBO_MODEL_PATH', ''),
        os.path.expanduser('~/.gazebo/models'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Gazebo world file to load',
        ),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            LaunchConfiguration('world'),
        ], output='screen'),

        # Spawn tello.urdf
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', '1', '0']),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen',
             namespace=ns),

        # Joystick controller, generates /namespace/cmd_vel messages
        Node(package='tello_driver', executable='tello_joy_main', output='screen',
             namespace=ns),
    ])
