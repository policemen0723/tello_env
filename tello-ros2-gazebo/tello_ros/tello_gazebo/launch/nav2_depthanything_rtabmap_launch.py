import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('tello_gazebo')
    nav2_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Top-level namespace for Nav2 and RTAB-Map'
    )
    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Use namespace for Nav2 nodes'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_rtabmap.yaml'),
        description='Nav2 parameters file'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart Nav2 lifecycle nodes'
    )

    # Starts Gazebo + Depth Anything + RTAB-Map (drone1 namespace is hardcoded inside rtabmap_launch.py).
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rtabmap_launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_namespace,
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        rtabmap_launch,
        nav2_launch,
    ])
