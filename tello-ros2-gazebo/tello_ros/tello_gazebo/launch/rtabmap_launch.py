
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ns = 'drone1'
    tello_gazebo_dir = get_package_share_directory('tello_gazebo')
    world_path = os.path.join(tello_gazebo_dir, 'worlds', 'takahashi.world')
    
    # 1. Include simple_launch.py to start Gazebo and Tello
    # We can invoke it directly or copy its content. Invoking is cleaner.
    simple_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tello_gazebo_dir, 'launch', 'simple_launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # 2. Start Depth Anything Node
    depth_node = Node(
        package='tello_gazebo',
        executable='depth_anything_node.py',
        name='depth_anything_node',
        output='screen',
        namespace=ns,
        parameters=[{'use_sim_time': True}]
    )

    # 3. Start RTAB-Map
    # RTAB-Map requires:
    # - RGB image
    # - Depth image
    # - Camera Info
    # - Odometry (provided by tello_driver or gazebo ground truth)
    
    # Arguments for rtabmap_ros
    rtabmap_args = {
        'frame_id': 'base_link_1',  
        'map_frame_id': 'map', 
        # Gazebo Ground Truth publishes 'map' -> 'base_link_1'.
        # RTAB-Map expects 'odom_frame' -> 'base_link_frame'.
        # By setting odom_frame_id='map', we tell RTAB-Map that our odometry frame is named 'map'.
        'odom_frame_id': 'map', 
        
        # Disable RTAB-Map from publishing map->odom transform because we use Ground Truth (map->base_link) directly.
        'publish_tf': 'false', 

        'subscribe_depth': 'true',
        'approx_sync': 'true',
        'rgb_topic': '/drone1/depth/rgb',
        'depth_topic': '/drone1/depth/image_raw',
        'camera_info_topic': '/drone1/depth/camera_info',
        'odom_topic': '/drone1/odom',
        'queue_size': '10',
        'rviz': 'false', 
        'rtabmap_viz': 'false',
        'use_sim_time': 'true',
        
        # Important: Wait for TF to be available
        'wait_for_transform': '0.2',
    }

    # Since rtabmap_launch might not be available or complex to configure via IncludeLaunchDescription without `rtabmap_ros` package explicitly installed in a standard way,
    # we can try to launch the nodes directly or use `rtabmap.launch.py` if present.
    # The user environment showed `rtabmap_launch` in `ros2 pkg list`.
    
    # Let's try to use the standard rtabmap launch
    # We need to find where rtabmap_launch is.
    try:
        rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
        rtabmap_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
            ),
                launch_arguments={
                'namespace': 'drone1',
                'use_sim_time': 'true',

                'frame_id': 'base_link_1',
                'odom_frame_id': 'odom',   # ← mapにしない！
                'map_frame_id': 'map',

                'publish_tf_map': 'true',
                'visual_odometry': 'true',
                'icp_odometry': 'false',

                # Depth/RGB are published with identical stamps by depth_anything_node.
                # Use exact sync to prevent mismatching frames.
                'approx_sync': 'false',
                'approx_sync_max_interval': '0.02',
                'topic_queue_size': '30',
                'sync_queue_size': '30',
                'qos': '2',

                'rgb_topic': '/drone1/depth/rgb',
                'depth_topic': '/drone1/depth/image_raw',
                'camera_info_topic': '/drone1/depth/camera_info',

                'rviz': 'true',
                'rtabmap_viz': 'false',
                }.items()

        )



    except Exception:
        # Fallback if rtabmap_launch is not found (though grep showed it)
        # Just a placeholder log
        rtabmap_launch = ExecuteProcess(cmd=['echo', 'RTAB-Map launch file not found'], output='screen')

    # Static transform might be needed between base_link and camera_link if not provided by URDF properly or if frames mismatch
    # simple_launch starts robot_state_publisher with tello.urdf. 
    # tello.xml has camera_joint fixed. 
    # However, optical frame is usually needed for cameras (z forward).
    # Standard ROS cameras: x right, y down, z forward.
    # Tello URDF camera_link: <visual> sphere... 
    # Gazebo camera plugin: <frameName>camera_link${suffix}</frameName>
    # Gazebo camera plugin outputs images in optical frame convention usually? No, it outputs in the link frame unless hackBaseline etc or specific convention is used.
    # Actually standard gazebo camera plugin aligns with the link orientation: x forward, y left, z up (standard ROS body).
    # But Images are usually x right, y down, z forward.
    # We might need a static transform for optical frame.
    
    tf_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'camera_link_1', 'camera_optical_link_1']
    )

    # 4. Odom TF is intentionally omitted here.
    # RTAB-Map (visual odometry) will publish odom -> base_link_1.
    # Broadcasting another odom TF from Gazebo ground truth can twist the tree.

    return LaunchDescription([
        simple_launch,
        depth_node,
        rtabmap_launch,
        tf_optical
    ])
