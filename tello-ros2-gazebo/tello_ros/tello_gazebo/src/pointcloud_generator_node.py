#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
import struct

class PointCloudGeneratorNode(Node):
    def __init__(self):
        super().__init__('pointcloud_generator_node')
        
        self.declare_parameter('depth_topic', '/drone1/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/drone1/depth/camera_info')
        self.declare_parameter('point_cloud_topic', '/drone1/depth/points')
        
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        
        self.bridge = CvBridge()
        
        self.info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.info_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        
        self.pub = self.create_publisher(PointCloud2, pub_topic, 10)
        
        self.get_logger().info(f"PointCloud Generator started. Subscribing to {depth_topic}, Publishing to {pub_topic}")

    def callback(self, depth_msg, info_msg):
        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            # Camera intrinsics
            fx = info_msg.k[0]
            fy = info_msg.k[4]
            cx = info_msg.k[2]
            cy = info_msg.k[5]
            
            # Factor for unit conversion (if depth is in mm, factor=1000. If meters, factor=1)
            # DepthAnything/Gazebo usually outputs meters (float32)
            factor = 1.0 
            
            height, width = depth_image.shape
            
            # Generate grid of coordinates
            # Optimization: Downsample if necessary for performance
            step = 4 # Downsample factor (1 = full res, 4 = 1/16th points)
            
            u = np.arange(0, width, step)
            v = np.arange(0, height, step)
            uu, vv = np.meshgrid(u, v)
            
            # Extract depth at grid points
            z = depth_image[vv, uu] * factor
            
            # Filter valid depth
            valid = (z > 0.1) & (z < 10.0) & np.isfinite(z)
            z = z[valid]
            uu = uu[valid]
            vv = vv[valid]
            
            # Compute x, y
            x = (uu - cx) * z / fx
            y = (vv - cy) * z / fy
            
            # Stack for PointCloud2
            # Structure: x, y, z
            points = np.stack((x, y, z), axis=-1)
            
            # Create header
            header = depth_msg.header
            
            # Publish
            pc_msg = pc2.create_cloud_xyz32(header, points)
            self.pub.publish(pc_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate point cloud: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
