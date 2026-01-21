#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import re
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from nav_msgs.msg import Odometry as NavOdometry, OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
from transformers import pipeline
from PIL import Image as PILImage
import message_filters

class DepthAnythingOutdoorNode(Node):
    def __init__(self):
        super().__init__('depth_anything_outdoor_node')
        
        self.get_logger().info("Loading Depth Anything V2 Metric Outdoor Small model...")
        self.get_logger().info("Note: If the model is not found locally, it will be downloaded from Hugging Face Hub. This may take a while...")
        # Using the official model fine-tuned for metric depth on outdoor scenes
        try:
            # モデルのダウンロードを許可するために local_files_only を False に変更
            self.depth_estimator = pipeline(
                task="depth-estimation", 
                model="depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf",
                model_kwargs={"local_files_only": False}
            )
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.depth_estimator = None

        self.bridge = CvBridge()
        self._warned_zero_stamp = False

        # Parameters for topic names (relative by default so namespace works).
        self.declare_parameter('rgb_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('depth_topic', 'depth/image_raw')
        self.declare_parameter('depth_rgb_topic', 'depth/rgb')
        self.declare_parameter('depth_camera_info_topic', 'depth/camera_info')
        self.declare_parameter('optical_frame_id', '')

        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        depth_rgb_topic = self.get_parameter('depth_rgb_topic').get_parameter_value().string_value
        depth_camera_info_topic = self.get_parameter('depth_camera_info_topic').get_parameter_value().string_value
        self.optical_frame_id = self.get_parameter('optical_frame_id').get_parameter_value().string_value

        # Subscribers
        self.img_sub = message_filters.Subscriber(self, ROSImage, rgb_topic)
        self.camera_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)

        # Synchronize exactly to prevent RGB/Depth/CameraInfo timestamp skew.
        self.ts = message_filters.TimeSynchronizer(
            [self.img_sub, self.camera_sub],
            queue_size=10,
        )
        self.ts.registerCallback(self.callback)

        # Publishers
        self.depth_image_pub = self.create_publisher(ROSImage, depth_topic, 10)
        self.rgb_pub = self.create_publisher(ROSImage, depth_rgb_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, depth_camera_info_topic, 10)
        
        self.get_logger().info("Depth Anything Outdoor Node initialized.")

    def _suffix_from_namespace(self):
        ns = self.get_namespace().strip('/')
        match = re.search(r'(\d+)$', ns)
        if match:
            return f"_{match.group(1)}"
        return ""

    def estimate_depth(self, cv_image):
        if self.depth_estimator is None:
            return None
        
        # Convert OpenCV image (BGR) to PIL Image (RGB)
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image = PILImage.fromarray(cv_image_rgb)

        # Inference
        result = self.depth_estimator(image)
        
        # Extract predicted depth (metric, meters)
        depth_tensor = result["predicted_depth"]
        depth_numpy = depth_tensor.numpy().astype(np.float32)

        return depth_numpy

    def callback(self, img_msg, cam_info_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        try:
            depth_image = self.estimate_depth(cv_image)
            if depth_image is not None:
                # Resize depth image to match original image size if necessary
                if depth_image.shape[:2] != cv_image.shape[:2]:
                    depth_image = cv2.resize(depth_image, (cv_image.shape[1], cv_image.shape[0]))

                # --- Current Time Synchronization Strategy ---
                # Always use current node time to avoid TF_OLD_DATA errors due to inference delay.
                timestamp = self.get_clock().now().to_msg()
                
                if self.optical_frame_id:
                    frame_id = self.optical_frame_id
                else:
                    frame_id = cam_info_msg.header.frame_id or ""
                    suffix = self._suffix_from_namespace()
                    if not frame_id or "camera" not in frame_id:
                        frame_id = f"camera_optical_link{suffix}"
                    elif "camera_optical_link" in frame_id:
                        pass
                    elif "camera_link" in frame_id:
                        frame_id = frame_id.replace("camera_link", "camera_optical_link")
                    else:
                        frame_id = f"camera_optical_link{suffix}"
                    if suffix and not re.search(r'_\d+$', frame_id):
                        frame_id = f"{frame_id}{suffix}"
                
                # 1. Prepare Depth Image Message
                depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
                depth_image_msg.header.stamp = timestamp
                depth_image_msg.header.frame_id = frame_id
                
                # 2. Prepare RGB Image Message
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = frame_id
                
                # 3. Prepare Camera Info Message
                cam_info_msg.header.stamp = timestamp
                cam_info_msg.header.frame_id = frame_id

                # Publish all
                self.depth_image_pub.publish(depth_image_msg)
                self.rgb_pub.publish(img_msg)
                self.camera_info_pub.publish(cam_info_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingOutdoorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
