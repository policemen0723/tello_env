#!/usr/bin/env python3

import os
# 【最強のオフライン設定】
os.environ["HF_HUB_OFFLINE"] = "1"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
import re
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from transformers import pipeline
from PIL import Image as PILImage

class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__('depth_anything_node_real')
        
        self.get_logger().info("Loading Depth Anything V2 (Debug Mode)...")
        try:
            self.depth_estimator = pipeline(
                task="depth-estimation", 
                model="depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",
                model_kwargs={"local_files_only": True}
            )
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.depth_estimator = None

        self.bridge = CvBridge()
        self.latest_camera_info = None

        # パラメータ設定
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

        # QoS設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            ROSImage, rgb_topic, self.image_callback, qos_profile
        )
        self.create_subscription(
            CameraInfo, camera_info_topic, self.info_callback, qos_profile
        )

        self.depth_image_pub = self.create_publisher(ROSImage, depth_topic, 10)
        self.rgb_pub = self.create_publisher(ROSImage, depth_rgb_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, depth_camera_info_topic, 10)
        
        self.get_logger().info("Depth Anything Node initialized.")
        self.get_logger().info(f"Target RGB Topic: {rgb_topic}")

    def info_callback(self, msg):
        if self.latest_camera_info is None:
            self.get_logger().info("[DEBUG] First Camera Info Received!", throttle_duration_sec=1.0)
        self.latest_camera_info = msg

    def estimate_depth(self, cv_image):
        if self.depth_estimator is None:
            self.get_logger().error("[DEBUG] Estimator is None!")
            return None
        
        # 推論開始ログ（遅い場合はここで止まる）
        # self.get_logger().info("[DEBUG] Start Inference...", throttle_duration_sec=2.0)
        
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image = PILImage.fromarray(cv_image_rgb)
        result = self.depth_estimator(image)
        
        depth_tensor = result["predicted_depth"]
        depth_numpy = depth_tensor.numpy().astype(np.float32)
        return depth_numpy

    def image_callback(self, img_msg):
        # 1. コールバックに入ったか確認（1秒に1回表示）
        self.get_logger().info(f"[DEBUG] Image Callback Triggered! Time: {img_msg.header.stamp.sec}.{img_msg.header.stamp.nanosec}", throttle_duration_sec=1.0)

        try:
            # 2. 時刻補正の確認
            if img_msg.header.stamp.sec == 0 and img_msg.header.stamp.nanosec == 0:
                self.get_logger().warn("[DEBUG] Zero Timestamp Detected! Fixing...", throttle_duration_sec=5.0)
                current_time = self.get_clock().now().to_msg()
                timestamp = current_time
                img_msg.header.stamp = current_time
            else:
                timestamp = img_msg.header.stamp

            # 3. 画像変換
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"[DEBUG] CvBridge Error: {e}")
            return

        try:
            # 4. 推論実行
            depth_image = self.estimate_depth(cv_image)
            
            if depth_image is not None:
                # 5. 推論成功
                # self.get_logger().info("[DEBUG] Inference Success!", throttle_duration_sec=1.0)

                if depth_image.shape[:2] != cv_image.shape[:2]:
                    depth_image = cv2.resize(depth_image, (cv_image.shape[1], cv_image.shape[0]))

                if self.optical_frame_id:
                    frame_id = self.optical_frame_id
                else:
                    frame_id = img_msg.header.frame_id
                    if "camera" not in frame_id: 
                         frame_id = "camera_optical_link_1" 

                # Depth画像作成
                depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
                depth_image_msg.header.stamp = timestamp
                depth_image_msg.header.frame_id = frame_id
                
                # 6. Publish実行
                self.depth_image_pub.publish(depth_image_msg)
                self.rgb_pub.publish(img_msg)
                
                self.get_logger().info("[DEBUG] Published Depth & RGB!", throttle_duration_sec=1.0)

                if self.latest_camera_info is not None:
                    out_info = self.latest_camera_info
                    out_info.header.stamp = timestamp
                    out_info.header.frame_id = frame_id
                    out_info.width = img_msg.width
                    out_info.height = img_msg.height
                    self.camera_info_pub.publish(out_info)
                else:
                    self.get_logger().warn("[DEBUG] No CameraInfo yet, skipping info pub.", throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f"[DEBUG] Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()