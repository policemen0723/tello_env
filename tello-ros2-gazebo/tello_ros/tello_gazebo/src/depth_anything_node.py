#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import re
import torch
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from transformers import pipeline
from PIL import Image as PILImage
import message_filters

class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__('depth_anything_node')
        
        self.get_logger().info("Loading Depth Anything V2 Metric Indoor Small model...")
        try:
            # Determine device
            device = 0 if torch.cuda.is_available() else -1
            device_name = "GPU" if device == 0 else "CPU"
            self.get_logger().info(f"Using device: {device_name}")

            # ローカルキャッシュのみを使用する設定
            self.depth_estimator = pipeline(
                task="depth-estimation", 
                model="depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",
                model_kwargs={"local_files_only": True},
                device=device
            )
            self.get_logger().info("Model loaded successfully (Offline Mode).")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.depth_estimator = None

        self.bridge = CvBridge()
        self._warned_zero_stamp = False

        # パラメータ設定
        self.declare_parameter('rgb_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('depth_topic', 'depth/image_raw')
        self.declare_parameter('depth_rgb_topic', 'depth/rgb')
        self.declare_parameter('depth_camera_info_topic', 'depth/camera_info')
        self.declare_parameter('optical_frame_id', '')

        rgb_topic = self.get_parameter('rgb_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_rgb_topic = self.get_parameter('depth_rgb_topic').value
        depth_camera_info_topic = self.get_parameter('depth_camera_info_topic').value
        self.optical_frame_id = self.get_parameter('optical_frame_id').value

        # 同期サブスクライバ
        self.img_sub = message_filters.Subscriber(self, ROSImage, rgb_topic)
        self.camera_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)

        self.ts = message_filters.TimeSynchronizer(
            [self.img_sub, self.camera_sub],
            queue_size=10,
        )
        self.ts.registerCallback(self.callback)

        # パブリッシャ
        self.depth_image_pub = self.create_publisher(ROSImage, depth_topic, 10)
        self.rgb_pub = self.create_publisher(ROSImage, depth_rgb_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, depth_camera_info_topic, 10)
        
        self.get_logger().info("Depth Anything Node initialized.")

    def _suffix_from_namespace(self):
        ns = self.get_namespace().strip('/')
        match = re.search(r'(\d+)$', ns)
        return f"_{match.group(1)}" if match else ""

    def estimate_depth(self, cv_image):
        if self.depth_estimator is None:
            return None
        
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image = PILImage.fromarray(cv_image_rgb)
        result = self.depth_estimator(image)
        
        depth_tensor = result["predicted_depth"]
        depth_numpy = depth_tensor.numpy().astype(np.float32)
        return np.squeeze(depth_numpy)

    def callback(self, img_msg, cam_info_msg):
        # 1. OpenCV 4.9.0 対策: 0x0 サイズは即座にリターン
        if img_msg.width == 0 or img_msg.height == 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            if cv_image is None or cv_image.size == 0:
                return

            depth_image = self.estimate_depth(cv_image)
            
            if depth_image is not None and depth_image.size > 0:
                h, w = cv_image.shape[:2]
                # サイズが異なる場合のみリサイズ (dsizeは width, height の順)
                if depth_image.shape[0] != h or depth_image.shape[1] != w:
                    depth_image = cv2.resize(depth_image, (w, h), interpolation=cv2.INTER_LINEAR)
            else:
                return

            # タイムスタンプ処理
            if img_msg.header.stamp.sec == 0 and img_msg.header.stamp.nanosec == 0:
                timestamp = self.get_clock().now().to_msg()
                if not self._warned_zero_stamp:
                    self.get_logger().warn("Image stamp is zero; using node clock.")
                    self._warned_zero_stamp = True
            else:
                timestamp = img_msg.header.stamp
            
            # フレームID処理
            if self.optical_frame_id:
                frame_id = self.optical_frame_id
            else:
                frame_id = cam_info_msg.header.frame_id or "camera_optical_link"
                suffix = self._suffix_from_namespace()
                if suffix and not frame_id.endswith(suffix):
                    frame_id = f"{frame_id}{suffix}"

            # メッセージ作成
            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
            depth_image_msg.header.stamp = timestamp
            depth_image_msg.header.frame_id = frame_id
            
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = frame_id
            
            cam_info_msg.header.stamp = timestamp
            cam_info_msg.header.frame_id = frame_id

            # 同時パブリッシュ
            self.depth_image_pub.publish(depth_image_msg)
            self.rgb_pub.publish(img_msg)
            self.camera_info_pub.publish(cam_info_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

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