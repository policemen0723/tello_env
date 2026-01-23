#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import re
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from transformers import pipeline
from PIL import Image as PILImage
import message_filters
from message_filters import ApproximateTimeSynchronizer # 近似同期用に追加

class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__('depth_anything_node')
        
        # モデル設定
        self.declare_parameter('input_scale', 1.0) 
        self.input_scale = self.get_parameter('input_scale').value
        self.get_logger().info("Loading Depth Anything V2 Metric Indoor Small model...")
        
        try:
            import torch
            if torch.cuda.is_available():
                device = 0  # Use first GPU
                self.get_logger().info(f"CUDA available: {torch.cuda.get_device_name(0)}")
            else:
                device = -1  # CPU fallback
                self.get_logger().warn("CUDA not available, using CPU (slow)")
            
            self.depth_estimator = pipeline(
                task="depth-estimation", 
                model="depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",
                model_kwargs={"local_files_only": True},
                device=device
            )
            self.get_logger().info(f"Model loaded successfully on {'GPU' if device >= 0 else 'CPU'}.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.depth_estimator = None

        self.bridge = CvBridge()
        
        # タイムスタンプ重複防止用の変数
        self.last_applied_stamp_nanos = 0

        # トピック名の設定
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

        # Subscribers
        self.img_sub = message_filters.Subscriber(self, ROSImage, rgb_topic)
        self.camera_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)

        # 【修正】ApproximateTimeSynchronizer に変更 (厳密な同期によるドロップを防止)
        self.ts = ApproximateTimeSynchronizer(
            [self.img_sub, self.camera_sub],
            queue_size=20,
            slop=0.05 # 50msまでのズレを許容
        )
        self.ts.registerCallback(self.callback)

        # Publishers
        self.depth_image_pub = self.create_publisher(ROSImage, depth_topic, 10)
        self.rgb_pub = self.create_publisher(ROSImage, depth_rgb_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, depth_camera_info_topic, 10)
        
        self.get_logger().info("Depth Anything Node initialized with Approximate Sync.")

    def _suffix_from_namespace(self):
        ns = self.get_namespace().strip('/')
        match = re.search(r'(\d+)$', ns)
        if match:
            return f"_{match.group(1)}"
        return ""

    def estimate_depth(self, cv_image):
        if self.depth_estimator is None:
            return None
        
        if self.input_scale != 1.0:
            h, w = cv_image.shape[:2]
            inference_image = cv2.resize(cv_image, (int(w * self.input_scale), int(h * self.input_scale)))
        else:
            inference_image = cv_image

        cv_image_rgb = cv2.cvtColor(inference_image, cv2.COLOR_BGR2RGB)
        image = PILImage.fromarray(cv_image_rgb)
        result = self.depth_estimator(image)
        
        depth_tensor = result["predicted_depth"]
        return depth_tensor.numpy().astype(np.float32)

    def callback(self, img_msg, cam_info_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        try:
            depth_image = self.estimate_depth(cv_image)
            if depth_image is not None:
                if depth_image.shape[:2] != cv_image.shape[:2]:
                    depth_image = cv2.resize(depth_image, (cv_image.shape[1], cv_image.shape[0]))

                # --- 【重要】タイムスタンプ戦略の修正 ---
                # 1. now() を使わず元の画像スタンプを継承 (TF同期のため)
                # 2. ただし前回と同じ時刻なら1ナノ秒進める (RTAB-Map重複拒否防止のため)
                
                timestamp = img_msg.header.stamp
                current_nanos = timestamp.sec * 10**9 + timestamp.nanosec

                if current_nanos <= self.last_applied_stamp_nanos:
                    current_nanos = self.last_applied_stamp_nanos + 1
                
                self.last_applied_stamp_nanos = current_nanos
                
                # 修正したナノ秒をスタンプに書き戻す
                timestamp.sec = current_nanos // 10**9
                timestamp.nanosec = current_nanos % 10**9
                
                # Frame ID の決定
                if self.optical_frame_id:
                    frame_id = self.optical_frame_id
                else:
                    frame_id = cam_info_msg.header.frame_id or "camera_optical_link"
                    suffix = self._suffix_from_namespace()
                    if suffix and not re.search(r'_\d+$', frame_id):
                        frame_id = f"{frame_id}{suffix}"
                
                # メッセージの準備とパブリッシュ
                depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
                depth_image_msg.header.stamp = timestamp
                depth_image_msg.header.frame_id = frame_id
                
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = frame_id
                
                cam_info_msg.header.stamp = timestamp
                cam_info_msg.header.frame_id = frame_id

                self.depth_image_pub.publish(depth_image_msg)
                self.rgb_pub.publish(img_msg)
                self.camera_info_pub.publish(cam_info_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    # 負荷分散のためマルチスレッドExecutorを推奨
    executor = rclpy.executors.MultiThreadedExecutor()
    node = DepthAnythingNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()