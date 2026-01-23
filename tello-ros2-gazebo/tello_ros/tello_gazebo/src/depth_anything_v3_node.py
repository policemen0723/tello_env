#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import re
import torch
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from depth_anything_3.api import DepthAnything3
import message_filters

class DepthAnythingV3Node(Node):
    def __init__(self):
        super().__init__('depth_anything_v3_node')
        
        # パラメータ設定 (V2の構成を継承)
        self.declare_parameter('input_scale', 1.0) 
        self.input_scale = self.get_parameter('input_scale').value
        
        # モデルのロード (DA3-SMALL をデフォルトに設定)
        self.get_logger().info(f"Loading Depth Anything V3 Small model with input scale {self.input_scale}...")
        
        try:
            # DA3 APIを使用してモデルをロード
            # Metric専用のSmallはないため、SMALLをロードして後で変換
            self.model = DepthAnything3.from_pretrained("depth-anything/DA3-SMALL")
            self.model.to("cuda" if torch.cuda.is_available() else "cpu")
            self.get_logger().info("DA3 Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load DA3 model: {e}")
            self.model = None

        self.bridge = CvBridge()

        # トピック名の設定 (V2と同一)
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

        # 同期設定
        self.ts = message_filters.TimeSynchronizer(
            [self.img_sub, self.camera_sub],
            queue_size=10,
        )
        self.ts.registerCallback(self.callback)

        # Publishers
        self.depth_image_pub = self.create_publisher(ROSImage, depth_topic, 10)
        self.rgb_pub = self.create_publisher(ROSImage, depth_rgb_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, depth_camera_info_topic, 10)
        
        self.get_logger().info("Depth Anything V3 Node initialized.")

    def _suffix_from_namespace(self):
        ns = self.get_namespace().strip('/')
        match = re.search(r'(\d+)$', ns)
        if match:
            return f"_{match.group(1)}"
        return ""

    def estimate_depth(self, cv_image, cam_info_msg):
        if self.model is None:
            return None
        
        # 推論用のリサイズ処理
        if self.input_scale != 1.0:
            h, w = cv_image.shape[:2]
            inference_image = cv2.resize(cv_image, (int(w * self.input_scale), int(h * self.input_scale)))
        else:
            inference_image = cv_image

        # DA3 推論 (BGRをそのまま受け取り可能)
        # prediction.depth は [1, H, W]
        prediction = self.model.inference([inference_image])
        raw_depth = prediction.depth[0]

        # --- DA3-SMALL 用の Metric 変換 ---
        # fx (焦点距離) を CameraInfo から取得
        fx = cam_info_msg.k[0] if cam_info_msg.k[0] != 0 else 1000.0
        # 公式の計算式: metric = (focal * raw) / 300
        metric_depth = (fx * raw_depth) / 300.0

        return metric_depth.astype(np.float32)

    def callback(self, img_msg, cam_info_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        try:
            # 深度推定実行
            depth_image = self.estimate_depth(cv_image, cam_info_msg)
            
            if depth_image is not None:
                # 出力を元の解像度に戻す
                if depth_image.shape[:2] != cv_image.shape[:2]:
                    depth_image = cv2.resize(depth_image, (cv_image.shape[1], cv_image.shape[0]))

                # タイムスタンプ生成 (TF_OLD_DATA防止)
                timestamp = self.get_clock().now().to_msg()
                
                # Frame ID の決定ロジック (V2と同一)
                if self.optical_frame_id:
                    frame_id = self.optical_frame_id
                else:
                    frame_id = cam_info_msg.header.frame_id or ""
                    suffix = self._suffix_from_namespace()
                    if not frame_id or "camera" not in frame_id:
                        frame_id = f"camera_optical_link{suffix}"
                    elif "camera_link" in frame_id:
                        frame_id = frame_id.replace("camera_link", "camera_optical_link")
                    if suffix and not re.search(r'_\d+$', frame_id):
                        frame_id = f"{frame_id}{suffix}"
                
                # 1. Depth Image (32FC1 = メートル単位の浮動小数)
                depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
                depth_image_msg.header.stamp = timestamp
                depth_image_msg.header.frame_id = frame_id
                
                # 2. RGB Image
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = frame_id
                
                # 3. Camera Info
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
    node = DepthAnythingV3Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()