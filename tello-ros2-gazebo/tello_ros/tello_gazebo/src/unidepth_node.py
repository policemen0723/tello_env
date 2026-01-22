#!/usr/bin/env python3

import re
import cv2
import numpy as np
import rclpy
import torch
import torch.nn.functional as F
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image as ROSImage
import message_filters

try:
    from unidepth.models import UniDepthV2
except ImportError:
    UniDepthV2 = None


class UniDepthNode(Node):
    def __init__(self):
        super().__init__('unidepth_node')

        # Declare parameters
        self.declare_parameter('rgb_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('depth_topic', 'depth/image_raw')
        self.declare_parameter('depth_rgb_topic', 'depth/rgb')
        self.declare_parameter('depth_camera_info_topic', 'depth/camera_info')
        self.declare_parameter('optical_frame_id', 'camera_optical_link')
        self.declare_parameter('model_type', 'vits14')

        # Get parameters
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        depth_rgb_topic = self.get_parameter('depth_rgb_topic').get_parameter_value().string_value
        depth_camera_info_topic = self.get_parameter('depth_camera_info_topic').get_parameter_value().string_value
        self.optical_frame_id = self.get_parameter('optical_frame_id').get_parameter_value().string_value
        model_type = self.get_parameter('model_type').get_parameter_value().string_value

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.bridge = CvBridge()
        self._warned_zero_stamp = False
        self.model = None

        # Publishers
        self.depth_image_pub = self.create_publisher(ROSImage, depth_topic, 10)
        self.rgb_pub = self.create_publisher(ROSImage, depth_rgb_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, depth_camera_info_topic, 10)

        # Subscribers
        self.rgb_sub = message_filters.Subscriber(self, ROSImage, rgb_topic)
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)
        
        # ApproximateTimeSynchronizer is usually safer than TimeSynchronizer for different topics
        # providing slop=0.1s
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.cam_info_sub], 10, 0.1
        )
        self.ts.registerCallback(self.callback)

        if UniDepthV2 is None:
            self.get_logger().error("UniDepth library not found...")
        else:
            try:
                self.get_logger().info(f"Loading UniDepth V2 model ({model_type}) to {self.device}...")
                self.model = UniDepthV2.from_pretrained(f"lpiccinelli/unidepth-v2-{model_type}")
                self.model.to(self.device)
                self.model.eval()
                
                # 【追加】モデルをFP16（半精度）に変換
                if self.device.type == 'cuda':
                    self.model.half() 
                    self.get_logger().info("Model loaded successfully with FP16 optimization.")
                else:
                    self.get_logger().info("Model loaded successfully (CPU mode, FP32).")

            except Exception as exc:
                self.get_logger().error(f"Failed to load UniDepth model: {exc}")

    def _suffix_from_namespace(self):
        ns = self.get_namespace().strip('/')
        # Extract number if present
        match = re.search(r'(\d+)$', ns)
        if match:
            return f"_{match.group(1)}"
        return ""

    def estimate_depth_with_intrinsics(self, cv_image, K):
        if self.model is None:
            return None

        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        orig_h, orig_w = rgb.shape[:2]

        # Prepare input tensor
        if self.device.type == 'cuda':
            input_tensor = (
                torch.from_numpy(rgb)
                .permute(2, 0, 1)
                .unsqueeze(0)
                .to(self.device)
                .half() / 255.0
            )
        else:
            input_tensor = (
                torch.from_numpy(rgb)
                .permute(2, 0, 1)
                .unsqueeze(0)
                .to(self.device)
                .float() / 255.0
            )

        with torch.no_grad():
            if self.device.type == 'cuda':
                with torch.cuda.amp.autocast():
                    predictions = self.model.infer(input_tensor)
            else:
                predictions = self.model.infer(input_tensor)

            if isinstance(predictions, dict) and 'depth' in predictions:
                depth_map = predictions['depth']
            else:
                depth_map = predictions

            if depth_map.dim() == 3:
                depth_map = depth_map.unsqueeze(1)

            # Interpolation
            if depth_map.shape[-2:] != (orig_h, orig_w):
                depth_map = F.interpolate(
                    depth_map,
                    size=(orig_h, orig_w),
                    mode='bilinear',
                    align_corners=False,
                )

            depth_map = depth_map.squeeze(1)

        # Output to numpy
        depth_numpy = depth_map.squeeze().float().cpu().numpy().astype(np.float32)
        return depth_numpy

    def callback(self, img_msg, cam_info_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Error converting image: {exc}")
            return

        K_ros = np.array(cam_info_msg.k).reshape(3, 3)
        if np.all(K_ros == 0):
            self.get_logger().warn_once(
                "CameraInfo K matrix is all zeros. Metric depth will be inaccurate."
            )

        try:
            depth_image = self.estimate_depth_with_intrinsics(cv_image, K_ros)
            if depth_image is None:
                return

            # Handle timestamp
            if img_msg.header.stamp.sec == 0 and img_msg.header.stamp.nanosec == 0:
                timestamp = self.get_clock().now().to_msg()
                if not self._warned_zero_stamp:
                    self.get_logger().warn(
                        "Image stamp is zero; using node clock for depth outputs."
                    )
                    self._warned_zero_stamp = True
            else:
                timestamp = img_msg.header.stamp

            # Handle frame_id
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

            # Publish depth
            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
            depth_image_msg.header.stamp = timestamp
            depth_image_msg.header.frame_id = frame_id
            self.depth_image_pub.publish(depth_image_msg)

            # Publish RGB (synced)
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = frame_id
            self.rgb_pub.publish(img_msg)

            # Publish CameraInfo (synced)
            cam_info_msg.header.stamp = timestamp
            cam_info_msg.header.frame_id = frame_id
            self.camera_info_pub.publish(cam_info_msg)

        except Exception as exc:
            self.get_logger().error(f"Error processing image in UniDepth: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = UniDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()