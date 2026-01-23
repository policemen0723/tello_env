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


# ... (前後のimportなどはそのまま)

class UniDepthNode(Node):
    def __init__(self):
        # ... (パラメータ宣言などはそのまま)

        if UniDepthV2 is None:
            self.get_logger().error("UniDepth library not found...")
        else:
            try:
                self.get_logger().info(f"Loading UniDepth V2 model ({model_type}) to {self.device}...")
                self.model = UniDepthV2.from_pretrained(f"lpiccinelli/unidepth-v2-{model_type}")
                self.model.to(self.device)
                self.model.eval()
                
                # 【追加】モデルをFP16（半精度）に変換
                self.model.half() 
                
                self.get_logger().info("Model loaded successfully with FP16 optimization.")
            except Exception as exc:
                self.get_logger().error(f"Failed to load UniDepth model: {exc}")

    def estimate_depth_with_intrinsics(self, cv_image, K):
        if self.model is None:
            return None

        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb_resized = cv2.resize(rgb, (512, 512))
        orig_h, orig_w = rgb.shape[:2]

        # 【最適化】画像データをGPUへ送り、FP16に変換
        input_tensor = (
            torch.from_numpy(rgb)
            .permute(2, 0, 1)
            .unsqueeze(0)
            .to(self.device)
            .half() / 255.0  # .float()を.half()に変更
        )

        with torch.no_grad():
            # 【最適化】混合精度コンテキストを使用して実行
            with torch.cuda.amp.autocast():
                predictions = self.model.infer(input_tensor)

            if isinstance(predictions, dict) and 'depth' in predictions:
                depth_map = predictions['depth']
            else:
                depth_map = predictions

            if depth_map.dim() == 3:
                depth_map = depth_map.unsqueeze(1)

            # 補間処理
            if depth_map.shape[-2:] != (orig_h, orig_w):
                depth_map = F.interpolate(
                    depth_map,
                    size=(orig_h, orig_w),
                    mode='bilinear',
                    align_corners=False,
                )

            depth_map = depth_map.squeeze(1)

        # 出力は32bit浮動小数（ROS標準）に戻してnumpy化
        depth_numpy = depth_map.squeeze().float().cpu().numpy().astype(np.float32)
        return depth_numpy

# ... (残りのcallbackなどはそのまま)

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

            if img_msg.header.stamp.sec == 0 and img_msg.header.stamp.nanosec == 0:
                timestamp = self.get_clock().now().to_msg()
                if not self._warned_zero_stamp:
                    self.get_logger().warn(
                        "Image stamp is zero; using node clock for depth outputs."
                    )
                    self._warned_zero_stamp = True
            else:
                timestamp = img_msg.header.stamp

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

            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
            depth_image_msg.header.stamp = timestamp
            depth_image_msg.header.frame_id = frame_id
            self.depth_image_pub.publish(depth_image_msg)

            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = frame_id
            self.rgb_pub.publish(img_msg)

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