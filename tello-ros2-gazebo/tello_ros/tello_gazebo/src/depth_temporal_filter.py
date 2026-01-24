#!/usr/bin/env python3
"""
Depth Temporal Filter Node

深度画像の時間平滑化を行い、Visual Odometryの精度向上を目指す。

理論:
- Depth Anything V2の深度推定はフレーム間でノイズがある
- EMA (Exponential Moving Average) で時間方向に平滑化
- 急な変化（動的物体）は新しい値を採用

データフロー:
  /depth/image_raw → [このノード] → /depth/filtered → RTAB-Map rgbd_odom
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters


class DepthTemporalFilter(Node):
    def __init__(self):
        super().__init__('depth_temporal_filter')

        # パラメータ宣言
        self.declare_parameter('input_depth_topic', 'depth/image_raw')
        self.declare_parameter('input_rgb_topic', 'depth/rgb')
        self.declare_parameter('input_camera_info_topic', 'depth/camera_info')
        self.declare_parameter('output_depth_topic', 'depth/filtered')
        self.declare_parameter('output_rgb_topic', 'depth/filtered_rgb')
        self.declare_parameter('output_camera_info_topic', 'depth/filtered_camera_info')
        
        # 平滑化パラメータ
        self.declare_parameter('ema_alpha', 0.7)  # 新しい値の重み (0.5-0.9推奨)
        self.declare_parameter('change_threshold', 0.3)  # これ以上の変化は新しい値を採用 (m)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('confidence_decay', 0.95)  # 信頼度の減衰率
        self.declare_parameter('min_confidence', 0.3)  # 最小信頼度（これ以下はNaN）

        # パラメータ取得
        self.input_depth_topic = self.get_parameter('input_depth_topic').value
        self.input_rgb_topic = self.get_parameter('input_rgb_topic').value
        self.input_camera_info_topic = self.get_parameter('input_camera_info_topic').value
        self.output_depth_topic = self.get_parameter('output_depth_topic').value
        self.output_rgb_topic = self.get_parameter('output_rgb_topic').value
        self.output_camera_info_topic = self.get_parameter('output_camera_info_topic').value
        
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.change_threshold = self.get_parameter('change_threshold').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.confidence_decay = self.get_parameter('confidence_decay').value
        self.min_confidence = self.get_parameter('min_confidence').value

        # 状態変数
        self.prev_depth = None
        self.confidence_map = None  # 各ピクセルの信頼度
        self.bridge = CvBridge()

        # Subscriber (同期)
        self.depth_sub = message_filters.Subscriber(self, Image, self.input_depth_topic)
        self.rgb_sub = message_filters.Subscriber(self, Image, self.input_rgb_topic)
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, self.input_camera_info_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.rgb_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.callback)

        # Publisher
        self.depth_pub = self.create_publisher(Image, self.output_depth_topic, 10)
        self.rgb_pub = self.create_publisher(Image, self.output_rgb_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.output_camera_info_topic, 10)

        self.get_logger().info(
            f"DepthTemporalFilter: {self.input_depth_topic} -> {self.output_depth_topic} "
            f"(alpha={self.ema_alpha}, threshold={self.change_threshold}m)"
        )

    def callback(self, depth_msg: Image, rgb_msg: Image, camera_info_msg: CameraInfo):
        # 深度画像をnumpy配列に変換
        if depth_msg.encoding == '32FC1':
            depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
                depth_msg.height, depth_msg.width
            ).copy()
        elif depth_msg.encoding == '16UC1':
            depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
                depth_msg.height, depth_msg.width
            ).astype(np.float32) * 0.001
        else:
            self.get_logger().warn(f"Unsupported encoding: {depth_msg.encoding}")
            return

        # 深度範囲外をNaNに
        depth[(depth < self.min_depth) | (depth > self.max_depth)] = np.nan

        # フィルタリング処理
        filtered_depth = self.filter_depth(depth)

        # メッセージ作成・発行
        filtered_msg = self.bridge.cv2_to_imgmsg(filtered_depth, encoding='32FC1')
        filtered_msg.header = depth_msg.header

        # RGB と CameraInfo はそのまま転送（タイムスタンプを揃える）
        rgb_out = rgb_msg
        camera_info_out = camera_info_msg

        self.depth_pub.publish(filtered_msg)
        self.rgb_pub.publish(rgb_out)
        self.camera_info_pub.publish(camera_info_out)

    def filter_depth(self, depth: np.ndarray) -> np.ndarray:
        """
        深度画像の時間平滑化
        
        アルゴリズム:
        1. 前フレームがなければそのまま返す
        2. 各ピクセルで:
           - 大きな変化（> threshold）→ 新しい値を採用、信頼度リセット
           - 小さな変化 → EMA平滑化、信頼度増加
        3. 信頼度が低いピクセルはNaN
        """
        if self.prev_depth is None:
            self.prev_depth = depth.copy()
            self.confidence_map = np.where(np.isfinite(depth), 0.5, 0.0)
            return depth

        # 出力配列
        filtered = depth.copy()
        
        # 有効なピクセルのマスク
        curr_valid = np.isfinite(depth)
        prev_valid = np.isfinite(self.prev_depth)
        both_valid = curr_valid & prev_valid

        # 変化量を計算
        change = np.abs(depth - self.prev_depth)
        
        # ケース1: 両方有効で小さな変化 → EMA平滑化
        small_change = both_valid & (change <= self.change_threshold)
        filtered[small_change] = (
            self.ema_alpha * depth[small_change] +
            (1.0 - self.ema_alpha) * self.prev_depth[small_change]
        )
        
        # 信頼度更新
        # - 小さな変化で観測継続 → 信頼度増加
        # - 大きな変化 → 信頼度リセット
        # - 観測なし → 信頼度減衰
        
        new_confidence = self.confidence_map * self.confidence_decay  # 減衰
        
        # 小さな変化で継続観測 → 信頼度増加
        new_confidence[small_change] = np.minimum(
            1.0,
            self.confidence_map[small_change] + 0.15
        )
        
        # 大きな変化（動的物体 or ノイズ）→ 信頼度リセット
        big_change = both_valid & (change > self.change_threshold)
        new_confidence[big_change] = 0.3  # 新しい観測なので中程度の信頼度
        
        # 新しく観測された領域 → 初期信頼度
        new_observation = curr_valid & ~prev_valid
        new_confidence[new_observation] = 0.4
        
        # 観測がなくなった領域 → 信頼度急減
        lost_observation = ~curr_valid & prev_valid
        new_confidence[lost_observation] *= 0.5
        
        # 信頼度が低いピクセルはNaN（Visual Odomで使わない）
        low_confidence = new_confidence < self.min_confidence
        filtered[low_confidence] = np.nan

        # 画像の左右端（5%ずつ）をNaNにする（端ノイズ除去）
        h, w = filtered.shape
        edge_ratio = 0.05  # 5%ずつ
        edge_px = int(w * edge_ratio)
        if edge_px > 0:
            filtered[:, :edge_px] = np.nan  # 左端
            filtered[:, -edge_px:] = np.nan  # 右端
        
        # 状態を更新
        self.prev_depth = filtered.copy()
        self.confidence_map = new_confidence
        
        return filtered


def main(args=None):
    rclpy.init(args=args)
    node = DepthTemporalFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
