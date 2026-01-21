#!/usr/bin/env python3

import math
import threading
from typing import Dict, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CameraInfo, Image as ROSImage, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
import message_filters
from std_msgs.msg import Header


class TemporalFusionNode(Node):
    def __init__(self):
        super().__init__('temporal_fusion_node')

        self.declare_parameter('depth_topic', 'depth/image_raw')
        self.declare_parameter('camera_info_topic', 'depth/camera_info')
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('output_topic', 'temporal/points')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('depth_frame', '')
        self.declare_parameter('stride', 4)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('ema_alpha', 0.6)
        self.declare_parameter('prune_after', 1.0)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('min_weight', 0.2)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('allow_latest_tf', False)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.depth_frame = self.get_parameter('depth_frame').value
        self.stride = int(self.get_parameter('stride').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.prune_after = float(self.get_parameter('prune_after').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.min_weight = float(self.get_parameter('min_weight').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.allow_latest_tf = bool(self.get_parameter('allow_latest_tf').value)

        cb_group = ReentrantCallbackGroup()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.depth_sub = message_filters.Subscriber(self, ROSImage, self.depth_topic, callback_group=cb_group)
        self.camera_sub = message_filters.Subscriber(self, CameraInfo, self.camera_info_topic, callback_group=cb_group)

        self.sync = message_filters.TimeSynchronizer(
            [self.depth_sub, self.camera_sub],
            queue_size=5,
        )
        self.sync.registerCallback(self.depth_callback)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_points, callback_group=cb_group)

        self._lock = threading.Lock()
        self._voxels: Dict[Tuple[int, int, int], Tuple[np.ndarray, float, float]] = {}
        self._last_stamp = None
        self._warned_tf = False
        self._warned_encoding = False

        self.get_logger().info(
            f"TemporalFusion: {self.depth_topic} + {self.camera_info_topic} -> {self.output_topic}"
        )

    def depth_callback(self, depth_msg: ROSImage, cam_info: CameraInfo):
        if depth_msg.encoding not in ("32FC1", "16UC1"):
            if not self._warned_encoding:
                self.get_logger().warn(
                    f"Unsupported depth encoding: {depth_msg.encoding}"
                )
                self._warned_encoding = True
            return

        source_frame = self.depth_frame or depth_msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                source_frame,
                depth_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            if self.allow_latest_tf:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.output_frame,
                        source_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.05),
                    )
                except Exception:
                    if not self._warned_tf:
                        self.get_logger().warn(
                            f"TF missing: {source_frame} -> {self.output_frame}"
                        )
                        self._warned_tf = True
                    return
            else:
                if not self._warned_tf:
                    self.get_logger().warn(
                        f"TF missing at stamp: {source_frame} -> {self.output_frame}"
                    )
                    self._warned_tf = True
                return

        depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
            depth_msg.height, depth_msg.width
        )
        if depth_msg.encoding == "16UC1":
            depth = depth.astype(np.float32) * 0.001

        K = np.array(cam_info.k, dtype=np.float32).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

        stride = max(self.stride, 1)
        v_coords = np.arange(0, depth.shape[0], stride)
        u_coords = np.arange(0, depth.shape[1], stride)
        uu, vv = np.meshgrid(u_coords, v_coords)

        z = depth[vv, uu]
        valid = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)
        if not np.any(valid):
            return

        z = z[valid]
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        points = np.stack([x, y, z], axis=1)
        points = self.apply_transform(points, transform)

        stamp_sec = float(depth_msg.header.stamp.sec) + depth_msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._last_stamp = depth_msg.header.stamp
            for p in points:
                key = (
                    int(math.floor(p[0] / self.voxel_size)),
                    int(math.floor(p[1] / self.voxel_size)),
                    int(math.floor(p[2] / self.voxel_size)),
                )
                if key in self._voxels:
                    prev_pos, weight, _ = self._voxels[key]
                    new_pos = self.ema_alpha * p + (1.0 - self.ema_alpha) * prev_pos
                    new_weight = min(1.0, weight + (1.0 - weight) * 0.1)
                    self._voxels[key] = (new_pos, new_weight, stamp_sec)
                else:
                    self._voxels[key] = (p, 0.2, stamp_sec)

    def publish_points(self):
        with self._lock:
            if not self._voxels:
                return
            if self._last_stamp is None:
                return

            now_sec = self.get_clock().now().nanoseconds * 1e-9
            stale_keys = [
                k for k, (_, _, t) in self._voxels.items()
                if (now_sec - t) > self.prune_after
            ]
            for k in stale_keys:
                del self._voxels[k]

            pts = [
                pos for (pos, weight, _) in self._voxels.values()
                if weight >= self.min_weight
            ]

        if not pts:
            return

        if len(pts) > self.max_points:
            pts = pts[: self.max_points]

        header = self._make_header(self._last_stamp, self.output_frame)
        cloud = point_cloud2.create_cloud_xyz32(header, pts)
        self.pub.publish(cloud)

    def _make_header(self, stamp, frame_id):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    @staticmethod
    def apply_transform(points: np.ndarray, tf_msg: tf2_ros.TransformStamped):
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        tx, ty, tz = t.x, t.y, t.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9:
            qx = qy = qz = 0.0
            qw = 1.0
        else:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rot = np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ], dtype=np.float32)

        return (points @ rot.T) + np.array([tx, ty, tz], dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = TemporalFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
