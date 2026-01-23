#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuSanitizer(Node):
    def __init__(self):
        super().__init__('imu_sanitizer')

        self.declare_parameter('imu_in_topic', 'imu')
        self.declare_parameter('imu_out_topic', 'imu_sanitized')
        self.declare_parameter('frame_id', 'base_link_1')
        self.declare_parameter('orientation_covariance', [0.05, 0.05, 0.1])
        self.declare_parameter('angular_velocity_covariance', [0.01, 0.01, 0.02])
        self.declare_parameter('linear_acceleration_covariance', [0.1, 0.1, 0.2])

        self.imu_in_topic = self.get_parameter(
            'imu_in_topic').get_parameter_value().string_value
        self.imu_out_topic = self.get_parameter(
            'imu_out_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.orientation_cov = self.get_parameter(
            'orientation_covariance').get_parameter_value().double_array_value
        self.angular_vel_cov = self.get_parameter(
            'angular_velocity_covariance').get_parameter_value().double_array_value
        self.linear_acc_cov = self.get_parameter(
            'linear_acceleration_covariance').get_parameter_value().double_array_value

        self.pub = self.create_publisher(Imu, self.imu_out_topic, 10)
        self.sub = self.create_subscription(Imu, self.imu_in_topic, self.callback, 10)

        self.get_logger().info(
            f"IMU sanitizer: {self.imu_in_topic} -> {self.imu_out_topic}")

    def _diag_to_covariance(self, diag):
        return [
            diag[0], 0.0, 0.0,
            0.0, diag[1], 0.0,
            0.0, 0.0, diag[2],
        ]

    def callback(self, msg: Imu):
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id or msg.header.frame_id

        # Copy raw values
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.orientation = msg.orientation

        # Normalize quaternion to avoid denormalized / NaN issues.
        qx, qy, qz, qw = out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w
        if not all(math.isfinite(v) for v in (qx, qy, qz, qw)):
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        if not all(math.isfinite(v) for v in (
            out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z
        )):
            out.angular_velocity.x = 0.0
            out.angular_velocity.y = 0.0
            out.angular_velocity.z = 0.0
        if not all(math.isfinite(v) for v in (
            out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z
        )):
            out.linear_acceleration.x = 0.0
            out.linear_acceleration.y = 0.0
            out.linear_acceleration.z = 0.0
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9 or not math.isfinite(norm):
            out.orientation.x = 0.0
            out.orientation.y = 0.0
            out.orientation.z = 0.0
            out.orientation.w = 1.0
        else:
            out.orientation.x = qx / norm
            out.orientation.y = qy / norm
            out.orientation.z = qz / norm
            out.orientation.w = qw / norm

        out.orientation_covariance = self._diag_to_covariance(self.orientation_cov)
        out.angular_velocity_covariance = self._diag_to_covariance(self.angular_vel_cov)
        out.linear_acceleration_covariance = self._diag_to_covariance(self.linear_acc_cov)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
