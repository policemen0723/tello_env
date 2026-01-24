#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped


class CmdVelToTwistCov(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_twist_cov')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('twist_topic', 'twist')
        self.declare_parameter('frame_id', 'base_link_1')
        self.declare_parameter('scale_xy', 2.0)
        self.declare_parameter('scale_z', 4.0)
        self.declare_parameter('scale_yaw', math.pi)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_z', False)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('zero_y', True)
        self.declare_parameter('linear_covariance', 0.2)
        self.declare_parameter('angular_covariance', 0.5)

        cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value
        twist_topic = self.get_parameter(
            'twist_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.scale_xy = self.get_parameter(
            'scale_xy').get_parameter_value().double_value
        self.scale_z = self.get_parameter(
            'scale_z').get_parameter_value().double_value
        self.scale_yaw = self.get_parameter(
            'scale_yaw').get_parameter_value().double_value
        self.invert_x = self.get_parameter(
            'invert_x').get_parameter_value().bool_value
        self.invert_z = self.get_parameter(
            'invert_z').get_parameter_value().bool_value
        self.invert_yaw = self.get_parameter(
            'invert_yaw').get_parameter_value().bool_value
        self.zero_y = self.get_parameter(
            'zero_y').get_parameter_value().bool_value
        self.linear_covariance = self.get_parameter(
            'linear_covariance').get_parameter_value().double_value
        self.angular_covariance = self.get_parameter(
            'angular_covariance').get_parameter_value().double_value

        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, twist_topic, 10)
        self.sub = self.create_subscription(
            Twist, cmd_vel_topic, self.callback, 10)

        self.get_logger().info(
            f"cmd_vel -> TwistWithCovariance: {cmd_vel_topic} -> {twist_topic}")

    def callback(self, msg: Twist):
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id

        sign_x = -1.0 if self.invert_x else 1.0
        sign_z = -1.0 if self.invert_z else 1.0
        sign_yaw = -1.0 if self.invert_yaw else 1.0
        twist_msg.twist.twist.linear.x = msg.linear.x * self.scale_xy * sign_x
        twist_msg.twist.twist.linear.y = 0.0 if self.zero_y else msg.linear.y * self.scale_xy
        twist_msg.twist.twist.linear.z = msg.linear.z * self.scale_z * sign_z
        twist_msg.twist.twist.angular.z = msg.angular.z * self.scale_yaw * sign_yaw

        lin_cov = self.linear_covariance
        ang_cov = self.angular_covariance
        twist_msg.twist.covariance = [
            lin_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, lin_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, lin_cov, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, ang_cov,
        ]

        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToTwistCov()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
