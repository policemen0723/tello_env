#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from tello_msgs.msg import FlightData


class FlightDataToTwist(Node):
    def __init__(self):
        super().__init__('flight_data_to_twist')

        self.declare_parameter('flight_data_topic', 'flight_data')
        self.declare_parameter('twist_topic', 'flight_twist_cov')
        self.declare_parameter('frame_id', 'base_link_1')
        self.declare_parameter('velocity_scale', 0.01)  # cm/s -> m/s
        self.declare_parameter('invert_y', True)
        self.declare_parameter('linear_covariance', 0.05)

        flight_data_topic = self.get_parameter(
            'flight_data_topic').get_parameter_value().string_value
        twist_topic = self.get_parameter(
            'twist_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.velocity_scale = self.get_parameter(
            'velocity_scale').get_parameter_value().double_value
        self.invert_y = self.get_parameter(
            'invert_y').get_parameter_value().bool_value
        self.linear_covariance = self.get_parameter(
            'linear_covariance').get_parameter_value().double_value

        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, twist_topic, 10)
        self.sub = self.create_subscription(
            FlightData, flight_data_topic, self.callback, 10)

        self.get_logger().info(
            f"FlightData -> TwistStamped: {flight_data_topic} -> {twist_topic}")

    def callback(self, msg: FlightData):
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = msg.header.stamp
        twist_msg.header.frame_id = self.frame_id

        scale = self.velocity_scale
        twist_msg.twist.twist.linear.x = msg.vgx * scale
        twist_msg.twist.twist.linear.y = (-msg.vgy if self.invert_y else msg.vgy) * scale
        twist_msg.twist.twist.linear.z = msg.vgz * scale
        cov = self.linear_covariance
        twist_msg.twist.covariance = [
            cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, cov, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 9999.0,
        ]

        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlightDataToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
