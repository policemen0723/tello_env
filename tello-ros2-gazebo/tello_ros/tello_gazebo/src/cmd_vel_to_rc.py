#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelToRC(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_rc')
        
        # Scaling parameters
        # Tello RC input is -100 to 100.
        # Max real speed of Tello is approx 8m/s (fast mode) but normally ~1-2 m/s.
        # We need to map linear velocity (m/s) to RC value (0-100).
        
        # Example: if 1.0 m/s is desired, what RC value should be sent?
        # Assuming 1.0 m/s ~= RC 30 (adjust as needed based on field tests)
        self.declare_parameter('linear_scale', 30.0)
        self.declare_parameter('angular_scale', 30.0)

        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value

        self.get_logger().info(f"CmdVelToRC initialized. Linear Scale: {self.linear_scale}, Angular Scale: {self.angular_scale}")

        # Subscriber: Listens to Nav2 cmd_vel (m/s)
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback,
            10
        )

        # Publisher: Sends RC commands to Tello driver
        self.pub = self.create_publisher(
            Twist,
            'control',
            10
        )

    def callback(self, msg):
        rc_msg = Twist()
        
        # Convert m/s to RC value (-100 to 100)
        # Swapping X and Y as requested by the user
        rc_msg.linear.x = msg.linear.y * self.linear_scale
        rc_msg.linear.y = msg.linear.x * self.linear_scale
        rc_msg.linear.z = msg.linear.z * self.linear_scale 

        # Angular velocity (rad/s) to RC value
        # Inverting the sign to match Tello's rotation convention
        rc_msg.angular.z = -1.0 * msg.angular.z * self.angular_scale

        # Clamp values to -100 to 100
        rc_msg.linear.x = max(min(rc_msg.linear.x, 100.0), -100.0)
        rc_msg.linear.y = max(min(rc_msg.linear.y, 100.0), -100.0)
        rc_msg.linear.z = max(min(rc_msg.linear.z, 100.0), -100.0)
        rc_msg.angular.z = max(min(rc_msg.angular.z, 100.0), -100.0)

        self.pub.publish(rc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
