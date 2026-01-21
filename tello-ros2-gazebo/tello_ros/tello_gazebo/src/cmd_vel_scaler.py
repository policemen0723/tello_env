#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelScaler(Node):
    def __init__(self):
        super().__init__('cmd_vel_scaler')
        
        # Scaling factor parameter (default: 0.3)
        self.declare_parameter('scale_factor', 0.3)
        self.scale_factor = self.get_parameter('scale_factor').get_parameter_value().double_value

        self.get_logger().info(f"CmdVelScaler initialized with factor: {self.scale_factor}")

        # Subscriber: Listens to the original command from Nav2
        # We expect Nav2 to publish to 'cmd_vel_nav' (remapped from cmd_vel)
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.callback,
            10
        )

        # Publisher: Sends the scaled command to the drone
        self.pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def callback(self, msg):
        scaled_msg = Twist()
        
        # Scale linear velocities
        scaled_msg.linear.x = msg.linear.x * self.scale_factor
        scaled_msg.linear.y = msg.linear.y * self.scale_factor
        scaled_msg.linear.z = msg.linear.z * self.scale_factor # Assuming z is used for drone

        # Scale angular velocities
        scaled_msg.angular.x = msg.angular.x * self.scale_factor
        scaled_msg.angular.y = msg.angular.y * self.scale_factor
        scaled_msg.angular.z = msg.angular.z * self.scale_factor

        self.pub.publish(scaled_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelScaler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
