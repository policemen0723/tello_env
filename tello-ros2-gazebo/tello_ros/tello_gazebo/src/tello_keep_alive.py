#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TelloKeepAlive(Node):
    def __init__(self):
        super().__init__('tello_keep_alive')
        
        # Publisher to the control topic used by the real Tello driver
        self.pub = self.create_publisher(Twist, 'control', 10)
        
        # Timer to send hover command every 2 seconds
        # Tello auto-lands after ~15s of no activity, so 2s is safe.
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info("Tello Keep-Alive node started. Sending hover command every 2 seconds.")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = 0.0
        
        self.pub.publish(msg)
        self.get_logger().debug("Sent hover command to keep Tello alive.")

def main(args=None):
    rclpy.init(args=args)
    node = TelloKeepAlive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
