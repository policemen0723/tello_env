#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class TelloTakeoffHover(Node):
    def __init__(self):
        super().__init__('tello_takeoff_hover')
        
        # Publishers
        self.takeoff_pub = self.create_publisher(Empty, 'takeoff', 10)
        self.control_pub = self.create_publisher(Twist, 'control', 10)
        
        # 1. Send Takeoff command immediately
        self.get_logger().info("Sending Takeoff command...")
        msg = Empty()
        self.takeoff_pub.publish(msg)
        
        # 2. Wait for the drone to finish taking off
        # Tello usually takes about 5-7 seconds to stabilize at 1m.
        self.get_logger().info("Waiting 7 seconds for takeoff to complete...")
        
        # We use a timer to start the hover loop after the wait, 
        # so we don't block the rclpy thread.
        self.start_time = time.time()
        self.hover_started = False
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        
        if elapsed > 7.0:
            if not self.hover_started:
                self.get_logger().info("Takeoff complete. Starting continuous hover commands (Keep-Alive).")
                self.hover_started = True
            
            # Send hover command (all zeros)
            # Tello driver (tello-ros) uses RC values (-100 to 100)
            hover_msg = Twist()
            hover_msg.linear.x = 0.0
            hover_msg.linear.y = 0.0
            hover_msg.linear.z = 0.0
            hover_msg.angular.z = 0.0
            
            self.control_pub.publish(hover_msg)
        else:
            # Still waiting
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TelloTakeoffHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping hover commands...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
