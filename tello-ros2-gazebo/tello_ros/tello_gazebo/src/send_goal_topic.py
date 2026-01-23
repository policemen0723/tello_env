#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

class GoalTopicPublisher(Node):
    def __init__(self):
        super().__init__('goal_topic_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/drone1/goal_pose', 10)
        # Wait a bit for the publisher to connect
        self.timer = self.create_timer(1.0, self.publish_goal)
        self.goal_sent = False

    def publish_goal(self):
        if self.goal_sent:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set position from arguments or default
        try:
            msg.pose.position.x = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
            msg.pose.position.y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
            msg.pose.position.z = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
        except ValueError:
            self.get_logger().error("Invalid arguments. Use: ros2 run tello_gazebo send_goal_topic.py <x> <y> <z>")
            rclpy.shutdown()
            return

        msg.pose.orientation.w = 1.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published goal to /drone1/goal_pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')
        self.goal_sent = True
        
        # Give it a moment to actually send before shutting down
        self.create_timer(1.0, rclpy.shutdown)

def main(args=None):
    rclpy.init(args=args)
    node = GoalTopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
