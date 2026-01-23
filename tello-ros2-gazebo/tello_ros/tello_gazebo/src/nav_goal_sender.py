#!/usr/bin/env python3

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('action_name', 'navigate_to_pose')

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.action_name = self.get_parameter('action_name').value

        self.client = ActionClient(self, NavigateToPose, self.action_name)

    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server '{self.action_name}' not available.")
            return False

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        qx, qy, qz, qw = yaw_to_quaternion(self.goal_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={self.goal_yaw:.2f} ({self.frame_id})"
        )
        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return False
        self.get_logger().info("Goal accepted.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSender()
    ok = node.send_goal()
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
