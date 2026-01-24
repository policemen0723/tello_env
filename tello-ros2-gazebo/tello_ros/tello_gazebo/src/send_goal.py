#!/usr/bin/env python3
"""
Simple Goal Sender - RViz2と同じ方法でゴールを送信

使い方:
  # 座標を指定してゴールを送信
  ros2 run tello_gazebo send_goal.py --ros-args -p x:=2.0 -p y:=3.0

  # 高度とyaw角も指定
  ros2 run tello_gazebo send_goal.py --ros-args -p x:=2.0 -p y:=3.0 -p z:=1.5 -p yaw:=90.0

  # namespaceを指定
  ros2 run tello_gazebo send_goal.py --ros-args -r __ns:=/drone1 -p x:=2.0 -p y:=3.0
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw_deg: float):
    """Yaw角（度）からクォータニオンを計算"""
    yaw = math.radians(yaw_deg)
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

        # パラメータ
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.0)  # デフォルト高度 1m
        self.declare_parameter('yaw', 0.0)  # 度数法
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', 'goal_pose')

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.topic = self.get_parameter('topic').value

        # Publisher
        self.pub = self.create_publisher(PoseStamped, self.topic, 10)

    def send_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        # 少し待ってからパブリッシュ（サブスクライバーの接続待ち）
        self.get_logger().info(f'Sending goal: x={self.x}, y={self.y}, z={self.z}, yaw={self.yaw}°')
        self.pub.publish(msg)
        self.get_logger().info('Goal sent!')


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    
    # 接続待ち
    import time
    time.sleep(0.5)
    
    node.send_goal()
    
    # 少し待ってから終了
    time.sleep(0.5)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
