#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class TelloAutonomousController(Node):
    def __init__(self):
        super().__init__('tello_autonomous_controller')
        
        # パブリッシャー (離陸・緊急停止用)
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        
        # Nav2 アクションクライアント (自律移動用)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/drone1/navigate_to_pose')
        
        # タイマー (状態管理用 10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.start_time = time.time()
        self.state = 'TAKEOFF' # 状態: TAKEOFF, SEND_GOAL, NAVIGATING, DONE
        self.get_logger().info('Tello Autonomous Controller Started. State: TAKEOFF')

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time
        msg = Twist()

        if self.state == 'TAKEOFF':
            if elapsed_time < 3.0:
                # 離陸フェーズ: 3秒間上昇
                msg.linear.z = 0.5
                self.cmd_vel_pub.publish(msg)
                if int(elapsed_time * 10) % 10 == 0:
                    self.get_logger().info(f'Taking off... ({elapsed_time:.1f}s)')
            else:
                # 離陸完了 -> ゴール送信へ
                self.state = 'SEND_GOAL'
                self.get_logger().info('Takeoff complete. Preparing to send Nav2 Goal...')

        elif self.state == 'SEND_GOAL':
            # Nav2に目標地点を送信
            self.send_nav2_goal(x=2.0, y=2.0)
            self.state = 'NAVIGATING'
            self.get_logger().info('Goal sent to Nav2! Waiting for Path Planning...')

        elif self.state == 'NAVIGATING':
            # Nav2が制御中。ここでは何もしない（監視のみ）
            pass

        elif self.state == 'DONE':
            # 到着後
            pass

    def send_nav2_goal(self, x, y):
        # Nav2がアクションサーバーを起動するのを待つ
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 目標座標の設定
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0 # 正面を向く
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}')
        
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 :(')
            return

        self.get_logger().info('Goal accepted by Nav2! Tracking progress...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.state = 'DONE'
        self.get_logger().info('Navigation complete! Tello reached the goal.')

def main(args=None):
    rclpy.init(args=args)
    node = TelloAutonomousController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時は安全のため停止
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
