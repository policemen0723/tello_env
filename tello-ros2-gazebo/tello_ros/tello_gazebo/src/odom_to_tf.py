#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        # Gazeboの真値オドメトリを購読
        self.subscription = self.create_subscription(
            Odometry,
            '/drone1/odom',
            self.handle_odom,
            10)
        self.get_logger().info("Odom to TF broadcaster started. Listening to /drone1/odom")

    def handle_odom(self, msg):
        t = TransformStamped()

        # タイムスタンプをコピー
        t.header.stamp = msg.header.stamp
        # フレームIDの設定（必要に応じて変更）
        # Gazeboのodomメッセージは frame_id: 'map', child_frame_id: 'base_link_1' になっているが、
        # TFツリーの標準的な構成として odom -> base_link_1 を繋ぐ役割にする。
        # RTAB-Mapが map -> odom を出す設定になっているため。
        t.header.frame_id = 'odom' 
        t.child_frame_id = msg.child_frame_id # 'base_link_1'

        # 位置
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # 回転
        t.transform.rotation = msg.pose.pose.orientation

        # TFを配信
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
