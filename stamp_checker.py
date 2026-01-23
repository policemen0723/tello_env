#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters

class StampChecker(Node):
    def __init__(self):
        super().__init__('stamp_checker')
        self.count = 0
        self.diffs = []
        
        self.sub_depth = message_filters.Subscriber(self, Image, '/drone1/depth/image_raw')
        self.sub_rgb = message_filters.Subscriber(self, Image, '/drone1/depth/rgb')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_depth, self.sub_rgb],
            queue_size=10,
            slop=1.0  # Increased slop to catch even large offsets
        )
        self.ts.registerCallback(self.callback)
        
        self.get_logger().info("StampChecker started. Will check 10 samples and exit.")

    def callback(self, depth_msg, rgb_msg):
        depth_time = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        rgb_time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        diff = depth_time - rgb_time
        self.diffs.append(diff)
        self.count += 1
        
        self.get_logger().info(f"Sample {self.count}/10 - Depth: {depth_time:.6f}, RGB: {rgb_time:.6f}, Diff: {diff:.6f} sec")

        if self.count >= 10:
            avg_diff = sum(self.diffs) / len(self.diffs)
            self.get_logger().info(f"--- Finished --- Average Diff: {avg_diff:.6f} sec")
            # Force shutdown
            import sys
            import os
            import signal
            os.kill(os.getpid(), signal.SIGINT)

def main(args=None):
    rclpy.init(args=args)
    checker = StampChecker()
    rclpy.spin(checker)
    checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
