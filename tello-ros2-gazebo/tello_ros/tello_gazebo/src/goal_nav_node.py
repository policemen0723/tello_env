#!/usr/bin/env python3

import math
import heapq
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_nav_node')

        self.declare_parameter('use_sim_time', False)
        
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('grid_topic', 'rtabmap/grid_map')
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('grid_frame_id', 'map')
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('inflation_radius', 0.2)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_vel_xy', 0.5)
        self.declare_parameter('max_vel_z', 0.3)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('kp_xy', 0.8)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('goal_tolerance_xy', 0.4)
        self.declare_parameter('goal_tolerance_z', 0.3)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.grid_topic = self.get_parameter('grid_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.grid_frame_id = self.get_parameter('grid_frame_id').value
        self.obstacle_threshold = int(self.get_parameter('obstacle_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.max_vel_xy = float(self.get_parameter('max_vel_xy').value)
        self.max_vel_z = float(self.get_parameter('max_vel_z').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.goal_tolerance_xy = float(self.get_parameter('goal_tolerance_xy').value)
        self.goal_tolerance_z = float(self.get_parameter('goal_tolerance_z').value)

        self.goal: Optional[PoseStamped] = None
        self.odom: Optional[Odometry] = None
        self.grid: Optional[OccupancyGrid] = None
        self.path_world: List[Tuple[float, float]] = []
        self.path_index = 0
        self.needs_replan = False
        self.warned_frame_mismatch = False

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.grid_topic, self.grid_callback, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.get_logger().info(
            f"Goal navigator ready. goal: {self.goal_topic}, grid: {self.grid_topic}, odom: {self.odom_topic}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def grid_callback(self, msg: OccupancyGrid) -> None:
        self.grid = msg
        if self.goal is not None:
            self.needs_replan = True

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal = msg
        self.path_world = []
        self.path_index = 0
        self.needs_replan = True
        self.get_logger().info(
            f"Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})"
        )

    def control_loop(self) -> None:
        if self.goal is None or self.odom is None:
            return

        if self.grid is None:
            self.publish_stop()
            return

        if self.grid.header.frame_id and self.grid.header.frame_id != self.grid_frame_id:
            if not self.warned_frame_mismatch:
                self.get_logger().warn(
                    f"Grid frame_id '{self.grid.header.frame_id}' != expected '{self.grid_frame_id}'."
                )
                self.warned_frame_mismatch = True

        if self.needs_replan:
            self.plan_path()
            self.needs_replan = False
            if not self.path_world:
                self.get_logger().warn("No valid path found. Holding position.")
                self.publish_stop()
                return

        if self.path_index >= len(self.path_world):
            self.publish_stop()
            self.goal = None
            self.path_world = []
            return

        current = self.odom.pose.pose.position
        current_xy = (current.x, current.y)
        target_xy = self.path_world[self.path_index]

        if self.distance_xy(current_xy, target_xy) < self.waypoint_tolerance:
            self.path_index += 1
            if self.path_index >= len(self.path_world):
                self.publish_stop()
                self.goal = None
                self.path_world = []
                return
            target_xy = self.path_world[self.path_index]

        goal_z = self.goal.pose.position.z
        self.publish_cmd(current_xy, current.z, target_xy, goal_z)

    def publish_stop(self) -> None:
        msg = Twist()
        self.cmd_pub.publish(msg)

    def publish_cmd(
        self,
        current_xy: Tuple[float, float],
        current_z: float,
        target_xy: Tuple[float, float],
        goal_z: float,
    ) -> None:
        dx = target_xy[0] - current_xy[0]
        dy = target_xy[1] - current_xy[1]

        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(desired_yaw - yaw)

        vx_map = max(-self.max_vel_xy, min(self.max_vel_xy, self.kp_xy * dx))
        vy_map = max(-self.max_vel_xy, min(self.max_vel_xy, self.kp_xy * dy))

        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        vx_body = cos_yaw * vx_map - sin_yaw * vy_map
        vy_body = sin_yaw * vx_map + cos_yaw * vy_map

        vz = max(-self.max_vel_z, min(self.max_vel_z, self.kp_z * (goal_z - current_z)))
        wz = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.kp_yaw * yaw_error))

        if self.goal_reached(current_xy, current_z, goal_z):
            self.publish_stop()
            self.goal = None
            self.path_world = []
            return

        msg = Twist()
        msg.linear.x = vx_body
        msg.linear.y = vy_body
        msg.linear.z = vz
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    def goal_reached(self, current_xy: Tuple[float, float], current_z: float, goal_z: float) -> bool:
        if self.goal is None:
            return True
        if self.distance_xy(current_xy, (self.goal.pose.position.x, self.goal.pose.position.y)) > self.goal_tolerance_xy:
            return False
        if abs(goal_z - current_z) > self.goal_tolerance_z:
            return False
        return True

    def plan_path(self) -> None:
        if self.grid is None or self.goal is None or self.odom is None:
            return

        origin = self.grid.info.origin.position
        resolution = self.grid.info.resolution
        width = self.grid.info.width
        height = self.grid.info.height

        start = self.world_to_grid(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, origin, resolution)
        goal = self.world_to_grid(self.goal.pose.position.x, self.goal.pose.position.y, origin, resolution)

        if start is None or goal is None:
            self.path_world = []
            return

        self.path_world = self.a_star(start, goal, width, height, origin, resolution)
        self.path_index = 0

    def a_star(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        width: int,
        height: int,
        origin,
        resolution: float,
    ) -> List[Tuple[float, float]]:
        open_set = []
        heapq.heappush(open_set, (0.0, 0, start))
        came_from = {}
        g_score = {start: 0.0}
        count = 0

        while open_set:
            _, _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current, origin, resolution)

            for neighbor, step_cost in self.neighbors(current, width, height):
                if self.is_occupied(neighbor, width, height):
                    continue
                tentative = g_score[current] + step_cost
                if tentative < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    count += 1
                    f_score = tentative + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, count, neighbor))

        return []

    def neighbors(self, cell: Tuple[int, int], width: int, height: int) -> List[Tuple[Tuple[int, int], float]]:
        x, y = cell
        options = [
            (x + 1, y, 1.0),
            (x - 1, y, 1.0),
            (x, y + 1, 1.0),
            (x, y - 1, 1.0),
            (x + 1, y + 1, math.sqrt(2.0)),
            (x - 1, y + 1, math.sqrt(2.0)),
            (x + 1, y - 1, math.sqrt(2.0)),
            (x - 1, y - 1, math.sqrt(2.0)),
        ]
        neighbors = []
        for nx, ny, cost in options:
            if 0 <= nx < width and 0 <= ny < height:
                neighbors.append(((nx, ny), cost))
        return neighbors

    def is_occupied(self, cell: Tuple[int, int], width: int, height: int) -> bool:
        if self.grid is None:
            return True
        x, y = cell
        data = self.grid.data
        index = y * width + x
        val = data[index]
        if val < 0:
            if self.unknown_is_obstacle:
                return True
        if val >= self.obstacle_threshold:
            return True

        inflation_cells = int(self.inflation_radius / max(self.grid.info.resolution, 1e-6))
        if inflation_cells <= 0:
            return False

        for ix in range(max(0, x - inflation_cells), min(width, x + inflation_cells + 1)):
            for iy in range(max(0, y - inflation_cells), min(height, y + inflation_cells + 1)):
                idx = iy * width + ix
                if data[idx] >= self.obstacle_threshold:
                    return True
        return False

    def world_to_grid(
        self, x: float, y: float, origin, resolution: float
    ) -> Optional[Tuple[int, int]]:
        gx = int((x - origin.x) / resolution)
        gy = int((y - origin.y) / resolution)
        if gx < 0 or gy < 0 or gx >= self.grid.info.width or gy >= self.grid.info.height:
            return None
        return gx, gy

    def reconstruct_path(
        self,
        came_from: dict,
        current: Tuple[int, int],
        origin,
        resolution: float,
    ) -> List[Tuple[float, float]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return [self.grid_to_world(cell[0], cell[1], origin, resolution) for cell in path]

    def grid_to_world(self, x: int, y: int, origin, resolution: float) -> Tuple[float, float]:
        wx = origin.x + (x + 0.5) * resolution
        wy = origin.y + (y + 0.5) * resolution
        return wx, wy

    @staticmethod
    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def distance_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
