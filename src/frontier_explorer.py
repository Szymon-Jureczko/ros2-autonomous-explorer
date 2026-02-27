#!/usr/bin/env python3
"""Frontier Explorer — Nav2 integration: send NavigateToPose goals."""

import math
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped


FREE = 0
UNKNOWN = -1
LETHAL = 100

MIN_FRONTIER_SIZE = 3
REPLAN_INTERVAL = 4.0


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_frontier_size', MIN_FRONTIER_SIZE)
        self.declare_parameter('replan_interval', REPLAN_INTERVAL)
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.replan_interval = self.get_parameter('replan_interval').value

        self.slam_map = None
        self.robot_pose = None
        self.navigating = False

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos
        )

        self.tf_buffer = None
        self.tf_listener = None
        self._setup_tf()

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')

        self.timer = self.create_timer(self.replan_interval, self._explore_tick)

    def _setup_tf(self):
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _get_robot_position(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _map_cb(self, msg: OccupancyGrid):
        self.slam_map = msg

    def _explore_tick(self):
        if self.slam_map is None:
            return
        pos = self._get_robot_position()
        if pos is None:
            return
        self.robot_pose = pos

        if self.navigating:
            self.get_logger().info('Navigation in progress, waiting...')
            return

        clusters = self._find_frontiers()
        if not clusters:
            self.get_logger().info('No frontiers detected')
            return

        goal = self._select_frontier(clusters)
        if goal is None:
            return
        self._send_goal(goal)

    def _find_frontiers(self):
        cm = self.slam_map
        w, h = cm.info.width, cm.info.height
        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        data = np.array(cm.data, dtype=np.int8).reshape((h, w))

        frontier_mask = np.zeros((h, w), dtype=bool)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(data, dy, axis=0), dx, axis=1)
            frontier_mask |= (data == FREE) & (shifted == UNKNOWN)

        rows, cols = np.where(frontier_mask)
        if len(rows) == 0:
            return []

        visited = np.zeros((h, w), dtype=bool)
        clusters = []

        for idx in range(len(rows)):
            r0, c0 = int(rows[idx]), int(cols[idx])
            if visited[r0, c0]:
                continue
            cluster = []
            queue = deque([(r0, c0)])
            visited[r0, c0] = True
            while queue:
                r, c = queue.popleft()
                wx = ox + (c + 0.5) * res
                wy = oy + (r + 0.5) * res
                cluster.append((wx, wy))
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc] and frontier_mask[nr, nc]:
                            visited[nr, nc] = True
                            queue.append((nr, nc))
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        return clusters

    def _select_frontier(self, frontiers):
        rx, ry = self.robot_pose
        best = None
        best_score = -1.0
        for frontier in frontiers:
            cx = sum(p[0] for p in frontier) / len(frontier)
            cy = sum(p[1] for p in frontier) / len(frontier)
            dist = math.hypot(cx - rx, cy - ry)
            score = len(frontier) / (dist + 0.1)
            if score > best_score:
                best_score = score
                best = (cx, cy)
        return best

    def _send_goal(self, goal_xy):
        x, y = goal_xy
        rx, ry = self.robot_pose
        yaw = math.atan2(y - ry, x - rx)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        self.get_logger().info(f'Navigating to frontier at ({x:.2f}, {y:.2f})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.navigating = True
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            self.navigating = False
            return
        self.get_logger().info('Goal accepted by Nav2')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().warn(f'Goal finished with status {status}')
        self.navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
