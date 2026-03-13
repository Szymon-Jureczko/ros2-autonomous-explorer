#!/usr/bin/env python3
"""Frontier Explorer — Nav2 integration with blacklist for failed goals."""

import math
import time
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rosgraph_msgs.msg import Clock


FREE = 0
UNKNOWN = -1
LETHAL = 100

MIN_FRONTIER_SIZE = 3
REPLAN_INTERVAL = 4.0
BLACKLIST_RADIUS = 0.8
MAX_BLACKLIST_SIZE = 50


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_frontier_size', MIN_FRONTIER_SIZE)
        self.declare_parameter('replan_interval', REPLAN_INTERVAL)
        self.declare_parameter('blacklist_radius', BLACKLIST_RADIUS)
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.replan_interval = self.get_parameter('replan_interval').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value

        self.slam_map = None
        self.robot_pose = None
        self.blacklisted_goals = []
        self.navigating = False
        self.current_goal = None
        self.clock_ok = False
        self.last_clock_time = None

        # Verify sim-time before proceeding
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self._clock_cb, 10)

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

        # Wait for /clock first, then connect to Nav2
        self.get_logger().info('Waiting for /clock to confirm sim-time is active...')
        self._clock_check_timer = self.create_timer(2.0, self._check_clock_health)

    def _clock_cb(self, msg):
        self.last_clock_time = time.monotonic()
        if not self.clock_ok:
            self.clock_ok = True
            self.get_logger().info('/clock is being published — sim-time OK')

    def _check_clock_health(self):
        if not self.clock_ok:
            self.get_logger().warn('/clock not yet received — bridge may not be ready')
            return
        if self.last_clock_time and (time.monotonic() - self.last_clock_time) > 5.0:
            self.get_logger().error('/clock appears stale (>5s since last msg)')
            return
        self._clock_check_timer.cancel()
        self.get_logger().info('Clock health OK. Waiting for Nav2 action servers...')
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
            self.get_logger().warn('All frontiers blacklisted, clearing blacklist.')
            self.blacklisted_goals.clear()
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
            if self._is_blacklisted(cx, cy):
                continue
            score = len(frontier) / (dist + 0.1)
            if score > best_score:
                best_score = score
                best = (cx, cy)
        return best

    def _is_blacklisted(self, x, y):
        for bx, by in self.blacklisted_goals:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

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
        self.current_goal = goal_xy
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            self._blacklist_current_goal()
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
            self.get_logger().warn(f'Goal failed with status {status}, blacklisting.')
            self._blacklist_current_goal()
        self.navigating = False
        self.current_goal = None

    def _blacklist_current_goal(self):
        if self.current_goal:
            self.blacklisted_goals.append(self.current_goal)
            if len(self.blacklisted_goals) > MAX_BLACKLIST_SIZE:
                self.blacklisted_goals.pop(0)


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
