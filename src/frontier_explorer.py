#!/usr/bin/env python3
"""
Frontier Explorer — autonomous exploration via Nav2 + SLAM.

Detects frontiers (free/unknown boundaries) in the SLAM map, picks
the best one (large & close), and sends it as a Nav2 goal. Repeats
until no frontiers remain. Does an initial 360° spin to seed the map.
"""

import math
import time
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, Spin
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rosgraph_msgs.msg import Clock


# OccupancyGrid cell values
FREE = 0
UNKNOWN = -1
LETHAL = 100

# Defaults (overridable via ROS params)
MIN_FRONTIER_SIZE = 3
GOAL_TOLERANCE = 0.5
REPLAN_INTERVAL = 4.0
BLACKLIST_RADIUS = 0.8
MAX_BLACKLIST_SIZE = 50


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # --- Parameters ---
        self.declare_parameter('min_frontier_size', MIN_FRONTIER_SIZE)
        self.declare_parameter('replan_interval', REPLAN_INTERVAL)
        self.declare_parameter('blacklist_radius', BLACKLIST_RADIUS)

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.replan_interval = self.get_parameter('replan_interval').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value

        # State
        self.slam_map = None
        self.robot_pose = None
        self.blacklisted_goals = []
        self.navigating = False
        self.spinning = False
        self.current_goal = None
        self.current_goal_handle = None
        self.goals_sent = 0
        self.ticks_without_frontier = 0
        self.initial_spin_done = False
        self.clock_ok = False
        self.last_clock_time = None
        self.nav_start_time = None

        # Verify sim-time before proceeding
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self._clock_cb, 10)

        # SLAM map subscription (transient local for late joiners)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_cb,
            map_qos
        )

        # TF for robot position
        self.tf_buffer = None
        self.tf_listener = None
        self._setup_tf()

        # Nav2 action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')

        # Wait for /clock first, then connect to Nav2
        self.get_logger().info('Waiting for /clock to confirm sim-time is active...')
        self._clock_check_timer = self.create_timer(2.0, self._check_clock_health)

    def _clock_cb(self, msg):
        """Track /clock to verify sim-time is flowing."""
        self.last_clock_time = time.monotonic()
        if not self.clock_ok:
            self.clock_ok = True
            self.get_logger().info('/clock is being published — sim-time OK')

    def _check_clock_health(self):
        """Wait for /clock before connecting to Nav2."""
        if not self.clock_ok:
            self.get_logger().warn('/clock not yet received — bridge may not be ready')
            return
        # Check for stale clock (>5s since last msg)
        if self.last_clock_time and (time.monotonic() - self.last_clock_time) > 5.0:
            self.get_logger().error('/clock appears stale (>5s since last msg)')
            return

        # Clock is healthy — stop this timer and proceed to Nav2 setup
        self._clock_check_timer.cancel()
        self.get_logger().info('Clock health OK. Waiting for Nav2 action servers...')
        self.nav_client.wait_for_server()
        self.spin_client.wait_for_server()
        self.get_logger().info('Nav2 action servers connected!')

        # Wait for Nav2 lifecycle nodes to become active before sending goals
        self._spin_retries = 0
        self.get_logger().info('Waiting for Nav2 lifecycle activation...')
        self.create_timer(3.0, self._try_initial_spin)

    def _try_initial_spin(self):
        """Attempt the initial spin; retry if behavior_server is still inactive."""
        if self.initial_spin_done:
            return  # Already done
        self._spin_retries += 1
        self.get_logger().info(
            f'Attempting initial 360° spin (attempt {self._spin_retries})...')
        self._do_nav2_spin(6.28, self._initial_spin_done_cb)

    def _do_nav2_spin(self, angle_rad, done_callback):
        """Send a Spin goal to Nav2's behavior server."""
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = angle_rad
        self.spinning = True
        future = self.spin_client.send_goal_async(spin_goal)
        future.add_done_callback(
            lambda f: self._spin_accepted_cb(f, done_callback))

    def _spin_accepted_cb(self, future, done_callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.spinning = False
            if done_callback == self._initial_spin_done_cb:
                self.get_logger().warn(
                    'Spin goal rejected (behavior_server not active yet), '
                    'will retry in 3s...')
                return  # The retry timer will try again
            else:
                self.get_logger().warn('Recovery spin rejected.')
                return
        self.get_logger().info('Spin goal accepted, rotating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._spin_result_cb(f, done_callback))

    def _spin_result_cb(self, future, done_callback):
        self.spinning = False
        self.get_logger().info('Spin complete!')
        done_callback()

    def _initial_spin_done_cb(self):
        """Called after the initial 360° spin finishes."""
        self.initial_spin_done = True
        self.get_logger().info('Initial spin done — starting frontier exploration timer.')
        self.timer = self.create_timer(self.replan_interval, self._explore_tick)

    def _setup_tf(self):
        """Set up TF2 listener to get robot position in map frame."""
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _get_robot_position(self):
        """Get robot (x, y) in the map frame via TF."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _map_cb(self, msg: OccupancyGrid):
        """Cache the latest SLAM map."""
        self.slam_map = msg

    # ------------------------------------------------------------------
    #  MAIN LOOP
    # ------------------------------------------------------------------
    def _explore_tick(self):
        """Find the best frontier and navigate to it."""
        # Skip if we don't have a SLAM map yet
        if self.slam_map is None:
            self.get_logger().info('Waiting for SLAM map on /map...')
            return

        # Get current robot position
        pos = self._get_robot_position()
        if pos is None:
            self.get_logger().warn('Cannot get robot position from TF, skipping.')
            return
        self.robot_pose = pos

        # If spinning, don't interrupt
        if self.spinning:
            self.get_logger().info('Spin in progress, waiting...')
            return

        # If navigating, check for timeout (robot might be stuck)
        if self.navigating:
            elapsed = time.monotonic() - self.nav_start_time if self.nav_start_time else 0
            if elapsed > 60.0:
                self.get_logger().warn(
                    f'Navigation goal timed out after {elapsed:.0f}s — cancelling & blacklisting.')
                self._cancel_current_goal()
                return
            self.get_logger().info(
                f'Navigation in progress ({elapsed:.0f}s elapsed), waiting...')
            return

        # Detect frontiers from SLAM map
        frontiers = self._find_frontiers()
        if not frontiers:
            self.ticks_without_frontier += 1
            if self.goals_sent > 2 and self.ticks_without_frontier >= 5:
                self.get_logger().info('=== NO FRONTIERS REMAINING — EXPLORATION COMPLETE! ===')
                self.timer.cancel()
                return
            # Do a small rotation to reveal more map
            self.get_logger().info(
                f'No frontiers found (tick {self.ticks_without_frontier}, '
                f'goals_sent={self.goals_sent}). Spinning to reveal more...')
            self._do_nav2_spin(3.14, lambda: None)  # 180° recovery spin
            return

        # Reset the no-frontier counter since we found frontiers
        self.ticks_without_frontier = 0
        self.get_logger().info(f'Found {len(frontiers)} frontier(s)')

        # Pick the best frontier
        goal = self._select_frontier(frontiers)
        if goal is None:
            self.get_logger().warn('All frontiers blacklisted, clearing blacklist.')
            self.blacklisted_goals.clear()
            return

        # Send Nav2 goal
        self._send_goal(goal)

    # ------------------------------------------------------------------
    #  FRONTIER DETECTION & SELECTION
    # ------------------------------------------------------------------
    def _find_frontiers(self):
        """Find frontier cells (FREE adjacent to UNKNOWN) and cluster them via BFS."""
        cm = self.slam_map
        w, h = cm.info.width, cm.info.height
        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        data = np.array(cm.data, dtype=np.int8).reshape((h, w))

        n_free = int(np.sum(data == FREE))
        n_unknown = int(np.sum(data == UNKNOWN))
        self.get_logger().info(
            f'Map: {w}x{h}, free={n_free}, unknown={n_unknown}, '
            f'origin=({ox:.1f},{oy:.1f}), res={res}')

        # Find all frontier cells: FREE cell with at least one UNKNOWN 4-neighbor
        frontier_mask = np.zeros((h, w), dtype=bool)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(data, dy, axis=0), dx, axis=1)
            frontier_mask |= (data == FREE) & (shifted == UNKNOWN)

        frontier_rows, frontier_cols = np.where(frontier_mask)
        if len(frontier_rows) == 0:
            return []

        self.get_logger().info(f'Frontier cells: {len(frontier_rows)}')

        # Cluster frontier cells using BFS (connected components)
        visited = np.zeros((h, w), dtype=bool)
        clusters = []

        for idx in range(len(frontier_rows)):
            r0, c0 = int(frontier_rows[idx]), int(frontier_cols[idx])
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
        """Score frontiers by size/distance ratio. Returns best (x, y) or None."""
        MIN_GOAL_DIST = 0.3   # metres — skip centroids closer than this
        rx, ry = self.robot_pose
        best = None
        best_score = -1.0

        for frontier in frontiers:
            # Centroid of this frontier cluster
            cx = sum(p[0] for p in frontier) / len(frontier)
            cy = sum(p[1] for p in frontier) / len(frontier)
            dist = math.hypot(cx - rx, cy - ry)

            if dist < MIN_GOAL_DIST:
                # Centroid is under the robot — pick the farthest point instead
                fx, fy = max(frontier, key=lambda p: math.hypot(p[0] - rx, p[1] - ry))
                dist = math.hypot(fx - rx, fy - ry)
                if dist < MIN_GOAL_DIST:
                    continue
                cx, cy = fx, fy

            if self._is_blacklisted(cx, cy):
                continue

            # Score: prefer larger frontiers that aren't too far away
            score = len(frontier) / (dist + 0.1)
            if score > best_score:
                best_score = score
                best = (cx, cy)

        return best

    def _is_blacklisted(self, x, y):
        """Check if (x, y) is near any blacklisted goal."""
        for bx, by in self.blacklisted_goals:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    # ------------------------------------------------------------------
    #  NAV2 GOALS
    # ------------------------------------------------------------------
    def _send_goal(self, goal_xy):
        """Send a NavigateToPose goal oriented toward the frontier."""
        x, y = goal_xy
        rx, ry = self.robot_pose
        # Orient the robot to face FROM its current position TOWARD the goal
        yaw = math.atan2(y - ry, x - rx)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        dist = math.hypot(x - rx, y - ry)
        self.get_logger().info(
            f'Navigating to frontier at ({x:.2f}, {y:.2f}), '
            f'dist={dist:.1f}m, yaw={math.degrees(yaw):.0f}°')

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
        self.current_goal_handle = None
        self.nav_start_time = time.monotonic()
        self.goals_sent += 1

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _cancel_current_goal(self):
        """Cancel the current Nav2 goal and blacklist it."""
        if self.current_goal_handle is not None:
            self.get_logger().info('Cancelling current Nav2 goal...')
            self.current_goal_handle.cancel_goal_async()
        self._blacklist_current_goal()
        self.navigating = False
        self.current_goal = None
        self.current_goal_handle = None
        self.nav_start_time = None

    def _goal_response_cb(self, future):
        """Called when Nav2 accepts/rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            self._blacklist_current_goal()
            self.navigating = False
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        """Called when Nav2 finishes (success or failure)."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal was cancelled.')
        else:
            self.get_logger().warn(f'Goal failed with status {status}, blacklisting.')
            self._blacklist_current_goal()

        self.navigating = False
        self.current_goal = None
        self.current_goal_handle = None
        self.nav_start_time = None

    def _blacklist_current_goal(self):
        """Add the current goal to the blacklist."""
        if self.current_goal:
            self.blacklisted_goals.append(self.current_goal)
            # Cap the blacklist size
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
