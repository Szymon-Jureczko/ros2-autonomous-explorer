#!/usr/bin/env python3
"""Frontier Explorer — detect + cluster frontier cells via BFS."""

from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid


FREE = 0
UNKNOWN = -1
LETHAL = 100

MIN_FRONTIER_SIZE = 3


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_frontier_size', MIN_FRONTIER_SIZE)
        self.min_frontier_size = self.get_parameter('min_frontier_size').value

        self.slam_map = None
        self.robot_pose = None

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

        self.timer = self.create_timer(3.0, self._tick)

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

    def _tick(self):
        if self.slam_map is None:
            return
        pos = self._get_robot_position()
        if pos is None:
            return
        self.robot_pose = pos

        clusters = self._find_frontiers()
        self.get_logger().info(f'Found {len(clusters)} frontier cluster(s)')

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
