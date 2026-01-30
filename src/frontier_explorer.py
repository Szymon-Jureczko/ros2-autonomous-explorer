#!/usr/bin/env python3
"""Frontier Explorer — skeleton: subscribe to /map and look up robot pose via TF."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

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

        self.timer = self.create_timer(2.0, self._tick)

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
            self.get_logger().info('Waiting for SLAM map on /map...')
            return
        pos = self._get_robot_position()
        if pos is None:
            self.get_logger().warn('Cannot get robot position from TF, skipping.')
            return
        self.robot_pose = pos
        self.get_logger().info(
            f'Map: {self.slam_map.info.width}x{self.slam_map.info.height}, '
            f'robot at ({pos[0]:.2f}, {pos[1]:.2f})')


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
