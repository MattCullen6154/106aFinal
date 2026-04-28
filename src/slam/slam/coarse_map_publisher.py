#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node

from .planner import build_planner


class CoarseMapPublisher(Node):
    def __init__(self):
        super().__init__("coarse_map_publisher")

        package_share = Path(get_package_share_directory("slam"))
        default_map_yaml = str(package_share / "maps" / "restaurant_map.yaml")
        default_waypoints_yaml = str(package_share / "config" / "waypoints.yaml")

        self.declare_parameter("map_yaml", default_map_yaml)
        self.declare_parameter("waypoints_yaml", default_waypoints_yaml)
        self.declare_parameter("block_size", 4)
        self.declare_parameter("inflation_radius", 0.15)
        self.declare_parameter("map_topic", "/coarse_map")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period", 1.0)

        self.map_yaml = self.get_parameter("map_yaml").value
        self.waypoints_yaml = self.get_parameter("waypoints_yaml").value
        self.block_size = int(self.get_parameter("block_size").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.map_topic = self.get_parameter("map_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.map_msg = self._build_map_message()
        self.create_timer(float(self.get_parameter("publish_period").value), self.publish_map)

        self.get_logger().info(
            "CoarseMapPublisher: topic='%s', size=%dx%d, resolution=%.3fm"
            % (
                self.map_topic,
                self.map_msg.info.width,
                self.map_msg.info.height,
                self.map_msg.info.resolution,
            )
        )

    def _build_map_message(self):
        grid_map, coarse_map, _ = build_planner(
            self.map_yaml,
            self.waypoints_yaml,
            block_size=self.block_size,
            inflation_radius_m=self.inflation_radius,
        )

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = coarse_map.resolution
        msg.info.width = coarse_map.width
        msg.info.height = coarse_map.height
        msg.info.origin.position.x = grid_map.meta.origin_x
        msg.info.origin.position.y = grid_map.meta.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = [0 if cell == 0 else 100 for cell in coarse_map.grid.flatten().tolist()]
        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CoarseMapPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
