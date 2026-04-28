#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.node import Node

from .planner import astar, build_planner, path_to_world, simplify_path, snap_to_nearest_free, world_to_coarse_cell


class PlannedPathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")

        package_share = Path(get_package_share_directory("slam"))
        default_map_yaml = str(package_share / "maps" / "restaurant_map.yaml")
        default_waypoints_yaml = str(package_share / "config" / "waypoints.yaml")

        self.declare_parameter("map_yaml", default_map_yaml)
        self.declare_parameter("waypoints_yaml", default_waypoints_yaml)
        self.declare_parameter("start_waypoint", "recycle_bin")
        self.declare_parameter("goal_waypoint", "alice_corner")
        self.declare_parameter("block_size", 4)
        self.declare_parameter("inflation_radius", 0.15)
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period", 1.0)

        self.map_yaml = self.get_parameter("map_yaml").value
        self.waypoints_yaml = self.get_parameter("waypoints_yaml").value
        self.start_name = self.get_parameter("start_waypoint").value
        self.goal_name = self.get_parameter("goal_waypoint").value
        self.block_size = int(self.get_parameter("block_size").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.path_topic = self.get_parameter("path_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.path_pub = self.create_publisher(PathMsg, self.path_topic, 10)
        self.path_msg = self._build_path_message()
        self.create_timer(float(self.get_parameter("publish_period").value), self.publish_path)

        self.get_logger().info(
            "PathPublisher: %s -> %s on topic '%s'"
            % (self.start_name, self.goal_name, self.path_topic)
        )

    def _build_path_message(self):
        grid_map, coarse_map, waypoints = build_planner(
            self.map_yaml,
            self.waypoints_yaml,
            block_size=self.block_size,
            inflation_radius_m=self.inflation_radius,
        )

        if self.start_name not in waypoints or self.goal_name not in waypoints:
            available = ", ".join(sorted(waypoints))
            raise ValueError(
                f"Unknown waypoint name. start='{self.start_name}', goal='{self.goal_name}', "
                f"available={available}"
            )

        start_wp = waypoints[self.start_name]
        goal_wp = waypoints[self.goal_name]

        start_cell = world_to_coarse_cell(start_wp.x, start_wp.y, grid_map, coarse_map.block_size)
        goal_cell = world_to_coarse_cell(goal_wp.x, goal_wp.y, grid_map, coarse_map.block_size)
        start_cell = snap_to_nearest_free(start_cell, coarse_map)
        goal_cell = snap_to_nearest_free(goal_cell, coarse_map)

        path_cells = astar(coarse_map, start_cell, goal_cell)
        if not path_cells:
            raise RuntimeError(
                f"No path found between waypoints '{self.start_name}' and '{self.goal_name}'"
            )

        simplified = simplify_path(path_cells)
        world_points = path_to_world(simplified, grid_map, coarse_map.block_size)

        path_msg = PathMsg()
        path_msg.header.frame_id = self.frame_id
        for x_coord, y_coord in world_points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x_coord
            pose.pose.position.y = y_coord
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.get_logger().info(
            "Built path with %d poses (%d raw cells)"
            % (len(path_msg.poses), len(path_cells))
        )
        return path_msg

    def publish_path(self):
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlannedPathPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
