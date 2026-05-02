#!/usr/bin/env python3
from collections import deque
from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
import numpy as np
import rclpy
from rclpy.node import Node

from .waypoint_loader import load_waypoints


class LeePlanner(Node):
    def __init__(self):
        super().__init__("lee_planner")

        package_share = Path(get_package_share_directory("restaurant_mapping"))
        default_waypoints = str(package_share / "config" / "waypoints.yaml")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("planning_grid_topic", "/planning_grid")
        self.declare_parameter("waypoints_yaml", default_waypoints)
        self.declare_parameter("start_waypoint", "recycle_bin")
        self.declare_parameter("goal_waypoint", "alice_corner")
        self.declare_parameter("block_size", 7)
        self.declare_parameter("occupied_fraction_threshold", 0.15)
        self.declare_parameter("inflation_radius", 0.15)
        self.declare_parameter("treat_unknown_as_occupied", False)

        self.map_topic = self.get_parameter("map_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.planning_grid_topic = self.get_parameter("planning_grid_topic").value
        self.waypoints_yaml = self.get_parameter("waypoints_yaml").value
        self.start_waypoint = self.get_parameter("start_waypoint").value
        self.goal_waypoint = self.get_parameter("goal_waypoint").value
        self.block_size = int(self.get_parameter("block_size").value)
        self.occupied_fraction_threshold = float(
            self.get_parameter("occupied_fraction_threshold").value
        )
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.treat_unknown_as_occupied = bool(
            self.get_parameter("treat_unknown_as_occupied").value
        )

        self.waypoints = load_waypoints(self.waypoints_yaml)
        self.path_pub = self.create_publisher(PathMsg, self.path_topic, 10)
        self.planning_grid_pub = self.create_publisher(
            OccupancyGrid, self.planning_grid_topic, 1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 1
        )

        self.get_logger().info(
            "LeePlanner: %s -> %s, block_size=%d (~0.35m with 0.05m maps)"
            % (self.start_waypoint, self.goal_waypoint, self.block_size)
        )

    def map_callback(self, msg):
        if self.start_waypoint not in self.waypoints or self.goal_waypoint not in self.waypoints:
            names = ", ".join(sorted(self.waypoints))
            self.get_logger().error(f"Unknown waypoint. Available: {names}")
            return

        fine_grid = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        inflated = self.inflate_obstacles(fine_grid, msg.info.resolution)
        coarse_grid = self.coarsen_grid(inflated)

        start = self.world_to_coarse_cell(
            self.waypoints[self.start_waypoint].x,
            self.waypoints[self.start_waypoint].y,
            msg,
        )
        goal = self.world_to_coarse_cell(
            self.waypoints[self.goal_waypoint].x,
            self.waypoints[self.goal_waypoint].y,
            msg,
        )

        start = self.snap_to_free(start, coarse_grid)
        goal = self.snap_to_free(goal, coarse_grid)
        path = self.lee_search(coarse_grid, start, goal)
        if not path:
            self.get_logger().warn(
                "No Lee path found from %s to %s"
                % (self.start_waypoint, self.goal_waypoint),
                throttle_duration_sec=2.0,
            )
            self.publish_planning_grid(coarse_grid, msg)
            return

        self.publish_planning_grid(coarse_grid, msg)
        self.publish_path(path, msg)

    def inflate_obstacles(self, fine_grid, resolution):
        radius_cells = int(math.ceil(self.inflation_radius / resolution))
        if radius_cells <= 0:
            return fine_grid.copy()

        blocked = fine_grid == 100
        inflated = fine_grid.copy()
        occupied_rows, occupied_cols = np.nonzero(blocked)

        for row, col in zip(occupied_rows, occupied_cols):
            row_min = max(0, row - radius_cells)
            row_max = min(fine_grid.shape[0], row + radius_cells + 1)
            col_min = max(0, col - radius_cells)
            col_max = min(fine_grid.shape[1], col + radius_cells + 1)

            for nbr_row in range(row_min, row_max):
                row_offset = nbr_row - row
                remaining = radius_cells * radius_cells - row_offset * row_offset
                if remaining < 0:
                    continue
                col_span = int(math.floor(math.sqrt(remaining)))
                left = max(col_min, col - col_span)
                right = min(col_max, col + col_span + 1)
                inflated[nbr_row, left:right] = 100

        return inflated

    def coarsen_grid(self, fine_grid):
        height, width = fine_grid.shape
        coarse_height = int(math.ceil(height / self.block_size))
        coarse_width = int(math.ceil(width / self.block_size))
        coarse = np.zeros((coarse_height, coarse_width), dtype=np.int8)

        for coarse_row in range(coarse_height):
            row_start = coarse_row * self.block_size
            row_end = min(height, row_start + self.block_size)
            for coarse_col in range(coarse_width):
                col_start = coarse_col * self.block_size
                col_end = min(width, col_start + self.block_size)
                block = fine_grid[row_start:row_end, col_start:col_end]

                occupied_fraction = np.count_nonzero(block == 100) / block.size
                unknown_fraction = np.count_nonzero(block == -1) / block.size
                blocked = occupied_fraction >= self.occupied_fraction_threshold
                if self.treat_unknown_as_occupied:
                    blocked = blocked or unknown_fraction >= self.occupied_fraction_threshold

                coarse[coarse_row, coarse_col] = 100 if blocked else 0

        return coarse

    def world_to_coarse_cell(self, x, y, map_msg):
        fine_col = int((x - map_msg.info.origin.position.x) / map_msg.info.resolution)
        fine_row = int((y - map_msg.info.origin.position.y) / map_msg.info.resolution)
        return fine_row // self.block_size, fine_col // self.block_size

    def coarse_cell_to_world(self, row, col, map_msg):
        fine_col = col * self.block_size + self.block_size / 2.0
        fine_row = row * self.block_size + self.block_size / 2.0
        x = map_msg.info.origin.position.x + fine_col * map_msg.info.resolution
        y = map_msg.info.origin.position.y + fine_row * map_msg.info.resolution
        return x, y

    def snap_to_free(self, cell, coarse_grid):
        if self.is_free(cell[0], cell[1], coarse_grid):
            return cell

        visited = set()
        queue = deque([cell])
        while queue:
            row, col = queue.popleft()
            if (row, col) in visited:
                continue
            visited.add((row, col))

            if self.is_free(row, col, coarse_grid):
                return row, col

            for nbr in self.neighbors(row, col):
                nbr_row, nbr_col = nbr
                if self.in_bounds(nbr_row, nbr_col, coarse_grid) and nbr not in visited:
                    queue.append(nbr)

        raise RuntimeError("No free planning cell found near waypoint")

    def lee_search(self, coarse_grid, start, goal):
        visited = {start}
        parent = {}
        queue = deque([start])

        while queue:
            current = queue.popleft()
            if current == goal:
                return self.reconstruct_path(parent, goal)

            for nbr in self.neighbors(current[0], current[1]):
                row, col = nbr
                if nbr in visited or not self.is_free(row, col, coarse_grid):
                    continue
                visited.add(nbr)
                parent[nbr] = current
                queue.append(nbr)

        return []

    @staticmethod
    def neighbors(row, col):
        return [
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        ]

    @staticmethod
    def is_free(row, col, grid):
        return LeePlanner.in_bounds(row, col, grid) and grid[row, col] == 0

    @staticmethod
    def in_bounds(row, col, grid):
        return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]

    @staticmethod
    def reconstruct_path(parent, goal):
        path = [goal]
        while path[-1] in parent:
            path.append(parent[path[-1]])
        path.reverse()
        return path

    def publish_path(self, path, map_msg):
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_msg.header.frame_id

        for row, col in path:
            x, y = self.coarse_cell_to_world(row, col, map_msg)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info(
            "Published Lee path with %d coarse cells" % len(path),
            throttle_duration_sec=2.0,
        )

    def publish_planning_grid(self, coarse_grid, map_msg):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_msg.header.frame_id
        msg.info.resolution = map_msg.info.resolution * self.block_size
        msg.info.width = coarse_grid.shape[1]
        msg.info.height = coarse_grid.shape[0]
        msg.info.origin = map_msg.info.origin
        msg.data = coarse_grid.flatten().tolist()
        self.planning_grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
