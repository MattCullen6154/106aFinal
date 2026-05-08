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
from std_msgs.msg import String, Bool
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from .waypoint_loader import load_waypoints


class LeePlanner(Node):
    def __init__(self):
        super().__init__("lee_planner")

        package_share = Path(get_package_share_directory("restaurant_mapping"))
        default_waypoints = str(package_share / "config" / "waypoints.yaml")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("planning_grid_topic", "/planning_grid")
        self.declare_parameter("waypoint_marker_topic", "/waypoint_markers")
        self.declare_parameter("goal_waypoint_topic", "/nav_goal_waypoint")
        self.declare_parameter("waypoints_yaml", default_waypoints)
        self.declare_parameter("start_mode", "waypoint")
        self.declare_parameter("start_waypoint", "table")
        self.declare_parameter("goal_waypoint", "kitchen")
        self.declare_parameter("plan_on_start", False)
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("block_size", 7)
        self.declare_parameter("occupied_fraction_threshold", 0.1)
        self.declare_parameter("inflation_radius", 0.05)
        self.declare_parameter("treat_unknown_as_occupied", False)

        self.map_topic = self.get_parameter("map_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.planning_grid_topic = self.get_parameter("planning_grid_topic").value
        self.waypoint_marker_topic = self.get_parameter("waypoint_marker_topic").value
        self.goal_waypoint_topic = self.get_parameter("goal_waypoint_topic").value
        self.waypoints_yaml = self.get_parameter("waypoints_yaml").value
        self.start_mode = self.get_parameter("start_mode").value
        self.start_waypoint = self.get_parameter("start_waypoint").value
        self.goal_waypoint = self.get_parameter("goal_waypoint").value
        self.has_goal = bool(self.get_parameter("plan_on_start").value)
        self.robot_frame = self.get_parameter("robot_frame").value
        self.block_size = int(self.get_parameter("block_size").value)
        self.occupied_fraction_threshold = float(
            self.get_parameter("occupied_fraction_threshold").value
        )
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.treat_unknown_as_occupied = bool(
            self.get_parameter("treat_unknown_as_occupied").value
        )

        self.waypoints = load_waypoints(self.waypoints_yaml)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(PathMsg, self.path_topic, 10)
        self.planning_grid_pub = self.create_publisher(
            OccupancyGrid, self.planning_grid_topic, 1
        )
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, self.waypoint_marker_topic, 1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 1
        )
        self.goal_sub = self.create_subscription(
            String, self.goal_waypoint_topic, self.goal_callback, 10
        )
        self.hold_sub = self.create_subscription(
            Bool, "/obstacle_hold", self.obstacle_hold_callback, 10
        )   # Subscribe to obstacle_hold so the planner can replan when an obstacle clears — handles static obstacles added after map build.
        self.create_timer(1.0, self.publish_waypoint_markers)

        # replan_needed flag prevents map_callback from republishing
        # /planned_path on every map update (which reset path_index=0 in
        # turtlebot_control 4x/second, causing the robot to never advance).
        self.replan_needed    = False
        self.obstacle_held    = False
        self.last_coarse_grid = None
        self.last_map_msg     = None

        self.get_logger().info(
            "LeePlanner: start_mode=%s, start=%s, goal=%s, block_size=%d"
            % (self.start_mode, self.start_waypoint, self.goal_waypoint, self.block_size)
        )

    def goal_callback(self, msg):
        waypoint_name = msg.data.strip()
        if waypoint_name not in self.waypoints:
            names = ", ".join(sorted(self.waypoints))
            self.get_logger().error(
                "Ignoring unknown nav goal '%s'. Available: %s" % (waypoint_name, names)
            )
            return

        self.start_waypoint = self.goal_waypoint # When a new goal is received, treat the old goal as the new start
        self.goal_waypoint = waypoint_name
        self.start_mode = "robot"
        self.has_goal = True
        self.replan_needed  = True  # trigger exactly one replan on next map frame
        self.get_logger().info("New navigation goal: %s" % self.goal_waypoint)

    def obstacle_hold_callback(self, msg):
        """
        When an obstacle clears (True → False), trigger a replan so the robot
        gets a fresh path that avoids any newly mapped static obstacle rather
        than resuming the old blocked path straight through it.
        """
        if self.obstacle_held and not msg.data and self.has_goal:
            self.replan_needed = True
            self.get_logger().info("Obstacle cleared — replanning path.")
        self.obstacle_held = msg.data

    def map_callback(self, msg):
        fine_grid = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        inflated    = self.inflate_obstacles(fine_grid, msg.info.resolution)
        coarse_grid = self.coarsen_grid(inflated)
 
        self.last_coarse_grid = coarse_grid
        self.last_map_msg     = msg
        self.publish_planning_grid(coarse_grid, msg)
 
        if not self.has_goal or not self.replan_needed:
            return
 
        if self.goal_waypoint not in self.waypoints:
            names = ", ".join(sorted(self.waypoints))
            self.get_logger().error(f"Unknown waypoint. Available: {names}")
            return
        if self.start_mode == "waypoint" and self.start_waypoint not in self.waypoints:
            names = ", ".join(sorted(self.waypoints))
            self.get_logger().error(f"Unknown start waypoint. Available: {names}")
            return
 
        start = self.start_cell(msg)
        if start is None:
            return
 
        goal = self.world_to_coarse_cell(
            self.waypoints[self.goal_waypoint].x,
            self.waypoints[self.goal_waypoint].y,
            msg,
        )
 
        start = self.snap_to_free(start, coarse_grid)
        goal  = self.snap_to_free(goal,  coarse_grid)
        path  = self.lee_search(coarse_grid, start, goal)
 
        if not path:
            self.get_logger().warn(
                "No Lee path found from %s to %s" % (self.start_label(), self.goal_waypoint),
                throttle_duration_sec=2.0,
            )
            return
 
        path = self.simplify_line_of_sight(path, coarse_grid)
        self.publish_path(path, msg)
        self.replan_needed = False  # clear — do not replan until next goal or obstacle clear

    def start_cell(self, map_msg):
        if self.start_mode == "waypoint":
            start = self.waypoints[self.start_waypoint]
            return self.world_to_coarse_cell(start.x, start.y, map_msg)

        if self.start_mode != "robot":
            self.get_logger().error(
                "Invalid start_mode '%s'. Use 'waypoint' or 'robot'." % self.start_mode
            )
            return None

        try:
            pose = self.tf_buffer.lookup_transform(
                map_msg.header.frame_id, self.robot_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().warn(
                "Robot pose lookup failed: %s" % exc,
                throttle_duration_sec=2.0,
            )
            return None

        return self.world_to_coarse_cell(
            pose.transform.translation.x,
            pose.transform.translation.y,
            map_msg,
        )

    def start_label(self):
        if self.start_mode == "robot":
            return self.robot_frame
        return self.start_waypoint

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

    def simplify_line_of_sight(self, path, coarse_grid):
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        current_index = 0

        while current_index < len(path) - 1:
            next_index = len(path) - 1
            while next_index > current_index + 1:
                if self.line_is_clear(path[current_index], path[next_index], coarse_grid):
                    break
                next_index -= 1

            simplified.append(path[next_index])
            current_index = next_index

        return simplified

    def line_is_clear(self, start, goal, coarse_grid):
        for row, col in self.bresenham_cells(start[0], start[1], goal[0], goal[1]):
            if not self.is_free(row, col, coarse_grid):
                return False
        return True

    @staticmethod
    def bresenham_cells(row0, col0, row1, col1):
        cells = []
        d_col = abs(col1 - col0)
        d_row = abs(row1 - row0)
        step_col = 1 if col0 < col1 else -1
        step_row = 1 if row0 < row1 else -1
        error = d_col - d_row

        row = row0
        col = col0
        while True:
            cells.append((row, col))
            if row == row1 and col == col1:
                return cells

            error2 = 2 * error
            if error2 > -d_row:
                error -= d_row
                col += step_col
            if error2 < d_col:
                error += d_col
                row += step_row

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

    def publish_waypoint_markers(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for marker_id, waypoint in enumerate(self.waypoints.values()):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = "map"
            marker.ns = "waypoints"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.x
            marker.pose.position.y = waypoint.y
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.18
            marker.scale.y = 0.18
            marker.scale.z = 0.18
            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.b = 0.05
            marker.color.a = 1.0
            markers.markers.append(marker)

            label = Marker()
            label.header.stamp = now
            label.header.frame_id = "map"
            label.ns = "waypoint_labels"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = waypoint.x
            label.pose.position.y = waypoint.y
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = waypoint.name
            markers.markers.append(label)

        self.waypoint_marker_pub.publish(markers)

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
