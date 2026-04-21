#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class SimpleLidarSlam(Node):
    """Odometry-based occupancy-grid mapper for getting the project moving."""

    def __init__(self):
        super().__init__("slam_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width_m", 10.0)
        self.declare_parameter("height_m", 10.0)
        self.declare_parameter("origin_x", -5.0)
        self.declare_parameter("origin_y", -5.0)
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("max_range", 3.5)
        self.declare_parameter("occupied_log_odds", 0.85)
        self.declare_parameter("free_log_odds", -0.4)
        self.declare_parameter("min_log_odds", -4.0)
        self.declare_parameter("max_log_odds", 4.0)
        self.declare_parameter("occupied_threshold", 0.65)
        self.declare_parameter("free_threshold", 0.35)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.width_m = float(self.get_parameter("width_m").value)
        self.height_m = float(self.get_parameter("height_m").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.occupied_update = float(self.get_parameter("occupied_log_odds").value)
        self.free_update = float(self.get_parameter("free_log_odds").value)
        self.min_log_odds = float(self.get_parameter("min_log_odds").value)
        self.max_log_odds = float(self.get_parameter("max_log_odds").value)
        self.occupied_threshold = float(self.get_parameter("occupied_threshold").value)
        self.free_threshold = float(self.get_parameter("free_threshold").value)

        self.width_cells = int(round(self.width_m / self.resolution))
        self.height_cells = int(round(self.height_m / self.resolution))
        self.log_odds = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)
        self.robot_pose = None

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_timer(float(self.get_parameter("publish_period").value), self.publish_map)

        self.get_logger().info(
            "SimpleLidarSlam: scan='%s', odom='%s', map='%s', size=%.1fm x %.1fm, resolution=%.2fm"
            % (
                self.scan_topic,
                self.odom_topic,
                self.map_topic,
                self.width_m,
                self.height_m,
                self.resolution,
            )
        )

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.robot_pose = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def scan_callback(self, msg):
        if self.robot_pose is None:
            self.get_logger().warn("Waiting for odometry before integrating scans.", throttle_duration_sec=2.0)
            return

        robot_x, robot_y, robot_yaw = self.robot_pose
        start_cell = self.world_to_grid(robot_x, robot_y)
        if start_cell is None:
            self.get_logger().warn("Robot pose is outside the configured map bounds.", throttle_duration_sec=2.0)
            return

        for i, scan_range in enumerate(msg.ranges):
            if not math.isfinite(scan_range) or scan_range < msg.range_min:
                continue

            hit_obstacle = scan_range <= min(msg.range_max, self.max_range)
            usable_range = min(scan_range, msg.range_max, self.max_range)
            beam_angle = robot_yaw + msg.angle_min + i * msg.angle_increment
            end_x = robot_x + usable_range * math.cos(beam_angle)
            end_y = robot_y + usable_range * math.sin(beam_angle)
            end_cell = self.world_to_grid(end_x, end_y)
            if end_cell is None:
                continue

            ray_cells = self.bresenham(start_cell[0], start_cell[1], end_cell[0], end_cell[1])
            for cell_x, cell_y in ray_cells[:-1]:
                self.update_cell(cell_x, cell_y, self.free_update)

            if hit_obstacle:
                self.update_cell(end_cell[0], end_cell[1], self.occupied_update)

    def world_to_grid(self, x, y):
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        if 0 <= grid_x < self.width_cells and 0 <= grid_y < self.height_cells:
            return grid_x, grid_y
        return None

    def update_cell(self, grid_x, grid_y, update):
        self.log_odds[grid_y, grid_x] = np.clip(
            self.log_odds[grid_y, grid_x] + update,
            self.min_log_odds,
            self.max_log_odds,
        )

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                return cells

            err2 = 2 * err
            if err2 > -dy:
                err -= dy
                x0 += step_x
            if err2 < dx:
                err += dx
                y0 += step_y

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        probabilities = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        grid = np.full(probabilities.shape, -1, dtype=np.int8)
        grid[probabilities >= self.occupied_threshold] = 100
        grid[probabilities <= self.free_threshold] = 0
        msg.data = grid.flatten().tolist()

        self.map_pub.publish(msg)
        self.publish_map_to_odom_tf(msg.header.stamp)

    def publish_map_to_odom_tf(self, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLidarSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
