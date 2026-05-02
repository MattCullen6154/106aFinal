import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
import tf2_ros
from visualization_msgs.msg import Marker

from .map_loader import load_map


class OccupancyGrid2d(Node):
    def __init__(self):
        super().__init__("restaurant_mapping")
        self._initialized = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.load_parameters()
        self.register_callbacks()
        self.load_lab4_map()

        self._dynamic_log_odds = np.zeros((self._y_num, self._x_num), dtype=np.float32)
        self._initialized = True
        self.get_logger().info(
            "Restaurant map loaded: %dx%d cells, %.3fm resolution, origin=(%.2f, %.2f)"
            % (self._x_num, self._y_num, self._resolution, self._x_min, self._y_min)
        )

    def load_parameters(self):
        package_share = Path(get_package_share_directory("restaurant_mapping"))
        default_map_yaml = str(package_share / "maps" / "slam_map.yaml")

        self.declare_parameter("map_yaml", default_map_yaml)
        self.declare_parameter("random_downsample", 0.1)
        self.declare_parameter("update/occupied", 0.7)
        self.declare_parameter("update/occupied_threshold", 0.97)
        self.declare_parameter("update/free", 0.3)
        self.declare_parameter("update/free_threshold", 0.03)
        self.declare_parameter("topics/sensor", "/scan")
        self.declare_parameter("topics/static_map", "/static_map")
        self.declare_parameter("topics/dynamic_map", "/dynamic_map")
        self.declare_parameter("topics/combined_map", "/map")
        self.declare_parameter("topics/vis", "/map_vis")
        self.declare_parameter("frames/sensor", "base_scan")
        self.declare_parameter("frames/fixed", "map")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("max_range", 3.5)

        self._map_yaml = self.get_parameter("map_yaml").value
        self._random_downsample = float(self.get_parameter("random_downsample").value)
        self._occupied_update = self.probability_to_logodds(
            float(self.get_parameter("update/occupied").value)
        )
        self._occupied_threshold = self.probability_to_logodds(
            float(self.get_parameter("update/occupied_threshold").value)
        )
        self._free_update = self.probability_to_logodds(
            float(self.get_parameter("update/free").value)
        )
        self._free_threshold = self.probability_to_logodds(
            float(self.get_parameter("update/free_threshold").value)
        )
        self._sensor_topic = self.get_parameter("topics/sensor").value
        self._static_map_topic = self.get_parameter("topics/static_map").value
        self._dynamic_map_topic = self.get_parameter("topics/dynamic_map").value
        self._combined_map_topic = self.get_parameter("topics/combined_map").value
        self._vis_topic = self.get_parameter("topics/vis").value
        self._sensor_frame = self.get_parameter("frames/sensor").value
        self._fixed_frame = self.get_parameter("frames/fixed").value
        self._publish_period = float(self.get_parameter("publish_period").value)
        self._max_range = float(self.get_parameter("max_range").value)

    def register_callbacks(self):
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sensor_sub = self.create_subscription(
            LaserScan, self._sensor_topic, self.sensor_callback, qos_profile
        )
        self._static_map_pub = self.create_publisher(OccupancyGrid, self._static_map_topic, 1)
        self._dynamic_map_pub = self.create_publisher(OccupancyGrid, self._dynamic_map_topic, 1)
        self._combined_map_pub = self.create_publisher(OccupancyGrid, self._combined_map_topic, 1)
        self._vis_pub = self.create_publisher(Marker, self._vis_topic, 10)
        self.create_timer(self._publish_period, self.publish_maps)

    def load_lab4_map(self):
        loaded_map = load_map(self._map_yaml)
        meta = loaded_map.metadata

        self._static_occupancy = loaded_map.occupancy
        self._resolution = meta.resolution
        self._x_res = meta.resolution
        self._y_res = meta.resolution
        self._x_num = meta.width
        self._y_num = meta.height
        self._x_min = meta.origin_x
        self._y_min = meta.origin_y
        self._x_max = self._x_min + self._x_num * self._x_res
        self._y_max = self._y_min + self._y_num * self._y_res

    def sensor_callback(self, msg):
        if not self._initialized:
            return

        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}", throttle_duration_sec=2.0)
            return

        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
        qx = pose.transform.rotation.x
        qy = pose.transform.rotation.y
        qz = pose.transform.rotation.z
        qw = pose.transform.rotation.w
        yaw = self.yaw_from_quaternion(qx, qy, qz, qw)

        start_cell = self.point_to_voxel(sensor_x, sensor_y)
        if start_cell is None:
            self.get_logger().warn("Sensor pose is outside the map.", throttle_duration_sec=2.0)
            return

        for index, scan_range in enumerate(msg.ranges):
            if np.random.rand() > self._random_downsample:
                continue
            if not math.isfinite(scan_range) or scan_range < msg.range_min:
                continue

            usable_range = min(scan_range, msg.range_max, self._max_range)
            hit_obstacle = scan_range <= min(msg.range_max, self._max_range)
            angle = yaw + msg.angle_min + index * msg.angle_increment
            end_x = sensor_x + usable_range * math.cos(angle)
            end_y = sensor_y + usable_range * math.sin(angle)
            end_cell = self.point_to_voxel(end_x, end_y)
            if end_cell is None:
                continue

            ray_cells = self.bresenham(start_cell[0], start_cell[1], end_cell[0], end_cell[1])
            for x_cell, y_cell in ray_cells[:-1]:
                self.update_cell(x_cell, y_cell, self._free_update)

            if hit_obstacle:
                self.update_cell(end_cell[0], end_cell[1], self._occupied_update)

        self.visualize()

    def point_to_voxel(self, x, y):
        ii = int((x - self._x_min) / self._x_res)
        jj = int((y - self._y_min) / self._y_res)

        if ii < 0 or ii >= self._x_num or jj < 0 or jj >= self._y_num:
            return None
        return ii, jj

    def voxel_center(self, ii, jj):
        return (
            self._x_min + (0.5 + ii) * self._x_res,
            self._y_min + (0.5 + jj) * self._y_res,
        )

    def update_cell(self, ii, jj, update):
        self._dynamic_log_odds[jj, ii] = np.clip(
            self._dynamic_log_odds[jj, ii] + update,
            self._free_threshold,
            self._occupied_threshold,
        )

    def dynamic_occupancy(self):
        probability = 1.0 - 1.0 / (1.0 + np.exp(self._dynamic_log_odds))
        grid = np.full(probability.shape, -1, dtype=np.int8)
        grid[probability >= 0.65] = 100
        grid[probability <= 0.35] = 0
        return grid

    def combined_occupancy(self):
        dynamic = self.dynamic_occupancy()
        combined = self._static_occupancy.copy()

        known_dynamic = dynamic != -1
        combined[known_dynamic] = dynamic[known_dynamic]
        combined[self._static_occupancy == 100] = 100
        return combined

    def publish_maps(self):
        if not self._initialized:
            return

        self._static_map_pub.publish(self.make_occupancy_grid(self._static_occupancy))
        self._dynamic_map_pub.publish(self.make_occupancy_grid(self.dynamic_occupancy()))
        self._combined_map_pub.publish(self.make_occupancy_grid(self.combined_occupancy()))

    def make_occupancy_grid(self, occupancy):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._fixed_frame
        msg.info.resolution = self._resolution
        msg.info.width = self._x_num
        msg.info.height = self._y_num
        msg.info.origin.position.x = self._x_min
        msg.info.origin.position.y = self._y_min
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy.flatten().tolist()
        return msg

    def visualize(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._fixed_frame
        marker.ns = "dynamic_occupancy"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self._x_res
        marker.scale.y = self._y_res
        marker.scale.z = 0.02
        marker.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.45)

        dynamic = self.dynamic_occupancy()
        occupied_rows, occupied_cols = np.nonzero(dynamic == 100)
        for row, col in zip(occupied_rows, occupied_cols):
            x, y = self.voxel_center(col, row)
            marker.points.append(Point(x=x, y=y, z=0.01))

        self._vis_pub.publish(marker)

    @staticmethod
    def probability_to_logodds(probability):
        return math.log(probability / (1.0 - probability))

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

    @staticmethod
    def yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
