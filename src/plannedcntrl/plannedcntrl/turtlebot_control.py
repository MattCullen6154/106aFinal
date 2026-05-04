#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros


class TurtleBotController(Node):
    def __init__(self):
        super().__init__("turtlebot_controller")

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("status_topic", "/nav_status")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("position_tolerance", 0.22)
        self.declare_parameter("goal_tolerance", 0.5)
        self.declare_parameter("linear_gain", 0.25)
        self.declare_parameter("angular_gain", 0.5)
        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("max_angular_speed", 0.4)

        self.path_topic = self.get_parameter("path_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_frame = "map"
        self.path_points = []
        self.path_index = 0
        self.arrival_reported = True

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("TurtleBot path controller listening on %s" % self.path_topic)

    def path_callback(self, msg):
        if not msg.poses:
            self.get_logger().warn("Ignoring empty planned path.")
            return

        self.path_frame = msg.header.frame_id or "map"
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        self.path_index = 0
        self.arrival_reported = False
        self.publish_status("moving")
        self.get_logger().info("Received path with %d poses." % len(self.path_points))

    def control_loop(self):
        if not self.path_points:
            return

        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y, robot_yaw = robot_pose
        target_x, target_y = self.path_points[self.path_index]
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)

        tolerance = self.current_target_tolerance()
        if distance < tolerance:
            self.path_index += 1
            if self.path_index >= len(self.path_points):
                self.stop_robot()
                self.path_points = []
                if not self.arrival_reported:
                    self.publish_status("arrived")
                    self.arrival_reported = True
                    self.get_logger().info(
                        "Arrived at planned path goal within %.2fm." % self.goal_tolerance
                    )
                return
            return

        heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(heading - robot_yaw)

        cmd = Twist()

        if abs(heading_error) < 1.0:
            cmd.linear.x = self.clamp(
                self.linear_gain * distance,
                -self.max_linear_speed,
                self.max_linear_speed,
            )
        cmd.angular.z = self.clamp(
            self.angular_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self.cmd_pub.publish(cmd)

    def lookup_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame, self.robot_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().warn(
                "Robot pose lookup failed: %s" % exc,
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            translation.x,
            translation.y,
            self.yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def current_target_tolerance(self):
        if self.path_index >= len(self.path_points) - 1:
            return self.goal_tolerance
        return self.position_tolerance

    def publish_status(self, status):
        self.status_pub.publish(String(data=status))

    @staticmethod
    def yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
