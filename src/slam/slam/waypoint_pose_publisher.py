#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

from .planner import load_waypoints


class WaypointPosePublisher(Node):
    def __init__(self):
        super().__init__("waypoint_pose_publisher")

        package_share = Path(get_package_share_directory("slam"))
        default_waypoints_yaml = str(package_share / "config" / "waypoints.yaml")

        self.declare_parameter("waypoints_yaml", default_waypoints_yaml)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("topic_prefix", "/waypoint")

        self.waypoints_yaml = self.get_parameter("waypoints_yaml").value
        self.frame_id = self.get_parameter("frame_id").value
        self.topic_prefix = self.get_parameter("topic_prefix").value.rstrip("/")

        self.publishers = {}
        self.messages = {}
        self._load_waypoint_messages()
        self.create_timer(float(self.get_parameter("publish_period").value), self.publish_waypoints)

        self.get_logger().info(
            "WaypointPosePublisher: publishing %d waypoints from '%s'"
            % (len(self.messages), self.waypoints_yaml)
        )

    def _load_waypoint_messages(self):
        waypoints = load_waypoints(self.waypoints_yaml)
        for name, waypoint in waypoints.items():
            topic = f"{self.topic_prefix}/{name}"
            publisher = self.create_publisher(PoseStamped, topic, 1)

            msg = PoseStamped()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = waypoint.x
            msg.pose.position.y = waypoint.y
            msg.pose.position.z = 0.0
            if waypoint.quaternion is None:
                msg.pose.orientation.w = 1.0
            else:
                msg.pose.orientation.x = waypoint.quaternion[0]
                msg.pose.orientation.y = waypoint.quaternion[1]
                msg.pose.orientation.z = waypoint.quaternion[2]
                msg.pose.orientation.w = waypoint.quaternion[3]

            self.publishers[name] = publisher
            self.messages[name] = msg

    def publish_waypoints(self):
        now = self.get_clock().now().to_msg()
        for name, msg in self.messages.items():
            msg.header.stamp = now
            self.publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
