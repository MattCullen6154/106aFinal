import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("slam")
    params_file = os.path.join(package_share, "config", "slam_params.yaml")

    return LaunchDescription(
        [
            Node(
                package="slam",
                executable="slam_node",
                name="slam_node",
                output="screen",
                parameters=[params_file],
            )
        ]
    )