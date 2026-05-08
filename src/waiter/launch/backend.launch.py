import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mapping_pkg_dir = get_package_share_directory("restaurant_mapping")
    mapping_launch_path = os.path.join(mapping_pkg_dir, "launch", "mapping.launch.py")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path)
            ),
            Node(
                package="plannedcntrl",
                executable="turtlebot_control",
                name="turtlebot_control",
                output="screen",
            ),
            Node(
                package="perception",
                executable="camera_perception",
                name="camera_perception",
                output="screen",
            ),
        ]
    )
