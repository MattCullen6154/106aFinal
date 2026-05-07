import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Path to the restaurant_mapping launch file
    mapping_pkg_dir = get_package_share_directory("restaurant_mapping")
    mapping_launch_path = os.path.join(mapping_pkg_dir, "launch", "mapping.launch.py")

    return LaunchDescription(
        [
            # 2. Include the Mapping Launch File
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path)
            ),
            # 3. Run the Control Node (from plannedcntrl package)
            Node(
                package="plannedcntrl",
                executable="turtlebot_control",
                name="turtlebot_control",
                output="screen",
            ),
            # 4. Run the Waiter Executive Node (from waiter package)
            Node(
                package="waiter",
                executable="waiter_executive",
                name="waiter_executive",
                output="screen",
                parameters=[{"use_sim_time": True}],  # Add parameters if needed
            ),
        ]
    )
