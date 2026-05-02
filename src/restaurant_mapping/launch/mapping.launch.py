from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("restaurant_mapping"))

    return LaunchDescription(
        [
            Node(
                package="restaurant_mapping",
                executable="mapping_node",
                name="restaurant_mapping",
                output="screen",
                parameters=[
                    {
                        "map_yaml": str(package_share / "maps" / "slam_map.yaml"),
                        "topics/sensor": "/scan",
                        "topics/static_map": "/static_map",
                        "topics/dynamic_map": "/dynamic_map",
                        "topics/combined_map": "/map",
                        "topics/vis": "/map_vis",
                        "frames/sensor": "base_scan",
                        "frames/fixed": "map",
                        "random_downsample": 0.1,
                        "publish_period": 1.0,
                        "max_range": 3.5,
                    }
                ],
            ),
            Node( #Map/scan alignment (done with Kiwi, check others)| -0.87 -1.875 0 -.12 0 0 map odom
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=[
                
                     "-0.97",
                    "-1.875",
                    "0.0",
                    "-0.12",
                    "0.0",
                    "0.0",
                    "map",
                    "odom",
                    
                ],
            )
        ]
    )
