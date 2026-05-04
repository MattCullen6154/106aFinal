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
            #Robot placed on X in front of station 2, facing NE
            Node( #Map/scan alignment (done with Kiwi, check others)| -0.55 -2.6 0 .6 0 0 map odom
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=[
                
                     "-0.55",
                    "-2.6",
                    "0.0",
                    ".7",
                    "0.0",
                    "0.0",
                    "map",
                    "odom",
                    
                ],
            ),
            Node(
                package="restaurant_mapping",
                executable="lee_planner",
                name="lee_planner",
                output="screen",
                parameters=[
                    {
                        "map_topic": "/map",
                        "path_topic": "/planned_path",
                        "planning_grid_topic": "/planning_grid",
                        "waypoint_marker_topic": "/waypoint_markers",
                        "goal_waypoint_topic": "/nav_goal_waypoint",
                        "waypoints_yaml": str(package_share / "config" / "waypoints.yaml"),
                        "start_mode": "robot",
                        "start_waypoint": "table",
                        "goal_waypoint": "kitchen",
                        "plan_on_start": False,
                        "robot_frame": "base_link",
                        "block_size": 7,
                        "occupied_fraction_threshold": 0.1,
                        "inflation_radius": 0.05,
                        "treat_unknown_as_occupied": False,
                    }
                ],
            ),
        ]
    )
