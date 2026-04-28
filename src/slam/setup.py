from glob import glob
import os

from setuptools import find_packages, setup

package_name = "slam"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join('share', package_name, 'maps'), ['maps/restaurant_map.yaml']),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Matt Cullen",
    maintainer_email="matthew_cullen@berkeley.edu",
    description="Starter LiDAR occupancy-grid mapping node for the 106A robot waiter project.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "slam_node = slam.slam_node:main",
            "plan_path = slam.plan_path:main",
            "path_publisher = slam.path_publisher:main",
        ],
    },
)

"""ros2 [581] ~/ros_workspaces/106aFinal # ros2 run slam plan_path recycle_bin alice_corner
Traceback (most recent call last):
  File "/home/cc/ee106a/sp26/class/ee106a-aac/ros_workspaces/106aFinal/install/slam/lib/slam/plan_path", line 33, in <module>
    sys.exit(load_entry_point('slam==0.0.0', 'console_scripts', 'plan_path')())
  File "/home/cc/ee106a/sp26/class/ee106a-aac/ros_workspaces/106aFinal/install/slam/lib/python3.10/site-packages/slam/plan_path.py", line 32, in main
    grid_map, coarse_map, waypoints = build_planner(
  File "/home/cc/ee106a/sp26/class/ee106a-aac/ros_workspaces/106aFinal/install/slam/lib/python3.10/site-packages/slam/planner.py", line 220, in build_planner
    waypoints = load_waypoints(waypoint_yaml_path)
  File "/home/cc/ee106a/sp26/class/ee106a-aac/ros_workspaces/106aFinal/install/slam/lib/python3.10/site-packages/slam/planner.py", line 33, in load_waypoints
    with yaml_path.open("r", encoding="utf-8") as handle:
  File "/usr/lib/python3.10/pathlib.py", line 1119, in open
    return self._accessor.open(self, mode, buffering, encoding, errors,
FileNotFoundError: [Errno 2] No such file or directory: '/home/cc/ee106a/sp26/class/ee106a-aac/ros_workspaces/106aFinal/install/slam/lib/python3.10/site-packages/config/waypoints.yaml'
"""