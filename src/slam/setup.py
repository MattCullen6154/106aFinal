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
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
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
            "coarse_map_publisher = slam.coarse_map_publisher:main",
            "waypoint_pose_publisher = slam.waypoint_pose_publisher:main",
        ],
    },
)
