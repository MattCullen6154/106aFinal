from glob import glob
import os

from setuptools import find_packages, setup


package_name = "restaurant_mapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Matt Cullen",
    maintainer_email="matthew_cullen@berkeley.edu",
    description="Lab4-map-backed occupancy grid mapping for the robot waiter project.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "mapping_node = restaurant_mapping.mapping_node:main",
            "lee_planner = restaurant_mapping.lee_planner:main",
        ],
    },
)
