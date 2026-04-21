# robot_waiter_slam

Starter SLAM package for the 106A robot waiter project.

This package follows the ROS 2 Python package shape used in `lab4_code/lab4/src`.
The first node, `slam_node`, builds a simple 2D occupancy grid from TurtleBot
LiDAR scans and odometry:

- subscribes to `/scan` (`sensor_msgs/LaserScan`)
- subscribes to `/odom` (`nav_msgs/Odometry`)
- publishes `/map` (`nav_msgs/OccupancyGrid`)

This is intentionally a readable baseline, not a replacement for a production
SLAM package like `slam_toolbox`. It gives the team a project-owned mapping node
that can be extended with loop closure, better pose estimation, semantic
obstacle layers, or Nav2 integration.

## Build

From `/Users/matt/Documents/106aFinal/lab4_code/lab4`:

```bash
colcon build --symlink-install
source install/setup.zsh
```

## Run

With the TurtleBot publishing `/scan` and `/odom`:

```bash
ros2 launch robot_waiter_slam slam.launch.py
```

To inspect the map:

```bash
ros2 topic echo /map --once
rviz2
```

In RViz, set the fixed frame to `map` and add a `Map` display for `/map`.

## Next good extensions

- Add a static or TF-based transform from the LiDAR frame to `base_link`.
- Replace odometry-only pose with a scan-matching correction step.
- Save named restaurant stations, such as kitchen, water, and table waypoints.
- Feed the published map into Nav2 for global planning.
