# restaurant_mapping

Small mapping package built around two lab ideas:

- Lab 4 provides the fixed saved map: `maps/slam_map.yaml` and `maps/slam_map.pgm`.
- Lab 6 provides the occupancy-grid update idea: LaserScan rays update a 2D log-odds grid.

The map YAML is the coordinate contract. `map_loader.py` reads its image, resolution, origin, and thresholds. `occupancy_grid_2d.py` then uses those values to set the grid bounds:

```text
x_min = origin_x
y_min = origin_y
x_num = image_width
y_num = image_height
resolution = yaml resolution
```

The node publishes:

- `/static_map`: the Lab 4 map as a ROS `OccupancyGrid`
- `/dynamic_map`: live Lab 6-style scan updates
- `/map`: static map plus dynamic overlay
- `/map_vis`: marker cubes for dynamic occupied cells
- `/waypoint_markers`: RViz markers for named waypoints
- `/planning_grid`: coarsened grid used by Lee's algorithm
- `/planned_path`: Lee path between named waypoints
- `/nav_goal_waypoint`: waypoint-name command input for the planner


Run after building and sourcing the ROS workspace:

ros2 launch restaurant_mapping mapping.launch.py
ros2 run plannedcntrl turtlebot_control
ros2 run waiter waiter_executive

IN RViz display:
TF  
MAP            /static_map       - map saved from slam
MAP            /dynamic_map      - map showing robot's lidar vision
MAP            /planning_grid    - the occupancy grid
MARKER ARRAY   /waypoint_markers - points of interest
MARKER         /map_vis          - obstacles detected by lidar
PATH           /planned_path     - simplified path of robot around obstacles

The Lee planner uses `block_size = 7` by default. With the Lab 4 map's
`0.05 m` resolution, each planning square is about `0.35 m x 0.35 m`.
Set `start_mode` to `robot` to plan from the current robot pose, or `waypoint`
to plan from `start_waypoint`.
