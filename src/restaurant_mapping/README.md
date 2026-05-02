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

Run after building and sourcing the ROS workspace:

```bash
ros2 launch restaurant_mapping mapping.launch.py
```
