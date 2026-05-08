The Toasted Turtle 🐢 - Autonomous Robot Waiter

An autonomous TurtleBot3 waiter that navigates a simulated restaurant, takes orders from a diner, picks up food and water from the correct stations, delivers to the table, and avoids static and dynamic obstacles in real time.

1. System Architecture

┌─────────────────────────────────────┐
│           SENSING LAYER             │
│  LiDAR ──► occupancy_grid_2d        │
│  Camera ──► camera_perception       │
└───────────────┬─────────────────────┘
                │ /map  /obstacle_hold
┌───────────────▼─────────────────────┐
│           PLANNING LAYER            │
│  lee_planner ──► /planned_path      │
└───────────────┬─────────────────────┘
                │ /planned_path
┌───────────────▼─────────────────────┐
│           CONTROL LAYER             │
│  turtlebot_control ──► /cmd_vel     │
└───────────────┬─────────────────────┘
                │ /nav_status
┌───────────────▼─────────────────────┐
│        INTERACTION LAYER            │
│  waiter_executive ◄── diner input   │
│  kitchen_node ◄── /orders           │
└─────────────────────────────────────┘

2. Running the Project
- Run source install/setup.bash in every new terminal.

=== On the TurtleBot (SSH in) ===
- Terminal 1 — motors, LiDAR, TF
ros2 launch turtlebot3_bringup robot.launch.py
- Terminal 2 — webcam stream
ros2 launch usb_cam usb_cam.launch.py

=== Table ===
- Terminal 1: Backend (Mapping, Planning, Control, Perception)
colcon build
source install/setup.bash
ros2 launch waiter backend.launch.py

- Terminal 2: Frontend (Waiter CLI)
colcon build
source install/setup.bash
ros2 run waiter waiter_executive


=== Kitchen ===
colcon build
source install/setup.bash
export ROS_DOMAIN_ID=[domain id of table]
ros2 run waiter kitchen_node

=== RViz visualisation ===
rviz2
IN RViz display:
TF  
MAP            /static_map       - map saved from slam
MAP            /dynamic_map      - map showing robot's lidar vision
MAP            /planning_grid    - the occupancy grid
MARKER ARRAY   /waypoint_markers - points of interest
MARKER         /map_vis          - obstacles detected by lidar
PATH           /planned_path     - simplified path of robot around obstacles