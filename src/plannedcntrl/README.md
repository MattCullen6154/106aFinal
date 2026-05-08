# plannedcntrl

- Trajectory planning and motion control package for the autonomous robot waiter.
- Provides a Bézier curve trajectory generator and a path-following controller that subscribes to pre-planned paths from the restaurant_mapping Lee planner and halts when the perception layer signals an obstacle.

1. Nodes
turtlebot_control: Path-following controller. Tracks waypoints from /planned_path, applies proportional heading and distance control, and freezes motion when /obstacle_hold is True.
File: plannedcntrl/turtlebot_control.py
Launch: ros2 run plannedcntrl turtlebot_control

2. Subscribed topics
Topic           Type            Description
/planned_path   nav_msgs/Path   Sequence of waypoints from the Lee planner
/obstacle_hold  std_msgs/Bool   Hold signal from the perception node

3. Published topics
Topic           Type                    Description
/cmd_vel        geometry_msgs/Twist     Wheel velocity commands
/nav_status     std_msgs/String         moving/ holding/ arrived

4. Parameters
Parameter               Default         Description
path_topic              /planned_path   Incoming path topic
status_topic            /nav_status     Outgoing status topic
cmd_vel_topic           /cmd_vel        Velocity command topic
obstacle_hold_topic     /obstacle_hold  Hold signal topic
robot_frame             base_link       TF frame used for pose lookup
position_tolerance      0.22 m          Waypoint acceptance radius (intermediate points)
goal_tolerance          0.3 m           Waypoint acceptance radius (final goal)
linear_gain             0.35            Proportional gain for forward speed
angular_gain            0.2             Proportional gain for heading correction
max_linear_speed        0.15 m/s        Speed cap — kept low for safe food/drink transport
max_angular_speed       0.18 rad/s      Turn rate cap


# trajectory
Offline Bézier trajectory generator. Computes smooth cubic Bézier paths between two poses. Can run standalone for testing without a live robot.

File: plannedcntrl/trajectory.py
Launch: ros2 run plannedcntrl trajectory

Key functions:
Function                                                                Description
generate_bezier_waypoints(x1,y1,θ1, x2,y2,θ2, offset, num_points)       Pure geometry — returns list of (x, y, θ) tuples along a smooth Bézier path
plan_curved_trajectory(target_position, offset, num_points)             Looks up live robot pose via TF, converts a base_link-relative target to an odom-frame Bézier path
plot_trajectory(waypoints)                                              Matplotlib debug visualisation

Running ros2 run plannedcntrl trajectory executes an offline smoke-test simulating a kitchen → dining table segment and displays the trajectory plot.

5. Package files
File                    Purpose
turtlebot_control.py    Path-following controller node
trajectory.py           Bézier trajectory generator
package.xml             ROS 2 dependencies
setup.py                Entry points

6. Dependencies
ROS 2 packages (declared in package.xml):
rclpy
geometry_msgs
nav_msgs
std_msgs
tf2_ros

7. Build & run
# From workspace root
colcon build
source install/setup.bash

# Terminal 1 — path-following controller
ros2 run plannedcntrl turtlebot_control

# Terminal 2 — offline Bézier smoke-test (no robot required)
ros2 run plannedcntrl trajectory
Send a navigation goal at runtime:
ros2 topic pub --once /nav_goal_waypoint std_msgs/String "{data: 'table'}"
Valid waypoint names are defined in restaurant_mapping/config/waypoints.yaml (e.g. kitchen, table, water).

Verify output:
ros2 topic echo /nav_status     # moving/ holding/ arrived
ros2 topic echo /cmd_vel        # wheel velocity commands

8. Architecture
restaurant_mapping                 perception
  (lee_planner)                  (camera_perception)
       │                                │
       │ /planned_path                  │ /obstacle_hold
       ▼                                ▼
              turtlebot_control  ──▶  /cmd_vel
                     │
                     ▼
               /nav_status
          (→ interaction node)
The controller sits between the global planner and the motors. It tracks the path waypoint-by-waypoint and defers entirely to the perception node for obstacle-induced holds — it does not do its own obstacle detection.

9. Notes
- Speed parameters (max_linear_speed, max_angular_speed) are intentionally conservative to keep motion smooth and predictable while carrying food and drinks.
- The heading deadband (commented out in turtlebot_control.py) can be re-enabled to reduce jerkiness if oscillation is observed during straight-line segments.