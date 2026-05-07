=== Table ===
Terminal 1: Backend (Mapping, Planning, Control)
colcon build
source install/setup.bash
ros2 launch waiter backend.launch.py

Terminal 2: Frontend (Waiter CLI)
colcon build
source install/setup.bash
ros2 run waiter waiter_executive


=== Kitchen ===
colcon build
source install/setup.bash
export ROS_DOMAIN_ID=[domain id of table]
ros2 run waiter kitchen_node
