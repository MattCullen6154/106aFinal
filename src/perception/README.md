# perception

Camera perception package for the autonomous robot waiter.
Subscribes to the Logitech webcam stream, runs a YOLOv8 segmentation model to detect people and large obstacles, estimates their depth from mask pixel count and camera intrinsics, and publishes a 3-D obstacle position and a binary hold/clear signal for the navigation stack.

1. Nodes:
camera_perception
File: perception/camera_perception.py
Launch: ros2 run perception camera_perception

2. Subscribed topics
Topic           Type                    Description
/image_raw      sensor_msgs/Image       Raw frames from the Logitech webcam
/camera_info    sensor_msgs/CameraInfo  Camera intrinsics (fx, fy, cx, cy)

3. Published topics
Topic               Type                            Description
/obstacle_point     geometry_msgs/PointStamped      3-D position of the nearest obstacle in the base_link frame
/obstacle_hold      std_msgs/Bool                   True = obstacle within safety threshold, stop; False = path clear, resume

4. Package files
File                        Purpose
camera_perception.py        Main perception node
package.xml                 ROS2 dependencies
setup.py                    Entry points and model asset registration
utilities/yolov8n-seg.pt    YOLOv8 nano segmentation model (COCO pretrained)

5. Dependencies
ROS2 packages (declared in package.xml):
rclpy
sensor_msgs
geometry_msgs
std_msgs
vision_msgs
cv_bridge
python3-opencv

Python packages (install separately):
pip install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolov8n-seg.pt')"

6. Build & run
# From workspace root
colcon build
source install/setup.bash
ros2 run perception camera_perception

Verify output:
ros2 topic echo /obstacle_hold    # True when obstacle within 0.5 m
ros2 topic echo /obstacle_point   # 3-D obstacle position in base_link frame

Architecture
/image_raw  ──┐
              ├──▶  camera_perception  ──▶  /obstacle_point
/camera_info──┘                       ──▶  /obstacle_hold
/obstacle_hold is consumed by plannedcntrl/turtlebot_control to pause wheel velocity commands when an obstacle is too close. /obstacle_point is available for the global planner to incorporate into replanning.

7. Notes
- The node only reacts to COCO classes relevant to a lab environment. Edit OBSTACLE_CLASSES and OBSTACLE_AREA at the top of camera_perception.py to add or remove classes.
- Depth estimation uses the mask pixel count and an assumed real-world frontal area. Accuracy degrades at distances beyond ~3 m or when objects are partially occluded.