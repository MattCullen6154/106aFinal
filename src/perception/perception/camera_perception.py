#camera_perception.py

import rclpy
import cv2
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

# COCO class IDs that count as semantic obstacles the waiter robot should react to.
# 0 = person, 24 = backpack, 26 = handbag, 56 = chair, 60 = dining table, 63 = laptop
OBSTACLE_CLASSES = {0, 24, 26, 56, 60, 63}

# Approximate real-world frontal area (m²) used for mask-based depth estimation.
OBSTACLE_AREA = {
    0:  0.50,   # person
    24: 0.04,   # backpack
    26: 0.03,   # handbag
    56: 0.30,   # chair
    60: 0.80,   # dining table
    63: 0.04,   # laptop
}
DEFAULT_OBSTACLE_AREA = 0.03  # fallback for any unlisted class
 
# If the closest detected obstacle is within this distance (meters), command a hold.
HOLD_DISTANCE_THRESHOLD = 0.5

class CameraPerceptionNode(Node):
    """
    Camera perception node for the autonomous robot waiter.
    Subscribes to the Logitech webcam image stream, runs a YOLO segmentation
    model to detect people and large obstacles, estimates their depth from the
    mask pixel count and camera intrinsics, and publishes:
      - /obstacle_point  (geometry_msgs/PointStamped) : 3-D position of the
                          nearest obstacle in the base_link frame.
      - /obstacle_hold   (std_msgs/Bool)              : True  → stop / hold,
                                                        False → path is clear.
    """
    def __init__(self):
        super().__init__('camera_perception')

        self.bridge = CvBridge()

        # Load YOLO model
        package_share_dir = get_package_share_directory('perception')
        model_path = os.path.join(package_share_dir, 'utilities', 'yolov8n-seg.pt')
        self.model = YOLO(model_path)

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 1)
        self.obstacle_position_pub = self.create_publisher(PointStamped, '/obstacle_point', 1)
        self.hold_pub = self.create_publisher(Bool, '/obstacle_hold', 1)
        self.camera_intrinsics = None

        self.get_logger().info('Camera Perception Node initialized')

    def image_callback(self, msg):
        """Process each frame: detect obstacles, estimate depth, publish."""
        if self.camera_intrinsics is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        results = self.model(cv_image, verbose=False)

        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx = self.camera_intrinsics['cx']
        cy = self.camera_intrinsics['cy']

        closest_distance = float('inf')
        closest_point_base = None

        for result in results:
            if result.masks is None:
                continue
            masks = result.masks.data.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)

            for i, (mask, cls_id) in enumerate(zip(masks, classes)):
                # Only react to the obstacle classes defined above
                if cls_id not in OBSTACLE_CLASSES:
                    continue
                # Depth estimation from mask area
                pixel_count = np.sum(mask > 0)
                if pixel_count == 0:
                    continue

                real_area = OBSTACLE_AREA.get(cls_id, DEFAULT_OBSTACLE_AREA)
                depth = np.sqrt(fx * fy * real_area / pixel_count)
 
                cls_name = self.model.names.get(cls_id, str(cls_id))
                self.get_logger().info(f'Obstacle {i + 1} ({cls_name}): depth={depth:.3f} m')
                
                # Image-plane centroid of the mask
                ys, xs = np.where(mask > 0)
                u, v = np.mean(xs), np.mean(ys)

                # Back-project to 3-D camera coordinates
                X = (u - cx) * depth / fx
                Y = (v - cy) * depth / fy
                Z = depth

                # Convert to turtlebot frame
                # Camera is mounted on the robot without a dedicated TF frame,
                # so we apply the fixed camera-to-base_link transform directly.
                G = np.array([[0, 0, 1, 0.115],
                    [-1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]])
                goal_point = (G @ np.array([X, Y, Z, 1]).reshape(4, 1)).flatten()

                # Track the nearest obstacle to decide on hold command
                if depth < closest_distance:
                    closest_distance = depth
                    closest_point_base = goal_point
 
        # Publish results
        hold_msg = Bool()
 
        if closest_point_base is not None:
            # Publish 3-D position of the nearest obstacle
            point_msg = PointStamped()
            point_msg.header.stamp = msg.header.stamp
            point_msg.header.frame_id = 'base_link'
            point_msg.point.x = float(closest_point_base[0])
            point_msg.point.y = float(closest_point_base[1])
            point_msg.point.z = float(closest_point_base[2])
            self.obstacle_position_pub.publish(point_msg)
 
            # Command a hold if the obstacle is within the safety threshold
            hold_msg.data = bool(closest_distance < HOLD_DISTANCE_THRESHOLD)
            if hold_msg.data:
                self.get_logger().warn(
                    f'Obstacle within {HOLD_DISTANCE_THRESHOLD} m '
                    f'({closest_distance:.2f} m) — publishing HOLD'
                )
        else:
            # No relevant obstacles detected — path is clear
            hold_msg.data = False
            self.get_logger().info('No obstacles detected — path clear')
 
        self.hold_pub.publish(hold_msg)
 


    def camera_info_callback(self, msg):
        # Extract and cache camera intrinsic parameters from CameraInfo
        self.get_logger().info("Recieved Camera Info")
        K = msg.k
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]
        self.camera_intrinsics = {'fx' : fx,
                                  'fy' : fy,
                                  'cx' : cx,
                                  'cy' : cy}
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

