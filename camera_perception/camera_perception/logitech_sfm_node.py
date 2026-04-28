import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

class LogitechSFMNode(Node):
    def __init__(self):
        super().__init__('logitech_sfm_node')
        
        # ROS Parameters
        self.declare_parameter('video_device', '/dev/video0')
        self.video_device = self.get_parameter('video_device').value
        
        # Publisher for the 3D Point Cloud
        self.pc_pub = self.create_publisher(PointCloud2, '/camera/sfm/point_cloud', 10)
        
        # Initialize Logitech Webcam
        self.cap = cv2.VideoCapture(self.video_device)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open webcam at {self.video_device}")
            return
            
        # Logitech C922 / Standard Webcam Intrinsic Camera Matrix (K)
        self.K = np.array([[1420.0, 0.0, 960.0],
                           [0.0, 1420.0, 540.0],
                           [0.0, 0.0, 1.0]])
        
        # SfM Variables
        self.prev_frame = None
        self.prev_points = None
        
        # Initial Camera Pose (Identity Matrix & Zero Translation)
        self.R_f = np.eye(3, dtype=np.float64)
        self.t_f = np.zeros((3, 1), dtype=np.float64)
        
        # Timer to run the camera capture and SfM loop (~15 FPS)
        self.timer = self.create_timer(1.0 / 15.0, self.sfm_loop)
        self.get_logger().info("Logitech SfM Node Initialized. Capturing and Triangulating...")

    def detect_features(self, frame_gray):
        # Use Shi-Tomasi corner detection to find good points to track
        points = cv2.goodFeaturesToTrack(frame_gray, maxCorners=1000, qualityLevel=0.01, minDistance=10)
        return points

    def sfm_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from Logitech camera.")
            return

        curr_frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Initialization step: grab first frame features
        if self.prev_frame is None or self.prev_points is None or len(self.prev_points) < 100:
            self.prev_frame = curr_frame_gray
            self.prev_points = self.detect_features(curr_frame_gray)
            return

        # 1. Feature Tracking using Lucas-Kanade Optical Flow
        curr_points, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_frame, curr_frame_gray, self.prev_points, None)

        # Keep only the features that were successfully tracked
        good_old = self.prev_points[status == 1]
        good_new = curr_points[status == 1]

        if len(good_old) < 10 or len(good_new) < 10:
            self.prev_points = None # Force re-initialization if we lose tracking
            return

        # 2. Estimate Camera Motion (Essential Matrix)
        E, mask = cv2.findEssentialMat(good_new, good_old, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None or E.shape != (3, 3):
            return

        # Recover Pose (Rotation and Translation from Essential Matrix)
        _, R, t, mask_pose = cv2.recoverPose(E, good_new, good_old, self.K)

        # 3. Triangulate 3D Points
        # Create Projection matrices
        P1 = np.hstack((np.eye(3, 3), np.zeros((3, 1)))) # Previous frame projection (origin)
        P2 = np.hstack((R, t))                           # Current frame projection
        
        P1 = self.K @ P1
        P2 = self.K @ P2

        # Triangulate
        points_4d = cv2.triangulatePoints(P1, P2, good_old.T, good_new.T)
        
        # Convert Homogeneous Coordinates (4D) to 3D by dividing by the 4th coordinate (w)
        points_3d = points_4d[:3, :] / points_4d[3, :]
        points_3d = points_3d.T

        # Filter points that are behind the camera (z < 0) or too far away
        valid_points = []
        for pt in points_3d:
            if pt[2] > 0.1 and pt[2] < 10.0:  # Z is depth; keep points between 0.1m and 10m
                valid_points.append([pt[0], pt[1], pt[2]])

        # 4. Publish as ROS 2 PointCloud2
        if len(valid_points) > 0:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "usb_cam"
            
            # Use ROS 2 helper to generate PointCloud2 message
            pc_msg = point_cloud2.create_cloud_xyz32(header, valid_points)
            self.pc_pub.publish(pc_msg)

        # Update previous frame and points for the next iteration
        self.prev_frame = curr_frame_gray
        self.prev_points = good_new.reshape(-1, 1, 2)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LogitechSFMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()