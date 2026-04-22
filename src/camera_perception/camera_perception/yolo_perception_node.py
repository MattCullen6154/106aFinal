import rclpy    # ROS 2 Python client library
from rclpy.node import Node # Base class for ROS 2 nodes
import cv2  # OpenCV for image processing
from cv_bridge import CvBridge, CvBridgeError   # Bridge to convert ROS Image messages to OpenCV images
from sensor_msgs.msg import Image   # ROS message type for images
from camera_perception_msgs.msg import ObstacleState    # Custom message type for publishing obstacle state
from ultralytics import YOLO    # The library for YOLOv8 object detection model
class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')
        
        # Declare ROS 2 parameters
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('slow_threshold', 40000.0)
        self.declare_parameter('stop_threshold', 80000.0)

        # Retrieve parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.slow_threshold = self.get_parameter('slow_threshold').value
        self.stop_threshold = self.get_parameter('stop_threshold').value
        
        # Load lightweight YOLOv8 nano model
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()    # Initialize CvBridge for image conversion
        
        # Classes to watch out for (0 - person, 24 - backpack, 25 - umbrella, 26 - handbag
        # 32 - sports ball, 39 - bottle, 56 - chair, 60 - dining table, 63 - laptop.
        self.target_classes = [0, 24, 25, 26, 32, 39, 56, 60, 63]
        
        # Publishers and Subscribers
        self.state_pub = self.create_publisher(ObstacleState, '/perception/obstacle_state', 10)
        
        # Use QoS depth of 10 for standard sensor data
        self.image_sub = self.create_subscription(
            Image, 
            self.camera_topic, 
            self.image_callback, 
            10
        )
        
        self.get_logger().info(f"YOLO Perception Node Initialized. Listening to: {self.camera_topic}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Run YOLO inference
        results = self.model.predict(cv_image, classes=self.target_classes, verbose=False)
        
        # Initialize default state
        state_msg = ObstacleState()
        state_msg.header.stamp = self.get_clock().now().to_msg() # ROS 2 clock
        state_msg.obstacle_detected = False
        state_msg.action = "CLEAR"
        state_msg.obstacle_class = "None"
        state_msg.bounding_area = 0.0

        largest_area = 0.0
        detected_class = "None"

        # Parse results
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                area = (x2 - x1) * (y2 - y1)
                
                if area > largest_area:
                    largest_area = float(area)
                    class_id = int(box.cls[0].item())
                    detected_class = self.model.names[class_id]

        # Determine action
        if largest_area > 0:
            state_msg.obstacle_detected = True
            state_msg.obstacle_class = detected_class
            state_msg.bounding_area = largest_area

            if largest_area >= self.stop_threshold:
                state_msg.action = "STOP"
            elif largest_area >= self.slow_threshold:
                state_msg.action = "SLOW"
            else:
                state_msg.action = "CLEAR"

        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()