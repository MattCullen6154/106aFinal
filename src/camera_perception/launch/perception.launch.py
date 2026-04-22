import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Standard ROS 2 USB camera node (requires standard 'usb_cam' or 'v4l2_camera' package)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'usb_cam'
            }]
        ),
        
        # Our custom YOLO perception node
        Node(
            package='camera_perception',
            executable='yolo_perception_node',
            name='yolo_perception_node',
            output='screen',
            parameters=[{
                'camera_topic': '/image_raw',
                'slow_threshold': 40000.0,
                'stop_threshold': 90000.0
            }]
        )
    ])