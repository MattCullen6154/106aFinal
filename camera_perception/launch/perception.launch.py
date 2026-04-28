import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        Node(
            package='camera_perception',
            executable='yolo_perception_node',
            name='yolo_perception_node',
            output='screen',
            parameters=[{'camera_topic': '/image_raw'}]
        ),
        Node(
            package='camera_perception',
            executable='logitech_sfm_node',
            name='logitech_sfm_node',
            output='screen'
        )
    ])