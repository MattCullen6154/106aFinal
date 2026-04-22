import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line makes sure your launch files get installed
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Camera perception nodes for the 106A robot waiter project.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This is how you run the nodes: ros2 run camera_perception sfm_node
            'sfm_node = camera_perception.logitech_sfm_node:main',
            'yolo_node = camera_perception.yolo_perception_node:main',
        ],
    },
)