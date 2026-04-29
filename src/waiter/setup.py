from setuptools import find_packages, setup

package_name = 'waiter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ee106a-abn',
    maintainer_email='somil_03@berkeley.edu',
    description='Core waiter state machine',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "waiter_executive = waiter.waiter_executive:main",
            "kitchen_node = waiter.kitchen_node:main",
        ],
    },
)
