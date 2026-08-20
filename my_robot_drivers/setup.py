import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line ensures your launch files are copied during colcon build
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # Install all config files (.yaml AND .npz)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='a.waly1980b@gmail.com',
    description='Driver nodes for micro-ROS ESP32 differential drive robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_tcp_node = my_robot_drivers.lidar_tcp_node:main',
            'odom_broker_node = my_robot_drivers.odom_broker:main',
            'arm_trajectory_bridge = my_robot_drivers.arm_trajectory_bridge:main',
            'esp32_cam_node = my_robot_drivers.esp32_cam_node:main',
            'aruco_tf_node.py = my_robot_drivers.aruco_tf_node:main',
        ],
    },
)