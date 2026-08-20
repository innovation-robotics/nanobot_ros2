import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_drivers = get_package_share_directory('my_robot_drivers')
    params_file = os.path.join(pkg_drivers, 'config', 'aruco_params.yaml')

    # 1. ESP32-CAM Stream Publisher Node
    esp32_cam_node = Node(
        package='my_robot_drivers',
        executable='esp32_cam_node',
        name='esp32_cam_node',
        output='screen',
        parameters=[{
            'stream_url': 'http://192.168.1.5:81/stream', # Update port if needed
            'camera_frame': 'camera_optical_frame',
            'frame_rate': 15.0
        }]
    )

    # 2. Static Room Marker Transforms
    static_map_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drivers, 'launch', 'aruco_static_tf.launch.py')
        )
    )

    # # 3. ArUco Marker Detector Node
    # aruco_node = Node(
    #     package='aruco_ros',
    #     executable='marker_publisher',
    #     name='aruco_marker_publisher',
    #     output='screen',
    #     parameters=[params_file],
    #     remappings=[
    #         ('image', '/camera/image_raw'),
    #         ('camera_info', '/camera/camera_info')
    #     ]
    # )

    aruco_detector_node = Node(
        package='my_robot_drivers',         # Make sure this matches your package name
        executable='aruco_tf_node.py',      # Executable script name
        name='aruco_marker_publisher',
        output='screen',
        parameters=[{
            'marker_size': 0.08,
            'dictionary_id': 'DICT_4X4_250',
            'camera_frame': 'camera_optical_frame',
            'reference_frame': 'map',
        }],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/camera/camera_info', '/camera/camera_info'),
        ]
    )

    return LaunchDescription([
        esp32_cam_node,
        static_map_tf,
        aruco_detector_node,
    ])


