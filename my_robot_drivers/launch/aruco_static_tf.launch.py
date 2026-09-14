# import math
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
    
#     deg_to_rad = math.pi / 180.0

#     # -------------------------------------------------------------------------
#     # MARKER 0: West Wall (Middle) -> ID 0
#     # -------------------------------------------------------------------------
#     tf_marker_0 = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_marker_0',
#         arguments=[
#             '0.0', '1.5', '0.2',             # X, Y, Z (meters)
#             '0', '0', '0',                   # Yaw, Pitch, Roll (facing East)
#             'map', 'marker_0'                # Match your ArUco Detector child frame
#         ]
#     )

#     # -------------------------------------------------------------------------
#     # MARKER 1: North Wall (Left) -> ID 1
#     # -------------------------------------------------------------------------
#     tf_marker_1 = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_marker_1',
#         arguments=[
#             '0.5', '3.0', '0.2',
#             str(-90.0 * deg_to_rad), '0', '0', # Yaw -90 deg (facing South)
#             'map', 'marker_1'
#         ]
#     )

#     # -------------------------------------------------------------------------
#     # MARKER 2: North Wall (MIDDLE EXTRA TAG) -> ID 2
#     # -------------------------------------------------------------------------
#     tf_marker_2 = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_marker_2',
#         arguments=[
#             '2.0', '3.0', '0.2',             # Placed right in the middle (X=2.0m)
#             str(-90.0 * deg_to_rad), '0', '0',
#             'map', 'marker_2'
#         ]
#     )

#     # -------------------------------------------------------------------------
#     # MARKER 3: North Wall (Right) -> ID 3
#     # -------------------------------------------------------------------------
#     tf_marker_3 = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_marker_3',
#         arguments=[
#             '3.5', '3.0', '0.2',
#             str(-90.0 * deg_to_rad), '0', '0',
#             'map', 'marker_3'
#         ]
#     )

#     return LaunchDescription([
#         tf_marker_0,
#         tf_marker_1,
#         tf_marker_2,
#         tf_marker_3,
#     ])

import math
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    deg_to_rad = math.pi / 180.0

    # rotation guidelines
    # rotate around the object z axis (yaw), then rotate around the new y axis (pich), then rotate around the new x axis (roll)
    # the pitch and roll are made around the new axis not the first one 
    # -------------------------------------------------------------------------
    # MARKER 0: West Wall (X=0m, Y=1.5m, Z=0.2m)
    # Mounted vertically facing East (+X_map)
    # Pitch = +90 deg brings the Z-axis out from pointing UP to pointing EAST (+X)
    # -------------------------------------------------------------------------

    tf_marker_0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_marker_0',
        arguments=[
            '0.0', '0.0', '0.1275',
            str(-45.0*deg_to_rad), str(180.0 * deg_to_rad), str(-90.0 * deg_to_rad),       # Yaw, Pitch, Roll (rad)
            'map', 'marker_0'
        ]
    )

    tf_marker_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_marker_1',
        arguments=[
            '2.13', '0.0', '0.1275',
            str(45.0*deg_to_rad), str(180.0 * deg_to_rad), str(-90.0 * deg_to_rad),       # Yaw, Pitch, Roll (rad)
            'map', 'marker_1'
        ]
    )


    tf_marker_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_marker_2',
        arguments=[
            '2.13', '1.51', '0.1275',
            '0.0', '0.0', str(90.0 * deg_to_rad),       # Yaw, Pitch, Roll (rad)
            'map', 'marker_2'
        ]
    )

    return LaunchDescription([
        tf_marker_0,
        tf_marker_1,
        tf_marker_2,
    ])
