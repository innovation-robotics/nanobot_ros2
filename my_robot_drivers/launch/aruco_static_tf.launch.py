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
            '0.76', '0.0', '0.1275',
            str(45.0*deg_to_rad), str(180.0 * deg_to_rad), str(-90.0 * deg_to_rad),       # Yaw, Pitch, Roll (rad)
            'map', 'marker_1'
        ]
    )


    tf_marker_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_marker_2',
        arguments=[
            '0.76', '1.12', '0.1275',
            '0.0', '0.0', str(90.0 * deg_to_rad),       # Yaw, Pitch, Roll (rad)
            'map', 'marker_2'
        ]
    )

    return LaunchDescription([
        tf_marker_0,
        tf_marker_1,
        tf_marker_2,
    ])
