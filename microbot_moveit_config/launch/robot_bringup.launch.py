import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_drivers = get_package_share_directory('my_robot_drivers')

    # 0. Micro-ROS Agent Node
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888'],
        respawn=True,
        respawn_delay=2.0
    )

    # Connects the map origin directly to the odom origin
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=[
            '0.0', '0.0', '0.0', # X, Y, Z
            '0.0', '0.0', '0.0', # Yaw, Pitch, Roll
            'map', 'odom'        # Parent -> Child
        ]
    )

    # 1. MoveIt Configurations
    moveit_config = MoveItConfigsBuilder("mobile_microbot", package_name="microbot_moveit_config").to_moveit_configs()

    # 2. Odometry Broker Node (Publishes odom -> base_footprint)
    odom_broker_node = Node(
        package="my_robot_drivers",
        executable="odom_broker_node",
        output="screen",
        parameters=[{
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }]
    )

    # 3. Micro-ROS Trajectory Bridge
    arm_trajectory_bridge_node = Node(
        package='my_robot_drivers',
        executable='arm_trajectory_bridge',
        name='arm_trajectory_bridge',
        output='screen'
    )

    # 4. Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/wheel_joint_states', '/arm_joint_states'],
            'rate': 30.0
        }],
        output='screen'
    )

    # 5. Robot State Publisher (Publishes base_link -> camera_link -> camera_optical_frame)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description]
    )

    # 6. MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()]
    )

    # 7. RViz2 Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[moveit_config.to_dict()],
        arguments=['-d', os.path.join(get_package_share_directory("microbot_moveit_config"), "config", "moveit.rviz")]
    )

    # 8. Include ArUco Detection & Room Map TFs
    aruco_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drivers, 'launch', 'aruco_detection.launch.py')
        )
    )

    return LaunchDescription([
        microros_agent,
        tf_map_to_odom,
        odom_broker_node,
        arm_trajectory_bridge_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        aruco_detection_launch  # <--- Integrated here!
    ])