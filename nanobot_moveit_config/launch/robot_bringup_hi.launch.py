import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _load_robot_description() -> str:
    # Adjust package/filename if your URDF or xacro is stored elsewhere
    for pkg_name, rel_paths in [
        ('my_robot_description', ['urdf/mobile_nanobot.xacro', 'urdf/mobile_nanobot.urdf']),
        ('my_robot_drivers', ['urdf/mobile_nanobot.xacro', 'urdf/mobile_nanobot.urdf']),
    ]:
        try:
            pkg_dir = get_package_share_directory(pkg_name)
        except Exception:
            continue
        for rel in rel_paths:
            full_path = os.path.join(pkg_dir, rel)
            if os.path.isfile(full_path):
                if full_path.endswith('.xacro'):
                    return subprocess.check_output(['xacro', full_path], text=True)
                with open(full_path, 'r', encoding='utf-8') as f:
                    return f.read()
    raise FileNotFoundError(
        "Could not locate URDF/xacro file; update _load_robot_description() with your robot description path."
    )


def generate_launch_description():
    pkg_drivers = get_package_share_directory('my_robot_drivers')
    ekf_config_path = os.path.join(pkg_drivers, 'config', 'ekf_fusion.yaml')
    robot_description_xml = _load_robot_description()

    # Connects the map origin directly to the odom origin
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'map', 'odom'
        ]
    )

    # Mobile base hardware interface (publishes odometry and wheel joint states; EKF handles TF)
    mobile_base_hardware_interface_node = Node(
        package='my_robot_drivers',
        executable='mobile_base_hardware_interface_node',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }]
    )

    # Base and arm action controllers
    arm_action_controller_hi_node = Node(
        package='my_robot_drivers',
        executable='arm_action_controller_hi_node',
        name='arm_action_controller_hi_node',
        output='screen',
        parameters=[{
            'arm_joint_states_topic': '/arm_joint_states',
            'joint_state_rate_hz': 30.0,
        }]
    )

    mobile_base_action_controller_node = Node(
        package='my_robot_drivers',
        executable='mobile_base_action_controller_node',
        name='mobile_base_action_controller_node',
        output='screen'
    )

    # Joint State Publisher (merges /wheel_joint_states and /arm_joint_states into /joint_states)
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

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}]
    )

    # Robot Localization (EKF Node)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # RViz2 Visualizer
    rviz_config_path = os.path.join(get_package_share_directory("nanobot_moveit_config"), 'config', 'no_moveit.rviz')
    rviz_args = ['-d', rviz_config_path] if os.path.isfile(rviz_config_path) else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}],
        arguments=rviz_args
    )

    # ArUco Detection & Room Map TFs
    aruco_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drivers, 'launch', 'aruco_detection.launch.py')
        )
    )

    return LaunchDescription([
        tf_map_to_odom,
        mobile_base_hardware_interface_node,
        mobile_base_action_controller_node,
        arm_action_controller_hi_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        ekf_node,
        rviz_node,
        aruco_detection_launch,
    ])