import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Locate package paths
    pkg_share = get_package_share_directory('my_robot_description')
    # default_xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    default_xacro_path = os.path.join(pkg_share, 'urdf', 'mobile_nanobot.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # 2. Declare launch arguments (allows overriding parameters from CLI)
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    xacro_file_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_xacro_path,
        description='Absolute path to robot xacro file'
    )

    # 3. Process Xacro file into raw URDF string
    xacro_file = LaunchConfiguration('model').perform
    # Fallback to direct path processing for local execution
    robot_description_config = xacro.process_file(default_xacro_path)
    robot_description_raw = robot_description_config.toxml()

    # 4. Node Definitions
    
    # Robot State Publisher Node (publishes TFs based on URDF and joint states)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Joint State Publisher GUI Node (provides sliders to manually rotate wheels/joints)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 Node
    # Automatically loads display.rviz if it exists, otherwise starts clean
    rviz_args = ['-d', default_rviz_config_path] if os.path.exists(default_rviz_config_path) else []
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    # 5. Return Launch Description
    return LaunchDescription([
        use_sim_time_arg,
        xacro_file_arg,
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])