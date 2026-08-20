import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    description_pkg_share = get_package_share_directory('my_robot_description')
    
    robot_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg_share, 'launch', 'display.launch.py')
        )
    )

    # 1. Micro-ROS Agent Node
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888'],
        respawn=True,
        respawn_delay=2.0
    )

    # 2. Odometry Broker Node
    odom_node = Node(
        package='my_robot_drivers',
        executable='odom_broker_node',
        output='screen'
    )

    # # 3. LiDAR TCP Node
    # lidar_node = Node(
    #     package='my_robot_drivers',
    #     executable='lidar_tcp_node',
    #     output='screen',
    #     emulate_tty=True,
    #     # ADD THIS: Forces Python to flush I/O streams instantly in launch files
    #     additional_env={'PYTHONUNBUFFERED': '1'},
    #     respawn=True,
    #     respawn_delay=2.0
    # )

    # # Trigger LiDAR ONLY after micro_ros_agent process is up and running, plus a 5-second buffer
    # delay_lidar_after_agent_start = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=microros_agent,
    #         on_start=[
    #             TimerAction(
    #                 period=5.0,
    #                 actions=[lidar_node]
    #             )
    #         ]
    #     )
    # )

    return LaunchDescription([
        microros_agent,
        robot_display_launch,
        odom_node
        # delay_lidar_after_agent_start
    ])