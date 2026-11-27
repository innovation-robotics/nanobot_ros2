#!/usr/bin/env python3
import time
import math
import socket
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class RobotHardwareBridge(Node):
    def __init__(self):
        super().__init__('arm_trajectory_bridge')

        # Optional: make IP/port ROS parameters while keeping your defaults
        self.declare_parameter('esp32_ip', '192.168.1.13')
        self.declare_parameter('esp32_port', 8890)
        self.declare_parameter('interp_step_sec', 0.025)  # ~40 Hz sub-waypoint pacing

        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = int(self.get_parameter('esp32_port').value)
        self.interp_step_sec = float(self.get_parameter('interp_step_sec').value)


        # Connect to ESP32 TCP Server
        self.connect_to_esp32()

        # 2. Arm & Hand Joint Names (Matches URDF and MoveIt YAML)
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'wrist_joint']
        self.hand_joints = [
            'robot_finger_joint1',
            'Tip_Gripper_Servo_Joint',
            'robot_finger_joint2',
            'Tip_Gripper_Idol_Joint'
        ]
        
        self.current_arm_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_gripper_positions = [0.0, 0.0, 0.0, 0.0]

        # 3. Publishers
        self.gripper_cmd_pub = self.create_publisher(Float32, '/gripper_cmd', 10)
        self.full_joint_state_pub = self.create_publisher(JointState, '/arm_joint_states', 10)

        # High-frequency Timer (50 Hz) for smooth state feedback in RViz
        self.create_timer(1.0 / 50.0, self.publish_full_joint_states)

        # 4. Action Servers
        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_arm_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/hand_controller/follow_joint_trajectory',
            execute_callback=self.execute_gripper_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(f'Arm Trajectory TCP Bridge initialized! Connected to ESP32 at {self.esp32_ip}:{self.esp32_port}')

    def connect_to_esp32(self):
        """Establishes or reconnects a persistent TCP socket to the ESP32."""
        try:
            if self.tcp_client:
                self.tcp_client.close()

            self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Disable Nagle algorithm to minimize transmission latency
            self.tcp_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.tcp_client.settimeout(2.0)
            self.tcp_client.connect((self.esp32_ip, self.esp32_port))
            self.get_logger().info("Successfully connected to ESP32 Arm TCP Server.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ESP32 TCP Server at {self.esp32_ip}:{self.esp32_port} - {e}")
            self.tcp_client = None

    def send_tcp_arm_cmd(self, rad_positions):
        """Converts joint positions (radians) to degrees and sends TCP frame '<d1,d2,d3,d4,d5>'."""
        if len(rad_positions) < 5:
            return

        # Convert radians to integer degrees
        deg_positions = [int(math.degrees(p)) for p in rad_positions[:5]]
        
        # Format payload expected by ESP32/Uno: <d1,d2,d3,d4,d5>
        formatted_cmd = f"<{','.join(map(str, deg_positions))}>\n"

        if self.tcp_client is None:
            self.connect_to_esp32()

        if self.tcp_client:
            try:
                self.tcp_client.sendall(formatted_cmd.encode('utf-8'))
            except (socket.error, socket.timeout) as e:
                self.get_logger().warn(f"TCP socket error: {e}. Attempting reconnect...")
                self.connect_to_esp32()

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def publish_full_joint_states(self):
        msg = JointState()
        msg.name = self.arm_joints + self.hand_joints
        msg.position = self.current_arm_positions + self.current_gripper_positions
        self.full_joint_state_pub.publish(msg)

    def execute_arm_callback(self, goal_handle):
        self.get_logger().info('Executing arm trajectory via TCP Direct Bridge...')
        trajectory = goal_handle.request.trajectory

        if not trajectory.points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        # Map incoming MoveIt joint order to bridge expected joint order
        joint_map = [
            self.arm_joints.index(name) if name in self.arm_joints else -1
            for name in trajectory.joint_names
        ]

        interp_step_sec = float(getattr(self, 'interp_step_sec', 0.025))
        prev_time = 0.0
        prev_pos = list(self.current_arm_positions)

        for pt_idx, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_time = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            duration = max(0.0, target_time - prev_time)

            # Reconstruct ordered target array for this waypoint
            ordered_target_pos = list(prev_pos)
            for idx, pos_val in enumerate(point.positions):
                if joint_map[idx] != -1:
                    ordered_target_pos[joint_map[idx]] = pos_val

            if duration > 0.0 and interp_step_sec > 0.0:
                steps = max(1, int(math.ceil(duration / interp_step_sec)))
                step_dt = duration / steps
                for s in range(1, steps + 1):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return FollowJointTrajectory.Result()

                    alpha = s / steps
                    interp_pos = [
                        prev_pos[j] + alpha * (ordered_target_pos[j] - prev_pos[j])
                        for j in range(len(ordered_target_pos))
                    ]
                    self.current_arm_positions = interp_pos
                    self.send_tcp_arm_cmd(interp_pos)
                    time.sleep(step_dt)
            else:
                if duration > 0.0:
                    time.sleep(duration)
                self.current_arm_positions = list(ordered_target_pos)
                self.send_tcp_arm_cmd(self.current_arm_positions)

            prev_pos = list(self.current_arm_positions)
            prev_time = target_time

            # Debug logging per waypoint
            deg_vals = [int(math.degrees(p)) for p in self.current_arm_positions[:5]]
            self.get_logger().info(
                f'[Waypoint {pt_idx + 1}/{len(trajectory.points)}] '
                f'Degrees Sent over TCP: {deg_vals}'
            )

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self.arm_joints
            feedback.actual.positions = list(self.current_arm_positions)
            feedback.desired.positions = list(ordered_target_pos)
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def execute_gripper_trajectory_callback(self, goal_handle):
        self.get_logger().info('Executing gripper trajectory...')
        trajectory = goal_handle.request.trajectory

        if not trajectory.points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        joint_map = [self.hand_joints.index(name) for name in trajectory.joint_names]
        prev_time = 0.0

        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_time = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            duration = target_time - prev_time

            ordered_target_pos = list(self.current_gripper_positions)
            for idx, pos_val in enumerate(point.positions):
                ordered_target_pos[joint_map[idx]] = pos_val

            if duration > 0:
                time.sleep(duration)

            self.current_gripper_positions = ordered_target_pos

            msg = Float32()
            msg.data = float(ordered_target_pos[0])
            self.gripper_cmd_pub.publish(msg)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = trajectory.joint_names
            feedback.actual.positions = list(point.positions)
            feedback.desired.positions = list(point.positions)
            goal_handle.publish_feedback(feedback)

            prev_time = target_time

        goal_handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return res

    def destroy_node(self):
        if self.tcp_client:
            self.tcp_client.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotHardwareBridge()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()