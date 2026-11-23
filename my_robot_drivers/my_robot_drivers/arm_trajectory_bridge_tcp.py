#!/usr/bin/env python3
import time
import math
import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState


class TcpSender:
    """Reusable persistent TCP client with auto-reconnect and thread safety."""

    def __init__(self, logger, ip: str, port: int, name: str):
        self._logger = logger
        self.ip = ip
        self.port = int(port)
        self.name = name
        self._sock = None
        self._lock = threading.Lock()

    def connect(self) -> bool:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(2.0)
                s.connect((self.ip, self.port))
                self._sock = s
                self._logger.info(f"[{self.name}] Connected to ESP32 at {self.ip}:{self.port}")
                return True
            except Exception as e:
                self._sock = None
                self._logger.warn(
                    f"[{self.name}] Failed connecting to {self.ip}:{self.port}: {e}",
                    throttle_duration_sec=2.0
                )
                return False

    def send_line(self, line: str) -> bool:
        payload = line if line.endswith("\n") else f"{line}\n"
        encoded = payload.encode("utf-8")

        with self._lock:
            if self._sock is None:
                self._sock = self._create_connected_socket()

            if self._sock is None:
                return False

            try:
                self._sock.sendall(encoded)
                return True
            except (socket.error, socket.timeout) as e:
                self._logger.warn(f"[{self.name}] Send error: {e}. Reconnecting...", throttle_duration_sec=2.0)
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = self._create_connected_socket()
                if self._sock is not None:
                    try:
                        self._sock.sendall(encoded)
                        return True
                    except Exception:
                        pass
                return False

    def _create_connected_socket(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(2.0)
            s.connect((self.ip, self.port))
            self._logger.info(f"[{self.name}] Reconnected to ESP32 at {self.ip}:{self.port}")
            return s
        except Exception:
            return None

    def close(self):
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None


class RobotHardwareBridge(Node):
    def __init__(self):
        super().__init__('arm_trajectory_bridge')

        # 1. ESP32 TCP Server Parameters (Arm: 8890, Gripper: 8891)
        self.declare_parameter('esp32_ip', '192.168.1.13')
        self.declare_parameter('arm_tcp_port', 8890)
        self.declare_parameter('gripper_tcp_port', 8891)
        self.declare_parameter('interp_step_sec', 0.025)

        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.arm_tcp_port = int(self.get_parameter('arm_tcp_port').value)
        self.gripper_tcp_port = int(self.get_parameter('gripper_tcp_port').value)
        self.interp_step_sec = float(self.get_parameter('interp_step_sec').value)

        self.arm_tcp = TcpSender(self.get_logger(), self.esp32_ip, self.arm_tcp_port, "ARM")
        self.gripper_tcp = TcpSender(self.get_logger(), self.esp32_ip, self.gripper_tcp_port, "GRIPPER")
        self.arm_tcp.connect()
        self.gripper_tcp.connect()

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

        # 3. Publisher for RViz / state feedback
        self.full_joint_state_pub = self.create_publisher(JointState, '/arm_joint_states', 10)
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

        self.get_logger().info(
            f"Arm & Gripper TCP Bridge initialized! ESP32={self.esp32_ip} "
            f"(arm:{self.arm_tcp_port}, gripper:{self.gripper_tcp_port})"
        )

    def send_tcp_arm_cmd(self, rad_positions):
        """Sends 3 comma-delimited arm angles in degrees: 'd1,d2,d3\\n'."""
        if len(rad_positions) < 3:
            return
        deg_positions = [int(round(math.degrees(p))) for p in rad_positions[:3]]
        cmd = f"a,{deg_positions[0]},{deg_positions[1]},{deg_positions[2]}\n"
        self.arm_tcp.send_line(cmd)

    def send_tcp_gripper_cmd(self, rad_pos: float):
        """Sends 1 gripper angle in degrees: 'g1\\n'."""
        deg_val = int(round(math.degrees(rad_pos)))
        self.gripper_tcp.send_line(f"g,{deg_val}\n")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def publish_full_joint_states(self):
        msg = JointState()
        msg.name = self.arm_joints + self.hand_joints
        msg.position = list(self.current_arm_positions) + list(self.current_gripper_positions)
        self.full_joint_state_pub.publish(msg)

    def execute_arm_callback(self, goal_handle):
        self.get_logger().info('Executing arm trajectory via TCP Direct Bridge...')
        trajectory = goal_handle.request.trajectory

        if not trajectory.points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        joint_map = [
            self.arm_joints.index(name) if name in self.arm_joints else -1
            for name in trajectory.joint_names
        ]

        prev_time = 0.0
        prev_pos = list(self.current_arm_positions)

        for pt_idx, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_time = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            duration = max(0.0, target_time - prev_time)

            ordered_target_pos = list(prev_pos)
            for idx, pos_val in enumerate(point.positions):
                if joint_map[idx] != -1:
                    ordered_target_pos[joint_map[idx]] = pos_val

            if duration > 0.0 and self.interp_step_sec > 0.0:
                steps = max(1, int(math.ceil(duration / self.interp_step_sec)))
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

            deg_vals = [int(round(math.degrees(p))) for p in self.current_arm_positions[:3]]
            self.get_logger().info(
                f'[Waypoint {pt_idx + 1}/{len(trajectory.points)}] '
                f'3-Joint Arm Degrees Sent over TCP: {deg_vals}'
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
        self.get_logger().info('Executing gripper trajectory via TCP Bridge...')
        trajectory = goal_handle.request.trajectory

        if not trajectory.points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        joint_map = [
            self.hand_joints.index(name) if name in self.hand_joints else -1
            for name in trajectory.joint_names
        ]
        prev_time = 0.0

        for pt_idx, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_time = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            duration = max(0.0, target_time - prev_time)

            ordered_target_pos = list(self.current_gripper_positions)
            for idx, pos_val in enumerate(point.positions):
                if joint_map[idx] != -1:
                    ordered_target_pos[joint_map[idx]] = pos_val

            if duration > 0.0:
                time.sleep(duration)

            self.current_gripper_positions = ordered_target_pos

            # Send single gripper angle over port 8891
            self.send_tcp_gripper_cmd(float(ordered_target_pos[0]))

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = trajectory.joint_names
            feedback.actual.positions = list(point.positions)
            feedback.desired.positions = list(point.positions)
            goal_handle.publish_feedback(feedback)

            prev_time = target_time

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def destroy_node(self):
        self.arm_tcp.close()
        self.gripper_tcp.close()
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