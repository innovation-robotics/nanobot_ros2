#!/usr/bin/env python3
import math
import socket
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


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
                    throttle_duration_sec=2.0,
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
                self._logger.warn(
                    f"[{self.name}] Send error: {e}. Reconnecting...",
                    throttle_duration_sec=2.0,
                )
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


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.declare_parameter('esp32_ip', '192.168.1.50')
        self.declare_parameter('arm_port', 8890)
        self.declare_parameter('gripper_port', 8891)
        self.declare_parameter('interpolation_steps', 20)
        self.declare_parameter('step_delay_sec', 0.05)

        self.esp32_ip = str(self.get_parameter('esp32_ip').value)
        self.arm_port = int(self.get_parameter('arm_port').value)
        self.gripper_port = int(self.get_parameter('gripper_port').value)
        self.interp_steps = int(self.get_parameter('interpolation_steps').value)
        self.step_delay = float(self.get_parameter('step_delay_sec').value)

        self.arm_tcp = TcpSender(self.get_logger(), self.esp32_ip, self.arm_port, "ARM")
        self.gripper_tcp = TcpSender(self.get_logger(), self.esp32_ip, self.gripper_port, "GRIPPER")
        self.arm_tcp.connect()
        self.gripper_tcp.connect()

        # Current tracked joint angles (radians—converted to rounded degrees when sending over TCP)
        self.current_joints: List[float] = [0.0, 0.0, 0.0]
        self.e_stopped = False
        self._lock = threading.Lock()

        # Parameter-backed quick service hooks (easy to upgrade to custom .srv/.action later)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('gripper_angle', 0.0)

        self.move_xyz_srv = self.create_service(Trigger, '/arm_controller/move_to_xyz', self.handle_move_to_xyz)
        self.gripper_srv = self.create_service(Trigger, '/arm_controller/set_gripper', self.handle_set_gripper)

        self.get_logger().info(
            f"ArmController ready (ESP32={self.esp32_ip}, arm:{self.arm_port}, gripper:{self.gripper_port})"
        )

    # =========================================================================
    # INVERSE KINEMATICS DECLARATION (Fill in your own calculation later)
    # =========================================================================
    def solve_inverse_kinematics(self, x: float, y: float, z: float) -> Optional[List[float]]:
        """
        Compute target joint angles (in radians) for end-effector position (x, y, z) in link0 frame.

        Args:
            x: Target X coordinate in meters (link0 frame)
            y: Target Y coordinate in meters (link0 frame)
            z: Target Z coordinate in meters (link0 frame)

        Returns:
            List of joint angles in radians (e.g., [q1, q2, q3]), or None if unreachable.
        """
        raise NotImplementedError("Fill in solve_inverse_kinematics(x, y, z) with your IK solver.")

    # =========================================================================
    # TCP COMMAND FORMATTING
    # =========================================================================
    def send_arm_angles_command(self, rad_positions: List[float]) -> bool:
        """Sends 3 comma-delimited arm angles in degrees: 'a,d1,d2,d3\\n'."""
        if len(rad_positions) < 3:
            return False
        deg_positions = [int(round(math.degrees(p))) for p in rad_positions[:3]]
        cmd = f"a,{deg_positions[0]},{deg_positions[1]},{deg_positions[2]}\n"
        return self.arm_tcp.send_line(cmd)

    def send_gripper_angle_command(self, rad_pos: float) -> bool:
        """Sends 1 gripper angle in degrees: 'g,g1\\n'."""
        deg_val = int(round(math.degrees(rad_pos)))
        return self.gripper_tcp.send_line(f"g,{deg_val}\n")

    # =========================================================================
    # INTERPOLATION & EXECUTION
    # =========================================================================
    def interpolate_joint_trajectory(
        self, start_q: List[float], goal_q: List[float], steps: int
    ) -> List[List[float]]:
        if steps <= 1:
            return [list(goal_q)]
        traj = []
        for i in range(1, steps + 1):
            alpha = i / float(steps)
            waypoint = [s + alpha * (g - s) for s, g in zip(start_q, goal_q)]
            traj.append(waypoint)
        return traj

    def execute_joint_goal(self, goal_q: List[float]) -> Tuple[bool, str]:
        with self._lock:
            if self.e_stopped:
                return False, "Arm controller is E-stopped."
            start_q = list(self.current_joints)

        waypoints = self.interpolate_joint_trajectory(start_q, goal_q, self.interp_steps)
        for wp in waypoints:
            with self._lock:
                if self.e_stopped:
                    return False, "Arm trajectory aborted due to E-stop."
            if not self.send_arm_angles_command(wp):
                return False, "Failed sending arm waypoint over TCP."
            with self._lock:
                self.current_joints = list(wp)
            time.sleep(self.step_delay)

        return True, "Arm reached target joint angles."

    def execute_move_to_xyz(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        try:
            goal_q = self.solve_inverse_kinematics(x, y, z)
        except NotImplementedError as exc:
            return False, str(exc)

        if goal_q is None:
            return False, f"IK returned no solution for ({x:.3f}, {y:.3f}, {z:.3f})."

        return self.execute_joint_goal(goal_q)

    def execute_gripper(self, rad_angle: float) -> Tuple[bool, str]:
        with self._lock:
            if self.e_stopped:
                return False, "Gripper command blocked by E-stop."
        if not self.send_gripper_angle_command(rad_angle):
            return False, "Failed sending gripper command over TCP."
        return True, f"Gripper set to {rad_angle:.4f} rad."

    def set_e_stop(self, active: bool):
        with self._lock:
            self.e_stopped = active

    def handle_move_to_xyz(self, request, response):
        x = float(self.get_parameter('target_x').value)
        y = float(self.get_parameter('target_y').value)
        z = float(self.get_parameter('target_z').value)
        ok, msg = self.execute_move_to_xyz(x, y, z)
        response.success = ok
        response.message = msg
        return response

    def handle_set_gripper(self, request, response):
        angle = float(self.get_parameter('gripper_angle').value)
        ok, msg = self.execute_gripper(angle)
        response.success = ok
        response.message = msg
        return response

    def destroy_node(self):
        self.arm_tcp.close()
        self.gripper_tcp.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    executor = rclpy.executors.MultiThreadedExecutor()
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