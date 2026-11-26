#!/usr/bin/env python3
import math
import socket
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import MoveArmToPoint

from my_robot_drivers.esp32_tcp_client import Esp32TcpClient


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

        self.arm_tcp = Esp32TcpClient(
            self.get_logger(), 
            self.esp32_ip, 
            self.arm_port, 
            "ARM")
        self.gripper_tcp = Esp32TcpClient(
            self.get_logger(), 
            self.esp32_ip, 
            self.gripper_port, 
            "GRIPPER")
        self.arm_tcp.connect()
        self.gripper_tcp.connect()

        # Current tracked joint angles (radians—converted to rounded degrees when sending over TCP)
        self.current_joints: List[float] = [0.0, 0.0, 0.0]
        self.e_stopped = False
        self._lock = threading.Lock()

        self._move_to_point_server = ActionServer(
            self,
            MoveArmToPoint,
            '/arm_controller/move_to_point',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_move_to_point_callback,
        )

        self.get_logger().info(
            f"ArmController action server ready on /arm_controller/move_to_point "
            f"(ESP32={self.esp32_ip}, arm:{self.arm_port}, gripper:{self.gripper_port})"
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
    # INTERPOLATION & ACTION EXECUTION
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

    def goal_callback(self, goal_request):
        with self._lock:
            if self.e_stopped:
                self.get_logger().warn("Rejecting MoveArmToPoint goal: ArmController is E-stopped.")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested for MoveArmToPoint goal.")
        return CancelResponse.ACCEPT

    def execute_move_to_point_callback(self, goal_handle) -> MoveArmToPoint.Result:
        req = goal_handle.request
        x = float(req.x)
        y = float(req.y)
        z = float(req.z)
        duration_sec = float(req.duration_sec)

        result = MoveArmToPoint.Result()

        with self._lock:
            if self.e_stopped:
                goal_handle.abort()
                result.success = False
                result.message = "Arm controller is E-stopped."
                return result
            start_q = list(self.current_joints)

        try:
            goal_q = self.solve_inverse_kinematics(x, y, z)
        except NotImplementedError as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

        if goal_q is None:
            goal_handle.abort()
            result.success = False
            result.message = f"IK returned no solution for ({x:.3f}, {y:.3f}, {z:.3f})."
            return result

        total_steps = max(1, self.interp_steps)
        step_dt = (duration_sec / total_steps) if duration_sec > 0.0 else self.step_delay
        waypoints = self.interpolate_joint_trajectory(start_q, goal_q, total_steps)

        for idx, wp in enumerate(waypoints, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = f"MoveArmToPoint canceled at step {idx}/{total_steps}."
                return result

            with self._lock:
                if self.e_stopped:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"MoveArmToPoint aborted due to E-stop at step {idx}/{total_steps}."
                    return result

            if not self.send_arm_angles_command(wp):
                goal_handle.abort()
                result.success = False
                result.message = f"Failed sending arm waypoint {idx}/{total_steps} over TCP."
                return result

            with self._lock:
                self.current_joints = list(wp)

            feedback = MoveArmToPoint.Feedback()
            feedback.progress = float(idx) / float(total_steps)
            feedback.current_step = idx
            feedback.total_steps = total_steps
            goal_handle.publish_feedback(feedback)

            time.sleep(step_dt)

        goal_handle.succeed()
        result.success = True
        result.message = f"Reached target point ({x:.3f}, {y:.3f}, {z:.3f})."
        return result

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
    