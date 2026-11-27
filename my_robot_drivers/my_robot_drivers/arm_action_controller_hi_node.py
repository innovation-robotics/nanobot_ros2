#!/usr/bin/env python3
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from robot_interfaces.action import MoveArmToPoint, MoveArmToPose

from my_robot_drivers.esp32_tcp_client import Esp32TcpClient


ARM_JOINT_ORDER = ["joint1", "joint2", "joint3"]


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.declare_parameter('esp32_ip', '192.168.1.13')
        self.declare_parameter('arm_port', 8890)
        self.declare_parameter('gripper_port', 8891)
        self.declare_parameter('gripper_topic', '/arm_controller/gripper_angle')
        self.declare_parameter('arm_joint_states_topic', '/arm_joint_states')
        self.declare_parameter('joint_state_rate_hz', 30.0)
        self.declare_parameter('interpolation_steps', 20)
        self.declare_parameter('step_delay_sec', 0.05)

        self.esp32_ip = str(self.get_parameter('esp32_ip').value)
        self.arm_port = int(self.get_parameter('arm_port').value)
        self.gripper_port = int(self.get_parameter('gripper_port').value)
        gripper_topic = str(self.get_parameter('gripper_topic').value)
        arm_joint_states_topic = str(self.get_parameter('arm_joint_states_topic').value)
        joint_state_rate_hz = float(self.get_parameter('joint_state_rate_hz').value)
        self.interp_steps = int(self.get_parameter('interpolation_steps').value)
        self.step_delay = float(self.get_parameter('step_delay_sec').value)

        # Named joint targets in radians ordered as [joint1, joint2, joint3]
        self.srdf_arm_states: Dict[str, List[float]] = {
            "arm_ready": [0.0, 0.0, 0.0],
            "arm_pick_place": [0.0, -1.466076572, -1.850049007],
            "arm_left": [0.0, 0.0, 0.0],
        }

        self.arm_tcp = Esp32TcpClient(
            self.get_logger(),
            self.esp32_ip,
            self.arm_port,
            "ARM",
        )
        self.gripper_tcp = Esp32TcpClient(
            self.get_logger(),
            self.esp32_ip,
            self.gripper_port,
            "GRIPPER",
        )
        self.arm_tcp.connect()
        self.gripper_tcp.connect()

        # Current tracked joint angles (radians—converted to rounded degrees when sending over TCP)
        self.current_joints: List[float] = [0.0, 0.0, 0.0]
        self.e_stopped = False
        self._lock = threading.Lock()

        # Arm joint states publisher for RViz / joint_state_publisher
        self.arm_joint_pub = self.create_publisher(JointState, arm_joint_states_topic, 10)
        if joint_state_rate_hz > 0.0:
            self.joint_state_timer = self.create_timer(1.0 / joint_state_rate_hz, self.publish_arm_joint_state)

        # Gripper topic subscriber
        self.gripper_sub = self.create_subscription(
            Float32,
            gripper_topic,
            self._gripper_angle_callback,
            10,
        )

        self._move_to_point_server = ActionServer(
            self,
            MoveArmToPoint,
            '/arm_controller/move_to_point',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_move_to_point_callback,
        )

        self._move_to_pose_server = ActionServer(
            self,
            MoveArmToPose,
            '/arm_controller/move_to_pose',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_move_to_pose_callback,
        )

        self.publish_arm_joint_state()

        self.get_logger().info(
            f"ArmController ready on /arm_controller/move_to_point, /arm_controller/move_to_pose, "
            f"topic {gripper_topic}, and joint states {arm_joint_states_topic} "
            f"(ESP32={self.esp32_ip}, arm:{self.arm_port}, gripper:{self.gripper_port})"
        )

    def publish_arm_joint_state(self):
        with self._lock:
            positions = list(self.current_joints)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_ORDER)
        msg.position = positions
        self.arm_joint_pub.publish(msg)

    # =========================================================================
    # INVERSE KINEMATICS & POSE RESOLUTION
    # =========================================================================
    def solve_inverse_kinematics(self, x: float, y: float, z: float) -> Optional[List[float]]:
        """
        Compute target joint angles (in radians) for end-effector position (x, y, z) in link0 frame.
        Returns [a0, a1, a2] in radians, or None if out of reach.
        """
        self.get_logger().info(f"x:{x*100.0:.1f}\ty:{y*100.0:.1f}\tz:{z*100.0:.1f}")
        
        L1 = 0.165188
        L2 = 0.21
        tol = 1e-6

        f = 0.01
        l = math.sqrt(x**2 + y**2) + f
        m = l - 0.0405055
        n = 0.133875 - z
        h = math.sqrt(m**2 + n**2)

        max_reach = L1 + L2 + tol
        min_reach = 0.1  # max(1e-9, abs(L1 - L2) - tol)

        if h > max_reach or h < min_reach:
            return None

        cos_A = max(-1.0, min(1.0, (h**2 + L1**2 - L2**2) / (2.0 * h * L1)))
        cos_C = max(-1.0, min(1.0, (L2**2 + h**2 - L1**2) / (2.0 * L2 * h)))

        A = math.acos(cos_A)
        C = math.acos(cos_C)
        O = math.atan2(m, n)

        a0 = -1.0 * math.atan2(x,y)
        a1 = -1.0 * (math.pi - O - A)
        S = math.pi - C - A
        a2 = -1.0 * (math.pi - S)

        return [a0, a1, a2]

    def _resolve_pose_goal(self, req, fallback_q: List[float]) -> Tuple[Optional[List[float]], str]:
        pose_name = getattr(req, "pose_name", "").strip()
        if pose_name:
            if pose_name in self.srdf_arm_states:
                return list(self.srdf_arm_states[pose_name]), f"pose '{pose_name}'"
            return None, f"Unknown pose_name '{pose_name}'. Available: {list(self.srdf_arm_states.keys())}"

        joint_names = list(getattr(req, "joint_names", []))
        joint_positions = [float(v) for v in getattr(req, "joint_positions", [])]
        if joint_names and len(joint_names) == len(joint_positions):
            resolved = list(fallback_q)
            name_to_val = dict(zip(joint_names, joint_positions))
            for idx, j_name in enumerate(ARM_JOINT_ORDER):
                if j_name in name_to_val:
                    resolved[idx] = name_to_val[j_name]
            return resolved, "custom joint target"

        return None, "Provide either a valid 'pose_name' or matching 'joint_names' and 'joint_positions'."

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
                self.get_logger().warn("Rejecting arm goal: ArmController is E-stopped.")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested for arm goal.")
        return CancelResponse.ACCEPT

    def _follow_joint_goal(
        self,
        goal_handle,
        goal_q: List[float],
        duration_sec: float,
        feedback_cls,
        result_cls,
        action_label: str,
        success_detail: str,
    ):
        result = result_cls()

        with self._lock:
            if self.e_stopped:
                goal_handle.abort()
                result.success = False
                result.message = "Arm controller is E-stopped."
                return result
            start_q = list(self.current_joints)

        total_steps = max(1, self.interp_steps)
        step_dt = (duration_sec / total_steps) if duration_sec > 0.0 else self.step_delay
        waypoints = self.interpolate_joint_trajectory(start_q, goal_q, total_steps)

        for idx, wp in enumerate(waypoints, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = f"{action_label} canceled at step {idx}/{total_steps}."
                return result

            with self._lock:
                if self.e_stopped:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"{action_label} aborted due to E-stop at step {idx}/{total_steps}."
                    return result

            if not self.send_arm_angles_command(wp):
                goal_handle.abort()
                result.success = False
                result.message = f"Failed sending arm waypoint {idx}/{total_steps} over TCP."
                return result

            with self._lock:
                self.current_joints = list(wp)
            self.publish_arm_joint_state()

            feedback = feedback_cls()
            feedback.progress = float(idx) / float(total_steps)
            feedback.current_step = idx
            feedback.total_steps = total_steps
            goal_handle.publish_feedback(feedback)

            time.sleep(step_dt)

        goal_handle.succeed()
        result.success = True
        result.message = f"Reached {success_detail}."
        return result

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

        return self._follow_joint_goal(
            goal_handle=goal_handle,
            goal_q=goal_q,
            duration_sec=duration_sec,
            feedback_cls=MoveArmToPoint.Feedback,
            result_cls=MoveArmToPoint.Result,
            action_label="MoveArmToPoint",
            success_detail=f"target point ({x:.3f}, {y:.3f}, {z:.3f})",
        )

    def execute_move_to_pose_callback(self, goal_handle) -> MoveArmToPose.Result:
        req = goal_handle.request
        duration_sec = float(getattr(req, "duration_sec", 0.0))
        result = MoveArmToPose.Result()

        with self._lock:
            if self.e_stopped:
                goal_handle.abort()
                result.success = False
                result.message = "Arm controller is E-stopped."
                return result
            current_q = list(self.current_joints)

        goal_q, desc = self._resolve_pose_goal(req, current_q)
        if goal_q is None:
            goal_handle.abort()
            result.success = False
            result.message = desc
            return result

        return self._follow_joint_goal(
            goal_handle=goal_handle,
            goal_q=goal_q,
            duration_sec=duration_sec,
            feedback_cls=MoveArmToPose.Feedback,
            result_cls=MoveArmToPose.Result,
            action_label="MoveArmToPose",
            success_detail=desc,
        )

    def _gripper_angle_callback(self, msg: Float32):
        ok, info = self.execute_gripper(float(msg.data))
        if not ok:
            self.get_logger().warn(info)

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