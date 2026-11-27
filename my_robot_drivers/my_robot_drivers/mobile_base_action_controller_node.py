#!/usr/bin/env python3
import math
import threading
import time
import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_interfaces.action import DriveBaseGoal


class MobileBaseController(Node):
    def __init__(self):
        super().__init__('mobile_base_controller')

        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('min_linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('min_angular_speed', 0.4)
        self.declare_parameter('waypoint_tolerance', 0.07)
        self.declare_parameter('yaw_tolerance', 0.13)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/filtered')

        self.v_max = float(self.get_parameter('max_linear_speed').value)
        self.v_min = float(self.get_parameter('min_linear_speed').value)
        self.w_max = float(self.get_parameter('max_angular_speed').value)
        self.w_min = float(self.get_parameter('min_angular_speed').value)
        self.wp_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)

        self.robot_pos = None
        self.robot_yaw = None
        self.e_stopped = False
        self._lock = threading.Lock()

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self._drive_server = ActionServer(
            self,
            DriveBaseGoal,
            '/mobile_base_controller/drive_base',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_drive_callback,
        )

        self.get_logger().info(
            f"MobileBaseController action server ready on /mobile_base_controller/drive_base "
            f"(odom: {odom_topic}, cmd_vel: {cmd_vel_topic})"
        )

    def odom_callback(self, msg: Odometry):
        with self._lock:
            self.robot_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=float)
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp_angular_velocity(self, w_calc: float, w_cap: float) -> float:
        if abs(w_calc) < 1e-5:
            return 0.0
        effective_max = max(self.w_min, w_cap)
        w_mag = float(np.clip(abs(w_calc), self.w_min, effective_max))
        return math.copysign(w_mag, w_calc)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def set_e_stop(self, active: bool):
        with self._lock:
            self.e_stopped = active
        if active:
            self.publish_stop()

    def _get_pose_snapshot(self):
        with self._lock:
            pos = None if self.robot_pos is None else self.robot_pos.copy()
            yaw = self.robot_yaw
            e_stop = self.e_stopped
        return pos, yaw, e_stop

    def _populate_result(self, result: DriveBaseGoal.Result, success: bool, message: str, pos, yaw):
        result.success = success
        result.message = message
        result.final_x = float(pos[0]) if pos is not None else 0.0
        result.final_y = float(pos[1]) if pos is not None else 0.0
        result.final_yaw = float(yaw) if yaw is not None else 0.0
        return result

    def goal_callback(self, goal_request):
        with self._lock:
            if self.e_stopped:
                self.get_logger().warn("Rejecting DriveBaseGoal: MobileBaseController is E-stopped.")
                return GoalResponse.REJECT
        if goal_request.mode not in (DriveBaseGoal.Goal.MODE_ROTATE_IN_PLACE, DriveBaseGoal.Goal.MODE_GO_TO):
            self.get_logger().warn(f"Rejecting DriveBaseGoal: unknown mode {goal_request.mode}.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested for DriveBaseGoal.")
        self.publish_stop()
        return CancelResponse.ACCEPT

    def execute_drive_callback(self, goal_handle) -> DriveBaseGoal.Result:
        req = goal_handle.request
        result = DriveBaseGoal.Result()
        pos, yaw, e_stop = self._get_pose_snapshot()

        if e_stop:
            self.publish_stop()
            goal_handle.abort()
            return self._populate_result(result, False, "Base controller is E-stopped.", pos, yaw)

        if req.mode == DriveBaseGoal.Goal.MODE_ROTATE_IN_PLACE:
            return self._run_rotate_in_place(goal_handle, float(req.angle_rad), float(req.max_speed), timeout_sec=20.0)
        if req.mode == DriveBaseGoal.Goal.MODE_GO_TO:
            return self._run_go_to(
                goal_handle, float(req.x), float(req.y), float(req.max_speed), timeout_sec=30.0
            )

        self.publish_stop()
        goal_handle.abort()
        return self._populate_result(result, False, f"Unsupported mode {req.mode}.", pos, yaw)

    def _run_go_to(self, goal_handle, x: float, y: float, max_speed_req: float, timeout_sec: float = 30.0) -> DriveBaseGoal.Result:
        result = DriveBaseGoal.Result()
        v_cap = max_speed_req if max_speed_req > 0.0 else self.v_max
        effective_v_max = max(self.v_min, v_cap)

        start_t = time.monotonic()
        rate_dt = 0.05

        while rclpy.ok() and (time.monotonic() - start_t) < timeout_sec:
            pos, yaw, e_stop = self._get_pose_snapshot()

            if goal_handle.is_cancel_requested:
                self.publish_stop()
                goal_handle.canceled()
                return self._populate_result(result, False, "MODE_GO_TO canceled.", pos, yaw)

            if e_stop:
                self.publish_stop()
                goal_handle.abort()
                return self._populate_result(result, False, "MODE_GO_TO aborted due to E-stop.", pos, yaw)

            if pos is None or yaw is None:
                time.sleep(rate_dt)
                continue

            dx = x - pos[0]
            dy = y - pos[1]
            dist = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            yaw_err = self.normalize_angle(target_yaw - yaw)

            if dist < self.wp_tolerance:
                self.publish_stop()
                goal_handle.succeed()
                return self._populate_result(result, True, f"Reached ({x:.3f}, {y:.3f}).", pos, yaw)

            cmd = Twist()
            cmd.linear.x = float(np.clip(0.5 * dist, self.v_min, effective_v_max))
            cmd.angular.z = self.clamp_angular_velocity(1.0 * yaw_err, self.w_max)
            self.cmd_pub.publish(cmd)

            feedback = DriveBaseGoal.Feedback()
            feedback.distance_remaining = float(dist)
            feedback.angle_remaining = float(yaw_err)
            goal_handle.publish_feedback(feedback)

            time.sleep(rate_dt)

        pos, yaw, _ = self._get_pose_snapshot()
        self.publish_stop()
        goal_handle.abort()
        return self._populate_result(result, False, f"MODE_GO_TO timed out reaching ({x:.3f}, {y:.3f}).", pos, yaw)

    def _run_rotate_in_place(
        self, goal_handle, target_yaw: float, max_speed_req: float, timeout_sec: float = 20.0
    ) -> DriveBaseGoal.Result:
        result = DriveBaseGoal.Result()
        w_cap = max_speed_req if max_speed_req > 0.0 else self.w_max

        start_t = time.monotonic()
        rate_dt = 0.05

        while rclpy.ok() and (time.monotonic() - start_t) < timeout_sec:
            pos, yaw, e_stop = self._get_pose_snapshot()

            if goal_handle.is_cancel_requested:
                self.publish_stop()
                goal_handle.canceled()
                return self._populate_result(result, False, "MODE_ROTATE_IN_PLACE canceled.", pos, yaw)

            if e_stop:
                self.publish_stop()
                goal_handle.abort()
                return self._populate_result(result, False, "MODE_ROTATE_IN_PLACE aborted due to E-stop.", pos, yaw)

            if yaw is None:
                time.sleep(rate_dt)
                continue

            yaw_err = self.normalize_angle(target_yaw - yaw)
            if abs(yaw_err) <= self.yaw_tolerance:
                self.publish_stop()
                goal_handle.succeed()
                return self._populate_result(result, True, f"Aligned to yaw {target_yaw:.3f} rad.", pos, yaw)

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.clamp_angular_velocity(1.0 * yaw_err, w_cap)
            self.cmd_pub.publish(cmd)

            feedback = DriveBaseGoal.Feedback()
            feedback.distance_remaining = 0.0
            feedback.angle_remaining = float(yaw_err)
            goal_handle.publish_feedback(feedback)

            time.sleep(rate_dt)

        pos, yaw, _ = self._get_pose_snapshot()
        self.publish_stop()
        goal_handle.abort()
        return self._populate_result(result, False, f"MODE_ROTATE_IN_PLACE timed out at yaw {target_yaw:.3f} rad.", pos, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = MobileBaseController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()