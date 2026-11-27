#!/usr/bin/env python3
import math
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


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
        self.cancel_active_goal = False
        self._lock = threading.Lock()

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Trigger-based services (pass target coordinates/angle via ROS parameters or dedicated msgs/actions later)
        self.go_to_srv = self.create_service(Trigger, '/base_controller/go_to', self.handle_go_to)
        self.rotate_srv = self.create_service(Trigger, '/base_controller/rotate_in_place', self.handle_rotate_in_place)
        self.stop_srv = self.create_service(Trigger, '/base_controller/stop', self.handle_stop)

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)

        self.get_logger().info(f"MobileBaseController ready (odom: {odom_topic}, cmd_vel: {cmd_vel_topic})")

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

    def clamp_angular_velocity(self, w_calc: float) -> float:
        if abs(w_calc) < 1e-5:
            return 0.0
        w_mag = float(np.clip(abs(w_calc), self.w_min, self.w_max))
        return math.copysign(w_mag, w_calc)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def set_e_stop(self, active: bool):
        with self._lock:
            self.e_stopped = active
            if active:
                self.cancel_active_goal = True
        if active:
            self.publish_stop()

    def handle_stop(self, request, response):
        with self._lock:
            self.cancel_active_goal = True
        self.publish_stop()
        response.success = True
        response.message = "Base stopped."
        return response

    def handle_go_to(self, request, response):
        target_x = float(self.get_parameter('goal_x').value)
        target_y = float(self.get_parameter('goal_y').value)
        ok, msg = self.execute_go_to(target_x, target_y)
        response.success = ok
        response.message = msg
        return response

    def handle_rotate_in_place(self, request, response):
        target_yaw = float(self.get_parameter('goal_yaw').value)
        ok, msg = self.execute_rotate_in_place(target_yaw)
        response.success = ok
        response.message = msg
        return response

    def execute_go_to(self, x: float, y: float, timeout_sec: float = 30.0):
        with self._lock:
            self.cancel_active_goal = False

        start_t = time.monotonic()
        rate_dt = 0.05

        while rclpy.ok() and (time.monotonic() - start_t) < timeout_sec:
            with self._lock:
                if self.e_stopped or self.cancel_active_goal:
                    self.publish_stop()
                    return False, "GO_TO interrupted/stopped."
                pos = None if self.robot_pos is None else self.robot_pos.copy()
                yaw = self.robot_yaw

            if pos is None or yaw is None:
                time.sleep(rate_dt)
                continue

            dx = x - pos[0]
            dy = y - pos[1]
            dist = math.hypot(dx, dy)

            if dist < self.wp_tolerance:
                self.publish_stop()
                return True, f"Reached ({x:.3f}, {y:.3f})."

            target_yaw = math.atan2(dy, dx)
            yaw_err = self.normalize_angle(target_yaw - yaw)

            cmd = Twist()
            cmd.linear.x = float(np.clip(0.5 * dist, self.v_min, self.v_max))
            cmd.angular.z = self.clamp_angular_velocity(1.0 * yaw_err)
            self.cmd_pub.publish(cmd)
            time.sleep(rate_dt)

        self.publish_stop()
        return False, f"GO_TO timed out reaching ({x:.3f}, {y:.3f})."

    def execute_rotate_in_place(self, target_yaw: float, timeout_sec: float = 20.0):
        with self._lock:
            self.cancel_active_goal = False

        start_t = time.monotonic()
        rate_dt = 0.05

        while rclpy.ok() and (time.monotonic() - start_t) < timeout_sec:
            with self._lock:
                if self.e_stopped or self.cancel_active_goal:
                    self.publish_stop()
                    return False, "ROTATE_IN_PLACE interrupted/stopped."
                yaw = self.robot_yaw

            if yaw is None:
                time.sleep(rate_dt)
                continue

            yaw_err = self.normalize_angle(target_yaw - yaw)
            if abs(yaw_err) <= self.yaw_tolerance:
                self.publish_stop()
                return True, f"Aligned to yaw {target_yaw:.3f} rad."

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.clamp_angular_velocity(1.0 * yaw_err)
            self.cmd_pub.publish(cmd)
            time.sleep(rate_dt)

        self.publish_stop()
        return False, f"ROTATE_IN_PLACE timed out at yaw {target_yaw:.3f} rad."


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