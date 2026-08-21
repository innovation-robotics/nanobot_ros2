#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import sys
import select
import termios
import tty
import threading
from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional, List

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

# ==========================================
# 1. TASK DATA STRUCTURE DEFINITIONS
# ==========================================

class TaskType(Enum):
    GO_TO = auto()
    ROTATE_IN_PLACE = auto()
    PICK = auto()
    PLACE = auto()

@dataclass
class RobotTask:
    type: TaskType
    target_pos: Optional[np.ndarray] = None  # [x, y] for GO_TO
    target_yaw: Optional[float] = None       # angle in radians for ROTATE_IN_PLACE
    item_id: Optional[str] = None            # Useful for PICK and PLACE operations
    duration: float = 2.0                    # Timer duration for mock PICK/PLACE delays

class CornerTurnUTrack(Node):
    def __init__(self):
        super().__init__('u_track_corner_turn')

        # Control Parameters
        self.declare_parameter('max_linear_speed', 0.12)     # m/s 0.16
        self.declare_parameter('min_linear_speed', 0.10)     # m/s
        self.declare_parameter('max_angular_speed', 0.5)    # rad/s
        self.declare_parameter('min_angular_speed', 0.4)    # rad/s
        self.declare_parameter('waypoint_tolerance', 0.07)  # Meters to consider corner reached
        self.declare_parameter('yaw_tolerance', 0.13)       # Radians (~7.5 degrees)

        self.v_max = self.get_parameter('max_linear_speed').value
        self.v_min = self.get_parameter('min_linear_speed').value
        self.w_max = self.get_parameter('max_angular_speed').value
        self.w_min = self.get_parameter('min_angular_speed').value
        self.wp_tolerance = self.get_parameter('waypoint_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        self.waypoints = [
            np.array([0.0, 1.0]),
            np.array([0.0, 0.35]),
            np.array([2.13, 0.35]),
            np.array([2.13, 1.0])
        ]

        self.prev_target_yaw = 0.0

        # ==========================================
        # 2. DEFINE TASK PLAN QUEUE
        # ==========================================
        # self.task_queue: List[RobotTask] = [
        #     RobotTask(type=TaskType.GO_TO, target_pos=np.array([0.0, 0.35])),
        #     RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=0.0),
        #     RobotTask(type=TaskType.GO_TO, target_pos=np.array([2.13, 0.35])),
        #     RobotTask(type=TaskType.PICK, item_id="Box_A", duration=3.0),
        #     RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=math.pi / 2),
        #     RobotTask(type=TaskType.GO_TO, target_pos=np.array([2.13, 1.0])),
        #     RobotTask(type=TaskType.PLACE, item_id="Box_A", duration=3.0)
        # ]
        self.task_queue: List[RobotTask] = [
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([0.0, 0.40])),
            RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=0.0),
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([2.13, 0.40])),
            RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=math.pi / 2),
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([2.13, 1.0])),
        ]
        
        self.current_task_idx = 0
        self.action_timer = None  # Non-blocking timer for PICK/PLACE tasks

        # State Variables
        self.robot_pos = None
        self.robot_yaw = None
        self.state = "EXECUTE_TASK"    
        
        # E-Stop State
        self.is_e_stopped = False
        self.running = True

        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)

        # ROS 2 Interfaces
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control Loop Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # Start dedicated keyboard thread
        self.key_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.key_thread.start()

        self.get_logger().info("==========================================")
        self.get_logger().info("Task Execution Planner Node Initialized.")
        self.get_logger().info("--> PRESS [SPACEBAR] TO TOGGLE EMERGENCY STOP <--")
        self.get_logger().info("==========================================")

    def keyboard_listener(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running and rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == ' ':
                        self.is_e_stopped = not self.is_e_stopped
                        if self.is_e_stopped:
                            self.get_logger().warn("🚨 EMERGENCY STOP ACTIVATED! 🚨")
                            self.stop_robot()
                        else:
                            self.get_logger().info("▶️ Emergency Stop Cleared! Resuming tasks...")
                    elif key == '\x03':
                        break
        except Exception:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info("Goal callback received. Resetting task execution queue.")
        self.current_task_idx = 0

    def odom_callback(self, msg: Odometry):
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp_angular_velocity(self, w_calc: float) -> float:
        """Clamps angular speed magnitude between w_min and w_max while keeping turn direction."""
        if abs(w_calc) < 1e-5:
            return 0.0
        
        # Clamp magnitude
        w_mag = np.clip(abs(w_calc), self.w_min, self.w_max)
        
        # Preserve direction sign (+1.0 or -1.0)
        return math.copysign(w_mag, w_calc)

    # ==========================================
    # 3. TASK EXECUTOR CONTROL LOOP
    # ==========================================
    def control_loop(self):
        if self.is_e_stopped:
            self.stop_robot()
            return

        if self.robot_pos is None or self.robot_yaw is None:
            return

        # Check if all tasks in queue are finished
        if self.current_task_idx >= len(self.task_queue):
            self.get_logger().info("All planned tasks completed successfully!", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        cmd = Twist()
        current_task = self.task_queue[self.current_task_idx]

        # --- TASK 1: GO_TO ---
        if current_task.type == TaskType.GO_TO:
            target_pos = current_task.target_pos
            dx = target_pos[0] - self.robot_pos[0]
            dy = target_pos[1] - self.robot_pos[1]
            dist_to_target = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(target_yaw - self.robot_yaw)
            self.get_logger().info(f"GO_TO_TASK yaw_error={yaw_error:.2f} target_yaw={target_yaw:.2f} robot_yaw={self.robot_yaw:.2f}")
            if dist_to_target < self.wp_tolerance:
                self.get_logger().info(f"✅ GO_TO Task {self.current_task_idx} Reached Target Pos.")
                self.stop_robot()
                self.current_task_idx += 1
            else:  # Drive straight toward goal
                cmd.linear.x = np.clip(0.5 * dist_to_target, self.v_min, self.v_max)
                cmd.angular.z = self.clamp_angular_velocity(1.0 * yaw_error)
                self.prev_target_yaw = target_yaw

        # --- TASK 2: ROTATE_IN_PLACE ---
        elif current_task.type == TaskType.ROTATE_IN_PLACE:
            yaw_error = self.normalize_angle(current_task.target_yaw - self.robot_yaw)
            self.get_logger().info(f"ROTATE_TASK yaw_error={yaw_error:.2f}")

            if abs(yaw_error) <= self.yaw_tolerance:
                self.get_logger().info(f"✅ ROTATE Task {self.current_task_idx} Aligned.")
                self.stop_robot()
                self.current_task_idx += 1
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.clamp_angular_velocity(1.0 * yaw_error)

        # --- TASK 3 & 4: PICK and PLACE ---
        elif current_task.type in [TaskType.PICK, TaskType.PLACE]:
            self.stop_robot()
            if self.action_timer is None:
                action_name = "PICKING" if current_task.type == TaskType.PICK else "PLACING"
                self.get_logger().info(f"🦾 {action_name} item '{current_task.item_id}'... ({current_task.duration}s)")
                
                # Non-blocking delay to simulate actuator/gripper operation
                self.action_timer = self.create_timer(current_task.duration, self._finish_pick_place_action)

        self.cmd_pub.publish(cmd)

    def _finish_pick_place_action(self):
        """Callback triggered when PICK/PLACE duration completes."""
        self.get_logger().info(f"✅ Task {self.current_task_idx} Complete.")
        self.action_timer.cancel()
        self.action_timer = None
        self.current_task_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = CornerTurnUTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()