#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
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

# MoveIt Action Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import time

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
    duration: float = 2.0                    # Fallback delay if MoveGroup action server is offline

class CornerTurnUTrack(Node):
    def __init__(self):
        super().__init__('u_track_corner_turn')

        # Control Parameters
        self.declare_parameter('max_linear_speed', 0.12)     # m/s
        self.declare_parameter('min_linear_speed', 0.10)     # m/s
        self.declare_parameter('max_angular_speed', 0.5)     # rad/s
        self.declare_parameter('min_angular_speed', 0.4)     # rad/s
        self.declare_parameter('waypoint_tolerance', 0.07)   # Meters
        self.yaw_tolerance = 0.13                            # Radians (~7.5 deg)

        self.v_max = self.get_parameter('max_linear_speed').value
        self.v_min = self.get_parameter('min_linear_speed').value
        self.w_max = self.get_parameter('max_angular_speed').value
        self.w_min = self.get_parameter('min_angular_speed').value
        self.wp_tolerance = self.get_parameter('waypoint_tolerance').value

        # MoveGroup Action Client
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # Pre-defined SRDF Joint Goals Mapping
        self.srdf_states = {
            # ARM Group Joint Targets
            "arm_ready": {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0, "wrist_joint": 0.0},
            "arm_pick":  {"joint1": 0.0, "joint2": -1.0935, "joint3": -0.8852, "joint4": -1.1629, "wrist_joint": 0.0},
            
            # HAND Group Joint Targets
            "hand_open": {
                "Tip_Gripper_Idol_Joint": -1.2,
                "Tip_Gripper_Servo_Joint": 1.2,
                "robot_finger_joint1": -1.2,
                "robot_finger_joint2": 1.2
            },
            "hand_closed": {
                "Tip_Gripper_Idol_Joint": -0.5235,
                "Tip_Gripper_Servo_Joint": 0.523597,
                "robot_finger_joint1": -0.523597,
                "robot_finger_joint2": 0.523597
            },
            "hand_fully_closed": {
                "Tip_Gripper_Idol_Joint": 0.0,
                "Tip_Gripper_Servo_Joint": 0.0,
                "robot_finger_joint1": 0.0,
                "robot_finger_joint2": 0.0
            }
        }

        # ==========================================
        # 2. DEFINE TASK PLAN QUEUE
        # ==========================================
        self.task_queue: List[RobotTask] = [
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([0.0, 0.65])),
            RobotTask(type=TaskType.PICK, item_id="Box_A"),
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([0.0, 0.35])),
            RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=0.0),
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([2.13, 0.35])),
            RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=math.pi / 2),
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([2.13, 0.8])),
            RobotTask(type=TaskType.PLACE, item_id="Box_A")
        ]
        
        self.current_task_idx = 0
        self.manipulation_in_progress = False

        # State Variables
        self.robot_pos = None
        self.robot_yaw = None
        
        # E-Stop State
        self.is_e_stopped = False
        self.running = True

        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)

        # ROS 2 Interfaces
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
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
        self.robot_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
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
        if abs(w_calc) < 1e-5:
            return 0.0
        w_mag = np.clip(abs(w_calc), self.w_min, self.w_max)
        return math.copysign(w_mag, w_calc)

    def send_joint_goal_to_movegroup(self, group_name: str, state_key: str, speed_scale: float = 0.7) -> bool:
        """Sends joint constraints to MoveGroup Action Server with controlled velocity."""
        if not self._move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f"MoveGroup action server unavailable. Skipping joint state '{state_key}'.")
            return False

        if state_key not in self.srdf_states:
            self.get_logger().error(f"State '{state_key}' not found in srdf_states map.")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0

        # =========================================================
        # SLOW DOWN ARM MOTION HERE (0.01 to 1.0)
        # =========================================================
        # Sets velocity and acceleration to 10% of max allowed speed
        goal_msg.request.max_velocity_scaling_factor = speed_scale
        goal_msg.request.max_acceleration_scaling_factor = speed_scale

        # Construct Joint Constraints
        constraints = Constraints()
        joint_targets = self.srdf_states[state_key]

        for joint_name, target_val in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target_val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Sending goal for group '{group_name}' -> '{state_key}' (Speed scale: {speed_scale})")

        # --- SAFE THREAD SYNCHRONIZATION ---
        event = threading.Event()
        goal_handle = None

        def goal_response_callback(future):
            nonlocal goal_handle
            goal_handle = future.result()
            event.set()

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(goal_response_callback)
        
        if not event.wait(timeout=5.0):
            self.get_logger().error(f"Timeout waiting for goal acceptance for '{state_key}'.")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal '{state_key}' rejected by MoveGroup.")
            return False

        event.clear()
        result_success = False

        def get_result_callback(future):
            nonlocal result_success
            result = future.result()
            result_success = (result.status == 4)
            event.set()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(get_result_callback)

        if not event.wait(timeout=25.0):  # Increased timeout since motion is slower
            self.get_logger().error(f"Timeout waiting for result of goal '{state_key}'.")
            return False

        if result_success:
            self.get_logger().info(f"✅ Executed state '{state_key}' successfully.")
        else:
            self.get_logger().error(f"❌ Failed executing state '{state_key}'.")

        return result_success

    def execute_pick_sequence(self, item_id: str):
        """Sequential execution for PICKing an object."""
        self.get_logger().info(f"🦾 Starting PICK Sequence for item: '{item_id}'...")
        
        # Step 1: Open Gripper
        if not self.send_joint_goal_to_movegroup("hand", "hand_open"):
            self.get_logger().error("❌ PICK sequence failed at Step 1 (hand_open). Aborting.")
            self.manipulation_in_progress = False
            return
        time.sleep(0.5)  # Let joints settle

        # Step 2: Lower Arm to Pick Position
        if not self.send_joint_goal_to_movegroup("arm", "arm_pick"):
            self.get_logger().error("❌ PICK sequence failed at Step 2 (arm_pick). Aborting.")
            self.manipulation_in_progress = False
            return
        time.sleep(0.5)  # Let joints settle

        # Step 3: Close Gripper on Object
        if not self.send_joint_goal_to_movegroup("hand", "hand_closed"):
            self.get_logger().error("❌ PICK sequence failed at Step 3 (hand_closed). Aborting.")
            self.manipulation_in_progress = False
            return
        time.sleep(0.5)  # Let joints settle

        # Step 4: Lift Arm to Ready Position
        if not self.send_joint_goal_to_movegroup("arm", "arm_ready"):
            self.get_logger().error("❌ PICK sequence failed at Step 4 (arm_ready). Aborting.")
            self.manipulation_in_progress = False
            return

        self.get_logger().info(f"✅ PICK Sequence Completed for item: '{item_id}'.")
        self.manipulation_in_progress = False
        self.current_task_idx += 1


    def execute_place_sequence(self, item_id: str):
        """Sequential execution for PLACing an object."""
        self.get_logger().info(f"🦾 Starting PLACE Sequence for item: '{item_id}'...")
        
        # Step 1: Lower Arm to Place Position
        if not self.send_joint_goal_to_movegroup("arm", "arm_pick"):
            self.get_logger().error("❌ PLACE sequence failed at Step 1 (arm_pick). Aborting.")
            self.manipulation_in_progress = False
            return
        time.sleep(0.5)

        # Step 2: Open Gripper
        if not self.send_joint_goal_to_movegroup("hand", "hand_open"):
            self.get_logger().error("❌ PLACE sequence failed at Step 2 (hand_open). Aborting.")
            self.manipulation_in_progress = False
            return
        time.sleep(0.5)

        # Step 3: Lift Arm back to Ready Position
        if not self.send_joint_goal_to_movegroup("arm", "arm_ready"):
            self.get_logger().error("❌ PLACE sequence failed at Step 3 (arm_ready). Aborting.")
            self.manipulation_in_progress = False
            return
        time.sleep(0.5)

        # Step 4: Close Gripper Fully
        if not self.send_joint_goal_to_movegroup("hand", "hand_fully_closed"):
            self.get_logger().error("❌ PLACE sequence failed at Step 4 (hand_fully_closed). Aborting.")
            self.manipulation_in_progress = False
            return

        self.get_logger().info(f"✅ PLACE Sequence Completed for item: '{item_id}'.")
        self.manipulation_in_progress = False
        self.current_task_idx += 1

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

            if dist_to_target < self.wp_tolerance:
                self.get_logger().info(f"✅ GO_TO Task {self.current_task_idx} Reached Target Pos.")
                self.stop_robot()
                self.current_task_idx += 1
            else:
                cmd.linear.x = np.clip(0.5 * dist_to_target, self.v_min, self.v_max)
                cmd.angular.z = self.clamp_angular_velocity(1.0 * yaw_error)

        # --- TASK 2: ROTATE_IN_PLACE ---
        elif current_task.type == TaskType.ROTATE_IN_PLACE:
            yaw_error = self.normalize_angle(current_task.target_yaw - self.robot_yaw)

            if abs(yaw_error) <= self.yaw_tolerance:
                self.get_logger().info(f"✅ ROTATE Task {self.current_task_idx} Aligned.")
                self.stop_robot()
                self.current_task_idx += 1
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.clamp_angular_velocity(1.0 * yaw_error)

        # --- TASK 3: PICK ---
        elif current_task.type == TaskType.PICK:
            self.stop_robot()
            if not self.manipulation_in_progress:
                self.manipulation_in_progress = True
                threading.Thread(
                    target=self.execute_pick_sequence,
                    args=(current_task.item_id,),
                    daemon=True
                ).start()

        # --- TASK 4: PLACE ---
        elif current_task.type == TaskType.PLACE:
            self.stop_robot()
            if not self.manipulation_in_progress:
                self.manipulation_in_progress = True
                threading.Thread(
                    target=self.execute_place_sequence,
                    args=(current_task.item_id,),
                    daemon=True
                ).start()

        self.cmd_pub.publish(cmd)


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