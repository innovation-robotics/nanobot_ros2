#!/usr/bin/env python3
from dataclasses import dataclass
from enum import Enum, auto
import math
import select
import sys
import termios
import threading
import time
import tty
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
from robot_interfaces.action import DriveBaseGoal, MoveArmToPoint, MoveArmToPose


class TaskType(Enum):
    GO_TO = auto()
    ROTATE_IN_PLACE = auto()
    PICK = auto()
    PICK_OBJECT = auto()
    PLACE = auto()


@dataclass
class RobotTask:
    type: TaskType
    target_pos: Optional[np.ndarray] = None
    target_yaw: Optional[float] = None
    item_id: Optional[str] = None


class MaestroNode(Node):
    def __init__(self):
        super().__init__('maestro')

        self.gripper_presets = {
            "hand_open": -1.2,
            "hand_closed": -0.261799388,
            "hand_fully_closed": 0.0,
        }

        self.task_queue: List[RobotTask] = [
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([0.23, 0.52])),
            RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=-math.pi),
            RobotTask(type=TaskType.PICK_OBJECT, item_id="Box_A"),
            RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=-math.pi / 2.0),
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([0.23, 0.35])),
            RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=0.0),
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([0.60, 0.35])),
            RobotTask(type=TaskType.ROTATE_IN_PLACE, target_yaw=math.pi / 2.0),
            RobotTask(type=TaskType.GO_TO, target_pos=np.array([0.60, 0.595])),
            RobotTask(type=TaskType.PLACE, item_id="Box_A"),
        ]

        self.current_task_idx = 0
        self.task_running = False
        self.is_e_stopped = False
        self.running = True
        self.obj18_pos: Optional[np.ndarray] = None

        self._goal_lock = threading.Lock()
        self._active_base_goal_handle = None
        self._active_arm_goal_handle = None

        self.old_settings = termios.tcgetattr(sys.stdin)

        # Subscriptions & direct stop publisher
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.obj18_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/aruco/pose18', self.obj18_pose_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Float32, '/arm_controller/gripper_angle', 10)

        # Action clients
        self._base_action_client = ActionClient(self, DriveBaseGoal, '/mobile_base_controller/drive_base')
        self._arm_action_client = ActionClient(self, MoveArmToPoint, '/arm_controller/move_to_point')
        self._arm_pose_action_client = ActionClient(self, MoveArmToPose, '/arm_controller/move_to_pose')

        self.timer = self.create_timer(0.1, self.maestro_tick)
        self.key_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.key_thread.start()

        self.get_logger().info("Maestro action client node initialized. Press [SPACEBAR] to toggle E-stop.")

    # -------------------------------------------------------------------------
    # ACTION CLIENT HELPERS
    # -------------------------------------------------------------------------
    def _send_action_and_wait(
        self,
        action_client: ActionClient,
        goal_msg,
        handle_attr: str,
        server_timeout_sec: float = 3.0,
        result_timeout_sec: float = 30.0,
    ):
        if not action_client.wait_for_server(timeout_sec=server_timeout_sec):
            return False, f"Action server {action_client._action_name} unavailable.", None

        send_future = action_client.send_goal_async(goal_msg)
        goal_done = threading.Event()
        send_future.add_done_callback(lambda _: goal_done.set())

        if not goal_done.wait(timeout=server_timeout_sec):
            return False, f"Timed out sending goal to {action_client._action_name}.", None

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, f"Goal rejected by {action_client._action_name}.", None

        with self._goal_lock:
            setattr(self, handle_attr, goal_handle)

        try:
            res_future = goal_handle.get_result_async()
            res_done = threading.Event()
            res_future.add_done_callback(lambda _: res_done.set())

            if not res_done.wait(timeout=result_timeout_sec):
                goal_handle.cancel_goal_async()
                return False, f"Timed out waiting for result from {action_client._action_name}.", None

            wrapped_result = res_future.result()
            if wrapped_result is None:
                return False, f"No result returned from {action_client._action_name}.", None

            return True, "", wrapped_result.result
        finally:
            with self._goal_lock:
                if getattr(self, handle_attr) is goal_handle:
                    setattr(self, handle_attr, None)

    def send_go_to(self, x: float, y: float, max_speed: float = 0.0) -> Tuple[bool, str]:
        goal = DriveBaseGoal.Goal()
        goal.mode = DriveBaseGoal.Goal.MODE_GO_TO
        goal.x = float(x)
        goal.y = float(y)
        goal.angle_rad = 0.0
        goal.max_speed = float(max_speed)

        ok, err, res = self._send_action_and_wait(
            self._base_action_client,
            goal,
            '_active_base_goal_handle',
            result_timeout_sec=30.0,
        )
        if not ok or res is None:
            return False, err
        return bool(res.success), str(res.message)

    def send_rotate_in_place(self, yaw: float, max_speed: float = 0.0) -> Tuple[bool, str]:
        goal = DriveBaseGoal.Goal()
        goal.mode = DriveBaseGoal.Goal.MODE_ROTATE_IN_PLACE
        goal.angle_rad = float(yaw)
        goal.x = 0.0
        goal.y = 0.0
        goal.max_speed = float(max_speed)

        ok, err, res = self._send_action_and_wait(
            self._base_action_client,
            goal,
            '_active_base_goal_handle',
            result_timeout_sec=20.0,
        )
        if not ok or res is None:
            return False, err
        return bool(res.success), str(res.message)

    def send_move_to_xyz(self, x: float, y: float, z: float, duration_sec: float = 0.0) -> Tuple[bool, str]:
        goal = MoveArmToPoint.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.z = float(z)
        goal.duration_sec = float(duration_sec)

        ok, err, res = self._send_action_and_wait(
            self._arm_action_client,
            goal,
            '_active_arm_goal_handle',
            result_timeout_sec=25.0,
        )
        if not ok or res is None:
            return False, err
        return bool(res.success), str(res.message)

    def send_move_to_pose(self, pose_name: str, duration_sec: float = 1.5) -> Tuple[bool, str]:
        goal = MoveArmToPose.Goal()
        goal.pose_name = str(pose_name)
        goal.duration_sec = float(duration_sec)

        ok, err, res = self._send_action_and_wait(
            self._arm_pose_action_client,
            goal,
            '_active_arm_goal_handle',
            result_timeout_sec=25.0,
        )
        if not ok or res is None:
            return False, err
        return bool(res.success), str(res.message)

    def send_gripper(self, angle: float) -> Tuple[bool, str]:
        msg = Float32()
        msg.data = float(angle)
        self.gripper_pub.publish(msg)
        time.sleep(2.0)  # Allow time for gripper to move
        return True, f"Published gripper angle {angle:.4f} rad."

    def cancel_and_stop_active_goals(self) -> None:
        with self._goal_lock:
            base_gh = self._active_base_goal_handle
            arm_gh = self._active_arm_goal_handle
        if base_gh is not None:
            base_gh.cancel_goal_async()
        if arm_gh is not None:
            arm_gh.cancel_goal_async()
        self.cmd_vel_pub.publish(Twist())

    # -------------------------------------------------------------------------
    # CALLBACKS & KEYBOARD E-STOP
    # -------------------------------------------------------------------------
    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info("Goal callback received. Resetting task queue.")
        self.current_task_idx = 0

    def obj18_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.obj18_pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=float,
        )

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
                            self.cancel_and_stop_active_goals()
                        else:
                            self.get_logger().info("▶️ Emergency Stop Cleared! Resuming...")
                    elif key == '\x03':
                        break
        except Exception:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    # -------------------------------------------------------------------------
    # TASK SEQUENCES
    # -------------------------------------------------------------------------
    def _run_pick_sequence(self, item_id: str) -> bool:
        self.get_logger().info(f"🦾 Starting PICK for '{item_id}'...")
        steps = [
            self.send_gripper(self.gripper_presets["hand_open"]),
            self.send_gripper(self.gripper_presets["hand_closed"]),
        ]
        for idx, (ok, msg) in enumerate(steps, start=1):
            if not ok:
                self.get_logger().error(f"PICK failed at step {idx}: {msg}")
                return False
            time.sleep(0.5)

        ok, msg = self.send_move_to_pose("arm_ready", duration_sec=1.5)
        if not ok:
            self.get_logger().error(f"PICK failed returning to 'arm_ready': {msg}")
            return False

        self.get_logger().info(f"✅ PICK completed for '{item_id}'.")
        return True

    def _run_pick_object_sequence(self, item_id: str) -> bool:
        while rclpy.ok() and not self.is_e_stopped and self.obj18_pos is None:
            self.get_logger().info("Waiting for /aruco/pose18...", throttle_duration_sec=1.0)
            time.sleep(0.2)

        if self.is_e_stopped or self.obj18_pos is None:
            return False

        self.get_logger().info("Settling down before object pick...")
        time.sleep(5.0)

        target_x = float(self.obj18_pos[0])
        target_y = float(self.obj18_pos[1])
        target_z = float(self.obj18_pos[2])

        ok, msg = self.send_gripper(self.gripper_presets["hand_open"])
        if not ok:
            self.get_logger().error(f"PICK_OBJECT failed opening gripper: {msg}")
            return False
        time.sleep(0.5)

        ok, msg = self.send_move_to_xyz(target_x, target_y, target_z, duration_sec=1.5)
        if not ok:
            self.get_logger().error(f"PICK_OBJECT failed moving arm to XYZ: {msg}")
            return False
        time.sleep(0.5)

        ok, msg = self.send_gripper(self.gripper_presets["hand_closed"])
        if not ok:
            self.get_logger().error(f"PICK_OBJECT failed closing gripper: {msg}")
            return False
        time.sleep(0.5)

        ok, msg = self.send_move_to_pose("arm_ready", duration_sec=1.5)
        if not ok:
            self.get_logger().error(f"PICK_OBJECT failed returning to 'arm_ready': {msg}")
            return False

        self.get_logger().info(f"✅ PICK_OBJECT completed for '{item_id}'.")
        return True

    def _run_place_sequence(self, item_id: str) -> bool:
        self.get_logger().info(f"🦾 Starting PLACE for '{item_id}'...")
        steps = [
            self.send_gripper(self.gripper_presets["hand_open"]),
            self.send_gripper(self.gripper_presets["hand_fully_closed"]),
        ]
        for idx, (ok, msg) in enumerate(steps, start=1):
            if not ok:
                self.get_logger().error(f"PLACE failed at step {idx}: {msg}")
                return False
            time.sleep(0.5)
        self.get_logger().info(f"✅ PLACE completed for '{item_id}'.")
        return True

    def _worker_execute_task(self, task: RobotTask):
        try:
            success = False
            if task.type == TaskType.GO_TO and task.target_pos is not None:
                success, msg = self.send_go_to(float(task.target_pos[0]), float(task.target_pos[1]))
                self.get_logger().info(msg)
            elif task.type == TaskType.ROTATE_IN_PLACE and task.target_yaw is not None:
                success, msg = self.send_rotate_in_place(float(task.target_yaw))
                self.get_logger().info(msg)
            elif task.type == TaskType.PICK:
                success = self._run_pick_sequence(task.item_id or "object")
            elif task.type == TaskType.PICK_OBJECT:
                success = self._run_pick_object_sequence(task.item_id or "object")
            elif task.type == TaskType.PLACE:
                success = self._run_place_sequence(task.item_id or "object")

            if success:
                self.current_task_idx += 1
            else:
                self.get_logger().warn(f"Task {task.type.name} did not complete; holding at index {self.current_task_idx}.")
        finally:
            self.task_running = False

    def maestro_tick(self):
        if self.is_e_stopped or self.task_running:
            return
        if self.current_task_idx >= len(self.task_queue):
            self.get_logger().info("All planned tasks completed successfully!", throttle_duration_sec=2.0)
            return

        self.task_running = True
        task = self.task_queue[self.current_task_idx]
        threading.Thread(target=self._worker_execute_task, args=(task,), daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    maestro = MaestroNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(maestro)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        maestro.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, maestro.old_settings)
        maestro.cancel_and_stop_active_goals()
        executor.shutdown()
        maestro.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()