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
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger


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

        # Presets converted from your original SRDF states
        self.joint_presets = {
            "arm_ready": [0.0, 0.0, 0.0],
            "arm_pick_place": [0.0, -1.466076572, -1.850049007],
            "arm_left": [0.0, 0.0, 0.0],
        }
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

        self.old_settings = termios.tcgetattr(sys.stdin)

        # Subscriptions
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.obj18_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/aruco/pose18', self.obj18_pose_callback, 10
        )

        # Base controller service & parameter clients
        self.go_to_cli = self.create_client(Trigger, '/base_controller/go_to')
        self.rotate_cli = self.create_client(Trigger, '/base_controller/rotate_in_place')
        self.base_stop_cli = self.create_client(Trigger, '/base_controller/stop')
        self.base_set_params_cli = self.create_client(SetParameters, '/mobile_base_controller/set_parameters')

        # Arm controller service & parameter clients
        self.move_xyz_cli = self.create_client(Trigger, '/arm_controller/move_to_xyz')
        self.gripper_cli = self.create_client(Trigger, '/arm_controller/set_gripper')
        self.arm_set_params_cli = self.create_client(SetParameters, '/arm_controller/set_parameters')

        self.timer = self.create_timer(0.1, self.maestro_tick)
        self.key_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.key_thread.start()

        self.get_logger().info("Maestro standalone node initialized. Press [SPACEBAR] to toggle E-stop.")

    # -------------------------------------------------------------------------
    # ROS 2 SERVICE & PARAMETER HELPERS
    # -------------------------------------------------------------------------
    def _set_remote_params(self, param_client, params: List[Parameter], timeout_sec: float = 2.0) -> bool:
        if not param_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"Parameter service {param_client.srv_name} unavailable.")
            return False

        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg() for p in params]

        future = param_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=timeout_sec):
            self.get_logger().error(f"Timed out setting parameters on {param_client.srv_name}.")
            return False

        res = future.result()
        return bool(res and all(r.successful for r in res.results))

    def _call_trigger(self, client, timeout_sec: float = 30.0) -> Tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"Service {client.srv_name} unavailable."

        future = client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=timeout_sec):
            return False, f"Timed out waiting for {client.srv_name}."

        res = future.result()
        if res is None:
            return False, f"No response from {client.srv_name}."
        return bool(res.success), str(res.message)

    def send_go_to(self, x: float, y: float) -> Tuple[bool, str]:
        ok = self._set_remote_params(
            self.base_set_params_cli,
            [Parameter('goal_x', value=float(x)), Parameter('goal_y', value=float(y))],
        )
        if not ok:
            return False, "Failed to set goal_x/goal_y on mobile_base_controller."
        return self._call_trigger(self.go_to_cli, timeout_sec=30.0)

    def send_rotate_in_place(self, yaw: float) -> Tuple[bool, str]:
        ok = self._set_remote_params(
            self.base_set_params_cli,
            [Parameter('goal_yaw', value=float(yaw))],
        )
        if not ok:
            return False, "Failed to set goal_yaw on mobile_base_controller."
        return self._call_trigger(self.rotate_cli, timeout_sec=20.0)

    def send_stop_base(self) -> None:
        self._call_trigger(self.base_stop_cli, timeout_sec=2.0)

    def send_move_to_xyz(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        ok = self._set_remote_params(
            self.arm_set_params_cli,
            [
                Parameter('target_x', value=float(x)),
                Parameter('target_y', value=float(y)),
                Parameter('target_z', value=float(z)),
            ],
        )
        if not ok:
            return False, "Failed to set target_x/target_y/target_z on arm_controller."
        return self._call_trigger(self.move_xyz_cli, timeout_sec=20.0)

    def send_gripper(self, angle: float) -> Tuple[bool, str]:
        ok = self._set_remote_params(
            self.arm_set_params_cli,
            [Parameter('gripper_angle', value=float(angle))],
        )
        if not ok:
            return False, "Failed to set gripper_angle on arm_controller."
        return self._call_trigger(self.gripper_cli, timeout_sec=10.0)

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
                            self.send_stop_base()
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

        target_x = float(self.obj18_pos[0] + 0.02)
        target_y = float(self.obj18_pos[1] - 0.03)
        target_z = -0.04

        ok, msg = self.send_gripper(self.gripper_presets["hand_open"])
        if not ok:
            self.get_logger().error(f"PICK_OBJECT failed opening gripper: {msg}")
            return False
        time.sleep(0.5)

        ok, msg = self.send_move_to_xyz(target_x, target_y, target_z)
        if not ok:
            self.get_logger().error(f"PICK_OBJECT failed moving arm to XYZ: {msg}")
            return False
        time.sleep(0.5)

        ok, msg = self.send_gripper(self.gripper_presets["hand_closed"])
        if not ok:
            self.get_logger().error(f"PICK_OBJECT failed closing gripper: {msg}")
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
        maestro.send_stop_base()
        executor.shutdown()
        maestro.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()