#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Float32, Float32MultiArray

class RobotHardwareBridge(Node):
    def __init__(self):
        super().__init__('arm_trajectory_bridge')

        # 1. Arm & Hand Joint Names (Matches URDF and MoveIt YAML)
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'wrist_joint']
        self.hand_joints = [
            'robot_finger_joint1',
            'Tip_Gripper_Servo_Joint',
            'robot_finger_joint2',
            'Tip_Gripper_Idol_Joint'
        ]
        
        self.current_arm_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        # State array matched index-by-index with self.hand_joints
        self.current_gripper_positions = [0.0, 0.0, 0.0, 0.0]

        # 2. Publishers
        # self.arm_cmd_pub = self.create_publisher(JointState, '/arm_servo_cmds', 10)
        self.arm_cmd_pub = self.create_publisher(Float32MultiArray, '/arm_servo_cmds', 10)
        self.gripper_cmd_pub = self.create_publisher(Float32, '/gripper_cmd', 10)
        self.full_joint_state_pub = self.create_publisher(JointState, '/arm_joint_states', 10)

        # High-frequency Timer (50 Hz) for smooth state feedback in RViz
        self.create_timer(1.0 / 50.0, self.publish_full_joint_states)

        # 3. Action Servers
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

        # self._gripper_action_server = ActionServer(
        #     self,
        #     FollowJointTrajectory,
        #     '/hand_controller/follow_joint_trajectory',
        #     execute_callback=self.execute_arm_callback,
        #     goal_callback=self.goal_callback,
        #     cancel_callback=self.cancel_callback
        # )

        self.get_logger().info('Arm Trajectory Bridge initialized without software interpolation!')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def publish_full_joint_states(self):
        msg = JointState()
        msg.name = self.arm_joints + self.hand_joints
        msg.position = self.current_arm_positions + self.current_gripper_positions
        self.full_joint_state_pub.publish(msg)

    def execute_arm_callback(self, goal_handle):
        self.get_logger().info('Executing arm trajectory...')
        trajectory = goal_handle.request.trajectory

        if not trajectory.points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        prev_time = 0.0

        for pt_idx, point in enumerate(trajectory.points):
        # for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_time = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            target_pos = list(point.positions)
            duration = target_time - prev_time

            # Delay to preserve trajectory timing spacing
            if duration > 0:
                time.sleep(duration)

            # Direct pass-through of waypoints
            self.current_arm_positions = target_pos
            
            # cmd = JointState()
            # cmd.name = self.arm_joints
            # cmd.position = target_pos
            # self.arm_cmd_pub.publish(cmd)

            # Send multi-array to ESP32
            cmd = Float32MultiArray()
            cmd.data = [float(x) for x in target_pos]

            # --- DEBUG LOGGING ---
            self.get_logger().info(
                f'[Waypoint {pt_idx + 1}/{len(trajectory.points)}] '
                f'Raw Positions: {[round(p, 4) for p in point.positions]} | '
                f'Mapped Cmd Data: {[round(x, 4) for x in cmd.data]}'
            )

            self.arm_cmd_pub.publish(cmd)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self.arm_joints
            feedback.actual.positions = target_pos
            feedback.desired.positions = target_pos
            goal_handle.publish_feedback(feedback)

            prev_time = target_time

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result



    def execute_arm_callback2(self, goal_handle):
        self.get_logger().info('Executing arm trajectory...')
        trajectory = goal_handle.request.trajectory

        if not trajectory.points:
            self.get_logger().warn('Received empty trajectory points!')
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        # Print incoming joint names from MoveIt to verify joint ordering
        self.get_logger().info(f'MoveIt Trajectory Joint Order: {trajectory.joint_names}')
        self.get_logger().info(f'Bridge Expected Arm Joints:   {self.arm_joints}')

        # Build map to align MoveIt joint order with self.arm_joints order
        joint_map = [self.arm_joints.index(name) if name in self.arm_joints else -1 for name in trajectory.joint_names]

        prev_time = 0.0

        for pt_idx, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Trajectory execution canceled by client.')
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_time = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            duration = target_time - prev_time

            # Reconstruct target array according to self.arm_joints ordering
            ordered_target_pos = list(self.current_arm_positions)
            for idx, pos_val in enumerate(point.positions):
                if joint_map[idx] != -1:
                    ordered_target_pos[joint_map[idx]] = pos_val

            # Delay to preserve trajectory timing spacing
            if duration > 0:
                time.sleep(duration)

            # Direct pass-through of mapped waypoints
            self.current_arm_positions = ordered_target_pos

            # Send multi-array to ESP32
            cmd = Float32MultiArray()
            cmd.data = [float(x) for x in ordered_target_pos]
            
            # --- DEBUG LOGGING ---
            self.get_logger().info(
                f'[Waypoint {pt_idx + 1}/{len(trajectory.points)}] '
                f'Raw Positions: {[round(p, 4) for p in point.positions]} | '
                f'Mapped Cmd Data: {[round(x, 4) for x in cmd.data]}'
            )

            self.arm_cmd_pub.publish(cmd)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self.arm_joints
            feedback.actual.positions = ordered_target_pos
            feedback.desired.positions = ordered_target_pos
            goal_handle.publish_feedback(feedback)

            prev_time = target_time

        self.get_logger().info('Arm trajectory successfully published to hardware!')
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


    def execute_gripper_trajectory_callback(self, goal_handle):
        self.get_logger().info('Executing gripper trajectory...')
        trajectory = goal_handle.request.trajectory

        if not trajectory.points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        # Map each incoming joint directly to its target index in self.hand_joints
        joint_map = [self.hand_joints.index(name) for name in trajectory.joint_names]

        prev_time = 0.0

        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_time = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            duration = target_time - prev_time

            # Update position array using the direct map
            ordered_target_pos = list(self.current_gripper_positions)
            for idx, pos_val in enumerate(point.positions):
                ordered_target_pos[joint_map[idx]] = pos_val

            if duration > 0:
                time.sleep(duration)

            # Update internal state for RViz state publisher
            self.current_gripper_positions = ordered_target_pos

            # Direct publish main servo position to ESP32
            msg = Float32()
            msg.data = float(ordered_target_pos[0])

                        # --- DEBUG LOGGING ---
            self.get_logger().info(f'gripper angle: {ordered_target_pos[0]}')

            self.gripper_cmd_pub.publish(msg)

            # Action feedback
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = trajectory.joint_names
            feedback.actual.positions = list(point.positions)
            feedback.desired.positions = list(point.positions)
            goal_handle.publish_feedback(feedback)

            prev_time = target_time

        goal_handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return res

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