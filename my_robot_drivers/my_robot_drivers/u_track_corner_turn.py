#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

class CornerTurnUTrack(Node):
    def __init__(self):
        super().__init__('u_track_corner_turn')

        # Control Parameters
        self.declare_parameter('max_linear_speed', 0.2)     # m/s
        self.declare_parameter('max_angular_speed', 0.6)    # rad/s
        self.declare_parameter('waypoint_tolerance', 0.05)  # Meters to consider corner reached
        self.declare_parameter('yaw_tolerance', 0.05)       # Radians (~2.8 degrees) to finish rotation

        self.v_max = self.get_parameter('max_linear_speed').value
        self.w_max = self.get_parameter('max_angular_speed').value
        self.wp_tolerance = self.get_parameter('waypoint_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        # Define the 4 waypoints forming the U-shaped track
        self.waypoints = [
            np.array([0.15, 1.0]),
            np.array([0.15, 0.3]),
            np.array([2.27, 0.3]),
            np.array([2.27, 1.0])
        ]

        # State Variables
        self.robot_pos = None
        self.robot_yaw = None
        self.current_wp_idx = 1  # Start tracking toward waypoint 1 (from starting position near wp 0)
        self.state = "ROTATE"    # State Machine: "ROTATE" or "DRIVE"

        # ROS 2 Interfaces
        # Set to /odometry/filtered by default to match EKF node output
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
        self.get_logger().info("Corner Turn U-Track Node Initialized (Point-to-Point Motion).")

    def goal_callback(self, msg: PoseStamped):
        """Allows resetting target waypoint indexing if a goal is published."""
        self.get_logger().info("Goal callback received. Resetting sequence to beginning.")
        self.current_wp_idx = 1
        self.state = "ROTATE"

    def odom_callback(self, msg: Odometry):
        # Extract robot position
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        # Extract yaw orientation from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Log position with 1-second throttling to avoid console spam
        self.get_logger().info(
            f"Robot Position: X={self.robot_pos[0]:.2f}, Y={self.robot_pos[1]:.2f}, yaw={self.robot_yaw:0.2f}",
            throttle_duration_sec=1.0
        )


    def normalize_angle(self, angle):
        """Wraps angle between -pi and pi."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if self.robot_pos is None or self.robot_yaw is None:
            return

        cmd = Twist()

        # Check if sequence is complete
        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info("Target goal reached! Stopping robot.", throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd)
            return

        target_wp = self.waypoints[self.current_wp_idx]

        self.get_logger().info(
            f"current_wp_idx={self.current_wp_idx}",
            throttle_duration_sec=1.0
        )

        # Calculate distance and desired angle to current target waypoint
        dx = target_wp[0] - self.robot_pos[0]
        dy = target_wp[1] - self.robot_pos[1]
        dist_to_wp = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)

        self.get_logger().info(
            f"dx={dx:.2f}, dy={dy:.2f}, dist_to_wp={dist_to_wp:.2f}, target_yaw={target_yaw:.2f}",
            throttle_duration_sec=1.0
        )

        # Compute heading error
        yaw_error = self.normalize_angle(target_yaw - self.robot_yaw)

        self.get_logger().info(
            f"yaw_error={yaw_error:.2f}",
            throttle_duration_sec=1.0
        )

        # --- STATE 1: ROTATE IN PLACE ---
        if self.state == "ROTATE":
            if abs(yaw_error) < self.yaw_tolerance:
                # Aligned with path segment -> switch to driving straight
                self.state = "DRIVE"
                self.get_logger().info(f"Alignment complete. Driving to Waypoint {self.current_wp_idx}")
            else:
                # Rotate in place toward target angle
                cmd.linear.x = 0.0
                cmd.angular.z = np.clip(1.5 * yaw_error, -self.w_max, self.w_max)

        # --- STATE 2: DRIVE STRAIGHT ---
        elif self.state == "DRIVE":
            if dist_to_wp < self.wp_tolerance:
                # Reached corner -> halt linear motion and increment waypoint index
                self.current_wp_idx += 1
                self.state = "ROTATE"
                self.get_logger().info(f"Corner reached! Advancing to Waypoint {self.current_wp_idx}")
            else:
                # Drive forward while making slight angular corrections to stay on track
                cmd.linear.x = min(self.v_max, 0.5 * dist_to_wp)  # Slow down smoothly at corner approach
                cmd.angular.z = np.clip(1.0 * yaw_error, -self.w_max, self.w_max)

        # self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CornerTurnUTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()