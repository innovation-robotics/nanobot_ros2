#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

class PurePursuitUTrack(Node):
    def __init__(self):
        super().__init__('u_track_pure_pursuit')

        # Control Parameters
        self.declare_parameter('lookahead_distance', 0.25)  # Meters (0.2 - 0.35m recommended)
        self.declare_parameter('max_linear_speed', 0.2)     # m/s
        self.declare_parameter('max_angular_speed', 0.8)    # rad/s
        self.declare_parameter('goal_tolerance', 0.05)      # Meters

        self.L = self.get_parameter('lookahead_distance').value
        self.v_max = self.get_parameter('max_linear_speed').value
        self.w_max = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Define the 4 waypoints forming the U-shaped track:
        # Segment 0: (0.15, 1.0) -> (0.15, 0.3)
        # Segment 1: (0.15, 0.3) -> (2.27, 0.3)
        # Segment 2: (2.27, 0.3) -> (2.27, 1.0)
        self.waypoints = [
            np.array([0.15, 1.0]),
            np.array([0.15, 0.3]),
            np.array([2.27, 0.3]),
            np.array([2.27, 1.0])
        ]

        # State Variables
        self.robot_pos = None
        self.robot_yaw = None
        self.target_goal = np.array([2.27, 1.0])  # Default end of U-track

        # ROS 2 Interfaces
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
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
        self.get_logger().info("Pure Pursuit U-Track Node Initialized.")

    def goal_callback(self, msg: PoseStamped):
        """Allows dynamically updating the target stop point along the track."""
        raw_goal = np.array([msg.pose.position.x, msg.pose.position.y])
        # Project goal onto nearest track point to ensure it lies strictly on the U-track
        self.target_goal, _, _ = self.get_closest_track_point(raw_goal)
        self.get_logger().info(f"New Target Goal set on track: {self.target_goal}")

    def odom_callback(self, msg: Odometry):
        # Extract robot position
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        self.get_logger().info(f"Robot Position: X={self.robot_pos[0]:.2f}, Y={self.robot_pos[1]:.2f}")


        # Extract yaw orientation from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def get_closest_track_point(self, point: np.ndarray):
        """Projects any 2D point onto the nearest segment of the U-track."""
        closest_pt = None
        min_dist = float('inf')
        best_seg_idx = 0

        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]

            v = p2 - p1
            u = point - p1
            t = np.dot(u, v) / np.dot(v, v)
            t = np.clip(t, 0.0, 1.0)  # Clamp to line segment boundaries

            proj = p1 + t * v
            dist = np.linalg.norm(point - proj)

            if dist < min_dist:
                min_dist = dist
                closest_pt = proj
                best_seg_idx = i

        return closest_pt, min_dist, best_seg_idx

    def get_lookahead_point(self, current_seg_idx: int, proj_point: np.ndarray):
        """Finds a target point L meters ahead along the U-track sequence."""
        remaining_lookahead = self.L
        curr_pt = proj_point
        seg_idx = current_seg_idx

        while seg_idx < len(self.waypoints) - 1:
            p2 = self.waypoints[seg_idx + 1]
            dist_to_next = np.linalg.norm(p2 - curr_pt)

            if dist_to_next >= remaining_lookahead:
                # Target lookahead point lies within this segment
                direction = (p2 - curr_pt) / dist_to_next
                return curr_pt + direction * remaining_lookahead
            else:
                # Advance to the next segment
                remaining_lookahead -= dist_to_next
                curr_pt = p2
                seg_idx += 1

        # Fallback to the final waypoint if lookahead exceeds track length
        return self.waypoints[-1]

    def control_loop(self):
        if self.robot_pos is None or self.robot_yaw is None:
            return

        cmd = Twist()

        # 1. Check distance to final goal
        dist_to_goal = np.linalg.norm(self.target_goal - self.robot_pos)
        if dist_to_goal < self.goal_tolerance:
            # Reached goal -> Stop
            self.cmd_pub.publish(cmd)
            return

        # 2. Find closest point on U-track and current segment index
        proj_pt, cross_track_err, seg_idx = self.get_closest_track_point(self.robot_pos)

        # 3. Calculate lookahead target point along the track
        lookahead_pt = self.get_lookahead_point(seg_idx, proj_pt)

        # 4. Transform lookahead point into Robot Local Base Frame
        dx = lookahead_pt[0] - self.robot_pos[0]
        dy = lookahead_pt[1] - self.robot_pos[1]

        # Rotate by -robot_yaw
        local_x = dx * math.cos(-self.robot_yaw) - dy * math.sin(-self.robot_yaw)
        local_y = dx * math.sin(-self.robot_yaw) + dy * math.cos(-self.robot_yaw)

        # 5. Pure Pursuit Curvature calculation: gamma = (2 * y) / (L^2)
        curvature = (2.0 * local_y) / (self.L ** 2)

        # 6. Compute control commands
        # Slow down smoothly when turning sharply or when approaching the final goal
        speed_scale = max(0.2, 1.0 - min(abs(curvature), 1.0) * 0.5)
        goal_scale = min(1.0, dist_to_goal / 0.3)  # Decelerate within 30cm of goal

        v_lin = self.v_max * speed_scale * goal_scale
        w_ang = curvature * v_lin

        # Clamp angular velocity limits
        w_ang = max(-self.w_max, min(self.w_max, w_ang))

        cmd.linear.x = v_lin
        cmd.angular.z = w_ang
        # self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitUTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()