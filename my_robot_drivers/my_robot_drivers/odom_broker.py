#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomBrokerNode(Node):
    def __init__(self):
        super().__init__('odom_broker_node')

        # Physical parameters matching the ESP32 firmware exactly
        self.declare_parameter('wheel_radius', 0.0325)       # 65mm diameter
        self.declare_parameter('wheel_separation', 0.23)     # Wheel track width
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Robot Pose State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()

        # ROS 2 Subscribers & Publishers
        # 1. Subscribe to the raw ESP32 joint state topic
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states_raw', 
            self.joint_state_callback, 
            10
        )
        
        # 2. Publisher for re-stamped joint states (consumed by robot_state_publisher)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 3. Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom2', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Differential Drive Odometry Broker Node Initialized with /joint_states re-stamping.")

    def joint_state_callback(self, msg: JointState):
        # Ensure the incoming joint states match our expected wheel joints
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
        except ValueError:
            # If the joints aren't named properly yet, skip this frame
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = current_time

        # Capture a rock-solid unified timestamp from the host PC right now
        now_msg = current_time.to_msg()

        # ===================================================================
        # RE-STAMP & RE-PUBLISH JOINT STATES FOR ROBOT_STATE_PUBLISHER
        # ===================================================================
        msg.header.stamp = now_msg
        self.joint_pub.publish(msg)

        # Get angular velocities (rad/s) from micro-ROS
        left_rad_s = msg.velocity[left_idx]
        right_rad_s = msg.velocity[right_idx]

        # 1. Convert angular velocities to linear velocities (m/s)
        v_left = left_rad_s * self.r
        v_right = right_rad_s * self.r

        # 2. Forward Kinematics formulas
        v_linear = (v_right + v_left) / 2.0
        w_angular = (v_right - v_left) / self.L

        # 3. Integrate pose over time (Runge-Kutta 2 approximation)
        delta_th = w_angular * dt
        self.th += delta_th
        self.x += v_linear * math.cos(self.th + (delta_th / 2.0)) * dt
        self.y += v_linear * math.sin(self.th + (delta_th / 2.0)) * dt

        # 4. Convert yaw heading angle to quaternion for ROS 2 compatibility
        cy = math.cos(self.th * 0.5)
        sy = math.sin(self.th * 0.5)
        quat_z = sy
        quat_w = cy

        # # 5. Broadcast coordinate Transform (odom -> base_footprint) using fresh timestamp
        # t = TransformStamped()
        # t.header.stamp = now_msg
        # t.header.frame_id = self.odom_frame
        # t.child_frame_id = self.base_frame
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = quat_z
        # t.transform.rotation.w = quat_w
        # self.tf_broadcaster.sendTransform(t)

        # 6. Publish /odom Odometry message using IDENTICAL timestamp
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position data
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w

        # Velocity data
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = w_angular

# 6x6 flattened covariance array (36 values)
        odom.pose.covariance = [
            0.001, 0.0,   0.0,   0.0,   0.0,   0.0,    # X variance (low error, but NOT 0)
            0.0,   0.001, 0.0,   0.0,   0.0,   0.0,    # Y variance
            0.0,   0.0,   999.0, 0.0,   0.0,   0.0,    # Z variance (unused in 2D)
            0.0,   0.0,   0.0,   999.0, 0.0,   0.0,    # Roll variance
            0.0,   0.0,   0.0,   0.0,   999.0, 0.0,    # Pitch variance
            0.0,   0.0,   0.0,   0.0,   0.0,   0.005   # Yaw variance (wheel slip error)
        ]
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomBrokerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()