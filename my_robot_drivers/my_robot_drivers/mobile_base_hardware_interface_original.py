#!/usr/bin/env python3
import math
import socket
import threading
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class MobileBaseHardwareInterface(Node):
    def __init__(self):
        super().__init__('mobile_base_hardware_interface')

        # Physical & TCP parameters (aligned with ESP32 baseTcp on port 8892)
        self.declare_parameter('esp32_ip', '192.168.1.13')
        self.declare_parameter('base_tcp_port', 8892)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.233)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('raw_odom_topic', '/odom2')

        self.esp32_ip = str(self.get_parameter('esp32_ip').value)
        self.base_tcp_port = int(self.get_parameter('base_tcp_port').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_separation').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        raw_odom_topic = str(self.get_parameter('raw_odom_topic').value)

        # Robot Pose State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # TCP Socket State
        self.sock = None
        self.sock_lock = threading.Lock()
        self.running = True

        # ROS 2 Subscribers & Publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.joint_raw_pub = self.create_publisher(JointState, '/joint_states_raw', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, raw_odom_topic, 10)

        # Start background TCP reader thread for incoming "left_rad_s,right_rad_s\n"
        self.rx_thread = threading.Thread(target=self._tcp_reader_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(
            f"MobileBaseHardwareInterface ready on {self.esp32_ip}:{self.base_tcp_port} "
            f"({cmd_vel_topic} -> TCP, TCP -> /joint_states_raw, /joint_states, {raw_odom_topic})"
        )

    def _connect_tcp(self):
        with self.sock_lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(1.0)
                s.connect((self.esp32_ip, self.base_tcp_port))
                s.settimeout(0.2)
                self.sock = s
                self.get_logger().info(
                    f"Connected to ESP32 baseTcp at {self.esp32_ip}:{self.base_tcp_port}"
                )
            except Exception as e:
                self.sock = None
                self.get_logger().warn(
                    f"baseTcp connect failed ({self.esp32_ip}:{self.base_tcp_port}): {e}",
                    throttle_duration_sec=2.0,
                )

    def cmd_vel_callback(self, msg: Twist):
        payload = f"{msg.linear.x:.4f},{msg.angular.z:.4f}\n".encode('utf-8')
        with self.sock_lock:
            if self.sock is None:
                return
            try:
                self.sock.sendall(payload)
            except Exception as e:
                self.get_logger().warn(
                    f"Failed sending /cmd_vel over baseTcp: {e}",
                    throttle_duration_sec=2.0,
                )
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

    def _tcp_reader_loop(self):
        buf = b""
        while self.running and rclpy.ok():
            with self.sock_lock:
                sock = self.sock

            if sock is None:
                self._connect_tcp()
                continue

            try:
                chunk = sock.recv(256)
                if not chunk:
                    with self.sock_lock:
                        if self.sock is sock:
                            try:
                                sock.close()
                            except Exception:
                                pass
                            self.sock = None
                    continue

                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text:
                        self._handle_wheel_line(text)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f"baseTcp read error: {e}", throttle_duration_sec=2.0)
                with self.sock_lock:
                    if self.sock is sock:
                        try:
                            sock.close()
                        except Exception:
                            pass
                        self.sock = None

    def _handle_wheel_line(self, text: str):
        parts = text.split(',')
        if len(parts) < 2:
            return
        try:
            left_rad_s = float(parts[0])
            right_rad_s = float(parts[1])
        except ValueError:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = current_time
        now_msg = current_time.to_msg()

        # Publish /joint_states_raw and /joint_states with host timestamp
        js = JointState()
        js.header.stamp = now_msg
        js.header.frame_id = 'base_link'
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [0.0, 0.0]
        js.velocity = [left_rad_s, right_rad_s]
        self.joint_raw_pub.publish(js)
        self.joint_pub.publish(js)

        # Forward kinematics & RK2 odometry integration
        v_left = left_rad_s * self.r
        v_right = right_rad_s * self.r
        v_linear = (v_right + v_left) / 2.0
        w_angular = (v_right - v_left) / self.L

        delta_th = w_angular * dt
        self.th += delta_th
        self.x += v_linear * math.cos(self.th + (delta_th / 2.0)) * dt
        self.y += v_linear * math.sin(self.th + (delta_th / 2.0)) * dt

        cy = math.cos(self.th * 0.5)
        sy = math.sin(self.th * 0.5)

        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = w_angular
        odom.pose.covariance = [
            0.001, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.001, 0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   999.0, 0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   999.0, 0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   999.0, 0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.005,
        ]
        self.odom_pub.publish(odom)

    def destroy_node(self):
        self.running = False
        with self.sock_lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MobileBaseHardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()