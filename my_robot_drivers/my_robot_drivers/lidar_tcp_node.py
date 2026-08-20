#!/usr/bin/env python3
import math
import socket
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PACKET_SIZE = 22

class LidarTcpNode(Node):
    def __init__(self):
        super().__init__('lds02rr_tcp_node')
        
        # Configuration
        self.declare_parameter('esp32_ip', '192.168.1.12')
        self.declare_parameter('esp32_port', 8889)
        self.declare_parameter('frame_id', 'laser_frame')

        self.ip = self.get_parameter('esp32_ip').value
        self.port = self.get_parameter('esp32_port').value
        self.frame_id = self.get_parameter('frame_id').value

        # FIX 1: Define self.sock right away so it exists when connect_to_esp runs!
        self.sock = None

        # 2. Establish initial connection
        self.connect_to_esp()
        
        # ROS 2 Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Pre-fill standard LaserScan message structure
        self.scan_msg = LaserScan()
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_msg.header.frame_id = self.frame_id
        self.scan_msg.angle_min = 0.0
        self.scan_msg.angle_max = 2.0 * math.pi
        self.scan_msg.angle_increment = (2.0 * math.pi) / 360.0
        self.scan_msg.range_min = 0.12  # 12 cm minimum range
        self.scan_msg.range_max = 3.5   # 3.5 meters max range

        # Persistent byte buffer for handling TCP stream fragmentation safely
        self.raw_buffer = bytearray()

        # Buffers for accumulating 360 degrees
        self.ranges = [float('inf')] * 360

        self.timer = self.create_timer(0.1, self.read_and_publish_loop)

    def connect_to_esp(self):
        """Attempts to establish a clean TCP connection with a timeout."""
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to ESP32 LiDAR Server at {self.ip}:{self.port}...")
                
                # Close any old socket safely before opening a new one
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2.0)  # Stop socket from locking up forever
                try:
                    self.sock.connect((self.ip, self.port))
                    self.get_logger().info("Connected successfully to LiDAR TCP stream!")
                except Exception as e:
                    self.get_logger().error(f"Failed to connect: {e}. Retrying...")
                    time.sleep(2.0)
                    continue

                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.setblocking(False)  # Set non-blocking to work nicely inside ROS timer
                
                self.get_logger().info("Connected to ESP32 LiDAR TCP server successfully!")
                return True
            except (socket.error, socket.timeout) as e:
                self.get_logger().warn(f"Connection failed: {e}. Retrying in 2 seconds...")
                time.sleep(2.0)
        return False

    def reconnect(self):
        """Explicit helper method to handle drops mid-execution."""
        self.get_logger().warn("Re-initiating socket connection sequence...")
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.connect_to_esp()

    def read_and_publish_loop(self):
        """Your timer callback reading raw buffer streams and publishing to /scan"""
        if not self.sock:
            self.reconnect()
            return

        try:
            data = self.sock.recv(2048)
            if not data:
                self.get_logger().warn("ESP32 closed the TCP socket channel. Reconnecting...")
                self.reconnect()
                return
            
            # FIX 2: You MUST append the incoming data to the raw buffer!
            self.raw_buffer.extend(data)
            
            # 2. Synchronize and frame-parse raw buffer streams
            while len(self.raw_buffer) >= PACKET_SIZE:
                sync_index = -1
                for i in range(len(self.raw_buffer) - 1):
                    if self.raw_buffer[i] == 0xFA and 0xA0 <= self.raw_buffer[i+1] <= 0xF9:
                        sync_index = i
                        break

                if sync_index == -1:
                    del self.raw_buffer[:-1]
                    break

                if sync_index > 0:
                    del self.raw_buffer[:sync_index]

                if len(self.raw_buffer) < PACKET_SIZE:
                    break

                # Sift full 22-byte verified frame pack
                packet = self.raw_buffer[:PACKET_SIZE]
                del self.raw_buffer[:PACKET_SIZE]
                
                self.parse_packet(packet)
            
        except BlockingIOError:
            # Socket is in non-blocking mode and no fresh data is available right now. Pass cleanly.
            pass
        except socket.timeout:
            self.get_logger().warn("LiDAR data stream quiet... waiting for packets.")
        except socket.error as e:
            self.get_logger().error(f"Socket transport error: {e}")
            self.reconnect()

    def parse_packet(self, packet):
        packet_code = packet[1]
        start_deg = (packet_code - 0xA0) * 4

        for offset in range(4):
            deg = start_deg + offset
            if 0 <= deg < 360:
                byte_idx = 4 + (offset * 4)
                
                dist_mm = packet[byte_idx] | (packet[byte_idx + 1] << 8)
                invalid_data = (packet[byte_idx + 1] & 0x80) != 0
                
                if dist_mm > 0 and not invalid_data:
                    self.ranges[deg] = dist_mm / 1000.0  
                else:
                    self.ranges[deg] = float('inf')

        if packet_code == 0xF9:
            now = self.get_clock().now().to_msg()
            self.scan_msg.header.stamp = now
            self.scan_msg.ranges = self.ranges
            self.scan_pub.publish(self.scan_msg)
            
            self.ranges = [float('inf')] * 360

def main(args=None):
    rclpy.init(args=args)
    node = LidarTcpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock:
            try:
                node.sock.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()