# #!/usr/bin/env python3
# import os
# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image, CameraInfo
# from ament_index_python.packages import get_package_share_directory

# class ESP32CamNode(Node):
#     def __init__(self):
#         super().__init__('esp32_cam_node')

#         # Declare parameters
#         self.declare_parameter('stream_url', 'http://192.168.1.5:81/stream')
#         self.declare_parameter('camera_frame', 'camera_optical_frame')
#         self.declare_parameter('frame_rate', 15.0)

#         # ---------------------------------------------------------------------
#         # LOAD CALIBRATION FROM .NPZ FILE
#         # ---------------------------------------------------------------------
#         pkg_share = get_package_share_directory('my_robot_drivers')
#         default_calib_path = os.path.join(pkg_share, 'config', 'esp32_cam_calibration.npz')
        
#         self.declare_parameter('calibration_file', default_calib_path)
#         calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value

#         self.camera_matrix = None
#         self.dist_coeffs = None

#         if os.path.exists(calib_file):
#             try:
#                 data = np.load(calib_file)
#                 # Match common numpy calibration array keys (e.g. 'camera_matrix' / 'K' and 'dist_coeff' / 'D')
#                 if 'camera_matrix' in data:
#                     self.camera_matrix = data['camera_matrix']
#                 elif 'K' in data:
#                     self.camera_matrix = data['K']

#                 if 'dist_coeff' in data:
#                     self.dist_coeffs = data['dist_coeff']
#                 elif 'D' in data:
#                     self.dist_coeffs = data['D']

#                 self.get_logger().info(f"Successfully loaded calibration matrix from {calib_file}")
#             except Exception as e:
#                 self.get_logger().error(f"Failed to load calibration file: {e}")
#         else:
#             self.get_logger().warn(f"Calibration file not found at {calib_file}. Publishing uncalibrated CameraInfo.")

#         # Parameters
#         self.stream_url = self.get_parameter('stream_url').get_parameter_value().string_value
#         self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
#         fps = self.get_parameter('frame_rate').get_parameter_value().double_value

#         # Initialize Video Capture & Bridge
#         self.cap = cv2.VideoCapture(self.stream_url)
#         self.bridge = CvBridge()

#         # Publishers
#         self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
#         self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

#         # Timer loop
#         timer_period = 1.0 / fps
#         self.timer = self.create_timer(timer_period, self.timer_callback)

#     def timer_callback(self):
#         if not self.cap.isOpened():
#             self.cap.open(self.stream_url)
#             return

#         ret, frame = self.cap.read()
#         if not ret or frame is None:
#             return

#         now = self.get_clock().now().to_msg()

#         # 1. Publish Image Message
#         img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
#         img_msg.header.stamp = now
#         img_msg.header.frame_id = self.camera_frame
#         self.image_pub.publish(img_msg)

#         # 2. Publish CameraInfo Message with Calibrated Intrinsic Data
#         info_msg = CameraInfo()
#         info_msg.header.stamp = now
#         info_msg.header.frame_id = self.camera_frame
#         info_msg.height = frame.shape[0]
#         info_msg.width = frame.shape[1]

#         if self.camera_matrix is not None and self.dist_coeffs is not None:
#             info_msg.k = self.camera_matrix.flatten().tolist()
#             info_msg.d = self.dist_coeffs.flatten().tolist()
#             info_msg.distortion_model = 'plumb_bob'
            
#             # Projection Matrix P (3x4)
#             p = np.zeros((3, 4))
#             p[0:3, 0:3] = self.camera_matrix
#             info_msg.p = p.flatten().tolist()

#         self.info_pub.publish(info_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ESP32CamNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.cap.release()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python.packages import get_package_share_directory

class ESP32CamNode(Node):
    def __init__(self):
        super().__init__('esp32_cam_node')

        # Declare parameters
        self.declare_parameter('stream_url', 'http://192.168.1.5:81/stream')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('frame_rate', 15.0)

        # ---------------------------------------------------------------------
        # LOAD CALIBRATION FROM .NPZ FILE
        # ---------------------------------------------------------------------
        pkg_share = get_package_share_directory('my_robot_drivers')
        default_calib_path = os.path.join(pkg_share, 'config', 'esp32_cam_calibration.npz')
        
        self.declare_parameter('calibration_file', default_calib_path)
        calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        self.camera_matrix = None
        self.dist_coeffs = None

        if os.path.exists(calib_file):
            try:
                data = np.load(calib_file)
                self.get_logger().info(f"NPZ keys found: {list(data.keys())}")

                # 1. Match Camera Matrix Keys
                for k_key in ['camera_matrix', 'K', 'mtx', 'matrix']:
                    if k_key in data:
                        self.camera_matrix = np.array(data[k_key], dtype=np.float64)
                        break

                # 2. Match Distortion Coefficients Keys
                for d_key in ['dist_coeffs', 'dist_coeff', 'dist', 'D']:
                    if d_key in data:
                        self.dist_coeffs = np.array(data[d_key], dtype=np.float64)
                        break

                if self.camera_matrix is not None and self.dist_coeffs is not None:
                    self.get_logger().info(f"Successfully loaded calibration matrix from {calib_file}")
                    self.get_logger().info(f"Camera Matrix (K):\n{self.camera_matrix}")
                    self.get_logger().info(f"Distortion Coeffs (D): {self.dist_coeffs.flatten()}")
                else:
                    self.get_logger().error(f"Calibration file found, but required keys (K/D) were missing!")

            except Exception as e:
                self.get_logger().error(f"Failed to load calibration file: {e}")
        else:
            self.get_logger().warn(f"Calibration file not found at {calib_file}. Publishing uncalibrated CameraInfo.")

        # Parameters
        self.stream_url = self.get_parameter('stream_url').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        fps = self.get_parameter('frame_rate').get_parameter_value().double_value

        # Initialize Video Capture & Bridge
        self.cap = cv2.VideoCapture(self.stream_url)
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Timer loop
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            self.cap.open(self.stream_url)
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        now = self.get_clock().now().to_msg()

        # 1. Publish Image Message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.camera_frame
        self.image_pub.publish(img_msg)

        # 2. Publish CameraInfo Message with Calibrated Intrinsic Data
        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = self.camera_frame
        info_msg.height = frame.shape[0]
        info_msg.width = frame.shape[1]

        if self.camera_matrix is not None and self.dist_coeffs is not None:
            info_msg.k = self.camera_matrix.flatten().tolist()
            info_msg.d = self.dist_coeffs.flatten().tolist()
            info_msg.distortion_model = 'plumb_bob'
            
            # Rectification matrix (3x3 Identity for monocular cameras)
            r = np.eye(3)
            info_msg.r = r.flatten().tolist()

            # Projection Matrix P (3x4)
            p = np.zeros((3, 4))
            p[0:3, 0:3] = self.camera_matrix
            info_msg.p = p.flatten().tolist()

        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()