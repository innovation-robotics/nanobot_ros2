import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

deg_to_rad = math.pi / 180.0

class ArucoLocalizationPublisher(Node):
    def __init__(self):
        super().__init__('aruco_localization_publisher')

        self.declare_parameter('marker_size', 0.18)
        self.declare_parameter('object_marker_size', 0.03)
        self.declare_parameter('dictionary_id', 'DICT_4X4_250')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('map_frame', 'map')

        self.marker_size = self.get_parameter('marker_size').value
        self.object_marker_size = self.get_parameter('object_marker_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.dict_name = self.get_parameter('dictionary_id').value

        # Standard known static map poses for markers in 'map' frame (X, Y, Yaw_rad)
        # Match these to your static_transform_publisher launch file
        # self.marker_map_poses = {
        #     0: {'pos': np.array([0.0, 0.0, 0.0]), 'yaw': -45.0 * np.pi / 180.0},
        #     1: {'pos': np.array([2.42, 0.0, 0.0]), 'yaw': 45.0 * np.pi / 180.0}
        # }
        self.marker_map_poses = {
            0: {'pos': np.array([0.0, 0.0, 0.1275]), 'rot': np.array([-45.0*deg_to_rad, 180.0 * deg_to_rad, -90.0 * deg_to_rad])},
            1: {'pos': np.array([2.13, 0.0, 0.1275]), 'rot': np.array([45.0*deg_to_rad, 180.0 * deg_to_rad, -90.0 * deg_to_rad])},           
            2: {'pos': np.array([2.13, 1.51, 0.1275]), 'rot': np.array([0.0, 0.0, 90.0 * deg_to_rad])}            
        }

        # Initialize ArUco Detector
        dict_enum = getattr(cv2.aruco, self.dict_name, cv2.aruco.DICT_4X4_250)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_enum)

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.params = cv2.aruco.DetectorParameters()
        else:
            self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        else:
            self.detector = None

        self.bridge = CvBridge()

        # Camera Intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers & Publishers
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publish Pose to be consumed by robot_localization EKF
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/aruco/pose', 10)
        self.object20_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/aruco/pose18', 10)
        self.pub_result = self.create_publisher(Image, '/aruco/result', 10)

        self.get_logger().info("ArUco EKF Pose Publisher Node Started.")

    def info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            half_s = self.marker_size / 2.0
            obj_points = np.array([
                [-half_s,  half_s, 0],
                [ half_s,  half_s, 0],
                [ half_s, -half_s, 0],
                [-half_s, -half_s, 0]
            ], dtype=np.float32)

            for i in range(len(ids)):
                marker_id = ids[i][0]
                
                # Process only known map markers
                if marker_id in self.marker_map_poses:
                    img_points = corners[i][0].astype(np.float32)
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, img_points, self.camera_matrix, self.dist_coeffs
                    )

                    if success:
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                        self.publish_aruco_pose(rvec, tvec, marker_id, msg.header.stamp)
                elif marker_id == 18:
                    self.publish_object18_pose(corners[i][0], msg.header.stamp)

        self.pub_result.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

    def publish_object18_pose(self, corners, timestamp):
        half_s = self.object_marker_size / 2.0
        img_points = corners.astype(np.float32)
        obj_points = np.array([
            [-half_s,  half_s, 0],
            [ half_s,  half_s, 0],
            [ half_s, -half_s, 0],
            [-half_s, -half_s, 0]
        ], dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
        # 5. Build and publish PoseWithCovarianceStamped
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        T_cam_marker = tvec.reshape((3, 1))

        # 2. Position of Camera Optical Frame relative to Marker
        R_marker_cam = R_cam_marker.T
        T_marker_cam = -np.dot(R_marker_cam, T_cam_marker)

        # 3. Apply the URDF joint rotation (rpy="-1.570796327 0 -1.570796327")
        # Rotates vectors from camera_optical_frame back to base_link alignment
        R_base_camera = R.from_euler('xyz', [-np.pi/2, 0.0, -np.pi/2]).as_matrix()  # from the urdf rpy
        T_base_camera = np.array([[0.013], [0.0], [0.072]]) # 6cm forward on chassis

        R_cam_base = R_base_camera.T
        T_cam_base = -np.dot(R_cam_base, T_base_camera)

        R_base_link0 = R.from_euler('xyz', [0.0, 0.0, -np.pi/2]).as_matrix()  # from the urdf rpy
        T_base_link0 = np.array([[-0.037], [0.0], [0.0965]]) # 6cm forward on chassis

        R_marker_link0 = np.dot(R_marker_cam, np.dot(R_cam_base, R_base_link0))
        T_marker_link0 = np.dot(R_marker_cam, np.dot(R_cam_base,T_base_link0)) + np.dot(R_marker_cam, T_cam_base) + T_marker_cam

        R_link0_marker = R_marker_link0.T
        T_link0_marker = -np.dot(R_link0_marker, T_marker_link0)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.map_frame

        pose_msg.pose.pose.position.x = float(T_link0_marker[0][0])
        pose_msg.pose.pose.position.y = float(T_link0_marker[1][0])
        pose_msg.pose.pose.position.z = float(T_link0_marker[2][0])

        quat = R.from_matrix(R_link0_marker).as_quat()
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        cov = np.zeros((6, 6), dtype=np.float64)
        np.fill_diagonal(cov, [0.02, 0.02, 0.02, 0.05, 0.05, 0.05])
        pose_msg.pose.covariance = cov.flatten().tolist()

        self.object20_pose_pub.publish(pose_msg)

        return
    
    def publish_aruco_pose(self, rvec, tvec, marker_id, timestamp):
        # 4. Transform into Map frame using known Marker position in Map
        marker_map = self.marker_map_poses[marker_id]
        rot = marker_map['rot'].flatten()

        R_map_marker = R.from_euler('ZYX', rot).as_matrix()     #classic yaw pitch roll, (intrinsic rotation in ZYX order)
        T_map_marker = marker_map['pos'].reshape((3, 1))

        # 1. Pose of Marker in Camera Optical Frame from solvePnP
        # transform a point from the marker frame to the camera frame
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        T_cam_marker = tvec.reshape((3, 1))

        # 2. Position of Camera Optical Frame relative to Marker
        R_marker_cam = R_cam_marker.T
        T_marker_cam = -np.dot(R_marker_cam, T_cam_marker)

        # 3. Apply the URDF joint rotation (rpy="-1.570796327 0 -1.570796327")
        # Rotates vectors from camera_optical_frame back to base_link alignment
        R_base_camera = R.from_euler('xyz', [-np.pi/2, 0.0, -np.pi/2]).as_matrix()  # from the urdf rpy
        T_base_camera = np.array([[0.013], [0.0], [0.072]]) # 6cm forward on chassis

        R_camera_base = R_base_camera.T
        T_camera_base = -np.dot(R_camera_base, T_base_camera)

        # t_marker_base = np.dot(R_base_camera, t_marker_cam) - T_base_camera

        R_map_base = np.dot(R_map_marker, np.dot(R_marker_cam, R_camera_base))
        T_map_base = np.dot(R_map_marker, np.dot(R_marker_cam,T_camera_base)) + np.dot(R_map_marker, T_marker_cam) + T_map_marker


        # Combined orientation for robot in Map frame
        R_map_robot = R_map_base
        # Final Robot Base position in Map frame
        t_map_robot = T_map_base


        # 5. Build and publish PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.map_frame

        pose_msg.pose.pose.position.x = float(t_map_robot[0][0])
        pose_msg.pose.pose.position.y = float(t_map_robot[1][0])
        pose_msg.pose.pose.position.z = 0.0 #float(t_map_robot[2][0])

        quat = R.from_matrix(R_map_robot).as_quat()
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # cov = np.zeros((6, 6), dtype=np.float64)
        # np.fill_diagonal(cov, [0.02, 0.02, 0.02, 0.05, 0.05, 0.05])
        # pose_msg.pose.covariance = cov.flatten().tolist()



        # 1. Calculate Euclidean distance from camera to marker
        distance = math.sqrt(T_cam_marker[0]**2 + T_cam_marker[1]**2 + T_cam_marker[2]**2)

        # 2. Base covariance for close-range detection (< 1.0 meter)
        base_pos_var = 0.01   # ~10cm confidence
        base_yaw_var = 0.05   # ~12 deg confidence

        # 3. Dynamic Covariance Scaling
        # Option A: Hard Threshold (> 1.5 meters -> extremely high uncertainty)
        if distance > 1.5:
            pos_var = 9999.0  # EKF will ignore position completely
            yaw_var = 9999.0  # EKF will ignore orientation completely
        else:
            # Option B: Quadratic scaling with distance (Variance scales with distance^2)
            scale_factor = (distance / 1.0) ** 2
            pos_var = base_pos_var * scale_factor
            yaw_var = base_yaw_var * scale_factor

        # 4. Populate 6x6 Covariance Matrix (Flattened to 36 elements)
        cov = [0.0] * 36
        cov[0]  = pos_var    # X variance
        cov[7]  = pos_var    # Y variance
        cov[14] = 9999.0     # Z (unused in 2D mode)
        cov[21] = 9999.0     # Roll
        cov[28] = 9999.0     # Pitch
        cov[35] = yaw_var    # Yaw variance

        pose_msg.pose.covariance = cov

        self.pose_pub.publish(pose_msg)


def main():
    rclpy.init()
    node = ArucoLocalizationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()