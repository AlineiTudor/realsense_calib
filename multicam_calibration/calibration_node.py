#!/usr/bin/env python3
"""
Multi-camera extrinsic calibration using an L-shaped ChArUco target.

Each camera sees one face of the L-shape. The known geometry of the L-shape
bridges the two views to compute the camera1-to-camera2 transform.

Math:
    T_cam1_cam2 = T_cam1_boardA @ T_boardA_boardB @ inv(T_cam2_boardB)
"""

import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
import yaml


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('multicam_calibration')

        self._declare_params()
        self._load_params()

        self.bridge = CvBridge()
        self.samples = []

        self._setup_boards()
        self._setup_subscribers()

        self.get_logger().info(
            f'Calibration node ready. Collecting {self.min_samples} samples...'
        )
        self.get_logger().info(
            'Hold the L-shape so each camera sees its board face.'
        )

    def _declare_params(self):
        self.declare_parameter('config_file', '')

    def _load_params(self):
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        if not config_file or not os.path.exists(config_file):
            self.get_logger().fatal(f'Config file not found: {config_file}')
            raise SystemExit(1)

        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)

        self.board_a_cfg = cfg['board_a']
        self.board_b_cfg = cfg['board_b']
        self.l_shape_cfg = cfg['l_shape']
        self.calib_cfg = cfg['calibration']
        self.min_samples = self.calib_cfg['min_samples']
        self.output_file = self.calib_cfg['output_file']

        self.T_boardA_boardB = self._build_l_shape_transform()

    def _build_l_shape_transform(self):
        """Build the 4x4 transform from board_a origin to board_b origin."""
        cfg = self.l_shape_cfg
        tx = cfg['translation_x']
        ty = cfg['translation_y']
        tz = cfg['translation_z']
        rpy = cfg['rotation_rpy']

        R = Rotation.from_euler('xyz', rpy).as_matrix()

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        return T

    def _setup_boards(self):
        self.board_a = self._make_charuco_board(self.board_a_cfg)
        self.board_b = self._make_charuco_board(self.board_b_cfg)

        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.adaptiveThreshWinSizeMin = 3
        self.detector_params.adaptiveThreshWinSizeMax = 23

        self.dict_a = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.board_a_cfg['dictionary'])
        )
        self.dict_b = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.board_b_cfg['dictionary'])
        )
        self.detector_a = cv2.aruco.ArucoDetector(self.dict_a, self.detector_params)
        self.detector_b = cv2.aruco.ArucoDetector(self.dict_b, self.detector_params)

        self.cam1_intrinsics = None
        self.cam2_intrinsics = None

    @staticmethod
    def _make_charuco_board(cfg):
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, cfg['dictionary'])
        )
        return cv2.aruco.CharucoBoard(
            (cfg['squares_x'], cfg['squares_y']),
            cfg['square_length'],
            cfg['marker_length'],
            aruco_dict,
        )

    def _setup_subscribers(self):
        self.sub_img1 = Subscriber(
            self, Image, self.calib_cfg['camera1_image_topic']
        )
        self.sub_info1 = Subscriber(
            self, CameraInfo, self.calib_cfg['camera1_info_topic']
        )
        self.sub_img2 = Subscriber(
            self, Image, self.calib_cfg['camera2_image_topic']
        )
        self.sub_info2 = Subscriber(
            self, CameraInfo, self.calib_cfg['camera2_info_topic']
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_img1, self.sub_info1, self.sub_img2, self.sub_info2],
            queue_size=10,
            slop=0.05,
        )
        self.sync.registerCallback(self._sync_callback)

    def _sync_callback(self, img1_msg, info1_msg, img2_msg, info2_msg):
        if len(self.samples) >= self.min_samples:
            return

        if self.cam1_intrinsics is None:
            self.cam1_intrinsics = self._camera_info_to_intrinsics(info1_msg)
        if self.cam2_intrinsics is None:
            self.cam2_intrinsics = self._camera_info_to_intrinsics(info2_msg)

        img1 = self.bridge.imgmsg_to_cv2(img1_msg, desired_encoding='bgr8')
        img2 = self.bridge.imgmsg_to_cv2(img2_msg, desired_encoding='bgr8')

        T_cam1_boardA = self._detect_and_solve(
            img1, self.board_a, self.detector_a, self.cam1_intrinsics
        )
        T_cam2_boardB = self._detect_and_solve(
            img2, self.board_b, self.detector_b, self.cam2_intrinsics
        )

        if T_cam1_boardA is None or T_cam2_boardB is None:
            return

        T_cam1_cam2 = T_cam1_boardA @ self.T_boardA_boardB @ np.linalg.inv(T_cam2_boardB)
        self.samples.append(T_cam1_cam2)

        n = len(self.samples)
        t = T_cam1_cam2[:3, 3]
        self.get_logger().info(
            f'Sample {n}/{self.min_samples} — '
            f'translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]'
        )

        if n >= self.min_samples:
            self._compute_final_result()

    @staticmethod
    def _camera_info_to_intrinsics(info_msg):
        K = np.array(info_msg.k).reshape(3, 3)
        D = np.array(info_msg.d)
        return K, D

    def _detect_and_solve(self, img, board, detector, intrinsics):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        camera_matrix, dist_coeffs = intrinsics

        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None or len(ids) < 4:
            return None

        charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board,
        )
        if charuco_retval is None or charuco_retval < 6:
            return None

        success, rvec, tvec = cv2.solvePnP(
            board.getChessboardCorners()[charuco_ids.flatten()],
            charuco_corners,
            camera_matrix,
            dist_coeffs,
        )
        if not success:
            return None

        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T

    def _compute_final_result(self):
        translations = np.array([s[:3, 3] for s in self.samples])
        rotations = Rotation.from_matrix([s[:3, :3] for s in self.samples])

        median_t = np.median(translations, axis=0)
        mean_r = rotations.mean()
        rpy = mean_r.as_euler('xyz')

        std_t = np.std(translations, axis=0)
        std_r_deg = np.std(
            [r.as_euler('xyz', degrees=True) for r in rotations], axis=0
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('CALIBRATION RESULT (camera1 -> camera2)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'Translation (xyz): [{median_t[0]:.6f}, {median_t[1]:.6f}, {median_t[2]:.6f}]'
        )
        self.get_logger().info(
            f'Rotation    (rpy): [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]'
        )
        self.get_logger().info(f'Translation std:   [{std_t[0]:.6f}, {std_t[1]:.6f}, {std_t[2]:.6f}]')
        self.get_logger().info(f'Rotation std (deg): [{std_r_deg[0]:.4f}, {std_r_deg[1]:.4f}, {std_r_deg[2]:.4f}]')
        self.get_logger().info('')
        self.get_logger().info('URDF joint (paste into your xacro):')
        self.get_logger().info(
            f'  <origin xyz="{median_t[0]:.6f} {median_t[1]:.6f} {median_t[2]:.6f}" '
            f'rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>'
        )
        self.get_logger().info('=' * 60)

        result = {
            'transform_camera1_to_camera2': {
                'translation': {
                    'x': float(median_t[0]),
                    'y': float(median_t[1]),
                    'z': float(median_t[2]),
                },
                'rotation_rpy': {
                    'roll': float(rpy[0]),
                    'pitch': float(rpy[1]),
                    'yaw': float(rpy[2]),
                },
                'rotation_quaternion': {
                    'x': float(mean_r.as_quat()[0]),
                    'y': float(mean_r.as_quat()[1]),
                    'z': float(mean_r.as_quat()[2]),
                    'w': float(mean_r.as_quat()[3]),
                },
            },
            'statistics': {
                'num_samples': len(self.samples),
                'translation_std': {
                    'x': float(std_t[0]),
                    'y': float(std_t[1]),
                    'z': float(std_t[2]),
                },
                'rotation_std_deg': {
                    'roll': float(std_r_deg[0]),
                    'pitch': float(std_r_deg[1]),
                    'yaw': float(std_r_deg[2]),
                },
            },
        }

        with open(self.output_file, 'w') as f:
            yaml.dump(result, f, default_flow_style=False)
        self.get_logger().info(f'Result saved to {self.output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
