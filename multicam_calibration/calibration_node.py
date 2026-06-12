#!/usr/bin/env python3
"""
Multi-camera extrinsic calibration via hand-eye (AX=XB) method.

Two ChArUco boards are placed in the scene (no rigid attachment needed).
The camera rig moves to multiple positions while both boards remain visible.
The solver recovers T_cam1_cam2 with no manual measurements.

Math (for each pair of rig positions i, j):
    A_ij @ X = X @ B_ij
where:
    A_ij = T_cam1_boardA_j @ inv(T_cam1_boardA_i)  (cam1 motion)
    B_ij = T_cam2_boardB_j @ inv(T_cam2_boardB_i)  (cam2 motion)
    X    = T_cam1_cam2                               (solved)
"""

import os
import threading

import cv2
import numpy as np
import rclpy
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

        self.pose_pairs = []
        self.last_accepted_T1 = None
        self.last_capture_time = 0.0
        self._processing_lock = threading.Lock()
        self._last_process_time = 0.0

        self._setup_boards()
        self._setup_subscribers()

        self.get_logger().info(
            f'Hand-eye calibration ready. Need {self.min_samples} poses '
            f'from distinct rig positions.'
        )
        self.get_logger().info(
            'Place two ChArUco boards so each camera sees one. '
            'Move the CAMERA RIG between captures (boards stay still).'
        )
        self.get_logger().info(
            f'Each pose must differ by >= {self.min_translation_m:.2f}m AND '
            f'>= {self.min_rotation_deg:.0f}deg. '
            f'Cooldown: {self.capture_cooldown_s:.0f}s between captures.'
        )
        self.get_logger().info(
            'IMPORTANT: Include large rotations (tilt/rotate the rig 15-30 '
            'degrees between positions) for accurate results.'
        )

    def _declare_params(self):
        self.declare_parameter('config_file', '')

    def _load_params(self):
        config_file = (
            self.get_parameter('config_file').get_parameter_value().string_value
        )
        if not config_file or not os.path.exists(config_file):
            self.get_logger().fatal(f'Config file not found: {config_file}')
            raise SystemExit(1)

        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)

        self.board_a_cfg = cfg['board_a']
        self.board_b_cfg = cfg['board_b']
        self.calib_cfg = cfg['calibration']
        self.min_samples = self.calib_cfg['min_samples']
        self.min_translation_m = self.calib_cfg.get('min_translation_m', 0.05)
        self.min_rotation_deg = self.calib_cfg.get('min_rotation_deg', 15.0)
        self.capture_cooldown_s = self.calib_cfg.get('capture_cooldown_s', 2.0)
        self.output_file = self.calib_cfg['output_file']

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
        self.detector_a = cv2.aruco.ArucoDetector(
            self.dict_a, self.detector_params
        )
        self.detector_b = cv2.aruco.ArucoDetector(
            self.dict_b, self.detector_params
        )

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
            queue_size=2,
            slop=0.1,
        )
        self.sync.registerCallback(self._sync_callback)
        self._frame_count = 0
        self._last_log_time = 0.0

    def _sync_callback(self, img1_msg, info1_msg, img2_msg, info2_msg):
        if len(self.pose_pairs) >= self.min_samples:
            return

        # Discard frame if previous one is still being processed
        if not self._processing_lock.acquire(blocking=False):
            return

        try:
            self._frame_count += 1
            now = self.get_clock().now().nanoseconds / 1e9

            # Rate-limit: process at most ~2 fps to avoid CPU saturation
            if now - self._last_process_time < 0.5:
                return
            self._last_process_time = now

            if self.cam1_intrinsics is None:
                self.cam1_intrinsics = self._camera_info_to_intrinsics(info1_msg)
            if self.cam2_intrinsics is None:
                self.cam2_intrinsics = self._camera_info_to_intrinsics(info2_msg)

            img1 = self._imgmsg_to_cv2(img1_msg)
            img2 = self._imgmsg_to_cv2(img2_msg)

            T_cam1_boardA = self._detect_and_solve(
                img1, self.board_a, self.detector_a, self.cam1_intrinsics
            )
            T_cam2_boardB = self._detect_and_solve(
                img2, self.board_b, self.detector_b, self.cam2_intrinsics
            )

            if T_cam1_boardA is None or T_cam2_boardB is None:
                if now - self._last_log_time > 3.0:
                    det_a = T_cam1_boardA is not None
                    det_b = T_cam2_boardB is not None
                    self.get_logger().info(
                        f'Detection — board_a: {"OK" if det_a else "FAIL"}, '
                        f'board_b: {"OK" if det_b else "FAIL"}  '
                        f'(frames: {self._frame_count}, '
                        f'poses: {len(self.pose_pairs)}/{self.min_samples})'
                    )
                    self._last_log_time = now
                return

            if now - self.last_capture_time < self.capture_cooldown_s:
                return

            moved, dt, da = self._movement_from_last(T_cam1_boardA)
            if not moved:
                if now - self._last_log_time > 3.0:
                    self.get_logger().info(
                        f'Both boards detected, waiting for more movement '
                        f'(dt={dt:.3f}m, dr={da:.1f}deg). '
                        f'Need {self.min_translation_m:.2f}m AND '
                        f'{self.min_rotation_deg:.0f}deg.'
                    )
                    self._last_log_time = now
                return

            self.pose_pairs.append((T_cam1_boardA.copy(), T_cam2_boardB.copy()))
            self.last_accepted_T1 = T_cam1_boardA.copy()
            self.last_capture_time = now

            n = len(self.pose_pairs)
            self.get_logger().info(
                f'Pose {n}/{self.min_samples} captured '
                f'(moved {dt:.3f}m, {da:.1f}deg from last)'
            )

            if n >= self.min_samples:
                self._compute_hand_eye()
        finally:
            self._processing_lock.release()

    def _movement_from_last(self, T_cam1_boardA):
        if self.last_accepted_T1 is None:
            return True, 0.0, 0.0

        T_rel = np.linalg.inv(self.last_accepted_T1) @ T_cam1_boardA
        dt = np.linalg.norm(T_rel[:3, 3])
        cos_angle = np.clip((np.trace(T_rel[:3, :3]) - 1) / 2, -1.0, 1.0)
        da = np.degrees(np.arccos(cos_angle))
        moved = dt >= self.min_translation_m and da >= self.min_rotation_deg
        return moved, dt, da

    @staticmethod
    def _imgmsg_to_cv2(msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

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

        charuco_retval, charuco_corners, charuco_ids = (
            cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
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

    @staticmethod
    def _optical_to_link(T_optical):
        """Optical (X right, Y down, Z fwd) -> camera_link (X fwd, Y left, Z up)."""
        R_opt_to_link = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ], dtype=float)
        T_conv = np.eye(4)
        T_conv[:3, :3] = R_opt_to_link
        return T_conv @ T_optical @ np.linalg.inv(T_conv)

    def _compute_hand_eye(self):
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        for T1, T2 in self.pose_pairs:
            # gripper2base = inv(T_cam1_boardA) = T_boardA_cam1
            T_g2b = np.linalg.inv(T1)
            R_gripper2base.append(T_g2b[:3, :3])
            t_gripper2base.append(T_g2b[:3, 3].reshape(3, 1))
            # target2cam = T_cam2_boardB (directly from solvePnP)
            R_target2cam.append(T2[:3, :3])
            t_target2cam.append(T2[:3, 3].reshape(3, 1))

        methods = [
            ('TSAI', cv2.CALIB_HAND_EYE_TSAI),
            ('PARK', cv2.CALIB_HAND_EYE_PARK),
            ('HORAUD', cv2.CALIB_HAND_EYE_HORAUD),
            ('ANDREFF', cv2.CALIB_HAND_EYE_ANDREFF),
            ('DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]

        results_optical = {}
        for name, method in methods:
            try:
                R, t = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    method=method,
                )
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = t.flatten()
                results_optical[name] = T
            except cv2.error as e:
                self.get_logger().warn(f'{name} method failed: {e}')

        if not results_optical:
            self.get_logger().error('All hand-eye methods failed!')
            return

        results_link = {
            name: self._optical_to_link(T)
            for name, T in results_optical.items()
        }

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'Hand-eye calibration complete ({len(self.pose_pairs)} poses)'
        )

        self.get_logger().info('')
        self.get_logger().info(
            'OPTICAL FRAME results (X right, Y down, Z forward):'
        )
        for name, T in results_optical.items():
            t = T[:3, 3]
            rpy = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
            self.get_logger().info(
                f'  {name:12s}  t=[{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]  '
                f'rpy=[{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]'
            )

        self.get_logger().info('')
        self.get_logger().info(
            'CAMERA_LINK FRAME results (X forward, Y left, Z up):'
        )
        for name, T in results_link.items():
            t = T[:3, 3]
            rpy = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
            self.get_logger().info(
                f'  {name:12s}  t=[{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]  '
                f'rpy=[{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]'
            )

        primary = 'PARK' if 'PARK' in results_link else next(
            iter(results_link)
        )
        T_link = results_link[primary]
        T_opt = results_optical[primary]

        link_t = T_link[:3, 3]
        link_r = Rotation.from_matrix(T_link[:3, :3])
        link_rpy = link_r.as_euler('xyz')

        # Consistency check: T_boardA_boardB should be constant across poses
        board_transforms = []
        for T1, T2 in self.pose_pairs:
            T_a2b = np.linalg.inv(T1) @ T_opt @ T2
            board_transforms.append(T_a2b)
        bt_translations = np.array([T[:3, 3] for T in board_transforms])
        bt_rotations = np.array([
            Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
            for T in board_transforms
        ])
        bt_std_t = np.std(bt_translations, axis=0)
        bt_std_r = np.std(bt_rotations, axis=0)

        self.get_logger().info('')
        self.get_logger().info(f'Primary method: {primary}')
        self.get_logger().info(
            f'Consistency check (board-to-board transform std across poses):'
        )
        self.get_logger().info(
            f'  Translation std: [{bt_std_t[0]:.4f}, {bt_std_t[1]:.4f}, '
            f'{bt_std_t[2]:.4f}] m'
        )
        self.get_logger().info(
            f'  Rotation std:    [{bt_std_r[0]:.2f}, {bt_std_r[1]:.2f}, '
            f'{bt_std_r[2]:.2f}] deg'
        )

        self.get_logger().info('')
        self.get_logger().info('URDF joint (paste into your xacro):')
        self.get_logger().info(
            f'  <origin xyz="{link_t[0]:.6f} {link_t[1]:.6f} '
            f'{link_t[2]:.6f}" '
            f'rpy="{link_rpy[0]:.6f} {link_rpy[1]:.6f} '
            f'{link_rpy[2]:.6f}"/>'
        )
        self.get_logger().info('=' * 60)

        result = {
            'transform_camera1_to_camera2': {
                'frame': 'camera_link',
                'method': primary,
                'translation': {
                    'x': float(link_t[0]),
                    'y': float(link_t[1]),
                    'z': float(link_t[2]),
                },
                'rotation_rpy': {
                    'roll': float(link_rpy[0]),
                    'pitch': float(link_rpy[1]),
                    'yaw': float(link_rpy[2]),
                },
                'rotation_quaternion': {
                    'x': float(link_r.as_quat()[0]),
                    'y': float(link_r.as_quat()[1]),
                    'z': float(link_r.as_quat()[2]),
                    'w': float(link_r.as_quat()[3]),
                },
            },
            'all_methods': {},
            'statistics': {
                'num_poses': len(self.pose_pairs),
                'consistency_translation_std': {
                    'x': float(bt_std_t[0]),
                    'y': float(bt_std_t[1]),
                    'z': float(bt_std_t[2]),
                },
                'consistency_rotation_std_deg': {
                    'roll': float(bt_std_r[0]),
                    'pitch': float(bt_std_r[1]),
                    'yaw': float(bt_std_r[2]),
                },
            },
        }

        for name, T in results_link.items():
            t_m = T[:3, 3]
            r_m = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
            result['all_methods'][name] = {
                'translation': [float(t_m[0]), float(t_m[1]), float(t_m[2])],
                'rotation_rpy': [float(r_m[0]), float(r_m[1]), float(r_m[2])],
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
