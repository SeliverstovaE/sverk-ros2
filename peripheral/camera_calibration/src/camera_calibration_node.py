#!/usr/bin/env python3
import json
import os
import threading
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from camera_calibration.srv import (
    GetState,
    ListCaptures,
    RemoveCapture,
    SetAutoCapture,
    SetCalibrationConfig,
)

SUPPORTED_BOARD_TYPES = {"chessboard", "charuco", "circles", "acircles"}
SUPPORTED_CAMERA_MODELS = {"pinhole", "fisheye"}


@dataclass
class CaptureRecord:
    index: int
    stamp_sec: float
    raw_path: str
    overlay_path: str
    board_type: str


@dataclass
class CalibrationConfig:
    board_type: str = "chessboard"
    camera_model: str = "pinhole"
    board_width: int = 9
    board_height: int = 6
    square_size: float = 0.024
    marker_size: float = 0.018
    aruco_dict: str = "DICT_4X4_50"
    min_frames: int = 15
    save_captures: bool = True


class CameraCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_calibration_node")

        self.declare_parameter("image_topic", "/image/raw")
        self.declare_parameter("output_dir", "~/camera_calibrations")
        self.declare_parameter("camera_name", "calibrated_camera")
        self.declare_parameter("frame_id", "camera_optical_1")
        self.declare_parameter("preview_rate_hz", 5.0)
        self.declare_parameter("publish_preview_when_idle", True)
        self.declare_parameter("min_charuco_corners", 10)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.preview_rate_hz = float(self.get_parameter("preview_rate_hz").value)
        self.publish_preview_when_idle = bool(self.get_parameter("publish_preview_when_idle").value)
        self.min_charuco_corners = int(self.get_parameter("min_charuco_corners").value)

        self._lock = threading.Lock()
        self._bridge = CvBridge()

        self._last_frame_bgr: Optional[np.ndarray] = None
        self._image_size: Optional[Tuple[int, int]] = None  # (w, h)

        self._cfg = CalibrationConfig()
        self._session_active = False
        self._session_id: Optional[str] = None
        self._session_dir: Optional[str] = None

        self._objpoints: List[np.ndarray] = []
        self._imgpoints: List[np.ndarray] = []
        self._captures: List[CaptureRecord] = []

        self._auto_capture_enabled = False
        self._auto_capture_period_sec = 5.0
        self._auto_capture_timer = None

        self._calibrated = False
        self._calib_result: Dict[str, object] = {}
        self._last_saved_yaml: Optional[str] = None

        self._base_objp: Optional[np.ndarray] = None
        self._blob_detector = None
        self._aruco_dictionary = None
        self._charuco_board = None
        self._aruco_detector = None
        self._aruco_params = None

        self._status_pub = self.create_publisher(String, "/calibration/status", 10)
        self._state_pub = self.create_publisher(String, "/calibration/state", 10)
        self._processed_image_pub = self.create_publisher(Image, "/calibration/processed_image", 10)

        ok, msg = self._apply_config(self._cfg, reset_session_data=True)
        if not ok:
            self.get_logger().error(msg)

        self._image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_cb,
            qos_profile_sensor_data,
        )

        self.create_service(Trigger, "/calibration/start_session", self._srv_start_session)
        self.create_service(Trigger, "/calibration/reset_session", self._srv_reset_session)
        self.create_service(Trigger, "/calibration/capture", self._srv_capture)
        self.create_service(Trigger, "/calibration/calibrate", self._srv_calibrate)
        self.create_service(Trigger, "/calibration/save", self._srv_save)

        self.create_service(SetCalibrationConfig, "/calibration/set_config", self._srv_set_config)
        self.create_service(SetAutoCapture, "/calibration/set_auto_capture", self._srv_set_auto_capture)
        self.create_service(GetState, "/calibration/get_state", self._srv_get_state)
        self.create_service(ListCaptures, "/calibration/list_captures", self._srv_list_captures)
        self.create_service(RemoveCapture, "/calibration/remove_capture", self._srv_remove_capture)

        if self.preview_rate_hz > 0.0:
            self._preview_timer = self.create_timer(1.0 / self.preview_rate_hz, self._preview_timer_cb)
        else:
            self._preview_timer = None

        self._state_timer = self.create_timer(1.0, self._publish_state)

        self._publish_status("Ready")

    def _image_cb(self, msg: Image) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        with self._lock:
            self._last_frame_bgr = cv_image
            self._image_size = (cv_image.shape[1], cv_image.shape[0])

    def _preview_timer_cb(self) -> None:
        if not self.publish_preview_when_idle and not self._session_active:
            return

        frame = self._get_last_frame_copy()
        if frame is None:
            return

        overlay, _, _ = self._detect_and_draw(frame, for_preview=True)
        self._publish_overlay(overlay)

    def _srv_start_session(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._start_new_session()
        res.success = True
        res.message = f"Session started: {self._session_id}"
        return res

    def _srv_reset_session(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._reset_session(clear_files=False)
        res.success = True
        res.message = "Session reset"
        return res

    def _srv_capture(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._capture_once(publish_result=True)
        res.success = ok
        res.message = msg
        return res

    def _srv_calibrate(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run_calibration()
        res.success = ok
        res.message = msg
        return res

    def _srv_save(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._save_yaml()
        res.success = ok
        res.message = msg
        return res

    def _srv_set_config(self, req: SetCalibrationConfig.Request, res: SetCalibrationConfig.Response) -> SetCalibrationConfig.Response:
        cfg = CalibrationConfig(
            board_type=req.board_type.strip() or self._cfg.board_type,
            camera_model=req.camera_model.strip() or self._cfg.camera_model,
            board_width=int(req.board_width) if req.board_width > 0 else self._cfg.board_width,
            board_height=int(req.board_height) if req.board_height > 0 else self._cfg.board_height,
            square_size=float(req.square_size) if req.square_size > 0 else self._cfg.square_size,
            marker_size=float(req.marker_size) if req.marker_size > 0 else self._cfg.marker_size,
            aruco_dict=req.aruco_dict.strip() or self._cfg.aruco_dict,
            min_frames=int(req.min_frames) if req.min_frames > 0 else self._cfg.min_frames,
            save_captures=bool(req.save_captures),
        )
        ok, msg = self._apply_config(cfg, reset_session_data=True)
        res.success = ok
        res.message = msg
        return res

    def _srv_set_auto_capture(self, req: SetAutoCapture.Request, res: SetAutoCapture.Response) -> SetAutoCapture.Response:
        enable = bool(req.enable)
        period = float(req.period_sec)

        if enable:
            if period < 0.2:
                res.success = False
                res.message = "period_sec must be >= 0.2"
                return res
            self._enable_auto_capture(period)
            self._publish_status(f"Auto-capture enabled ({period:.2f}s)")
            res.success = True
            res.message = f"Auto-capture enabled ({period:.2f}s)"
            return res

        self._disable_auto_capture()
        self._publish_status("Auto-capture disabled")
        res.success = True
        res.message = "Auto-capture disabled"
        return res

    def _srv_get_state(self, req: GetState.Request, res: GetState.Response) -> GetState.Response:
        res.success = True
        res.state_json = json.dumps(self._build_state(), ensure_ascii=False)
        return res

    def _srv_list_captures(self, req: ListCaptures.Request, res: ListCaptures.Response) -> ListCaptures.Response:
        res.success = True
        res.captures = [json.dumps(asdict(c), ensure_ascii=False) for c in self._captures]
        return res

    def _srv_remove_capture(self, req: RemoveCapture.Request, res: RemoveCapture.Response) -> RemoveCapture.Response:
        idx = int(req.index)
        ok, msg = self._remove_capture(idx)
        res.success = ok
        res.message = msg
        return res

    def _apply_config(self, cfg: CalibrationConfig, reset_session_data: bool) -> Tuple[bool, str]:
        if cfg.board_type not in SUPPORTED_BOARD_TYPES:
            return False, f"Unsupported board_type: {cfg.board_type}"
        if cfg.camera_model not in SUPPORTED_CAMERA_MODELS:
            return False, f"Unsupported camera_model: {cfg.camera_model}"
        if cfg.board_width <= 1 or cfg.board_height <= 1:
            return False, "board_width and board_height must be > 1"
        if cfg.square_size <= 0:
            return False, "square_size must be > 0"
        if cfg.board_type == "charuco" and cfg.marker_size <= 0:
            return False, "marker_size must be > 0 for charuco"
        if cfg.min_frames < 3:
            return False, "min_frames must be >= 3"

        self._cfg = cfg
        self._calibrated = False
        self._calib_result = {}
        self._last_saved_yaml = None

        if reset_session_data:
            self._clear_capture_data(keep_session=True)

        self._rebuild_target_helpers()
        self._publish_status(
            f"Config set: board={cfg.board_type} {cfg.board_width}x{cfg.board_height}, "
            f"model={cfg.camera_model}, min_frames={cfg.min_frames}"
        )
        self._publish_state()
        return True, "OK"

    def _rebuild_target_helpers(self) -> None:
        self._base_objp = None
        self._blob_detector = None
        self._aruco_dictionary = None
        self._charuco_board = None
        self._aruco_detector = None
        self._aruco_params = None

        if self._cfg.board_type == "chessboard":
            objp = np.zeros((self._cfg.board_width * self._cfg.board_height, 3), np.float32)
            objp[:, :2] = np.mgrid[0:self._cfg.board_width, 0:self._cfg.board_height].T.reshape(-1, 2)
            objp *= float(self._cfg.square_size)
            self._base_objp = objp
            return

        if self._cfg.board_type in {"circles", "acircles"}:
            self._base_objp = self._make_circles_objpoints(
                self._cfg.board_width, self._cfg.board_height, self._cfg.square_size, self._cfg.board_type == "acircles"
            )
            self._blob_detector = self._make_blob_detector()
            return

        if self._cfg.board_type == "charuco":
            self._aruco_dictionary = self._get_aruco_dictionary(self._cfg.aruco_dict)
            self._aruco_params = self._make_aruco_params()
            self._charuco_board = self._make_charuco_board(
                self._cfg.board_width,
                self._cfg.board_height,
                float(self._cfg.square_size),
                float(self._cfg.marker_size),
                self._aruco_dictionary,
            )
            if hasattr(cv2.aruco, "ArucoDetector"):
                self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dictionary, self._aruco_params)

    def _start_new_session(self) -> None:
        self._reset_session(clear_files=False)
        self._session_active = True
        self._session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = os.path.expanduser(self.output_dir)
        self._session_dir = os.path.join(base_dir, "sessions", self._session_id)
        os.makedirs(self._session_dir, exist_ok=True)
        os.makedirs(os.path.join(self._session_dir, "captures"), exist_ok=True)
        self._publish_status(f"Session started ({self._session_id})")
        self._publish_state()

    def _reset_session(self, clear_files: bool) -> None:
        self._disable_auto_capture()
        if clear_files and self._session_dir and os.path.isdir(self._session_dir):
            try:
                for root, _, files in os.walk(self._session_dir):
                    for f in files:
                        try:
                            os.remove(os.path.join(root, f))
                        except Exception:
                            pass
            except Exception:
                pass
        self._session_active = False
        self._session_id = None
        self._session_dir = None
        self._clear_capture_data(keep_session=False)
        self._publish_state()

    def _clear_capture_data(self, keep_session: bool) -> None:
        self._objpoints = []
        self._imgpoints = []
        self._captures = []
        self._calibrated = False
        self._calib_result = {}
        if not keep_session:
            self._last_saved_yaml = None

    def _enable_auto_capture(self, period_sec: float) -> None:
        self._auto_capture_enabled = True
        self._auto_capture_period_sec = float(period_sec)
        if self._auto_capture_timer is not None:
            try:
                self._auto_capture_timer.cancel()
            except Exception:
                pass
        self._auto_capture_timer = self.create_timer(self._auto_capture_period_sec, self._auto_capture_timer_cb)

    def _disable_auto_capture(self) -> None:
        self._auto_capture_enabled = False
        if self._auto_capture_timer is not None:
            try:
                self._auto_capture_timer.cancel()
            except Exception:
                pass
        self._auto_capture_timer = None

    def _auto_capture_timer_cb(self) -> None:
        if not self._session_active:
            return
        if len(self._objpoints) >= self._cfg.min_frames:
            self._disable_auto_capture()
            self._publish_status("Auto-capture stopped (min_frames reached)")
            return
        self._capture_once(publish_result=False)

    def _get_last_frame_copy(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._last_frame_bgr is None:
                return None
            return self._last_frame_bgr.copy()

    def _capture_once(self, publish_result: bool) -> Tuple[bool, str]:
        if not self._session_active:
            return False, "Session not started"

        frame = self._get_last_frame_copy()
        if frame is None:
            return False, "No image received yet"

        overlay, found, info = self._detect_and_draw(frame, for_preview=False)
        if publish_result:
            self._publish_overlay(overlay)

        if not found:
            msg = f"Target not detected ({info})"
            self._publish_status(msg)
            return False, msg

        if self._image_size is None:
            self._image_size = (frame.shape[1], frame.shape[0])

        ok, save_msg = self._store_capture(frame, overlay)
        if not ok:
            self._publish_status(save_msg)
            return False, save_msg

        msg = f"Captured {len(self._objpoints)}/{self._cfg.min_frames}"
        self._publish_status(msg)
        self._publish_state()

        if self._auto_capture_enabled and len(self._objpoints) >= self._cfg.min_frames:
            self._disable_auto_capture()
            self._publish_status("Auto-capture stopped (min_frames reached)")

        return True, msg

    def _store_capture(self, raw_bgr: np.ndarray, overlay_bgr: np.ndarray) -> Tuple[bool, str]:
        objp, imgp = self._extract_points(raw_bgr)
        if objp is None or imgp is None:
            return False, "Target points could not be extracted"

        self._objpoints.append(objp)
        self._imgpoints.append(imgp)

        stamp = float(self.get_clock().now().nanoseconds) / 1e9
        index = len(self._captures)

        raw_path = ""
        overlay_path = ""
        if self._cfg.save_captures and self._session_dir:
            captures_dir = os.path.join(self._session_dir, "captures")
            raw_path = os.path.join(captures_dir, f"raw_{index:04d}.jpg")
            overlay_path = os.path.join(captures_dir, f"overlay_{index:04d}.jpg")
            try:
                cv2.imwrite(raw_path, raw_bgr)
                cv2.imwrite(overlay_path, overlay_bgr)
            except Exception as e:
                self._objpoints.pop()
                self._imgpoints.pop()
                return False, f"Failed to write capture images: {e}"

        self._captures.append(
            CaptureRecord(
                index=index,
                stamp_sec=stamp,
                raw_path=raw_path,
                overlay_path=overlay_path,
                board_type=self._cfg.board_type,
            )
        )
        return True, "OK"

    def _remove_capture(self, index: int) -> Tuple[bool, str]:
        if index < 0 or index >= len(self._captures):
            return False, "Index out of range"

        rec = self._captures.pop(index)
        try:
            self._objpoints.pop(index)
            self._imgpoints.pop(index)
        except Exception:
            pass

        for p in [rec.raw_path, rec.overlay_path]:
            if p and os.path.isfile(p):
                try:
                    os.remove(p)
                except Exception:
                    pass

        for i, c in enumerate(self._captures):
            c.index = i

        self._calibrated = False
        self._calib_result = {}
        self._publish_status(f"Removed capture {index}")
        self._publish_state()
        return True, "OK"

    def _detect_and_draw(self, frame_bgr: np.ndarray, for_preview: bool) -> Tuple[np.ndarray, bool, str]:
        overlay = frame_bgr.copy()
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        found = False
        info = self._cfg.board_type

        if self._cfg.board_type == "chessboard":
            found, corners = cv2.findChessboardCorners(
                gray,
                (self._cfg.board_width, self._cfg.board_height),
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
            )
            if found and corners is not None:
                corners_refined = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                )
                cv2.drawChessboardCorners(overlay, (self._cfg.board_width, self._cfg.board_height), corners_refined, found)

        elif self._cfg.board_type in {"circles", "acircles"}:
            flags = cv2.CALIB_CB_SYMMETRIC_GRID if self._cfg.board_type == "circles" else cv2.CALIB_CB_ASYMMETRIC_GRID
            found, centers = cv2.findCirclesGrid(
                gray,
                (self._cfg.board_width, self._cfg.board_height),
                flags=flags,
                blobDetector=self._blob_detector,
            )
            if found and centers is not None:
                cv2.drawChessboardCorners(overlay, (self._cfg.board_width, self._cfg.board_height), centers, found)

        elif self._cfg.board_type == "charuco":
            if self._aruco_dictionary is None or self._charuco_board is None:
                found = False
                info = "charuco not configured"
            else:
                corners, ids, _ = self._detect_aruco_markers(gray)
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(overlay, corners, ids)
                    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        corners,
                        ids,
                        gray,
                        self._charuco_board,
                    )
                    if charuco_ids is not None and charuco_corners is not None and len(charuco_ids) >= self.min_charuco_corners:
                        found = True
                        info = f"charuco corners={len(charuco_ids)}"
                        cv2.aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids)
                    else:
                        found = False
                        info = "charuco corners insufficient"
                else:
                    found = False
                    info = "aruco markers not detected"

        self._annotate_overlay(overlay, found, info)
        return overlay, found, info

    def _annotate_overlay(self, img: np.ndarray, found: bool, info: str) -> None:
        color = (0, 255, 0) if found else (0, 0, 255)
        lines = [
            f"session: {self._session_id or 'idle'}",
            f"board: {self._cfg.board_type} {self._cfg.board_width}x{self._cfg.board_height}",
            f"model: {self._cfg.camera_model}",
            f"frames: {len(self._objpoints)}/{self._cfg.min_frames}",
            f"auto: {'on' if self._auto_capture_enabled else 'off'}",
            f"status: {'detected' if found else 'not detected'}",
            f"info: {info}",
        ]
        y = 25
        for line in lines:
            cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
            y += 22

    def _extract_points(self, frame_bgr: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        if self._cfg.board_type == "chessboard":
            found, corners = cv2.findChessboardCorners(
                gray,
                (self._cfg.board_width, self._cfg.board_height),
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
            )
            if not found or corners is None:
                return None, None
            corners_refined = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            if self._base_objp is None:
                return None, None
            objp = self._base_objp.copy()
            imgp = corners_refined.reshape(-1, 2).astype(np.float32)
            return objp, imgp

        if self._cfg.board_type in {"circles", "acircles"}:
            flags = cv2.CALIB_CB_SYMMETRIC_GRID if self._cfg.board_type == "circles" else cv2.CALIB_CB_ASYMMETRIC_GRID
            found, centers = cv2.findCirclesGrid(
                gray,
                (self._cfg.board_width, self._cfg.board_height),
                flags=flags,
                blobDetector=self._blob_detector,
            )
            if not found or centers is None:
                return None, None
            if self._base_objp is None:
                return None, None
            objp = self._base_objp.copy()
            imgp = centers.reshape(-1, 2).astype(np.float32)
            return objp, imgp

        if self._cfg.board_type == "charuco":
            if self._aruco_dictionary is None or self._charuco_board is None:
                return None, None
            corners, ids, _ = self._detect_aruco_markers(gray)
            if ids is None or len(ids) == 0:
                return None, None
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners,
                ids,
                gray,
                self._charuco_board,
            )
            if charuco_ids is None or charuco_corners is None or len(charuco_ids) < self.min_charuco_corners:
                return None, None

            objp = self._charuco_board.chessboardCorners[charuco_ids.flatten()].astype(np.float32)
            imgp = charuco_corners.reshape(-1, 2).astype(np.float32)
            return objp, imgp

        return None, None

    def _run_calibration(self) -> Tuple[bool, str]:
        if len(self._objpoints) < 3:
            return False, f"Not enough frames: {len(self._objpoints)}"
        if self._image_size is None:
            return False, "Unknown image size"

        image_size = self._image_size

        if self._cfg.camera_model == "pinhole":
            return self._calibrate_pinhole(image_size)
        if self._cfg.camera_model == "fisheye":
            return self._calibrate_fisheye(image_size)

        return False, f"Unsupported camera_model: {self._cfg.camera_model}"

    def _calibrate_pinhole(self, image_size: Tuple[int, int]) -> Tuple[bool, str]:
        try:
            rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
                self._objpoints,
                self._imgpoints,
                image_size,
                None,
                None,
            )
            reproj = self._compute_reprojection_error_pinhole(K, D, rvecs, tvecs)
            self._calibrated = True
            self._calib_result = {
                "camera_model": "pinhole",
                "distortion_model": "plumb_bob",
                "rms": float(rms),
                "reprojection_error": float(reproj),
                "K": K,
                "D": D.reshape(-1, 1),
            }
            msg = f"Calibrated (pinhole). reproj_error={reproj:.6f}, rms={rms:.6f}"
            self._publish_status(msg)
            self._publish_state()
            return True, msg
        except Exception as e:
            return False, f"Calibration failed: {e}"

    def _calibrate_fisheye(self, image_size: Tuple[int, int]) -> Tuple[bool, str]:
        try:
            objp = [o.reshape(-1, 1, 3).astype(np.float64) for o in self._objpoints]
            imgp = [i.reshape(-1, 1, 2).astype(np.float64) for i in self._imgpoints]

            K = np.zeros((3, 3), dtype=np.float64)
            D = np.zeros((4, 1), dtype=np.float64)

            flags = (
                cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                | cv2.fisheye.CALIB_CHECK_COND
                | cv2.fisheye.CALIB_FIX_SKEW
            )
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                objp,
                imgp,
                image_size,
                K,
                D,
                None,
                None,
                flags=flags,
                criteria=criteria,
            )
            reproj = self._compute_reprojection_error_fisheye(K, D, rvecs, tvecs)
            self._calibrated = True
            self._calib_result = {
                "camera_model": "fisheye",
                "distortion_model": "equidistant",
                "rms": float(rms),
                "reprojection_error": float(reproj),
                "K": K,
                "D": D.reshape(-1, 1),
            }
            msg = f"Calibrated (fisheye). reproj_error={reproj:.6f}, rms={rms:.6f}"
            self._publish_status(msg)
            self._publish_state()
            return True, msg
        except Exception as e:
            return False, f"Calibration failed: {e}"

    def _compute_reprojection_error_pinhole(self, K, D, rvecs, tvecs) -> float:
        total_err = 0.0
        total_points = 0
        for objp, imgp, rv, tv in zip(self._objpoints, self._imgpoints, rvecs, tvecs):
            proj, _ = cv2.projectPoints(objp, rv, tv, K, D)
            proj2 = proj.reshape(-1, 2)
            err = np.linalg.norm(imgp - proj2, axis=1).sum()
            total_err += float(err)
            total_points += imgp.shape[0]
        return total_err / total_points if total_points > 0 else float("inf")

    def _compute_reprojection_error_fisheye(self, K, D, rvecs, tvecs) -> float:
        total_err = 0.0
        total_points = 0
        for objp, imgp, rv, tv in zip(self._objpoints, self._imgpoints, rvecs, tvecs):
            objp64 = objp.reshape(-1, 1, 3).astype(np.float64)
            imgp64 = imgp.reshape(-1, 1, 2).astype(np.float64)
            proj, _ = cv2.fisheye.projectPoints(objp64, rv, tv, K, D)
            diff = imgp64 - proj
            err = np.linalg.norm(diff.reshape(-1, 2), axis=1).sum()
            total_err += float(err)
            total_points += imgp.shape[0]
        return total_err / total_points if total_points > 0 else float("inf")

    def _save_yaml(self) -> Tuple[bool, str]:
        if not self._calibrated:
            return False, "Calibration not performed"
        if self._image_size is None:
            return False, "Unknown image size"

        base_dir = os.path.expanduser(self.output_dir)
        os.makedirs(base_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(base_dir, f"camera_calibration_{ts}.yaml")

        K = np.array(self._calib_result["K"], dtype=np.float64)
        D = np.array(self._calib_result["D"], dtype=np.float64).reshape(-1, 1)

        w, h = self._image_size
        P = np.zeros((3, 4), dtype=np.float64)
        P[:3, :3] = K
        R = np.eye(3, dtype=np.float64)

        distortion_model = str(self._calib_result.get("distortion_model", "plumb_bob"))

        yaml_data = {
            "calibration_date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "camera_name": self.camera_name,
            "image_width": int(w),
            "image_height": int(h),
            "distortion_model": distortion_model,
            "camera_model": self._cfg.camera_model,
            "board_type": self._cfg.board_type,
            "board_width": int(self._cfg.board_width),
            "board_height": int(self._cfg.board_height),
            "square_size": float(self._cfg.square_size),
            "marker_size": float(self._cfg.marker_size) if self._cfg.board_type == "charuco" else 0.0,
            "aruco_dict": self._cfg.aruco_dict if self._cfg.board_type == "charuco" else "",
            "num_calibration_frames": int(len(self._objpoints)),
            "reprojection_error": float(self._calib_result.get("reprojection_error", 0.0)),
            "camera_matrix": {"rows": 3, "cols": 3, "data": K.reshape(-1).tolist()},
            "distortion_coefficients": {"rows": 1, "cols": int(D.shape[0]), "data": D.reshape(-1).tolist()},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": R.reshape(-1).tolist()},
            "projection_matrix": {"rows": 3, "cols": 4, "data": P.reshape(-1).tolist()},
        }

        try:
            import yaml as _yaml
        except Exception as e:
            return False, f"PyYAML not available: {e}"

        try:
            with open(out_path, "w", encoding="utf-8") as f:
                _yaml.safe_dump(yaml_data, f, sort_keys=False)
        except Exception as e:
            return False, f"Failed to write YAML: {e}"

        latest_path = os.path.join(base_dir, "latest_calibration.yaml")
        try:
            if os.path.lexists(latest_path):
                os.remove(latest_path)
            os.symlink(os.path.basename(out_path), latest_path)
        except Exception:
            try:
                import shutil
                shutil.copyfile(out_path, latest_path)
            except Exception:
                pass

        self._last_saved_yaml = out_path
        self._publish_status(f"Saved: {out_path}")
        self._publish_state()
        return True, out_path

    def _publish_overlay(self, frame_bgr: np.ndarray) -> None:
        try:
            msg = self._bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            msg.header.frame_id = self.frame_id
            self._processed_image_pub.publish(msg)
        except Exception:
            pass

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = str(text)
        self._status_pub.publish(msg)

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps(self._build_state(), ensure_ascii=False)
        self._state_pub.publish(msg)

    def _build_state(self) -> Dict[str, object]:
        return {
            "session_active": bool(self._session_active),
            "session_id": self._session_id or "",
            "session_dir": self._session_dir or "",
            "image_topic": self.image_topic,
            "config": asdict(self._cfg),
            "frames_captured": int(len(self._objpoints)),
            "auto_capture_enabled": bool(self._auto_capture_enabled),
            "auto_capture_period_sec": float(self._auto_capture_period_sec),
            "calibrated": bool(self._calibrated),
            "calibration": {
                "reprojection_error": float(self._calib_result.get("reprojection_error", 0.0)) if self._calibrated else None,
                "rms": float(self._calib_result.get("rms", 0.0)) if self._calibrated else None,
                "distortion_model": self._calib_result.get("distortion_model", ""),
            },
            "last_saved_yaml": self._last_saved_yaml or "",
            "captures": [asdict(c) for c in self._captures],
        }

    def _detect_aruco_markers(self, gray: np.ndarray):
        if self._aruco_detector is not None:
            return self._aruco_detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self._aruco_dictionary, parameters=self._aruco_params)

    @staticmethod
    def _get_aruco_dictionary(name: str):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV aruco module is not available")

        if hasattr(cv2.aruco, name):
            dict_id = getattr(cv2.aruco, name)
            return cv2.aruco.getPredefinedDictionary(dict_id)

        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    @staticmethod
    def _make_aruco_params():
        if hasattr(cv2.aruco, "DetectorParameters"):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    @staticmethod
    def _make_charuco_board(
        squares_x: int,
        squares_y: int,
        square_length: float,
        marker_length: float,
        dictionary,
    ):
        if hasattr(cv2.aruco, "CharucoBoard_create"):
            return cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, dictionary)
        return cv2.aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)

    @staticmethod
    def _make_blob_detector():
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 50000
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False
        params.filterByColor = False
        return cv2.SimpleBlobDetector_create(params)

    @staticmethod
    def _make_circles_objpoints(width: int, height: int, spacing: float, asymmetric: bool) -> np.ndarray:
        objp = np.zeros((width * height, 3), np.float32)
        idx = 0
        for i in range(height):
            for j in range(width):
                if asymmetric:
                    x = (2 * j + (i % 2)) * spacing
                    y = i * spacing
                else:
                    x = j * spacing
                    y = i * spacing
                objp[idx, 0] = x
                objp[idx, 1] = y
                idx += 1
        return objp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()