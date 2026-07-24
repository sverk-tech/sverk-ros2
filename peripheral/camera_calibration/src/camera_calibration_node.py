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
        # Авто-деплой: при reproj_error <= auto_deploy_max_reproj калибровка
        # автоматически сохраняется и копируется в config-папку драйвера камеры
        # под именем <sensor_name>_<w>x<h>.yaml (соглашение драйверов *_csi_cam_ros2).
        # Удобный путь — задать ИМЯ пакета драйвера (deploy_driver_package): нода
        # сама найдёт и install/share/<pkg>/config (действует сразу), и
        # src/**/<pkg>/config (переживает colcon build) и запишет YAML в оба.
        # deploy_config_dir — необязательный доп. путь для нестандартных случаев.
        # Авто-отсев выбросов: после калибровки кадры, у которых средняя ошибка
        # репроекции выше max_frame_reproj (px), итеративно выкидываются и
        # калибровка пересчитывается. 0 — отключить отсев.
        self.declare_parameter("max_frame_reproj", 2.0)
        self.declare_parameter("auto_deploy", False)
        self.declare_parameter("auto_deploy_max_reproj", 1.0)
        self.declare_parameter("deploy_driver_package", "a733_csi_cam_ros2")
        self.declare_parameter("workspace_src", "/home/sverk/sverk_ws/src")
        self.declare_parameter("deploy_config_dir", "")
        self.declare_parameter("sensor_name", "imx219")

        # Стартовая конфигурация доски из launch — чтобы не задавать её отдельным
        # вызовом set_config. Особенно нужно для batch-режима (папка с готовыми
        # кадрами). Значения по умолчанию совпадают с CalibrationConfig.
        _dc = CalibrationConfig()
        self.declare_parameter("board_type", _dc.board_type)
        self.declare_parameter("camera_model", _dc.camera_model)
        self.declare_parameter("board_width", _dc.board_width)
        self.declare_parameter("board_height", _dc.board_height)
        self.declare_parameter("square_size", _dc.square_size)
        self.declare_parameter("marker_size", _dc.marker_size)
        self.declare_parameter("aruco_dict", _dc.aruco_dict)
        self.declare_parameter("min_frames", _dc.min_frames)
        # Batch-режим: если задан batch_input_dir — на старте нода прочитает
        # картинки из папки, откалибруется и (при наличии целей) задеплоит,
        # без сервисных вызовов. batch_shutdown — выйти после завершения.
        self.declare_parameter("batch_input_dir", "")
        self.declare_parameter("batch_shutdown", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.preview_rate_hz = float(self.get_parameter("preview_rate_hz").value)
        self.publish_preview_when_idle = bool(self.get_parameter("publish_preview_when_idle").value)
        self.min_charuco_corners = int(self.get_parameter("min_charuco_corners").value)
        self.max_frame_reproj = float(self.get_parameter("max_frame_reproj").value)
        self.auto_deploy = bool(self.get_parameter("auto_deploy").value)
        self.auto_deploy_max_reproj = float(self.get_parameter("auto_deploy_max_reproj").value)
        self.deploy_driver_package = str(self.get_parameter("deploy_driver_package").value)
        self.workspace_src = str(self.get_parameter("workspace_src").value)
        self.deploy_config_dir = str(self.get_parameter("deploy_config_dir").value)
        self.sensor_name = str(self.get_parameter("sensor_name").value)

        self.batch_input_dir = str(self.get_parameter("batch_input_dir").value)
        self.batch_shutdown = bool(self.get_parameter("batch_shutdown").value)

        self._lock = threading.Lock()
        self._bridge = CvBridge()

        self._last_frame_bgr: Optional[np.ndarray] = None
        self._image_size: Optional[Tuple[int, int]] = None  # (w, h)

        self._cfg = CalibrationConfig(
            board_type=str(self.get_parameter("board_type").value),
            camera_model=str(self.get_parameter("camera_model").value),
            board_width=int(self.get_parameter("board_width").value),
            board_height=int(self.get_parameter("board_height").value),
            square_size=float(self.get_parameter("square_size").value),
            marker_size=float(self.get_parameter("marker_size").value),
            aruco_dict=str(self.get_parameter("aruco_dict").value),
            min_frames=int(self.get_parameter("min_frames").value),
            save_captures=_dc.save_captures,
        )
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
        self._last_deployed_yaml: Optional[str] = None

        self._base_objp: Optional[np.ndarray] = None
        self._blob_detector = None
        self._aruco_dictionary = None
        self._charuco_board = None
        self._aruco_detector = None
        self._aruco_params = None

        self._status_pub = self.create_publisher(String, "/camera_calibration/status", 10)
        self._state_pub = self.create_publisher(String, "/camera_calibration/state", 10)
        self._processed_image_pub = self.create_publisher(Image, "/camera_calibration/processed_image", 10)

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
        self.create_service(Trigger, "/calibration/deploy", self._srv_deploy)

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

        # Batch-режим: одноразовый таймер, чтобы нода успела подняться в executor.
        self._batch_timer = None
        if self.batch_input_dir.strip():
            self._batch_timer = self.create_timer(0.5, self._batch_once)

    def _batch_once(self) -> None:
        # Срабатывает один раз: калибровка по папке с готовыми кадрами.
        if self._batch_timer is not None:
            try:
                self._batch_timer.cancel()
            except Exception:
                pass
            self._batch_timer = None
        try:
            self._run_batch(self.batch_input_dir)
        except Exception as e:
            self.get_logger().error(f"Batch failed: {e}")
        if self.batch_shutdown:
            self.get_logger().info("Batch done, shutting down.")
            rclpy.shutdown()

    def _run_batch(self, input_dir: str) -> None:
        import glob
        base = os.path.expanduser(input_dir)
        if not os.path.isdir(base):
            self._publish_status(f"Batch: dir not found: {base}")
            self.get_logger().error(f"Batch: dir not found: {base}")
            return

        files: List[str] = []
        for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.JPG", "*.PNG"):
            files.extend(glob.glob(os.path.join(base, ext)))
        # Игнорируем overlay_*.jpg (это разметка, а не сырые кадры).
        files = sorted(f for f in set(files) if not os.path.basename(f).startswith("overlay_"))
        if not files:
            self._publish_status(f"Batch: no images in {base}")
            self.get_logger().error(f"Batch: no images in {base}")
            return

        self.get_logger().info(
            f"Batch: {len(files)} images, board={self._cfg.board_type} "
            f"{self._cfg.board_width}x{self._cfg.board_height}, model={self._cfg.camera_model}"
        )

        used = 0
        for f in files:
            img = cv2.imread(f)
            if img is None:
                self.get_logger().warn(f"  skip {os.path.basename(f)}: unreadable")
                continue
            self._image_size = (img.shape[1], img.shape[0])
            _, found, info, objp, imgp = self._detect_and_draw(img, for_preview=False)
            if found and objp is not None and imgp is not None:
                self._objpoints.append(objp)
                self._imgpoints.append(imgp)
                used += 1
            else:
                self.get_logger().warn(f"  skip {os.path.basename(f)}: {info}")

        self._publish_status(f"Batch: detected {used}/{len(files)} frames")
        self.get_logger().info(f"Batch: detected {used}/{len(files)} frames")
        if used < 3:
            self._publish_status("Batch: not enough detected frames (<3)")
            self.get_logger().error("Batch: not enough detected frames (<3)")
            return

        ok, msg = self._run_calibration()
        self.get_logger().info(msg)
        if not ok:
            return

        # Всегда сохраняем YAML; деплоим, если заданы цели.
        sok, spath = self._save_yaml()
        self.get_logger().info(f"Saved: {spath}" if sok else f"Save failed: {spath}")
        if self._resolve_deploy_dirs():
            dok, dmsg = self._deploy_current_calibration()
            self.get_logger().info(dmsg)

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

        overlay, _, _, _, _ = self._detect_and_draw(frame, for_preview=True)
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

    def _srv_deploy(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._deploy_current_calibration()
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

        overlay, found, info, objp, imgp = self._detect_and_draw(frame, for_preview=False)
        if publish_result:
            self._publish_overlay(overlay)

        if not found or objp is None or imgp is None:
            msg = f"Target not detected ({info})"
            self._publish_status(msg)
            return False, msg

        if self._image_size is None:
            self._image_size = (frame.shape[1], frame.shape[0])

        ok, save_msg = self._store_capture(frame, overlay, objp, imgp)
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

    def _store_capture(
        self,
        raw_bgr: np.ndarray,
        overlay_bgr: np.ndarray,
        objp: np.ndarray,
        imgp: np.ndarray,
    ) -> Tuple[bool, str]:
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

    def _detect_and_draw(
        self, frame_bgr: np.ndarray, for_preview: bool
    ) -> Tuple[np.ndarray, bool, str, Optional[np.ndarray], Optional[np.ndarray]]:
        # Детекция выполняется один раз: тот же результат используется и для
        # отрисовки overlay, и для извлечения objp/imgp при захвате кадра.
        overlay = frame_bgr.copy()
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        found = False
        info = self._cfg.board_type
        objp: Optional[np.ndarray] = None
        imgp: Optional[np.ndarray] = None

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
                if self._base_objp is not None:
                    objp = self._base_objp.copy()
                    imgp = corners_refined.reshape(-1, 2).astype(np.float32)
                else:
                    found = False

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
                if self._base_objp is not None:
                    objp = self._base_objp.copy()
                    imgp = centers.reshape(-1, 2).astype(np.float32)
                else:
                    found = False

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
                        objp = self._charuco_board.chessboardCorners[charuco_ids.flatten()].astype(np.float32)
                        imgp = charuco_corners.reshape(-1, 2).astype(np.float32)
                    else:
                        found = False
                        info = "charuco corners insufficient"
                else:
                    found = False
                    info = "aruco markers not detected"

        self._annotate_overlay(overlay, found, info)
        return overlay, found, info, objp, imgp

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

    def _run_calibration(self) -> Tuple[bool, str]:
        if len(self._objpoints) < 3:
            return False, f"Not enough frames: {len(self._objpoints)}"
        if self._image_size is None:
            return False, "Unknown image size"

        image_size = self._image_size

        if self._cfg.camera_model in ("pinhole", "fisheye"):
            return self._calibrate_model(self._cfg.camera_model, image_size)

        return False, f"Unsupported camera_model: {self._cfg.camera_model}"

    def _calibrate_model(self, model: str, image_size: Tuple[int, int]) -> Tuple[bool, str]:
        # Единый путь для pinhole и fisheye с итеративным авто-отсевом:
        #   1) калибруем по текущему набору кадров;
        #   2) fisheye дополнительно выкидывает ill-conditioned кадры (CHECK_COND);
        #   3) если max_frame_reproj > 0 и худший кадр по ошибке превышает порог —
        #      выкидываем ЕГО и калибруем заново; повторяем, пока все кадры <= порога
        #      (или пока не останется минимум кадров). Один битый кадр больше не
        #      портит всю калибровку.
        # Исходные захваты (_objpoints/_imgpoints) не меняются — отсев работает
        # на подмножестве индексов.
        try:
            n = len(self._objpoints)
            keep = list(range(n))
            illcond: List[int] = []
            outliers: List[int] = []
            floor = max(3, min(self._cfg.min_frames // 2, n))

            while True:
                objp_list = [self._objpoints[k] for k in keep]
                imgp_list = [self._imgpoints[k] for k in keep]
                if len(keep) < 3:
                    return False, (
                        f"Calibration failed: too few usable frames "
                        f"(dropped {len(illcond)} ill-conditioned, {len(outliers)} outliers)"
                    )

                rms, K, D, errs, dropped_pos = self._fit_once(model, objp_list, imgp_list, image_size)

                # ill-conditioned кадры (только fisheye): убираем и пересчитываем
                if dropped_pos:
                    dropped_global = {keep[p] for p in dropped_pos}
                    illcond.extend(sorted(dropped_global))
                    keep = [g for g in keep if g not in dropped_global]
                    continue

                # авто-отсев выброса: худший кадр выше порога
                if self.max_frame_reproj > 0.0 and len(keep) > floor:
                    worst_pos = int(np.argmax(errs))
                    if errs[worst_pos] > self.max_frame_reproj:
                        outliers.append(keep.pop(worst_pos))
                        continue

                break

            reproj = float(np.mean(errs)) if errs else float("inf")
            distortion_model = "plumb_bob" if model == "pinhole" else "equidistant"
            self._calibrated = True
            self._calib_result = {
                "camera_model": model,
                "distortion_model": distortion_model,
                "rms": float(rms),
                "reprojection_error": reproj,
                "K": K,
                "D": D.reshape(-1, 1),
                "frames_used": len(keep),
                "dropped_outliers": len(outliers),
                "dropped_illcond": len(illcond),
            }
            msg = f"Calibrated ({model}). reproj_error={reproj:.6f}, rms={rms:.6f}, frames={len(keep)}/{n}"
            if outliers:
                msg += f", dropped {len(outliers)} outlier frame(s) (>{self.max_frame_reproj:g}px)"
            if illcond:
                msg += f", dropped {len(illcond)} ill-conditioned frame(s)"
            self._publish_status(msg)
            self._publish_state()
            msg += self._maybe_auto_deploy(reproj)
            return True, msg
        except Exception as e:
            return False, f"Calibration failed: {e}"

    def _fit_once(self, model: str, objp_list, imgp_list, image_size):
        # Одна калибровка по переданному подмножеству кадров.
        # Возвращает (rms, K, D, per_frame_errs, dropped_positions), где
        # dropped_positions — позиции во входных списках, убранные из-за
        # ill-conditioned (только fisheye; для pinhole всегда []).
        # per_frame_errs выровнен по СОХРАНЁННЫМ кадрам (в порядке keep).
        if model == "pinhole":
            op = [o.reshape(-1, 1, 3).astype(np.float32) for o in objp_list]
            ip = [i.reshape(-1, 1, 2).astype(np.float32) for i in imgp_list]
            rms, K, D, rvecs, tvecs = cv2.calibrateCamera(op, ip, image_size, None, None)
            errs = [self._perframe_err_pinhole(op[j], ip[j], K, D, rvecs[j], tvecs[j])
                    for j in range(len(op))]
            return rms, K, D, errs, []

        # fisheye
        op = [o.reshape(-1, 1, 3).astype(np.float64) for o in objp_list]
        ip = [i.reshape(-1, 1, 2).astype(np.float64) for i in imgp_list]
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        base_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW

        keep = list(range(len(op)))
        dropped: List[int] = []
        while True:
            o = [op[k] for k in keep]
            i = [ip[k] for k in keep]
            if len(o) < 3:
                raise RuntimeError("too few usable frames (ill-conditioned)")
            K = np.zeros((3, 3), dtype=np.float64)
            D = np.zeros((4, 1), dtype=np.float64)
            try:
                rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                    o, i, image_size, K, D, None, None,
                    flags=base_flags | cv2.fisheye.CALIB_CHECK_COND, criteria=criteria)
                break
            except cv2.error as e:
                bad = self._parse_illcond_index(str(e))
                if bad is not None and 0 <= bad < len(keep):
                    dropped.append(keep.pop(bad))
                    continue
                rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                    o, i, image_size, K, D, None, None,
                    flags=base_flags, criteria=criteria)
                break

        errs = [self._perframe_err_fisheye(op[keep[j]], ip[keep[j]], K, D, rvecs[j], tvecs[j])
                for j in range(len(keep))]
        return rms, K, D, errs, sorted(dropped)

    @staticmethod
    def _perframe_err_pinhole(objp, imgp, K, D, rv, tv) -> float:
        proj, _ = cv2.projectPoints(objp, rv, tv, K, D)
        return float(np.linalg.norm(imgp.reshape(-1, 2) - proj.reshape(-1, 2), axis=1).mean())

    @staticmethod
    def _perframe_err_fisheye(objp, imgp, K, D, rv, tv) -> float:
        o = objp.reshape(-1, 1, 3).astype(np.float64)
        i = imgp.reshape(-1, 1, 2).astype(np.float64)
        proj, _ = cv2.fisheye.projectPoints(o, rv, tv, K, D)
        return float(np.linalg.norm((i - proj).reshape(-1, 2), axis=1).mean())

    @staticmethod
    def _parse_illcond_index(err_text: str) -> Optional[int]:
        # OpenCV: "CALIB_CHECK_COND - Ill-conditioned matrix for input array N"
        import re
        m = re.search(r"input array (\d+)", err_text)
        if m:
            try:
                return int(m.group(1))
            except ValueError:
                return None
        return None

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
            "num_calibration_frames": int(self._calib_result.get("frames_used", len(self._objpoints))),
            "num_captured_frames": int(len(self._objpoints)),
            "dropped_outliers": int(self._calib_result.get("dropped_outliers", 0)),
            "dropped_illcond": int(self._calib_result.get("dropped_illcond", 0)),
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

    def _maybe_auto_deploy(self, reproj: float) -> str:
        # Вызывается после успешной калибровки. Возвращает суффикс для статуса.
        if not self.auto_deploy:
            return ""
        if reproj > self.auto_deploy_max_reproj:
            note = f" | auto-deploy skipped: reproj {reproj:.4f} > {self.auto_deploy_max_reproj:.4f}"
            self._publish_status(f"Auto-deploy skipped (reproj {reproj:.4f} > {self.auto_deploy_max_reproj:.4f})")
            return note
        ok, msg = self._deploy_current_calibration()
        return f" | {msg}" if ok else f" | auto-deploy failed: {msg}"

    def _resolve_deploy_dirs(self) -> List[str]:
        # Собирает список config-папок, куда деплоить калибровку.
        # По имени пакета драйвера находит и install/share/<pkg>/config
        # (действует сразу, без пересборки), и src/**/<pkg>/config
        # (переживает colcon build). deploy_config_dir добавляется как есть.
        dirs: List[str] = []
        pkg = self.deploy_driver_package.strip()

        if pkg:
            # install/share/<pkg>/config
            try:
                from ament_index_python.packages import get_package_share_directory
                share = get_package_share_directory(pkg)
                dirs.append(os.path.join(share, "config"))
            except Exception:
                pass
            # src/**/<pkg>/config — ищем каталог пакета по package.xml
            if self.workspace_src and os.path.isdir(self.workspace_src):
                import glob
                for pxml in glob.glob(
                    os.path.join(self.workspace_src, "**", pkg, "package.xml"),
                    recursive=True,
                ):
                    dirs.append(os.path.join(os.path.dirname(pxml), "config"))

        if self.deploy_config_dir.strip():
            dirs.append(os.path.expanduser(self.deploy_config_dir.strip()))

        # Дедуп с сохранением порядка.
        seen = set()
        uniq: List[str] = []
        for d in dirs:
            real = os.path.normpath(d)
            if real not in seen:
                seen.add(real)
                uniq.append(real)
        return uniq

    def _deploy_current_calibration(self) -> Tuple[bool, str]:
        if not self._calibrated:
            return False, "Calibration not performed"
        if self._image_size is None:
            return False, "Unknown image size"

        targets = self._resolve_deploy_dirs()
        if not targets:
            return False, (
                "No deploy target: set deploy_driver_package "
                "(e.g. a733_csi_cam_ros2) or deploy_config_dir"
            )

        # Гарантируем актуальный YAML на диске (последний сохранённый может
        # относиться к прошлой калибровке).
        ok, saved = self._save_yaml()
        if not ok:
            return False, f"Save failed: {saved}"

        w, h = self._image_size
        filename = f"{self.sensor_name}_{int(w)}x{int(h)}.yaml"

        import shutil
        written: List[str] = []
        errors: List[str] = []
        for target_dir in targets:
            try:
                os.makedirs(target_dir, exist_ok=True)
                dst = os.path.join(target_dir, filename)
                shutil.copyfile(saved, dst)
                written.append(dst)
            except Exception as e:
                errors.append(f"{target_dir}: {e}")

        if not written:
            return False, f"Deploy failed for all targets: {'; '.join(errors)}"

        self._last_deployed_yaml = written[0]
        msg = f"Deployed {filename} to {len(written)} location(s): " + ", ".join(written)
        if errors:
            msg += f" | skipped: {'; '.join(errors)}"
        self._publish_status(msg)
        self._publish_state()
        return True, msg

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
                "frames_used": self._calib_result.get("frames_used") if self._calibrated else None,
                "dropped_outliers": self._calib_result.get("dropped_outliers") if self._calibrated else None,
                "dropped_illcond": self._calib_result.get("dropped_illcond") if self._calibrated else None,
            },
            "max_frame_reproj": float(self.max_frame_reproj),
            "last_saved_yaml": self._last_saved_yaml or "",
            "last_deployed_yaml": self._last_deployed_yaml or "",
            "auto_deploy": bool(self.auto_deploy),
            "auto_deploy_max_reproj": float(self.auto_deploy_max_reproj),
            "deploy_driver_package": self.deploy_driver_package,
            "deploy_config_dir": self.deploy_config_dir,
            "deploy_targets": self._resolve_deploy_dirs(),
            "sensor_name": self.sensor_name,
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
        # В batch-режиме нода могла сама вызвать rclpy.shutdown() — не дублируем.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()