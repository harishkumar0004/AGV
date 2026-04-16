"""
Real-time AprilTag detector tuned for low-power hardware such as Raspberry Pi.

The key design choice is to keep camera capture and tag detection separated:
- a capture loop always grabs the newest frame from the camera
- the detection loop always processes only the latest frame available

That avoids the common Raspberry Pi problem where slow detection causes a queue
of stale frames, making the live preview look delayed.
"""

import json
import math
import os
import threading
import time
from collections import deque
from enum import Enum
from typing import Callable, Dict, Optional, Tuple

import cv2
import numpy as np

try:
    from PyQt5.QtCore import QThread
except ImportError:
    QThread = None

try:
    import apriltag

    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("[WARNING] apriltag module not available. Install: pip install apriltag")


class DetectorState(Enum):
    """Detector lifecycle states."""

    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    RUNNING = "RUNNING"
    STOPPED = "STOPPED"
    ERROR = "ERROR"


if QThread is not None:
    class _DetectorQThread(QThread):
        """Run camera detection inside a Qt-managed thread when PyQt is available."""

        def __init__(self, target: Callable[[], None]):
            super().__init__()
            self._target = target

        def run(self) -> None:
            self._target()


class AprilTagDetectorRealtime:
    """
    Real-time AprilTag detector running in the background.

    Defaults are automatically reduced on Raspberry Pi so the preview stays
    fresh even when AprilTag detection takes longer than a desktop.
    """

    def __init__(
        self,
        layout_file: str = "apriltag_layout.json",
        camera_id: int = 0,
        tag_family: str = "tag36h11",
        confidence_threshold: float = 0.5,
        detection_smoothing: int = 3,
        capture_width: Optional[int] = None,
        capture_height: Optional[int] = None,
        camera_fps: Optional[int] = None,
        preview_max_fps: Optional[float] = None,
    ):
        """
        Initialize the detector.

        Args:
            layout_file: Path to apriltag_layout.json defining tag positions
            camera_id: OpenCV camera device ID
            tag_family: AprilTag family ('tag36h11', 'tag25h9', etc.)
            confidence_threshold: Reserved for future filtering logic
            detection_smoothing: Reserved for future smoothing logic
            capture_width: Camera capture width override
            capture_height: Camera capture height override
            camera_fps: Camera FPS override
            preview_max_fps: Maximum FPS for annotated preview generation
        """
        self.layout_file = layout_file
        self.camera_id = camera_id
        self.tag_family = tag_family
        self.confidence_threshold = confidence_threshold
        self.detection_smoothing = detection_smoothing

        self.is_raspberry_pi = self._detect_raspberry_pi()
        default_width, default_height = (320, 240) if self.is_raspberry_pi else (640, 480)
        default_camera_fps = 20 if self.is_raspberry_pi else 30
        default_preview_fps = 8.0 if self.is_raspberry_pi else 12.0

        self.capture_width = capture_width or self._env_int("AGV_CAMERA_WIDTH", default_width)
        self.capture_height = capture_height or self._env_int("AGV_CAMERA_HEIGHT", default_height)
        self.camera_fps = camera_fps or self._env_int("AGV_CAMERA_FPS", default_camera_fps)
        self.preview_max_fps = preview_max_fps or self._env_float("AGV_PREVIEW_MAX_FPS", default_preview_fps)
        self.preview_interval_sec = 1.0 / max(self.preview_max_fps, 1.0)
        self.use_mjpeg = self._env_bool("AGV_CAMERA_USE_MJPEG", self.is_raspberry_pi)

        self.detection_width = self._env_int("AGV_DETECTION_WIDTH", self.capture_width)
        self.detection_height = self._env_int("AGV_DETECTION_HEIGHT", self.capture_height)
        self.detector_threads = self._env_int(
            "AGV_APRILTAG_THREADS",
            max(1, min(os.cpu_count() or 1, 2 if self.is_raspberry_pi else 4)),
        )
        self.detector_decimate = self._env_float(
            "AGV_APRILTAG_DECIMATE",
            2.5 if self.is_raspberry_pi else 2.0,
        )

        self.tag_layout: Dict[int, Tuple[int, int]] = {}
        self._load_layout()

        self.detector_thread = None
        self._capture_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_ready = threading.Event()
        self._lock = threading.Lock()

        self.state = DetectorState.IDLE
        self.fps = 0.0
        self.frame_count = 0
        self.last_detection_time = 0.0

        self.cap: Optional[cv2.VideoCapture] = None
        self.detector: Optional["apriltag.Detector"] = None

        self.last_detected_tags: Dict[int, Dict] = {}
        self.detection_history = deque(maxlen=10)
        self._latest_preview_frame = None
        self._latest_raw_frame = None
        self._latest_raw_frame_id = 0
        self._capture_error: Optional[str] = None
        self._last_preview_update = 0.0
        self._last_preview_signature: Tuple[int, ...] = tuple()

        self.on_tag_detected: Optional[Callable[[int, Tuple[int, int]], None]] = None
        self.on_frame_processed: Optional[Callable[[int, float], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None

        self._detected_tag_id: Optional[int] = None
        self._tag_detection_event = threading.Event()

    @staticmethod
    def _detect_raspberry_pi() -> bool:
        """Best-effort Raspberry Pi detection with env override."""
        if os.getenv("AGV_FORCE_RPI_CAMERA", "").strip() in {"1", "true", "TRUE", "yes", "YES"}:
            return True

        model_paths = (
            "/proc/device-tree/model",
            "/sys/firmware/devicetree/base/model",
        )
        for path in model_paths:
            try:
                with open(path, "r", encoding="utf-8", errors="ignore") as handle:
                    if "raspberry pi" in handle.read().lower():
                        return True
            except OSError:
                continue
        return False

    @staticmethod
    def _env_int(name: str, default: int) -> int:
        value = os.getenv(name)
        if value is None:
            return default
        try:
            return int(value)
        except ValueError:
            return default

    @staticmethod
    def _env_float(name: str, default: float) -> float:
        value = os.getenv(name)
        if value is None:
            return default
        try:
            return float(value)
        except ValueError:
            return default

    @staticmethod
    def _env_bool(name: str, default: bool) -> bool:
        value = os.getenv(name)
        if value is None:
            return default
        return value.strip().lower() in {"1", "true", "yes", "on"}

    def wait_until_running(self, timeout: float = 5.0) -> bool:
        """Wait until the detector reaches RUNNING or ERROR."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                current_state = self.state
            if current_state == DetectorState.RUNNING:
                return True
            if current_state == DetectorState.ERROR:
                return False
            time.sleep(0.05)
        return False

    def _load_layout(self) -> bool:
        """Load tag layout from JSON file."""
        try:
            if not os.path.isfile(self.layout_file):
                self._error(f"Layout file not found: {self.layout_file}")
                return False

            with open(self.layout_file, "r", encoding="utf-8") as handle:
                entries = json.load(handle)

            self.tag_layout.clear()
            for entry in entries:
                tag_id = int(entry["id"])
                row = int(entry["row"])
                col = int(entry["col"])
                self.tag_layout[tag_id] = (row, col)

            print(f"[AprilTagDetector] Loaded {len(self.tag_layout)} tags from {self.layout_file}")
            return True
        except json.JSONDecodeError as exc:
            self._error(f"Failed to parse layout JSON: {exc}")
            return False
        except Exception as exc:
            self._error(f"Failed to load layout: {exc}")
            return False

    def start(self) -> bool:
        """Start the detector thread."""
        if self.state in (DetectorState.RUNNING, DetectorState.INITIALIZING):
            print("[AprilTagDetector] Already running or initializing")
            return False

        self._stop_event.clear()
        self._frame_ready.clear()
        self._capture_error = None

        try:
            if QThread is not None:
                self.detector_thread = _DetectorQThread(self._detection_loop)
            else:
                self.detector_thread = threading.Thread(target=self._detection_loop, daemon=True)
            self.detector_thread.start()
            print("[AprilTagDetector] Detector thread started")
            return True
        except Exception as exc:
            self._error(f"Failed to start detector thread: {exc}")
            return False

    def stop(self) -> None:
        """Stop the detector thread gracefully."""
        if self.state in (DetectorState.IDLE, DetectorState.STOPPED):
            print("[AprilTagDetector] Not running, nothing to stop")
            return

        print("[AprilTagDetector] Stopping detector...")
        self._stop_event.set()
        self._frame_ready.set()

        if self.detector_thread:
            if hasattr(self.detector_thread, "isRunning"):
                if self.detector_thread.isRunning():
                    self.detector_thread.wait(5000)
            elif self.detector_thread.is_alive():
                self.detector_thread.join(timeout=5.0)

        self._cleanup()
        self._set_state(DetectorState.STOPPED)

    def _cleanup(self) -> None:
        """Release camera and thread resources."""
        self._frame_ready.set()

        cap = self.cap
        self.cap = None
        if cap and cap.isOpened():
            cap.release()

        capture_thread = self._capture_thread
        if capture_thread and capture_thread.is_alive() and capture_thread is not threading.current_thread():
            capture_thread.join(timeout=1.0)
        self._capture_thread = None

        with self._lock:
            self.last_detected_tags.clear()
            self._latest_preview_frame = None
            self._latest_raw_frame = None
            self._latest_raw_frame_id = 0
            self._last_preview_update = 0.0
            self._last_preview_signature = tuple()

    def _set_state(self, new_state: DetectorState) -> None:
        """Update detector state."""
        with self._lock:
            if self.state != new_state:
                old_state = self.state
                self.state = new_state
                print(f"[AprilTagDetector] State: {old_state.value} -> {new_state.value}")

    def _error(self, msg: str) -> None:
        """Log error and emit callback."""
        print(f"[AprilTagDetector] ERROR: {msg}")
        self._set_state(DetectorState.ERROR)
        if self.on_error:
            self.on_error(msg)

    def _open_camera(self) -> Optional[cv2.VideoCapture]:
        """Open the camera with low-latency defaults."""
        candidate_backends = []
        if hasattr(cv2, "CAP_V4L2"):
            candidate_backends.append(cv2.CAP_V4L2)
        candidate_backends.append(None)

        for backend in candidate_backends:
            try:
                cap = cv2.VideoCapture(self.camera_id, backend) if backend is not None else cv2.VideoCapture(self.camera_id)
            except TypeError:
                cap = cv2.VideoCapture(self.camera_id)

            if cap and cap.isOpened():
                if self.use_mjpeg and hasattr(cv2, "CAP_PROP_FOURCC"):
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
                cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
                return cap

            if cap:
                cap.release()

        return None

    def _camera_capture_loop(self) -> None:
        """Continuously grab frames and keep only the newest one."""
        consecutive_failures = 0

        while not self._stop_event.is_set():
            cap = self.cap
            if cap is None or not cap.isOpened():
                return

            ret, frame = cap.read()
            if not ret:
                consecutive_failures += 1
                if consecutive_failures >= 10:
                    self._capture_error = f"Failed to read frames from camera {self.camera_id}"
                    self._frame_ready.set()
                    return
                time.sleep(0.02)
                continue

            consecutive_failures = 0
            with self._lock:
                self._latest_raw_frame = frame
                self._latest_raw_frame_id += 1
            self._frame_ready.set()

    def _wait_for_next_frame(
        self,
        last_processed_frame_id: int,
        timeout_sec: float = 0.25,
    ):
        """Wait for a newer frame than the one already processed."""
        deadline = time.time() + timeout_sec

        while not self._stop_event.is_set():
            with self._lock:
                frame = self._latest_raw_frame
                frame_id = self._latest_raw_frame_id
                capture_error = self._capture_error

            if capture_error:
                return None, last_processed_frame_id, capture_error

            if frame is not None and frame_id != last_processed_frame_id:
                return frame, frame_id, None

            remaining = deadline - time.time()
            if remaining <= 0:
                return None, last_processed_frame_id, None

            self._frame_ready.wait(timeout=min(remaining, 0.05))
            self._frame_ready.clear()

        return None, last_processed_frame_id, None

    def _prepare_detection_frame(self, frame):
        """Resize the frame for faster AprilTag detection when configured."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_height, frame_width = gray.shape[:2]

        target_width = max(1, min(self.detection_width, frame_width))
        target_height = max(1, min(self.detection_height, frame_height))
        scale_x = 1.0
        scale_y = 1.0

        if target_width != frame_width or target_height != frame_height:
            gray = cv2.resize(gray, (target_width, target_height), interpolation=cv2.INTER_AREA)
            scale_x = frame_width / float(target_width)
            scale_y = frame_height / float(target_height)

        return gray, scale_x, scale_y

    def _detection_loop(self) -> None:
        """
        Main detection loop.

        Frames are captured continuously in a companion thread so this loop
        always works on the latest frame available.
        """
        try:
            self._set_state(DetectorState.INITIALIZING)

            if not APRILTAG_AVAILABLE:
                self._error("apriltag module not installed")
                return

            self.cap = self._open_camera()
            if not self.cap or not self.cap.isOpened():
                self._error(f"Failed to open camera {self.camera_id}")
                return

            options = apriltag.DetectorOptions(
                families=self.tag_family,
                nthreads=self.detector_threads,
                quad_decimate=self.detector_decimate,
                quad_blur=0.0,
                refine_edges=True,
                refine_decode=False,
                refine_pose=False,
            )
            self.detector = apriltag.Detector(options=options)

            self._capture_thread = threading.Thread(target=self._camera_capture_loop, daemon=True)
            self._capture_thread.start()

            print(
                "[AprilTagDetector] Camera initialized "
                f"({self.capture_width}x{self.capture_height} @ {self.camera_fps}fps, "
                f"detect={self.detection_width}x{self.detection_height}, "
                f"decimate={self.detector_decimate})"
            )
            self._set_state(DetectorState.RUNNING)

            last_fps_time = time.time()
            fps_count = 0
            last_processed_frame_id = -1

            while not self._stop_event.is_set():
                frame, last_processed_frame_id, capture_error = self._wait_for_next_frame(
                    last_processed_frame_id=last_processed_frame_id,
                    timeout_sec=0.25,
                )

                if capture_error:
                    self._error(capture_error)
                    return

                if frame is None:
                    continue

                gray, detection_scale_x, detection_scale_y = self._prepare_detection_frame(frame)
                detections = self.detector.detect(gray)
                self._process_detections(detections, frame, detection_scale_x, detection_scale_y)

                fps_count += 1
                self.frame_count += 1

                current_time = time.time()
                if current_time - last_fps_time >= 1.0:
                    elapsed = max(current_time - last_fps_time, 1e-6)
                    self.fps = fps_count / elapsed
                    if self.on_frame_processed:
                        self.on_frame_processed(self.frame_count, self.fps)
                    fps_count = 0
                    last_fps_time = current_time

        except Exception as exc:
            self._error(f"Detection loop error: {exc}")
        finally:
            self._cleanup()
            self._set_state(DetectorState.IDLE)
            print("[AprilTagDetector] Detection loop ended")

    def _process_detections(
        self,
        detections,
        frame,
        detection_scale_x: float = 1.0,
        detection_scale_y: float = 1.0,
    ) -> None:
        """
        Process detected tags and emit callbacks.

        Args:
            detections: List of apriltag.Detection objects
            frame: Current frame used for preview
            detection_scale_x: X scale from detection image back to capture image
            detection_scale_y: Y scale from detection image back to capture image
        """
        frame_height, frame_width = frame.shape[:2]
        current_tags: Dict[int, Dict] = {}
        detection_timestamp = time.time()

        for det in detections:
            tag_id = int(det.tag_id)
            center = np.asarray(det.center, dtype=float).copy()
            corners = np.asarray(det.corners, dtype=float).copy()

            if detection_scale_x != 1.0 or detection_scale_y != 1.0:
                center[0] *= detection_scale_x
                center[1] *= detection_scale_y
                corners[:, 0] *= detection_scale_x
                corners[:, 1] *= detection_scale_y

            center_error_px = float(center[0]) - (frame_width / 2.0)
            image_rotation_deg = self._estimate_image_rotation_deg(corners)
            rotation_error_deg = self._rotation_deg_to_signed_error(image_rotation_deg)
            expected_pos = self.tag_layout.get(tag_id)

            current_tags[tag_id] = {
                "tag_id": tag_id,
                "center": center,
                "corners": corners,
                "frame_size": (frame_width, frame_height),
                "center_error_px": center_error_px,
                "center_error_norm": center_error_px / max(frame_width / 2.0, 1.0),
                "image_rotation_deg": image_rotation_deg,
                "rotation_error_deg": rotation_error_deg,
                "expected_pos": expected_pos,
                "timestamp": detection_timestamp,
            }

        preview_signature = tuple(sorted(current_tags.keys()))

        with self._lock:
            self.last_detected_tags = current_tags

            should_refresh_preview = (
                self._latest_preview_frame is None
                or preview_signature != self._last_preview_signature
                or (detection_timestamp - self._last_preview_update) >= self.preview_interval_sec
            )
            if should_refresh_preview:
                self._latest_preview_frame = self._annotate_frame(frame, current_tags)
                self._last_preview_update = detection_timestamp
                self._last_preview_signature = preview_signature

            if current_tags:
                self.detection_history.append(
                    {
                        "timestamp": detection_timestamp,
                        "tags": list(current_tags.keys()),
                        "count": len(current_tags),
                    }
                )
                self.last_detection_time = detection_timestamp

        for tag_id, detection_data in current_tags.items():
            if self.on_tag_detected:
                self.on_tag_detected(tag_id, detection_data["expected_pos"])

            if detection_data["expected_pos"] is not None:
                self._detected_tag_id = tag_id
                self._tag_detection_event.set()

    def _annotate_frame(self, frame, current_tags: Dict[int, Dict]):
        """Draw detection overlays for the UI preview."""
        preview = frame.copy()
        frame_height, frame_width = preview.shape[:2]

        cv2.line(preview, (frame_width // 2, 0), (frame_width // 2, frame_height), (0, 255, 255), 1)
        cv2.putText(
            preview,
            f"State: {self.state.value} FPS: {self.fps:.1f}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        detected_text = ", ".join(str(tag_id) for tag_id in sorted(current_tags)) if current_tags else "None"
        cv2.putText(
            preview,
            f"Detected: {detected_text}",
            (10, 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        for tag_id, data in current_tags.items():
            corners = np.asarray(data["corners"], dtype=np.int32)
            center = tuple(int(round(v)) for v in data["center"])
            cv2.polylines(preview, [corners], True, (0, 255, 0), 2)
            cv2.circle(preview, center, 5, (0, 0, 255), -1)

            label = f"ID:{tag_id}"
            if data["expected_pos"] is not None:
                row, col = data["expected_pos"]
                label += f" ({row},{col})"
            label += f" a={data['image_rotation_deg']:.1f}"

            text_x = int(corners[:, 0].min())
            text_y = max(25, int(corners[:, 1].min()) - 10)
            cv2.putText(
                preview,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2,
            )

        return preview

    def wait_for_tag(self, expected_tag: Optional[int] = None, timeout: float = 5.0) -> Optional[int]:
        """
        Block and wait for a specific tag detection.

        Args:
            expected_tag: Tag ID to wait for (if None, accept any tag)
            timeout: Max seconds to wait (0 = no timeout)

        Returns:
            Detected tag_id if found, None on timeout or error
        """
        if self.state != DetectorState.RUNNING:
            print("[AprilTagDetector] Detector not running")
            return None

        print(f"[AprilTagDetector] Waiting for tag {expected_tag} (timeout={timeout}s)...")

        start_time = time.time()
        self._tag_detection_event.clear()

        while not self._stop_event.is_set():
            with self._lock:
                if expected_tag is None:
                    if self.last_detected_tags:
                        detected_id = list(self.last_detected_tags.keys())[0]
                        print(f"[AprilTagDetector] Detected tag {detected_id}")
                        return detected_id
                else:
                    if expected_tag in self.last_detected_tags:
                        print(f"[AprilTagDetector] Detected expected tag {expected_tag}")
                        return expected_tag

            if timeout > 0 and (time.time() - start_time) > timeout:
                print(f"[AprilTagDetector] Timeout waiting for tag {expected_tag}")
                return None

            self._tag_detection_event.wait(timeout=0.1)
            self._tag_detection_event.clear()

        return None

    def get_last_detected_tags(self) -> Dict[int, Tuple[int, int]]:
        """Get most recent tag detections."""
        with self._lock:
            return {
                tag_id: data["expected_pos"]
                for tag_id, data in self.last_detected_tags.items()
                if data["expected_pos"] is not None
            }

    def get_last_detection_details(self) -> Dict[int, Dict]:
        """Get full measurement details for the most recent tag detections."""
        with self._lock:
            return {
                tag_id: {
                    "tag_id": data["tag_id"],
                    "center": tuple(float(v) for v in data["center"]),
                    "corners": [tuple(float(v) for v in corner) for corner in data["corners"]],
                    "frame_size": tuple(int(v) for v in data["frame_size"]),
                    "center_error_px": float(data["center_error_px"]),
                    "center_error_norm": float(data["center_error_norm"]),
                    "image_rotation_deg": float(data["image_rotation_deg"]),
                    "rotation_error_deg": float(data.get("rotation_error_deg", 0.0)),
                    "expected_pos": data["expected_pos"],
                    "timestamp": float(data["timestamp"]),
                }
                for tag_id, data in self.last_detected_tags.items()
            }

    def get_latest_preview_frame(self):
        """Get the most recent annotated camera frame for UI display."""
        with self._lock:
            if self._latest_preview_frame is None:
                return None
            return self._latest_preview_frame.copy()

    def get_detection_stats(self) -> Dict:
        """Get detector statistics."""
        with self._lock:
            return {
                "state": self.state.value,
                "fps": self.fps,
                "frame_count": self.frame_count,
                "total_tags_in_layout": len(self.tag_layout),
                "last_detected_count": len(self.last_detected_tags),
                "last_detection_time": self.last_detection_time,
                "detection_history_size": len(self.detection_history),
                "capture_resolution": (self.capture_width, self.capture_height),
                "detection_resolution": (self.detection_width, self.detection_height),
                "is_raspberry_pi": self.is_raspberry_pi,
            }

    def is_running(self) -> bool:
        """Check if detector is actively running."""
        return self.state == DetectorState.RUNNING

    @staticmethod
    def _estimate_image_rotation_deg(corners) -> float:
        """
        Estimate the tag's rotation in the image plane from its top edge.

        This is not a full 3D pose estimate, but it is still useful as a
        visual-alignment signal during slow turning.
        """
        if corners is None or len(corners) < 2:
            return 0.0

        top_left = corners[0]
        top_right = corners[1]
        dy = float(top_right[1]) - float(top_left[1])
        dx = float(top_right[0]) - float(top_left[0])
        return (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0

    @staticmethod
    def _rotation_deg_to_signed_error(rotation_deg: float) -> float:
        """Convert a normalized 0..360 angle into a signed -180..180 error."""
        return ((rotation_deg + 180.0) % 360.0) - 180.0


if __name__ == "__main__":
    """Standalone test of the detector."""

    def on_tag_detected(tag_id, pos):
        if pos:
            print(f"  -> Tag {tag_id} detected at grid position {pos}")
        else:
            print(f"  -> Tag {tag_id} detected (position unknown)")

    def on_frame_processed(frame_idx, fps):
        print(f"  FPS: {fps:.1f}")

    detector = AprilTagDetectorRealtime(layout_file="apriltag_layout.json", camera_id=0)
    detector.on_tag_detected = on_tag_detected
    detector.on_frame_processed = on_frame_processed

    print("Starting detector (press Ctrl+C to stop)...")
    if detector.start():
        try:
            for expected_tag in [1, 2, 3]:
                print(f"\nWaiting for tag {expected_tag}...")
                detected = detector.wait_for_tag(expected_tag=expected_tag, timeout=10.0)
                if detected:
                    print(f"✓ Got tag {detected}")
                else:
                    print(f"✗ Timeout waiting for tag {expected_tag}")
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[USER] Stopping...")
        finally:
            detector.stop()
            stats = detector.get_detection_stats()
            print(f"\nFinal Stats: {stats}")
