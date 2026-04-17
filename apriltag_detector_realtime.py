"""
Real-Time AprilTag Detector for AGV Hardware Confirmation

This module continuously monitors the camera for AprilTag detections
and provides real-time position confirmation for the executing robot.

Key Features:
- Background thread-based continuous detection
- Detects specific tag ID or any tag in frame
- Confidence/quality filtering
- Thread-safe callbacks
- Timeout handling for missed detections
- FPS monitoring and logging

Usage:
    detector = AprilTagDetectorRealtime(layout_file='apriltag_layout.json')
    detector.on_tag_detected = lambda tag_id, pos: print(f"Tag {tag_id} at {pos}")
    detector.start()
    
    # When waiting for next waypoint:
    tag_id = detector.wait_for_tag(expected_tag=8, timeout=5.0)
    if tag_id:
        detector.stop()
"""

import threading
import time
import json
import os
import math
from collections import deque
from enum import Enum
from typing import Optional, Callable, Tuple, Dict
import cv2

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
        """Run camera capture inside a Qt-managed thread when PyQt is available."""

        def __init__(self, target: Callable[[], None]):
            super().__init__()
            self._target = target

        def run(self) -> None:
            self._target()


class AprilTagDetectorRealtime:
    """
    Real-time AprilTag detector running in background thread.
    
    Provides continuous camera monitoring and tag position confirmation
    for AGV hardware navigation.
    """

    def __init__(self, layout_file: str = 'apriltag_layout.json', 
                 camera_id: int = 0, 
                 tag_family: str = 'tag36h11',
                 confidence_threshold: float = 0.5,
                 detection_smoothing: int = 3):
        """
        Initialize the detector.

        Args:
            layout_file: Path to apriltag_layout.json defining tag positions
            camera_id: OpenCV camera device ID (0 for default/integrated)
            tag_family: AprilTag family ('tag36h11', 'tag25h9', etc.)
            confidence_threshold: Minimum detection confidence (0-1)
            detection_smoothing: Number of frames to smooth detections (noise reduction)
        """
        self.layout_file = layout_file
        self.camera_id = camera_id
        self.tag_family = tag_family
        self.confidence_threshold = confidence_threshold
        self.detection_smoothing = detection_smoothing

        # Tag layout mapping: tag_id -> (row, col)
        self.tag_layout: Dict[int, Tuple[int, int]] = {}
        self._load_layout()

        # Thread management
        self.detector_thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        # State
        self.state = DetectorState.IDLE
        self.fps = 0.0
        self.frame_count = 0
        self.last_detection_time = 0.0

        # Camera and AprilTag detector
        self.cap: Optional[cv2.VideoCapture] = None
        self.detector: Optional[apriltag.Detector] = None

        # Latest detection cache
        self.last_detected_tags: Dict[int, Dict] = {}  # tag_id -> {center, corners, ...}
        self.detection_history = deque(maxlen=10)  # Recent detections
        self._latest_preview_frame = None

        # Callbacks
        self.on_tag_detected: Optional[Callable[[int, Tuple[int, int]], None]] = None
        self.on_frame_processed: Optional[Callable[[int, float], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None

        # Wait-for-tag synchronization
        self._detected_tag_id: Optional[int] = None
        self._tag_detection_event = threading.Event()

    def reset_detection_cache(self) -> None:
        """Clear cached detections so the next tag confirmation must be live."""
        with self._lock:
            self.last_detected_tags = {}
            self.detection_history.clear()
            self.last_detection_time = 0.0
            self._detected_tag_id = None
            self._tag_detection_event.clear()

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

            with open(self.layout_file, 'r', encoding='utf-8') as f:
                entries = json.load(f)

            self.tag_layout.clear()
            for entry in entries:
                tag_id = int(entry['id'])
                row = int(entry['row'])
                col = int(entry['col'])
                self.tag_layout[tag_id] = (row, col)

            print(f"[AprilTagDetector] Loaded {len(self.tag_layout)} tags from {self.layout_file}")
            return True

        except json.JSONDecodeError as e:
            self._error(f"Failed to parse layout JSON: {e}")
            return False
        except Exception as e:
            self._error(f"Failed to load layout: {e}")
            return False

    def start(self) -> bool:
        """Start the detector thread."""
        if self.state in (DetectorState.RUNNING, DetectorState.INITIALIZING):
            print("[AprilTagDetector] Already running or initializing")
            return False

        self._stop_event.clear()

        try:
            if QThread is not None:
                self.detector_thread = _DetectorQThread(self._detection_loop)
            else:
                self.detector_thread = threading.Thread(target=self._detection_loop, daemon=False)
            self.detector_thread.start()
            print("[AprilTagDetector] Detector thread started")
            return True
        except Exception as e:
            self._error(f"Failed to start detector thread: {e}")
            return False

    def stop(self) -> None:
        """Stop the detector thread gracefully."""
        if self.state not in (DetectorState.RUNNING, DetectorState.INITIALIZING):
            print("[AprilTagDetector] Not running, nothing to stop")
            return

        print("[AprilTagDetector] Stopping detector...")
        self._stop_event.set()

        if self.detector_thread:
            if hasattr(self.detector_thread, "isRunning"):
                if self.detector_thread.isRunning():
                    self.detector_thread.wait(5000)
            elif self.detector_thread.is_alive():
                self.detector_thread.join(timeout=5.0)

        self._cleanup()
        self._set_state(DetectorState.STOPPED)

    def _cleanup(self) -> None:
        """Release camera and detector resources."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None

        with self._lock:
            self.last_detected_tags.clear()
            self._latest_preview_frame = None

    def _set_state(self, new_state: DetectorState) -> None:
        """Update detector state."""
        with self._lock:
            if self.state != new_state:
                old_state = self.state
                self.state = new_state
                print(f"[AprilTagDetector] State: {old_state.value} → {new_state.value}")

    def _error(self, msg: str) -> None:
        """Log error and emit callback."""
        print(f"[AprilTagDetector] ERROR: {msg}")
        self._set_state(DetectorState.ERROR)
        if self.on_error:
            self.on_error(msg)

    def _detection_loop(self) -> None:
        """
        Main detection loop (runs in background thread).
        Continuously reads frames and detects AprilTags.
        """
        try:
            # Initialize camera
            self._set_state(DetectorState.INITIALIZING)

            if not APRILTAG_AVAILABLE:
                self._error("apriltag module not installed")
                return

            # Create camera instance
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap or not self.cap.isOpened():
                self._error(f"Failed to open camera {self.camera_id}")
                return

            # Set camera properties for faster processing
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

            # Create AprilTag detector
            options = apriltag.DetectorOptions(
                families=self.tag_family,
                nthreads=4,
                quad_decimate=2.0,
                quad_blur=0.0,
                refine_edges=True,
                refine_decode=False,
                refine_pose=False
            )
            self.detector = apriltag.Detector(options=options)

            print("[AprilTagDetector] Camera and detector initialized")
            self._set_state(DetectorState.RUNNING)

            # Detection loop
            last_fps_time = time.time()
            fps_count = 0

            while not self._stop_event.is_set():
                ret, frame = self.cap.read()
                if not ret:
                    print("[AprilTagDetector] Failed to read frame")
                    break

                # Convert to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detect tags
                detections = self.detector.detect(gray)

                # Process detections
                self._process_detections(detections, frame)

                # FPS calculation
                fps_count += 1
                current_time = time.time()
                if current_time - last_fps_time >= 1.0:
                    self.fps = fps_count
                    if self.on_frame_processed:
                        self.on_frame_processed(fps_count, self.fps)
                    fps_count = 0
                    last_fps_time = current_time

                self.frame_count += 1

                # Small sleep to prevent CPU spinning
                time.sleep(0.01)

        except Exception as e:
            self._error(f"Detection loop error: {e}")
        finally:
            self._cleanup()
            self._set_state(DetectorState.IDLE)
            print("[AprilTagDetector] Detection loop ended")

    def _process_detections(self, detections, frame) -> None:
        """
        Process detected tags and emit callbacks.

        Args:
            detections: List of apriltag.Detection objects
            frame: Current camera frame (for visualization if needed)
        """
        frame_height, frame_width = frame.shape[:2]
        current_tags: Dict[int, Dict] = {}
        detection_timestamp = time.time()

        with self._lock:
            for det in detections:
                tag_id = det.tag_id
                center = det.center
                corners = det.corners
                center_error_px = float(center[0]) - (frame_width / 2.0)
                image_rotation_deg = self._estimate_image_rotation_deg(corners)
                rotation_error_deg = self._rotation_deg_to_signed_error(image_rotation_deg)

                # Get expected position from layout
                expected_pos = self.tag_layout.get(tag_id)

                # Store detection
                current_tags[tag_id] = {
                    'tag_id': tag_id,
                    'center': center,
                    'corners': corners,
                    'frame_size': (frame_width, frame_height),
                    'center_error_px': center_error_px,
                    'center_error_norm': center_error_px / max(frame_width / 2.0, 1.0),
                    'image_rotation_deg': image_rotation_deg,
                    'rotation_error_deg': rotation_error_deg,
                    'expected_pos': expected_pos,
                    'timestamp': detection_timestamp
                }

            preview_frame = self._annotate_frame(frame, current_tags)

            # Update last detected tags
            self.last_detected_tags = current_tags
            self._latest_preview_frame = preview_frame

            # Record in history
            if current_tags:
                self.detection_history.append({
                    'timestamp': detection_timestamp,
                    'tags': list(current_tags.keys()),
                    'count': len(current_tags)
                })
                self.last_detection_time = detection_timestamp

        # Emit callbacks (outside lock to avoid deadlock)
        for tag_id, detection_data in current_tags.items():
            if self.on_tag_detected:
                expected_pos = detection_data['expected_pos']
                self.on_tag_detected(tag_id, expected_pos)

            # Signal wait_for_tag if waiting
            if expected_pos:
                self._detected_tag_id = tag_id
                self._tag_detection_event.set()

    def _annotate_frame(self, frame, current_tags: Dict[int, Dict]):
        """Draw detection overlays for UI preview."""
        preview = frame.copy()
        frame_height, frame_width = preview.shape[:2]

        # Center guide line helps align the target tag during turning.
        cv2.line(preview, (frame_width // 2, 0), (frame_width // 2, frame_height), (0, 255, 255), 1)
        cv2.putText(
            preview,
            f"State: {self.state.value}  FPS: {self.fps:.0f}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        if current_tags:
            detected_text = ", ".join(str(tag_id) for tag_id in sorted(current_tags))
        else:
            detected_text = "None"
        cv2.putText(
            preview,
            f"Detected: {detected_text}",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        for tag_id, data in current_tags.items():
            corners = data['corners'].astype(int)
            center = tuple(int(v) for v in data['center'])
            cv2.polylines(preview, [corners], True, (0, 255, 0), 2)
            cv2.circle(preview, center, 5, (0, 0, 255), -1)

            label = f"ID:{tag_id}"
            if data['expected_pos'] is not None:
                row, col = data['expected_pos']
                label += f" ({row},{col})"
            label += f" a={data['image_rotation_deg']:.1f}"

            text_x = int(corners[:, 0].min())
            text_y = max(25, int(corners[:, 1].min()) - 10)
            cv2.putText(
                preview,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2,
            )

        return preview

    def wait_for_tag(self, expected_tag: Optional[int] = None, 
                     timeout: float = 5.0) -> Optional[int]:
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
            # Check current detections
            with self._lock:
                if expected_tag is None:
                    # Accept any tag
                    if self.last_detected_tags:
                        detected_id = list(self.last_detected_tags.keys())[0]
                        print(f"[AprilTagDetector] Detected tag {detected_id}")
                        return detected_id
                else:
                    # Wait for specific tag
                    if expected_tag in self.last_detected_tags:
                        print(f"[AprilTagDetector] Detected expected tag {expected_tag}")
                        return expected_tag

            # Check timeout
            if timeout > 0 and (time.time() - start_time) > timeout:
                print(f"[AprilTagDetector] Timeout waiting for tag {expected_tag}")
                return None

            # Wait for detection event with small timeout
            self._tag_detection_event.wait(timeout=0.1)
            self._tag_detection_event.clear()

        return None

    def get_last_detected_tags(self) -> Dict[int, Tuple[int, int]]:
        """Get most recent tag detections."""
        with self._lock:
            return {
                tag_id: data['expected_pos']
                for tag_id, data in self.last_detected_tags.items()
                if data['expected_pos'] is not None
            }

    def get_last_detection_details(self) -> Dict[int, Dict]:
        """Get full measurement details for the most recent tag detections."""
        with self._lock:
            return {
                tag_id: {
                    'tag_id': data['tag_id'],
                    'center': tuple(float(v) for v in data['center']),
                    'corners': [tuple(float(v) for v in corner) for corner in data['corners']],
                    'frame_size': tuple(int(v) for v in data['frame_size']),
                    'center_error_px': float(data['center_error_px']),
                    'center_error_norm': float(data['center_error_norm']),
                    'image_rotation_deg': float(data['image_rotation_deg']),
                    'rotation_error_deg': float(data.get('rotation_error_deg', 0.0)),
                    'expected_pos': data['expected_pos'],
                    'timestamp': float(data['timestamp']),
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
                'state': self.state.value,
                'fps': self.fps,
                'frame_count': self.frame_count,
                'total_tags_in_layout': len(self.tag_layout),
                'last_detected_count': len(self.last_detected_tags),
                'last_detection_time': self.last_detection_time,
                'detection_history_size': len(self.detection_history)
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


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    """
    Standalone test of the detector.
    Run: python apriltag_detector_realtime.py
    """
    
    def on_tag_detected(tag_id, pos):
        if pos:
            print(f"  → Tag {tag_id} detected at grid position {pos}")
        else:
            print(f"  → Tag {tag_id} detected (position unknown)")

    def on_frame_processed(frame_idx, fps):
        print(f"  FPS: {fps:.1f}")

    # Create detector
    detector = AprilTagDetectorRealtime(
        layout_file='apriltag_layout.json',
        camera_id=0
    )

    detector.on_tag_detected = on_tag_detected
    detector.on_frame_processed = on_frame_processed

    # Start detection
    print("Starting detector (press Ctrl+C to stop)...")
    if detector.start():
        try:
            # Wait for tags
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
