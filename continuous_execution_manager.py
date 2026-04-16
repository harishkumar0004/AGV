"""
Continuous Path Execution Manager - Integrated with Real Hardware

This replaces the old cell-by-cell motion execution with continuous
motion for entire paths using dynamic distance calculation.

Key Changes:
1. Sends total distance to Arduino (not per-cell 500mm commands)
2. Arduino executes ONE continuous trapezoidal profile
3. AprilTag detections used for trajectory verification only
4. Supports steering correction if robot drifts
"""

import math
import serial
import time
from threading import Lock
from typing import List, Tuple, Optional, Callable, Dict

from continuous_path_executor import ContinuousPathExecutor, MotionProfile, TrajectoryCorrection
from apriltag_detector_realtime import AprilTagDetectorRealtime
from motion import EAST, NORTH, SOUTH, WEST, heading_from_coords

try:
    from PyQt5.QtWidgets import QApplication
except ImportError:
    QApplication = None


class ContinuousExecutionManager:
    """
    Executes continuous motion for entire paths with AprilTag trajectory verification.
    """
    
    def __init__(self, robot_state, port: str = '/dev/ttyUSB0', baudrate: int = 115200,
                 layout_file: str = 'apriltag_layout.json', use_detector: bool = True):
        """
        Args:
            robot_state: RobotState instance
            port: Serial port for Arduino
            baudrate: Serial baud rate (115200 for logs)
            layout_file: Path to apriltag_layout.json
            use_detector: Enable AprilTag trajectory verification
        """
        self.robot_state = robot_state
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self._serial_lock = Lock()
        
        # Path executor for distance calculation & motion planning
        self.path_executor = ContinuousPathExecutor(
            distance_per_cell=500.0,  # Fixed 500mm per cell
            max_rpm=60,
            initial_rpm=2,
            acc_rpm_per_sec=25
        )
        
        # AprilTag detector for trajectory verification (lazy initialization)
        self.use_detector = use_detector
        self.detector: Optional[AprilTagDetectorRealtime] = None
        self._detector_initialized = False
        self._layout_file = layout_file
        self.trajectory_corrector = TrajectoryCorrection(self._layout_file)
        
        # Execution state
        self.is_executing = False
        self.current_exec_plan: Optional[Dict] = None
        self.detected_tags_during_execution = []
        self.current_heading = SOUTH

        # Closed-loop turn tuning
        self.turn_track_width_mm = 360.0
        self.turn_max_rpm = 20
        self.turn_initial_rpm = 2
        self.turn_acc_rpm_per_sec = 10
        self.turn_search_step_deg = 12.0
        self.turn_fine_step_deg = 3.0
        self.turn_center_tolerance_px = 35.0
        self.turn_rotation_tolerance_deg = 15.0
        self.turn_alignment_stable_reads = 2
        self.turn_detector_settle_sec = 0.20
        self.turn_timeout_sec = 20.0
        self._max_detection_age_sec = 0.75

        # If the robot reaches the planned distance but the goal tag is still
        # not visible, creep forward slowly in short pulses to recover.
        self.goal_tag_creep_step_mm = 40.0
        self.goal_tag_creep_max_distance_mm = 200.0
        self.goal_tag_creep_max_rpm = 8
        self.goal_tag_creep_initial_rpm = 2
        self.goal_tag_creep_acc_rpm_per_sec = 8
        
        # Callbacks
        self.on_execution_started: Optional[Callable] = None
        self.on_position_update: Optional[Callable[[float, float, float], None]] = None  # (distance, progress_percent, speed_rpm)
        self.on_waypoint_verified: Optional[Callable[[int, bool], None]] = None  # (tag_id, expected)
        self.on_trajectory_correction_needed: Optional[Callable[[int], None]] = None  # (tag_id)
        self.on_execution_complete: Optional[Callable] = None
        self.on_error: Optional[Callable[[str], None]] = None

    @staticmethod
    def _pump_ui_events() -> None:
        """Keep the Qt UI responsive while execution runs in the foreground."""
        if QApplication is not None:
            QApplication.processEvents()

    def is_connected(self) -> bool:
        """Return True when the Arduino serial port is open."""
        return bool(self.serial and getattr(self.serial, 'is_open', False))

    def set_heading(self, heading: int) -> None:
        """Update the robot's tracked heading for the next execution segment."""
        if heading in (NORTH, EAST, SOUTH, WEST):
            self.current_heading = heading

    def ensure_detector_running(self, wait: bool = False, timeout_sec: float = 5.0) -> bool:
        """Initialize and optionally wait for the AprilTag detector."""
        if not self.use_detector:
            return False

        if not self._detector_initialized or self.detector is None:
            print("[ContinuousExecutionManager] Initializing AprilTag detector...")
            try:
                self.detector = AprilTagDetectorRealtime(layout_file=self._layout_file)
                self._detector_initialized = True
            except Exception as detector_error:
                print(f"[ContinuousExecutionManager] ⚠ WARNING: Detector init failed: {detector_error}")
                self.use_detector = False
                return False

        if not self.detector.is_running():
            print("[ContinuousExecutionManager] Starting detector thread...")
            if not self.detector.start():
                if not wait:
                    return False
                if not self.detector.wait_until_running(timeout_sec):
                    print("[ContinuousExecutionManager] ⚠ WARNING: Detector startup failed (camera may not be available)")
                    return False

        if wait:
            if not self.detector.wait_until_running(timeout_sec):
                self._error("AprilTag detector did not reach RUNNING state")
                return False
            print("[ContinuousExecutionManager] AprilTag detector is RUNNING")

        return True
    
    def connect(self) -> bool:
        """
        Establish connection to Arduino.
        
        Returns:
            True if connection successful, False otherwise
        """
        if self.is_connected():
            print(f"[ContinuousExecutionManager] Already connected to {self.port}")
            return True

        if self.serial is not None:
            try:
                if getattr(self.serial, 'is_open', False):
                    self.serial.close()
            except Exception as stale_serial_error:
                print(f"[ContinuousExecutionManager] Warning closing stale serial handle: {stale_serial_error}")
            finally:
                self.serial = None

        try:
            # First, establish serial connection to Arduino
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                write_timeout=1.0
            )
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            print(f"[ContinuousExecutionManager] ✓ Connected to {self.port} @ {self.baudrate}bps")

            # Many Arduino boards reset when the serial port opens. Wait for the
            # firmware banner so motion commands are not sent during reboot.
            self._wait_for_ready_banner()
            
            if self.use_detector:
                self.ensure_detector_running(wait=False)
            
            return True
        except serial.SerialException as e:
            error_msg = f"Serial connection failed: {str(e)}"
            print(f"[ContinuousExecutionManager] ✗ {error_msg}")
            self._error(error_msg)
            return False
        except Exception as e:
            error_msg = f"Connection error: {str(e)}"
            print(f"[ContinuousExecutionManager] ✗ {error_msg}")
            self._error(error_msg)
            return False

    def _wait_for_ready_banner(self, timeout_sec: float = 5.0) -> bool:
        """Wait for the Arduino startup banner after opening the serial port."""
        if not self.serial or not self.serial.is_open:
            return False

        print("[ContinuousExecutionManager] Waiting for Arduino READY banner...")
        deadline = time.time() + timeout_sec
        original_timeout = self.serial.timeout

        try:
            self.serial.timeout = 0.25
            while time.time() < deadline:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                print(f"  [Arduino] {line}")
                if '[READY]' in line:
                    self.serial.reset_input_buffer()
                    return True
        finally:
            self.serial.timeout = original_timeout

        print("[ContinuousExecutionManager] ⚠ WARNING: READY banner not seen; continuing anyway")
        return False
    
    def disconnect(self) -> None:
        """Close connection and stop detector."""
        if self._detector_initialized and self.detector:
            print("[ContinuousExecutionManager] Stopping detector...")
            try:
                self.detector.stop()
            except Exception as e:
                print(f"[ContinuousExecutionManager] Warning stopping detector: {e}")
        
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
                self.serial = None
                print("[ContinuousExecutionManager] Disconnected")
            except Exception as e:
                print(f"[ContinuousExecutionManager] Warning closing serial: {e}")
    
    def prepare_path_execution(self, path_coords: List[Tuple[int, int]],
                               path_node_ids: List[int]) -> Dict:
        """
        Prepare execution plan for a path.
        
        Args:
            path_coords: List of (row, col) grid coordinates  
            path_node_ids: List of 1-based node IDs
        
        Returns:
            Execution plan with distances and parameters
        """
        exec_plan = self.path_executor.prepare_execution(path_coords, path_node_ids)
        self.current_exec_plan = exec_plan
        
        print(f"\n[ContinuousExecutionManager] Path prepared:")
        print(f"  Total distance: {exec_plan['total_distance_mm']:.1f} mm")
        print(f"  Number of cells: {exec_plan['num_cells']}")
        print(f"  Expected execution time: {exec_plan['total_execution_time_sec']:.2f}s")
        print(f"  Waypoint tags: {exec_plan['waypoint_tags']}")
        
        return exec_plan
    
    def execute_path_continuous(self, path_coords: List[Tuple[int, int]],
                               path_node_ids: List[int],
                               direction: str = 'F') -> bool:
        """
        Execute continuous motion for a path.
        
        Args:
            path_coords: List of (row, col) grid coordinates
            path_node_ids: List of 1-based node IDs
            direction: Direction command ('F'=forward, 'B'=backward, 'L'=left, 'R'=right)
        
        Returns:
            True if execution started successfully
        """
        if not self.serial or not self.serial.is_open:
            self._error("Not connected to Arduino")
            return False
        
        if self.is_executing:
            self._error("Already executing a path")
            return False

        if not path_coords or len(path_coords) < 2:
            self.robot_state.update_node(path_node_ids[-1] if path_node_ids else self.robot_state.current_node)
            self._on_execution_complete()
            return True

        if self.use_detector and not self.ensure_detector_running(wait=True, timeout_sec=5.0):
            return False

        overall_plan = self.prepare_path_execution(path_coords, path_node_ids)
        self.path_executor.print_execution_plan(overall_plan)

        self.is_executing = True
        self.detected_tags_during_execution = []

        if self.on_execution_started:
            self.on_execution_started()

        success = False
        try:
            success = self._execute_segmented_path(path_coords, path_node_ids)
            if success:
                self.current_exec_plan = overall_plan
                self.robot_state.update_node(path_node_ids[-1])
                self._on_execution_complete()
            return success
        finally:
            if not success and self.is_executing:
                self.is_executing = False

    def _execute_segmented_path(self, path_coords: List[Tuple[int, int]], path_node_ids: List[int]) -> bool:
        """Execute a path as turn + straight segments using closed-loop AprilTag turns."""
        idx = 0
        tracked_heading = self.current_heading

        while idx < len(path_coords) - 1 and self.is_executing:
            required_heading = heading_from_coords(path_coords[idx], path_coords[idx + 1])
            if required_heading is None:
                idx += 1
                continue

            expected_tag_id = path_node_ids[idx + 1] if idx + 1 < len(path_node_ids) else None
            if required_heading != tracked_heading:
                if not self._turn_to_heading(tracked_heading, required_heading, expected_tag_id):
                    return False
                tracked_heading = required_heading
                self.current_heading = tracked_heading

            segment_end = idx + 1
            while segment_end < len(path_coords) - 1:
                next_heading = heading_from_coords(path_coords[segment_end], path_coords[segment_end + 1])
                if next_heading != required_heading:
                    break
                segment_end += 1

            segment_coords = path_coords[idx:segment_end + 1]
            segment_node_ids = path_node_ids[idx:segment_end + 1]
            if not self._execute_linear_segment(segment_coords, segment_node_ids):
                return False

            tracked_heading = required_heading
            self.current_heading = tracked_heading
            idx = segment_end

        return True

    def _turn_to_heading(self, current_heading: int, required_heading: int, expected_tag_id: Optional[int]) -> bool:
        """Turn in place until the robot is aligned with the next path heading."""
        turn_needed = (required_heading - current_heading) % 4

        if turn_needed == 0:
            return True
        if turn_needed == 1:
            return self._execute_closed_loop_turn('R', expected_tag_id)
        if turn_needed == 3:
            return self._execute_closed_loop_turn('L', expected_tag_id)

        # Fall back to a 180° open-loop turn if ever needed.
        return self._execute_turn_pulse('R', 180.0)

    def _execute_closed_loop_turn(self, turn_direction: str, expected_tag_id: Optional[int]) -> bool:
        """
        Rotate in place in small pulses until the expected next AprilTag is centered.

        This uses the expected next tag from the planned path as the visual
        reference for left/right turns.
        """
        if not self.ensure_detector_running(wait=True, timeout_sec=5.0):
            self._error("Closed-loop turn requires a running AprilTag detector")
            return False

        if expected_tag_id is None:
            self._error("Closed-loop turn requires an expected next tag")
            return False

        print(
            f"\n[ContinuousExecutionManager] Closed-loop turn {turn_direction} toward Tag {expected_tag_id}"
        )
        stable_reads = 0
        deadline = time.time() + self.turn_timeout_sec
        search_rotated_deg = 0.0

        while self.is_executing and time.time() < deadline:
            self._pump_ui_events()
            alignment = self._get_expected_tag_alignment(expected_tag_id)

            if alignment is None:
                stable_reads = 0
                if not self._execute_turn_pulse(turn_direction, self.turn_search_step_deg):
                    return False
                search_rotated_deg += self.turn_search_step_deg
                if search_rotated_deg > 150.0:
                    self._error(
                        f"Closed-loop turn could not find Tag {expected_tag_id} within {search_rotated_deg:.1f}deg"
                    )
                    return False
                time.sleep(self.turn_detector_settle_sec)
                continue

            search_rotated_deg = 0.0
            error_px = alignment['center_error_px']
            rotation_error_deg = alignment.get('rotation_error_deg', 0.0)
            if (
                abs(error_px) <= self.turn_center_tolerance_px
                and abs(rotation_error_deg) <= self.turn_rotation_tolerance_deg
            ):
                stable_reads += 1
                print(
                    f"  [Turn] Tag {expected_tag_id} aligned "
                    f"(error={error_px:.1f}px, rot={rotation_error_deg:.1f}deg, "
                    f"stable={stable_reads}/{self.turn_alignment_stable_reads})"
                )
                if stable_reads >= self.turn_alignment_stable_reads:
                    return True
                time.sleep(self.turn_detector_settle_sec)
                continue

            stable_reads = 0
            if abs(error_px) > self.turn_center_tolerance_px:
                correction_direction = 'R' if error_px > 0 else 'L'
                pulse_deg = (
                    self.turn_fine_step_deg
                    if abs(error_px) <= self.turn_center_tolerance_px * 3.0
                    else self.turn_search_step_deg
                )
            else:
                # Image rotation is opposite the camera yaw, so correct in the opposite direction.
                correction_direction = 'R' if rotation_error_deg > 0 else 'L'
                pulse_deg = self.turn_fine_step_deg
            print(
                f"  [Turn] Tag {expected_tag_id} error={error_px:.1f}px, "
                f"rot={rotation_error_deg:.1f}deg, "
                f"pulse={correction_direction} {pulse_deg:.1f}deg"
            )
            if not self._execute_turn_pulse(correction_direction, pulse_deg):
                return False
            time.sleep(self.turn_detector_settle_sec)

        self._error(f"Closed-loop turn timed out while aligning to Tag {expected_tag_id}")
        return False

    def _execute_turn_pulse(self, turn_direction: str, angle_deg: float) -> bool:
        """Execute a short in-place turn pulse using the same trapezoidal controller."""
        turn_distance_mm = self._angle_to_turn_distance_mm(angle_deg)
        expected_exec_time = self._estimate_motion_time_sec(
            turn_distance_mm,
            self.turn_max_rpm,
            self.turn_initial_rpm,
            self.turn_acc_rpm_per_sec,
        )
        print(
            f"[ContinuousExecutionManager] Turn pulse {turn_direction}: "
            f"{angle_deg:.1f}deg ({turn_distance_mm:.1f}mm wheel travel)"
        )
        if not self._start_motion(
            direction=turn_direction,
            total_distance_mm=turn_distance_mm,
            distance_per_cell_mm=turn_distance_mm,
            max_rpm=self.turn_max_rpm,
            initial_rpm=self.turn_initial_rpm,
            acc_rpm_per_sec=self.turn_acc_rpm_per_sec,
        ):
            self._error(f"Failed to start turn pulse {turn_direction}")
            return False
        return self._wait_for_motion_complete(
            expected_exec_time,
            waypoint_tags=None,
            emit_waypoints=False,
            require_stop_tag_detection=False,
        )

    def _execute_linear_segment(self, path_coords: List[Tuple[int, int]], path_node_ids: List[int]) -> bool:
        """Execute one straight segment between heading changes."""
        exec_plan = self.prepare_path_execution(path_coords, path_node_ids)
        ap = exec_plan['arduino_params']

        expected_tags = set(exec_plan['waypoint_tags'])
        if path_node_ids:
            expected_tags.discard(path_node_ids[0])

        if not self._start_motion(
            direction='F',
            total_distance_mm=ap['total_distance_mm'],
            distance_per_cell_mm=ap['distance_per_cell_mm'],
            max_rpm=ap['max_rpm'],
            initial_rpm=ap['initial_rpm'],
            acc_rpm_per_sec=ap['acc_rpm_per_sec'],
        ):
            self._error("Failed to start straight segment")
            return False

        success = self._wait_for_motion_complete(
            exec_plan['total_execution_time_sec'],
            waypoint_tags=expected_tags,
            emit_waypoints=True,
            stop_on_tag_id=path_node_ids[-1] if path_node_ids else None,
            require_stop_tag_detection=True,
        )
        if success and path_node_ids:
            self.robot_state.update_node(path_node_ids[-1])
        return success

    def _start_motion(
        self,
        direction: str,
        total_distance_mm: float,
        distance_per_cell_mm: float,
        max_rpm: float,
        initial_rpm: float,
        acc_rpm_per_sec: float,
    ) -> bool:
        """Send direction and START commands for a motion segment."""
        if not self._send_command(direction):
            return False
        time.sleep(0.1)

        start_command = (
            f"START:{int(round(total_distance_mm))}:{int(round(max(distance_per_cell_mm, 1.0)))}:"
            f"{int(round(max_rpm))}:{int(round(initial_rpm))}:{int(round(acc_rpm_per_sec))}\n"
        )
        print(f"[ContinuousExecutionManager] START command: {start_command.strip()}")
        return self._send_command(start_command)

    def _capture_waypoint_detections(
        self,
        waypoint_tags: Optional[set],
        detected_waypoints: set,
        min_timestamp: float,
    ) -> None:
        """Record fresh AprilTag detections for the active motion window."""
        if not self.detector or not self.detector.is_running():
            return

        detected_tags = self.detector.get_last_detection_details()
        for tag_id, tag_details in detected_tags.items():
            if waypoint_tags is not None and tag_id not in waypoint_tags:
                continue
            if tag_details['timestamp'] < min_timestamp:
                continue
            if tag_id in detected_waypoints:
                continue

            tag_pos = tag_details['expected_pos']
            print(f"\n  ✓ Waypoint {tag_id} detected at {tag_pos}")
            detected_waypoints.add(tag_id)
            if tag_id not in self.detected_tags_during_execution:
                self.detected_tags_during_execution.append(tag_id)
            self.robot_state.update_node(tag_id)

            if self.on_waypoint_verified:
                self.on_waypoint_verified(tag_id, True)

    def _creep_forward_until_goal_tag(self, goal_tag_id: int, detected_waypoints: set) -> bool:
        """Slowly creep forward in short pulses until the goal tag is detected."""
        if not self.ensure_detector_running(wait=True, timeout_sec=5.0):
            return False

        print(
            f"\n[ContinuousExecutionManager] Goal tag {goal_tag_id} missing at planned stop - "
            f"creeping forward slowly"
        )
        total_creep_distance_mm = 0.0

        while self.is_executing and total_creep_distance_mm < self.goal_tag_creep_max_distance_mm:
            fresh_detection_window = time.time() - self._max_detection_age_sec
            self._capture_waypoint_detections({goal_tag_id}, detected_waypoints, fresh_detection_window)
            if goal_tag_id in detected_waypoints:
                print(f"[ContinuousExecutionManager] Goal tag {goal_tag_id} detected without extra movement")
                return True

            pulse_distance_mm = min(
                self.goal_tag_creep_step_mm,
                self.goal_tag_creep_max_distance_mm - total_creep_distance_mm,
            )
            expected_exec_time = self._estimate_motion_time_sec(
                pulse_distance_mm,
                self.goal_tag_creep_max_rpm,
                self.goal_tag_creep_initial_rpm,
                self.goal_tag_creep_acc_rpm_per_sec,
            )
            print(
                f"[ContinuousExecutionManager] Creep pulse: {pulse_distance_mm:.1f}mm "
                f"(extra {total_creep_distance_mm + pulse_distance_mm:.1f}/"
                f"{self.goal_tag_creep_max_distance_mm:.1f}mm)"
            )

            if not self._start_motion(
                direction='F',
                total_distance_mm=pulse_distance_mm,
                distance_per_cell_mm=pulse_distance_mm,
                max_rpm=self.goal_tag_creep_max_rpm,
                initial_rpm=self.goal_tag_creep_initial_rpm,
                acc_rpm_per_sec=self.goal_tag_creep_acc_rpm_per_sec,
            ):
                self._error(f"Failed to start slow creep toward goal tag {goal_tag_id}")
                return False

            pulse_start_time = time.time()
            last_progress_time = pulse_start_time
            hard_timeout = max(expected_exec_time * 2.0, expected_exec_time + 10.0)
            tag_stop_requested = False

            while self.is_executing:
                self._pump_ui_events()
                motion_done = False

                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        last_progress_time = time.time()

                    if line.startswith('[POS]'):
                        self._parse_position_update(line)
                    elif line == "DONE":
                        motion_done = True
                    elif line.startswith('['):
                        print(f"  [Arduino] {line}")

                self._capture_waypoint_detections(
                    {goal_tag_id},
                    detected_waypoints,
                    pulse_start_time - self._max_detection_age_sec,
                )

                if goal_tag_id in detected_waypoints and not tag_stop_requested and not motion_done:
                    print(
                        f"[ContinuousExecutionManager] Goal tag {goal_tag_id} detected during creep - "
                        f"requesting stop"
                    )
                    if not self._send_command('X'):
                        self._error(f"Failed to stop creep after detecting goal tag {goal_tag_id}")
                        return False
                    tag_stop_requested = True

                if motion_done:
                    total_creep_distance_mm += pulse_distance_mm
                    if goal_tag_id in detected_waypoints:
                        print(f"[ContinuousExecutionManager] Goal tag {goal_tag_id} confirmed after creep")
                        return True
                    break

                elapsed = time.time() - pulse_start_time
                stalled_for = time.time() - last_progress_time

                if elapsed > hard_timeout:
                    self._error(
                        f"Slow creep timed out while searching for goal tag {goal_tag_id}"
                    )
                    self._stop_execution()
                    return False

                if elapsed > expected_exec_time and stalled_for > 5.0:
                    self._error(
                        f"Slow creep stalled while searching for goal tag {goal_tag_id}"
                    )
                    self._stop_execution()
                    return False

                time.sleep(0.05)

        return goal_tag_id in detected_waypoints

    def _wait_for_motion_complete(
        self,
        expected_exec_time: float,
        waypoint_tags: Optional[set],
        emit_waypoints: bool,
        stop_on_tag_id: Optional[int] = None,
        require_stop_tag_detection: bool = False,
    ) -> bool:
        """Wait for one motion segment to report DONE while monitoring progress."""
        print("\n[ContinuousExecutionManager] Monitoring motion segment...")
        start_time = time.time()
        last_progress_time = start_time
        hard_timeout = max(expected_exec_time * 2.0, expected_exec_time + 20.0)
        detected_waypoints = set()
        tag_stop_requested = False

        try:
            while self.is_executing:
                self._pump_ui_events()
                motion_done = False
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        last_progress_time = time.time()

                    if line.startswith('[POS]'):
                        self._parse_position_update(line)
                    elif line == "DONE":
                        motion_done = True
                    elif line.startswith('['):
                        print(f"  [Arduino] {line}")

                if emit_waypoints and self.detector and self.detector.is_running():
                    self._capture_waypoint_detections(waypoint_tags, detected_waypoints, start_time)

                    if stop_on_tag_id is not None and stop_on_tag_id in detected_waypoints and not tag_stop_requested and not motion_done:
                        print(
                            f"[ContinuousExecutionManager] Goal tag {stop_on_tag_id} detected - requesting motion stop"
                        )
                        if not self._send_command('X'):
                            self._error(f"Failed to send stop command after detecting goal tag {stop_on_tag_id}")
                            return False
                        tag_stop_requested = True

                if motion_done:
                    if require_stop_tag_detection and stop_on_tag_id is not None and stop_on_tag_id not in detected_waypoints:
                        if self._creep_forward_until_goal_tag(stop_on_tag_id, detected_waypoints):
                            print("\n[ContinuousExecutionManager] Motion complete after slow goal-tag recovery")
                            return True
                        self._error(
                            f"Motion finished before expected tag {stop_on_tag_id} was detected, "
                            f"even after {self.goal_tag_creep_max_distance_mm:.0f}mm slow creep"
                        )
                        self._stop_execution()
                        return False
                    if waypoint_tags is not None and len(waypoint_tags) > 0 and not detected_waypoints:
                        self._error("Motion finished before any expected path tag was detected in real time")
                        self._stop_execution()
                        return False
                    print("\n[ContinuousExecutionManager] Motion complete (Arduino)")
                    return True

                elapsed = time.time() - start_time
                stalled_for = time.time() - last_progress_time

                if elapsed > hard_timeout:
                    self._error(
                        f"Execution timeout (expected {expected_exec_time:.1f}s, exceeded {hard_timeout:.1f}s)"
                    )
                    self._stop_execution()
                    return False

                if elapsed > expected_exec_time and stalled_for > 5.0:
                    self._error(
                        f"Execution stalled after {elapsed:.1f}s (no progress update for {stalled_for:.1f}s)"
                    )
                    self._stop_execution()
                    return False

                time.sleep(0.05)

        except Exception as e:
            self._error(f"Execution error: {str(e)}")
            self._stop_execution()
            return False

        return False

    def _get_expected_tag_alignment(self, expected_tag_id: int) -> Optional[Dict]:
        """Return fresh detector measurements for the expected next tag."""
        if not self.detector or not self.detector.is_running():
            return None

        detections = self.detector.get_last_detection_details()
        details = detections.get(expected_tag_id)
        if not details:
            return None

        if (time.time() - details['timestamp']) > self._max_detection_age_sec:
            return None

        return details

    def _angle_to_turn_distance_mm(self, angle_deg: float) -> float:
        """Convert an in-place robot turn angle to per-wheel travel distance."""
        return math.pi * self.turn_track_width_mm * (angle_deg / 360.0)

    @staticmethod
    def _estimate_motion_time_sec(distance_mm: float, max_rpm: float, initial_rpm: float, acc_rpm_per_sec: float) -> float:
        """Estimate motion duration using the same profile equations as the Python planner."""
        profile = MotionProfile(
            total_distance_mm=distance_mm,
            max_rpm=max_rpm,
            initial_rpm=initial_rpm,
            acc_rpm_per_sec=acc_rpm_per_sec,
        )
        return profile.get_profile_summary()['total_time_sec']
    
    def _parse_position_update(self, message: str) -> bool:
        """
        Parse position update from Arduino.
        
        Format: [POS] d=<distance>mm p=<percent>% s=<steps> rpm=<rpm>
        
        Args:
            message: Message from Arduino
        """
        try:
            # Extract values
            parts = message.split()
            
            distance_mm = 0.0
            progress_percent = 0.0
            speed_rpm = 0.0
            
            for part in parts:
                if part.startswith('d='):
                    distance_mm = float(part[2:-2])  # Remove 'd=' and 'mm'
                elif part.startswith('p='):
                    progress_percent = float(part[2:-1])  # Remove 'p=' and '%'
                elif part.startswith('rpm='):
                    speed_rpm = float(part[4:])
            
            if self.on_position_update:
                self.on_position_update(distance_mm, progress_percent, speed_rpm)
            return True
        
        except Exception:
            return False
    
    def _stop_execution(self):
        """Stop current execution."""
        if self.is_executing:
            print("[ContinuousExecutionManager] Stopping execution...")
            self._send_command('X')  # Emergency stop
            self.is_executing = False
    
    def stop_execution(self):
        """Public method to stop current execution (compatibility with RealRobotExecutor)."""
        self._stop_execution()
    
    def _on_execution_complete(self):
        """Handle successful execution completion."""
        self.is_executing = False
        
        print("\n[ContinuousExecutionManager] Execution complete!")
        if self.current_exec_plan:
            print(f"  Total distance: {self.current_exec_plan['total_distance_mm']:.1f} mm")
        
        if self.on_execution_complete:
            self.on_execution_complete()
    
    def _send_command(self, cmd: str) -> bool:
        """Send command to Arduino."""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            with self._serial_lock:
                if isinstance(cmd, str) and '\n' not in cmd:
                    cmd += '\n'
                
                if isinstance(cmd, str):
                    self.serial.write(cmd.encode('utf-8'))
                else:
                    self.serial.write(cmd)
                
                self.serial.flush()
                return True
        except serial.SerialException as e:
            print(f"[ContinuousExecutionManager] Serial error: {e}")
            return False
    
    def _error(self, msg: str) -> None:
        """Log error and emit callback."""
        print(f"[ContinuousExecutionManager] ERROR: {msg}")
        if self.robot_state:
            self.robot_state.set_status("ERROR")
        if self.on_error:
            self.on_error(msg)


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    """
    Example: Execute path from tag 1 to tag 5 (2000mm)
    """
    
    class DummyRobotState:
        def __init__(self):
            self.status = "IDLE"
            self.current_node = 1
        
        def set_status(self, status):
            self.status = status
            print(f"  [RobotState] {status}")
        
        def update_node(self, node_id):
            self.current_node = node_id
    
    # Create manager
    robot_state = DummyRobotState()
    manager = ContinuousExecutionManager(
        robot_state=robot_state,
        port='/dev/ttyUSB0',
        baudrate=115200,
        use_detector=True
    )
    
    # Connect
    if not manager.connect():
        print("Failed to connect to Arduino")
        exit(1)
    
    # Example path: Tag 1 → Tag 2 → Tag 3 → Tag 4 → Tag 5
    # (straight line, 2000mm total)
    path_coords = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]
    path_node_ids = [1, 2, 3, 4, 5]
    
    print("\n" + "="*70)
    print("CONTINUOUS PATH EXECUTION EXAMPLE")
    print("="*70)
    
    # Set callbacks
    def on_position(distance, progress, speed):
        print(f"    Motion progress: {distance:.1f}mm ({progress}%) @ {speed:.1f} RPM")
    
    def on_waypoint(tag_id, expected=True):
        print(f"    ✓ Waypoint {tag_id} verified (expected={expected})")
    
    manager.on_position_update = on_position
    manager.on_waypoint_verified = on_waypoint
    
    # Execute
    print("\nExecuting path...")
    manager.execute_path_continuous(path_coords, path_node_ids, direction='F')
    
    # Cleanup
    manager.disconnect()
