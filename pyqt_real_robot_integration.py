"""
PyQt Integration Module - Synchronize Real Robot with UI Animation

This module bridges the ExecutionManager (real robot control) with the 
existing PyQt GUI to keep the robot visualization synchronized with 
actual hardware motion.

Key Features:
- ExecutionThread: Runs real robot execution off-main-thread
- Signals: Emit position updates to PyQt UI
- Animation: Graceful cell-to-cell movement visualization
- Callbacks: Hooks for status updates, error handling
"""


from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QMessageBox
from typing import List, Tuple, Optional, Callable
import time

from raspberry_pi_executor import (
    ExecutionManager,
    CommandType,
    Heading,
    path_to_commands,
)
from motion import heading_from_coords


class ExecutionThread(QThread):
    """
    Worker thread that runs path execution on real robot.
    
    Signals:
    - robotMoved: Emitted when robot reaches a waypoint
    - executionStarted: Path execution begins
    - executionProgress: Command executed (index, total)
    - executionComplete: All commands finished
    - executionError: Error occurred
    """

    robotMoved = pyqtSignal(int, int)  # (row, col)
    executionStarted = pyqtSignal(str)  # (status message)
    executionProgress = pyqtSignal(int, int)  # (current_idx, total_commands)
    executionComplete = pyqtSignal()
    executionError = pyqtSignal(str)  # (error message)


    def __init__(self, robot_state, path_coords: List[Tuple[int, int]], path_node_ids: List[int],
                 port: str = '/dev/ttyACM0', parent=None):
        """
        Initialize the ExecutionThread.

        Args:
            path_coords: List of (row, col) grid coordinates
            port: Serial port for Arduino (default: '/dev/ttyACM0')
            parent: Parent QObject (default: None)
            port: Serial port for Arduino
            parent: Parent QObject
        """
        super().__init__(parent)

        # List of (row, col) grid coordinates
        self.robot_state = robot_state
        self.path_coords: List[Tuple[int, int]] = path_coords
        self.path_node_ids: List[int] = path_node_ids

        # Serial port for Arduino (e.g., '/dev/ttyACM0')
        self.port: str = port

        # ExecutionManager instance
        self.path_coords = path_coords
        self.port = port
        self.executor: Optional[ExecutionManager] = None

        # Flag to request stop (gracefully)
        self._stop_requested: bool = False
        self._stop_requested = False

    def run(self) -> None:
        """
        Main thread function: Execute path on real robot.
        Called when self.start() is invoked.
        """
        try:
            # Create executor
            self.executor = ExecutionManager(robot_state=self.robot_state, port=self.port)

            # Attach callbacks
            self.executor.on_position_update = self._on_robot_position_update
            self.executor.on_command_sent = self._on_command_executed
            self.executor.on_error = self._on_execution_error
            self.executor.on_execution_complete = self._on_execution_complete

            # Connect to Arduino
            self.executionStarted.emit("Connecting to Arduino...")
            if not self.executor.connect():
                raise RuntimeError("Failed to connect to Arduino")

            self.executionStarted.emit(f"Executing path with {len(self.path_coords)} waypoints...")

            # Execute path (blocking until complete or error)
            self.executor.execute_path(
                self.path_coords,
                self.path_node_ids,
                start_heading=Heading.SOUTH
            )

        except Exception as e:
            error_msg = f"Execution thread error: {str(e)}"
            self.executionError.emit(error_msg)

        finally:
            if self.executor:
                self.executor.disconnect()

    def _on_robot_position_update(self, row: int, col: int) -> None:
        """Callback: Robot moved to new position."""
        self.robotMoved.emit(row, col)

    def _on_command_executed(self, cmd: CommandType) -> None:
        """Callback: Command was executed on robot."""
        if self.executor:
            # command_index is incremented after this callback in ExecutionManager,
            # so add 1 to report human-friendly progress (1..total).
            idx = self.executor.command_index + 1
            total = len(self.executor.commands)
            self.executionProgress.emit(idx, total)

    def _on_execution_complete(self) -> None:
        """Callback: All commands executed."""
        self.executionComplete.emit()

    def _on_execution_error(self, msg: str) -> None:
        """Callback: Error during execution."""
        self.executionError.emit(msg)

    def request_stop(self) -> None:
        """Request thread to stop gracefully."""
        self._stop_requested = True
        if self.executor:
            self.executor.request_stop()


class RealRobotExecutor:
    """
    High-level coordinator for integrating real robot execution with PyQt UI.
    
    Usage in AGVPathPlannerApp:
    
        class AGVPathPlannerApp(QMainWindow):
            def __init__(self):
                ...
                self.robot_executor = RealRobotExecutor(self.robot_state, self.canvas)
                
            def execute_selected_path(self):
                if self.selected_path_index >= 0:
                    path_coords = self.found_paths[self.selected_path_index]
                    path_node_ids = [...]
                    self.robot_executor.execute_path_on_real_robot(path_coords, path_node_ids)
    """

    def __init__(self, robot_state, canvas, port: str = '/dev/ttyACM0'):
        """
        Args:
            canvas: GridView instance (for updating visualization)
            port: Serial port for Arduino
        """
        super().__init__()
        self.robot_state = robot_state
        self.canvas = canvas
        self.port = port
        self.execution_thread: Optional[ExecutionThread] = None
        self.is_executing = False
        self._last_robot_pos: Optional[Tuple[int, int]] = None

        # UI callbacks
        self.on_status_changed: Optional[Callable[[str], None]] = None
        self.on_progress_changed: Optional[Callable[[int, int], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None
        self.on_execution_complete: Optional[Callable[[], None]] = None

    def execute_path_on_real_robot(self, path_coords: List[Tuple[int, int]], path_node_ids: List[int]) -> None:
        """
        Execute path on real robot with synchronized UI animation.

        Args:
            path_coords: List of (row, col) grid coordinates

        Returns:
            None (execution happens in background thread)
        """
        if self.is_executing:
            self._status("Already executing a path!")
            return

        if not path_coords or len(path_coords) < 1:
            self._error("Empty path cannot be executed")
            return

        print(f"[RealRobotExecutor] Starting execution of path: {path_coords}")
        self._last_robot_pos = None

        # Create and configure execution thread
        self.execution_thread = ExecutionThread(
            robot_state=self.robot_state,
            path_coords=path_coords,
            path_node_ids=path_node_ids,
            port=self.port,
            parent=self.canvas
        )

        # Connect signals
        self.execution_thread.robotMoved.connect(self._on_robot_moved)
        self.execution_thread.executionStarted.connect(self._on_execution_started)
        self.execution_thread.executionProgress.connect(self._on_execution_progress)
        self.execution_thread.executionComplete.connect(self._on_execution_complete)
        self.execution_thread.executionError.connect(self._on_execution_error)

        # Start thread
        self.is_executing = True
        self.execution_thread.start()

    def stop_execution(self) -> None:
        """Stop ongoing execution (gracefully)."""
        if self.execution_thread and self.execution_thread.isRunning():
            self.execution_thread.request_stop()
            self.execution_thread.wait(5000)
            self.is_executing = False

    # ---- Signal Handlers ----

    def _on_robot_moved(self, row: int, col: int) -> None:
        """Called when robot position is updated."""
        print(f"[RealRobotExecutor] Robot moved to ({row}, {col})")
        heading = None
        if self._last_robot_pos is not None:
            heading = heading_from_coords(self._last_robot_pos, (row, col))
        self._last_robot_pos = (row, col)

        # Update canvas to show robot at new position
        self.canvas.show_robot_at((row, col), heading=heading)
        if hasattr(self.canvas, "robotMoved"):
            self.canvas.robotMoved.emit(row, col)
        
        self._status(f"Robot at ({row}, {col})")

    def _on_execution_started(self, msg: str) -> None:
        """Called when execution thread starts."""
        print(f"[RealRobotExecutor] {msg}")
        self._status(msg)

    def _on_execution_progress(self, current_idx: int, total: int) -> None:
        """Called after each command is executed."""
        progress_msg = f"Command {current_idx}/{total}"
        print(f"[RealRobotExecutor] {progress_msg}")
        if self.on_progress_changed:
            self.on_progress_changed(current_idx, total)

    def _on_execution_complete(self) -> None:
        """Called when all commands are executed."""
        print("[RealRobotExecutor] Path execution complete!")
        self.is_executing = False
        self._status("Execution complete!")
        if self.on_execution_complete:
            self.on_execution_complete()

    def _on_execution_error(self, error_msg: str) -> None:
        """Called on execution error."""
        print(f"[RealRobotExecutor] ERROR: {error_msg}")
        self.is_executing = False
        self._error(error_msg)

    # ---- Helper Methods ----

    def _status(self, msg: str) -> None:
        """Update status callback."""
        if self.on_status_changed:
            self.on_status_changed(msg)

    def _error(self, msg: str) -> None:
        """Trigger error callback."""
        if self.on_error:
            self.on_error(msg)


# ============================================================================
# EXAMPLE: Integration with AGVPathPlannerApp
# ============================================================================

"""
Add this to app_pyqt.py AGVPathPlannerApp class:

class AGVPathPlannerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        ...
        
        # Initialize real robot executor
        self.robot_executor = RealRobotExecutor(
            canvas=self.canvas,
            port='/dev/ttyACM0'  # Or 'COM3' on Windows
        )
        self.robot_executor.on_status_changed = self.on_robot_status_changed
        self.robot_executor.on_progress_changed = self.on_robot_progress_changed
        self.robot_executor.on_error = self.on_robot_error
        
        # Add "Execute on Real Robot" button
        self.execute_real_btn = QPushButton("🤖 Execute Real Robot")
        self.execute_real_btn.clicked.connect(self.execute_on_real_robot)
        
    def execute_on_real_robot(self):
        '''Execute selected path on real robot hardware.'''
        if self.selected_path_index < 0:
            QMessageBox.warning(self, "No Path Selected", "Please select a path first")
            return
        
        path = self.found_paths[self.selected_path_index]
        
        # Option: Confirm before running
        reply = QMessageBox.question(
            self,
            "Execute on Real Robot?",
            f"Execute path with {len(path)} waypoints on real robot?\\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.robot_executor.execute_path_on_real_robot(path)
    
    def on_robot_status_changed(self, status: str):
        '''Update status display.'''
        self.agv_state.setText(status)
    
    def on_robot_progress_changed(self, current: int, total: int):
        '''Update progress display.'''
        pct = int(100 * current / total) if total > 0 else 0
        self.commands_text.setText(f"Execution Progress: {current}/{total} ({pct}%)")
    
    def on_robot_error(self, error_msg: str):
        '''Show error dialog.'''
        QMessageBox.critical(self, "Execution Error", error_msg)
"""


# ============================================================================
# SIMULATION MODE (For testing without hardware)
# ============================================================================

class SimulatedExecutionThread(QThread):
    """
    Mock execution thread for testing UI without hardware.
    Uses time-based simulation instead of real serial communication.
    """

    robotMoved = pyqtSignal(int, int)
    executionStarted = pyqtSignal(str)
    executionProgress = pyqtSignal(int, int)
    executionComplete = pyqtSignal()
    executionError = pyqtSignal(str)

    def __init__(self, path_coords: List[Tuple[int, int]], parent=None):
        """
        Args:
            path_coords: List of (row, col) grid coordinates
            parent: Parent QObject
        """
        super().__init__(parent)
        self.path_coords = path_coords
        self._stop_requested = False

    def run(self) -> None:
        """
        Simulate execution with timed delays.
        """
        try:
            self.executionStarted.emit(
                f"SIMULATED: Executing path with {len(self.path_coords)} waypoints..."
            )

            # Generate commands from path
            cmds = path_to_commands(self.path_coords, start_heading=Heading.SOUTH)

            # Simulate each command
            waypoint_idx = 0  # Track which waypoint we're at
            for idx, cmd in enumerate(cmds):
                if self._stop_requested:
                    break

                # Simulate command execution time
                if cmd in [CommandType.FORWARD, CommandType.BACKWARD]:
                    delay_sec = 2.0
                else:  # TURN_LEFT or TURN_RIGHT
                    delay_sec = 0.8

                print(f"[SimulatedExecution] Command {idx + 1}/{len(cmds)}: {cmd.value} ({delay_sec}s)")
                self.executionProgress.emit(idx + 1, len(cmds))

                # Update robot position after simulated execution
                if cmd == CommandType.FORWARD:
                    # Move to next waypoint
                    waypoint_idx = min(waypoint_idx + 1, len(self.path_coords) - 1)
                    pos = self.path_coords[waypoint_idx]
                    time.sleep(delay_sec)
                    self.robotMoved.emit(pos[0], pos[1])
                elif cmd == CommandType.BACKWARD:
                    # Move to previous waypoint
                    waypoint_idx = max(waypoint_idx - 1, 0)
                    pos = self.path_coords[waypoint_idx]
                    time.sleep(delay_sec)
                    self.robotMoved.emit(pos[0], pos[1])
                else:
                    # Turns don't move position, just execute delay
                    time.sleep(delay_sec)

                time.sleep(0.2)  # Brief pause between commands

            self.executionComplete.emit()

        except Exception as e:
            self.executionError.emit(str(e))

    def request_stop(self) -> None:
        """Request stop."""
        self._stop_requested = True


def create_simulator_executor(robot_state, canvas) -> "RealRobotExecutor":
    """
    Factory function to create a simulator-based executor for testing.
    
    Usage:
        executor = create_simulator_executor(self.robot_state, self.canvas)
        executor.execute_path_on_real_robot(path, path_node_ids)
    """
    class SimulatedRobotExecutor(RealRobotExecutor):
        def execute_path_on_real_robot(self, path_coords: List[Tuple[int, int]], path_node_ids: List[int]) -> None:
            if self.is_executing:
                self._status("Already executing a path!")
                return

            self.execution_thread = SimulatedExecutionThread(
                path_coords=path_coords,
                parent=canvas
            )

            self.execution_thread.robotMoved.connect(self._on_robot_moved)
            self.execution_thread.executionStarted.connect(self._on_execution_started)
            self.execution_thread.executionProgress.connect(self._on_execution_progress)
            self.execution_thread.executionComplete.connect(self._on_execution_complete)
            self.execution_thread.executionError.connect(self._on_execution_error)

            self.is_executing = True
            self.execution_thread.start()

    return SimulatedRobotExecutor(robot_state, canvas, port='SIMULATION')


if __name__ == "__main__":
    print(__doc__)
