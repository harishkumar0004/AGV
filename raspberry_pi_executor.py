"""
Raspberry Pi Master Controller - AGV Path Execution

This module implements the master-slave execution model:
1. Accepts a path (list of grid coordinates)
2. Converts path to execution commands (F/B/L/R)
3. Sends commands ONE at a time to Arduino
4. Waits for explicit ACK (DONE) before next command
5. Emits signals for PyQt UI synchronization

Architecture:
- ExecutionManager: Orchestrates the execution flow
- SerialCommander: Handles protocol with Arduino
- Runs in separate QThread to avoid blocking UI
"""

import serial
import time
from threading import Event, Lock
from enum import Enum
from typing import List, Tuple, Optional, Callable

class ExecutionState(Enum):
    """Master controller states."""
    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    READY_TO_SEND = "READY_TO_SEND"
    WAITING_FOR_ACK = "WAITING_FOR_ACK"
    ACK_RECEIVED = "ACK_RECEIVED"
    ERROR = "ERROR"
    EXECUTION_COMPLETE = "EXECUTION_COMPLETE"


class CommandType(Enum):
    """Motion commands."""
    FORWARD = 'f'      # Move forward 1 cell (50cm)
    BACKWARD = 'b'     # Move backward 1 cell
    TURN_LEFT = 'l'    # 90° CCW turn
    TURN_RIGHT = 'r'   # 90° CW turn


class Heading(Enum):
    """Cardinal directions for path heading."""
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


# Direction vectors: (row_delta, col_delta)
DIR_VECTORS = {
    Heading.NORTH: (-1, 0),
    Heading.EAST: (0, 1),
    Heading.SOUTH: (1, 0),
    Heading.WEST: (0, -1),
}


# ============================================================================
# SERIAL COMMUNICATION LAYER
# ============================================================================

class SerialCommander:
    """
    Low-level serial protocol handler.
    Encapsulates all communication with Arduino.
    """

    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 9600, 
                 timeout: float = 10.0, read_timeout: float = 0.1):
        """
        Args:
            port: Serial port (e.g., '/dev/ttyACM0' on Pi, 'COM3' on Windows)
            baudrate: Baud rate (must match Arduino)
            timeout: Max time to wait for ACK from Arduino
            read_timeout: Blocking read timeout (non-blocking simulation)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.read_timeout = read_timeout
        self.serial = None
        self._lock = Lock()

    def connect(self) -> bool:
        """Establish serial connection."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.read_timeout,
                write_timeout=1.0
            )
            print(f"[SerialCommander] Connected to {self.port} @ {self.baudrate}bps")
            # Flush buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(2.0)  # Give Arduino time to reset after port open
            return True
        except serial.SerialException as e:
            print(f"[SerialCommander] ERROR: Failed to connect: {e}")
            return False

    def disconnect(self) -> None:
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[SerialCommander] Disconnected")

    def send_command(self, cmd: CommandType) -> bool: 
        """
        Send a single command to Arduino.

        Args:
            cmd: CommandType enum

        Returns:
            True if sent successfully
        """
        if not self.serial or not self.serial.is_open:
            print("[SerialCommander] ERROR: Serial not connected")
            return False

        try:
            with self._lock:
                msg = cmd.value  # single-byte command, no newline
                self.serial.write(msg.encode('ascii'))
                self.serial.flush()
                print(f"[SerialCommander] Sent: {repr(msg)}")
                return True
        except serial.SerialException as e:
            print(f"[SerialCommander] ERROR: Failed to send: {e}")
            return False

    def wait_for_ack(self) -> Tuple[bool, Optional[str]]:
        """
        Block and wait for ACK from Arduino.

        Returns:
            (success, response_str)
            - success: True if "DONE\n" received before timeout
            - response_str: The raw response string, or None on timeout
        """
        if not self.serial or not self.serial.is_open:
            print("[SerialCommander] ERROR: Serial not connected")
            return False, None

        start_time = time.time()

        try:
            while time.time() - start_time < self.timeout:
                with self._lock:
                    line = self.serial.readline()

                if line:
                    response = line.decode('utf-8', errors='ignore').strip()
                    print(f"[SerialCommander] Received: {repr(response)} (raw={repr(line)})")

                    if response == "DONE":
                        return True, response
                    elif response.startswith("ERROR"):
                        return False, response
                    else:
                        # Unexpected response, but continue waiting
                        print(f"[SerialCommander] WARN: Unexpected response: {response}")
                        continue

                time.sleep(0.01)  # Small sleep to avoid busy-waiting

        except serial.SerialException as e:
            print(f"[SerialCommander] ERROR: Serial error during read: {e}")
            return False, None

        print(f"[SerialCommander] ERROR: Timeout waiting for ACK after {self.timeout}s")
        return False, None

    def send_and_wait(self, cmd: CommandType) -> bool:
        """
        Send command and block until ACK or timeout.

        Args:
            cmd: CommandType to send

        Returns:
            True if ACK received, False on error or timeout
        """
        if not self.send_command(cmd):
            return False

        success, response = self.wait_for_ack()
        if success:
            print(f"[SerialCommander] Command completed: {cmd.value}")
            return True
        else:
            print(f"[SerialCommander] Command failed: {cmd.value} (response: {response})")
            return False


# ============================================================================
# COMMAND GENERATION
# ============================================================================

def heading_from_coords(current: Tuple[int, int], 
                       next_pos: Tuple[int, int]) -> Optional[Heading]:
    """
    Determine heading direction from current to next position.

    Args:
        current: (row, col)
        next_pos: (row, col)

    Returns:
        Heading enum or None if same position
    """
    dr = next_pos[0] - current[0]
    dc = next_pos[1] - current[1]

    if dr < 0:
        return Heading.NORTH
    if dr > 0:
        return Heading.SOUTH
    if dc > 0:
        return Heading.EAST
    if dc < 0:
        return Heading.WEST

    return None


def path_to_commands(path_coords: List[Tuple[int, int]], 
                     start_heading: Heading = Heading.SOUTH) -> List[CommandType]:
    """
    Convert path (list of grid coordinates) to execution commands.

    Algorithm:
    1. Track current heading (changes as we move)
    2. For each move in path:
       - Determine required heading
       - Calculate turn needed (0, 90°, 180°, 270°)
       - Emit turn commands (L, R, or R+R)
       - Emit move command (F, B)

    Args:
        path_coords: List of (row, col) tuples
        start_heading: Initial robot heading (default: facing SOUTH)

    Returns:
        List of CommandType enums
    """
    if len(path_coords) < 2:
        return []

    commands = []
    current_heading = start_heading

    for i in range(len(path_coords) - 1):
        current = path_coords[i]
        next_pos = path_coords[i + 1]

        required_heading = heading_from_coords(current, next_pos)
        if required_heading is None:
            print(f"[path_to_commands] WARN: Same position repeated at index {i}")
            continue

        # Calculate turn: (required - current) % 4
        # 0: No turn, 1: 90° CW, 2: 180°, 3: 90° CCW
        turn_needed = (required_heading.value - current_heading.value) % 4

        if turn_needed == 1:  # 90° CW
            commands.append(CommandType.TURN_RIGHT)
        elif turn_needed == 3:  # 90° CCW
            commands.append(CommandType.TURN_LEFT)
        elif turn_needed == 2:  # 180° turn (2 × 90° CW)
            commands.append(CommandType.TURN_RIGHT)
            commands.append(CommandType.TURN_RIGHT)

        # Move forward in required direction
        commands.append(CommandType.FORWARD)

        current_heading = required_heading

    print(f"[path_to_commands] Converted {len(path_coords)} coords → {len(commands)} commands")
    return commands


# ============================================================================
# EXECUTION MANAGER (MASTER STATE MACHINE)
# ============================================================================

class ExecutionManager:
    """
    Orchestrates execution of a path on the real robot.

    Responsibilities:
    - Manage master state machine (IDLE → READY_TO_SEND → WAITING_FOR_ACK → ACK_RECEIVED)
    - Convert path coordinates to commands
    - Send commands to Arduino one at a time
    - Track position and emit position updates
    - Handle timeouts and errors

    Usage:
        executor = ExecutionManager(robot_state, port='/dev/ttyACM0')
        executor.on_position_update = lambda r, c: print(f"Robot at ({r}, {c})")
        executor.execute_path(path_coords, path_node_ids)
    """

    def __init__(self, robot_state, port: str = '/dev/ttyACM0', baudrate: int = 9600):
        """
        Args:
            port: Serial port for Arduino
            baudrate: Serial baud rate
        """
        self.robot_state = robot_state
        self.serial_cmd = SerialCommander(port=port, baudrate=baudrate)
        self.state = ExecutionState.IDLE
        self.commands: List[CommandType] = []
        self.command_index = 0
        self.path_coords: List[Tuple[int, int]] = []
        self.path_node_ids: List[int] = []
        self.path_step_index = 0
        self.current_position: Tuple[int, int] = (0, 0)
        self.current_heading = Heading.SOUTH
        self._stop_requested = False

        # Callbacks
        self.on_state_change: Optional[Callable[[ExecutionState], None]] = None
        self.on_position_update: Optional[Callable[[int, int], None]] = None
        self.on_command_sent: Optional[Callable[[CommandType], None]] = None
        self.on_execution_complete: Optional[Callable[[], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None

    def _set_state(self, new_state: ExecutionState) -> None:
        """Update state and emit callback."""
        if self.state != new_state:
            print(f"[ExecutionManager] State: {self.state.value} → {new_state.value}")
            self.state = new_state
            if self.on_state_change:
                self.on_state_change(new_state)

    def _error(self, msg: str) -> None:
        """Log error and emit callback."""
        print(f"[ExecutionManager] ERROR: {msg}")
        if self.robot_state:
            self.robot_state.set_status("ERROR")
        self._set_state(ExecutionState.ERROR)
        if self.on_error:
            self.on_error(msg)

    def connect(self) -> bool:
        """Establish connection to Arduino."""
        self._set_state(ExecutionState.INITIALIZING)
        if self.serial_cmd.connect():
            self._set_state(ExecutionState.IDLE)
            return True
        else:
            self._error("Failed to connect to Arduino")
            return False

    def disconnect(self) -> None:
        """Close connection to Arduino."""
        self.serial_cmd.disconnect()
        self._set_state(ExecutionState.IDLE)

    def execute_path(self, path_coords: List[Tuple[int, int]], path_node_ids: List[int],
                    start_heading: Heading = Heading.SOUTH) -> None:
        """
        Execute a path on the real robot.

        Args:
            path_coords: List of (row, col) grid coordinates
            start_heading: Initial robot heading
        """
        if not path_coords or len(path_coords) < 1:
            self._error("Empty path")
            return

        if not path_node_ids or len(path_node_ids) != len(path_coords):
            self._error("path_node_ids must be same length as path_coords")
            return

        print(f"[ExecutionManager] Starting execution of path with {len(path_coords)} waypoints")

        self.path_coords = path_coords
        self.path_node_ids = path_node_ids
        self.path_step_index = 0
        self.current_position = path_coords[0]
        self.current_heading = start_heading
        self.commands = path_to_commands(path_coords, start_heading)
        self.command_index = 0
        self._stop_requested = False

        if not self.commands:
            self._error("No commands generated from path")
            return

        if self.robot_state:
            self.robot_state.set_status("MOVING")
            self.robot_state.update_node(self.path_node_ids[0])

        # Begin state machine
        self._execute_next_command()

    def _execute_next_command(self) -> None:
        """
        Core state machine iteration.
        Sends next command and waits for ACK.
        """
        if self.robot_state and self.robot_state.status == "ERROR":
            self._error("Execution stopped due to ERROR status")
            return
        if self._stop_requested:
            self._set_state(ExecutionState.IDLE)
            if self.robot_state:
                self.robot_state.set_status("IDLE")
            return

        if self.command_index >= len(self.commands):
            # All commands executed
            self._set_state(ExecutionState.EXECUTION_COMPLETE)
            print("[ExecutionManager] Path execution complete!")
            if self.robot_state:
                self.robot_state.set_status("IDLE")
            if self.on_execution_complete:
                self.on_execution_complete()
            return

        self._set_state(ExecutionState.READY_TO_SEND)
        cmd = self.commands[self.command_index]

        # Send command
        self._set_state(ExecutionState.WAITING_FOR_ACK)
        if not self.serial_cmd.send_and_wait(cmd):
            self._error(f"Command failed: {cmd.value}")
            return

        # Command executed successfully
        self._set_state(ExecutionState.ACK_RECEIVED)
        self._update_position(cmd)
        self._update_robot_state_for_command(cmd)

        if self.on_command_sent:
            self.on_command_sent(cmd)

        self.command_index += 1

        # Continue to next command (non-blocking)
        self._execute_next_command()

    def request_stop(self) -> None:
        """Request to stop after current command completes."""
        self._stop_requested = True

    def _update_robot_state_for_command(self, cmd: CommandType) -> None:
        """
        Update RobotState current node based on movement commands.
        """
        if not self.robot_state:
            return

        if cmd == CommandType.FORWARD:
            if self.path_step_index + 1 < len(self.path_node_ids):
                self.path_step_index += 1
                self.robot_state.update_node(self.path_node_ids[self.path_step_index])
        elif cmd == CommandType.BACKWARD:
            if self.path_step_index - 1 >= 0:
                self.path_step_index -= 1
                self.robot_state.update_node(self.path_node_ids[self.path_step_index])

    def _update_position(self, cmd: CommandType) -> None:
        """
        Update tracked position based on executed command.

        Args:
            cmd: CommandType that was just executed
        """
        try:
            if cmd == CommandType.FORWARD:
                # Move in current heading direction
                heading_key = self.current_heading if isinstance(self.current_heading, Heading) else Heading(self.current_heading)
                dr, dc = DIR_VECTORS[heading_key]
                new_r = self.current_position[0] + dr
                new_c = self.current_position[1] + dc
                self.current_position = (new_r, new_c)
                print(f"[ExecutionManager] Moved FORWARD to {self.current_position}")

            elif cmd == CommandType.BACKWARD:
                # Move opposite to current heading
                heading_key = self.current_heading if isinstance(self.current_heading, Heading) else Heading(self.current_heading)
                dr, dc = DIR_VECTORS[heading_key]
                new_r = self.current_position[0] - dr
                new_c = self.current_position[1] - dc
                self.current_position = (new_r, new_c)
                print(f"[ExecutionManager] Moved BACKWARD to {self.current_position}")

            elif cmd == CommandType.TURN_LEFT:
                # Heading: CCW (subtract 1, mod 4)
                current_val = self.current_heading.value if isinstance(self.current_heading, Heading) else self.current_heading
                self.current_heading = Heading((current_val - 1) % 4)
                print(f"[ExecutionManager] Turned LEFT, now facing {self.current_heading.name}")

            elif cmd == CommandType.TURN_RIGHT:
                # Heading: CW (add 1, mod 4)
                current_val = self.current_heading.value if isinstance(self.current_heading, Heading) else self.current_heading
                self.current_heading = Heading((current_val + 1) % 4)
                print(f"[ExecutionManager] Turned RIGHT, now facing {self.current_heading.name}")

        except Exception as e:
            self._error(f"Error updating position: {str(e)}")
            return

        # Emit position update signal for PyQt UI
        if self.on_position_update:
            self.on_position_update(self.current_position[0], self.current_position[1])


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

def example_standalone():
    """Standalone test (without PyQt)."""
    print("=" * 70)
    print("AGV Execution Manager - Standalone Example")
    print("=" * 70)

    # Example path: (0,0) → (0,1) → (1,1) → (2,1)
    path = [(0, 0), (0, 1), (1, 1), (2, 1)]

    print(f"\nPath: {path}")

    # Generate commands
    cmds = path_to_commands(path, start_heading=Heading.SOUTH)
    print(f"Commands: {[c.value for c in cmds]}")
    print()

    class _DummyRobotState:
        def __init__(self):
            self._status = "IDLE"

        @property
        def status(self):
            return self._status

        def set_status(self, new_status):
            self._status = new_status

        def update_node(self, new_node):
            pass

    # Create manager
    executor = ExecutionManager(robot_state=_DummyRobotState(), port='/dev/ttyACM0')

    # Attach callbacks
    executor.on_state_change = lambda s: print(f"  [UI] State → {s.value}")
    executor.on_position_update = lambda r, c: print(f"  [UI] Robot now at ({r}, {c})")
    executor.on_command_sent = lambda c: print(f"  [UI] Command executed: {c.value}")
    executor.on_error = lambda msg: print(f"  [UI] ERROR: {msg}")

    # Connect
    if not executor.connect():
        print("Failed to connect to Arduino. Running simulation mode...")
        return

    # Execute
    path_node_ids = list(range(1, len(path) + 1))
    executor.execute_path(path, path_node_ids, start_heading=Heading.SOUTH)

    # Cleanup
    executor.disconnect()


if __name__ == "__main__":
    # For testing without Arduino, modify SerialCommander.send_and_wait()
    # to simulate responses
    print(__doc__)
    example_standalone()
