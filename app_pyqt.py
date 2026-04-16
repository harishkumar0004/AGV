import os
import sys
import time
from typing import List, Tuple
import cv2
from PyQt5.QtCore import Qt, QSize, pyqtSignal, QTimer
from PyQt5.QtGui import QBrush, QColor, QFont, QIcon, QImage, QPainter, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QFrame, QGraphicsEllipseItem, QGraphicsPathItem,
    QGraphicsRectItem, QGraphicsScene, QGraphicsTextItem, QGraphicsView,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QMainWindow,
    QMessageBox, QPushButton, QSizePolicy, QSplitter, QTextEdit, QToolBox, QTreeWidget,
    QTreeWidgetItem, QVBoxLayout, QWidget
)

from pyqt_real_robot_integration import create_simulator_executor
from pyqt_real_robot_integration import RealRobotExecutor
from continuous_execution_manager import ContinuousExecutionManager
from astar import get_path_length, k_shortest_paths
from grid import (
    ROWS,
    COLS,
    create_networkx_grid,
    grid_coord_to_node_id,
    node_id_to_grid_coord,
)
from visualize import PATH_COLORS
from motion import NORTH, EAST, SOUTH, WEST, heading_from_coords
from robot_state import RobotState
from mission_controller import MissionController
from apriltag_view import ApriltagView

# Configuration
NUM_PATHS = 5
CELL_SIZE = 80


class GridView(QGraphicsView):
    cellClicked = pyqtSignal(int, int)
    robotMoved = pyqtSignal(int, int)  # Signal when robot moves during animation
    robotHeadingChanged = pyqtSignal(object)  # Emits heading int (or None)

    def __init__(self, rows, cols, cell_size=CELL_SIZE, parent=None):
        super().__init__(parent)
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.TextAntialiasing)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setAlignment(Qt.AlignCenter)
        self.setFrameShape(QFrame.NoFrame)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(self.cols * self.cell_size, self.rows * self.cell_size)

        self.grid_cells = {}
        self.grid_labels = {}
        self.path_items = []
        self.path_cells = []
        self.robot_item = None
        self.robot_radius = self.cell_size * 0.18  # Reduced from 0.28
        self.robot_heading = None
        self.robot_heading_item = None
        self.robot_heading_length = self.robot_radius * 1.2

        self.start_coord = None
        self.goal_coord = None
        self.blocked_nodes = set()

        self.animation_timer = QTimer(self)
        self.animation_timer.timeout.connect(self._advance_robot)
        self.animation_path = []
        self.animation_index = 0
        self.animation_callback = None

        self._draw_grid()

    def _draw_grid(self):
        self.scene.clear()
        self.grid_cells.clear()
        self.grid_labels.clear()
        self.path_items.clear()
        self.path_cells.clear()
        self.robot_item = None

        width = self.cols * self.cell_size
        height = self.rows * self.cell_size
        self.scene.setSceneRect(0, 0, width, height)

        base_pen = QPen(QColor(30, 30, 30), 1)
        base_brush = QBrush(QColor(250, 250, 250))
        label_font = QFont("JetBrains Mono", 9)
        label_font.setBold(True)

        for r in range(self.rows):
            for c in range(self.cols):
                rect = QGraphicsRectItem(
                    c * self.cell_size,
                    r * self.cell_size,
                    self.cell_size,
                    self.cell_size,
                )
                rect.setPen(base_pen)
                rect.setBrush(base_brush)
                rect.setZValue(0)
                self.scene.addItem(rect)
                self.grid_cells[(r, c)] = rect

                cell_id = grid_coord_to_node_id(r, c, self.cols)
                label = QGraphicsTextItem(str(cell_id))
                label.setFont(label_font)
                label.setDefaultTextColor(QColor(120, 120, 120))
                label.setZValue(1)
                label.setOpacity(0.75)
                self.scene.addItem(label)
                self._center_label(label, r, c)
                self.grid_labels[(r, c)] = label

        self._apply_cell_styles()
        self._ensure_robot_item()
        self.fitInView(self.scene.sceneRect(), Qt.IgnoreAspectRatio)

    def _center_label(self, label, row, col):
        rect = label.boundingRect()
        x = col * self.cell_size + (self.cell_size - rect.width()) / 2
        y = row * self.cell_size + (self.cell_size - rect.height()) / 2
        label.setPos(x, y)

    def _apply_cell_styles(self):
        for (r, c), rect in self.grid_cells.items():
            rect.setPen(QPen(QColor(30, 30, 30), 1))
            rect.setBrush(QBrush(QColor(250, 250, 250)))

        for coord in self.blocked_nodes:
            rect = self.grid_cells.get(coord)
            if rect:
                rect.setPen(QPen(QColor(190, 40, 40), 2))
                rect.setBrush(QBrush(QColor(220, 60, 60), Qt.BDiagPattern))

        if self.start_coord in self.grid_cells:
            rect = self.grid_cells[self.start_coord]
            rect.setPen(QPen(QColor(20, 140, 60), 3))
            rect.setBrush(QBrush(QColor(80, 200, 120)))

        if self.goal_coord in self.grid_cells:
            rect = self.grid_cells[self.goal_coord]
            rect.setPen(QPen(QColor(170, 120, 20), 3))
            rect.setBrush(QBrush(QColor(240, 210, 80)))

    def set_overlays(self, start_coord, goal_coord, blocked_nodes):
        self.start_coord = start_coord
        self.goal_coord = goal_coord
        self.blocked_nodes = set(blocked_nodes)
        self._apply_cell_styles()

    def set_paths(self, paths, selected_index=-1, visible=None):
        self._clear_paths()
        if visible is None:
            visible = [True for _ in paths]

        for idx, path_coords in enumerate(paths):
            if not visible[idx]:
                continue
            color = QColor(PATH_COLORS[idx % len(PATH_COLORS)])
            is_selected = idx == selected_index
            alpha = 220 if is_selected else 160
            pen_width = 4 if is_selected else 2
            pen = QPen(color, pen_width)
            pen.setCosmetic(True)
            pen.setColor(QColor(color.red(), color.green(), color.blue(), alpha))

            path = QPainterPath()
            for i, (r, c) in enumerate(path_coords):
                center = self._cell_center(r, c)
                if i == 0:
                    path.moveTo(center[0], center[1])
                else:
                    path.lineTo(center[0], center[1])

            path_item = QGraphicsPathItem(path)
            path_item.setPen(pen)
            path_item.setZValue(5)
            self.scene.addItem(path_item)
            self.path_items.append(path_item)

            for i, (r, c) in enumerate(path_coords):
                if i == 0 or i == len(path_coords) - 1:
                    continue
                rect = QGraphicsRectItem(
                    c * self.cell_size,
                    r * self.cell_size,
                    self.cell_size,
                    self.cell_size,
                )
                rect.setPen(QPen(color, 1))
                fill = QColor(color)
                fill.setAlpha(80)
                rect.setBrush(QBrush(fill))
                rect.setZValue(3)
                self.scene.addItem(rect)
                self.path_cells.append(rect)

    def update_path_styles(self, paths, selected_index=-1, visible=None):
        self.set_paths(paths, selected_index, visible)

    def _clear_paths(self):
        for item in self.path_items:
            self.scene.removeItem(item)
        for item in self.path_cells:
            self.scene.removeItem(item)
        self.path_items.clear()
        self.path_cells.clear()

    def _cell_center(self, row, col):
        x = col * self.cell_size + self.cell_size / 2
        y = row * self.cell_size + self.cell_size / 2
        return (x, y)

    def _ensure_robot_item(self):
        if self.robot_item is None:
            self.robot_item = QGraphicsEllipseItem(0, 0, self.robot_radius * 2, self.robot_radius * 2)
            self.robot_item.setBrush(QBrush(QColor(66, 150, 240)))  # Brighter blue
            self.robot_item.setPen(QPen(QColor(30, 100, 180), 0.5))  # Thinner border
            self.robot_item.setZValue(10)
            self.scene.addItem(self.robot_item)
            self.robot_item.setVisible(False)
        if self.robot_heading_item is None:
            self.robot_heading_item = QGraphicsPathItem()
            heading_pen = QPen(QColor(10, 60, 120), 2.5)
            heading_pen.setCapStyle(Qt.RoundCap)
            self.robot_heading_item.setPen(heading_pen)
            self.robot_heading_item.setZValue(11)
            self.scene.addItem(self.robot_heading_item)
            self.robot_heading_item.setVisible(False)

    def _heading_to_vector(self, heading):
        if heading == NORTH:
            return (0, -1)
        if heading == EAST:
            return (1, 0)
        if heading == SOUTH:
            return (0, 1)
        if heading == WEST:
            return (-1, 0)
        return None

    def show_robot_at(self, coord, heading=None):
        if coord is None:
            if self.robot_item:
                self.robot_item.setVisible(False)
            if self.robot_heading_item:
                self.robot_heading_item.setVisible(False)
            return
        self._ensure_robot_item()
        center = self._cell_center(coord[0], coord[1])
        self.robot_item.setRect(
            center[0] - self.robot_radius,
            center[1] - self.robot_radius,
            self.robot_radius * 2,
            self.robot_radius * 2,
        )
        self.robot_item.setVisible(True)

        if heading is not None:
            self.robot_heading = heading
            self.robotHeadingChanged.emit(heading)

        if self.robot_heading is None:
            if self.robot_heading_item:
                self.robot_heading_item.setVisible(False)
            return

        vec = self._heading_to_vector(self.robot_heading)
        if vec is None:
            if self.robot_heading_item:
                self.robot_heading_item.setVisible(False)
            return

        end_x = center[0] + vec[0] * self.robot_heading_length
        end_y = center[1] + vec[1] * self.robot_heading_length
        path = QPainterPath()
        path.moveTo(center[0], center[1])
        path.lineTo(end_x, end_y)
        self.robot_heading_item.setPath(path)
        self.robot_heading_item.setVisible(True)

    def set_robot_position(self, coord, heading=None):
        """Compatibility helper used by startup positioning code."""
        self.show_robot_at(coord, heading=heading)

    def animate_path(self, path_coords, callback=None):
        if not path_coords:
            return
        self.animation_timer.stop()
        self.animation_path = list(path_coords)
        self.animation_index = 0
        self.animation_callback = callback
        # Show initial heading immediately
        initial_heading = None
        if len(self.animation_path) > 1:
            initial_heading = heading_from_coords(self.animation_path[0], self.animation_path[1])
        self.show_robot_at(self.animation_path[0], heading=initial_heading)
        self.robotMoved.emit(self.animation_path[0][0], self.animation_path[0][1])
        self.animation_timer.start(300)

    def _heading_for_index(self, idx):
        if not self.animation_path or len(self.animation_path) < 2:
            return None
        if idx == 0:
            return heading_from_coords(self.animation_path[0], self.animation_path[1])
        if idx > 0:
            return heading_from_coords(self.animation_path[idx - 1], self.animation_path[idx])
        return None

    def _advance_robot(self):
        if self.animation_index >= len(self.animation_path):
            self.animation_timer.stop()
            if self.animation_callback:
                self.animation_callback()
            return
        coord = self.animation_path[self.animation_index]
        heading = self._heading_for_index(self.animation_index)
        self.show_robot_at(coord, heading=heading)
        self.robotMoved.emit(coord[0], coord[1])  # Emit position signal
        self.animation_index += 1

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            pos = self.mapToScene(event.pos())
            row = int(pos.y() / self.cell_size)
            col = int(pos.x() / self.cell_size)
            if 0 <= row < self.rows and 0 <= col < self.cols:
                self.cellClicked.emit(row, col)
                return
        super().mousePressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.scene.sceneRect(), Qt.IgnoreAspectRatio)


class AGVPathPlannerApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.robot_state = RobotState()
        self.robot_state.node_changed.connect(self.on_node_changed)
        self.robot_state.status_changed.connect(self.on_status_changed)
        self.setWindowTitle("AGV Path Planner - Advanced")
        self.setGeometry(100, 100, 1600, 900)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #ecf0f1;
            }
            QLabel {
                color: #2c3e50;
            }
        """)

        self.found_paths = []
        self.path_visible = []
        self.path_distances = []  # Store path distances in mm
        self.blocked_nodes = set()
        self.G = create_networkx_grid(rows=ROWS, cols=COLS)
        self.start_coord = None
        self.goal_coord = None
        self.selected_path_index = -1
        self.block_mode = False
        self.simulation_mode = True  # Default to simulation mode (no Arduino required)

        self.init_ui()
        
        # Initialize continuous execution manager (lazy connection)
        self.continuous_executor = ContinuousExecutionManager(
            robot_state=self.robot_state,
            port='/dev/ttyUSB0',
            baudrate=115200,
            layout_file='apriltag_layout.json',
            use_detector=True
        )
        self.continuous_executor.set_heading(self.canvas.robot_heading or SOUTH)
        
        # Set up callbacks
        self.continuous_executor.on_position_update = self.on_executor_position_update
        self.continuous_executor.on_waypoint_verified = self.on_executor_waypoint_verified
        self.continuous_executor.on_execution_complete = self.on_executor_complete
        self.continuous_executor.on_error = self.show_error_dialog

        self.camera_preview_interval_ms = max(80, int(os.getenv("AGV_PREVIEW_INTERVAL_MS", "125")))
        self.camera_preview_timer = QTimer(self)
        self.camera_preview_timer.timeout.connect(self.refresh_camera_preview)
        self.camera_preview_timer.start(self.camera_preview_interval_ms)
        QTimer.singleShot(0, self.start_camera_preview)
        
        # Keep backward compatibility
        self.robot_executor = self.continuous_executor
        
        # Set robot initial position to Tag 1 (row=0, col=0)
        self._set_robot_to_tag_1()
        
        # Note: Arduino connection is deferred until execution is requested
        # This allows path planning without requiring hardware to be connected
        
        self.mission_controller = MissionController(
            robot_state=self.robot_state,
            robot_executor=self.robot_executor,
            planner=self.compute_single_path,
            get_selected_path=self.get_selected_path_coords,
        )
        self.mission_controller.on_phase_changed = self.on_mission_phase_changed
        self.mission_controller.on_active_path_changed = self.show_active_phase_path
        self.on_node_changed(self.robot_state.current_node)

    def on_node_changed(self, node_id: int):
        coord = node_id_to_grid_coord(node_id, COLS)
        self.current_node.setText(f"Node {node_id} ({coord[0]}, {coord[1]})")
        self.canvas.show_robot_at(coord)
        if hasattr(self, "april_view") and self.april_view is not None:
            self.april_view.set_robot_node(node_id)

    def on_status_changed(self, status: str):
        self.agv_state.setText(status)

    def on_mission_phase_changed(self, phase: str):
        self.mission_phase.setText(phase)

    def show_active_phase_path(self, path_coords):
        if not path_coords:
            return
        self.canvas.set_paths([path_coords], selected_index=0, visible=[True])
    def execute_on_real_robot(self):
        """Execute selected path on actual hardware using continuous motion."""
        try:
            if self.selected_path_index < 0:
                QMessageBox.warning(self, "No Path", "Select a path first")
                return
            
            # Extract path data from the tuple (path_coords, path_ids, length, cost)
            path_data = self.found_paths[self.selected_path_index]
            path_coords = path_data[0] if isinstance(path_data, tuple) else path_data
            path_distance = path_data[2] if isinstance(path_data, tuple) and len(path_data) > 2 else 0
            
            if not path_coords:
                QMessageBox.warning(self, "Invalid Path", "Path is empty")
                return
            
            start_id = grid_coord_to_node_id(path_coords[0][0], path_coords[0][1], COLS)
            goal_id = grid_coord_to_node_id(path_coords[-1][0], path_coords[-1][1], COLS)
            self.target_node.setText(f"Node {goal_id} ({path_coords[-1][0]}, {path_coords[-1][1]})")
            
            # Display path distance and motion info
            num_cells = len(path_coords) - 1
            distance_mm = num_cells * 500  # Each cell is 500mm
            
            # Check if simulation mode is enabled
            if self.simulation_mode:
                self.commands_text.setText(
                    f"🎮 Simulated Execution (No Hardware)\n"
                    f"Path: {start_id} → {goal_id}\n"
                    f"Distance: {distance_mm}mm ({num_cells} cells)\n"
                    f"Status: Starting animation...\n"
                )
                
                # Run simulation
                self.simulate_path_execution(path_coords)
                return
            
            # Real hardware execution
            self.commands_text.setText(
                f"🚀 Continuous Motion Execution\n"
                f"Path: {start_id} → {goal_id}\n"
                f"Distance: {distance_mm}mm ({num_cells} cells)\n"
                f"Status: Establishing connection...\n"
            )
            
            path_node_ids = [grid_coord_to_node_id(r, c, COLS) for r, c in path_coords]
            
            if not self._ensure_arduino_connection(
                f"🔌 Connecting to Arduino at /dev/ttyUSB0...\n"
                f"Path: {start_id} → {goal_id}\n"
                f"Distance: {distance_mm}mm ({num_cells} cells)\n"
            ):
                return
            
            # Execute using new continuous executor (non-blocking, uses callbacks)
            self.continuous_executor.execute_path_continuous(
                path_coords=path_coords,
                path_node_ids=path_node_ids,
                direction='F'  # Forward direction
            )
        except Exception as e:
            print(f"[ERROR] execute_on_real_robot failed: {e}")
            import traceback
            traceback.print_exc()
            self.show_error_dialog(f"Failed to execute path: {str(e)}")
    
    def update_progress(self, current: int, total: int):
        """Update progress display."""
        pct = int(100 * current / total) if total > 0 else 0
        self.commands_text.setText(f"Progress: {current}/{total} ({pct}%)")

    def _is_arduino_connected(self) -> bool:
        """Check whether the executor already has an open Arduino serial port."""
        if hasattr(self.continuous_executor, "is_connected"):
            return self.continuous_executor.is_connected()

        return (
            self.continuous_executor.serial is not None and
            hasattr(self.continuous_executor.serial, 'is_open') and
            self.continuous_executor.serial.is_open
        )

    def _set_robot_to_tag_1(self):
        """Set robot's initial position to Tag 1 (row=0, col=0)."""
        tag_1_coord = (0, 0)  # Tag 1 is at row 0, col 0
        tag_1_id = 1

        # Keep the planner state aligned with the robot's real startup cell.
        self.start_coord = tag_1_coord
        self.goal_coord = None

        # Update robot state
        self.robot_state.set_position_from_tag(tag_1_id)

        # Update canvas to show robot at Tag 1
        self.canvas.set_robot_position(tag_1_coord, heading=SOUTH)

        # Update UI labels
        if hasattr(self, 'start_input'):
            self.start_input.setText("Node 1 (0, 0)")
        if hasattr(self, 'goal_input'):
            self.goal_input.clear()
        if hasattr(self, 'target_node'):
            self.target_node.setText("---")
    
    def _ensure_arduino_connection(self, status_text: str) -> bool:
        """Open the Arduino serial connection on demand."""
        if self._is_arduino_connected():
            return True

        self.commands_text.setText(status_text)
        QApplication.processEvents()

        if self.continuous_executor.connect():
            return True

        self.commands_text.setText(
            f"{status_text}"
            "❌ Could not open Arduino serial connection.\n\n"
            "Check USB cable, /dev/ttyUSB0, firmware upload, and serial permissions.\n"
        )
        return False
    
    def show_error_dialog(self, error_msg: str):
        """Show error popup."""
        self.commands_text.setText(f"❌ Execution Error\n{error_msg}")
        QMessageBox.critical(self, "Execution Error", error_msg)

    def start_camera_preview(self):
        """Start the detector as soon as the app launches."""
        if not hasattr(self, 'continuous_executor'):
            return
        self._set_camera_preview_status("Starting AprilTag camera preview...")
        self.continuous_executor.ensure_detector_running(wait=False)

    def refresh_camera_preview(self):
        """Display the latest annotated detector frame inside the PyQt UI."""
        if not hasattr(self, 'camera_feed_label'):
            return

        detector = getattr(self.continuous_executor, 'detector', None) if hasattr(self, 'continuous_executor') else None
        if detector is None:
            self._set_camera_preview_status("Starting AprilTag camera preview...")
            return

        frame = detector.get_latest_preview_frame()
        if frame is None:
            detector_state = getattr(detector, 'state', None)
            state_label = detector_state.value if detector_state is not None else "IDLE"
            self._set_camera_preview_status(f"Camera preview waiting... detector state: {state_label}")
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_frame.shape
        image = QImage(
            rgb_frame.data,
            width,
            height,
            channels * width,
            QImage.Format_RGB888,
        ).copy()

        pixmap = QPixmap.fromImage(image)
        scaled_pixmap = pixmap.scaled(
            self.camera_feed_label.size(),
            Qt.KeepAspectRatio,
            Qt.FastTransformation,
        )
        self.camera_feed_label.setText("")
        self.camera_feed_label.setPixmap(scaled_pixmap)

    def _set_camera_preview_status(self, message: str):
        """Show a friendly placeholder when live video is not available yet."""
        self.camera_feed_label.setPixmap(QPixmap())
        self.camera_feed_label.setText(message)

    def closeEvent(self, event):
        """Stop detector and serial resources when the UI closes."""
        try:
            self.camera_preview_timer.stop()
        except Exception:
            pass

        try:
            if hasattr(self, 'continuous_executor') and self.continuous_executor is not None:
                self.continuous_executor.disconnect()
        except Exception as close_error:
            print(f"[WARN] Cleanup during closeEvent failed: {close_error}")

        super().closeEvent(event)

    def on_executor_position_update(self, distance_mm: float, progress_percent: float, speed_rpm: float = 0):
        """Callback from continuous executor reporting position updates."""
        status_text = (
            f"📍 Motion Update\n"
            f"Distance: {distance_mm:.1f}mm\n"
            f"Progress: {progress_percent:.1f}%\n"
            f"Speed: {speed_rpm:.1f} RPM\n"
        )
        self.commands_text.setText(status_text)

    def on_executor_waypoint_verified(self, tag_id: int, expected: bool = True):
        """Callback when AprilTag waypoint is verified during motion."""
        status = "✓ Verified" if expected else "⚠ Unexpected"
        self.commands_text.setText(
            f"📌 Waypoint Detected\n"
            f"Tag ID: {tag_id}\n"
            f"Status: {status}\n"
        )

    def on_executor_complete(self, success: bool = True, message: str = ""):
        """Callback when continuous motion execution completes."""
        if success:
            status_text = f"✅ Execution Complete\n{message}"
            QMessageBox.information(self, "Success", "Path execution completed successfully!")
        else:
            status_text = f"❌ Execution Failed\n{message}"
            QMessageBox.warning(self, "Execution Failed", message)
        
        self.commands_text.setText(status_text)

    def test_arduino_connection(self):
        """Test connection to Arduino."""
        self.commands_text.setText("🔌 Testing Arduino connection...\n")
        QApplication.processEvents()
        
        if self._is_arduino_connected():
            self.commands_text.setText("🔌 Already connected to Arduino\n✓ Serial port is open\n")
            return
        
        try:
            if not self._ensure_arduino_connection(
                "🔌 Testing Arduino connection...\n"
                "Attempting to connect to /dev/ttyUSB0 @115200...\n"
            ):
                return
            
            self.commands_text.setText(
                "✅ Connected to Arduino!\n\n"
                "Status:\n"
                "• Port: /dev/ttyUSB0\n"
                "• Baud rate: 115200\n"
                "• Ready for execution\n"
                "• Uncheck Simulation Mode before starting the real mission\n"
            )
            QMessageBox.information(self, "Success", "Successfully connected to Arduino!")
            
        except Exception as e:
            error_text = f"❌ Error: {str(e)}\n"
            self.commands_text.setText(error_text)
            QMessageBox.critical(self, "Connection Error", error_text)

    def on_simulation_mode_changed(self, state):
        """Handle simulation mode toggle."""
        self.simulation_mode = (state == Qt.Checked)
        mode_text = "ON (No Hardware)" if self.simulation_mode else "OFF (Real Hardware)"
        self.commands_text.setText(f"🎮 Simulation Mode: {mode_text}\n\nIn simulation mode, the app will animate robot movement\non the grid without requiring Arduino hardware.")

    def simulate_path_execution(self, path_coords: List[Tuple[int, int]]):
        """Simulate path execution by animating robot movement on grid."""
        if not path_coords or len(path_coords) < 1:
            return
        
        print(f"[Simulator] Animating path: {path_coords}")
        
        # Calculate animation parameters
        num_cells = len(path_coords) - 1
        distance_mm = num_cells * 500
        animation_time_sec = 14.0  # Simulate ~14 seconds for full motion
        steps = max(len(path_coords), 10)
        delay_per_step = (animation_time_sec * 1000) // steps  # milliseconds
        
        # Update status
        self.commands_text.setText(
            f"🎮 Simulated Execution\n"
            f"Distance: {distance_mm}mm ({num_cells} cells)\n"
            f"Status: Starting animation...\n"
        )
        
        # Animate along path
        for i, coord in enumerate(path_coords):
            progress = int(100 * i / len(path_coords))
            self.canvas.show_robot_at(coord)
            self.commands_text.setText(
                f"🎮 Simulated Execution\n"
                f"Distance: {distance_mm}mm ({num_cells} cells)\n"
                f"Progress: {progress}%\n"
                f"Position: ({coord[0]}, {coord[1]})\n"
            )
            QApplication.processEvents()  # Update UI
            time.sleep(delay_per_step / 1000.0)  # Delay in seconds
        
        # Completion
        self.commands_text.setText(
            f"✅ Simulation Complete\n"
            f"Distance: {distance_mm}mm\n"
            f"Status: Path animation finished successfully!"
        )
        QMessageBox.information(self, "Success", "Simulated path execution completed!")

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)

        left_panel = self.create_left_panel()
        right_panel = self.create_right_panel()

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        main_layout.addWidget(splitter)

    def create_left_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        title_label = QLabel("🗺️ AGV Map")
        title_font = QFont()
        title_font.setPointSize(13)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("color: #2c3e50;")
        layout.addWidget(title_label)

        layout_dir = os.path.dirname(os.path.abspath(__file__))
        layout_file = os.path.join(layout_dir, "apriltag_layout.json")
        tag_folder = os.path.join(layout_dir, "apriltag_tags")

        views_splitter = QSplitter(Qt.Horizontal)

        apriltag_group = QGroupBox("AprilTag View")
        apriltag_layout = QVBoxLayout(apriltag_group)
        apriltag_layout.setContentsMargins(8, 8, 8, 8)
        self.april_view = ApriltagView(layout_file, tag_folder, self)
        self.april_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.april_view.tag_clicked.connect(self.on_tag_clicked)
        apriltag_layout.addWidget(self.april_view)

        grid_group = QGroupBox("Grid View")
        grid_layout = QVBoxLayout(grid_group)
        grid_layout.setContentsMargins(8, 8, 8, 8)
        self.canvas = GridView(ROWS, COLS, CELL_SIZE, self)
        self.canvas.cellClicked.connect(self.on_cell_clicked)
        self.canvas.robotMoved.connect(self.on_robot_moved)  # Connect signal
        self.canvas.robotHeadingChanged.connect(self.on_robot_heading_changed)
        grid_layout.addWidget(self.canvas)

        views_splitter.addWidget(apriltag_group)
        views_splitter.addWidget(grid_group)
        views_splitter.setStretchFactor(0, 1)
        views_splitter.setStretchFactor(1, 1)

        commands_group = QGroupBox("Robot Commands")
        commands_layout = QVBoxLayout(commands_group)
        commands_layout.setContentsMargins(8, 8, 8, 8)

        self.commands_text = QTextEdit()
        self.commands_text.setReadOnly(True)
        self.commands_text.setMinimumHeight(220)
        self.commands_text.setPlainText("Commands will appear here when a path is executed...")
        self.commands_text.setStyleSheet("""
            QTextEdit {
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                background-color: #f8f9fa;
                color: #2c3e50;
                font-family: 'Courier New';
            }
        """)
        commands_layout.addWidget(self.commands_text)

        camera_group = QGroupBox("Live Camera Feed")
        camera_layout = QVBoxLayout(camera_group)
        camera_layout.setContentsMargins(8, 8, 8, 8)

        self.camera_feed_label = QLabel("Starting AprilTag camera preview...")
        self.camera_feed_label.setAlignment(Qt.AlignCenter)
        self.camera_feed_label.setMinimumSize(420, 260)
        self.camera_feed_label.setStyleSheet("""
            QLabel {
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                background-color: #17202a;
                color: #d6dbdf;
                padding: 8px;
            }
        """)
        camera_layout.addWidget(self.camera_feed_label)

        lower_splitter = QSplitter(Qt.Horizontal)
        lower_splitter.setChildrenCollapsible(False)
        lower_splitter.addWidget(camera_group)
        lower_splitter.addWidget(commands_group)
        lower_splitter.setStretchFactor(0, 2)
        lower_splitter.setStretchFactor(1, 1)

        vertical_splitter = QSplitter(Qt.Vertical)
        vertical_splitter.setChildrenCollapsible(False)
        vertical_splitter.addWidget(views_splitter)
        vertical_splitter.addWidget(lower_splitter)
        vertical_splitter.setStretchFactor(0, 5)
        vertical_splitter.setStretchFactor(1, 2)
        vertical_splitter.setSizes([720, 320])

        layout.addWidget(vertical_splitter, 1)

        panel.setLayout(layout)
        return panel

    def create_right_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        # Node Selection Group
        node_group = QGroupBox("🔍 Node Selection")
        node_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 11px;
                border: 2px solid #3b7ddd;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                color: #2c3e50;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
            }
        """)
        node_layout = QVBoxLayout(node_group)
        node_layout.setSpacing(8)

        start_label = QLabel("Start Node")
        start_label.setStyleSheet("font-weight: bold; color: #2c3e50;")
        self.start_input = QLineEdit()
        self.start_input.setReadOnly(True)
        self.start_input.setPlaceholderText("Click grid to select START")
        self.start_input.setStyleSheet("QLineEdit { padding: 6px; border: 1px solid #bdc3c7; border-radius: 3px; background-color: #ecf0f1; }")
        node_layout.addWidget(start_label)
        node_layout.addWidget(self.start_input)

        goal_label = QLabel("Goal Node")
        goal_label.setStyleSheet("font-weight: bold; color: #2c3e50;")
        self.goal_input = QLineEdit()
        self.goal_input.setReadOnly(True)
        self.goal_input.setPlaceholderText("Click grid to select GOAL")
        self.goal_input.setStyleSheet("QLineEdit { padding: 6px; border: 1px solid #bdc3c7; border-radius: 3px; background-color: #ecf0f1; }")
        node_layout.addWidget(goal_label)
        node_layout.addWidget(self.goal_input)

        layout.addWidget(node_group)

        # Action Buttons Row
        action_row = QHBoxLayout()
        action_row.setSpacing(6)

        button_style_template = "QPushButton {{ background-color: {color}; color: white; font-weight: bold; padding: 8px; border: none; border-radius: 4px; }} QPushButton:hover {{ background-color: {hover_color}; }} QPushButton:pressed {{ background-color: {pressed_color}; }}"
        
        # self.compute_btn = QPushButton("🔍 Compute")
        # self.compute_btn.setStyleSheet(button_style_template.format(color="#3b7ddd", hover_color="#2e66c4", pressed_color="#264fab"))
        # self.compute_btn.clicked.connect(self.compute_paths)
        # action_row.addWidget(self.compute_btn)

        self.start_mission_btn = QPushButton("Start Mission")
        self.start_mission_btn.setStyleSheet(button_style_template.format(
            color="#2ecc71", hover_color="#27ae60", pressed_color="#229954"
        ))
        self.start_mission_btn.clicked.connect(self.start_mission)
        action_row.addWidget(self.start_mission_btn)

        self.stop_mission_btn = QPushButton("Stop Mission")
        self.stop_mission_btn.setStyleSheet(button_style_template.format(
            color="#e67e22", hover_color="#d35400", pressed_color="#ba4a00"
        ))
        self.stop_mission_btn.clicked.connect(self.stop_mission)
        action_row.addWidget(self.stop_mission_btn)

        self.block_btn = QPushButton("⛔ Block")
        self.block_btn.setStyleSheet(button_style_template.format(color="#d9534f", hover_color="#c9302c", pressed_color="#ac2925"))
        self.block_btn.clicked.connect(self.block_selected_path)
        action_row.addWidget(self.block_btn)
        
        self.test_connection_btn = QPushButton("🔌 Test Arduino")
        self.test_connection_btn.setStyleSheet(button_style_template.format(
            color="#8e44ad", hover_color="#7d3c98", pressed_color="#6c2d7c"
        ))
        self.test_connection_btn.clicked.connect(self.test_arduino_connection)
        action_row.addWidget(self.test_connection_btn)

        # self.replan_btn = QPushButton("Replan")
        # self.replan_btn.setStyleSheet(button_style_template.format(color="#f0ad4e", hover_color="#ec971f", pressed_color="#d58512"))
        # self.replan_btn.clicked.connect(self.replan_paths)
        # action_row.addWidget(self.replan_btn)

        layout.addLayout(action_row)

        # Simulation Mode Toggle
        mode_layout = QHBoxLayout()
        mode_label = QLabel("🎮 Simulation Mode:")
        mode_label.setStyleSheet("font-weight: bold; color: #2c3e50; font-size: 10px;")
        
        self.simulation_mode_checkbox = QCheckBox("Enabled (No Hardware)")
        self.simulation_mode_checkbox.setChecked(True)
        self.simulation_mode_checkbox.stateChanged.connect(self.on_simulation_mode_changed)
        self.simulation_mode_checkbox.setStyleSheet("""
            QCheckBox {
                color: #2c3e50;
                font-weight: bold;
            }
        """)
        
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.simulation_mode_checkbox)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)
        self.toolbox = QToolBox()
        self.toolbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.toolbox.setStyleSheet("""
            QToolBox::tab {
                background-color: #34495e;
                color: white;
                padding: 6px;
                border-radius: 3px;
                font-weight: bold;
            }
            QToolBox::tab:selected {
                background-color: #2c3e50;
            }
        """)

        paths_page = QWidget()
        paths_layout = QVBoxLayout(paths_page)

        self.paths_tree = QTreeWidget()
        self.paths_tree.setHeaderLabels(["Path", "Length", "Cost"])
        self.paths_tree.setRootIsDecorated(False)
        self.paths_tree.setAlternatingRowColors(True)
        self.paths_tree.setStyleSheet("""
            QTreeWidget {
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                background-color: #ecf0f1;
            }
            QTreeWidget::item:selected {
                background-color: #3b7ddd;
                color: white;
            }
        """)
        self.paths_tree.itemSelectionChanged.connect(self.on_path_selected)
        self.paths_tree.itemChanged.connect(self.on_path_item_changed)
        paths_layout.addWidget(self.paths_tree)
        paths_page.setLayout(paths_layout)

        self.toolbox.addItem(paths_page, "📊 Available Paths")

        layout.addWidget(self.toolbox, 1)

        # AGV Status Group - Always Visible
        status_group = QGroupBox("AGV Status")
        status_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 11px;
                border: 2px solid #2e9f64;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                color: #2c3e50;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
            }
        """)
        status_layout = QVBoxLayout(status_group)
        status_layout.setSpacing(8)

        state_label = QLabel("State")
        state_label.setStyleSheet("font-weight: bold; color: #2c3e50; font-size: 10px;")
        self.agv_state = QLineEdit()
        self.agv_state.setReadOnly(True)
        self.agv_state.setText("IDLE")
        self.agv_state.setStyleSheet("QLineEdit { padding: 6px; border: 1px solid #2e9f64; border-radius: 3px; background-color: #d5f4e6; color: #27ae60; font-weight: bold; }")
        status_layout.addWidget(state_label)
        status_layout.addWidget(self.agv_state)

        current_label = QLabel("Current Position")
        current_label.setStyleSheet("font-weight: bold; color: #2c3e50; font-size: 10px;")
        self.current_node = QLineEdit()
        self.current_node.setReadOnly(True)
        self.current_node.setText("---")
        self.current_node.setStyleSheet("QLineEdit { padding: 6px; border: 1px solid #3b7ddd; border-radius: 3px; background-color: #ebf5fb; color: #2980b9; }")
        status_layout.addWidget(current_label)
        status_layout.addWidget(self.current_node)

        target_label = QLabel("Target Position")
        target_label.setStyleSheet("font-weight: bold; color: #2c3e50; font-size: 10px;")
        self.target_node = QLineEdit()
        self.target_node.setReadOnly(True)
        self.target_node.setText("---")
        self.target_node.setStyleSheet("QLineEdit { padding: 6px; border: 1px solid #f39c12; border-radius: 3px; background-color: #fef5e7; color: #d68910; }")
        status_layout.addWidget(target_label)
        status_layout.addWidget(self.target_node)

        heading_label = QLabel("Heading")
        heading_label.setStyleSheet("font-weight: bold; color: #2c3e50; font-size: 10px;")
        self.heading = QLineEdit()
        self.heading.setReadOnly(True)
        self.heading.setText("---")
        self.heading.setStyleSheet("QLineEdit { padding: 6px; border: 1px solid #8e44ad; border-radius: 3px; background-color: #f4ecf7; color: #7d3c98; }")
        status_layout.addWidget(heading_label)
        status_layout.addWidget(self.heading)

        phase_label = QLabel("Mission Phase")
        phase_label.setStyleSheet("font-weight: bold; color: #2c3e50; font-size: 10px;")
        self.mission_phase = QLineEdit()
        self.mission_phase.setReadOnly(True)
        self.mission_phase.setText("IDLE")
        self.mission_phase.setStyleSheet("QLineEdit { padding: 6px; border: 1px solid #7f8c8d; border-radius: 3px; background-color: #f2f3f4; color: #2c3e50; }")
        status_layout.addWidget(phase_label)
        status_layout.addWidget(self.mission_phase)

        layout.addWidget(status_group)
        layout.addStretch()

        panel.setLayout(layout)
        return panel

    def on_cell_clicked(self, row, col):
        try:
            print(f"[DEBUG] Grid cell clicked: ({row}, {col})")
            self._handle_coord_selected((row, col))
        except Exception as e:
            print(f"[ERROR] Grid click failed: {e}")
            import traceback
            traceback.print_exc()
            self.show_error_dialog(f"Grid click error: {str(e)}")

    def on_tag_clicked(self, tag_id):
        try:
            print(f"[DEBUG] Tag clicked: {tag_id}")
            coord = node_id_to_grid_coord(tag_id, COLS)
            self._handle_coord_selected(coord, node_id=tag_id)
        except Exception as e:
            print(f"[ERROR] Tag click failed: {e}")
            import traceback
            traceback.print_exc()
            self.show_error_dialog(f"Tag click error: {str(e)}")

    def _handle_coord_selected(self, coord, node_id=None):
        try:
            row, col = coord
            if node_id is None:
                node_id = grid_coord_to_node_id(row, col, COLS)
            else:
                node_id = int(node_id)

            if self.block_mode:
                self.block_mode = False
                if coord != self.start_coord and coord != self.goal_coord:
                    if coord in self.blocked_nodes:
                        self.blocked_nodes.remove(coord)
                    else:
                        self.blocked_nodes.add(coord)
                    self.replan_paths()
                self.canvas.set_overlays(self.start_coord, self.goal_coord, self.blocked_nodes)
                self.robot_state.set_status("IDLE")
                return

            if self.start_coord is None:
                if coord in self.blocked_nodes:
                    self.blocked_nodes.remove(coord)
                self.start_coord = coord
                self.start_input.setText(f"Node {node_id} ({row}, {col})")
                self.canvas.robot_heading = None
                self.canvas.robotHeadingChanged.emit(None)
                self.target_node.setText("---")
            elif self.goal_coord is None:
                if coord == self.start_coord:
                    return
                if coord in self.blocked_nodes:
                    self.blocked_nodes.remove(coord)
                self.goal_coord = coord
                self.goal_input.setText(f"Node {node_id} ({row}, {col})")
                self.target_node.setText(f"Node {node_id} ({row}, {col})")
                print(f"[DEBUG] Computing paths from {self.start_coord} to {self.goal_coord}")
                self.compute_paths()
            else:
                if coord in self.blocked_nodes:
                    self.blocked_nodes.remove(coord)
                self.start_coord = coord
                self.goal_coord = None
                self.start_input.setText(f"Node {node_id} ({row}, {col})")
                self.goal_input.clear()
                self.target_node.setText("---")
                self.canvas.robot_heading = None
                self.canvas.robotHeadingChanged.emit(None)
                self.clear_paths()

            self.canvas.set_overlays(self.start_coord, self.goal_coord, self.blocked_nodes)
        except Exception as e:
            print(f"[ERROR] Coordinate selection error: {e}")
            import traceback
            traceback.print_exc()
            raise

    def on_robot_moved(self, row, col):
        """Update current position display when robot moves during animation."""
        node_id = grid_coord_to_node_id(row, col, COLS)
        self.robot_state.update_node(node_id)

    def on_robot_heading_changed(self, heading):
        if heading is None:
            self.heading.setText("---")
            return
        if hasattr(self, "continuous_executor") and self.continuous_executor is not None:
            self.continuous_executor.set_heading(heading)
        label = {
            NORTH: "NORTH",
            EAST: "EAST",
            SOUTH: "SOUTH",
            WEST: "WEST",
        }.get(heading, "---")
        self.heading.setText(label)

    def rebuild_graph(self):
        if self.start_coord in self.blocked_nodes:
            self.blocked_nodes.remove(self.start_coord)
        if self.goal_coord in self.blocked_nodes:
            self.blocked_nodes.remove(self.goal_coord)
        self.G = create_networkx_grid(rows=ROWS, cols=COLS, obstacles=self.blocked_nodes)

    def compute_paths(self):
        try:
            if self.start_coord is None or self.goal_coord is None:
                return

            print(f"[DEBUG] compute_paths: start={self.start_coord}, goal={self.goal_coord}")
            self.rebuild_graph()
            path_coords_list = k_shortest_paths(
                self.G,
                self.start_coord,
                self.goal_coord,
                k=NUM_PATHS,
            )

            self.found_paths.clear()
            self.path_visible.clear()
            self.path_distances.clear()
            self.paths_tree.blockSignals(True)
            self.paths_tree.clear()
            self.selected_path_index = -1

            if not path_coords_list:
                print("[DEBUG] No paths found")
                self.paths_tree.blockSignals(False)
                self.canvas.set_paths([], selected_index=-1, visible=[])
                return

            print(f"[DEBUG] Found {len(path_coords_list)} paths")
            for i, path_coords in enumerate(path_coords_list):
                path_node_ids = [grid_coord_to_node_id(r, c, COLS) for r, c in path_coords]
                path_length = len(path_coords)
                cost = get_path_length(self.G, path_coords)
                
                # Calculate continuous motion distance: (num_cells) * 500mm per cell
                num_cells = path_length - 1
                distance_mm = num_cells * 500
                
                self.found_paths.append((path_coords, path_node_ids, distance_mm, cost))
                self.path_distances.append(distance_mm)
                self.path_visible.append(True)

                item = QTreeWidgetItem()
                item.setText(0, f"Path {i + 1}")
                item.setText(1, f"{path_length} cells")
                item.setText(2, f"{distance_mm}mm")  # Show distance in mm
                item.setData(0, Qt.UserRole, i)
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable | Qt.ItemIsSelectable)
                item.setCheckState(0, Qt.Checked)

                color = QColor(PATH_COLORS[i % len(PATH_COLORS)])
                pixmap = QPixmap(16, 16)
                pixmap.fill(color)
                item.setIcon(0, QIcon(pixmap))

                self.paths_tree.addTopLevelItem(item)

            self.paths_tree.blockSignals(False)
            self.paths_tree.resizeColumnToContents(0)
            print(f"[DEBUG] Drawing {len(self.found_paths)} paths on canvas")
            self.canvas.set_paths([p[0] for p in self.found_paths])

            if self.paths_tree.topLevelItemCount() > 0:
                first_item = self.paths_tree.topLevelItem(0)
                self.paths_tree.clearSelection()
                self.paths_tree.setCurrentItem(first_item)
                first_item.setSelected(True)
                self.selected_path_index = 0
                self.canvas.update_path_styles(
                    [p[0] for p in self.found_paths],
                    self.selected_path_index,
                    self.path_visible,
                )

            if self.goal_coord is not None:
                goal_id = grid_coord_to_node_id(self.goal_coord[0], self.goal_coord[1], COLS)
                self.target_node.setText(f"Node {goal_id} ({self.goal_coord[0]}, {self.goal_coord[1]})")
            
            print("[DEBUG] compute_paths completed successfully")
        except Exception as e:
            print(f"[ERROR] compute_paths failed: {e}")
            import traceback
            traceback.print_exc()
            self.show_error_dialog(f"Path computation error: {str(e)}")

    def compute_single_path(self, start_coord, goal_coord):
        self.rebuild_graph()
        paths = k_shortest_paths(self.G, start_coord, goal_coord, k=1)
        if not paths:
            return []
        return paths[0]

    def get_selected_path_coords(self):
        if self.selected_path_index < 0 or self.selected_path_index >= len(self.found_paths):
            return None
        return self.found_paths[self.selected_path_index][0]

    def clear_paths(self):
        self.found_paths.clear()
        self.path_visible.clear()
        self.paths_tree.blockSignals(True)
        self.paths_tree.clear()
        self.paths_tree.blockSignals(False)
        self.selected_path_index = -1
        self.canvas.set_paths([], selected_index=-1, visible=[])

    def _activate_path_item(self, item):
        """Make one path the single active execution path in the UI."""
        idx = item.data(0, Qt.UserRole)
        if idx is None:
            return
        idx = int(idx)
        if idx < 0 or idx >= len(self.found_paths):
            return

        self.paths_tree.blockSignals(True)
        for row in range(self.paths_tree.topLevelItemCount()):
            row_item = self.paths_tree.topLevelItem(row)
            row_idx = row_item.data(0, Qt.UserRole)
            if row_idx is None:
                continue
            row_idx = int(row_idx)
            is_active = row_idx == idx
            row_item.setCheckState(0, Qt.Checked if is_active else Qt.Unchecked)
            if row_idx < len(self.path_visible):
                self.path_visible[row_idx] = is_active
        self.paths_tree.blockSignals(False)

        self.paths_tree.setCurrentItem(item)
        item.setSelected(True)
        self.selected_path_index = idx
        self.canvas.update_path_styles([p[0] for p in self.found_paths], self.selected_path_index, self.path_visible)

    def on_path_selected(self):
        items = self.paths_tree.selectedItems()
        if not items:
            self.selected_path_index = -1
            self.canvas.update_path_styles([p[0] for p in self.found_paths], -1, self.path_visible)
            return
        self._activate_path_item(items[0])

    def on_path_item_changed(self, item, column):
        if column != 0:
            return
        idx = item.data(0, Qt.UserRole)
        if idx is None:
            return
        idx = int(idx)
        if idx < 0 or idx >= len(self.path_visible):
            return

        if item.checkState(0) == Qt.Checked:
            self._activate_path_item(item)
            return

        if idx < len(self.path_visible):
            self.path_visible[idx] = False
        if self.selected_path_index == idx:
            self.selected_path_index = -1
        self.canvas.update_path_styles([p[0] for p in self.found_paths], self.selected_path_index, self.path_visible)

    def execute_selected_path(self):
        if self.selected_path_index < 0 or self.selected_path_index >= len(self.found_paths):
            return

        path_coords, path_node_ids, path_len, _ = self.found_paths[self.selected_path_index]

        self.commands_text.setPlainText(f"Executing path with {path_len} waypoints...")
        self.robot_state.set_status("MOVING")
        self.robot_state.update_node(grid_coord_to_node_id(*path_coords[0], COLS))
        self.target_node.setText(f"Node {grid_coord_to_node_id(*path_coords[-1], COLS)}")

        def on_done():
            self.robot_state.set_status("IDLE")

        self.canvas.animate_path(path_coords, callback=on_done)

    def block_selected_path(self):
        self.block_mode = True
        self.robot_state.set_status("BLOCK MODE: Click a node")

    def replan_paths(self):
        if self.start_coord is None or self.goal_coord is None:
            return
        self.compute_paths()
        self.canvas.set_overlays(self.start_coord, self.goal_coord, self.blocked_nodes)

    def start_mission(self):
        if self.start_coord is None or self.goal_coord is None:
            QMessageBox.warning(self, "No Mission", "Select pickup and drop nodes first")
            return

        if self.simulation_mode:
            self.commands_text.setText(
                "🎮 Simulation Mode is enabled.\n\n"
                "Uncheck 'Simulation Mode' to run the mission on Arduino.\n"
                "You can use 'Test Arduino' first to verify /dev/ttyUSB0.\n"
            )
            QMessageBox.information(
                self,
                "Simulation Mode Enabled",
                "Uncheck 'Simulation Mode' before starting a real Arduino mission."
            )
            return

        pickup_id = grid_coord_to_node_id(self.start_coord[0], self.start_coord[1], COLS)
        drop_id = grid_coord_to_node_id(self.goal_coord[0], self.goal_coord[1], COLS)

        if not self._ensure_arduino_connection(
            f"🚀 Starting Mission\n"
            f"Current node: {self.robot_state.current_node}\n"
            f"Pickup node: {pickup_id}\n"
            f"Drop node: {drop_id}\n"
            f"Status: Connecting to Arduino...\n"
        ):
            return

        self.mission_controller.start_mission(self.start_coord, self.goal_coord)

    def stop_mission(self):
        self.mission_controller.stop_mission()




def main():
    app = QApplication(sys.argv)
    window = AGVPathPlannerApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
