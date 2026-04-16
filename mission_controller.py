from typing import Optional, Callable

from grid import grid_coord_to_node_id, node_id_to_grid_coord, COLS


class MissionController:
    def __init__(self, robot_state, robot_executor, planner, get_selected_path: Optional[Callable[[], list]] = None):
        self.robot_state = robot_state
        self.robot_executor = robot_executor
        self.planner = planner
        self.get_selected_path = get_selected_path
        self.phase = None
        self.pickup_coord = None
        self.drop_coord = None
        self._external_execution_complete = getattr(self.robot_executor, "on_execution_complete", None)
        self.robot_executor.on_execution_complete = self._on_execution_complete
        self.on_phase_changed = None
        self.on_active_path_changed = None

    def start_mission(self, pickup_coord, drop_coord):
        if self.robot_executor.is_executing:
            self.robot_state.set_status("BUSY")
            return
        self.pickup_coord = pickup_coord
        self.drop_coord = drop_coord
        self.phase = "TO_PICKUP"
        self._emit_phase()
        self._execute_current_phase()

    def _execute_current_phase(self):
        if self.phase == "TO_PICKUP":
            start_coord = node_id_to_grid_coord(self.robot_state.current_node, COLS)
            goal_coord = self.pickup_coord
            path_coords = self.planner(start_coord, goal_coord)
        elif self.phase == "TO_DROP":
            start_coord = node_id_to_grid_coord(self.robot_state.current_node, COLS)
            goal_coord = self.drop_coord
            path_coords = self._get_selected_or_planned_path(start_coord, goal_coord)
        else:
            return

        if start_coord == goal_coord:
            current_node_id = grid_coord_to_node_id(start_coord[0], start_coord[1], COLS)
            self.robot_state.update_node(current_node_id)
            self._on_execution_complete()
            return

        if not path_coords:
            self.robot_state.set_status("ERROR")
            self.phase = "ERROR"
            self._emit_phase()
            return

        path_node_ids = [grid_coord_to_node_id(r, c, COLS) for r, c in path_coords]
        if self.on_active_path_changed:
            self.on_active_path_changed(path_coords)

        if not self._ensure_executor_ready():
            self.robot_state.set_status("ERROR")
            self.phase = "ERROR"
            self._emit_phase()
            return

        self.robot_state.set_status("MOVING")

        # Use new continuous executor interface
        if not self.robot_executor.execute_path_continuous(path_coords, path_node_ids, direction='F'):
            self.robot_state.set_status("ERROR")
            self.phase = "ERROR"
            self._emit_phase()

    def _on_execution_complete(self, *args, **kwargs):
        if self.phase == "TO_PICKUP":
            self.phase = "TO_DROP"
            self._emit_phase()
            self._execute_current_phase()
        elif self.phase == "TO_DROP":
            self.phase = None
            self.robot_state.set_status("IDLE")
            self._emit_phase("IDLE")
            if callable(self._external_execution_complete):
                self._external_execution_complete(*args, **kwargs)

    def _emit_phase(self, override=None):
        if self.on_phase_changed:
            self.on_phase_changed(override if override is not None else self.phase)

    def _get_selected_or_planned_path(self, start_coord, goal_coord):
        if self.get_selected_path:
            selected = self.get_selected_path()
            if selected:
                if selected[0] == start_coord and selected[-1] == goal_coord:
                    return selected
        return self.planner(start_coord, goal_coord)

    def _ensure_executor_ready(self):
        """Connect hardware executor on demand before starting a mission phase."""
        is_connected = getattr(self.robot_executor, "is_connected", None)
        if callable(is_connected) and is_connected():
            return True

        connect = getattr(self.robot_executor, "connect", None)
        if callable(connect):
            self.robot_state.set_status("CONNECTING")
            return bool(connect())

        return True

    def stop_mission(self):
        self.robot_executor.stop_execution()
        self.phase = None
        self.robot_state.set_status("IDLE")
        self._emit_phase("STOPPED")
