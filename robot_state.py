import json
import os
from PyQt5.QtCore import QObject, pyqtSignal


class RobotState(QObject):
    node_changed = pyqtSignal(int)
    status_changed = pyqtSignal(str)

    def __init__(self, current_node: int = 1, status: str = "IDLE", active_mission=None, parent=None):
        super().__init__(parent)
        self._state_path = os.path.join(os.path.dirname(__file__), "robot_state.json")
        self._current_node = current_node
        self._status = status
        self._active_mission = active_mission
        self._load_state()

    @property
    def current_node(self) -> int:
        return self._current_node

    @property
    def status(self) -> str:
        return self._status

    @property
    def active_mission(self):
        return self._active_mission

    def update_node(self, new_node: int) -> None:
        if new_node != self._current_node:
            self._current_node = new_node
            self._save_state()
            self.node_changed.emit(new_node)

    def set_position_from_tag(self, tag_id: int) -> None:
        """Compatibility helper: tag IDs map directly to node IDs in this layout."""
        self.update_node(tag_id)

    def set_status(self, new_status: str) -> None:
        if new_status != self._status:
            self._status = new_status
            self.status_changed.emit(new_status)

    def _load_state(self) -> None:
        if not os.path.exists(self._state_path):
            return
        try:
            with open(self._state_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            node = data.get("current_node")
            if isinstance(node, int):
                self._current_node = node
        except (OSError, json.JSONDecodeError):
            return

    def _save_state(self) -> None:
        data = {"current_node": self._current_node}
        try:
            with open(self._state_path, "w", encoding="utf-8") as f:
                json.dump(data, f)
        except OSError:
            return
