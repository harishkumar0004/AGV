import json
import os

from PyQt5.QtCore import Qt, pyqtSignal, QRect
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QPixmap
from PyQt5.QtWidgets import QWidget


class ApriltagView(QWidget):
    tag_clicked = pyqtSignal(int)

    def __init__(self, layout_file, tag_folder, parent=None, tag_size=60, spacing=70):
        super().__init__(parent)
        self.tags = {}  # id -> {"x": int, "y": int, "pixmap": QPixmap, "rect": QRect}
        self.current_robot_node = None
        self.tag_size = int(tag_size)
        self.spacing = int(spacing)
        self.base_width = 0
        self.base_height = 0
        self._scale = 1.0
        self.load_layout(layout_file, tag_folder)

    def load_layout(self, layout_file, tag_folder):
        self.tags.clear()
        if not os.path.isfile(layout_file):
            return

        with open(layout_file, "r", encoding="utf-8") as f:
            entries = json.load(f)

        max_row = -1
        max_col = -1

        for entry in entries:
            try:
                tag_id = int(entry["id"])
                row = int(entry["row"])
                col = int(entry["col"])
            except (KeyError, TypeError, ValueError):
                continue

            x = col * self.spacing
            y = row * self.spacing

            png_path = os.path.join(tag_folder, f"AprilTag-tag36h11-ID{tag_id}.png")
            pixmap = QPixmap(png_path)
            if pixmap.isNull():
                continue
            if self.tag_size > 0:
                pixmap = pixmap.scaled(
                    self.tag_size,
                    self.tag_size,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )

            rect = QRect(x, y, pixmap.width(), pixmap.height())
            self.tags[tag_id] = {"x": x, "y": y, "pixmap": pixmap, "rect": rect}

            max_row = max(max_row, row)
            max_col = max(max_col, col)

        if max_row >= 0 and max_col >= 0:
            self.base_width = max_col * self.spacing + self.tag_size
            self.base_height = max_row * self.spacing + self.tag_size
            self.setMinimumSize(self.base_width, self.base_height)

        self.update()

    def set_robot_node(self, node_id):
        self.current_robot_node = node_id
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        if self.base_width > 0 and self.base_height > 0:
            scale_x = self.width() / self.base_width
            scale_y = self.height() / self.base_height
            self._scale = min(scale_x, scale_y)
            painter.scale(self._scale, self._scale)

        for tag_id, data in self.tags.items():
            painter.drawPixmap(data["x"], data["y"], data["pixmap"])

        if self.current_robot_node in self.tags:
            rect = self.tags[self.current_robot_node]["rect"]
            center = rect.center()
            radius = max(6, int(min(rect.width(), rect.height()) * 0.15))
            painter.setPen(QPen(QColor(180, 40, 40), 2))
            painter.setBrush(QBrush(QColor(220, 60, 60, 160)))
            painter.drawEllipse(center, radius, radius)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self._scale <= 0:
                super().mousePressEvent(event)
                return
            pos = event.pos()
            scaled_pos = pos / self._scale
            for tag_id, data in self.tags.items():
                if data["rect"].contains(scaled_pos):
                    self.tag_clicked.emit(tag_id)
                    return
        super().mousePressEvent(event)
