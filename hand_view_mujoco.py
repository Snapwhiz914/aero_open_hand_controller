"""
hand_view_mujoco.py — 3D MuJoCo hand visualization widget for PyQt6.

Loads the MuJoCo XML model (left_hand.xml / right_hand.xml) with STL meshes
and renders an offscreen isometric view that updates with servo positions.
Falls back to the 2D HandGraphic in gui.py when mujoco is not installed.
"""

import os

import mujoco
import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel


# Mapping from actuator name keywords to (servo_index, inverted).
# Tendon actuators are inverted: pos=0 (extended) → ctrl_max, pos=65535 (grasped) → ctrl_min.
# The thumb abduction joint is direct: pos=0 → ctrl_min, pos=65535 → ctrl_max.
_ACTUATOR_KEYWORDS = {
    "index":   (3, True),
    "middle":  (4, True),
    "ring":    (5, True),
    "pinky":   (6, True),
    "cmc_abd": (0, False),
    "th1":     (1, True),
    "th2":     (2, True),
}


class MuJoCoHandWidget(QWidget):
    """Renders the MuJoCo hand model offscreen and displays it in a QLabel."""

    def __init__(self, hand_type: str = "left", parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 340)

        # Resolve model path
        model_dir = os.path.join(
            os.path.dirname(__file__), "aero-open-sim", "mujoco")
        xml_path = os.path.join(model_dir, f"{hand_type}_hand.xml")

        self._model = mujoco.MjModel.from_xml_path(xml_path)
        self._data = mujoco.MjData(self._model)

        self._width = 640
        self._height = 480
        self._renderer = mujoco.Renderer(self._model, self._height, self._width)

        # Build servo-to-actuator mapping: list of (ctrl_idx, servo_idx, inverted, ctrl_min, ctrl_max)
        self._mapping: list[tuple[int, int, bool, float, float]] = []
        for i in range(self._model.nu):
            name = self._model.actuator(i).name
            for keyword, (servo_idx, inverted) in _ACTUATOR_KEYWORDS.items():
                if keyword in name:
                    ctrl_min = self._model.actuator_ctrlrange[i, 0]
                    ctrl_max = self._model.actuator_ctrlrange[i, 1]
                    self._mapping.append((i, servo_idx, inverted, ctrl_min, ctrl_max))
                    break

        # Settle the model at its rest pose so the initial render is valid
        for _ in range(2000):
            mujoco.mj_step(self._model, self._data)

        # Camera — palm facing the viewer, wrist at bottom, fingers at top.
        # The hand extends along +X (wrist→fingers), Z is the palm normal.
        self._camera = mujoco.MjvCamera()
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self._camera.lookat[:] = [0.08, 0.0, -0.04]
        self._camera.distance = 0.30
        self._camera.azimuth = 0
        self._camera.elevation = -90

        # Scene options — hide joint/actuator markers
        self._scene_option = mujoco.MjvOption()
        self._scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
        self._scene_option.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = False
        self._scene_option.frame = mujoco.mjtFrame.mjFRAME_NONE

        # Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._label = QLabel()
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._label)

        # Render initial pose
        self._render()

    def update_positions(self, positions: list[int]):
        """Update the 3D view from 7 normalized servo positions (0-65535)."""
        for ctrl_idx, servo_idx, inverted, ctrl_min, ctrl_max in self._mapping:
            t = positions[servo_idx] / 65535.0
            if inverted:
                self._data.ctrl[ctrl_idx] = ctrl_max - t * (ctrl_max - ctrl_min)
            else:
                self._data.ctrl[ctrl_idx] = ctrl_min + t * (ctrl_max - ctrl_min)

        # Run simulation steps so position actuators drive the joints to target
        for _ in range(2000):
            mujoco.mj_step(self._model, self._data)
        self._render()

    def _render(self):
        """Offscreen render and display in the label."""
        self._renderer.update_scene(
            self._data, self._camera, self._scene_option)
        rgb = self._renderer.render()

        # Convert numpy RGB array to QPixmap
        h, w, _ = rgb.shape
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        # Scale to fit the label while keeping aspect ratio
        label_size = self._label.size()
        if label_size.width() > 0 and label_size.height() > 0:
            pixmap = pixmap.scaled(
                label_size,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation)

        self._label.setPixmap(pixmap)

    def close_renderer(self):
        """Clean up the MuJoCo renderer (frees OpenGL context)."""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
