#!/usr/bin/env python3
"""
gui.py — PyQt6 GUI for the Aero Open Hand Controller.

Usage:
    python gui.py [--hand left|right]
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import serial.tools.list_ports
from PyQt6.QtCore import Qt, QTimer, QPointF, pyqtSignal, QObject
from PyQt6.QtGui import (
    QPen, QColor, QBrush, QPainter, QPalette, QShortcut
)
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGridLayout, QLabel, QSlider, QPushButton, QRadioButton, QButtonGroup,
    QGroupBox, QTableWidget, QTableWidgetItem, QDialog, QDialogButtonBox,
    QLineEdit, QFormLayout, QComboBox, QMessageBox, QStatusBar,
    QSplitter, QGraphicsScene, QGraphicsView, QHeaderView,
    QSpinBox, QDoubleSpinBox, QCheckBox, QStackedWidget,
    QListWidget, QInputDialog
)

from hand import Hand, MacroRecorder, MacroPlayer, list_macros, delete_macro
from hand.ttl import HandTTL
from hand.sdk import HandESP32
from hand_detector import (
    HandDetector, hand_data_to_positions, LandmarkData, FrameType,
    AVAILABLE_SOURCES, AVAILABLE_DETECTORS, AVAILABLE_CALCULATORS,
    get_detector_class,
)

try:
    from sim.hand_view_mujoco import MuJoCoHandWidget
    _HAS_MUJOCO = True
except ImportError:
    _HAS_MUJOCO = False

_LOG_DIR = os.path.join("runtime", "logs")

NUM_SERVOS = 7
SERVO_NAMES = [
    "Thumb Abd.",
    "Thumb Flex.",
    "Thumb Tend.",
    "Index",
    "Middle",
    "Ring",
    "Pinky",
]


# ─────────────────────────────────────────────────────────────────────────────
# Port / hand-type selection dialog
# ─────────────────────────────────────────────────────────────────────────────

class PortDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connect to Hand")
        self.setMinimumWidth(440)
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("Serial port:"))
        self.combo = QComboBox()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.combo.addItem(
                f"{p.device}  —  {p.description or 'unknown'}", p.device)
        self.combo.addItem("Enter manually…", "__manual__")
        self.combo.addItem("Simulate (no hardware)", "__simulate__")
        layout.addWidget(self.combo)

        self.manual = QLineEdit()
        self.manual.setPlaceholderText("e.g. COM3 or /dev/ttyUSB0")
        self.manual.setVisible(False)
        layout.addWidget(self.manual)

        hand_box = QGroupBox("Hand type")
        hl = QHBoxLayout(hand_box)
        self.rb_left  = QRadioButton("Left");  self.rb_left.setChecked(True)
        self.rb_right = QRadioButton("Right")
        hl.addWidget(self.rb_left)
        hl.addWidget(self.rb_right)
        hl.addStretch()
        layout.addWidget(hand_box)

        proto_box = QGroupBox("Control protocol")
        pl = QHBoxLayout(proto_box)
        self.rb_ttl   = QRadioButton("TTL (direct Feetech serial)")
        self.rb_esp32 = QRadioButton("ESP32 (SDK via ESP32 firmware)")
        self.rb_ttl.setChecked(True)
        pl.addWidget(self.rb_ttl)
        pl.addWidget(self.rb_esp32)
        pl.addStretch()
        layout.addWidget(proto_box)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.combo.currentIndexChanged.connect(
            lambda _: self.manual.setVisible(
                self.combo.currentData() == "__manual__"))

    def port(self) -> "str | None":
        data = self.combo.currentData()
        if data == "__manual__":
            t = self.manual.text().strip()
            return t if t else None
        return data   # returns "__simulate__" or a real port string

    def hand_type(self) -> str:
        return "right" if self.rb_right.isChecked() else "left"

    def protocol(self) -> str:
        return "esp32" if self.rb_esp32.isChecked() else "ttl"


# ─────────────────────────────────────────────────────────────────────────────
# PID constants dialog
# ─────────────────────────────────────────────────────────────────────────────

class PIDDialog(QDialog):
    def __init__(self, hand: Hand, parent=None):
        super().__init__(parent)
        self.hand = hand
        self.setWindowTitle("PID Constants")
        self.setMinimumWidth(520)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Edit PID constants for each servo:"))

        form = QFormLayout()
        self._spins: list[tuple] = []
        for i in range(NUM_SERVOS):
            cfg = hand.pid_config.get(i)
            kp = QDoubleSpinBox(); kp.setRange(0, 1000); kp.setDecimals(4)
            kp.setSingleStep(0.1);  kp.setValue(cfg["kp"])
            ki = QDoubleSpinBox(); ki.setRange(0, 1000); ki.setDecimals(4)
            ki.setSingleStep(0.01); ki.setValue(cfg["ki"])
            kd = QDoubleSpinBox(); kd.setRange(0, 1000); kd.setDecimals(4)
            kd.setSingleStep(0.01); kd.setValue(cfg["kd"])
            row_w = QWidget()
            row_l = QHBoxLayout(row_w); row_l.setContentsMargins(0, 0, 0, 0)
            row_l.addWidget(QLabel("Kp:")); row_l.addWidget(kp)
            row_l.addWidget(QLabel("Ki:")); row_l.addWidget(ki)
            row_l.addWidget(QLabel("Kd:")); row_l.addWidget(kd)
            form.addRow(f"Servo {i} — {SERVO_NAMES[i]}:", row_w)
            self._spins.append((kp, ki, kd))
        layout.addLayout(form)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save |
            QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _save(self):
        for i, (kp, ki, kd) in enumerate(self._spins):
            self.hand.update_pid(i, kp.value(), ki.value(), kd.value())
        self.accept()


# ─────────────────────────────────────────────────────────────────────────────
# Trim dialog
# ─────────────────────────────────────────────────────────────────────────────

class TrimDialog(QDialog):
    def __init__(self, hand: Hand, parent=None):
        super().__init__(parent)
        self.hand = hand
        self.setWindowTitle("Trim Servos")
        self.setMinimumWidth(400)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(
            "Adjust the extend position for each servo (±degrees).\n"
            "Changes are applied and saved when you click Apply."))

        form = QFormLayout()
        self._spins: list[QDoubleSpinBox] = []
        for i in range(NUM_SERVOS):
            ext = hand._sd[i].extend_count
            sb = QDoubleSpinBox()
            sb.setRange(-360, 360)
            sb.setDecimals(2)
            sb.setSingleStep(0.5)
            sb.setValue(0.0)
            form.addRow(f"Servo {i} — {SERVO_NAMES[i]}  (ext={ext}):", sb)
            self._spins.append(sb)
        layout.addLayout(form)

        btns = QDialogButtonBox()
        apply_btn = btns.addButton(
            "Apply & Close", QDialogButtonBox.ButtonRole.AcceptRole)
        cancel_btn = btns.addButton(QDialogButtonBox.StandardButton.Cancel)
        apply_btn.clicked.connect(self._apply)
        cancel_btn.clicked.connect(self.reject)
        layout.addWidget(btns)

    def _apply(self):
        for i, sb in enumerate(self._spins):
            if sb.value() != 0.0:
                self.hand.trim(i, sb.value())
        self.accept()


# ─────────────────────────────────────────────────────────────────────────────
# Grasp parameters dialog
# ─────────────────────────────────────────────────────────────────────────────

class GraspDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Grasp")
        self.setMinimumWidth(320)
        layout = QVBoxLayout(self)

        form = QFormLayout()
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(1.0, 360.0)
        self.speed_spin.setDecimals(1)
        self.speed_spin.setSingleStep(1.0)
        self.speed_spin.setSuffix(" °/s")
        self.speed_spin.setValue(15.0)

        self.torque_spin = QSpinBox()
        self.torque_spin.setRange(0, 1023)
        self.torque_spin.setValue(500)

        form.addRow("Closing speed:", self.speed_spin)
        form.addRow("Torque limit (0–1023):", self.torque_spin)
        layout.addLayout(form)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def speed_dps(self) -> float:  return self.speed_spin.value()
    def torque_limit(self) -> int: return self.torque_spin.value()


# ─────────────────────────────────────────────────────────────────────────────
# Set Servo ID dialog
# ─────────────────────────────────────────────────────────────────────────────

class SetIDDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Set Servo ID")
        self.setMinimumWidth(360)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(
            "<b>Connect ONLY the target servo before proceeding.</b><br>"
            "The scan will fail if more than one servo responds."))

        form = QFormLayout()
        self.id_spin = QSpinBox()
        self.id_spin.setRange(0, 253)
        self.id_spin.setValue(0)
        self.cl_spin = QSpinBox()
        self.cl_spin.setRange(0, 1023)
        self.cl_spin.setValue(1023)
        form.addRow("New servo ID (0–253, not 254):", self.id_spin)
        form.addRow("Current limit (0–1023):", self.cl_spin)
        layout.addLayout(form)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def new_id(self) -> int:      return self.id_spin.value()
    def current_limit(self) -> int: return self.cl_spin.value()


# ─────────────────────────────────────────────────────────────────────────────
# Macro selection dialog
# ─────────────────────────────────────────────────────────────────────────────

MACRO_DIR = os.path.join(os.path.dirname(__file__), "macros")


class MacroSelectDialog(QDialog):
    """Dialog to pick a macro from the macros/ directory."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Load Macro")
        self.setMinimumSize(480, 320)
        layout = QVBoxLayout(self)

        self._macros = list_macros(MACRO_DIR)
        self._selected_path = None

        self._list = QListWidget()
        for m in self._macros:
            dur = f"{m.duration:.1f}s"
            self._list.addItem(f"{m.name}  —  {dur}  —  {m.recorded}")
        layout.addWidget(self._list)

        btn_row = QHBoxLayout()
        self._btn_delete = QPushButton("Delete")
        self._btn_delete.clicked.connect(self._on_delete)
        btn_row.addWidget(self._btn_delete)
        btn_row.addStretch()

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._on_accept)
        buttons.rejected.connect(self.reject)
        btn_row.addWidget(buttons)
        layout.addLayout(btn_row)

    def _on_accept(self):
        row = self._list.currentRow()
        if 0 <= row < len(self._macros):
            self._selected_path = self._macros[row].path
            self.accept()

    def _on_delete(self):
        row = self._list.currentRow()
        if row < 0 or row >= len(self._macros):
            return
        macro = self._macros[row]
        reply = QMessageBox.question(
            self, "Delete Macro",
            f"Delete '{macro.name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel)
        if reply == QMessageBox.StandardButton.Yes:
            delete_macro(macro.path)
            self._macros.pop(row)
            self._list.takeItem(row)

    def selected_path(self):
        return self._selected_path


# ─────────────────────────────────────────────────────────────────────────────
# Detection pipeline selection dialog
# ─────────────────────────────────────────────────────────────────────────────

class DetectionDialog(QDialog):
    """Dialog for selecting the detection pipeline components.

    Greys out detectors and joint calculators that are incompatible with the
    currently selected source's frame type.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configure Detection Pipeline")
        self.setMinimumWidth(480)
        layout = QVBoxLayout(self)

        # ── Source ───────────────────────────────────────────────────────
        src_box = QGroupBox("Frame source")
        src_layout = QVBoxLayout(src_box)
        self._src_group = QButtonGroup(self)
        self._src_keys: list[str] = []
        for i, (key, info) in enumerate(AVAILABLE_SOURCES.items()):
            rb = QRadioButton(key)
            rb.setToolTip(info["description"])
            if i == 0:
                rb.setChecked(True)
            self._src_group.addButton(rb, i)
            src_layout.addWidget(rb)
            self._src_keys.append(key)
        layout.addWidget(src_box)

        # ── Detector ─────────────────────────────────────────────────────
        det_box = QGroupBox("Hand landmark detector")
        det_layout = QVBoxLayout(det_box)
        self._det_group = QButtonGroup(self)
        self._det_keys: list[str] = []
        self._det_buttons: list[QRadioButton] = []
        for i, (key, info) in enumerate(AVAILABLE_DETECTORS.items()):
            rb = QRadioButton(key)
            rb.setToolTip(info["description"])
            if i == 0:
                rb.setChecked(True)
            self._det_group.addButton(rb, i)
            det_layout.addWidget(rb)
            self._det_keys.append(key)
            self._det_buttons.append(rb)
        layout.addWidget(det_box)

        # ── Joint calculator ─────────────────────────────────────────────
        calc_box = QGroupBox("Joint angle calculator")
        calc_layout = QVBoxLayout(calc_box)
        self._calc_group = QButtonGroup(self)
        self._calc_keys: list[str] = []
        self._calc_buttons: list[QRadioButton] = []
        for i, (key, info) in enumerate(AVAILABLE_CALCULATORS.items()):
            rb = QRadioButton(key)
            rb.setToolTip(info["description"])
            if i == 0:
                rb.setChecked(True)
            self._calc_group.addButton(rb, i)
            calc_layout.addWidget(rb)
            self._calc_keys.append(key)
            self._calc_buttons.append(rb)
        layout.addWidget(calc_box)

        # ── Preview checkbox ─────────────────────────────────────────────
        self._chk_preview = QCheckBox("Show camera preview window")
        self._chk_preview.setToolTip(
            "Open a cv2 window with hand landmark overlay during detection")
        layout.addWidget(self._chk_preview)

        # ── Buttons ──────────────────────────────────────────────────────
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        # ── Wire up compatibility filtering ──────────────────────────────
        self._src_group.idToggled.connect(self._update_compatibility)
        self._det_group.idToggled.connect(self._update_compatibility)
        self._update_compatibility()

    # ── Compatibility logic ──────────────────────────────────────────────

    def _selected_source_frame_type(self):
        idx = self._src_group.checkedId()
        if idx < 0:
            return None
        key = self._src_keys[idx]
        return AVAILABLE_SOURCES[key]["frame_type"]

    def _selected_detector_name(self):
        idx = self._det_group.checkedId()
        if idx < 0:
            return None
        key = self._det_keys[idx]
        return AVAILABLE_DETECTORS[key]["name"]

    def _update_compatibility(self):
        frame_type = self._selected_source_frame_type()
        if frame_type is None:
            return

        # Filter detectors by frame type
        for i, key in enumerate(self._det_keys):
            info = AVAILABLE_DETECTORS[key]
            compatible = frame_type in info["supported_frame_types"]
            self._det_buttons[i].setEnabled(compatible)
            if not compatible and self._det_buttons[i].isChecked():
                self._det_buttons[i].setChecked(False)
                self._select_first_enabled(self._det_buttons, self._det_group)

        # Filter calculators by frame type AND detector
        det_name = self._selected_detector_name()
        for i, key in enumerate(self._calc_keys):
            info = AVAILABLE_CALCULATORS[key]
            ft_ok = frame_type in info["supported_frame_types"]
            det_ok = (
                info["supported_detectors"] is None
                or det_name in info["supported_detectors"]
            )
            compatible = ft_ok and det_ok
            self._calc_buttons[i].setEnabled(compatible)
            if not compatible and self._calc_buttons[i].isChecked():
                self._calc_buttons[i].setChecked(False)
                self._select_first_enabled(self._calc_buttons, self._calc_group)

    @staticmethod
    def _select_first_enabled(buttons, group):
        for btn in buttons:
            if btn.isEnabled():
                btn.setChecked(True)
                return

    # ── Accessors ────────────────────────────────────────────────────────

    def selected_source_key(self) -> str:
        return self._src_keys[self._src_group.checkedId()]

    def selected_detector_key(self) -> str:
        return self._det_keys[self._det_group.checkedId()]

    def selected_calculator_key(self) -> str:
        return self._calc_keys[self._calc_group.checkedId()]

    def show_preview(self) -> bool:
        return self._chk_preview.isChecked()


# ─────────────────────────────────────────────────────────────────────────────
# Hand graphic widget (QGraphicsScene)
# ─────────────────────────────────────────────────────────────────────────────

class HandGraphic(QGraphicsView):
    """
    Schematic line drawing of the hand that updates with servo positions.

    Fingers (servos 3-6): lines whose endpoint moves to show curl.
    Thumb  (servos 0-2):  line whose angle and length change.
    """

    PALM_W    = 110
    PALM_H    = 80
    FIN_LEN   = 100   # max finger line length (fully extended)
    FIN_MIN   = 38    # min finger line length (fully grasped)
    THUMB_LEN = 90    # max thumb line length

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene()
        self.setScene(self._scene)
        self.setMinimumSize(260, 340)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setBackgroundBrush(QBrush(QColor(28, 28, 32)))
        self._scene.setSceneRect(-170, -230, 340, 380)

        self._pen_palm  = QPen(QColor(160, 160, 190), 2)
        self._pen_fing  = QPen(QColor(90, 170, 255), 3)
        self._pen_thumb = QPen(QColor(70, 210, 130), 3)
        self._pen_palm.setCosmetic(False)

        # Draw static palm
        cx, cy = 0, 80
        pw, ph = self.PALM_W, self.PALM_H
        self._scene.addRect(
            cx - pw // 2, cy - ph // 2, pw, ph,
            self._pen_palm, QBrush(QColor(45, 45, 65)))

        # Wrist stub
        wrist_pen = QPen(QColor(130, 130, 160), 2)
        self._scene.addLine(
            -pw // 4, cy + ph // 2,
            -pw // 4, cy + ph // 2 + 30,
            wrist_pen)
        self._scene.addLine(
            pw // 4, cy + ph // 2,
            pw // 4, cy + ph // 2 + 30,
            wrist_pen)

        # Dynamic lines
        self._finger_lines = []
        for i in range(4):
            base = self._finger_base(i)
            line = self._scene.addLine(
                base.x(), base.y(),
                base.x(), base.y() - self.FIN_LEN,
                self._pen_fing)
            self._finger_lines.append(line)

        # Thumb: single line from left side of palm
        thumb_line = self._scene.addLine(
            -pw // 2, cy,
            -pw // 2 - self.THUMB_LEN,
            cy - self.THUMB_LEN // 2,
            self._pen_thumb)
        self._thumb_line = thumb_line

        # Servo labels
        label_pen = QPen(QColor(160, 160, 200), 1)
        for i in range(4):
            base = self._finger_base(i)
            lbl = self._scene.addText(SERVO_NAMES[3 + i][0])  # first letter
            lbl.setDefaultTextColor(QColor(140, 140, 180))
            lbl.setPos(base.x() - 5, base.y() - self.FIN_LEN - 18)

    def _finger_base(self, finger_idx: int) -> QPointF:
        """Base point at top of palm for finger 0-3 (index … pinky)."""
        cx = -self.PALM_W // 2 + (finger_idx + 0.5) * self.PALM_W / 4
        cy = 80 - self.PALM_H // 2
        return QPointF(cx, cy)

    def update_positions(self, positions: list[int]):
        """Redraw lines from normalized 0-65535 servo positions."""
        # Fingers (servos 3-6)
        for i in range(4):
            t    = positions[3 + i] / 65535.0      # 0=extended, 1=grasped
            base = self._finger_base(i)
            length = self.FIN_LEN - t * (self.FIN_LEN - self.FIN_MIN)
            # Angle: 90° (straight up) → ~15° tilted inward when grasped
            angle_deg = 90.0 - t * 72.0
            angle_rad = math.radians(angle_deg)
            ex = base.x() + length * math.cos(angle_rad)
            ey = base.y() - length * math.sin(angle_rad)
            self._finger_lines[i].setLine(base.x(), base.y(), ex, ey)

        # Thumb: combine servos 0 (abduction), 1 (flexion), 2 (tendon)
        t0 = positions[0] / 65535.0  # abduction: spread
        t1 = positions[1] / 65535.0  # flexion: curl
        t2 = positions[2] / 65535.0  # tendon: secondary curl
        spread_angle_deg = 155 + t0 * 25          # 155°–180° from +x axis
        curl = (t1 + t2 * 0.5) / 1.5              # 0-1 combined curl
        length = self.THUMB_LEN - curl * (self.THUMB_LEN - 28)
        angle_rad = math.radians(spread_angle_deg)
        bx = -self.PALM_W // 2
        by = 80
        self._thumb_line.setLine(
            bx, by,
            bx + length * math.cos(angle_rad),
            by - length * math.sin(angle_rad))


# ─────────────────────────────────────────────────────────────────────────────
# Cross-thread signals
# ─────────────────────────────────────────────────────────────────────────────

class _Signals(QObject):
    status        = pyqtSignal(str)
    homing_done   = pyqtSignal(str)   # empty = success, else error text
    manual_prompt = pyqtSignal(str)
    hand_positions  = pyqtSignal(list)  # list[int], 7 values; emitted on detector thread
    detector_error  = pyqtSignal(str)  # fatal error from detector background thread
    grasp_done      = pyqtSignal()     # emitted when grasp movement finishes


# ─────────────────────────────────────────────────────────────────────────────
# Main window
# ─────────────────────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    SENSOR_MS = 200     # sensor table refresh interval

    def __init__(self, hand: Hand, port: str, hand_type: str, serial_logger=None):
        super().__init__()
        self.hand          = hand
        self.port          = port
        self.hand_type     = hand_type
        self._serial_logger = serial_logger

        self._cmd_positions = [32767] * NUM_SERVOS
        self._cmd_torques   = [0]     * NUM_SERVOS
        self._cmd_pid_pos   = [32767] * NUM_SERVOS
        self._block_sliders = False   # suppress feedback when we set values

        self._sig = _Signals()
        self._sig.status.connect(self._set_status)
        self._sig.homing_done.connect(self._on_homing_done)
        self._sig.grasp_done.connect(self._sync_position_sliders)
        self._sig.manual_prompt.connect(self._set_status)
        self._space_shortcut    = None   # QShortcut, live only during manual homing
        self._manual_step_event = None   # threading.Event

        _PARAMS_PATH = os.path.join(os.path.dirname(__file__), "..", "configs", "detector_params.json")
        _PARAM_DEFAULTS = {"mcp_pip_coupling": 0.5, "cmc_abd_scale": 1.0, "cmc_abd_offset": 0.0, "smoothing_alpha": 0.3}

        self._detector: HandDetector | None = None
        self._detector_cb_id: str | None = None
        self._joint_calculator = None
        self._params_path = _PARAMS_PATH

        try:
            with open(_PARAMS_PATH) as f:
                self._detector_params = {**_PARAM_DEFAULTS, **json.load(f)}
        except (FileNotFoundError, json.JSONDecodeError):
            self._detector_params = dict(_PARAM_DEFAULTS)

        self._smoothed_positions: list[float] | None = None

        self._sig.hand_positions.connect(self._on_detector_positions)
        self._sig.detector_error.connect(self._on_detector_error)

        # Macro recording/playback
        self._macro_recorder = MacroRecorder(hand, macro_dir=MACRO_DIR,
                                             hand_type=hand_type)
        self._macro_player = MacroPlayer(hand)
        self._playback_active = False  # True while playback controls own the hand

        self.setWindowTitle(
            f"Aero Open Hand  —  {port}  ({hand_type})")
        self.resize(1060, 720)

        self._build_ui()

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_sensors)
        self._timer.start(self.SENSOR_MS)

        QTimer.singleShot(500, self._sync_position_sliders)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 4)
        root.setSpacing(4)

        root.addWidget(self._make_toolbar())
        root.addWidget(self._make_macro_toolbar())

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._make_graphic_panel())
        splitter.addWidget(self._make_control_panel())
        splitter.setSizes([290, 740])
        root.addWidget(splitter, stretch=3)

        root.addWidget(self._make_sensor_panel(), stretch=1)

        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)
        self._set_status("Ready.")

    def _make_toolbar(self) -> QWidget:
        bar = QWidget()
        hl  = QHBoxLayout(bar)
        hl.setContentsMargins(0, 0, 0, 0)

        for label, slot, tip in [
            ("Automatic Homing", self._cmd_home,        "Run automatic homing sequence"),
            ("Manual Homing",    self._cmd_manual_home, "Step-by-step manual homing (Space to advance)"),
            ("Release",          self._cmd_release,     "Move all servos to extend position"),
            ("Grasp",            self._cmd_grasp,       "Move all servos to grasp position"),
            ("Ping",   self._cmd_ping,   "Ping all 7 servos"),
            ("Trim",   self._cmd_trim,   "Trim servo extend positions"),
            ("PID",    self._cmd_pid,    "Edit per-servo PID constants"),
            ("Set ID", self._cmd_set_id, "Set servo ID (connect only one servo first)"),
        ]:
            b = QPushButton(label)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            hl.addWidget(b)

        self._btn_start_detector = QPushButton("Start Detection")
        self._btn_start_detector.setToolTip("Start hand tracking and mirror pose to servos")
        self._btn_start_detector.clicked.connect(self._cmd_start_detector)

        self._btn_stop_detector = QPushButton("Stop Detection")
        self._btn_stop_detector.setToolTip("Stop hand tracking and re-enable manual control")
        self._btn_stop_detector.clicked.connect(self._cmd_stop_detector)
        self._btn_stop_detector.setEnabled(False)

        hl.addWidget(self._btn_start_detector)
        hl.addWidget(self._btn_stop_detector)

        hl.addStretch()
        info = QLabel(f"<b>{self.port}</b>  —  {self.hand_type} hand")
        hl.addWidget(info)
        return bar

    def _make_macro_toolbar(self) -> QWidget:
        bar = QWidget()
        hl = QHBoxLayout(bar)
        hl.setContentsMargins(0, 0, 0, 0)

        # ── Recording ──
        hl.addWidget(QLabel("<b>Macro:</b>"))

        self._btn_start_record = QPushButton("Start Recording")
        self._btn_start_record.setToolTip("Record hand positions to a macro file")
        self._btn_start_record.clicked.connect(self._cmd_start_record)
        hl.addWidget(self._btn_start_record)

        self._btn_stop_record = QPushButton("Stop Recording")
        self._btn_stop_record.setToolTip("Stop recording and save macro")
        self._btn_stop_record.setEnabled(False)
        self._btn_stop_record.clicked.connect(self._cmd_stop_record)
        hl.addWidget(self._btn_stop_record)

        hl.addWidget(QLabel("  "))  # spacer

        # ── Playback ──
        self._btn_load_macro = QPushButton("Load Macro")
        self._btn_load_macro.setToolTip("Load a macro for playback")
        self._btn_load_macro.clicked.connect(self._cmd_load_macro)
        hl.addWidget(self._btn_load_macro)

        self._btn_play = QPushButton("Play")
        self._btn_play.setToolTip("Start or resume playback")
        self._btn_play.setEnabled(False)
        self._btn_play.clicked.connect(self._cmd_play)
        hl.addWidget(self._btn_play)

        self._btn_pause = QPushButton("Pause")
        self._btn_pause.setToolTip("Pause playback")
        self._btn_pause.setEnabled(False)
        self._btn_pause.clicked.connect(self._cmd_pause)
        hl.addWidget(self._btn_pause)

        self._macro_scrubber = QSlider(Qt.Orientation.Horizontal)
        self._macro_scrubber.setRange(0, 1000)
        self._macro_scrubber.setValue(0)
        self._macro_scrubber.setEnabled(False)
        self._macro_scrubber.setMinimumWidth(120)
        self._macro_scrubber.setMaximumWidth(250)
        self._macro_scrubber.sliderReleased.connect(self._on_scrubber_released)
        hl.addWidget(self._macro_scrubber)

        self._macro_speed = QDoubleSpinBox()
        self._macro_speed.setRange(0.1, 5.0)
        self._macro_speed.setSingleStep(0.1)
        self._macro_speed.setValue(1.0)
        self._macro_speed.setSuffix("x")
        self._macro_speed.setToolTip("Playback speed multiplier")
        self._macro_speed.setMaximumWidth(80)
        self._macro_speed.valueChanged.connect(self._on_speed_changed)
        hl.addWidget(self._macro_speed)

        self._btn_end_playback = QPushButton("End Playback")
        self._btn_end_playback.setToolTip("Stop playback and return to manual control")
        self._btn_end_playback.setEnabled(False)
        self._btn_end_playback.clicked.connect(self._cmd_end_playback)
        hl.addWidget(self._btn_end_playback)

        self._lbl_macro_name = QLabel("")
        hl.addWidget(self._lbl_macro_name)

        hl.addStretch()
        return bar

    def _make_graphic_panel(self) -> QWidget:
        box = QGroupBox("Hand Graphic")
        vl  = QVBoxLayout(box)
        if _HAS_MUJOCO:
            self.hand_graphic = MuJoCoHandWidget(hand_type=self.hand_type)
        else:
            self.hand_graphic = HandGraphic()
        vl.addWidget(self.hand_graphic)
        return box

    def _make_control_panel(self) -> QWidget:
        box = QGroupBox("Servo Control")
        vl  = QVBoxLayout(box)

        # Mode row
        mode_row = QWidget()
        ml = QHBoxLayout(mode_row)
        ml.setContentsMargins(0, 0, 0, 0)
        ml.addWidget(QLabel("Control mode:"))
        self._mode_group = QButtonGroup(self)
        for btn_id, label in enumerate(("Position", "Raw Torque", "PID Torque")):
            rb = QRadioButton(label)
            if btn_id == 0:
                rb.setChecked(True)
            self._mode_group.addButton(rb, btn_id)
            ml.addWidget(rb)
        self._rb_hand_detection = QRadioButton("Hand Detection")
        self._rb_hand_detection.setEnabled(False)   # disabled until detector starts
        self._mode_group.addButton(self._rb_hand_detection, 3)
        ml.addWidget(self._rb_hand_detection)

        ml.addStretch()
        vl.addWidget(mode_row)
        self._mode_group.idToggled.connect(self._on_mode_changed)

        # Slider grid
        grid = QGridLayout()
        grid.setSpacing(4)
        for col, header in enumerate(("#", "Name", "Value")):
            lbl = QLabel(f"<b>{header}</b>")
            grid.addWidget(lbl, 0, col)
        self._slider_col_header = QLabel("<b>Position / Torque</b>")
        grid.addWidget(self._slider_col_header, 0, 3)
        grid.setColumnStretch(3, 1)

        self._sliders: list[QSlider]  = []
        self._val_labels: list[QLabel] = []

        for i in range(NUM_SERVOS):
            row = i + 1
            grid.addWidget(QLabel(str(i)), row, 0)
            grid.addWidget(QLabel(SERVO_NAMES[i]), row, 1)

            vl_w = QLabel("32767")
            vl_w.setMinimumWidth(52)
            vl_w.setAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self._val_labels.append(vl_w)
            grid.addWidget(vl_w, row, 2)

            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(0, 65535)
            sl.setValue(32767)
            sl.setProperty("servo_idx", i)
            sl.valueChanged.connect(self._on_slider)
            self._sliders.append(sl)
            grid.addWidget(sl, row, 3)

        manual_page = QWidget()
        manual_page.setLayout(grid)

        self._detection_page = self._make_detection_page()

        self._mode_stack = QStackedWidget()
        self._mode_stack.addWidget(manual_page)           # index 0 — modes 0/1/2
        self._mode_stack.addWidget(self._detection_page)  # index 1 — mode 3

        vl.addWidget(self._mode_stack)
        vl.addStretch()
        return box

    def _make_detection_page(self) -> QWidget:
        page = QWidget()
        vl = QVBoxLayout(page)

        # Section A — Tuning parameters
        tune_box = QGroupBox("Tuning Parameters")
        tune_grid = QGridLayout(tune_box)
        tune_grid.setSpacing(4)

        self._tune_sliders: dict[str, QSlider] = {}
        self._tune_labels:  dict[str, QLabel]  = {}

        tune_specs = [
            ("mcp_pip_coupling", 0,     200,  "MCP/PIP coupling"),
            ("cmc_abd_scale",    0,     500,  "CMC abd. scale"),
            ("cmc_abd_offset",   -9000, 9000, "CMC abd. offset"),
            ("smoothing_alpha",  1,     100,  "Smoothing alpha"), # Lower alpha = more smoothing (slower response), higher alpha = less smoothing (faster but jittery). Alpha=1.0 means no smoothing (pass-through). recommended 0.2-0.5
        ]
        for row, (key, lo, hi, display_name) in enumerate(tune_specs):
            default_val = int(self._detector_params[key] * 100)
            lbl_name  = QLabel(display_name)
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(lo, hi)
            sl.setValue(default_val)
            lbl_val = QLabel(f"{default_val / 100:.2f}")
            lbl_val.setMinimumWidth(46)
            sl.valueChanged.connect(
                lambda val, lv=lbl_val: lv.setText(f"{val / 100:.2f}"))
            self._tune_sliders[key] = sl
            self._tune_labels[key]  = lbl_val
            tune_grid.addWidget(lbl_name, row, 0)
            tune_grid.addWidget(sl,       row, 1)
            tune_grid.addWidget(lbl_val,  row, 2)

        vl.addWidget(tune_box)

        # Section B — Commanded positions
        pos_box  = QGroupBox("Commanded Positions")
        pos_grid = QGridLayout(pos_box)
        pos_grid.setSpacing(4)

        self._det_pos_sliders: list[QSlider] = []
        self._det_pos_labels:  list[QLabel]  = []

        for i in range(NUM_SERVOS):
            lbl_i    = QLabel(str(i))
            lbl_name = QLabel(SERVO_NAMES[i])
            lbl_val  = QLabel("32767")
            lbl_val.setMinimumWidth(52)
            lbl_val.setAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(0, 65535)
            sl.setValue(32767)
            sl.setEnabled(False)
            self._det_pos_sliders.append(sl)
            self._det_pos_labels.append(lbl_val)
            pos_grid.addWidget(lbl_i,    i, 0)
            pos_grid.addWidget(lbl_name, i, 1)
            pos_grid.addWidget(lbl_val,  i, 2)
            pos_grid.addWidget(sl,       i, 3)

        pos_grid.setColumnStretch(3, 1)
        vl.addWidget(pos_box)

        return page

    def _make_sensor_panel(self) -> QWidget:
        box = QGroupBox("Live Sensors")
        vl  = QVBoxLayout(box)

        self.sensor_table = QTableWidget(NUM_SERVOS, 5)
        self.sensor_table.setHorizontalHeaderLabels(
            ["Servo", "Position", "Velocity", "Temp (°C)", "Current"])
        self.sensor_table.verticalHeader().setVisible(False)
        self.sensor_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers)
        self.sensor_table.setSelectionMode(
            QTableWidget.SelectionMode.NoSelection)
        self.sensor_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.sensor_table.setMaximumHeight(210)

        for i in range(NUM_SERVOS):
            self.sensor_table.setItem(
                i, 0, QTableWidgetItem(f"{i} — {SERVO_NAMES[i]}"))
            for col in range(1, 5):
                item = QTableWidgetItem("—")
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.sensor_table.setItem(i, col, item)

        vl.addWidget(self.sensor_table)
        return box

    # ── Slider sync helper ────────────────────────────────────────────────────

    def _sync_position_sliders(self):
        """Read current servo positions and push them to the position sliders."""
        positions = self.hand.get_positions()
        self._cmd_positions = list(positions)
        if self._mode_group.checkedId() == 0:
            self._block_sliders = True
            for i, sl in enumerate(self._sliders):
                sl.setValue(positions[i])
                self._val_labels[i].setText(str(positions[i]))
            self._block_sliders = False

    # ── Sensor refresh ────────────────────────────────────────────────────────

    def _refresh_sensors(self):
        pos  = self.hand.get_positions()
        vel  = self.hand.get_velocities()
        temp = self.hand.get_temperatures()
        cur  = self.hand.get_currents()

        for i in range(NUM_SERVOS):
            self.sensor_table.item(i, 1).setText(str(pos[i]))
            self.sensor_table.item(i, 2).setText(str(vel[i]))
            t_item = self.sensor_table.item(i, 3)
            t_item.setText(str(temp[i]))
            hot = temp[i] >= 70
            t_item.setForeground(
                QColor(255, 80, 80) if hot else QColor(220, 220, 220))
            self.sensor_table.item(i, 4).setText(str(cur[i]))

        self.hand_graphic.update_positions(pos)

        # Macro status updates
        if self._macro_recorder.is_recording:
            self._set_status(f"Recording: {self._macro_recorder.elapsed:.1f}s")
        if self._playback_active:
            player = self._macro_player
            pct = int(player.progress * 100)
            name = player.metadata.name if player.metadata else "macro"
            if not self._macro_scrubber.isSliderDown():
                self._macro_scrubber.setValue(int(player.progress * 1000))
            if player.is_paused:
                self._set_status(f"Paused: {name} — {pct}%")
            elif player.is_playing:
                self._set_status(f"Playing: {name} — {pct}%")
            else:
                self._set_status(f"Finished: {name} — 100%")

    # ── Mode change ───────────────────────────────────────────────────────────

    def _on_mode_changed(self, btn_id: int, checked: bool):
        if not checked:
            return
        if btn_id == 3:
            self._mode_stack.setCurrentIndex(1)
            return
        self._mode_stack.setCurrentIndex(0)
        self._block_sliders = True
        if btn_id == 1:     # raw torque: -1000..+1000
            self._slider_col_header.setText("<b>← Extend  |  0  |  Grasp →</b>")
            for i, sl in enumerate(self._sliders):
                sl.setRange(-1000, 1000)
                sl.setValue(self._cmd_torques[i])
                self._val_labels[i].setText(str(self._cmd_torques[i]))
        else:               # position or PID: 0-65535
            self._slider_col_header.setText("<b>Position / Torque</b>")
            src = self._cmd_positions if btn_id == 0 else self._cmd_pid_pos
            for i, sl in enumerate(self._sliders):
                sl.setRange(0, 65535)
                sl.setValue(src[i])
                self._val_labels[i].setText(str(src[i]))
        self._block_sliders = False

    # ── Slider ────────────────────────────────────────────────────────────────

    def _on_slider(self, value: int):
        if self._block_sliders or self._playback_active:
            return
        idx  = self.sender().property("servo_idx")
        self._val_labels[idx].setText(str(value))
        mode = self._mode_group.checkedId()

        try:
            if mode == 0:   # position
                self._cmd_positions[idx] = value
                self.hand.set_positions(list(self._cmd_positions))
            elif mode == 1: # raw torque
                self._cmd_torques[idx] = value
                self.hand.set_torques(list(self._cmd_torques))
            else:           # PID torque
                self._cmd_pid_pos[idx] = value
                self.hand.set_torque_positions(list(self._cmd_pid_pos))
        except Exception as e:
            self._set_status(f"Error: {e}")

    # ── Toolbar actions ───────────────────────────────────────────────────────

    def _cmd_home(self):
        reply = QMessageBox.question(
            self, "Home Hand",
            "Run the homing sequence?\nDo not touch the hand while it runs.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel)
        if reply != QMessageBox.StandardButton.Yes:
            return
        self._set_status("Homing in progress — please wait…")

        def _do():
            try:
                self.hand.home()
                self._sig.homing_done.emit("")
            except Exception as e:
                self._sig.homing_done.emit(str(e))

        threading.Thread(target=_do, daemon=True).start()

    def _cmd_manual_home(self):
        reply = QMessageBox.question(
            self, "Manual Homing",
            "Manual homing will:\n"
            "  1. Read each servo's current position as its extend position.\n"
            "  2. For each servo, press Space to ramp torque toward grasp.\n"
            "  3. Press Space again to record the grasped position.\n\n"
            "Move all servos to their extend (open) position first.\n"
            "Proceed?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel)
        if reply != QMessageBox.StandardButton.Yes:
            return

        self._manual_step_event = threading.Event()
        self._space_shortcut = QShortcut(Qt.Key.Key_Space, self)
        self._space_shortcut.activated.connect(self._on_space_pressed)
        self._set_status("Manual homing started — press Space to advance each step.")

        def _do():
            try:
                self.hand.home_manual(
                    self._manual_step_event,
                    lambda msg: self._sig.manual_prompt.emit(msg))
                self._sig.homing_done.emit("")
            except Exception as e:
                self._sig.homing_done.emit(str(e))

        threading.Thread(target=_do, daemon=True).start()

    def _on_space_pressed(self):
        if self._manual_step_event is not None:
            self._manual_step_event.set()

    def _on_homing_done(self, err: str):
        if self._space_shortcut is not None:
            self._space_shortcut.setEnabled(False)
            self._space_shortcut.deleteLater()
            self._space_shortcut = None
        self._manual_step_event = None

        if err:
            self._set_status(f"Homing error: {err}")
            QMessageBox.critical(self, "Homing Error", err)
        else:
            self._set_status("Homing complete.")
            self._sync_position_sliders()

    def _cmd_release(self):
        try:
            self.hand.set_positions([0] * NUM_SERVOS)
            self._cmd_positions = [0] * NUM_SERVOS
            if self._mode_group.checkedId() == 0:
                self._block_sliders = True
                for i, sl in enumerate(self._sliders):
                    sl.setValue(0)
                    self._val_labels[i].setText("0")
                self._block_sliders = False
            self._set_status("Released — all servos at extend.")
        except Exception as e:
            self._set_status(f"Release error: {e}")

    def _cmd_grasp(self):
        dlg = GraspDialog(self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        try:
            self.hand.grasp(dlg.speed_dps(), dlg.torque_limit())
            self._set_status(
                f"Grasping — {dlg.speed_dps():.1f} °/s, "
                f"torque limit {dlg.torque_limit()}.")
            threading.Thread(
                target=self._wait_for_grasp, daemon=True).start()
        except Exception as e:
            self._set_status(f"Grasp error: {e}")

    def _wait_for_grasp(self):
        """Poll velocities until all servos stop, then sync sliders."""
        # Give servos time to start moving before polling
        time.sleep(0.3)
        while True:
            vels = self.hand.get_velocities()
            if all(v == 0 for v in vels):
                break
            time.sleep(0.05)
        self._sig.grasp_done.emit()

    def _cmd_ping(self):
        self._set_status("Pinging servos…")
        QApplication.processEvents()
        results = self.hand.ping_all()
        lines = []
        for sid, ok in results.items():
            lines.append(
                f"Servo {sid} — {SERVO_NAMES[sid]}:  "
                f"{'OK' if ok else 'NO RESPONSE'}")
        n = sum(1 for ok in results.values() if ok)
        QMessageBox.information(
            self, "Ping Results",
            f"{n}/{NUM_SERVOS} servos responded:\n\n" + "\n".join(lines))
        self._set_status(f"Ping: {n}/{NUM_SERVOS} responded.")

    def _cmd_trim(self):
        dlg = TrimDialog(self.hand, self)
        dlg.exec()
        self._set_status(
            "Trim applied and saved." if dlg.result() else "Trim cancelled.")

    def _cmd_pid(self):
        dlg = PIDDialog(self.hand, self)
        dlg.exec()
        self._set_status(
            "PID constants saved." if dlg.result() else "PID edit cancelled.")

    def _cmd_set_id(self):
        dlg = SetIDDialog(self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            self._set_status("Set ID cancelled.")
            return
        new_id = dlg.new_id()
        cl     = dlg.current_limit()
        self._set_status("Scanning for a single servo on the bus…")
        QApplication.processEvents()
        ok = self.hand.set_servo_id(new_id, cl)
        if ok:
            QMessageBox.information(
                self, "Set Servo ID",
                f"Servo ID successfully changed to {new_id}.")
            self._set_status(f"Servo ID set to {new_id}.")
        else:
            QMessageBox.warning(
                self, "Set Servo ID",
                "Failed: expected exactly one servo on the bus, "
                "or the requested ID is already taken.")
            self._set_status("Set ID failed.")

    def _cmd_start_detector(self):
        dlg = DetectionDialog(self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return

        # Build the selected pipeline components
        src_info = AVAILABLE_SOURCES[dlg.selected_source_key()]
        source = src_info["class"]()

        det_key = dlg.selected_detector_key()
        detector_cls = get_detector_class(det_key)
        detector = detector_cls()

        calc_info = AVAILABLE_CALCULATORS[dlg.selected_calculator_key()]
        self._joint_calculator = calc_info["class"]()
        det_info = AVAILABLE_DETECTORS[det_key]
        self._det_frame_type = src_info["frame_type"]
        self._det_detector_name = det_info["name"]

        self._detector = HandDetector(source=source, detector=detector)
        self._detector.set_error_callback(
            lambda msg: self._sig.detector_error.emit(msg)
        )
        self._detector_cb_id = self._detector.register_callback(self._detector_callback)
        self._detector.start(show_preview=dlg.show_preview())

        for btn_id in (0, 1, 2):
            self._mode_group.button(btn_id).setEnabled(False)
        for sl in self._sliders:
            sl.setEnabled(False)

        self._rb_hand_detection.setEnabled(True)
        self._rb_hand_detection.setChecked(True)

        self._btn_start_detector.setEnabled(False)
        self._btn_stop_detector.setEnabled(True)
        self._set_status("Hand detection active.")

    def _cmd_stop_detector(self):
        if self._detector is not None:
            if self._detector_cb_id is not None:
                self._detector.remove_callback(self._detector_cb_id)
                self._detector_cb_id = None
            self._detector.stop()
            self._detector = None
        self._joint_calculator = None
        self._smoothed_positions = None

        params = {k: self._tune_sliders[k].value() / 100 for k in self._tune_sliders}
        with open(self._params_path, "w") as f:
            json.dump(params, f, indent=2)
        self._detector_params = params

        for btn_id in (0, 1, 2):
            self._mode_group.button(btn_id).setEnabled(True)
        for sl in self._sliders:
            sl.setEnabled(True)

        self._rb_hand_detection.setEnabled(False)
        self._mode_group.button(0).setChecked(True)

        self._block_sliders = True
        for i, sl in enumerate(self._sliders):
            sl.setValue(self._cmd_positions[i])
            self._val_labels[i].setText(str(self._cmd_positions[i]))
        self._block_sliders = False

        self._btn_start_detector.setEnabled(True)
        self._btn_stop_detector.setEnabled(False)
        self._set_status("Hand detection stopped.")

    def _on_detector_error(self, message: str) -> None:
        """Called (on the main thread via signal) when the detector thread
        encounters a fatal error.  Cleans up state and shows a dialog."""
        self._cmd_stop_detector()
        QMessageBox.critical(self, "Detection Error", message)

    def _detector_callback(self, hand_data: dict) -> None:
        if self._joint_calculator is None:
            return
        landmarks = LandmarkData(
            hands=hand_data.get("hands", []),
            timestamp=hand_data.get("timestamp", 0.0),
            frame_type=self._det_frame_type,
            detector_name=self._det_detector_name,
        )
        positions = self._joint_calculator.calculate(
            landmarks,
            hand_side=self.hand_type.capitalize(),
            mcp_pip_coupling=self._tune_sliders["mcp_pip_coupling"].value() / 100,
            cmc_abd_scale=   self._tune_sliders["cmc_abd_scale"].value()    / 100,
            cmc_abd_offset=  self._tune_sliders["cmc_abd_offset"].value()   / 100,
        )
        if positions is not None:
            self._sig.hand_positions.emit(positions)

    def _on_detector_positions(self, positions: list[int]) -> None:
        alpha = self._tune_sliders["smoothing_alpha"].value() / 100

        if self._smoothed_positions is None:
            self._smoothed_positions = [float(v) for v in positions]
        else:
            for i in range(len(positions)):
                self._smoothed_positions[i] += alpha * (positions[i] - self._smoothed_positions[i])

        smoothed = [max(0, min(65535, int(round(v)))) for v in self._smoothed_positions]

        self._cmd_positions = smoothed
        try:
            self.hand.set_positions(smoothed)
        except Exception as e:
            self._set_status(f"Detection error: {e}")
        for i, val in enumerate(smoothed):
            self._det_pos_sliders[i].setValue(val)
            self._det_pos_labels[i].setText(str(val))

    # ── Macro recording ─────────────────────────────────────────────────────

    def _cmd_start_record(self):
        self._macro_recorder.start()
        self._btn_start_record.setEnabled(False)
        self._btn_stop_record.setEnabled(True)
        self._set_status("Recording — move the hand to capture motion…")

    def _cmd_stop_record(self):
        path = self._macro_recorder.stop()
        self._btn_start_record.setEnabled(True)
        self._btn_stop_record.setEnabled(False)
        if path is None:
            self._set_status("Recording cancelled (no file).")
            return

        name, ok = QInputDialog.getText(
            self, "Save Macro", "Macro name:",
            text="my_macro")
        if ok and name.strip():
            path = self._macro_recorder.rename(name.strip())
            self._set_status(f"Macro saved: {path.name}")
        else:
            self._set_status(f"Macro saved: {path.name}")

    # ── Macro playback ───────────────────────────────────────────────────────

    def _cmd_load_macro(self):
        dlg = MacroSelectDialog(self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        path = dlg.selected_path()
        if path is None:
            return
        meta = self._macro_player.load(path)
        self._lbl_macro_name.setText(f"Loaded: {meta.name} ({meta.duration:.1f}s)")
        self._btn_play.setEnabled(True)
        self._btn_end_playback.setEnabled(True)
        self._macro_scrubber.setEnabled(True)
        self._macro_scrubber.setValue(0)
        self._set_status(f"Macro loaded: {meta.name}")

    def _cmd_play(self):
        player = self._macro_player
        if player.is_paused:
            player.resume()
            self._btn_play.setEnabled(False)
            self._btn_pause.setEnabled(True)
            return
        if player.is_playing:
            return
        # Enter playback mode — disable manual controls
        self._enter_playback_mode()
        player.play(
            speed=self._macro_speed.value(),
            on_finished=lambda: self._sig.status.emit("__playback_finished__"),
        )

    def _cmd_pause(self):
        self._macro_player.pause()
        self._btn_pause.setEnabled(False)
        self._btn_play.setEnabled(True)

    def _cmd_end_playback(self):
        self._macro_player.stop()
        self._exit_playback_mode()
        self._lbl_macro_name.setText("")
        self._macro_player.unload()
        self._set_status("Playback ended — manual control restored.")

    def _on_scrubber_released(self):
        fraction = self._macro_scrubber.value() / 1000.0
        self._macro_player.seek(fraction)

    def _on_speed_changed(self, value: float):
        self._macro_player.set_speed(value)

    def _enter_playback_mode(self):
        """Disable manual controls for exclusive playback."""
        self._playback_active = True
        for sl in self._sliders:
            sl.setEnabled(False)
        for btn_id in (0, 1, 2):
            self._mode_group.button(btn_id).setEnabled(False)
        self._btn_start_record.setEnabled(False)
        self._btn_load_macro.setEnabled(False)
        self._btn_start_detector.setEnabled(False)
        self._btn_play.setEnabled(False)
        self._btn_pause.setEnabled(True)
        self._btn_end_playback.setEnabled(True)
        self._macro_scrubber.setEnabled(True)

    def _exit_playback_mode(self):
        """Re-enable manual controls after playback."""
        self._playback_active = False
        for sl in self._sliders:
            sl.setEnabled(True)
        for btn_id in (0, 1, 2):
            self._mode_group.button(btn_id).setEnabled(True)
        self._btn_start_record.setEnabled(True)
        self._btn_load_macro.setEnabled(True)
        self._btn_start_detector.setEnabled(True)
        self._btn_play.setEnabled(False)
        self._btn_pause.setEnabled(False)
        self._btn_end_playback.setEnabled(False)
        self._macro_scrubber.setEnabled(False)
        self._macro_scrubber.setValue(0)

        # Restore slider values from current positions
        self._block_sliders = True
        for i, sl in enumerate(self._sliders):
            sl.setValue(self._cmd_positions[i])
            self._val_labels[i].setText(str(self._cmd_positions[i]))
        self._block_sliders = False

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_status(self, msg: str):
        self._statusbar.showMessage(msg)

    def closeEvent(self, event):
        if self._macro_recorder.is_recording:
            self._macro_recorder.stop()
        if self._macro_player.is_playing:
            self._macro_player.stop()
        if self._detector is not None and self._detector.is_running():
            if self._detector_cb_id:
                self._detector.remove_callback(self._detector_cb_id)
            self._detector.stop()
        self._timer.stop()
        if hasattr(self.hand_graphic, 'close_renderer'):
            self.hand_graphic.close_renderer()
        self.hand.close()
        if self._serial_logger is not None:
            self._serial_logger.close()
        event.accept()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="PyQt6 GUI for the Aero Open Hand Controller")
    parser.add_argument(
        "--hand", choices=["left", "right"], default=None,
        metavar="TYPE",
        help="Hand type: 'left' or 'right' (overrides dialog selection)")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
    pal = QPalette()
    c = {
        QPalette.ColorRole.Window:          QColor(45,  45,  48),
        QPalette.ColorRole.WindowText:      QColor(220, 220, 220),
        QPalette.ColorRole.Base:            QColor(28,  28,  30),
        QPalette.ColorRole.AlternateBase:   QColor(45,  45,  48),
        QPalette.ColorRole.ToolTipBase:     QColor(220, 220, 220),
        QPalette.ColorRole.ToolTipText:     QColor(30,  30,  30),
        QPalette.ColorRole.Text:            QColor(220, 220, 220),
        QPalette.ColorRole.Button:          QColor(60,  60,  65),
        QPalette.ColorRole.ButtonText:      QColor(220, 220, 220),
        QPalette.ColorRole.BrightText:      QColor(255, 80,  80),
        QPalette.ColorRole.Highlight:       QColor(42,  130, 218),
        QPalette.ColorRole.HighlightedText: QColor(255, 255, 255),
    }
    for role, color in c.items():
        pal.setColor(role, color)
    app.setPalette(pal)

    # Connection dialog
    dlg = PortDialog()
    if dlg.exec() != QDialog.DialogCode.Accepted:
        sys.exit(0)
    port      = dlg.port()
    hand_type = args.hand or dlg.hand_type()
    protocol  = dlg.protocol()

    serial_logger = None
    config_dir = os.path.join(os.path.dirname(__file__), "..", "configs")

    if port == "__simulate__":
        if protocol != "ttl":
            QMessageBox.warning(None, "Simulate",
                                "Simulation is only supported with the TTL protocol.\n"
                                "Switching to TTL.")
        from sim.hand_simulator import HandSimulator
        ser  = HandSimulator(os.path.join(_LOG_DIR, "sim"), servo_ids=list(range(NUM_SERVOS)))
        hand = HandTTL(ser, hand_type=hand_type, config_dir=config_dir)
        port = "SIMULATED"
    elif protocol == "ttl":
        if not port:
            QMessageBox.critical(None, "No Port", "No serial port was specified.")
            sys.exit(1)
        try:
            hand = HandTTL(port, hand_type=hand_type, config_dir=config_dir)
        except Exception as e:
            QMessageBox.critical(None, "Connection Failed", str(e))
            sys.exit(1)
        from hand.ttl.serial_file_logger import SerialFileLogger
        serial_logger = SerialFileLogger(os.path.join(_LOG_DIR, "real"))
        hand._protocol._logger = serial_logger
    else:  # esp32
        if not port:
            QMessageBox.critical(None, "No Port", "No serial port was specified.")
            sys.exit(1)
        try:
            hand = HandESP32(port, hand_type=hand_type)
        except Exception as e:
            QMessageBox.critical(None, "Connection Failed", str(e))
            sys.exit(1)

    win = MainWindow(hand, port, hand_type, serial_logger=serial_logger)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
