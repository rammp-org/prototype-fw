"""The RAMMP MIB bench window, in Qt.

PySide6 is the binding of choice (LGPL, Qt's own); PyQt6 works as a drop-in if
that is what is installed - only the import and the signal class differ. The
window owns a `MibBench` harness thread, drains its log through a queue on a
timer and redraws from `MibBench.snapshot()` (the harness never touches a
widget; Qt widgets are UI-thread only, like Tk's were).
"""
from __future__ import annotations

import argparse
import math
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError:  # pragma: no cover - the other binding
    from PyQt6 import QtCore, QtGui, QtWidgets  # type: ignore[no-redef]

import rammp_spec as spec
import rtps_host
import rtps_net
from mib_bench import (AUTO_ADAPTER, CONFIG_ADDRESS, CONFIG_PEER, JOY_TICK_MS, KEY_RATE,
                       KEY_STUCK_S, LOG_MAX_LINES, MOTOR_STALE_S, QUIET_PREFIXES, SPRING_RATE,
                       STATE_COLOURS, TICK_MS, MibBench, Stick, build_harness_args, format_age,
                       format_clock, format_diag_item, format_seat)

Qt = QtCore.Qt

RED = "#c62828"
ORANGE = "#b06a00"
GREEN = "#1e8a2e"
BLUE = "#1a6dd4"
#: "use the palette's text colour" (light and dark themes alike)
DEFAULT: Optional[str] = None


def _mix(a: QtGui.QColor, b: QtGui.QColor, t: float) -> str:
    return QtGui.QColor(int(a.red() * (1 - t) + b.red() * t),
                        int(a.green() * (1 - t) + b.green() * t),
                        int(a.blue() * (1 - t) + b.blue() * t)).name()


def _dim(amount: float = 0.45) -> str:
    """A caption colour between the theme's text and its background, so grey
    captions read on a dark theme too (a fixed #666666 vanished there)."""
    palette = QtWidgets.QApplication.palette()
    return _mix(palette.color(QtGui.QPalette.ColorRole.WindowText),
                palette.color(QtGui.QPalette.ColorRole.Window), amount)


def _colour(label: QtWidgets.QLabel, colour: Optional[str]) -> None:
    label.setStyleSheet("" if colour is None else f"color: {colour};")


def _caption(text: str) -> QtWidgets.QLabel:
    label = QtWidgets.QLabel(text)
    _colour(label, _dim())
    return label


def _note(text: str) -> QtWidgets.QLabel:
    label = QtWidgets.QLabel(text)
    label.setWordWrap(True)
    _colour(label, _dim())
    return label


def _disable_app_nap() -> None:
    """macOS puts a backgrounded app to sleep (App Nap): its timers slow to a
    crawl, so the tables stopped updating until the window was clicked. Tell
    the OS this process is doing user-initiated work. Best effort, through the
    Objective-C runtime; nothing to do elsewhere."""
    import sys
    if sys.platform != "darwin":
        return
    try:
        import ctypes
        import ctypes.util
        objc = ctypes.cdll.LoadLibrary(ctypes.util.find_library("objc"))
        ctypes.cdll.LoadLibrary(ctypes.util.find_library("Foundation"))
        objc.objc_getClass.restype = ctypes.c_void_p
        objc.objc_getClass.argtypes = [ctypes.c_char_p]
        objc.sel_registerName.restype = ctypes.c_void_p
        objc.sel_registerName.argtypes = [ctypes.c_char_p]
        send = objc.objc_msgSend
        send.restype = ctypes.c_void_p
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        info = send(objc.objc_getClass(b"NSProcessInfo"), objc.sel_registerName(b"processInfo"))
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p]
        reason = send(objc.objc_getClass(b"NSString"),
                      objc.sel_registerName(b"stringWithUTF8String:"),
                      b"RTPS bench: live telemetry")
        options = 0x00FFFFFF & ~(1 << 20)  # NSActivityUserInitiatedAllowingIdleSystemSleep
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint64, ctypes.c_void_p]
        token = send(info, objc.sel_registerName(b"beginActivityWithOptions:reason:"), options,
                     reason)
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        send(token, objc.sel_registerName(b"retain"))  # the activity lives as long as we do
        globals()["_APP_NAP_TOKEN"] = token
    except Exception:  # noqa: BLE001 - a missing runtime must not take the tool down
        pass


class JoystickPad(QtWidgets.QWidget):
    """The virtual joystick pad: a unit circle, the knob, the twist marker.

    Mouse drags and the keyboard (W/S A/D Q/E, space) are reported to the
    window through callbacks; the window integrates keys on its timer and
    hands the state back with set_state()."""

    SIZE = 240
    RADIUS = 100

    def __init__(self, on_press, on_move, on_release, on_key_press, on_key_release,
                 on_focus_out) -> None:
        super().__init__()
        self._on_press, self._on_move, self._on_release = on_press, on_move, on_release
        self._on_key_press, self._on_key_release = on_key_press, on_key_release
        self._on_focus_out = on_focus_out
        self._x = self._y = self._twist = 0.0
        self.setFixedSize(self.SIZE, self.SIZE)
        self.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.setCursor(Qt.CursorShape.CrossCursor)

    def set_state(self, x: float, y: float, twist: float) -> None:
        if (x, y, twist) != (self._x, self._y, self._twist):
            self._x, self._y, self._twist = x, y, twist
            self.update()

    def axes_at(self, pos: QtCore.QPointF) -> Tuple[float, float]:
        centre = self.SIZE / 2
        return (pos.x() - centre) / self.RADIUS, (centre - pos.y()) / self.RADIUS

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:  # noqa: N802 - Qt API
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        palette = self.palette()
        painter.fillRect(self.rect(), palette.color(QtGui.QPalette.ColorRole.Base))
        border = QtGui.QColor(BLUE if self.hasFocus() else _dim(0.6))
        painter.setPen(QtGui.QPen(border, 2))
        painter.drawRect(self.rect().adjusted(1, 1, -1, -1))
        centre = self.SIZE / 2
        painter.setPen(QtGui.QPen(QtGui.QColor(_dim(0.75)), 1))
        painter.drawLine(QtCore.QPointF(centre - self.RADIUS, centre),
                         QtCore.QPointF(centre + self.RADIUS, centre))
        painter.drawLine(QtCore.QPointF(centre, centre - self.RADIUS),
                         QtCore.QPointF(centre, centre + self.RADIUS))
        painter.setPen(QtGui.QPen(QtGui.QColor(_dim(0.5)), 2))
        painter.drawEllipse(QtCore.QPointF(centre, centre), self.RADIUS, self.RADIUS)
        painter.setPen(QtGui.QColor(_dim(0.5)))
        small = painter.font()
        small.setPointSize(9)
        painter.setFont(small)
        painter.drawText(QtCore.QRectF(0, 2, self.SIZE, 16), Qt.AlignmentFlag.AlignHCenter,
                         "forward (+y)")
        painter.drawText(QtCore.QRectF(self.SIZE - 40, centre - 18, 36, 16),
                         Qt.AlignmentFlag.AlignRight, "+x")
        # + twist = clockwise from the top
        angle = math.radians(90.0 - self._twist * 90.0)
        painter.setPen(QtGui.QPen(QtGui.QColor("#e0a000"), 3))
        painter.drawLine(QtCore.QPointF(centre, centre),
                         QtCore.QPointF(centre + self.RADIUS * math.cos(angle),
                                        centre - self.RADIUS * math.sin(angle)))
        knob = QtCore.QPointF(centre + self._x * self.RADIUS, centre - self._y * self.RADIUS)
        painter.setPen(QtGui.QPen(QtGui.QColor("#0d4a96"), 2))
        painter.setBrush(QtGui.QColor(BLUE))
        painter.drawEllipse(knob, 12, 12)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            self.setFocus()
            self._on_press(self.axes_at(event.position()))

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if event.buttons() & Qt.MouseButton.LeftButton:
            self._on_move(self.axes_at(event.position()))

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            self._on_release()

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:  # noqa: N802
        if not self._on_key_press(event.key(), event.isAutoRepeat()):
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:  # noqa: N802
        if event.isAutoRepeat():
            return  # the key is still down; the matching press refreshes it
        if not self._on_key_release(event.key()):
            super().keyReleaseEvent(event)

    def focusOutEvent(self, event: QtGui.QFocusEvent) -> None:  # noqa: N802
        self._on_focus_out()
        self.update()
        super().focusOutEvent(event)

    def focusInEvent(self, event: QtGui.QFocusEvent) -> None:  # noqa: N802
        self.update()
        super().focusInEvent(event)


class BaseView(QtWidgets.QWidget):
    """The base as the MIB reports it: side and front views of the plate (its
    elevation and tilts, from the M1..M3 seat rows) and a top view with each
    corner's linkage angle as a bar within its row's range (the L1..L4 rows).

    Display geometry only: the footprint below is nominal (the real one lives
    in the firmware's hw_config) and sets the drawing's proportions, not any
    number shown."""

    HALF_LENGTH_M = 0.406
    HALF_WIDTH_M = 0.267
    #: vertical scale of the side / front views: +/- this many mm fill the view
    ELEVATION_SPAN_MM = 150.0

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(640, 300)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                           QtWidgets.QSizePolicy.Policy.Expanding)
        self._pose: Optional[Dict[str, float]] = None  # elevation_mm, fb_deg, lat_deg
        self._corners: Dict[str, Optional[float]] = {}  # FR / FL / BR / BL -> deg
        self._ranges: Dict[str, Tuple[float, float]] = {}
        self._out_of_range: set = set()
        self._fresh = False

    def set_state(self, pose: Optional[Dict[str, float]], corners: Dict[str, Optional[float]],
                  ranges: Dict[str, Tuple[float, float]], out_of_range: set,
                  fresh: bool) -> None:
        self._pose, self._corners, self._ranges = pose, corners, ranges
        self._out_of_range, self._fresh = out_of_range, fresh
        self.update()

    # ------------------------------------------------------------ drawing

    def _corner_heights_mm(self) -> Dict[str, float]:
        """Corner heights from the pose (the firmware's plane model), for the views."""
        if self._pose is None:
            return {}
        e = self._pose["elevation_mm"]
        fb = math.tan(math.radians(self._pose["fb_deg"])) * self.HALF_LENGTH_M * 1000.0
        lat = math.tan(math.radians(self._pose["lat_deg"])) * self.HALF_WIDTH_M * 1000.0
        return {"FR": e + fb - lat, "FL": e + fb + lat, "BR": e - fb - lat, "BL": e - fb + lat}

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:  # noqa: N802 - Qt API
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        palette = self.palette()
        painter.fillRect(self.rect(), palette.color(QtGui.QPalette.ColorRole.Base))
        text = palette.color(QtGui.QPalette.ColorRole.WindowText)
        dim = QtGui.QColor(_dim(0.55))
        faint = QtGui.QColor(_dim(0.8))
        w, h = self.width(), self.height()
        third = w / 3.0
        pad = 14
        small = painter.font()
        small.setPointSize(9)
        painter.setFont(small)
        plate = QtGui.QColor(BLUE) if self._fresh else dim
        heights = self._corner_heights_mm()

        def elevation_view(x0: float, title: str, left_key: str, right_key: str,
                           left_label: str, right_label: str, half_m: float,
                           tilt_deg: float) -> None:
            # a plate seen edge-on: its two ends at the corner heights
            cx = x0 + third / 2
            ground = h - pad - 14
            top = pad + 16
            span = ground - top
            scale = span / (2 * self.ELEVATION_SPAN_MM)  # px per mm
            mid = top + span / 2
            painter.setPen(text)
            painter.drawText(QtCore.QRectF(x0, pad, third, 14), Qt.AlignmentFlag.AlignHCenter,
                             title)
            painter.setPen(QtGui.QPen(faint, 1, Qt.PenStyle.DashLine))
            painter.drawLine(QtCore.QPointF(x0 + pad, mid), QtCore.QPointF(x0 + third - pad, mid))
            painter.setPen(dim)
            painter.drawText(QtCore.QRectF(x0, mid + 2, third, 12), Qt.AlignmentFlag.AlignHCenter,
                             "level 0 mm")
            half_px = third / 2 - 2 * pad
            if heights:
                hl, hr = heights[left_key], heights[right_key]
                yl = mid - max(-self.ELEVATION_SPAN_MM, min(self.ELEVATION_SPAN_MM, hl)) * scale
                yr = mid - max(-self.ELEVATION_SPAN_MM, min(self.ELEVATION_SPAN_MM, hr)) * scale
                painter.setPen(QtGui.QPen(plate, 6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
                painter.drawLine(QtCore.QPointF(cx - half_px, yl), QtCore.QPointF(cx + half_px, yr))
                painter.setPen(QtGui.QPen(dim, 1))
                for x, y in ((cx - half_px, yl), (cx + half_px, yr)):
                    painter.drawLine(QtCore.QPointF(x, y), QtCore.QPointF(x, ground))
                painter.setPen(text)
                for x, y, value in ((cx - half_px, yl, hl), (cx + half_px, yr, hr)):
                    label_y = y - 28 if y - 28 > top else y + 10
                    painter.drawText(QtCore.QRectF(x - 30, label_y, 60, 14),
                                     Qt.AlignmentFlag.AlignHCenter, f"{value:+.0f} mm")
                painter.drawText(QtCore.QRectF(x0, ground - 2, third, 14),
                                 Qt.AlignmentFlag.AlignHCenter, f"tilt {tilt_deg:+.1f} deg")
            else:
                painter.setPen(dim)
                painter.drawText(QtCore.QRectF(x0, mid - 7, third, 14),
                                 Qt.AlignmentFlag.AlignHCenter, "no MibStatus")
            painter.setPen(dim)
            painter.drawText(QtCore.QRectF(cx - half_px - 30, ground + 1, 60, 12),
                             Qt.AlignmentFlag.AlignHCenter, left_label)
            painter.drawText(QtCore.QRectF(cx + half_px - 30, ground + 1, 60, 12),
                             Qt.AlignmentFlag.AlignHCenter, right_label)

        fb = self._pose["fb_deg"] if self._pose else 0.0
        lat = self._pose["lat_deg"] if self._pose else 0.0
        # side view from the LEFT: back on the left, front on the right
        elevation_view(0, "side (from the left)", "BL", "FL", "back", "front",
                       self.HALF_LENGTH_M, fb)
        # front view: left on the left
        elevation_view(third, "front", "FL", "FR", "left", "right", self.HALF_WIDTH_M, lat)

        # top view: the footprint with a corner bar per linkage
        x0 = 2 * third
        painter.setPen(text)
        painter.drawText(QtCore.QRectF(x0, pad, third, 14), Qt.AlignmentFlag.AlignHCenter,
                         "top (front up): linkage angle within its range")
        box = QtCore.QRectF(x0 + 2 * pad + 20, pad + 36, third - 4 * pad - 40, h - 2 * pad - 62)
        painter.setPen(QtGui.QPen(dim, 1))
        painter.drawRect(box)
        painter.drawText(QtCore.QRectF(box.left(), box.top() - 13, box.width(), 12),
                         Qt.AlignmentFlag.AlignHCenter, "front")
        bar_w, bar_h = 26, min(90.0, box.height() * 0.4)
        spots = {"FL": (box.left() - bar_w - 8, box.top() + 4),
                 "FR": (box.right() + 8, box.top() + 4),
                 "BL": (box.left() - bar_w - 8, box.bottom() - bar_h - 4),
                 "BR": (box.right() + 8, box.bottom() - bar_h - 4)}
        for corner, (bx, by) in spots.items():
            rect = QtCore.QRectF(bx, by, bar_w, bar_h)
            painter.setPen(QtGui.QPen(dim, 1))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRect(rect)
            angle = self._corners.get(corner)
            lo, hi = self._ranges.get(corner, (0.0, 1.0))
            colour = (QtGui.QColor(RED) if corner in self._out_of_range
                      else (QtGui.QColor(GREEN) if self._fresh else dim))
            if angle is not None and hi > lo:
                frac = max(0.0, min(1.0, (angle - lo) / (hi - lo)))
                fill_h = frac * (bar_h - 4)
                painter.fillRect(QtCore.QRectF(bx + 2, by + bar_h - 2 - fill_h, bar_w - 4, fill_h),
                                 colour)
                # a marker when the angle is outside the range (fill is pinned)
                if angle < lo or angle > hi:
                    painter.setPen(QtGui.QPen(QtGui.QColor(RED), 2))
                    y = by + 2 if angle > hi else by + bar_h - 2
                    painter.drawLine(QtCore.QPointF(bx - 3, y), QtCore.QPointF(bx + bar_w + 3, y))
            painter.setPen(text)
            label_rect = QtCore.QRectF(bx - 20, by + bar_h + 2, bar_w + 40, 26)
            painter.drawText(label_rect, Qt.AlignmentFlag.AlignHCenter | Qt.TextFlag.TextWordWrap,
                             corner + ("\n--" if angle is None else f"\n{angle:+.1f}"))
            painter.setPen(dim)
            painter.drawText(QtCore.QRectF(bx - 20, by - 13, bar_w + 40, 12),
                             Qt.AlignmentFlag.AlignHCenter, f"{hi:+.0f}")
        if self._pose:
            painter.setPen(text)
            painter.drawText(box.adjusted(4, 4, -4, -4),
                             Qt.AlignmentFlag.AlignCenter | Qt.TextFlag.TextWordWrap,
                             f"elev {self._pose['elevation_mm']:+.0f} mm\n"
                             f"fb {fb:+.1f} deg (front up +)\nlat {lat:+.1f} deg (left up +)")


class MibWindow(QtWidgets.QMainWindow):
    """The window, plus the harness thread it starts and stops."""

    KEY_AXES = {
        Qt.Key.Key_W: ("y", +1), Qt.Key.Key_S: ("y", -1), Qt.Key.Key_D: ("x", +1),
        Qt.Key.Key_A: ("x", -1), Qt.Key.Key_E: ("twist", +1), Qt.Key.Key_Q: ("twist", -1),
    }

    def __init__(self, cli: argparse.Namespace) -> None:
        super().__init__()
        self.cli = cli
        self.harness: Optional[MibBench] = None
        self.thread: Optional[threading.Thread] = None

        # The harness runs on its own thread and widgets are UI-thread only, so
        # log lines cross over through a queue that _tick drains here.
        # rtps_host.log calls this from the network thread.
        self.log_queue: "queue.Queue[str]" = queue.Queue()
        rtps_host.LOG_SINK = self.log_queue.put
        # Worker threads hand results back here; only _tick touches widgets.
        self.result_queue: "queue.Queue[tuple]" = queue.Queue()
        self.adapters: list = []
        self.config = rtps_net.load_config()

        # Virtual joystick bookkeeping (UI thread only).
        self._keys: Dict[int, float] = {}
        self._dragging = False
        self._twist_dragging = False
        self._seat_controls_enabled: Optional[bool] = None

        self.setWindowTitle("RAMMP MIB bench")
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        self.root_layout = QtWidgets.QVBoxLayout(central)
        self.root_layout.setContentsMargins(8, 8, 8, 8)
        self.root_layout.setSpacing(6)
        self._build_connection()
        self._build_status()
        self.tabs = QtWidgets.QTabWidget()
        self.root_layout.addWidget(self.tabs)
        self._build_drive()
        self._build_actuators()
        self._build_base()
        self._build_diagnostics()
        self._build_motors()
        self._build_log()

        estop = QtGui.QShortcut(QtGui.QKeySequence("Escape"), self)
        estop.setContext(Qt.ShortcutContext.ApplicationShortcut)
        estop.activated.connect(self._estop)

        self.tick_timer = QtCore.QTimer(self)
        self.tick_timer.timeout.connect(self._tick)
        self.tick_timer.start(TICK_MS)
        self.joy_timer = QtCore.QTimer(self)
        self.joy_timer.timeout.connect(self._joy_tick)
        self.joy_timer.start(JOY_TICK_MS)
        if self.peer_edit.text().strip():
            # A remembered or given MIB: connect straight away, the common case.
            QtCore.QTimer.singleShot(300, self._connect)

    # ------------------------------------------------------------ widgets

    def _group(self, title: str) -> Tuple[QtWidgets.QGroupBox, QtWidgets.QGridLayout]:
        box = QtWidgets.QGroupBox(title)
        grid = QtWidgets.QGridLayout(box)
        grid.setContentsMargins(8, 6, 8, 6)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(3)
        return box, grid

    def _build_connection(self) -> None:
        box, grid = self._group("Connection")
        self.root_layout.addWidget(box)
        grid.addWidget(QtWidgets.QLabel("MIB"), 0, 0)
        self.peer_edit = QtWidgets.QLineEdit(
            self.cli.peer[0] if self.cli.peer else self.config.get(CONFIG_PEER, ""))
        self.peer_edit.setFixedWidth(160)
        grid.addWidget(self.peer_edit, 0, 1)
        self.detect_button = QtWidgets.QPushButton("Detect")
        self.detect_button.clicked.connect(self._on_detect)
        grid.addWidget(self.detect_button, 0, 2)
        self.scan_button = QtWidgets.QPushButton("Scan...")
        self.scan_button.clicked.connect(self._on_scan)
        grid.addWidget(self.scan_button, 0, 3)
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self._toggle_connection)
        grid.addWidget(self.connect_button, 0, 4)

        grid.addWidget(QtWidgets.QLabel("Via"), 1, 0)
        self.address_combo = QtWidgets.QComboBox()
        self.address_combo.setEditable(True)
        self.address_combo.setMinimumWidth(380)
        grid.addWidget(self.address_combo, 1, 1, 1, 3)
        self._refresh_adapters()
        self.address_combo.setCurrentText(
            self.cli.advertised_address or self.config.get(CONFIG_ADDRESS) or AUTO_ADAPTER)
        self.connection_label = QtWidgets.QLabel("not connected")
        grid.addWidget(self.connection_label, 1, 4, 1, 2)
        grid.addWidget(_note(
            f"domain {self.cli.domain_id}, participant {self.cli.participant_id}, "
            f"node '{self.cli.node_name}'. Via defaults to Auto, which picks the adapter that "
            "actually routes to the MIB - not the one that reaches the internet."), 2, 0, 1, 6)
        grid.setColumnStretch(5, 1)

    def _build_status(self) -> None:
        box, grid = self._group("MibStatus (rammp/mib/status)")
        self.root_layout.addWidget(box)
        self.status_labels: Dict[str, QtWidgets.QLabel] = {}
        cells = (
            ("Link", "link"), ("State", "state"), ("Profile", "profile"),
            ("Speed", "speed"), ("Seq / gaps", "seq"), ("Rate", "rate"),
            ("MIB clock", "clock"), ("status_text", "status_text"), ("error_message", "error"),
        )
        for index, (caption, key) in enumerate(cells):
            row, column = divmod(index, 3)
            grid.addWidget(_caption(caption), row, column * 2)
            label = QtWidgets.QLabel("--")
            grid.addWidget(label, row, column * 2 + 1)
            self.status_labels[key] = label
        self.status_labels["link"].setText("not connected")
        grid.addWidget(_caption("error_footer"), 3, 0)
        self.status_labels["footer"] = QtWidgets.QLabel("--")
        grid.addWidget(self.status_labels["footer"], 3, 1, 1, 5)
        grid.addWidget(_caption("seat"), 4, 0, Qt.AlignmentFlag.AlignTop)
        self.seat_summary = QtWidgets.QLabel("--")
        self.seat_summary.setWordWrap(True)
        grid.addWidget(self.seat_summary, 4, 1, 1, 5)
        for column in (1, 3, 5):
            grid.setColumnStretch(column, 1)

    def _build_drive(self) -> None:
        tab = QtWidgets.QWidget()
        self.tabs.addTab(tab, "Drive")
        row = QtWidgets.QHBoxLayout(tab)
        left = QtWidgets.QVBoxLayout()
        row.addLayout(left)
        right = QtWidgets.QVBoxLayout()
        row.addLayout(right, 1)

        profile_box, profile_grid = self._group("Profile (DriveCommand.profile)")
        left.addWidget(profile_box)
        self.profile_group = QtWidgets.QButtonGroup(self)
        default_profile = spec.ENUM_VALUES["DriveProfile"].get("NORMAL", 1)
        for column, value in enumerate(sorted(spec.DRIVE_PROFILE_NAMES)):
            button = QtWidgets.QRadioButton(spec.DRIVE_PROFILE_NAMES[value])
            button.setChecked(value == default_profile)
            self.profile_group.addButton(button, value)
            profile_grid.addWidget(button, 0, column)
        self.profile_group.idClicked.connect(lambda _id: self._on_profile())

        drive_box, drive_grid = self._group("Drive (DriveCommand.request)")
        left.addWidget(drive_box)
        enable = QtWidgets.QPushButton("Enable")
        enable.clicked.connect(lambda: self._drive(spec.ENUM_VALUES["DriveRequest"]["ENABLE"]))
        drive_grid.addWidget(enable, 0, 0)
        disable = QtWidgets.QPushButton("Disable")
        disable.clicked.connect(lambda: self._drive(spec.ENUM_VALUES["DriveRequest"]["DISABLE"]))
        drive_grid.addWidget(disable, 0, 1)
        self.drive_label = _caption("nothing sent yet")
        drive_grid.addWidget(self.drive_label, 1, 0, 1, 2)

        self.estop_button = QtWidgets.QPushButton("E-STOP")
        self.estop_button.setFixedSize(220, 64)
        self.estop_button.setStyleSheet(
            "QPushButton { background: #c62828; color: white; font-size: 20px; font-weight: bold;"
            " border: 2px solid #7f0000; border-radius: 4px; }"
            "QPushButton:pressed { background: #8e0000; }")
        self.estop_button.clicked.connect(self._estop)
        left.addSpacing(6)
        left.addWidget(self.estop_button, 0, Qt.AlignmentFlag.AlignHCenter)
        left.addWidget(_caption("zero XYTwist, then DISABLE (also: Esc)"), 0,
                       Qt.AlignmentFlag.AlignHCenter)

        options_box, options_grid = self._group("Stream (XYTwist)")
        left.addWidget(options_box)
        self.always_stream_check = QtWidgets.QCheckBox("always stream (not only while ENABLED)")
        self.always_stream_check.toggled.connect(lambda _c: self._apply_options())
        options_grid.addWidget(self.always_stream_check, 0, 0)
        self.hold_check = QtWidgets.QCheckBox("hold (no spring return)")
        self.hold_check.toggled.connect(lambda _c: self._apply_options())
        options_grid.addWidget(self.hold_check, 1, 0)
        self.button_check = QtWidgets.QCheckBox("JOYSTICK button pressed (buttons bit 0)")
        self.button_check.toggled.connect(lambda _c: self._apply_options())
        options_grid.addWidget(self.button_check, 2, 0)
        self.stream_label = _caption(f"{self.cli.xy_hz:g} Hz: idle")
        options_grid.addWidget(self.stream_label, 3, 0)
        left.addStretch(1)

        pad_box, pad_grid = self._group("Virtual joystick (click the pad for keyboard: "
                                        "W/S A/D Q/E, space = centre)")
        right.addWidget(pad_box, 1)
        self.pad = JoystickPad(self._on_pad_press, self._on_pad_motion, self._on_pad_release,
                               self._on_key_press, self._on_key_release, self._clear_keys)
        pad_grid.addWidget(self.pad, 0, 0, 4, 1, Qt.AlignmentFlag.AlignTop)
        pad_grid.addWidget(QtWidgets.QLabel("twist (- anticlockwise, + clockwise)"), 0, 1,
                           Qt.AlignmentFlag.AlignBottom)
        self.twist_slider = QtWidgets.QSlider(Qt.Orientation.Horizontal)
        self.twist_slider.setRange(-1000, 1000)
        self.twist_slider.setFixedWidth(200)
        self.twist_slider.sliderPressed.connect(self._on_twist_press)
        self.twist_slider.valueChanged.connect(self._on_twist_changed)
        self.twist_slider.sliderReleased.connect(self._on_twist_release)
        pad_grid.addWidget(self.twist_slider, 1, 1, Qt.AlignmentFlag.AlignTop)
        self.live_label = QtWidgets.QLabel("x +0.00  y +0.00  twist +0.00")
        self.live_label.setFont(
            QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        pad_grid.addWidget(self.live_label, 2, 1, Qt.AlignmentFlag.AlignTop)
        centre = QtWidgets.QPushButton("Centre")
        centre.clicked.connect(self._centre_stick)
        pad_grid.addWidget(centre, 3, 1, Qt.AlignmentFlag.AlignTop)
        pad_grid.setColumnStretch(2, 1)
        pad_grid.setRowStretch(3, 1)

    def _build_actuators(self) -> None:
        tab = QtWidgets.QWidget()
        self.tabs.addTab(tab, "Actuators")
        column = QtWidgets.QVBoxLayout(tab)
        box, grid = self._group("Seat axes (SeatCommand -> the MIB; value from "
                                "MibStatus.currentSeatState)")
        column.addWidget(box)
        column.addStretch(1)
        for col, heading in enumerate(("", "Axis", "Value", "", "", "", "Target", "", "",
                                       "Spec row (HMI range)", "Jog")):
            grid.addWidget(_caption(heading), 0, col)
        self.seat_value_labels: List[QtWidgets.QLabel] = []
        self.seat_target_edits: List[QtWidgets.QLineEdit] = []
        self.seat_controls: List[QtWidgets.QWidget] = []
        for index, axis in enumerate(spec.SEAT_AXES):
            row = index + 1
            grid.addWidget(QtWidgets.QLabel(axis.short), row, 0)
            grid.addWidget(QtWidgets.QLabel(axis.label), row, 1)
            value = QtWidgets.QLabel("--")
            value.setMinimumWidth(90)
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            grid.addWidget(value, row, 2)
            self.seat_value_labels.append(value)
            minus = QtWidgets.QPushButton("-")
            minus.setFixedWidth(36)
            minus.clicked.connect(lambda _c=False, i=index: self._seat_step(i, -1))
            grid.addWidget(minus, row, 3)
            plus = QtWidgets.QPushButton("+")
            plus.setFixedWidth(36)
            plus.clicked.connect(lambda _c=False, i=index: self._seat_step(i, +1))
            grid.addWidget(plus, row, 4)
            grid.addWidget(_caption(f"step {axis.format(axis.step)}"), row, 5)
            target = QtWidgets.QLineEdit()
            target.setFixedWidth(80)
            target.setAlignment(Qt.AlignmentFlag.AlignRight)
            target.returnPressed.connect(lambda i=index: self._seat_set(i))
            grid.addWidget(target, row, 6)
            self.seat_target_edits.append(target)
            set_button = QtWidgets.QPushButton("Set")
            set_button.setFixedWidth(48)
            set_button.clicked.connect(lambda _c=False, i=index: self._seat_set(i))
            grid.addWidget(set_button, row, 7)
            controls = [minus, plus, target, set_button]
            if axis.min_value <= 0 <= axis.max_value:
                zero = QtWidgets.QPushButton("Zero")
                zero.setFixedWidth(56)
                zero.clicked.connect(lambda _c=False, i=index: self._seat_zero(i))
                grid.addWidget(zero, row, 8)
                controls.append(zero)
            grid.addWidget(_caption(f"{axis.format(axis.min_value)} .. "
                                    f"{axis.format(axis.max_value)} {axis.unit}"), row, 9)
            if axis.short.upper().startswith("L"):
                # the linkage rows: the recovery move for a corner outside its
                # range (the MIB refuses every other move for it)
                jog = QtWidgets.QHBoxLayout()
                for delta in (-0.5, +0.5):
                    button = QtWidgets.QPushButton(f"{delta:+g}")
                    button.setFixedWidth(48)
                    button.clicked.connect(lambda _c=False, i=index, d=delta: self._seat_jog(i, d))
                    jog.addWidget(button)
                    # not in seat_controls: a jog must stay available while the MIB
                    # reports ERROR for a corner outside its range
                grid.addLayout(jog, row, 10)
            self.seat_controls.extend(controls)
        footer = QtWidgets.QHBoxLayout()
        self.force_seat_check = QtWidgets.QCheckBox("force (send even when the MIB is not IDLE)")
        footer.addWidget(self.force_seat_check)
        self.seat_age_label = _caption("values: no MibStatus yet")
        footer.addWidget(self.seat_age_label)
        self.seat_note = _caption("")
        footer.addWidget(self.seat_note)
        footer.addStretch(1)
        grid.addLayout(footer, len(spec.SEAT_AXES) + 1, 0, 1, 10)
        grid.addWidget(_note(
            "Targets are absolute, in whole units, like the HMI's seat_step: - / + send one "
            "step from the value the MIB last reported - only while that value is fresh (the "
            "MIB reports every 0.5 s; a step from an old value is not one step). Nothing is "
            "clamped here: the MIB's own per-corner limits decide (they are not on the wire; "
            "the spec row is the HMI's jog range). A corner the MIB reports out of range takes "
            "only a jog. The MIB refuses seat commands unless its state is IDLE, so the "
            "controls grey out otherwise."), len(spec.SEAT_AXES) + 2, 0, 1, 10)
        grid.setColumnStretch(10, 1)

    @staticmethod
    def _table(headings: List[str], rows: int) -> QtWidgets.QTableWidget:
        table = QtWidgets.QTableWidget(rows, len(headings))
        table.setHorizontalHeaderLabels(headings)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.NoSelection)
        table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        table.horizontalHeader().setStretchLastSection(True)
        table.setAlternatingRowColors(True)
        for row in range(rows):
            for col in range(len(headings)):
                table.setItem(row, col, QtWidgets.QTableWidgetItem("--"))
        return table

    @staticmethod
    def _set_row(table: QtWidgets.QTableWidget, row: int, values: Tuple[Any, ...],
                 colour: Optional[str]) -> None:
        # None = the theme's text colour (an empty brush resets to the palette)
        brush = QtGui.QBrush() if colour is None else QtGui.QBrush(QtGui.QColor(colour))
        for col, value in enumerate(values):
            item = table.item(row, col)
            text = str(value)
            if item.text() != text:
                item.setText(text)
            item.setForeground(brush)

    def _build_base(self) -> None:
        tab = QtWidgets.QWidget()
        self.tabs.addTab(tab, "Base")
        column = QtWidgets.QVBoxLayout(tab)
        self.base_view = BaseView()
        column.addWidget(self.base_view, 1)
        column.addWidget(_note(
            "Drawn from MibStatus: the plate from the M1..M3 rows (elevation, front/back tilt, "
            "lateral tilt - corner heights are the plane model with a nominal footprint), the "
            "corner bars from the L1..L4 rows within their spec-row range. Blue / green = fresh, "
            "grey = no recent status, red = the MIB reports that corner out of range."))

    def _build_diagnostics(self) -> None:
        tab = QtWidgets.QWidget()
        self.tabs.addTab(tab, "Diagnostics")
        column = QtWidgets.QVBoxLayout(tab)
        box, grid = self._group("Diagnostics (from the MIB, one row per RAMMP_DIAG_TABLE item)")
        column.addWidget(box, 1)
        self.diag_table = self._table(["short", "label", "r1", "r2", "r3", "age", "rate"],
                                      len(spec.DIAGNOSTICS))
        for row, item in enumerate(spec.DIAGNOSTICS):
            self._set_row(self.diag_table, row, (item.short, item.label, "", "", "", "--", "--"),
                          _dim())
        grid.addWidget(self.diag_table, 0, 0)
        self.diag_summary = _caption("no Diagnostics yet")
        grid.addWidget(self.diag_summary, 1, 0)

    def _build_motors(self) -> None:
        tab = QtWidgets.QWidget()
        self.tabs.addTab(tab, "Motors")
        column = QtWidgets.QVBoxLayout(tab)
        box, grid = self._group("MotorState (one topic per RAMMP_AXIS_TABLE axis, all "
                                "subscribed)")
        column.addWidget(box, 1)
        headings = ["axis", "api", "state", "mode", "fault", "position rad (deg)",
                    "velocity rad/s (rpm)", "torque Nm", "iq A", "vbus V", "temps C", "cmd age",
                    "uptime", "age"]
        self.motor_table = self._table(headings, len(spec.AXES))
        self.motor_rows = {axis.id: row for row, axis in enumerate(spec.AXES)}
        for row, axis in enumerate(spec.AXES):
            self._set_row(self.motor_table, row, (axis.label,) + ("--",) * (len(headings) - 1),
                          _dim())
        grid.addWidget(self.motor_table, 0, 0)
        grid.addWidget(_caption("Rows never seen stay grey; a row with nothing for "
                                f"{MOTOR_STALE_S:g} s turns red; FAULT is orange."), 1, 0)

    def _build_log(self) -> None:
        box, grid = self._group("Log")
        self.root_layout.addWidget(box, 1)
        bar = QtWidgets.QHBoxLayout()
        self.verbose_check = QtWidgets.QCheckBox("verbose (discovery chatter)")
        self.verbose_check.setChecked(bool(self.cli.verbose))
        bar.addWidget(self.verbose_check)
        bar.addStretch(1)
        clear = QtWidgets.QPushButton("Clear")
        clear.clicked.connect(self._clear_log)
        bar.addWidget(clear)
        grid.addLayout(bar, 0, 0)
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(LOG_MAX_LINES)
        self.log_text.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self.log_text.setFont(
            QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        self.log_text.setMinimumHeight(140)
        grid.addWidget(self.log_text, 1, 0)

    # ------------------------------------------------------------ connection

    def _refresh_adapters(self) -> None:
        """Populate the Via dropdown with named adapters, Auto first."""
        self.adapters = rtps_net.list_adapters()
        current = self.address_combo.currentText()
        self.address_combo.clear()
        self.address_combo.addItem(AUTO_ADAPTER)
        for adapter in self.adapters:
            if adapter.is_up:
                self.address_combo.addItem(adapter.label())
        for adapter in self.adapters:
            if not adapter.is_up:
                self.address_combo.addItem(adapter.label())
        if current:
            self.address_combo.setCurrentText(current)

    def _selected_address(self) -> Optional[str]:
        """The advertised address to use, or None for Auto."""
        chosen = self.address_combo.currentText().strip()
        if not chosen or chosen == AUTO_ADAPTER:
            return None
        for adapter in self.adapters:
            if chosen == adapter.label():
                return adapter.ip
        return chosen  # a hand-typed address

    def _on_detect(self) -> None:
        """Find the MIB on a worker thread; never blocks the window."""
        if self.harness is not None:
            self._append_log("[gui] disconnect before detecting")
            return
        saved = self.peer_edit.text().strip() or self.config.get(CONFIG_PEER)
        self.detect_button.setEnabled(False)
        self.connection_label.setText("looking for the MIB...")

        def work() -> None:
            found = rtps_net.discover_board(saved, progress=self.log_queue.put)
            self.result_queue.put(("detect", found))

        threading.Thread(target=work, daemon=True).start()

    def _on_scan(self) -> None:
        """Sweep a subnet, after showing which one."""
        if self.harness is not None:
            self._append_log("[gui] disconnect before scanning")
            return
        default = rtps_net.subnet_guess(self.peer_edit.text().strip()
                                        or self.config.get(CONFIG_PEER))
        subnet, ok = QtWidgets.QInputDialog.getText(
            self, "Scan for the MIB", "Subnet to sweep with RTPS discovery probes:",
            text=default or "")
        if not ok or not subnet:
            return
        self.scan_button.setEnabled(False)
        self.connection_label.setText(f"scanning {subnet}...")

        def work() -> None:
            try:
                found = rtps_net.scan_subnet(subnet, progress=self.log_queue.put)
            except ValueError as exc:
                self.log_queue.put(f"[gui] {subnet} is not a valid subnet: {exc}")
                found = []
            self.result_queue.put(("scan", found[0] if found else None))

        threading.Thread(target=work, daemon=True).start()

    def _discovery_finished(self, kind: str, found: Optional[str]) -> None:
        self.detect_button.setEnabled(True)
        self.scan_button.setEnabled(True)
        if found:
            self.peer_edit.setText(found)
            adapter = rtps_net.adapter_for(found)
            via = adapter.label() if adapter else "unknown"
            self.connection_label.setText(f"found {found} via {via}")
            self._append_log(f"[gui] MIB at {found}, reachable via {via}")
            return
        hint = "try Scan" if kind == "detect" else "check the subnet, and that the MIB is on"
        self.connection_label.setText(f"MIB not found - {hint}")

    def _toggle_connection(self) -> None:
        if self.harness is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        if self.harness is not None:
            return
        peer = self.peer_edit.text().strip() or None
        args = build_harness_args(self.cli, peer, self._selected_address())
        try:
            self.harness = MibBench(args, self.cli.xy_hz)
        except OSError as exc:
            # a bad address or a port already taken by another script
            self._append_log(f"[gui] connect failed: {exc}")
            self.connection_label.setText("connect failed")
            self.harness = None
            return
        self._apply_options()
        self.harness.profile = self.profile_group.checkedId()
        self.thread = threading.Thread(target=self.harness.run, daemon=True)
        self.thread.start()
        self.connect_button.setText("Disconnect")
        self.connection_label.setText(
            f"participant up as {args.advertised_address}" + (f" -> {peer}" if peer else ""))
        self.status_labels["link"].setText("waiting for MibStatus")
        rtps_net.save_config({CONFIG_PEER: peer, CONFIG_ADDRESS: args.advertised_address})

    def _disconnect(self) -> None:
        if self.harness is not None:
            self.harness.stop()  # run() returns, sends a zero sample, closes the sockets
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.harness = None
        self.thread = None
        self._clear_keys()
        self.connect_button.setText("Connect")
        self.connection_label.setText("not connected")
        self.status_labels["link"].setText("not connected")
        self.stream_label.setText(f"{self.cli.xy_hz:g} Hz: idle")

    # ------------------------------------------------------------- drive

    def _apply_options(self) -> None:
        if self.harness is None:
            return
        self.harness.always_stream = self.always_stream_check.isChecked()
        self.harness.stick.hold = self.hold_check.isChecked()
        self.harness.stick.button = self.button_check.isChecked()
        if not self.harness.stick.hold and not self._dragging and not self._keys:
            self.harness.stick.set_pad(0.0, 0.0)

    def _on_profile(self) -> None:
        """The HMI sends DriveCommand on change; carry the current request with it."""
        if self.harness is None:
            return
        self.harness.profile = self.profile_group.checkedId()
        request = (spec.ENUM_VALUES["DriveRequest"]["ENABLE"] if self.harness.mib_enabled()
                   else spec.ENUM_VALUES["DriveRequest"]["DISABLE"])
        self.harness.send_drive(request)
        self._show_drive_request()

    def _drive(self, request: int) -> None:
        if self.harness is None:
            self._append_log("[gui] not connected")
            return
        self.harness.send_drive(request, self.profile_group.checkedId())
        self._show_drive_request()

    def _show_drive_request(self) -> None:
        if self.harness is None or self.harness.last_drive_request is None:
            return
        self.drive_label.setText(
            f"last sent: {spec.enum_name('DriveRequest', self.harness.last_drive_request)} "
            f"{spec.enum_name('DriveProfile', self.harness.profile)}")

    def _estop(self) -> None:
        self._clear_keys()
        self.hold_check.setChecked(False)
        self._set_slider(0.0)
        if self.harness is None:
            self._append_log("[gui] E-STOP: not connected, nothing to send")
            return
        self.harness.estop()
        self._show_drive_request()

    # ---------------------------------------------------------- joystick

    def _stick(self) -> Optional[Stick]:
        return None if self.harness is None else self.harness.stick

    def _slider_value(self) -> float:
        return self.twist_slider.value() / 1000.0

    def _set_slider(self, twist: float) -> None:
        value = int(round(twist * 1000))
        if self.twist_slider.value() != value:
            self.twist_slider.blockSignals(True)
            self.twist_slider.setValue(value)
            self.twist_slider.blockSignals(False)

    def _centre_stick(self) -> None:
        self._clear_keys()
        self._set_slider(0.0)
        stick = self._stick()
        if stick is not None:
            stick.centre()

    def _on_pad_press(self, axes: Tuple[float, float]) -> None:
        self._dragging = True
        self._on_pad_motion(axes)

    def _on_pad_motion(self, axes: Tuple[float, float]) -> None:
        stick = self._stick()
        if stick is None or not self._dragging:
            return
        stick.set_pad(*axes)

    def _on_pad_release(self) -> None:
        self._dragging = False
        stick = self._stick()
        if stick is not None and not stick.hold:
            stick.set_pad(0.0, 0.0)

    def _on_twist_press(self) -> None:
        self._twist_dragging = True

    def _on_twist_changed(self, _value: int) -> None:
        stick = self._stick()
        if stick is None:
            return
        stick.set_twist(self._slider_value())
        if not self._twist_dragging:
            # a click on the groove (page step) moves the slider without a
            # press/release pair: treat it as a tap that springs back
            QtCore.QTimer.singleShot(1, self._twist_released)

    def _on_twist_release(self) -> None:
        self._twist_released()

    def _twist_released(self) -> None:
        self._twist_dragging = False
        stick = self._stick()
        if stick is None:
            return
        stick.set_twist(self._slider_value())
        if not stick.hold:
            stick.set_twist(0.0)
            self._set_slider(0.0)

    def _on_key_press(self, key: int, _auto_repeat: bool) -> bool:
        if key == Qt.Key.Key_Space:
            self._centre_stick()
            return True
        if key not in self.KEY_AXES:
            return False
        self._keys[key] = time.monotonic()  # a repeat refreshes the stuck-key clock
        return True

    def _on_key_release(self, key: int) -> bool:
        if key not in self.KEY_AXES:
            return False
        self._keys.pop(key, None)
        return True

    def _clear_keys(self) -> None:
        self._keys.clear()

    def _joy_tick(self) -> None:
        """Integrate the held keys, spring released axes back, and redraw the pad."""
        now = time.monotonic()
        for key, pressed_at in list(self._keys.items()):
            if now - pressed_at > KEY_STUCK_S:
                self._keys.pop(key, None)  # no repeat, no release: treat as let go
        stick = self._stick()
        if stick is not None:
            targets = {"x": 0, "y": 0, "twist": 0}
            for key in self._keys:
                axis, direction = self.KEY_AXES[key]
                targets[axis] += direction
            x, y, twist = stick.x, stick.y, stick.twist
            if not self._dragging:
                x = self._approach(x, targets["x"], stick.hold)
                y = self._approach(y, targets["y"], stick.hold)
                stick.set_pad(x, y)
            if not self._twist_dragging:
                twist = self._approach(twist, targets["twist"], stick.hold)
                stick.set_twist(twist)
                if abs(self._slider_value() - stick.twist) > 1e-3:
                    self._set_slider(stick.twist)
            x, y, twist = stick.x, stick.y, stick.twist
        else:
            x = y = twist = 0.0
        self.pad.set_state(x, y, twist)
        self.live_label.setText(f"x {x:+.2f}  y {y:+.2f}  twist {twist:+.2f}")

    @staticmethod
    def _approach(value: float, target: int, hold: bool) -> float:
        dt = JOY_TICK_MS / 1000.0
        if target:
            step = KEY_RATE * dt
            return max(-1.0, min(1.0, value + target * step))
        if hold:
            return value
        step = SPRING_RATE * dt
        if abs(value) <= step:
            return 0.0
        return value - math.copysign(step, value)

    # ---------------------------------------------------------- actuators

    def _seat_allowed(self, jog: bool = False) -> bool:
        if self.harness is None:
            return False
        if self.force_seat_check.isChecked():
            return True
        age = self.harness.link_age()
        if age is None or age > spec.MIB_STATUS_TIMEOUT_MS / 1000.0:
            return False
        states = spec.ENUM_VALUES["MibSystemState"]
        allowed = {states.get("IDLE")}
        if jog:
            allowed.add(states.get("ERROR"))  # a corner outside its range: jog is the way back
        return self.harness.system_state() in allowed

    def _seat_step(self, index: int, direction: int) -> None:
        if not self._seat_allowed():
            self._append_log("[gui] seat command blocked: the MIB is not IDLE (tick force)")
            return
        if self.harness.seat_step(index, direction) is None:
            self._append_log("[gui] step refused: no fresh value from the MIB to step from")

    def _seat_set(self, index: int) -> None:
        if not self._seat_allowed():
            self._append_log("[gui] seat command blocked: the MIB is not IDLE (tick force)")
            return
        axis = spec.SEAT_AXES[index]
        text = self.seat_target_edits[index].text()
        try:
            raw = axis.parse(text)
        except ValueError:
            self._append_log(f"[gui] '{text}' is not a number")
            return
        self.harness.send_seat(index, axis.to_units(raw))
        self.seat_target_edits[index].clearFocus()

    def _seat_jog(self, index: int, delta: float) -> None:
        if not self._seat_allowed(jog=True):
            self._append_log("[gui] jog blocked: the MIB is not IDLE or ERROR (tick force)")
            return
        target = self.harness.seat_jog(index, delta)
        if target is None:
            self._append_log("[gui] jog refused: no fresh value from the MIB to jog from")
        else:
            self._append_log(f"[gui] jog {spec.SEAT_AXES[index].short} {delta:+g} -> {target:g}")

    def _seat_zero(self, index: int) -> None:
        if not self._seat_allowed():
            self._append_log("[gui] seat command blocked: the MIB is not IDLE (tick force)")
            return
        self.harness.send_seat(index, 0.0)

    # ----------------------------------------------------------------- tick

    def _append_log(self, message: str) -> None:
        self.log_text.appendPlainText(message)

    def _clear_log(self) -> None:
        self.log_text.clear()

    def _tick(self) -> None:
        verbose = self.verbose_check.isChecked()
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            if verbose or not line.startswith(QUIET_PREFIXES):
                self._append_log(line)
        while True:
            try:
                kind, payload = self.result_queue.get_nowait()
            except queue.Empty:
                break
            if kind in ("detect", "scan"):
                self._discovery_finished(kind, payload)
        if self.harness is not None:
            snapshot = self.harness.snapshot()
            self._refresh_status(snapshot)
            self._refresh_seat(snapshot)
            self._refresh_base(snapshot)
            self._refresh_diagnostics(snapshot)
            self._refresh_motors(snapshot)
            self._refresh_stream()
            self.diag_table.viewport().update()
            self.motor_table.viewport().update()

    def _refresh_status(self, snap: Dict[str, Any]) -> None:
        labels = self.status_labels
        status = snap["status"]
        if status is None:
            labels["link"].setText("waiting for MibStatus")
            _colour(labels["link"], ORANGE)
            return
        age = snap["now"] - snap["status_at"]
        lost = age > spec.MIB_STATUS_TIMEOUT_MS / 1000.0
        labels["link"].setText(f"LOST ({age:.0f}s silent)" if lost
                               else f"connected ({age * 1000:.0f} ms ago)")
        _colour(labels["link"], RED if lost else GREEN)
        state = spec.enum_name("MibSystemState", status["systemState"])
        labels["state"].setText(f"{state} ({status['systemState']})")
        _colour(labels["state"], _dim() if lost else STATE_COLOURS.get(state, "#6a1b9a"))
        labels["profile"].setText(spec.enum_name("DriveProfile", status["activeProfile"]))
        speed = float(status["speed"])
        labels["speed"].setText(f"{speed:.2f} m/s  ({speed * spec.MPH_PER_MPS:.1f} mph)")
        labels["seq"].setText(f"{status['seq']} / {snap['status_gaps']} gaps "
                              f"({snap['status_count']} rx)")
        labels["rate"].setText(f"{snap['status_rate']:.1f} Hz")
        labels["clock"].setText(format_clock(status))
        labels["status_text"].setText(status.get("status_text") or "(empty)")
        labels["error"].setText(status.get("error_message") or "(empty)")
        _colour(labels["error"], RED if state == "ERROR" else DEFAULT)
        labels["footer"].setText(status.get("error_footer") or "(empty)")
        values = spec.seat_values(status)
        self.seat_summary.setText("   ".join(
            f"{axis.label}: {format_seat(axis, values[i] if i < len(values) else None)}"
            for i, axis in enumerate(spec.SEAT_AXES)))

    def _refresh_seat(self, snap: Dict[str, Any]) -> None:
        status = snap["status"]
        values = spec.seat_values(status) if status is not None else []
        # The MIB's limits are its own (per corner, in hw_config) and are not on
        # the wire; what it does say is which corner is outside them, through
        # MibStatus.error_message ("Linkage FR out of range").
        error = (status.get("error_message") or "") if status is not None else ""
        for index, axis in enumerate(spec.SEAT_AXES):
            value = values[index] if index < len(values) else None
            text = format_seat(axis, value)
            corner = axis.name.rsplit("_", 1)[-1].upper()
            outside = (axis.name.upper().startswith("PEAR_LINK")
                       and error.startswith(f"Linkage {corner} out of range"))
            if outside:
                text += "  OUT OF RANGE (MIB)"
            label = self.seat_value_labels[index]
            if label.text() != text:
                label.setText(text)
                _colour(label, RED if outside else DEFAULT)
        age = self.harness.link_age() if self.harness is not None else None
        if age is None:
            self.seat_age_label.setText("values: no MibStatus yet")
            _colour(self.seat_age_label, ORANGE)
        else:
            fresh = age <= MibBench.SEAT_STEP_MAX_AGE_S
            self.seat_age_label.setText(f"values: {age * 1000:.0f} ms old"
                                        + ("" if fresh else " - too old to step from"))
            _colour(self.seat_age_label, _dim() if fresh else RED)
        allowed = self._seat_allowed()
        if allowed != self._seat_controls_enabled:
            self._seat_controls_enabled = allowed
            for widget in self.seat_controls:
                widget.setEnabled(allowed)
            self.seat_note.setText(
                "" if allowed else "controls greyed: the MIB is not IDLE (or no status)")

    def _refresh_base(self, snap: Dict[str, Any]) -> None:
        status = snap["status"]
        if status is None:
            self.base_view.set_state(None, {}, {}, set(), False)
            return
        values = spec.seat_values(status)
        by_name = {axis.name.upper(): (axis, values[i] if i < len(values) else None)
                   for i, axis in enumerate(spec.SEAT_AXES)}

        def value(name: str) -> Optional[float]:
            entry = by_name.get(name)
            return None if entry is None else entry[1]

        pose = None
        if all(value(n) is not None for n in ("FRONT_BACK_TILT", "LATERAL_TILT", "ELEVATION")):
            pose = {"elevation_mm": value("ELEVATION"), "fb_deg": value("FRONT_BACK_TILT"),
                    "lat_deg": value("LATERAL_TILT")}
        corners: Dict[str, Optional[float]] = {}
        ranges: Dict[str, Tuple[float, float]] = {}
        for corner in ("FR", "FL", "BR", "BL"):
            entry = by_name.get(f"PEAR_LINK_{corner}")
            if entry is None:
                continue
            axis, v = entry
            corners[corner] = v
            ranges[corner] = (axis.to_units(axis.min_value), axis.to_units(axis.max_value))
        error = status.get("error_message") or ""
        out = {c for c in corners if error.startswith(f"Linkage {c} out of range")}
        fresh = (snap["now"] - snap["status_at"]) <= spec.MIB_STATUS_TIMEOUT_MS / 1000.0
        self.base_view.set_state(pose, corners, ranges, out, fresh)

    def _refresh_diagnostics(self, snap: Dict[str, Any]) -> None:
        diag = snap["diag"]
        if diag is None:
            return
        age = snap["now"] - snap["diag_at"]
        stale = age > spec.DIAG_TIMEOUT_MS / 1000.0
        colour = RED if stale else DEFAULT
        items = diag["items"]
        for row, item in enumerate(spec.DIAGNOSTICS):
            values = items[item.id]["values"] if item.id < len(items) else []
            readings = format_diag_item(item, values)
            self._set_row(self.diag_table, row,
                          (item.short, item.label, *readings, format_age(age),
                           f"{snap['diag_rate']:.1f} Hz"), colour)
        extra = (f"; {len(items)} rows on the wire, table has {len(spec.DIAGNOSTICS)}"
                 if len(items) != len(spec.DIAGNOSTICS) else "")
        self.diag_summary.setText(
            f"seq {diag['seq']}, {snap['diag_gaps']} gaps, {snap['diag_count']} rx, "
            f"{snap['diag_rate']:.1f} Hz" + (" - STALE" if stale else "") + extra)
        _colour(self.diag_summary, RED if stale else _dim())

    def _refresh_motors(self, snap: Dict[str, Any]) -> None:
        for axis in spec.AXES:
            state, at, _gaps, rate = snap["motors"][axis.id]
            if state is None:
                continue
            age = snap["now"] - at
            stale = age > MOTOR_STALE_S
            board = spec.enum_name("BoardState", state["state"])
            colour = RED if stale else (ORANGE if board == "FAULT" else DEFAULT)
            position = float(state["position"])
            velocity = float(state["velocity"])
            self._set_row(self.motor_table, self.motor_rows[axis.id], (
                axis.label, state["api_version"], board,
                spec.enum_name("ControlMode", state["mode"]),
                spec.enum_name("FaultCode", state["fault_code"]),
                f"{position:.3f} ({math.degrees(position):.1f})",
                f"{velocity:.3f} ({velocity * spec.RPM_PER_RAD_S:.1f})",
                f"{state['torque_est']:.3f}", f"{state['iq']:.2f}", f"{state['vbus']:.1f}",
                " ".join(f"{t:.0f}" for t in state["temps"]),
                f"{state['cmd_age_ms']} ms", f"{state['uptime_ms'] / 1000:.0f} s",
                f"{format_age(age)} {rate:.0f}Hz"), colour)

    def _refresh_stream(self) -> None:
        harness = self.harness
        if harness is None:
            return
        if harness.streaming_wanted():
            why = "always" if harness.always_stream else "MIB ENABLED"
            text = (f"{harness.xy_hz:g} Hz: streaming ({why}), {harness.xy_sent} sent to "
                    f"{max(0, harness.xy_targets)} target(s)")
        else:
            text = f"{harness.xy_hz:g} Hz: idle (MIB not ENABLED), {harness.xy_sent} sent"
        self.stream_label.setText(text)
        self._show_drive_request()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802 - Qt API
        self._disconnect()
        rtps_host.LOG_SINK = None
        super().closeEvent(event)


def run(cli: argparse.Namespace) -> int:
    """Open the window and run the Qt event loop until it is closed."""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    _disable_app_nap()
    window = MibWindow(cli)
    window.show()
    return app.exec()
