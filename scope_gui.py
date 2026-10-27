#!/usr/bin/env python3
"""Nano Scope GUI: laptop front end for nano_scope.ino (v2)

Run:      python scope_gui.py                 (pick the port in the window)
          python scope_gui.py --port /dev/ttyUSB0
          python scope_gui.py --demo          (simulated Nano, no hardware needed)

Deps (Arch):  sudo pacman -S python-pyqt6 python-pyqtgraph python-pyserial python-numpy

Mouse:  wheel on the plot = time/div, wheel on the voltage axis = V/div,
        drag the dashed horizontal line = trigger level.
Keys:   Space run/stop, S single, 1-6 toggle channels, Left/Right time/div,
        Up/Down V/div.
"""
import signal
import argparse
import csv
import re
import struct
import sys
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports
from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

signal.signal(signal.SIGINT, signal.SIG_DFL)

# ----------------------------------------------------------------------------
# constants (must match nano_scope.ino)
# ----------------------------------------------------------------------------
NCH = 6
BUFSZ = 1020                      # shared sample buffer in the Nano
CMD_LEN = 14
PS_TICK_US = (0.5, 4.0, 16.0, 64.0)   # Timer1 tick length for the 4 prescalers
FRAME_HDR = struct.Struct("<BBBHHHH")  # mask flags ps ticks vcc n trig
FLAG_TRIG, FLAG_OVERRUN, FLAG_FORCED = 1, 2, 4

MIN_N = 32                        # fewest samples per channel we accept
T_PER_CH_US = 17.0                # estimated time to read one channel
T_BASE_US = 8.0                   # estimated per-sample-set overhead
TDIV_MAX = 10.0

COLORS = ["#ffd400", "#00e5ff", "#ff4df0", "#7cff4d", "#ff9a3c", "#e8e8e8"]


def _ladder(lo, hi):
    vals = sorted({round(m * 10.0 ** e, 9) for e in range(-7, 3) for m in (1, 2, 5)})
    return [v for v in vals if lo * 0.999 <= v <= hi * 1.001]


TDIV_LADDER = _ladder(100e-6, TDIV_MAX)
VDIV_LADDER = _ladder(0.02, 50.0)


def ladder_step(value, ladder, direction):
    if direction > 0:
        for x in ladder:
            if x > value * 1.0001:
                return x
        return ladder[-1]
    for x in reversed(ladder):
        if x < value * 0.9999:
            return x
    return ladder[0]


# ----------------------------------------------------------------------------
# number parsing / formatting
# ----------------------------------------------------------------------------
_TIME_UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "µs": 1e-6, "μs": 1e-6}
_VOLT_UNITS = {"v": 1.0, "mv": 1e-3}


def parse_value(text, units, default_unit):
    """'2.5ms', '250 us', '1.5' -> seconds/volts. Raises ValueError."""
    t = text.strip().lower().replace(" ", "").replace("/div", "")
    m = re.fullmatch(r"([0-9]*\.?[0-9]+(?:e[-+]?[0-9]+)?)([a-zµμ]*)", t)
    if not m:
        raise ValueError(text)
    num, unit = float(m.group(1)), m.group(2)
    if not unit:
        unit = default_unit
    if unit not in units:
        raise ValueError(text)
    return num * units[unit]


_PREFIXES = ((1e9, "G"), (1e6, "M"), (1e3, "k"), (1.0, ""), (1e-3, "m"), (1e-6, "µ"), (1e-9, "n"))


def fmt_si(x, unit, digits=4):
    if x is None or not np.isfinite(x):
        return "—"
    if x == 0:
        return f"0 {unit}"
    ax = abs(x)
    for scale, prefix in _PREFIXES:
        if ax >= scale * 0.99995:
            return f"{x / scale:.{digits}g} {prefix}{unit}"
    return f"{x / 1e-9:.{digits}g} n{unit}"


# ----------------------------------------------------------------------------
# measurements
# ----------------------------------------------------------------------------
def crossings(v, level, rising):
    """Fractional sample indices where v crosses level."""
    s = v >= level
    idx = np.flatnonzero(~s[:-1] & s[1:]) if rising else np.flatnonzero(s[:-1] & ~s[1:])
    if idx.size == 0:
        return idx.astype(float)
    d = v[idx + 1] - v[idx]
    d = np.where(d == 0, 1.0, d)
    return idx + (level - v[idx]) / d


def schmitt_edges(v, vmin, vpp):
    """Rising / falling edge positions (in samples), noise-proof via hysteresis."""
    mid, lo, hi = vmin + 0.5 * vpp, vmin + 0.3 * vpp, vmin + 0.7 * vpp
    ev = sorted([(float(x), 1) for x in crossings(v, mid, True)] +
                [(float(x), -1) for x in crossings(v, mid, False)])
    good = []
    for x, typ in ev:
        if not good:
            good.append((x, typ))
            continue
        px, ptyp = good[-1]
        if typ == ptyp:
            continue
        seg = v[int(px): int(x) + 2]
        if (typ == -1 and seg.max() >= hi) or (typ == 1 and seg.min() <= lo):
            good.append((x, typ))
    return [x for x, t in good if t == 1], [x for x, t in good if t == -1]


def measure(v, dt, lsb):
    """Scope style measurements on one channel. dt in seconds."""
    m = {"Vmax": float(v.max()), "Vmin": float(v.min())}
    m["Vpp"] = m["Vmax"] - m["Vmin"]
    m["Vavg"] = float(v.mean())
    m["Vrms"] = float(np.sqrt(np.mean(v * v)))
    vpp = m["Vpp"]
    if vpp < 10 * lsb:                      # flat line / noise only (< ~200 mV p-p)
        return m
    vmin = m["Vmin"]
    up, dn = schmitt_edges(v, vmin, vpp)
    for edges in (up, dn):
        if len(edges) >= 2:
            per = (edges[-1] - edges[0]) / (len(edges) - 1) * dt
            if per >= 4 * dt:               # fewer than 4 samples/cycle: not reported
                m["Period"] = per
                m["Freq"] = 1.0 / per
                m["approx"] = per < 10 * dt  # under 10 samples/cycle: treat with care
            break
    if len(up) >= 2:
        highs = []
        for r0, r1 in zip(up[:-1], up[1:]):
            f = [x for x in dn if r0 < x < r1]
            if f:
                highs.append((f[0] - r0) / (r1 - r0))
        if highs:
            m["Duty"] = float(np.mean(highs))
    lo10, hi90 = vmin + 0.1 * vpp, vmin + 0.9 * vpp
    c10, c90 = crossings(v, lo10, True), crossings(v, hi90, True)
    if c10.size:
        later = c90[c90 > c10[0]]
        if later.size:
            w = later[0] - c10[0]
            m["Rise"] = (max(w, 1.0) * dt, w < 1.0)
    c90f, c10f = crossings(v, hi90, False), crossings(v, lo10, False)
    if c90f.size:
        later = c10f[c10f > c90f[0]]
        if later.size:
            w = later[0] - c90f[0]
            m["Fall"] = (max(w, 1.0) * dt, w < 1.0)
    return m


# ----------------------------------------------------------------------------
# serial protocol
# ----------------------------------------------------------------------------
def build_cmd(mask, ps, ticks, n, pre, trigch, edge, level, mode):
    body = bytes([mask & 0x3F, ps, ticks & 255, ticks >> 8, n & 255, n >> 8,
                  pre & 255, pre >> 8, trigch, edge, level, mode])
    chk = 0
    for b in body:
        chk ^= b
    return bytes([0xA5]) + body + bytes([chk])


class SerialWorker(QtCore.QThread):
    frame_ready = QtCore.pyqtSignal()
    failed = QtCore.pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self._stop = False
        self._lock = threading.Lock()
        self._latest = None
        self.good = 0
        self.bad = 0

    def stop(self):
        self._stop = True

    def take(self):
        with self._lock:
            f, self._latest = self._latest, None
        return f

    def run(self):
        buf = bytearray()
        try:
            while not self._stop:
                chunk = self.ser.read(max(1, self.ser.in_waiting))
                if chunk:
                    buf += chunk
                    self._parse(buf)
        except (serial.SerialException, OSError) as e:
            self.failed.emit(str(e))

    def _parse(self, buf):
        hs = FRAME_HDR.size
        while True:
            i = buf.find(b"\xaa\x55")
            if i < 0:
                if len(buf) > 1:
                    del buf[:-1]
                return
            if i > 0:
                del buf[:i]
            if len(buf) < 2 + hs:
                return
            mask, flags, ps, ticks, vcc, n, trig = FRAME_HDR.unpack_from(bytes(buf[:2 + hs]), 2)
            nch = bin(mask & 0x3F).count("1")
            if nch == 0 or n < 1 or n * nch > BUFSZ or ps > 3 or ticks == 0:
                del buf[:2]
                self.bad += 1
                continue
            total = 2 + hs + n * nch + 1
            if len(buf) < total:
                return
            body = bytes(buf[2:total - 1])
            chk = int(np.bitwise_xor.reduce(np.frombuffer(body, dtype=np.uint8)))
            if chk != buf[total - 1]:
                del buf[:2]
                self.bad += 1
                continue
            data = np.frombuffer(body[hs:], dtype=np.uint8).reshape(n, nch).copy()
            del buf[:total]
            frame = dict(mask=mask & 0x3F, flags=flags, ps=ps, ticks=ticks,
                         vcc=vcc / 1000.0, n=n, trig=trig, data=data,
                         dt=ticks * PS_TICK_US[ps] * 1e-6, t=time.time())
            self.good += 1
            with self._lock:
                self._latest = frame
            self.frame_ready.emit()


# ----------------------------------------------------------------------------
# demo device (--demo): speaks the same protocol with synthetic signals
# ----------------------------------------------------------------------------
class FakeSerial:
    def __init__(self):
        self.cfg = dict(mask=0, ps=1, ticks=1000, n=128, pre=64, trigch=0,
                        edge=0, level=128, mode=0)
        self.out = bytearray()
        self.cmdbuf = bytearray()
        self.lock = threading.Lock()
        self.next_t = 0.0
        self.rng = np.random.default_rng(1)
        self.offs = self.rng.uniform(0, 0.01, NCH)
        self.is_open = True

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.out)

    def write(self, data):
        with self.lock:
            self.cmdbuf += data
            while True:
                i = self.cmdbuf.find(b"\xa5")
                if i < 0:
                    self.cmdbuf.clear()
                    return
                del self.cmdbuf[:i]
                if len(self.cmdbuf) < CMD_LEN:
                    return
                p = bytes(self.cmdbuf[:CMD_LEN])
                del self.cmdbuf[:CMD_LEN]
                x = 0
                for b in p[1:-1]:
                    x ^= b
                if x != p[-1]:
                    continue
                self.cfg = dict(mask=p[1] & 63, ps=p[2] & 3, ticks=p[3] | p[4] << 8,
                                n=p[5] | p[6] << 8, pre=p[7] | p[8] << 8, trigch=p[9],
                                edge=p[10], level=p[11], mode=p[12])
                self.next_t = time.time() + 0.05

    @staticmethod
    def _signal(ch, t):
        if ch == 0:   # 100 Hz square, 0-5 V, 50 %
            return np.where((t * 100) % 1 < 0.5, 5.0, 0.0)
        if ch == 1:   # 40 Hz sine
            return 2.5 + 2.0 * np.sin(2 * np.pi * 40 * t)
        if ch == 2:   # 20 Hz triangle
            return 2.5 + 2.5 * (2 / np.pi) * np.arcsin(np.sin(2 * np.pi * 20 * t))
        if ch == 3:   # DC
            return np.full_like(t, 3.3)
        if ch == 4:   # 250 Hz square, 0-3.3 V, 30 %
            return np.where((t * 250) % 1 < 0.3, 3.3, 0.0)
        return 5.0 * ((t * 10) % 1)   # 10 Hz saw

    def _maybe_frame(self):
        with self.lock:
            c = dict(self.cfg)
        if c["mode"] == 0 or c["mask"] == 0 or time.time() < self.next_t:
            return
        chans = [i for i in range(NCH) if c["mask"] >> i & 1]
        nch = len(chans)
        n = max(8, min(c["n"], BUFSZ // nch))
        pre = min(c["pre"], n - 1)
        dt = c["ticks"] * PS_TICK_US[c["ps"]] * 1e-6
        t = (np.arange(n) - pre) * dt
        data = np.zeros((n, nch), np.uint8)
        for k, ch in enumerate(chans):
            tt = t if ch == c["trigch"] else t + self.offs[ch]
            v = self._signal(ch, tt) + self.rng.normal(0, 0.012, n)
            data[:, k] = np.clip(v / 5.0 * 256.0, 0, 255).astype(np.uint8)
        hdr = FRAME_HDR.pack(c["mask"], FLAG_TRIG, c["ps"], c["ticks"], 4980, n, pre)
        body = hdr + data.tobytes()
        chk = int(np.bitwise_xor.reduce(np.frombuffer(body, np.uint8)))
        with self.lock:
            self.out += b"\xaa\x55" + body + bytes([chk])
            if c["mode"] == 3:
                self.cfg["mode"] = 0
        self.next_t = time.time() + max(0.04, min(n * dt, 0.5))

    def read(self, size=1):
        end = time.time() + 0.05
        while time.time() < end:
            self._maybe_frame()
            if self.in_waiting:
                break
            time.sleep(0.004)
        with self.lock:
            k = min(size, len(self.out))
            data = bytes(self.out[:k])
            del self.out[:k]
        return data

    def close(self):
        self.is_open = False


# ----------------------------------------------------------------------------
# widgets
# ----------------------------------------------------------------------------
class ScopeViewBox(pg.ViewBox):
    wheel_x = QtCore.pyqtSignal(int)
    wheel_y = QtCore.pyqtSignal(int)

    def __init__(self):
        super().__init__(enableMenu=False)
        self.setMouseEnabled(x=False, y=False)

    def wheelEvent(self, ev, axis=None):
        step = 1 if ev.delta() > 0 else -1
        (self.wheel_y if axis == 1 else self.wheel_x).emit(step)
        ev.accept()

    def mouseDragEvent(self, ev, axis=None):
        ev.ignore()


def mk_btn(text, slot=None, checkable=False):
    b = QtWidgets.QPushButton(text)
    b.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    b.setCheckable(checkable)
    if slot:
        (b.toggled if checkable else b.clicked).connect(slot)
    return b


def mk_spin(lo, hi, val, step, decimals=3):
    s = QtWidgets.QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    s.setValue(val)
    s.setKeyboardTracking(False)
    return s


def dark_palette():
    p = QtGui.QPalette()
    R = QtGui.QPalette.ColorRole
    for role, col in ((R.Window, "#1e1f24"), (R.WindowText, "#dddddd"), (R.Base, "#15161a"),
                      (R.AlternateBase, "#1e1f24"), (R.Text, "#dddddd"), (R.Button, "#2b2d33"),
                      (R.ButtonText, "#dddddd"), (R.Highlight, "#3d7be0"),
                      (R.HighlightedText, "#ffffff"), (R.ToolTipBase, "#2b2d33"),
                      (R.ToolTipText, "#dddddd")):
        p.setColor(role, QtGui.QColor(col))
    return p


MEAS_ROWS = ["Vmax", "Vmin", "Vpp", "Vavg", "Vrms", "Freq", "Period", "Duty", "Rise", "Fall"]


def cell_text(key, m):
    x = m.get(key)
    if x is None:
        return "—"
    if key in ("Vmax", "Vmin", "Vpp", "Vavg", "Vrms"):
        return fmt_si(x, "V")
    if key == "Freq":
        return ("~ " if m.get("approx") else "") + fmt_si(x, "Hz")
    if key == "Period":
        return ("~ " if m.get("approx") else "") + fmt_si(x, "s")
    if key == "Duty":
        return f"{x * 100:.1f} %"
    sec, bound = x
    return ("< " if bound else "") + fmt_si(sec, "s", 3)


# ----------------------------------------------------------------------------
# main window
# ----------------------------------------------------------------------------
class Scope(QtWidgets.QMainWindow):
    def __init__(self, demo=False, port=None, baud=500000):
        super().__init__()
        self.setWindowTitle("Nano Scope")
        self.resize(1450, 880)

        self.mask = 0b000011
        self.tdiv = 5e-3
        self.vdiv = 1.0
        self.vcenter = 2.5
        self.trig_ch = 0
        self.edge = 0
        self.level_v = 2.5
        self.trig_mode = "auto"
        self.running = True
        self.single = False
        self.pre_pct = 50
        self.scale = [1.0] * NCH
        self.offset = [0.0] * NCH
        self.vcc = 5.0
        self.tmin_factor = 1.0
        self.last = None
        self.worker = None
        self.last_cfg_time = 0.0
        self.last_frame_time = 0.0

        self.build_ui()
        self.refresh_ports()
        self.set_tdiv(self.tdiv)
        self.update_axes()

        self.watch = QtCore.QTimer(self)
        self.watch.timeout.connect(self.watchdog)
        self.watch.start(500)

        if demo:
            self.cb_port.setEditText("demo")
            self.connect_serial(FakeSerial())
        elif port:
            self.cb_port.setEditText(port)
            self.cb_baud.setCurrentText(str(baud))
            self.toggle_connection()
        else:
            self.cb_baud.setCurrentText(str(baud))

    # ---------------------------------------------------------------- UI
    def build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        left = QtWidgets.QVBoxLayout()
        root.addLayout(left, 1)

        bar = QtWidgets.QHBoxLayout()
        self.lbl_state = QtWidgets.QLabel("DISCONNECTED")
        f = self.lbl_state.font()
        f.setBold(True)
        f.setPointSize(f.pointSize() + 2)
        self.lbl_state.setFont(f)
        self.lbl_info = QtWidgets.QLabel("")
        bar.addWidget(self.lbl_state)
        bar.addSpacing(16)
        bar.addWidget(self.lbl_info, 1)
        left.addLayout(bar)

        pg.setConfigOptions(antialias=True, background="#101014", foreground="#cccccc")
        self.vb = ScopeViewBox()
        self.plotw = pg.PlotWidget(viewBox=self.vb)
        self.pi = self.plotw.getPlotItem()
        self.pi.setMenuEnabled(False)
        self.pi.hideButtons()
        self.pi.showGrid(x=True, y=True, alpha=0.35)
        self.pi.setLabel("bottom", "Time", units="s")
        self.pi.setLabel("left", "Voltage", units="V")
        self.pi.disableAutoRange()
        self.vb.wheel_x.connect(lambda s: self.step_tdiv(-s))
        self.vb.wheel_y.connect(lambda s: self.step_vdiv(-s))
        self.curves = [self.pi.plot(pen=pg.mkPen(COLORS[c], width=1.6)) for c in range(NCH)]
        dash = QtCore.Qt.PenStyle.DashLine
        self.ln_trig_t = pg.InfiniteLine(pos=0, angle=90, movable=False,
                                         pen=pg.mkPen("#777777", style=dash))
        self.ln_trig_v = pg.InfiniteLine(pos=self.level_v, angle=0, movable=True,
                                         pen=pg.mkPen(COLORS[0], width=1.5, style=dash),
                                         hoverPen=pg.mkPen("#ffffff", width=2))
        self.ln_trig_v.sigPositionChangeFinished.connect(
            lambda ln: self.set_level(float(ln.value())))
        self.pi.addItem(self.ln_trig_t, ignoreBounds=True)
        self.pi.addItem(self.ln_trig_v, ignoreBounds=True)
        self.cur_t = [pg.InfiniteLine(pos=p, angle=90, movable=True,
                                      pen=pg.mkPen((170, 200, 255), width=1.3,
                                                   style=QtCore.Qt.PenStyle.DotLine)) for p in (-0.2, 0.2)]
        self.cur_v = [pg.InfiniteLine(pos=p, angle=0, movable=True,
                                      pen=pg.mkPen((255, 190, 170), width=1.3,
                                                   style=QtCore.Qt.PenStyle.DotLine)) for p in (1.0, 4.0)]
        for ln in self.cur_t + self.cur_v:
            ln.sigPositionChanged.connect(self.update_cursor_readout)
            ln.setVisible(False)
            self.pi.addItem(ln, ignoreBounds=True)
        left.addWidget(self.plotw, 1)

        self.lbl_cur = QtWidgets.QLabel("")
        left.addWidget(self.lbl_cur)

        self.tbl = QtWidgets.QTableWidget(len(MEAS_ROWS), NCH)
        self.tbl.setVerticalHeaderLabels(MEAS_ROWS)
        self.tbl.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.tbl.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.NoSelection)
        self.tbl.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.tbl.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.tbl.verticalHeader().setDefaultSectionSize(21)
        self.tbl.setFixedHeight(21 * (len(MEAS_ROWS) + 1) + 8)
        for c in range(NCH):
            h = QtWidgets.QTableWidgetItem(f"CH{c + 1}")
            h.setForeground(QtGui.QBrush(QtGui.QColor(COLORS[c])))
            self.tbl.setHorizontalHeaderItem(c, h)
            for r in range(len(MEAS_ROWS)):
                it = QtWidgets.QTableWidgetItem("—")
                it.setForeground(QtGui.QBrush(QtGui.QColor(COLORS[c])))
                it.setTextAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                self.tbl.setItem(r, c, it)
        left.addWidget(self.tbl)

        # ---- right side controls
        panel = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(panel)

        g = QtWidgets.QGroupBox("Connection")
        gl = QtWidgets.QGridLayout(g)
        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setEditable(True)
        self.cb_baud = QtWidgets.QComboBox()
        self.cb_baud.setEditable(True)
        self.cb_baud.addItems(["115200", "250000", "500000", "1000000"])
        self.cb_baud.setCurrentText("500000")
        self.btn_refresh = mk_btn("↻", self.refresh_ports)
        self.btn_refresh.setFixedWidth(34)
        self.btn_connect = mk_btn("Connect", self.toggle_connection)
        gl.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        gl.addWidget(self.cb_port, 0, 1)
        gl.addWidget(self.btn_refresh, 0, 2)
        gl.addWidget(QtWidgets.QLabel("Baud"), 1, 0)
        gl.addWidget(self.cb_baud, 1, 1)
        gl.addWidget(self.btn_connect, 1, 2)
        pl.addWidget(g)

        g = QtWidgets.QGroupBox("Acquisition")
        gl = QtWidgets.QGridLayout(g)
        self.btn_run = mk_btn("Stop", self.toggle_run)
        self.btn_single = mk_btn("Single", self.do_single)
        self.cb_mode = QtWidgets.QComboBox()
        self.cb_mode.addItems(["Auto", "Normal"])
        self.cb_mode.currentIndexChanged.connect(self.on_trig_mode)
        gl.addWidget(self.btn_run, 0, 0)
        gl.addWidget(self.btn_single, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Trigger mode"), 1, 0)
        gl.addWidget(self.cb_mode, 1, 1)
        pl.addWidget(g)

        g = QtWidgets.QGroupBox("Horizontal (time)")
        gl = QtWidgets.QGridLayout(g)
        self.ed_tdiv = QtWidgets.QLineEdit()
        self.ed_tdiv.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.ed_tdiv.editingFinished.connect(self.on_tdiv_edit)
        b1 = mk_btn("◀", lambda: self.step_tdiv(-1))
        b2 = mk_btn("▶", lambda: self.step_tdiv(1))
        b1.setToolTip("Zoom in (smaller time/div)")
        b2.setToolTip("Zoom out (larger time/div)")
        gl.addWidget(QtWidgets.QLabel("Time/div"), 0, 0)
        gl.addWidget(b1, 0, 1)
        gl.addWidget(self.ed_tdiv, 0, 2)
        gl.addWidget(b2, 0, 3)
        self.lbl_plan = QtWidgets.QLabel("")
        self.lbl_plan.setWordWrap(True)
        gl.addWidget(self.lbl_plan, 1, 0, 1, 4)
        self.sl_pre = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.sl_pre.setRange(5, 95)
        self.sl_pre.setValue(self.pre_pct)
        self.sl_pre.valueChanged.connect(self.on_pre)
        self.lbl_pre = QtWidgets.QLabel("Trigger position 50 %")
        gl.addWidget(self.lbl_pre, 2, 0, 1, 4)
        gl.addWidget(self.sl_pre, 3, 0, 1, 4)
        pl.addWidget(g)

        g = QtWidgets.QGroupBox("Vertical (voltage)")
        gl = QtWidgets.QGridLayout(g)
        self.ed_vdiv = QtWidgets.QLineEdit()
        self.ed_vdiv.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.ed_vdiv.editingFinished.connect(self.on_vdiv_edit)
        b1 = mk_btn("◀", lambda: self.step_vdiv(-1))
        b2 = mk_btn("▶", lambda: self.step_vdiv(1))
        self.sp_center = mk_spin(-500, 500, self.vcenter, 0.5, 2)
        self.sp_center.valueChanged.connect(self.on_center)
        gl.addWidget(QtWidgets.QLabel("Volts/div"), 0, 0)
        gl.addWidget(b1, 0, 1)
        gl.addWidget(self.ed_vdiv, 0, 2)
        gl.addWidget(b2, 0, 3)
        gl.addWidget(QtWidgets.QLabel("Center [V]"), 1, 0)
        gl.addWidget(self.sp_center, 1, 2)
        gl.addWidget(mk_btn("Auto", self.auto_y), 1, 3)
        pl.addWidget(g)

        g = QtWidgets.QGroupBox("Channels")
        gl = QtWidgets.QGridLayout(g)
        gl.addWidget(QtWidgets.QLabel("on / off"), 0, 0)
        gl.addWidget(QtWidgets.QLabel("Probe ×"), 0, 1)
        gl.addWidget(QtWidgets.QLabel("Offset [V]"), 0, 2)
        self.btn_ch, self.sp_scale, self.sp_off = [], [], []
        for c in range(NCH):
            col = COLORS[c]
            b = mk_btn(f"CH{c + 1}  A{c}", checkable=True)
            b.setStyleSheet(
                f"QPushButton{{background:#2a2a2e;color:{col};border:1px solid {col};padding:5px;}}"
                f"QPushButton:checked{{background:{col};color:#000000;font-weight:bold;}}")
            b.setChecked(bool(self.mask >> c & 1))
            b.toggled.connect(lambda on, c=c: self.on_channel(c, on))
            sc = mk_spin(0.01, 1000, 1.0, 0.5, 3)
            sc.valueChanged.connect(lambda v, c=c: self.on_scale(c, v))
            of = mk_spin(-200, 200, 0.0, 0.1, 3)
            of.valueChanged.connect(lambda v, c=c: self.on_offset(c, v))
            gl.addWidget(b, c + 1, 0)
            gl.addWidget(sc, c + 1, 1)
            gl.addWidget(of, c + 1, 2)
            self.btn_ch.append(b)
            self.sp_scale.append(sc)
            self.sp_off.append(of)
        pl.addWidget(g)

        g = QtWidgets.QGroupBox("Trigger")
        gl = QtWidgets.QGridLayout(g)
        self.cb_src = QtWidgets.QComboBox()
        self.cb_src.addItems([f"CH{c + 1}" for c in range(NCH)])
        self.cb_src.currentIndexChanged.connect(self.on_trig_src)
        self.cb_edge = QtWidgets.QComboBox()
        self.cb_edge.addItems(["Rising", "Falling"])
        self.cb_edge.currentIndexChanged.connect(self.on_edge)
        self.sp_level = mk_spin(-1000, 1000, self.level_v, 0.1, 3)
        self.sp_level.valueChanged.connect(self.set_level)
        gl.addWidget(QtWidgets.QLabel("Source"), 0, 0)
        gl.addWidget(self.cb_src, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Edge"), 1, 0)
        gl.addWidget(self.cb_edge, 1, 1)
        gl.addWidget(QtWidgets.QLabel("Level [V]"), 2, 0)
        gl.addWidget(self.sp_level, 2, 1)
        pl.addWidget(g)

        g = QtWidgets.QGroupBox("Cursors / export")
        gl = QtWidgets.QGridLayout(g)
        self.chk_cur = QtWidgets.QCheckBox("Show cursors")
        self.chk_cur.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.chk_cur.toggled.connect(self.on_cursors)
        gl.addWidget(self.chk_cur, 0, 0, 1, 2)
        gl.addWidget(mk_btn("Save CSV", self.export_csv), 1, 0)
        gl.addWidget(mk_btn("Screenshot", self.export_png), 1, 1)
        pl.addWidget(g)

        tip = QtWidgets.QLabel(
            "Wheel on plot: time/div. Wheel on the voltage axis: V/div. "
            "Drag the dashed line: trigger level. Keys: Space, S, 1-6, arrows.\n"
            "Inputs are 0-5 V only. Use Probe × and Offset for dividers or biased inputs.")
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#8a8a92;")
        pl.addWidget(tip)
        pl.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(375)
        root.addWidget(scroll)

        def sc_(key, fn):
            s = QtGui.QShortcut(QtGui.QKeySequence(key), self)
            s.activated.connect(fn)
        sc_("Space", self.toggle_run)
        sc_("S", self.do_single)
        for i in range(NCH):
            sc_(str(i + 1), lambda i=i: self.btn_ch[i].toggle())
        sc_("Left", lambda: self.step_tdiv(-1))
        sc_("Right", lambda: self.step_tdiv(1))
        sc_("Up", lambda: self.step_vdiv(-1))
        sc_("Down", lambda: self.step_vdiv(1))

        self.ed_vdiv.setText(fmt_si(self.vdiv, "V"))

    # ---------------------------------------------------------------- planning
    def nch(self):
        return max(1, bin(self.mask).count("1"))

    def tmin_us(self, nch):
        return (nch * T_PER_CH_US + T_BASE_US) * self.tmin_factor

    def tdiv_min(self, nch):
        return MIN_N * self.tmin_us(nch) * 1e-6 / 10.0

    def plan(self):
        nch = self.nch()
        nmax = BUFSZ // nch
        window_us = 10.0 * self.tdiv * 1e6
        dt_us = max(window_us / nmax, self.tmin_us(nch))
        n = max(MIN_N, min(int(round(window_us / dt_us)), nmax))
        ps, ticks = 3, 65535
        for i, tick in enumerate(PS_TICK_US):
            k = int(round(dt_us / tick))
            if 2 <= k <= 65535:
                ps, ticks = i, k
                break
        dt_real = ticks * PS_TICK_US[ps]
        pre = max(0, min(int(round(n * self.pre_pct / 100.0)), n - 1))
        return dict(nch=nch, n=n, ps=ps, ticks=ticks, pre=pre,
                    dt=dt_real * 1e-6, window=n * dt_real * 1e-6)

    def level_code(self):
        c = self.trig_ch
        raw = self.level_v / self.scale[c] + self.offset[c]
        return int(max(0, min(255, round(raw / self.vcc * 256.0))))

    def mode_byte(self):
        if not self.running:
            return 0
        if self.single:
            return 3
        return 1 if self.trig_mode == "auto" else 2

    def send_config(self):
        self.last_cfg_time = time.time()
        if not self.worker:
            return
        pl = self.plan()
        pkt = build_cmd(self.mask, pl["ps"], pl["ticks"], pl["n"], pl["pre"],
                        self.trig_ch, self.edge, self.level_code(), self.mode_byte())
        try:
            self.worker.ser.write(pkt)
        except (serial.SerialException, OSError) as e:
            self.on_failed(str(e))
            return
        if self.running:
            self.set_state("WAIT" if self.trig_mode == "normal" or self.single else "AUTO")
        else:
            self.set_state("STOP")

    # ---------------------------------------------------------------- connection
    def refresh_ports(self):
        cur = self.cb_port.currentText()
        self.cb_port.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        ports.sort(key=lambda p: 0 if ("USB" in p.device or "ACM" in p.device) else 1)
        for p in ports:
            self.cb_port.addItem(f"{p.device}  ({p.description})")
        if cur and not cur.startswith("demo"):
            self.cb_port.setEditText(cur)

    def toggle_connection(self):
        if self.worker:
            self.disconnect_serial()
            return
        port = self.cb_port.currentText().split()[0] if self.cb_port.currentText().strip() else ""
        try:
            baud = int(self.cb_baud.currentText())
            ser = serial.Serial(port, baud, timeout=0.05)
        except (serial.SerialException, OSError, ValueError, IndexError) as e:
            QtWidgets.QMessageBox.critical(self, "Cannot open port", str(e))
            return
        self.connect_serial(ser)

    def connect_serial(self, ser):
        self.tmin_factor = 1.0
        self.worker = SerialWorker(ser)
        self.worker.frame_ready.connect(self.on_frame)
        self.worker.failed.connect(self.on_failed)
        self.worker.start()
        self.btn_connect.setText("Disconnect")
        self.set_state("CONNECTING")
        wait = 50 if isinstance(ser, FakeSerial) else 2200   # a real Nano resets on open
        QtCore.QTimer.singleShot(wait, self.send_config)

    def disconnect_serial(self):
        w, self.worker = self.worker, None
        if w:
            w.stop()
            w.wait(1500)
            try:
                w.ser.close()
            except Exception:
                pass
        self.btn_connect.setText("Connect")
        self.set_state("DISCONNECTED")

    def on_failed(self, msg):
        self.disconnect_serial()
        self.statusBar().showMessage(f"Serial error: {msg}", 8000)

    def closeEvent(self, ev):
        self.disconnect_serial()
        super().closeEvent(ev)

    # ---------------------------------------------------------------- controls
    def notify(self, text, ms=6000):
        self.statusBar().showMessage(text, ms)

    def set_state(self, text):
        self.lbl_state.setText(text)
        colour = {"STOP": "#ff6b6b", "TRIG'D": "#7cff4d", "AUTO": "#ffd400",
                  "WAIT": "#ffd400"}.get(text.split()[0], "#cccccc")
        self.lbl_state.setStyleSheet(f"color:{colour};")

    def toggle_run(self):
        self.running = not self.running
        self.single = False
        self.btn_run.setText("Stop" if self.running else "Run")
        self.send_config()

    def do_single(self):
        self.running = True
        self.single = True
        self.btn_run.setText("Stop")
        self.send_config()

    def on_trig_mode(self, i):
        self.trig_mode = "auto" if i == 0 else "normal"
        self.send_config()

    def set_tdiv(self, value):
        lo = self.tdiv_min(self.nch())
        v = min(max(value, lo), TDIV_MAX)
        if abs(v - value) > 1e-12 and value > 0:
            self.notify(f"Time/div limited to {fmt_si(v, 's')} "
                        f"(Nano sampling speed with {self.nch()} channel(s))")
        self.tdiv = v
        self.ed_tdiv.setText(fmt_si(v, "s"))
        self.update_axes()
        self.update_plan_label()
        self.render()
        self.send_config()

    def update_plan_label(self):
        pl = self.plan()
        self.lbl_plan.setText(
            f"{pl['n']} samples/ch at {fmt_si(1.0 / pl['dt'], 'S/s')} "
            f"({fmt_si(pl['dt'], 's')} apart), window {fmt_si(pl['window'], 's')}")

    def on_tdiv_edit(self):
        try:
            v = parse_value(self.ed_tdiv.text(), _TIME_UNITS, "ms")
            if v <= 0:
                raise ValueError
        except ValueError:
            self.notify("Could not read time/div. Examples: 2.5ms, 250us, 1s")
            self.ed_tdiv.setText(fmt_si(self.tdiv, "s"))
            return
        self.set_tdiv(v)

    def step_tdiv(self, direction):
        self.set_tdiv(ladder_step(self.tdiv, TDIV_LADDER, direction))

    def set_vdiv(self, v):
        self.vdiv = min(max(v, 0.001), 1000.0)
        self.ed_vdiv.setText(fmt_si(self.vdiv, "V"))
        self.update_axes()

    def on_vdiv_edit(self):
        try:
            v = parse_value(self.ed_vdiv.text(), _VOLT_UNITS, "v")
            if v <= 0:
                raise ValueError
        except ValueError:
            self.notify("Could not read volts/div. Examples: 500mV, 2V")
            self.ed_vdiv.setText(fmt_si(self.vdiv, "V"))
            return
        self.set_vdiv(v)

    def step_vdiv(self, direction):
        self.set_vdiv(ladder_step(self.vdiv, VDIV_LADDER, direction))

    def on_center(self, v):
        self.vcenter = v
        self.update_axes()

    def update_axes(self):
        pl = self.plan()
        self.pi.setXRange(-pl["pre"] * pl["dt"], (pl["n"] - pl["pre"]) * pl["dt"], padding=0)
        self.pi.setYRange(self.vcenter - 4 * self.vdiv, self.vcenter + 4 * self.vdiv, padding=0)
        self.pi.getAxis("bottom").setTickSpacing(self.tdiv, self.tdiv / 5)
        self.pi.getAxis("left").setTickSpacing(self.vdiv, self.vdiv / 5)

    def auto_y(self):
        if not self.last:
            return
        _, series, _, _ = self.compute_series()
        arrs = [v for c, v in series.items() if (self.mask >> c) & 1]
        if not arrs:
            return
        lo = min(float(a.min()) for a in arrs)
        hi = max(float(a.max()) for a in arrs)
        need = max(hi - lo, 0.2) * 1.25 / 8.0
        vd = next((x for x in VDIV_LADDER if x >= need), VDIV_LADDER[-1])
        self.vcenter = (hi + lo) / 2.0
        self.sp_center.blockSignals(True)
        self.sp_center.setValue(self.vcenter)
        self.sp_center.blockSignals(False)
        self.set_vdiv(vd)

    def on_pre(self, v):
        self.pre_pct = v
        self.lbl_pre.setText(f"Trigger position {v} %")
        self.update_axes()
        self.send_config()

    def on_channel(self, c, on):
        if on:
            self.mask |= 1 << c
        else:
            self.mask &= ~(1 << c)
        if self.mask and not (self.mask >> self.trig_ch) & 1:
            first = next(i for i in range(NCH) if self.mask >> i & 1)
            self.set_trig_ch(first)
        self.set_tdiv(self.tdiv)

    def set_trig_ch(self, c):
        self.trig_ch = c
        self.cb_src.blockSignals(True)
        self.cb_src.setCurrentIndex(c)
        self.cb_src.blockSignals(False)
        self.ln_trig_v.setPen(pg.mkPen(COLORS[c], width=1.5, style=QtCore.Qt.PenStyle.DashLine))

    def on_trig_src(self, i):
        self.set_trig_ch(i)
        if not (self.mask >> i) & 1:
            self.btn_ch[i].setChecked(True)      # triggers on_channel -> send_config
        else:
            self.send_config()

    def on_edge(self, i):
        self.edge = i
        self.send_config()

    def set_level(self, v):
        self.level_v = float(v)
        self.sp_level.blockSignals(True)
        self.sp_level.setValue(self.level_v)
        self.sp_level.blockSignals(False)
        self.ln_trig_v.blockSignals(True)
        self.ln_trig_v.setValue(self.level_v)
        self.ln_trig_v.blockSignals(False)
        self.send_config()

    def on_scale(self, c, v):
        self.scale[c] = v
        self.send_config()
        self.render()

    def on_offset(self, c, v):
        self.offset[c] = v
        self.send_config()
        self.render()

    def on_cursors(self, on):
        for ln in self.cur_t + self.cur_v:
            ln.setVisible(on)
        self.update_cursor_readout()

    def update_cursor_readout(self, *_):
        if not self.chk_cur.isChecked():
            self.lbl_cur.setText("")
            return
        t1, t2 = (ln.value() for ln in self.cur_t)
        v1, v2 = (ln.value() for ln in self.cur_v)
        dt = abs(t2 - t1)
        inv = fmt_si(1.0 / dt, "Hz") if dt > 0 else "—"
        self.lbl_cur.setText(
            f"Cursors   ΔT = {fmt_si(dt, 's')}   1/ΔT = {inv}   "
            f"ΔV = {fmt_si(abs(v2 - v1), 'V')}   (V1 {v1:.3f}, V2 {v2:.3f})")

    # ---------------------------------------------------------------- data
    def on_frame(self):
        fr = self.worker.take() if self.worker else None
        if fr is None or not self.running:
            return
        self.last = fr
        self.last_frame_time = time.time()
        self.vcc = fr["vcc"]
        if self.single:
            self.single = False
            self.running = False
            self.btn_run.setText("Run")
        if not self.running:
            self.set_state("STOP")
        elif fr["flags"] & FLAG_TRIG:
            self.set_state("TRIG'D")
        else:
            self.set_state("AUTO (no trigger)")
        if fr["flags"] & FLAG_OVERRUN:
            self.handle_overrun(fr)
        self.render()

    def handle_overrun(self, fr):
        nch = bin(fr["mask"]).count("1")
        at_limit = fr["dt"] * 1e6 <= self.tmin_us(nch) * 1.05
        if at_limit and self.tmin_factor < 3.0:
            self.tmin_factor *= 1.12
            self.notify("Nano could not keep up, sampling limit adjusted automatically")
            self.set_tdiv(self.tdiv)
        else:
            self.notify("Overrun: the Nano is sampling slower than requested")

    def compute_series(self):
        fr = self.last
        n, dt = fr["n"], fr["dt"]
        pre = fr["trig"] if fr["trig"] != 0xFFFF else int(round(n * self.pre_pct / 100.0))
        t = (np.arange(n) - pre) * dt
        out, k = {}, 0
        for c in range(NCH):
            if fr["mask"] >> c & 1:
                raw = fr["data"][:, k].astype(float) * fr["vcc"] / 256.0
                out[c] = (raw - self.offset[c]) * self.scale[c]
                k += 1
        return t, out, dt, pre

    def render(self):
        fr = self.last
        if fr is None:
            for cv in self.curves:
                cv.setData([], [])
            return
        t, series, dt, _ = self.compute_series()
        meas = {}
        for c in range(NCH):
            if c in series and (self.mask >> c) & 1:
                self.curves[c].setData(t, series[c])
                lsb = fr["vcc"] / 256.0 * self.scale[c]
                meas[c] = measure(series[c], dt, lsb)
            else:
                self.curves[c].setData([], [])
        for c in range(NCH):
            m = meas.get(c, {})
            for r, key in enumerate(MEAS_ROWS):
                self.tbl.item(r, c).setText(cell_text(key, m) if m else "—")
        self.update_cursor_readout()
        nch = bin(fr["mask"]).count("1")
        self.lbl_info.setText(
            f"{fr['n']} samples x {nch} ch   {fmt_si(1.0 / dt, 'S/s')} per channel   "
            f"window {fmt_si(fr['n'] * dt, 's')}   Vcc {fr['vcc']:.2f} V   "
            f"frames {self.worker.good if self.worker else 0}"
            + (f"   bad {self.worker.bad}" if self.worker and self.worker.bad else ""))

    def watchdog(self):
        if not self.worker or not self.running or self.single or self.trig_mode != "auto":
            return
        ref = max(self.last_cfg_time, self.last_frame_time)
        limit = max(3.0, 3.0 * self.plan()["window"] + 2.0)
        if ref and time.time() - ref > limit and self.lbl_state.text() != "NO DATA":
            self.set_state("NO DATA")
            self.notify("No frames received. Check the port, baud rate (500000) and that nano_scope.ino is flashed.", 10000)

    # ---------------------------------------------------------------- export
    def export_csv(self):
        if not self.last:
            self.notify("Nothing captured yet")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save CSV", "scope.csv", "CSV (*.csv)")
        if not path:
            return
        t, series, _, _ = self.compute_series()
        chans = [c for c in sorted(series) if (self.mask >> c) & 1]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s"] + [f"CH{c + 1}_V" for c in chans])
            for i in range(len(t)):
                w.writerow([f"{t[i]:.9g}"] + [f"{series[c][i]:.5g}" for c in chans])
        self.notify(f"Saved {path}")

    def export_png(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save screenshot", "scope.png", "PNG (*.png)")
        if path:
            self.grab().save(path)
            self.notify(f"Saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="simulated Nano, no hardware")
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=500000)
    a = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setPalette(dark_palette())
    w = Scope(demo=a.demo, port=a.port, baud=a.baud)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[!] Programm beendet (Ctrl+C).")
        sys.exit(0)