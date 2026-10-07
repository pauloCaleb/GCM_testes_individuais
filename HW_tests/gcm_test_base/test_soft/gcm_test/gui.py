"""
GUI de testes da GCM-PI2-2026.2 (PySide6 + pyqtgraph).

Mostra sensores de borda, botão de start e os 3 sensores ToF (VL53L1X) em
tempo real e controla os LEDs da fita e as 2 pontes H (BTS7960).

Execução:   python run_gui.py            (ou:  python -m gcm_test)
Sem placa:  python run_gui.py --demo     (usa o simulador do firmware)
"""

from __future__ import annotations

import os

# O pyqtgraph escolhe sozinho um binding Qt: se nenhum estiver importado ele tenta
# PyQt6 ANTES de PySide6. Com PyQt6 instalado isso carrega as DLLs do Qt do PyQt6 e
# depois o QtCore do PySide6 falha ("DLL load failed ... procedimento especificado").
# Por isso fixamos o PySide6 ANTES de importar o pyqtgraph.
os.environ["PYQTGRAPH_QT_LIB"] = "PySide6"

import argparse
import collections
import csv
import datetime as dt
import html
import sys
import time
from typing import Optional

from PySide6 import QtCore, QtGui, QtWidgets  # noqa: E402  (antes do pyqtgraph, de propósito)

import numpy as np  # noqa: E402
import pyqtgraph as pg  # noqa: E402

from . import __version__  # noqa: E402
from .client import DEFAULT_BAUD, DEMO_PORT, GcmClient, GcmError, available_ports

TOF_COLORS = ["#e74c3c", "#2980b9", "#27ae60"]
PLOT_SECONDS = 30
HEARTBEAT_MS = 150
HANDSHAKE_MS = 500
HANDSHAKE_TRIES = 60          # ~30 s: o firmware leva alguns segundos para dar boot

RANGE_STATUS = {
    0: "válida", 1: "sigma alto", 2: "sinal alto", 3: "min. range", 4: "sinal fraco",
    5: "sem sinal", 6: "erro de HW", 7: "warm-up", 9: "merged pulse", 10: "sinal muito fraco",
    11: "interferência", 12: "erro de intervalo", 13: "erro de calibração", 255: "sem leitura",
}

LED_LABELS = ["LED 1 (IO7)", "LED 2 (IO6)", "LED 3 (IO5)"]
BS_LABELS = ["BS_1 (GPIO34)", "BS_2 (GPIO35)", "BS_3 (GPIO16)", "BS_4 (GPIO14)"]
MOTOR_LABELS = ["Motor 1  (PWM1 / DIR_1)", "Motor 2  (PWM2 / DIR2)"]
CSV_COLUMNS = (["host_time", "fw_ms", "bs1", "bs2", "bs3", "bs4", "start"]
               + [f"tof{i}_{k}" for i in (1, 2, 3) for k in ("on", "mm", "st")]
               + ["led1", "led2", "led3", "en", "m1_cur", "m2_cur", "m1_tgt", "m2_tgt", "cap", "failsafe"])


class Dot(QtWidgets.QFrame):
    """Indicador circular (verde = ativo, cinza = inativo, claro = desconhecido)."""

    def __init__(self, diameter: int = 22) -> None:
        super().__init__()
        self._d = diameter
        self.setFixedSize(diameter, diameter)
        self.set_state(None)

    def set_state(self, state: Optional[bool]) -> None:
        color = "#bdc3c7" if state is None else ("#2ecc71" if state else "#566573")
        self.setStyleSheet(f"background:{color}; border:1px solid #222; border-radius:{self._d // 2}px;")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(f"GCM-PI2-2026.2 - Base de testes v{__version__}")
        self.resize(1280, 860)

        self.client = GcmClient()
        self.ready = False
        self._handshake_tries = 0
        self._tel_count = 0
        self._csv_file = None
        self._csv_writer = None
        self._t0 = time.monotonic()
        self._hist = collections.deque(maxlen=PLOT_SECONDS * 60)   # (t, [mm0, mm1, mm2])
        self._motor_dirty = [False, False]

        self._build_ui()
        self._build_timers()
        self._refresh_ports()
        self._set_connected_ui(False)

    # ------------------------------------------------------------------
    # construção da interface
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        root.addLayout(self._build_conn_bar())

        self.banner = QtWidgets.QLabel("")
        self.banner.setStyleSheet("background:#c0392b; color:white; font-weight:bold; padding:6px;")
        self.banner.setAlignment(QtCore.Qt.AlignCenter)
        self.banner.hide()
        root.addWidget(self.banner)

        self.panels = QtWidgets.QWidget()
        pl = QtWidgets.QHBoxLayout(self.panels)
        pl.setContentsMargins(0, 0, 0, 0)
        root.addWidget(self.panels, 1)

        left = QtWidgets.QVBoxLayout()
        left.addWidget(self._build_inputs())
        left.addWidget(self._build_leds())
        left.addWidget(self._build_motors())
        left.addWidget(self._build_config())
        left.addStretch(1)
        pl.addLayout(left, 0)

        right = QtWidgets.QVBoxLayout()
        right.addWidget(self._build_tof(), 3)
        right.addWidget(self._build_log(), 2)
        pl.addLayout(right, 1)

        self.rate_lbl = QtWidgets.QLabel("")
        self.statusBar().addPermanentWidget(self.rate_lbl)

        for key in ("Escape", "Space"):
            sc = QtGui.QShortcut(QtGui.QKeySequence(key), self)
            sc.setContext(QtCore.Qt.ApplicationShortcut)
            sc.activated.connect(self._do_stop)

    def _build_conn_bar(self) -> QtWidgets.QHBoxLayout:
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Porta:"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(180)
        bar.addWidget(self.port_combo)
        self.refresh_btn = QtWidgets.QPushButton("Atualizar")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        bar.addWidget(self.refresh_btn)
        bar.addWidget(QtWidgets.QLabel("Baud:"))
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(["921600", "460800", "115200"])
        self.baud_combo.setCurrentText(str(DEFAULT_BAUD))
        self.baud_combo.setFixedWidth(100)
        bar.addWidget(self.baud_combo)
        self.connect_btn = QtWidgets.QPushButton("Conectar")
        self.connect_btn.clicked.connect(self._toggle_connection)
        bar.addWidget(self.connect_btn)
        self.status_lbl = QtWidgets.QLabel("Desconectado")
        bar.addWidget(self.status_lbl, 1)

        self.stop_btn = QtWidgets.QPushButton("PARAR MOTORES  (Esc / Espaço)")
        self.stop_btn.setStyleSheet("background:#c0392b; color:white; font-weight:bold; padding:8px 18px;")
        self.stop_btn.setFocusPolicy(QtCore.Qt.NoFocus)
        self.stop_btn.clicked.connect(self._do_stop)
        bar.addWidget(self.stop_btn)
        return bar

    def _build_inputs(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Entradas digitais (CN15)")
        grid = QtWidgets.QGridLayout(box)
        self.in_dots, self.in_vals = [], []
        for i, name in enumerate(BS_LABELS + ["START_BOT (GPIO4)"]):
            dot, val = Dot(), QtWidgets.QLabel("-")
            grid.addWidget(dot, i, 0)
            grid.addWidget(QtWidgets.QLabel(name), i, 1)
            grid.addWidget(val, i, 2)
            self.in_dots.append(dot)
            self.in_vals.append(val)
        self.active_low = QtWidgets.QCheckBox("Ativo em nível baixo (0 = acionado)")
        grid.addWidget(self.active_low, len(self.in_dots), 0, 1, 3)
        return box

    def _build_leds(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("LEDs da fita (PCA9554A)")
        lay = QtWidgets.QVBoxLayout(box)
        row = QtWidgets.QHBoxLayout()
        self.led_btns = []
        for i, name in enumerate(LED_LABELS):
            b = QtWidgets.QPushButton(name)
            b.setCheckable(True)
            b.setFocusPolicy(QtCore.Qt.NoFocus)
            b.setStyleSheet("QPushButton:checked { background:#f1c40f; font-weight:bold; }")
            b.clicked.connect(lambda checked, idx=i + 1: self._send(lambda: self.client.led(idx, checked)))
            row.addWidget(b)
            self.led_btns.append(b)
        lay.addLayout(row)
        row2 = QtWidgets.QHBoxLayout()
        on_btn, off_btn = QtWidgets.QPushButton("Todos ligados"), QtWidgets.QPushButton("Todos desligados")
        on_btn.clicked.connect(lambda: self._send(lambda: self.client.led("all", True)))
        off_btn.clicked.connect(lambda: self._send(lambda: self.client.led("all", False)))
        for b in (on_btn, off_btn):
            b.setFocusPolicy(QtCore.Qt.NoFocus)
            row2.addWidget(b)
        lay.addLayout(row2)
        return box

    def _build_motors(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Pontes H / motores (BTS7960)")
        lay = QtWidgets.QVBoxLayout(box)

        self.en_btn = QtWidgets.QPushButton("Pontes H DESABILITADAS (EN_ALL = 0)")
        self.en_btn.setCheckable(True)
        self.en_btn.setFocusPolicy(QtCore.Qt.NoFocus)
        self.en_btn.setStyleSheet("QPushButton:checked { background:#e67e22; color:white; font-weight:bold; }")
        self.en_btn.clicked.connect(self._toggle_en)
        lay.addWidget(self.en_btn)

        self.sliders, self.duty_lbls, self.act_lbls, self.inv_chks = [], [], [], []
        self._send_timers = []
        for ch, name in enumerate(MOTOR_LABELS):
            g = QtWidgets.QGroupBox(name)
            gl = QtWidgets.QGridLayout(g)
            s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setRange(-30, 30)
            s.setTickPosition(QtWidgets.QSlider.TicksBelow)
            s.setTickInterval(10)
            s.valueChanged.connect(lambda _v, c=ch: self._on_slider(c))
            dl = QtWidgets.QLabel("alvo: 0 %")
            dl.setMinimumWidth(80)
            al = QtWidgets.QLabel("aplicado: 0.0 %")
            zero = QtWidgets.QPushButton("Zero")
            zero.setFocusPolicy(QtCore.Qt.NoFocus)
            zero.clicked.connect(lambda _c=False, c=ch: self._zero_motor(c))
            inv = QtWidgets.QCheckBox("Inverter sentido")
            inv.stateChanged.connect(lambda _s, c=ch: self._on_slider(c))
            gl.addWidget(s, 0, 0, 1, 3)
            gl.addWidget(dl, 1, 0)
            gl.addWidget(al, 1, 1)
            gl.addWidget(zero, 1, 2)
            gl.addWidget(inv, 2, 0, 1, 3)
            lay.addWidget(g)
            self.sliders.append(s)
            self.duty_lbls.append(dl)
            self.act_lbls.append(al)
            self.inv_chks.append(inv)

            t = QtCore.QTimer(self)
            t.setSingleShot(True)
            t.timeout.connect(lambda c=ch: self._send_timer_done(c))
            self._send_timers.append(t)
        return box

    def _build_config(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Configuração do firmware")
        form = QtWidgets.QFormLayout(box)
        self.cap_spin = QtWidgets.QSpinBox(); self.cap_spin.setRange(0, 100); self.cap_spin.setValue(30)
        self.cap_spin.setSuffix(" %")
        self.slew_spin = QtWidgets.QSpinBox(); self.slew_spin.setRange(0, 2000); self.slew_spin.setValue(300)
        self.slew_spin.setSuffix(" %/s"); self.slew_spin.setSpecialValueText("sem rampa")
        self.wd_spin = QtWidgets.QSpinBox(); self.wd_spin.setRange(0, 10000); self.wd_spin.setSingleStep(50)
        self.wd_spin.setValue(500); self.wd_spin.setSuffix(" ms"); self.wd_spin.setSpecialValueText("desligado")
        self.hz_spin = QtWidgets.QSpinBox(); self.hz_spin.setRange(1, 50); self.hz_spin.setValue(20)
        self.hz_spin.setSuffix(" Hz")
        form.addRow("Limite de duty:", self.cap_spin)
        form.addRow("Rampa (slew):", self.slew_spin)
        form.addRow("Watchdog:", self.wd_spin)
        form.addRow("Telemetria:", self.hz_spin)
        apply_btn = QtWidgets.QPushButton("Aplicar")
        apply_btn.clicked.connect(self._apply_cfg)
        form.addRow(apply_btn)
        return box

    def _build_tof(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Sensores de distância VL53L1X")
        lay = QtWidgets.QVBoxLayout(box)
        cards = QtWidgets.QHBoxLayout()
        self.tof_mm_lbls, self.tof_st_lbls = [], []
        for i in range(3):
            c = QtWidgets.QGroupBox(f"S{i + 1}")
            cl = QtWidgets.QVBoxLayout(c)
            mm = QtWidgets.QLabel("-")
            f = mm.font(); f.setPointSize(22); f.setBold(True); mm.setFont(f)
            mm.setStyleSheet(f"color:{TOF_COLORS[i]};")
            mm.setAlignment(QtCore.Qt.AlignCenter)
            st = QtWidgets.QLabel("-")
            st.setAlignment(QtCore.Qt.AlignCenter)
            cl.addWidget(mm)
            cl.addWidget(st)
            cards.addWidget(c)
            self.tof_mm_lbls.append(mm)
            self.tof_st_lbls.append(st)
        lay.addLayout(cards)

        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        self.plot = pg.PlotWidget()
        self.plot.setLabel("left", "Distância (mm)")
        self.plot.setLabel("bottom", "Tempo (s)")
        self.plot.getAxis("left").enableAutoSIPrefix(False)
        self.plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setXRange(-PLOT_SECONDS, 0)
        self.plot.setYRange(0, 4000)
        self.plot.addLegend()
        self.curves = [self.plot.plot(pen=pg.mkPen(TOF_COLORS[i], width=2), name=f"S{i + 1}")
                       for i in range(3)]
        lay.addWidget(self.plot, 1)
        return box

    def _build_log(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Log")
        lay = QtWidgets.QVBoxLayout(box)
        row = QtWidgets.QHBoxLayout()
        self.acks_chk = QtWidgets.QCheckBox("Mostrar acks")
        self.csv_chk = QtWidgets.QCheckBox("Gravar telemetria em CSV")
        self.csv_chk.toggled.connect(self._toggle_csv)
        clear = QtWidgets.QPushButton("Limpar")
        row.addWidget(self.acks_chk)
        row.addWidget(self.csv_chk)
        row.addStretch(1)
        row.addWidget(clear)
        lay.addLayout(row)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(3000)
        mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        self.log_view.setFont(mono)
        clear.clicked.connect(self.log_view.clear)
        lay.addWidget(self.log_view)
        return box

    def _build_timers(self) -> None:
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self._drain)
        self.poll_timer.start(25)

        self.hb_timer = QtCore.QTimer(self)
        self.hb_timer.timeout.connect(self._heartbeat)
        self.hb_timer.start(HEARTBEAT_MS)

        self.hs_timer = QtCore.QTimer(self)
        self.hs_timer.timeout.connect(self._handshake)

        self.rate_timer = QtCore.QTimer(self)
        self.rate_timer.timeout.connect(self._update_rate)
        self.rate_timer.start(1000)

    # ------------------------------------------------------------------
    # conexão
    # ------------------------------------------------------------------
    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(available_ports())
        if current:
            self.port_combo.setCurrentText(current)

    def _set_connected_ui(self, connected: bool) -> None:
        self.connect_btn.setText("Desconectar" if connected else "Conectar")
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.refresh_btn.setEnabled(not connected)
        self.stop_btn.setEnabled(connected)
        self.panels.setEnabled(connected and self.ready)

    def connect_to(self, port: str, baud: int = DEFAULT_BAUD) -> None:
        try:
            self.client.connect(port, baud)
        except GcmError as e:
            self.log(str(e), "err")
            self.status_lbl.setText(f"Erro: {e}")
            return
        self.ready = False
        self._t0 = time.monotonic()
        self._hist.clear()
        self._handshake_tries = 0
        self.banner.hide()
        self.status_lbl.setText(f"Conectado a {port}; aguardando o firmware (hello)...")
        self.log(f"conectado a {port} @ {baud}", "info")
        self._set_connected_ui(True)
        self.hs_timer.start(HANDSHAKE_MS)
        self._handshake()
        if self.csv_chk.isChecked():
            self._open_csv()

    def disconnect_from(self, reason: str = "") -> None:
        if self.client.connected:
            try:
                self.client.stop()
            except GcmError:
                pass
        self.client.disconnect()
        self.hs_timer.stop()
        self.ready = False
        self._close_csv()
        self._set_connected_ui(False)
        self.status_lbl.setText("Desconectado" + (f" ({reason})" if reason else ""))
        self.log("desconectado" + (f": {reason}" if reason else ""), "info")

    def _toggle_connection(self) -> None:
        if self.client.connected:
            self.disconnect_from()
            return
        port = self.port_combo.currentText().strip()
        if not port:
            self.status_lbl.setText("Selecione uma porta")
            return
        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            self.status_lbl.setText("Baud inválido")
            return
        self.connect_to(port, baud)

    def _handshake(self) -> None:
        if not self.client.connected or self.ready:
            self.hs_timer.stop()
            return
        self._handshake_tries += 1
        hint = self._handshake_hint()
        if self._handshake_tries > HANDSHAKE_TRIES:
            self.hs_timer.stop()
            self.status_lbl.setText(hint or "Sem resposta do firmware (confira porta, baud e firmware)")
            self.log(hint or "sem resposta do firmware ao hello", "err")
            return
        if hint and self._handshake_tries == 8:       # ~4 s: o boot do firmware leva ~4 s
            self.status_lbl.setText(hint)
            self.log(hint, "err")
        self._send(lambda: self.client.hello())

    def _handshake_hint(self) -> str:
        """Dica quando chegam bytes mas nenhuma mensagem JSON válida (baud errado)."""
        c = self.client
        if c.rx_bytes > 0 and c.rx_json == 0:
            return (f"Chegam dados ilegíveis ({c.rx_bytes} bytes, nenhum JSON): o baud provavelmente não "
                    f"confere com o do firmware ({self.baud_combo.currentText()} na GUI). "
                    f"Confira o CONFIG_ESP_CONSOLE_UART_BAUDRATE do firmware.")
        return ""

    def _heartbeat(self) -> None:
        if self.client.connected and self.ready:
            self._send(lambda: self.client.ping())

    # ------------------------------------------------------------------
    # envio
    # ------------------------------------------------------------------
    def _send(self, fn) -> None:
        try:
            fn()
        except GcmError as e:
            self.log(f"falha ao enviar: {e}", "err")

    def _do_stop(self) -> None:
        for ch in range(2):
            self.sliders[ch].blockSignals(True)
            self.sliders[ch].setValue(0)
            self.sliders[ch].blockSignals(False)
            self.duty_lbls[ch].setText("alvo: 0 %")
        self.en_btn.blockSignals(True)
        self.en_btn.setChecked(False)
        self.en_btn.blockSignals(False)
        self._update_en_text(False)
        if self.client.connected:
            self._send(lambda: self.client.stop())
            self.log("STOP enviado", "warn")

    def _toggle_en(self, checked: bool) -> None:
        if checked:
            for ch in range(2):
                self.sliders[ch].blockSignals(True)
                self.sliders[ch].setValue(0)
                self.sliders[ch].blockSignals(False)
                self.duty_lbls[ch].setText("alvo: 0 %")
            self.banner.hide()
        self._update_en_text(checked)
        self._send(lambda: self.client.enable(checked))

    def _update_en_text(self, on: bool) -> None:
        self.en_btn.setText("Pontes H HABILITADAS (EN_ALL = 1)" if on
                            else "Pontes H DESABILITADAS (EN_ALL = 0)")

    def _on_slider(self, ch: int) -> None:
        v = self.sliders[ch].value()
        sign = -1 if self.inv_chks[ch].isChecked() else 1
        self.duty_lbls[ch].setText(f"alvo: {v * sign:+d} %")
        if self._send_timers[ch].isActive():
            self._motor_dirty[ch] = True
        else:
            self._send_motor(ch)
            self._motor_dirty[ch] = False
            self._send_timers[ch].start(60)

    def _send_timer_done(self, ch: int) -> None:
        if self._motor_dirty[ch]:
            self._send_motor(ch)
            self._motor_dirty[ch] = False
            self._send_timers[ch].start(60)

    def _send_motor(self, ch: int) -> None:
        sign = -1 if self.inv_chks[ch].isChecked() else 1
        duty = self.sliders[ch].value() * sign
        self._send(lambda: self.client.motor(ch + 1, duty))

    def _zero_motor(self, ch: int) -> None:
        self.sliders[ch].setValue(0)
        self._send_motor(ch)

    def _apply_cfg(self) -> None:
        self._send(lambda: self.client.cfg(
            max_duty=self.cap_spin.value(), slew=self.slew_spin.value(),
            wd_ms=self.wd_spin.value(), tel_hz=self.hz_spin.value()))

    def _set_cap(self, cap: float) -> None:
        c = int(round(cap))
        for s in self.sliders:
            s.setRange(-c, c)
        self.cap_spin.blockSignals(True)
        self.cap_spin.setValue(c)
        self.cap_spin.blockSignals(False)

    # ------------------------------------------------------------------
    # recepção
    # ------------------------------------------------------------------
    def _drain(self) -> None:
        for m in self.client.drain():
            t = m.get("t")
            if t == "tel":
                self._apply_tel(m)
            elif t == "hello":
                self._apply_hello(m)
            elif t == "evt":
                self._apply_evt(m)
            elif t == "ack":
                if m.get("cmd") == "cfg":
                    self._set_cap(m.get("max_duty", self.cap_spin.value()))
                    self.slew_spin.setValue(int(m.get("slew", 0)))
                    self.wd_spin.setValue(int(m.get("wd_ms", 0)))
                    self.hz_spin.setValue(int(m.get("tel_hz", 20)))
                if self.acks_chk.isChecked() or m.get("cmd") == "cfg":
                    self.log(f"ack {m.get('cmd')}: " + ", ".join(
                        f"{k}={v}" for k, v in m.items() if k not in ("t", "cmd", "_rx")), "ack")
            elif t == "err":
                self.log(f"ERRO do firmware ({m.get('cmd')}): {m.get('msg')}", "err")
            elif t == "log":
                self.log(m.get("line", ""), "fw")
            elif t == "disconnect":
                self.disconnect_from(m.get("error", "erro de comunicação"))
            # "pong": só mantém o watchdog alimentado

    def _apply_hello(self, m: dict) -> None:
        first = not self.ready
        self.ready = True
        self.hs_timer.stop()
        self._set_cap(m.get("cap", 30))
        self.slew_spin.setValue(int(m.get("slew", 0)))
        self.wd_spin.setValue(int(m.get("wd_ms", 0)))
        self.hz_spin.setValue(int(m.get("tel_hz", 20)))
        self._set_connected_ui(True)
        self.status_lbl.setText(f"{m.get('fw')} v{m.get('ver')}  (protocolo {m.get('proto')})  - "
                                f"PCA9554A: {'ok' if m.get('exp') else 'FALHOU'}, "
                                f"ToF: {sum(m.get('tof', [0, 0, 0]))}/3")
        if first:
            self.log(f"hello: {m.get('fw')} v{m.get('ver')} proto={m.get('proto')} "
                     f"exp={m.get('exp')} tof={m.get('tof')} cap={m.get('cap')} "
                     f"wd={m.get('wd_ms')}ms slew={m.get('slew')}", "info")
            if not m.get("exp"):
                self.log("PCA9554A não respondeu: LEDs e sensores ToF indisponíveis", "err")

    def _apply_evt(self, m: dict) -> None:
        name = m.get("name")
        if name == "watchdog":
            self.banner.setText("WATCHDOG disparou: sem comandos da GUI, motores parados e EN_ALL desabilitado. "
                                "Habilite as pontes H novamente para continuar.")
            self.banner.show()
            self.en_btn.blockSignals(True)
            self.en_btn.setChecked(False)
            self.en_btn.blockSignals(False)
            self._update_en_text(False)
            self.log("WATCHDOG: motores parados pelo firmware", "err")
        elif name == "bs":
            self.log(f"evento: BS_{m.get('idx')} -> {m.get('v')}", "evt")
        elif name == "start":
            self.log(f"evento: START_BOT -> {m.get('v')}", "evt")
        else:
            self.log(f"evento: {m}", "evt")

    def _apply_tel(self, m: dict) -> None:
        self._tel_count += 1
        low = self.active_low.isChecked()

        vals = list(m.get("bs", [])) + [m.get("start")]
        for i, v in enumerate(vals[:5]):
            if v is None:
                continue
            self.in_dots[i].set_state((v == 0) if low else (v == 1))
            self.in_vals[i].setText(f"nível {v}")

        now = time.monotonic() - self._t0
        row = []
        for i, s in enumerate(m.get("tof", [])[:3]):
            if not s.get("on"):
                self.tof_mm_lbls[i].setText("OFFLINE")
                self.tof_st_lbls[i].setText("sensor não iniciou")
                row.append(float("nan"))
                continue
            mm, st = s.get("mm", -1), s.get("st", 255)
            valid = mm >= 0 and st == 0
            self.tof_mm_lbls[i].setText(f"{mm} mm" if mm >= 0 else "...")
            self.tof_st_lbls[i].setText(f"{RANGE_STATUS.get(st, st)}  (idade {s.get('age', 0)} ms, "
                                        f"errI2C {s.get('err', 0)})")
            row.append(float(mm) if valid else float("nan"))
        row += [float("nan")] * (3 - len(row))
        self._hist.append((now, row))
        self._update_plot(now)

        for i, b in enumerate(self.led_btns):
            b.blockSignals(True)
            b.setChecked(bool(m["led"][i]))
            b.blockSignals(False)

        en = bool(m.get("en"))
        self.en_btn.blockSignals(True)
        self.en_btn.setChecked(en)
        self.en_btn.blockSignals(False)
        self._update_en_text(en)

        for ch in range(2):
            cur, tgt = m["m"][ch], m["mt"][ch]
            self.act_lbls[ch].setText(f"aplicado: {cur:+.1f} %")
        if m.get("cap") is not None and int(round(m["cap"])) != self.sliders[0].maximum():
            self._set_cap(m["cap"])

        if m.get("fs"):
            if not self.banner.isVisible():
                self.banner.setText("Failsafe ativo: motores parados. Habilite as pontes H para continuar.")
                self.banner.show()

        if self._csv_writer:
            t = m.get("tof", [{}, {}, {}])
            tof_cols = []
            for i in range(3):
                s = t[i] if i < len(t) else {}
                tof_cols += [s.get("on", 0), s.get("mm", ""), s.get("st", "")]
            self._csv_writer.writerow(
                [dt.datetime.now().isoformat(timespec="milliseconds"), m.get("ms"), *m["bs"], m["start"],
                 *tof_cols, *m["led"], m["en"], *m["m"], *m["mt"], m.get("cap"), m.get("fs")])

    def _update_plot(self, now: float) -> None:
        if not self._hist:
            return
        data = np.array([[t, *vals] for t, vals in self._hist], dtype=float)
        x = data[:, 0] - now
        for i in range(3):
            self.curves[i].setData(x, data[:, i + 1], connect="finite")

    def _update_rate(self) -> None:
        self.rate_lbl.setText(f"telemetria: {self._tel_count} quadros/s" if self.client.connected else "")
        self._tel_count = 0

    # ------------------------------------------------------------------
    # log e CSV
    # ------------------------------------------------------------------
    def log(self, text: str, level: str = "info") -> None:
        colors = {"err": "#c0392b", "warn": "#d35400", "evt": "#2980b9", "ack": "#7f8c8d",
                  "fw": "#555555", "info": "#000000"}
        stamp = dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendHtml(
            f'<span style="color:{colors.get(level, "#000")}">{stamp}  {html.escape(text)}</span>')

    def _toggle_csv(self, on: bool) -> None:
        if on:
            if self.client.connected:
                self._open_csv()
        else:
            self._close_csv()

    def _open_csv(self) -> None:
        default = f"gcm_telemetria_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Gravar telemetria", default, "CSV (*.csv)")
        if not path:
            self.csv_chk.blockSignals(True)
            self.csv_chk.setChecked(False)
            self.csv_chk.blockSignals(False)
            return
        self._csv_file = open(path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(CSV_COLUMNS)
        self.log(f"gravando telemetria em {path}", "info")

    def _close_csv(self) -> None:
        if self._csv_file:
            self._csv_file.close()
            self.log("gravação CSV encerrada", "info")
        self._csv_file = None
        self._csv_writer = None

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        self.disconnect_from()
        super().closeEvent(event)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="GUI de testes da GCM-PI2-2026.2")
    ap.add_argument("--demo", action="store_true", help="usa o simulador do firmware (sem hardware)")
    ap.add_argument("--port", help="porta serial para conectar ao abrir")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = ap.parse_args(argv)

    app = QtWidgets.QApplication(sys.argv[:1])
    win = MainWindow()
    win.show()
    if args.demo:
        win.port_combo.setCurrentText(DEMO_PORT)
        win.connect_to(DEMO_PORT, args.baud)
    elif args.port:
        win.port_combo.setCurrentText(args.port)
        win.connect_to(args.port, args.baud)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
