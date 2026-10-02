#!/usr/bin/env python3
"""
battery_monitor.py — Monitor do teste de descarga da bateria (GCM-PI2-2026.2)

Conversa com o firmware `battery_discharge_test` (ESP32) pela serial:
  - envia os comandos TARE, START, STOP, RESET, CUTOFF e STATUS;
  - recebe as linhas D;... (dados) e E;... (eventos) e plota ao vivo;
  - grava um CSV continuo (auto-save) e permite exportar a sessao em CSV;
  - exporta os tres graficos como imagens PNG separadas.

Graficos (checkboxes): Tensao x tempo, Corrente x tempo e Tensao x Ah.
Os marcados dividem a area lado a lado.

O cutoff e decidido pelo FIRMWARE: fechar este programa nao interrompe o teste
(mas ver o aviso no README sobre DTR/RTS reiniciar a ESP32 ao abrir/fechar a porta).

Dependencias: pip install pyserial matplotlib
"""

import csv
import datetime as dt
import json
import math
import os
import queue
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib

matplotlib.use("TkAgg")
from matplotlib import ticker  # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

# --------------------------------------------------------------------------
# Protocolo
# --------------------------------------------------------------------------

BAUD = 115200
D_COLUMNS = ["t_boot_s", "t_test_s", "state", "v_batt_V", "i_A", "p_W",
             "q_Ah", "e_Wh", "ain0_V", "ain1_V"]
SESSION_STATES = ("REST", "DISCHARGE", "RECOVERY", "DONE", "FAULT")
END_STATES = ("DONE", "FAULT")
MAX_PLOT_POINTS = 20000          # acima disso o grafico ao vivo usa decimacao
MAX_EXPORT_POINTS = 200000       # nas imagens exportadas
PLOT_REFRESH_MS = 500
POLL_MS = 50
START_MAXIMIZED = True
CONFIG_PATH = os.path.join(os.path.expanduser("~"), ".gcm_battery_monitor.json")
# Pasta de saida dos CSV/resumos: a mesma pasta onde o programa esta (ou do executavel, se empacotado)
APP_DIR = os.path.dirname(os.path.abspath(sys.executable if getattr(sys, "frozen", False)
                                          else __file__))

# (chave, rotulo do checkbox, nome do arquivo exportado)
PLOT_DEFS = [
    ("v_t", "Tensão × tempo", "tensao_tempo"),
    ("i_t", "Corrente × tempo", "corrente_tempo"),
    ("v_q", "Tensão × Ah", "tensao_capacidade"),
]
TIME_KEYS = ("v_t", "i_t")
# grafico -> (tracker do eixo X, tracker do eixo Y)
AXIS_LIMITS = {"v_t": ("time", "volt"), "i_t": ("time", "cur"), "v_q": ("cap", "volt")}


def to_float(text):
    try:
        return float(text)
    except ValueError:
        return float("nan")


def parse_kv(text):
    """'a=1,b=2' -> {'a': '1', 'b': '2'}"""
    out = {}
    for item in text.split(","):
        if "=" in item:
            k, v = item.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def parse_line(line):
    """Classifica uma linha recebida. Retorna (tipo, payload).

    tipo: 'D' (dict de dados), 'E' (nome, detalhes), 'S' (dict de status),
          'H' (cabecalho) ou 'LOG' (qualquer outra linha).
    """
    line = line.strip()
    if line.startswith("D;"):
        parts = line.split(";")
        if len(parts) == len(D_COLUMNS) + 1:
            row = {"state": parts[3]}
            for name, value in zip(D_COLUMNS, parts[1:]):
                if name != "state":
                    row[name] = to_float(value)
            return "D", row
        return "LOG", line
    if line.startswith("E;"):
        parts = line.split(";", 2)
        name = parts[1] if len(parts) > 1 else ""
        details = parts[2] if len(parts) > 2 else ""
        return "E", (name, details)
    if line.startswith("S;"):
        return "S", parse_kv(line[2:])
    if line.startswith("H;"):
        return "H", line[2:].split(";")
    return "LOG", line


# --------------------------------------------------------------------------
# Eixo de tempo legivel (45s, 12m30s, 1h30)
# --------------------------------------------------------------------------

def fmt_seconds(x, pos=None):
    if x < 0:
        return ""
    s = int(round(x))
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    if h:
        return f"{h}h{m:02d}" if m else f"{h}h"
    if m:
        return f"{m}m{sec:02d}s" if sec else f"{m}m"
    return f"{sec}s"


class TimeLocator(ticker.Locator):
    """Marcas em multiplos 'redondos' de tempo (1, 5, 10, 30 s; 1, 5, 10, 30 min; 1, 2, 6 h...)."""
    STEPS = [1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800,
             3600, 7200, 10800, 21600, 43200, 86400]

    def __init__(self, target=6):
        super().__init__()
        self.target = target

    def tick_values(self, vmin, vmax):
        if vmax < vmin:
            vmin, vmax = vmax, vmin
        span = max(vmax - vmin, 1e-9)
        step = self.STEPS[-1]
        for s in self.STEPS:
            if span / s <= self.target + 1:
                step = s
                break
        start = math.ceil(max(vmin, 0.0) / step) * step
        return np.arange(start, vmax + step * 1e-6, step)

    def __call__(self):
        vmin, vmax = self.axis.get_view_interval()
        return self.tick_values(vmin, vmax)


def style_time_axis(ax):
    ax.xaxis.set_major_locator(TimeLocator())
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(fmt_seconds))


def nice_ceil(x, mantissas=(1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10)):
    """Menor valor 'redondo' (1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10 x 10^k) >= x."""
    if x <= 0:
        return 1.0
    base = 10.0 ** math.floor(math.log10(x))
    for m in mantissas:
        if x <= m * base * (1 + 1e-9):
            return m * base
    return 10.0 * base


class LimitTracker:
    """Limites de eixo ESTAVEIS para o grafico ao vivo.

    Em vez de reajustar a cada atualizacao (o que faz as margens 'dancarem'), o limite
    so muda quando os dados passam da faixa atual, e ai salta para um valor redondo com
    folga. Assim, durante quase todo o teste os eixos ficam parados.

    kind: 'time' (s), 'cap' (Ah), 'volt' (V) ou 'cur' (A).
    """

    def __init__(self, kind):
        self.kind = kind
        self.reset()

    def reset(self):
        self.lo = None
        self.hi = None
        self.fitted = False      # ja se ajustou aos primeiros dados reais?
        self.neg = 0.0

    def update(self, dmin, dmax, cutoff=None):
        """Retorna True se o limite mudou."""
        return getattr(self, "_" + self.kind)(dmin, dmax, cutoff)

    def _time(self, dmin, dmax, cutoff):
        dmax = 0.0 if dmax is None else dmax
        if self.hi is not None and dmax <= self.hi * 0.98:
            return False
        target = max(dmax * 1.5, 60.0)
        step = next((s for s in TimeLocator.STEPS if target / s <= 6), TimeLocator.STEPS[-1])
        self.hi = math.ceil(target / step) * step
        self.lo = -0.01 * self.hi
        return True

    def _cap(self, dmin, dmax, cutoff):
        dmax = 0.0 if dmax is None else dmax
        if self.hi is not None and dmax <= self.hi * 0.98:
            return False
        self.hi = nice_ceil(max(dmax * 1.5, 0.2))
        self.lo = -0.01 * self.hi
        return True

    def _volt(self, dmin, dmax, cutoff):
        c = cutoff if cutoff else 12.0
        has = dmin is not None and dmax is not None
        if self.hi is None or (has and not self.fitted):
            lo_src = min(dmin, c) if has else c
            hi_src = max(dmax, c + 1.0) if has else c + 5.0
            self.lo = math.floor((lo_src - 0.5) * 2) / 2
            self.hi = math.ceil((hi_src + 0.5) * 2) / 2
            self.fitted = has
            return True
        changed = False
        if has and dmin < self.lo + 0.05:
            self.lo = math.floor((dmin - 0.5) * 2) / 2
            changed = True
        if has and dmax > self.hi - 0.05:
            self.hi = math.ceil((dmax + 0.5) * 2) / 2
            changed = True
        if c - 0.2 < self.lo:                      # cutoff alterado para baixo
            self.lo = math.floor((c - 0.5) * 2) / 2
            changed = True
        return changed

    def _cur(self, dmin, dmax, cutoff):
        has = dmin is not None and dmax is not None
        if self.hi is None or (has and not self.fitted):
            self.hi = nice_ceil(max(dmax * 1.3, 2.0) if has else 2.0)
            self.neg = -nice_ceil(-dmin * 1.3) if (has and dmin < -0.05) else 0.0
            self.lo = min(-0.05 * self.hi, self.neg)
            self.fitted = has
            return True
        changed = False
        if has and dmax > self.hi * 0.97:
            self.hi = nice_ceil(dmax * 1.3)
            changed = True
        if has and dmin < self.lo:
            self.neg = -nice_ceil(-dmin * 1.3)
            changed = True
        if changed:
            self.lo = min(-0.05 * self.hi, self.neg)
        return changed


class PlotToolbar(NavigationToolbar2Tk):
    """Barra do matplotlib que avisa o app quando o usuario da zoom/pan (para parar o auto-ajuste)
    e quando aperta Home (para voltar a acompanhar os dados)."""

    def __init__(self, canvas, window, on_home, on_manual):
        self._on_home = on_home
        self._on_manual = on_manual
        super().__init__(canvas, window, pack_toolbar=False)

    def home(self, *args):
        self._on_home()
        super().home(*args)

    def release_zoom(self, event):
        super().release_zoom(event)
        self._on_manual()

    def release_pan(self, event):
        super().release_pan(event)
        self._on_manual()

    def back(self, *args):
        super().back(*args)
        self._on_manual()

    def forward(self, *args):
        super().forward(*args)
        self._on_manual()


# --------------------------------------------------------------------------
# Aplicacao
# --------------------------------------------------------------------------

class BatteryMonitor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GCM — Monitor de descarga de bateria")
        self.geometry("1280x800")
        self.minsize(900, 560)
        if START_MAXIMIZED:
            try:
                if sys.platform.startswith("win"):
                    self.state("zoomed")
                else:
                    self.attributes("-zoomed", True)
            except tk.TclError:
                pass

        # Serial
        self.ser = None
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.rx_queue = queue.Queue()

        # Estado do firmware
        self.fw_state = "-"
        self.fw_cutoff_mv = None
        self.fw_limits = (None, None)

        # Sessao (dados gravados desde o START)
        self.session_active = False
        self.session_start = None
        self.rows = []                 # lista de dicts
        self.events = []               # linhas de evento da sessao
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.summary = None
        self.rows_since_flush = 0
        self.exported = False
        self._plot_dirty = False

        # Dados para o grafico (listas paralelas)
        self.t = []
        self.v = []
        self.i = []
        self.q = []

        # Graficos
        self.follow = True             # False apos zoom/pan manual; Home volta a True
        self.axes = {}
        self.lines = {}
        self.cut_lines = []
        self.trackers = {k: LimitTracker(k) for k in ("time", "cap", "volt", "cur")}
        self.ext = self._new_ext()     # min/max correntes dos dados (evita varrer as listas)
        self._applied = {}

        cfg = self._load_config()
        self.out_dir = tk.StringVar(value=APP_DIR)   # sempre a pasta do programa ao abrir
        self.autosave = tk.BooleanVar(value=cfg.get("autosave", True))
        self.show = {key: tk.BooleanVar(value=cfg.get("show_" + key, True))
                     for key, _, _ in PLOT_DEFS}

        self._build_ui()
        self._rebuild_axes()
        self._refresh_ports()
        self._update_buttons()
        self.after(POLL_MS, self._drain_queue)
        self.after(PLOT_REFRESH_MS, self._refresh_plot)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------- Configuracao

    @staticmethod
    def _load_config():
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save_config(self):
        data = {"autosave": bool(self.autosave.get())}
        for key, var in self.show.items():
            data["show_" + key] = bool(var.get())
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except OSError:
            pass

    # ------------------------------------------------------------------ UI

    def _build_ui(self):
        # --- Conexao
        conn = ttk.LabelFrame(self, text="Conexão")
        conn.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Label(conn, text="Porta:").pack(side="left", padx=(8, 2))
        self.port_cb = ttk.Combobox(conn, width=22, state="readonly")
        self.port_cb.pack(side="left", padx=2, pady=4)
        ttk.Button(conn, text="Atualizar", command=self._refresh_ports).pack(side="left", padx=2)
        self.btn_connect = ttk.Button(conn, text="Conectar", command=self._connect)
        self.btn_connect.pack(side="left", padx=(10, 2))
        self.btn_disconnect = ttk.Button(conn, text="Desconectar", command=self._disconnect)
        self.btn_disconnect.pack(side="left", padx=2)
        self.lbl_conn = ttk.Label(conn, text="desconectado", foreground="#a00")
        self.lbl_conn.pack(side="left", padx=12)

        # --- Controle
        ctl = ttk.LabelFrame(self, text="Controle do teste")
        ctl.pack(fill="x", padx=8, pady=4)
        self.btn_tare = ttk.Button(ctl, text="Tara (zero do ACS758)", command=self._cmd_tare)
        self.btn_tare.pack(side="left", padx=(8, 4), pady=4)
        self.btn_start = ttk.Button(ctl, text="START descarga", command=self._cmd_start)
        self.btn_start.pack(side="left", padx=4)
        self.btn_stop = ttk.Button(ctl, text="STOP", command=self._cmd_stop)
        self.btn_stop.pack(side="left", padx=4)
        self.btn_reset = ttk.Button(ctl, text="Reset", command=self._cmd_reset)
        self.btn_reset.pack(side="left", padx=4)

        ttk.Separator(ctl, orient="vertical").pack(side="left", fill="y", padx=10, pady=4)
        ttk.Label(ctl, text="Cutoff (V):").pack(side="left")
        self.cutoff_var = tk.StringVar(value="12.00")
        self.ent_cutoff = ttk.Entry(ctl, textvariable=self.cutoff_var, width=7)
        self.ent_cutoff.pack(side="left", padx=4)
        self.btn_cutoff = ttk.Button(ctl, text="Aplicar", command=self._cmd_cutoff)
        self.btn_cutoff.pack(side="left", padx=2)
        self.lbl_cutoff_fw = ttk.Label(ctl, text="(firmware: -)")
        self.lbl_cutoff_fw.pack(side="left", padx=6)
        self.btn_save = ttk.Button(ctl, text="Salvar CSV...", command=self._export_csv)
        self.btn_save.pack(side="right", padx=8)

        # --- Valores ao vivo
        live = ttk.LabelFrame(self, text="Medidas")
        live.pack(fill="x", padx=8, pady=4)
        self.live_vars = {}
        fields = [("Estado", "state"), ("V bateria", "v"), ("Corrente", "i"),
                  ("Potência", "p"), ("Capacidade", "q"), ("Energia", "e"),
                  ("t teste", "t")]
        for col, (label, key) in enumerate(fields):
            frm = ttk.Frame(live)
            frm.grid(row=0, column=col, padx=12, pady=2, sticky="w")
            ttk.Label(frm, text=label, foreground="#555").pack(anchor="w")
            var = tk.StringVar(value="-")
            ttk.Label(frm, textvariable=var, font=("TkDefaultFont", 12, "bold")).pack(anchor="w")
            self.live_vars[key] = var

        # --- Log (reserva o espaco de baixo antes do grafico)
        logf = ttk.LabelFrame(self, text="Eventos / log serial")
        logf.pack(side="bottom", fill="x", padx=8, pady=(4, 8))
        self.log = tk.Text(logf, height=4, state="disabled", wrap="none", font=("TkFixedFont", 9))
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.pack(side="left", fill="x", expand=True)
        sb.pack(side="right", fill="y")
        self.log.tag_configure("evt", foreground="#064")
        self.log.tag_configure("err", foreground="#b00")
        self.log.tag_configure("tx", foreground="#00a")
        self.log.tag_configure("raw", foreground="#777")

        # --- Arquivos
        files = ttk.LabelFrame(self, text="Dados")
        files.pack(side="bottom", fill="x", padx=8, pady=4)
        ttk.Checkbutton(files, text="Auto-salvar CSV", variable=self.autosave,
                        command=self._save_config).pack(side="left", padx=8, pady=4)
        ttk.Entry(files, textvariable=self.out_dir, width=45).pack(side="left", padx=2)
        ttk.Button(files, text="Pasta...", command=self._choose_dir).pack(side="left", padx=2)
        self.lbl_file = ttk.Label(files, text="")
        self.lbl_file.pack(side="left", padx=12)

        # --- Graficos
        plot = ttk.Frame(self)
        plot.pack(side="top", fill="both", expand=True, padx=8, pady=4)

        opts = ttk.Frame(plot)
        opts.pack(side="top", fill="x")
        ttk.Label(opts, text="Gráficos:").pack(side="left", padx=(4, 6))
        for key, label, _ in PLOT_DEFS:
            ttk.Checkbutton(opts, text=label, variable=self.show[key],
                            command=self._on_toggle_plots).pack(side="left", padx=6)
        self.btn_images = ttk.Button(opts, text="Exportar imagens (3 PNG)...",
                                     command=self._export_images)
        self.btn_images.pack(side="right", padx=4)
        self.lbl_follow = ttk.Label(opts, text="", foreground="#a60")
        self.lbl_follow.pack(side="right", padx=10)

        self.fig = Figure(figsize=(10, 5), dpi=100, layout="constrained")
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot)
        self.toolbar = PlotToolbar(self.canvas, plot, self._on_home, self._on_manual_view)
        self.toolbar.pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

    # ------------------------------------------------------------ Graficos

    def _on_toggle_plots(self):
        self._rebuild_axes()
        self._update_plot_data()
        self._save_config()

    def _on_manual_view(self):
        self.follow = False
        self.lbl_follow.configure(text="vista manual — Home retoma o acompanhamento")

    def _on_home(self):
        self.follow = True
        self.lbl_follow.configure(text="")
        self._reset_limits()
        self._plot_dirty = True

    @staticmethod
    def _new_ext():
        return {"t_max": None, "q_max": None, "v_min": None, "v_max": None,
                "i_min": None, "i_max": None}

    def _reset_limits(self):
        for tr in self.trackers.values():
            tr.reset()
        self._applied.clear()

    def _update_ext(self, row):
        e = self.ext

        def lo(k, x):
            if not math.isnan(x) and (e[k] is None or x < e[k]):
                e[k] = x

        def hi(k, x):
            if not math.isnan(x) and (e[k] is None or x > e[k]):
                e[k] = x
        hi("t_max", row["t_test_s"])
        hi("q_max", row["q_Ah"])
        lo("v_min", row["v_batt_V"])
        hi("v_max", row["v_batt_V"])
        lo("i_min", row["i_A"])
        hi("i_max", row["i_A"])

    def _apply_limits(self):
        """Aplica limites estaveis: os eixos so se movem quando os dados estouram a faixa."""
        c = self.fw_cutoff_mv / 1000.0 if self.fw_cutoff_mv else None
        e, tr = self.ext, self.trackers
        tr["time"].update(None, e["t_max"])
        tr["cap"].update(None, e["q_max"])
        tr["volt"].update(e["v_min"], e["v_max"], c)
        tr["cur"].update(e["i_min"], e["i_max"])
        for key, ax in self.axes.items():
            xk, yk = AXIS_LIMITS[key]
            want = (tr[xk].lo, tr[xk].hi, tr[yk].lo, tr[yk].hi)
            if self._applied.get(key) != want:
                ax.set_xlim(want[0], want[1])
                ax.set_ylim(want[2], want[3])
                self._applied[key] = want

    def _rebuild_axes(self):
        """Recria os eixos so para os graficos marcados, lado a lado."""
        self.fig.clear()
        self.axes = {}
        self.lines = {}
        self.cut_lines = []
        self.follow = True
        self.lbl_follow.configure(text="")
        self._reset_limits()
        keys = [k for k, _, _ in PLOT_DEFS if self.show[k].get()]
        if not keys:
            self.fig.text(0.5, 0.5, "Nenhum gráfico selecionado", ha="center",
                          va="center", color="#777")
            self.canvas.draw_idle()
            return

        cutoff_v = (self.fw_cutoff_mv or 12000) / 1000.0
        time_ax = None
        for idx, key in enumerate(keys, start=1):
            share = time_ax if key in TIME_KEYS else None
            ax = self.fig.add_subplot(1, len(keys), idx, sharex=share)
            self.axes[key] = ax
            ax.grid(True, alpha=0.3)
            if key in TIME_KEYS:
                style_time_axis(ax)
                ax.set_xlabel("Tempo desde o início da descarga")
                if time_ax is None:
                    time_ax = ax
            ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.1f"))   # largura fixa
            if key == "v_t":
                ax.set_title("Tensão × tempo")
                ax.set_ylabel("V bateria (V)")
                self.lines[key], = ax.plot([], [], color="tab:blue", lw=1.2)
                self.cut_lines.append(ax.axhline(cutoff_v, color="tab:red", ls="--", lw=1,
                                                 label="cutoff"))
                ax.legend(loc="upper right", fontsize=8)
            elif key == "i_t":
                ax.set_title("Corrente × tempo")
                ax.set_ylabel("Corrente (A)")
                self.lines[key], = ax.plot([], [], color="tab:orange", lw=1.2)
            elif key == "v_q":
                ax.set_title("Tensão × capacidade")
                ax.set_xlabel("Capacidade descarregada (Ah)")
                ax.set_ylabel("V bateria (V)")
                ax.xaxis.set_major_formatter(ticker.FormatStrFormatter("%.2f"))
                self.lines[key], = ax.plot([], [], color="tab:green", lw=1.2)
                self.cut_lines.append(ax.axhline(cutoff_v, color="tab:red", ls="--", lw=1,
                                                 label="cutoff"))
                ax.legend(loc="upper right", fontsize=8)
        self.toolbar.update()          # descarta a pilha de zoom dos eixos antigos
        self.canvas.draw_idle()

    def _set_cutoff_lines(self):
        if self.fw_cutoff_mv:
            c = self.fw_cutoff_mv / 1000.0
            for ln in self.cut_lines:
                ln.set_ydata([c, c])

    def _update_plot_data(self, redraw=False):
        """Atualiza apenas os graficos visiveis."""
        n = len(self.t)
        step = max(1, int(math.ceil(n / MAX_PLOT_POINTS))) if n else 1
        t, v, i, q = self.t[::step], self.v[::step], self.i[::step], self.q[::step]
        if "v_t" in self.lines:
            self.lines["v_t"].set_data(t, v)
        if "i_t" in self.lines:
            self.lines["i_t"].set_data(t, i)
        if "v_q" in self.lines:
            self.lines["v_q"].set_data(q, v)
        if self.follow:
            self._apply_limits()
        self.canvas.draw_idle()

    def _refresh_plot(self):
        if self.session_active or self._plot_dirty:
            self._update_plot_data()
            self._plot_dirty = self.session_active
        self.after(PLOT_REFRESH_MS, self._refresh_plot)

    # ---------------------------------------------------- Exportar imagens

    def _session_cutoff_v(self):
        if self.summary and "cutoff_mV" in self.summary:
            val = to_float(self.summary["cutoff_mV"])
            if not math.isnan(val):
                return val / 1000.0
        return self.fw_cutoff_mv / 1000.0 if self.fw_cutoff_mv else None

    def _export_subtitle(self):
        start = self.session_start or dt.datetime.now()
        parts = [start.strftime("%d/%m/%Y %H:%M")]
        s = self.summary or {}
        if s:
            parts.append(f"fim: {s.get('end', '?')}")
            q, e = to_float(s.get("q_Ah", "nan")), to_float(s.get("e_Wh", "nan"))
        elif self.rows:
            q, e = self.rows[-1]["q_Ah"], self.rows[-1]["e_Wh"]
            parts.append("teste incompleto")
        else:
            q = e = float("nan")
        if not math.isnan(q):
            parts.append(f"{q:.3f} Ah")
        if not math.isnan(e):
            parts.append(f"{e:.2f} Wh")
        c = self._session_cutoff_v()
        if c:
            parts.append(f"cutoff {c:.2f} V")
        return "  |  ".join(parts)

    def _build_export_figure(self, key):
        """Figura independente (um grafico), com todos os pontos da sessao."""
        n = len(self.t)
        step = max(1, int(math.ceil(n / MAX_EXPORT_POINTS))) if n else 1
        t, v, i, q = self.t[::step], self.v[::step], self.i[::step], self.q[::step]
        fig = Figure(figsize=(9, 6), dpi=100, layout="constrained")
        ax = fig.add_subplot(111)
        ax.grid(True, alpha=0.3)
        cutoff = self._session_cutoff_v()
        dur = to_float((self.summary or {}).get("dur_s", "nan"))

        if key == "v_t":
            main = "Tensão da bateria × tempo"
            ax.plot(t, v, color="tab:blue", lw=1.3)
            ax.set_ylabel("V bateria (V)")
        elif key == "i_t":
            main = "Corrente de descarga × tempo"
            ax.plot(t, i, color="tab:orange", lw=1.3)
            ax.set_ylabel("Corrente (A)")
        else:
            main = "Curva de descarga: tensão × capacidade"
            ax.plot(q, v, color="tab:green", lw=1.3)
            ax.set_xlabel("Capacidade descarregada (Ah)")
            ax.set_ylabel("V bateria (V)")

        if key in TIME_KEYS:
            style_time_axis(ax)
            ax.set_xlabel("Tempo desde o início da descarga")
            if not math.isnan(dur):
                ax.axvline(dur, color="gray", ls=":", lw=1, label="fim da descarga")
        if key in ("v_t", "v_q") and cutoff:
            ax.axhline(cutoff, color="tab:red", ls="--", lw=1, label=f"cutoff {cutoff:.2f} V")
        if ax.get_legend_handles_labels()[0]:
            ax.legend(loc="best", fontsize=9)
        ax.set_title(f"{main}\n{self._export_subtitle()}", fontsize=11)
        return fig

    def _export_images(self):
        if not self.rows:
            messagebox.showinfo("Exportar imagens", "Nenhum dado de sessão para exportar.")
            return
        stamp = (self.session_start or dt.datetime.now()).strftime("%Y%m%d_%H%M%S")
        if self.csv_path:
            initial = os.path.splitext(os.path.basename(self.csv_path))[0]
        else:
            initial = f"descarga_{stamp}"
        init_dir = self.out_dir.get() if os.path.isdir(self.out_dir.get()) else None
        base = filedialog.asksaveasfilename(
            title="Nome-base das imagens (serão salvos 3 arquivos PNG)",
            defaultextension=".png", filetypes=[("PNG", "*.png")],
            initialfile=initial, initialdir=init_dir, confirmoverwrite=False)
        if not base:
            return
        stem = os.path.splitext(base)[0]
        targets = [(f"{stem}_{name}.png", key) for key, _, name in PLOT_DEFS]
        existing = [p for p, _ in targets if os.path.exists(p)]
        if existing and not messagebox.askyesno(
                "Sobrescrever?",
                "Já existem:\n" + "\n".join(os.path.basename(p) for p in existing)
                + "\n\nSobrescrever?"):
            return
        saved = []
        try:
            for path, key in targets:
                fig = self._build_export_figure(key)
                fig.savefig(path, dpi=200)
                saved.append(path)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Exportar imagens", f"Falha ao gravar:\n{exc}")
            return
        for p in saved:
            self._log(f"Imagem salva: {p}", "evt")

    # ------------------------------------------------------------- Serial

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_cb.get():
            self.port_cb.set(ports[0])

    def _connect(self):
        port = self.port_cb.get()
        if not port:
            messagebox.showwarning("Porta", "Selecione uma porta serial.")
            return
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = BAUD
            ser.timeout = 0.2
            ser.dtr = False      # evita (quando possivel) o reset automatico da ESP32
            ser.rts = False
            ser.open()
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("Serial", f"Não foi possível abrir {port}:\n{exc}")
            return
        self.ser = ser
        self.rx_stop.clear()
        self.rx_thread = threading.Thread(target=self._reader, daemon=True)
        self.rx_thread.start()
        self.lbl_conn.configure(text=f"conectado: {port} @ {BAUD}", foreground="#060")
        self._log(f"Conectado a {port}", "evt")
        self.after(600, lambda: self._send("STATUS"))
        self._update_buttons()

    def _disconnect(self):
        if self.session_active and self.fw_state in ("REST", "DISCHARGE", "RECOVERY"):
            if not messagebox.askyesno(
                    "Teste em andamento",
                    "Há um teste em andamento. O firmware continua e faz o cutoff sozinho, "
                    "mas fechar a porta pode reiniciar a ESP32 (DTR/RTS) e interromper o teste.\n\n"
                    "Desconectar mesmo assim?"):
                return
        self._close_port()
        self._end_session("desconectado")

    def _close_port(self):
        self.rx_stop.set()
        if self.rx_thread is not None:
            self.rx_thread.join(timeout=1.0)
            self.rx_thread = None
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None
        self.lbl_conn.configure(text="desconectado", foreground="#a00")
        self.fw_state = "-"
        self._update_buttons()

    def _reader(self):
        buf = b""
        while not self.rx_stop.is_set():
            try:
                chunk = self.ser.read(256)
            except (serial.SerialException, OSError, AttributeError):
                self.rx_queue.put(("ERRCONN", "porta serial perdida"))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", errors="replace").strip("\r")
                if text:
                    self.rx_queue.put(("LINE", text))
            if len(buf) > 4096:
                buf = b""

    def _send(self, text):
        if self.ser is None:
            return False
        try:
            self.ser.write((text + "\n").encode("ascii"))
            self._log(f"> {text}", "tx")
            return True
        except (serial.SerialException, OSError) as exc:
            self._log(f"Falha ao enviar '{text}': {exc}", "err")
            return False

    # ----------------------------------------------------------- Comandos

    def _cmd_tare(self):
        if messagebox.askokcancel(
                "Tara",
                "A carga eletrônica deve estar DESLIGADA/desconectada da corrente e nenhuma "
                "corrente pode estar passando pelo ACS758.\n\nExecutar a tara?"):
            self._send("TARE")

    def _cmd_start(self):
        cutoff = self.fw_cutoff_mv / 1000.0 if self.fw_cutoff_mv else None
        msg = ("Conferir antes de iniciar:\n"
               " - bateria ligada ao PWR_PAD_BATT e carga ao PWR_PAD_SENS;\n"
               " - retorno da carga direto ao negativo da bateria;\n"
               " - GND da bateria ligado ao GND da GCM;\n"
               " - GCM alimentada pela fonte de bancada.\n\n")
        if cutoff:
            msg += f"Cutoff atual no firmware: {cutoff:.2f} V.\n\n"
        msg += "Iniciar a descarga?"
        if not messagebox.askokcancel("START", msg):
            return
        if not self._begin_session():
            return
        self._send("START")

    def _cmd_stop(self):
        if messagebox.askokcancel("STOP", "Interromper o teste? A carga será desligada."):
            self._send("STOP")

    def _cmd_reset(self):
        self._send("RESET")

    def _cmd_cutoff(self):
        text = self.cutoff_var.get().strip().replace(",", ".")
        try:
            volts = float(text)
        except ValueError:
            messagebox.showerror("Cutoff", "Valor inválido. Use volts, por exemplo 12.00")
            return
        self._send(f"CUTOFF {int(round(volts * 1000))}")

    # ------------------------------------------------------------- Sessao

    def _has_unsaved(self):
        """Ha dados em memoria que nao foram para nenhum arquivo?"""
        return bool(self.rows) and not self.csv_path and not self.exported

    def _confirm_unsaved(self, what):
        """Pergunta se deve salvar dados nao salvos. Retorna False se o usuario cancelar."""
        if not self._has_unsaved():
            return True
        ans = messagebox.askyesnocancel(
            "Dados não salvos",
            f"A sessão atual ({len(self.rows)} linhas) não foi salva em CSV.\n\n"
            f"Salvar antes de {what}?")
        if ans is None:
            return False
        if ans:
            return self._export_csv()
        return True

    def _begin_session(self):
        if not self._confirm_unsaved("iniciar um novo teste"):
            return False
        self._end_session("nova sessao", quiet=True)
        self.exported = False
        self.session_active = True
        self.session_start = dt.datetime.now()
        self.rows = []
        self.events = []
        self.t, self.v, self.i, self.q = [], [], [], []
        self.ext = self._new_ext()
        self.summary = None
        self.csv_path = None
        self.rows_since_flush = 0
        self.follow = True
        self.lbl_follow.configure(text="")
        self._reset_limits()
        if self.autosave.get():
            try:
                os.makedirs(self.out_dir.get(), exist_ok=True)
                stamp = self.session_start.strftime("%Y%m%d_%H%M%S")
                self.csv_path = os.path.join(self.out_dir.get(), f"descarga_{stamp}.csv")
                self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(["host_time"] + D_COLUMNS)
                self.lbl_file.configure(text=os.path.basename(self.csv_path))
            except OSError as exc:
                self.csv_file = None
                self.csv_writer = None
                self.csv_path = None
                messagebox.showwarning("Auto-save", f"Não foi possível criar o CSV:\n{exc}")
        else:
            self.lbl_file.configure(text="(auto-save desligado)")
        self._update_plot_data(redraw=True)
        return True

    def _end_session(self, reason, quiet=False):
        """Fecha o CSV e grava o resumo. Os dados em memoria permanecem para exportar."""
        was_active = self.session_active
        self.session_active = False
        self._plot_dirty = True      # um ultimo redesenho com os pontos finais
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except OSError:
                pass
            self.csv_file = None
            self.csv_writer = None
        if was_active and self.csv_path and self.events:
            self._write_summary_file()
        if was_active and not quiet:
            self._log(f"Sessão encerrada ({reason}); {len(self.rows)} linhas em memória.", "evt")

    def _write_summary_file(self):
        path = os.path.splitext(self.csv_path)[0] + "_resumo.txt"
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(f"Arquivo de dados: {os.path.basename(self.csv_path)}\n")
                if self.summary:
                    f.write("\nRESUMO\n")
                    for k, v in self.summary.items():
                        f.write(f"  {k} = {v}\n")
                f.write("\nEVENTOS\n")
                for line in self.events:
                    f.write(f"  {line}\n")
        except OSError as exc:
            self._log(f"Não foi possível gravar o resumo: {exc}", "err")

    def _record_row(self, row):
        self.rows.append(row)
        self.t.append(row["t_test_s"])
        self.v.append(row["v_batt_V"])
        self.i.append(row["i_A"])
        self.q.append(row["q_Ah"])
        self._update_ext(row)
        if self.csv_writer is not None:
            host = dt.datetime.now().isoformat(timespec="milliseconds")
            self.csv_writer.writerow([host] + [row[c] if c == "state" else repr(row[c])
                                                for c in D_COLUMNS])
            self.rows_since_flush += 1
            if self.rows_since_flush >= 20:
                self.csv_file.flush()
                self.rows_since_flush = 0

    def _export_csv(self):
        if not self.rows:
            messagebox.showinfo("Exportar", "Nenhum dado de sessão para exportar.")
            return False
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=dt.datetime.now().strftime("descarga_%Y%m%d_%H%M%S.csv"),
            initialdir=self.out_dir.get() if os.path.isdir(self.out_dir.get()) else None)
        if not path:
            return False
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(D_COLUMNS)
                for r in list(self.rows):
                    w.writerow([r[c] if c == "state" else repr(r[c]) for c in D_COLUMNS])
            self.exported = True
            self._log(f"Sessão exportada: {path} ({len(self.rows)} linhas)", "evt")
            return True
        except OSError as exc:
            messagebox.showerror("Exportar", f"Falha ao gravar:\n{exc}")
            return False

    def _choose_dir(self):
        d = filedialog.askdirectory(initialdir=self.out_dir.get() or None)
        if d:
            self.out_dir.set(d)
            self._save_config()

    # ---------------------------------------------------- Fila / protocolo

    def _drain_queue(self):
        try:
            for _ in range(500):
                kind, payload = self.rx_queue.get_nowait()
                if kind == "ERRCONN":
                    self._log(payload, "err")
                    self._close_port()
                    self._end_session("porta perdida")
                    messagebox.showerror("Serial", "A porta serial foi perdida.")
                else:
                    self._handle_line(payload)
        except queue.Empty:
            pass
        self.after(POLL_MS, self._drain_queue)

    def _handle_line(self, line):
        kind, payload = parse_line(line)
        if kind == "D":
            self._handle_data(payload)
        elif kind == "E":
            self._handle_event(line, *payload)
        elif kind == "S":
            self._handle_status(payload)
        elif kind == "LOG":
            self._log(line, "raw")

    def _handle_data(self, row):
        prev_state = self.fw_state
        self.fw_state = row["state"]
        self.live_vars["state"].set(row["state"])
        self.live_vars["v"].set(f"{row['v_batt_V']:.3f} V")
        self.live_vars["i"].set(f"{row['i_A']:.3f} A")
        self.live_vars["p"].set(f"{row['p_W']:.1f} W")
        self.live_vars["q"].set(f"{row['q_Ah']:.4f} Ah")
        self.live_vars["e"].set(f"{row['e_Wh']:.3f} Wh")
        self.live_vars["t"].set(self._fmt_time(row["t_test_s"]))
        if self.fw_state != prev_state:
            self._update_buttons()

        if self.session_active and self.fw_state in SESSION_STATES:
            self._record_row(row)
            if self.fw_state in END_STATES:
                self._end_session(f"firmware em {self.fw_state}")
        elif self.session_active and self.rows and self.fw_state in ("READY", "IDLE"):
            self._end_session("abortado antes de ligar a carga")   # STOP durante o REST

    def _handle_event(self, line, name, details):
        tag = "err" if name in ("FAULT", "ERR") else "evt"
        if name == "WARN" and details.startswith("ADC_ERR"):
            return        # evita inundar o log; o firmware aborta se persistir
        self._log(line, tag)
        if self.session_active:
            self.events.append(line)
        if name == "SUMMARY":
            _, _, kv = line.partition("E;SUMMARY;")
            self.summary = parse_kv(kv)
            self._show_summary()
        elif name == "FAULT":
            messagebox.showerror("FAULT no firmware", f"Teste abortado: {details}\nA carga foi desligada.")
        elif name == "ERR" and self.session_active and details.startswith("START_REQUIRES"):
            self._end_session("START recusado", quiet=True)

    def _handle_status(self, kv):
        if "state" in kv:
            self.fw_state = kv["state"]
        try:
            self.fw_cutoff_mv = int(kv["cutoff_mV"])
            self.fw_limits = (int(kv["cutoff_min_mV"]), int(kv["cutoff_max_mV"]))
            self.lbl_cutoff_fw.configure(
                text=f"(firmware: {self.fw_cutoff_mv / 1000:.2f} V; faixa "
                     f"{self.fw_limits[0] / 1000:.1f}-{self.fw_limits[1] / 1000:.1f} V)")
            self._set_cutoff_lines()
            self._plot_dirty = True
        except (KeyError, ValueError):
            pass
        self._update_buttons()

    def _show_summary(self):
        s = self.summary or {}

        def f(key, unit="", scale=1.0, nd=3):
            try:
                x = float(s[key]) * scale
                return "n/d" if math.isnan(x) else f"{x:.{nd}f}{unit}"
            except (KeyError, ValueError):
                return "n/d"

        text = (f"Fim: {s.get('end', '?')}\n"
                f"Duração: {self._fmt_time(to_float(s.get('dur_s', 'nan')))}\n"
                f"Capacidade: {f('q_Ah', ' Ah', nd=4)}\n"
                f"Energia: {f('e_Wh', ' Wh')}\n"
                f"Corrente média: {f('i_avg_A', ' A')}\n"
                f"V repouso: {f('v_rest_V', ' V')}   V no cutoff: {f('v_cut_V', ' V')}\n"
                f"V recuperada: {f('v_rec_V', ' V')}\n"
                f"R interna (estimada): {f('rint_mohm', ' mOhm', nd=1)}\n"
                f"Zero do ACS758 no fim: {f('i_zero_end_mA', ' mA', nd=1)} "
                f"(deriva de offset; ideal ~0)")
        self._log("---- RESUMO ----\n" + text, "evt")
        messagebox.showinfo("Resumo do teste", text)

    # --------------------------------------------------------------- Util

    @staticmethod
    def _fmt_time(seconds):
        if seconds is None or math.isnan(seconds):
            return "-"
        s = int(seconds)
        return f"{s // 3600:d}:{(s % 3600) // 60:02d}:{s % 60:02d}"

    def _log(self, text, tag="raw"):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n", tag)
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 600:
            self.log.delete("1.0", f"{lines - 600}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _update_buttons(self):
        connected = self.ser is not None
        st = self.fw_state

        def en(widget, cond):
            widget.configure(state="normal" if (connected and cond) else "disabled")
        en(self.btn_tare, st in ("IDLE", "READY", "DONE", "FAULT"))
        en(self.btn_start, st == "READY")
        en(self.btn_stop, st in ("REST", "DISCHARGE", "RECOVERY"))
        en(self.btn_reset, st in ("DONE", "FAULT"))
        en(self.btn_cutoff, st not in ("REST", "DISCHARGE"))
        self.ent_cutoff.configure(
            state="normal" if (connected and st not in ("REST", "DISCHARGE")) else "disabled")
        self.btn_connect.configure(state="disabled" if connected else "normal")
        self.btn_disconnect.configure(state="normal" if connected else "disabled")

    def _on_close(self):
        if self.ser is not None and self.fw_state in ("REST", "DISCHARGE", "RECOVERY"):
            if not messagebox.askyesno(
                    "Teste em andamento",
                    "Há um teste em andamento. O firmware faz o cutoff sozinho, mas fechar a "
                    "porta serial pode reiniciar a ESP32 e interromper o teste.\n\nFechar mesmo assim?"):
                return
        if not self._confirm_unsaved("fechar"):
            return
        self._save_config()
        self._end_session("janela fechada", quiet=True)
        self._close_port()
        self.destroy()


def main():
    BatteryMonitor().mainloop()


if __name__ == "__main__":
    main()