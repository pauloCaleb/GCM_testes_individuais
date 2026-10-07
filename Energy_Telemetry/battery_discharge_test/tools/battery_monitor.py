#!/usr/bin/env python3
"""
battery_monitor.py — Monitor do teste de descarga da bateria (GCM-PI2-2026.2), v2

Conversa com o firmware `battery_discharge_test` (ESP32) por uma porta serial: cabo USB (CH340)
ou Bluetooth (SPP; o Windows cria uma porta COM ao parear o ESP32 "GCM-BATT").

Abas:
  Teste        Tara/START/STOP/Reset/cutoff, gráficos ao vivo, CSV e imagens;
  Calibração   ganho/offset de tensão e ganho de corrente (gravados no firmware);
  Análise      compara CSVs: Ah/Wh até tensões fixas, curvas sobrepostas, compensação por R0.

O cutoff é decidido pelo FIRMWARE: fechar este programa ou perder o link não interrompe o teste.
Ao reconectar, o app se anexa ao teste em andamento e recupera os pontos perdidos do log da
flash do ESP32 (1 Hz). Manual: MANUAL_USO_E_CALIBRACAO.md.

Dependências: pip install pyserial matplotlib numpy
"""

import csv
import datetime as dt
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports
import matplotlib

matplotlib.use("TkAgg")
from matplotlib import ticker  # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from bm_core import (  # noqa: E402
    APP_DIR, APP_VERSION, AXIS_LIMITS, BAUD, CONFIG_PATH, D_COLUMNS, END_STATES, META_KEYS,
    MAX_EXPORT_POINTS, MAX_PLOT_POINTS, PLOT_DEFS, PLOT_REFRESH_MS, POLL_MS, SESSION_STATES,
    START_MAXIMIZED, STATE_ORDER, TIME_KEYS, LimitTracker, PlotToolbar, parse_kv, parse_line,
    style_time_axis, to_float)
from bm_calibration import CalibrationTab  # noqa: E402
from bm_analysis import AnalysisTab  # noqa: E402

RESET_REASONS = {1: "energia ligada", 2: "reset externo", 3: "reset por software", 4: "falha (panic)",
                 5: "watchdog de interrupção", 6: "watchdog da task", 7: "watchdog", 8: "deep sleep",
                 9: "queda de tensão (brownout)", 10: "SDIO"}


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

        # Serial / Bluetooth (porta COM)
        self.ser = None
        self.port_name = ""
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.rx_queue = queue.Queue()
        self.connecting = False
        self.reconnect_at = None          # time.time() da proxima tentativa automatica

        # Firmware
        self.fw_state = "-"
        self.fw_cutoff_mv = None
        self.fw_limits = (None, None)
        self.status = {}
        self.meta = {}
        self.last_t_boot = None

        # Sessao (dados gravados desde o START ou desde que o app se anexou ao teste)
        self.session_active = False
        self.session_start = None
        self.rows = []                    # lista de dicts
        self.events = []                  # linhas de evento da sessao
        self.csv_want = False
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.session_dir = None           # pasta do teste: <saida>/descarga_AAAAMMDD_HHMMSS/
        self.session_suffix = ""          # "_logflash" para sessoes importadas do log da flash
        self.summary = None
        self.rows_since_flush = 0
        self.exported = False
        self.attach_declined = False
        self.last_seq = None
        self.lost_lines = 0
        self.recovered_rows = 0
        self.resync = None                # dict enquanto recupera o log da flash (anexar / reconectar)
        self.dump_rows = []
        self.dump_purpose = None          # "resync" ou "import"
        self._plot_dirty = False

        # Dados para o grafico (listas paralelas)
        self.t = []
        self.v = []
        self.i = []
        self.q = []

        # Graficos
        self.follow = True                # False apos zoom/pan manual; Home volta a True
        self.plot_from = 0                # indice da 1a amostra mostrada (Limpar graficos avanca isto)
        self.axes = {}
        self.lines = {}
        self.cut_lines = []
        self.trackers = {k: LimitTracker(k) for k in ("time", "cap", "volt", "cur")}
        self.ext = self._new_ext()        # min/max correntes dos dados (evita varrer as listas)
        self._applied = {}

        cfg = self._load_config()
        self.out_dir = tk.StringVar(value=APP_DIR)   # sempre a pasta do programa ao abrir
        self.autosave = tk.BooleanVar(value=cfg.get("autosave", True))
        self.csv_meta = tk.BooleanVar(value=cfg.get("csv_meta", True))
        self.auto_reconnect = tk.BooleanVar(value=cfg.get("auto_reconnect", True))
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
        data = {"autosave": bool(self.autosave.get()), "csv_meta": bool(self.csv_meta.get()),
                "auto_reconnect": bool(self.auto_reconnect.get())}
        for key, var in self.show.items():
            data["show_" + key] = bool(var.get())
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except OSError:
            pass

    # ------------------------------------------------------------------ UI

    def _build_ui(self):
        # --- Conexao (porta serial USB ou COM Bluetooth)
        conn = ttk.LabelFrame(self, text="Conexão (cabo USB ou Bluetooth — ambos aparecem como porta COM)")
        conn.pack(fill="x", padx=8, pady=(8, 2))
        ttk.Label(conn, text="Porta:").pack(side="left", padx=(8, 2))
        self.port_cb = ttk.Combobox(conn, width=22, state="readonly")
        self.port_cb.pack(side="left", padx=2, pady=4)
        ttk.Button(conn, text="Atualizar", command=self._refresh_ports).pack(side="left", padx=2)
        self.btn_connect = ttk.Button(conn, text="Conectar", command=self._connect)
        self.btn_connect.pack(side="left", padx=(10, 2))
        self.btn_disconnect = ttk.Button(conn, text="Desconectar", command=self._disconnect)
        self.btn_disconnect.pack(side="left", padx=2)
        ttk.Checkbutton(conn, text="Reconectar automaticamente", variable=self.auto_reconnect,
                        command=self._save_config).pack(side="left", padx=10)
        self.lbl_conn = ttk.Label(conn, text="desconectado", foreground="#a00")
        self.lbl_conn.pack(side="left", padx=8)

        # --- Abas
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=4, pady=4)
        tab_test = ttk.Frame(self.nb)
        self.nb.add(tab_test, text="  Teste  ")
        self.cal_tab = CalibrationTab(self.nb, self)
        self.nb.add(self.cal_tab, text="  Calibração  ")
        self.ana_tab = AnalysisTab(self.nb, self)
        self.nb.add(self.ana_tab, text="  Análise  ")

        # --- Controle
        ctl = ttk.LabelFrame(tab_test, text="Controle do teste")
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
        live = ttk.LabelFrame(tab_test, text="Medidas")
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
        logf = ttk.LabelFrame(tab_test, text="Eventos / log serial")
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
        files = ttk.LabelFrame(tab_test, text="Dados")
        files.pack(side="bottom", fill="x", padx=8, pady=4)
        ttk.Checkbutton(files, text="Auto-salvar (uma pasta por teste)", variable=self.autosave,
                        command=self._save_config).pack(side="left", padx=8, pady=4)
        ttk.Checkbutton(files, text="Metadados no CSV", variable=self.csv_meta,
                        command=self._save_config).pack(side="left", padx=4)
        ttk.Entry(files, textvariable=self.out_dir, width=40).pack(side="left", padx=2)
        ttk.Button(files, text="Pasta...", command=self._choose_dir).pack(side="left", padx=2)
        self.btn_log = ttk.Button(files, text="Baixar log da flash", command=self._import_log)
        self.btn_log.pack(side="left", padx=10)
        ttk.Button(files, text="Abrir pasta do teste", command=self._open_folder).pack(side="left", padx=4)
        self.lbl_file = ttk.Label(files, text="")
        self.lbl_file.pack(side="left", padx=8)

        # --- Graficos
        plot = ttk.Frame(tab_test)
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
        self.btn_clear = ttk.Button(opts, text="Limpar gráficos", command=self._clear_plots)
        self.btn_clear.pack(side="right", padx=4)
        self.btn_showall = ttk.Button(opts, text="Mostrar tudo", command=self._show_all_plots)
        self.btn_showall.pack(side="right", padx=4)
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

    def _update_view_label(self):
        parts = []
        if not self.follow:
            parts.append("vista manual — Home retoma o acompanhamento")
        if self.plot_from > 0:
            parts.append("gráfico limpo — os dados anteriores continuam no CSV e nas imagens")
        self.lbl_follow.configure(text="   |   ".join(parts))
        self.btn_showall.configure(state="normal" if self.plot_from > 0 else "disabled")

    def _on_manual_view(self):
        self.follow = False
        self._update_view_label()

    def _on_home(self):
        self.follow = True
        self._update_view_label()
        self._reset_limits()
        self._plot_dirty = True

    def _clear_plots(self):
        """Esvazia so a VISTA: os dados da sessao (CSV, resumo, imagens, Analise) ficam intactos."""
        if not self.t or self.plot_from >= len(self.t):
            return
        if self.session_active and self.fw_state in ("REST", "DISCHARGE", "RECOVERY"):
            if not messagebox.askokcancel(
                    "Limpar gráficos",
                    "O teste continua e o CSV segue sendo gravado. Só os gráficos serão limpos "
                    "(os pontos anteriores continuam no CSV e nas imagens exportadas).\n\nLimpar?"):
                return
        self.plot_from = len(self.t)
        self.ext = self._new_ext()
        self.follow = True
        self._reset_limits()
        self._update_view_label()
        self._update_plot_data()

    def _show_all_plots(self):
        self.plot_from = 0
        self.ext = self._new_ext()
        for r in self.rows:
            self._update_ext(r)
        self.follow = True
        self._reset_limits()
        self._update_view_label()
        self._update_plot_data()

    @staticmethod
    def _new_ext():
        return {"t_min": None, "t_max": None, "q_min": None, "q_max": None,
                "v_min": None, "v_max": None, "i_min": None, "i_max": None}

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
        lo("t_min", row["t_test_s"])
        hi("t_max", row["t_test_s"])
        lo("q_min", row["q_Ah"])
        hi("q_max", row["q_Ah"])
        lo("v_min", row["v_batt_V"])
        hi("v_max", row["v_batt_V"])
        lo("i_min", row["i_A"])
        hi("i_max", row["i_A"])

    def _apply_limits(self):
        """Aplica limites estaveis: os eixos so se movem quando os dados estouram a faixa."""
        c = self.fw_cutoff_mv / 1000.0 if self.fw_cutoff_mv else None
        e, tr = self.ext, self.trackers
        tr["time"].update(e["t_min"], e["t_max"])
        tr["cap"].update(e["q_min"], e["q_max"])
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
        self._update_view_label()
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
        a = self.plot_from
        n = len(self.t) - a
        step = max(1, int(math.ceil(n / MAX_PLOT_POINTS))) if n > 0 else 1
        t, v, i, q = self.t[a::step], self.v[a::step], self.i[a::step], self.q[a::step]
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
            qc = to_float(s.get("q_corr_Ah", "nan"))
        elif self.rows:
            q, e, qc = self.rows[-1]["q_Ah"], self.rows[-1]["e_Wh"], float("nan")
            parts.append("teste incompleto")
        else:
            q = e = qc = float("nan")
        if not math.isnan(q):
            parts.append(f"{q:.3f} Ah" + ("" if math.isnan(qc) else f" ({qc:.3f} corr.)"))
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
        init_dir = self.session_dir if (self.session_dir and os.path.isdir(self.session_dir)) \
            else (self.out_dir.get() if os.path.isdir(self.out_dir.get()) else None)
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
        try:
            self._save_images(stem)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Exportar imagens", f"Falha ao gravar:\n{exc}")

    def _save_images(self, stem):
        """Grava <stem>_tensao_tempo.png, <stem>_corrente_tempo.png e <stem>_tensao_capacidade.png."""
        saved = []
        for key, _, name in PLOT_DEFS:
            path = f"{stem}_{name}.png"
            fig = self._build_export_figure(key)
            fig.savefig(path, dpi=200)
            saved.append(path)
        for p in saved:
            self._log(f"Imagem salva: {p}", "evt")
        return saved

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
        self.reconnect_at = None
        self._open_port_async(port, user=True)

    def _open_port_async(self, port, user):
        """Abre a porta numa thread: portas Bluetooth (COM) podem levar varios segundos."""
        if self.connecting:
            return
        self.connecting = True
        self.port_name = port
        self.lbl_conn.configure(text=f"conectando a {port}...", foreground="#a60")
        self._update_buttons()

        def work():
            try:
                ser = serial.Serial()
                ser.port = port
                ser.baudrate = BAUD
                ser.timeout = 0.2
                ser.dtr = False      # evita (quando possivel) o reset automatico da ESP32
                ser.rts = False
                ser.open()
                self.rx_queue.put(("OPENED", ser))
            except (serial.SerialException, OSError, ValueError) as exc:
                self.rx_queue.put(("OPENERR", (str(exc), user)))
        threading.Thread(target=work, daemon=True).start()

    def _on_opened(self, ser):
        self.connecting = False
        self.ser = ser
        self.rx_stop.clear()
        self.rx_thread = threading.Thread(target=self._reader, daemon=True)
        self.rx_thread.start()
        self.lbl_conn.configure(text=f"conectado: {self.port_name} @ {BAUD}", foreground="#060")
        self._log(f"Conectado a {self.port_name}", "evt")
        self.last_seq = None
        self.after(500, lambda: self._send("SYNC"))
        if self.session_active:                      # reconexao no meio do teste: preenche o buraco
            last = self.rows[-1] if self.rows else None
            self._start_resync("gap", self._row_key(last) if last else None)
        self._update_buttons()

    def _disconnect(self):
        if self.session_active and self.fw_state in ("REST", "DISCHARGE", "RECOVERY"):
            if not messagebox.askyesno(
                    "Teste em andamento",
                    "Há um teste em andamento. O firmware continua e faz o cutoff sozinho; ao "
                    "reconectar o app se anexa de novo. Em cabo USB, fechar a porta pode reiniciar "
                    "a ESP32 (DTR/RTS) e interromper o teste.\n\nDesconectar mesmo assim?"):
                return
        self.reconnect_at = None
        self._close_port()
        self._end_session("desconectado")
        self.lbl_conn.configure(text="desconectado", foreground="#a00")
        self._update_buttons()

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
        self.connecting = False
        if self.resync is not None:
            self._finish_resync()
        self.dump_purpose = None
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
        warm = ""
        if self.last_t_boot is not None and not math.isnan(self.last_t_boot):
            mins = self.last_t_boot / 60.0
            warm = f"\n\nGCM ligada há {mins:.0f} min."
            if mins < 10:
                warm += " Para o zero estabilizar, o ideal é esperar uns 10 min depois de ligar."
        if messagebox.askokcancel(
                "Tara",
                "A carga eletrônica deve estar DESLIGADA/desconectada da corrente e nenhuma "
                "corrente pode estar passando pelo ACS758." + warm + "\n\nExecutar a tara?"):
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
        if messagebox.askokcancel(
                "STOP",
                "Interromper o teste? A carga será desligada. O registro segue na recuperação até o "
                "fim (STOP de novo encerra na hora) e tudo é salvo na pasta do teste."):
            self._send("STOP")
            self._checkpoint_save("STOP")

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
        self._start_session_state()
        return True

    def _start_session_state(self, keep_info=False):
        """Zera os dados da sessao. keep_info preserva eventos/resumo ja recebidos (anexar/importar)."""
        self._end_session("nova sessao", quiet=True)
        self.exported = False
        self.session_active = True
        self.session_start = dt.datetime.now()
        self.rows = []
        if not keep_info:
            self.events = []
            self.summary = None
        self.t, self.v, self.i, self.q = [], [], [], []
        self.plot_from = 0               # "Limpar graficos" vale so para a sessao em que foi usado
        self.ext = self._new_ext()
        self.csv_path = None
        self.session_dir = None
        self.rows_since_flush = 0
        self.csv_want = bool(self.autosave.get())
        self.lost_lines = 0
        self.recovered_rows = 0
        self.follow = True
        self._update_view_label()
        self._reset_limits()
        self.lbl_file.configure(text="" if self.csv_want else "(auto-save desligado)")
        self._update_plot_data(redraw=True)

    @staticmethod
    def _row_key(r):
        return (STATE_ORDER.get(r["state"], 9), r["t_test_s"])

    def _attach_session(self):
        """O firmware ja esta num teste que este app nao iniciou: anexa e recupera o historico."""
        if not self._confirm_unsaved("anexar ao teste em andamento"):
            self.attach_declined = True
            return
        self._start_session_state(keep_info=True)
        self._log("Teste em andamento detectado: anexando e recuperando o histórico da flash...", "evt")
        self._start_resync("attach", None)

    def _start_resync(self, mode, from_key):
        """Pede o log da flash; os dados ao vivo ficam em buffer ate o download terminar."""
        self.resync = {"mode": mode, "buffer": [], "from_key": from_key, "last_rx": time.time()}
        self.dump_rows = []
        self.dump_purpose = "resync"
        self._send("LOG")

    def _finish_resync(self):
        rs = self.resync
        if rs is None:
            return
        self.resync = None
        self.dump_purpose = None
        buf = rs["buffer"]
        lo = rs["from_key"]
        hi = self._row_key(buf[0]) if buf else None
        added = 0
        for r in self.dump_rows:
            k = self._row_key(r)
            if (lo is not None and k <= lo) or (hi is not None and k >= hi):
                continue
            self._record_row(r)
            added += 1
        self.recovered_rows += added
        self.dump_rows = []
        for row in buf:
            if not self.session_active:
                break
            self._record_live(row)
        if added:
            self._log(f"{added} pontos recuperados do log da flash (1 Hz).", "evt")
        self._update_view_label()
        self._plot_dirty = True

    def _import_log(self):
        if self.ser is None:
            messagebox.showinfo("Log da flash", "Conecte-se ao ESP32 primeiro.")
            return
        if self.session_active:
            messagebox.showinfo("Log da flash", "Há uma sessão em andamento. Aguarde o fim ou desconecte.")
            return
        if not self._confirm_unsaved("baixar o log da flash"):
            return
        self.dump_rows = []
        self.dump_purpose = "import"
        self._log("Baixando o log da flash do último teste...", "evt")
        self._send("LOG")

    def _finish_import(self):
        rows, self.dump_rows, self.dump_purpose = self.dump_rows, [], None
        if not rows:
            messagebox.showinfo("Log da flash", "O log está vazio (nenhum teste gravado).")
            return
        self.session_suffix = "_logflash"
        self._start_session_state(keep_info=True)
        for r in rows:
            self._record_row(r)
        self.recovered_rows = len(rows)
        self._end_session("log da flash importado")
        self.session_suffix = ""
        self._log(f"{len(rows)} pontos importados do log (1 Hz). Use Salvar CSV, Imagens ou a aba Análise.", "evt")

    @staticmethod
    def _log_row_to_full(r):
        v, i = r["v_batt_V"], r["i_A"]
        nan = float("nan")
        return {"seq": None, "t_boot_s": nan, "t_test_s": r["t_test_s"], "state": r["state"],
                "v_batt_V": v, "i_A": i, "p_W": v * i, "q_Ah": r["q_Ah"], "e_Wh": r["e_Wh"],
                "ain0_V": nan, "ain1_V": nan, "from_log": True}

    def _record_live(self, row):
        self._record_row(row)
        if row["state"] in END_STATES:
            self._end_session(f"firmware em {row['state']}")

    def _end_session(self, reason, quiet=False):
        """Fecha o CSV e grava o resumo. Os dados em memoria permanecem para exportar."""
        was_active = self.session_active
        self.session_active = False
        self.resync = None
        self._plot_dirty = True      # um ultimo redesenho com os pontos finais
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except OSError:
                pass
            self.csv_file = None
            self.csv_writer = None
        if was_active and self.csv_path:
            self._save_outputs()
        if was_active and not quiet:
            where = f" Salvo em {self.session_dir}." if self.session_dir else ""
            self._log(f"Sessão encerrada ({reason}); {len(self.rows)} linhas.{where}", "evt")

    def _write_summary_file(self):
        path = os.path.splitext(self.csv_path)[0] + "_resumo.txt"
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(f"Arquivo de dados: {os.path.basename(self.csv_path)}\n")
                f.write(f"Gerado por battery_monitor {APP_VERSION}\n")
                if self.summary:
                    f.write("\nRESUMO\n")
                    for k, v in self.summary.items():
                        f.write(f"  {k} = {v}\n")
                f.write("\nMETADADOS\n")
                for k in META_KEYS:
                    if k in self.meta:
                        f.write(f"  {k} = {self.meta[k]}\n")
                f.write("\nQUALIDADE DOS DADOS\n")
                f.write(f"  linhas_perdidas_10Hz = {self.lost_lines}\n")
                f.write(f"  pontos_recuperados_do_log_1Hz = {self.recovered_rows}\n")
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
        if self.csv_want and self.csv_writer is None and self.csv_path is None:
            self._open_csv()
        if self.csv_writer is not None:
            host = "" if row.get("from_log") else dt.datetime.now().isoformat(timespec="milliseconds")
            self.csv_writer.writerow([host] + [self._cell(row, c) for c in D_COLUMNS])
            self.rows_since_flush += 1
            if self.rows_since_flush >= 20:
                self.csv_file.flush()
                self.rows_since_flush = 0

    def _save_outputs(self):
        """Grava na pasta do teste: resumo (com metadados e eventos) e as 3 imagens."""
        if not self.csv_path:
            return
        self._write_summary_file()
        self._auto_save_images()

    def _checkpoint_save(self, why):
        """Salva agora o que existe (CSV em disco, resumo e imagens); o fim do teste sobrescreve."""
        if not (self.session_active and self.csv_path):
            return
        try:
            if self.csv_file is not None:
                self.csv_file.flush()
        except OSError:
            pass
        self.lbl_file.configure(text="salvando...")
        self.update_idletasks()
        self._save_outputs()
        self.lbl_file.configure(text=os.path.basename(self.session_dir) + "/")
        self._log(f"Dados salvos ({why}) em {self.session_dir}", "evt")

    def _auto_save_images(self):
        if not self.csv_path or not any(r["state"] == "DISCHARGE" for r in self.rows):
            return                                          # sem descarga nao ha o que plotar
        try:
            self._save_images(os.path.splitext(self.csv_path)[0])
        except (OSError, ValueError) as exc:
            self._log(f"Não foi possível salvar as imagens: {exc}", "err")

    def _open_folder(self):
        path = self.session_dir if (self.session_dir and os.path.isdir(self.session_dir)) \
            else self.out_dir.get()
        try:
            if sys.platform.startswith("win"):
                os.startfile(path)                          # noqa: S606 (Windows)
            elif sys.platform == "darwin":
                subprocess.Popen(["open", path])
            else:
                subprocess.Popen(["xdg-open", path])
        except (OSError, AttributeError) as exc:
            messagebox.showerror("Abrir pasta", f"Não foi possível abrir:\n{path}\n{exc}")

    @staticmethod
    def _cell(row, c):
        v = row.get(c)
        if c == "state":
            return v
        if v is None or (isinstance(v, float) and math.isnan(v)):
            return ""
        if c == "seq":
            return str(int(v))
        return repr(v)

    def _meta_header_lines(self):
        start = (self.session_start or dt.datetime.now()).isoformat(timespec="seconds")
        lines = [f"battery_monitor {APP_VERSION}", f"inicio={start}"]
        lines += [f"{k}={self.meta[k]}" for k in META_KEYS if k in self.meta]
        return lines

    def _open_csv(self):
        try:
            stamp = (self.session_start or dt.datetime.now()).strftime("%Y%m%d_%H%M%S")
            base = f"descarga_{stamp}{self.session_suffix}"
            folder = os.path.join(self.out_dir.get(), base)
            n = 2
            while os.path.exists(folder):                    # dois testes no mesmo segundo
                folder = os.path.join(self.out_dir.get(), f"{base}_{n}")
                n += 1
            os.makedirs(folder)
            self.session_dir = folder
            self.csv_path = os.path.join(folder, os.path.basename(folder) + ".csv")
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
            if self.csv_meta.get():
                for line in self._meta_header_lines():
                    self.csv_file.write(f"# {line}\n")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["host_time"] + D_COLUMNS)
            self.lbl_file.configure(text=os.path.basename(folder) + "/")
        except OSError as exc:
            self.csv_file = None
            self.csv_writer = None
            self.csv_path = None
            self.session_dir = None
            self.csv_want = False
            messagebox.showwarning("Auto-save", f"Não foi possível criar o CSV:\n{exc}")

    def _export_csv(self):
        if not self.rows:
            messagebox.showinfo("Exportar", "Nenhum dado de sessão para exportar.")
            return False
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=(os.path.basename(self.csv_path) if self.csv_path
                         else dt.datetime.now().strftime("descarga_%Y%m%d_%H%M%S.csv")),
            initialdir=self.session_dir if (self.session_dir and os.path.isdir(self.session_dir))
            else (self.out_dir.get() if os.path.isdir(self.out_dir.get()) else None))
        if not path:
            return False
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                if self.csv_meta.get():
                    for line in self._meta_header_lines():
                        f.write(f"# {line}\n")
                w = csv.writer(f)
                w.writerow(D_COLUMNS)
                for r in list(self.rows):
                    w.writerow([self._cell(r, c) for c in D_COLUMNS])
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
                if kind == "OPENED":
                    self._on_opened(payload)
                elif kind == "OPENERR":
                    msg, user = payload
                    self.connecting = False
                    self.lbl_conn.configure(text="desconectado", foreground="#a00")
                    self._update_buttons()
                    if user:
                        messagebox.showerror("Serial", f"Não foi possível abrir {self.port_name}:\n{msg}")
                    elif self.auto_reconnect.get():
                        self.reconnect_at = time.time() + 3
                        self.lbl_conn.configure(text="reconectando...", foreground="#a60")
                elif kind == "ERRCONN":
                    self._log(payload, "err")
                    self._close_port()
                    if self.auto_reconnect.get() and self.port_name:
                        self.reconnect_at = time.time() + 3
                        self.lbl_conn.configure(text="link perdido — reconectando...", foreground="#a60")
                        self._update_buttons()
                    else:
                        self._end_session("porta perdida")
                        messagebox.showerror("Serial", "A porta serial foi perdida.")
                else:
                    self._handle_line(payload)
        except queue.Empty:
            pass
        if (self.reconnect_at is not None and time.time() >= self.reconnect_at
                and not self.connecting and self.ser is None):
            self.reconnect_at = None
            self._open_port_async(self.port_name, user=False)
        if self.resync is not None and time.time() - self.resync["last_rx"] > 15:
            self._log("O log da flash parou de chegar; seguindo com os dados ao vivo.", "err")
            self._finish_resync()
        self.after(POLL_MS, self._drain_queue)

    def _handle_line(self, line):
        kind, payload = parse_line(line)
        if kind == "D":
            self._handle_data(payload)
        elif kind == "E":
            self._handle_event(line, *payload)
        elif kind == "R":                      # evento reenviado pelo SYNC
            k2, p2 = parse_line(payload)
            if k2 == "E":
                self._handle_event(payload, *p2, replay=True)
        elif kind == "S":
            self._handle_status(payload)
        elif kind == "L":
            self.dump_rows.append(self._log_row_to_full(payload))
            if self.resync is not None:
                self.resync["last_rx"] = time.time()
            n = len(self.dump_rows)
            if n % 100 == 0:
                self.lbl_file.configure(text=f"recuperando o log da flash: {n} pontos...")
        elif kind == "LOG":
            self._log(line, "raw")

    def _handle_data(self, row):
        prev_state = self.fw_state
        self.fw_state = row["state"]
        self.last_t_boot = row["t_boot_s"]

        seq = row.get("seq")                    # detecta linhas perdidas (principalmente via Bluetooth)
        if seq is not None and not math.isnan(seq):
            seq = int(seq)
            if self.last_seq is not None and seq > self.last_seq + 1 and self.session_active:
                self.lost_lines += seq - self.last_seq - 1
            self.last_seq = seq

        self.live_vars["state"].set(row["state"])
        self.live_vars["v"].set(f"{row['v_batt_V']:.3f} V")
        self.live_vars["i"].set(f"{row['i_A']:.3f} A")
        self.live_vars["p"].set(f"{row['p_W']:.1f} W")
        self.live_vars["q"].set(f"{row['q_Ah']:.4f} Ah")
        self.live_vars["e"].set(f"{row['e_Wh']:.3f} Wh")
        self.live_vars["t"].set(self._fmt_time(row["t_test_s"]))
        self.cal_tab.update_live(row)
        if self.fw_state != prev_state:
            self._update_buttons()
        if self.fw_state in ("READY", "IDLE", "DONE", "FAULT"):
            self.attach_declined = False

        if (not self.session_active and self.ser is not None and not self.attach_declined
                and self.fw_state in ("REST", "DISCHARGE", "RECOVERY")):
            self._attach_session()

        if self.session_active and self.fw_state in SESSION_STATES:
            if self.resync is not None:
                buf = self.resync["buffer"]
                if not (row["state"] in END_STATES and buf and buf[-1]["state"] in END_STATES):
                    buf.append(row)
            else:
                self._record_live(row)
        elif (self.session_active and self.rows and self.resync is None
              and self.fw_state in ("READY", "IDLE")):
            self._end_session("abortado antes de ligar a carga")   # STOP durante o REST

    def _handle_event(self, line, name, details, replay=False):
        tag = "err" if name in ("FAULT", "ERR") else "evt"
        if name == "WARN" and details.startswith("ADC_ERR"):
            return        # evita inundar o log; o firmware aborta se persistir
        if not replay:
            self._log(line, tag)
        if name not in ("SYNC_BEGIN", "SYNC_END", "LOGDUMP_START", "LOGDUMP_END") and \
                (self.session_active or self.resync is not None) and line not in self.events:
            self.events.append(line)

        self.cal_tab.on_event(name, details)

        if name == "META":
            self.meta.update(parse_kv(details))
        elif name == "SUMMARY":
            _, _, kv = line.partition("E;SUMMARY;")
            self.summary = parse_kv(kv)
            if not replay:
                self._show_summary()
        elif name == "FAULT":
            if not replay:
                messagebox.showerror("FAULT no firmware", f"Teste abortado: {details}\nA carga foi desligada.")
        elif name == "BOOT":
            kv = parse_kv(details)
            code = int(to_float(kv.get("reset", "nan"))) if not math.isnan(to_float(kv.get("reset", "nan"))) else 0
            self.meta.update(kv)
            if self.session_active and self.fw_state in ("REST", "DISCHARGE", "RECOVERY", "-") \
                    and self.rows and not replay:
                why = RESET_REASONS.get(code, f"código {code}")
                self._end_session("ESP32 reiniciou")
                messagebox.showwarning(
                    "ESP32 reiniciou",
                    f"O ESP32 reiniciou durante o teste (motivo: {why}). A carga foi desligada.\n"
                    "Os dados já recebidos estão no CSV; os pontos a 1 Hz do teste continuam na flash — "
                    "use 'Baixar log da flash'.")
        elif name == "LOGDUMP_START":
            self.dump_rows = []
            if self.resync is not None:
                self.resync["last_rx"] = time.time()
        elif name == "LOGDUMP_END":
            self.lbl_file.configure(text=os.path.basename(self.csv_path) if self.csv_path else "")
            if self.dump_purpose == "resync" and self.resync is not None:
                self._finish_resync()
            elif self.dump_purpose == "import":
                self._finish_import()
        elif name == "ERR":
            if details.startswith("LOG_UNAVAILABLE"):
                if self.resync is not None:
                    self._finish_resync()
                self.dump_purpose = None
            elif self.session_active and details.startswith("START_REQUIRES"):
                self._end_session("START recusado", quiet=True)

    def _handle_status(self, kv):
        self.status = kv
        self.meta.update(kv)
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
        self.cal_tab.on_status(kv)
        self._update_buttons()

    def _show_summary(self):
        s = self.summary or {}

        def f(key, unit="", scale=1.0, nd=3):
            try:
                x = float(s[key]) * scale
                return "n/d" if math.isnan(x) else f"{x:.{nd}f}{unit}"
            except (KeyError, ValueError):
                return "n/d"

        text = (f"Fim: {s.get('end', '?')}   (teste nº {s.get('test_id', '?')})\n"
                f"Duração: {self._fmt_time(to_float(s.get('dur_s', 'nan')))}\n"
                f"Capacidade: {f('q_Ah', ' Ah', nd=4)}   corrigida pela deriva do zero: {f('q_corr_Ah', ' Ah', nd=4)}\n"
                f"Energia: {f('e_Wh', ' Wh')}   corrigida: {f('e_corr_Wh', ' Wh')}\n"
                f"Corrente média: {f('i_avg_A', ' A')}\n"
                f"V repouso: {f('v_rest_V', ' V')}   V no cutoff: {f('v_cut_V', ' V')}   "
                f"V recuperada: {f('v_rec_V', ' V')}\n"
                f"R0 ao ligar: {f('r0_on_mohm', ' mΩ', nd=1)}   ao desligar: {f('r0_off_mohm', ' mΩ', nd=1)}   "
                f"(janela de ~1,5 s: {f('rint_mohm', ' mΩ', nd=1)})\n"
                f"Zero do ACS758: início {f('i_zero_start_mA', ' mA', nd=1)}, fim {f('i_zero_end_mA', ' mA', nd=1)}, "
                f"deriva média {f('drift_mA', ' mA', nd=2)}\n"
                f"Linhas perdidas (10 Hz): {self.lost_lines}   pontos recuperados do log: {self.recovered_rows}")
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
        en(self.btn_log, not self.session_active)
        self.ent_cutoff.configure(
            state="normal" if (connected and st not in ("REST", "DISCHARGE")) else "disabled")
        self.btn_connect.configure(state="disabled" if (connected or self.connecting) else "normal")
        self.btn_disconnect.configure(
            state="normal" if (connected or self.connecting or self.reconnect_at is not None) else "disabled")
        self.cal_tab.refresh_state(connected, st)

    def _on_close(self):
        if self.ser is not None and self.fw_state in ("REST", "DISCHARGE", "RECOVERY"):
            if not messagebox.askyesno(
                    "Teste em andamento",
                    "Há um teste em andamento. O firmware faz o cutoff sozinho; ao abrir o app de novo "
                    "ele se anexa ao teste. Em cabo USB, fechar a porta pode reiniciar a ESP32 e "
                    "interromper o teste.\n\nFechar mesmo assim?"):
                return
        if not self._confirm_unsaved("fechar"):
            return
        self._save_config()
        self.reconnect_at = None
        self._end_session("janela fechada", quiet=True)
        self._close_port()
        self.destroy()


def main():
    BatteryMonitor().mainloop()


if __name__ == "__main__":
    main()
