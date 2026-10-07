"""
bm_core.py — protocolo, constantes e utilitarios de grafico do battery_monitor.

Contem: parser das linhas do firmware (D/E/S/H/L/R), eixo de tempo legivel, limites de eixo
estaveis (LimitTracker) e a barra de ferramentas do matplotlib com deteccao de zoom/Home.
"""

import math
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("TkAgg")
from matplotlib import ticker  # noqa: E402
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk  # noqa: E402

# Protocolo
# --------------------------------------------------------------------------

BAUD = 115200
D_COLUMNS = ["seq", "t_boot_s", "t_test_s", "state", "v_batt_V", "i_A", "p_W",
             "q_Ah", "e_Wh", "ain0_V", "ain1_V"]
L_COLUMNS = ["idx", "t_test_s", "state", "v_batt_V", "i_A", "q_Ah", "e_Wh"]
STATE_ORDER = {"REST": 0, "DISCHARGE": 1, "RECOVERY": 2, "DONE": 3, "FAULT": 3}
APP_VERSION = "2.0"
# chaves de metadados gravadas no cabecalho do CSV e no resumo
META_KEYS = ["fw", "test_id", "cutoff_mV", "v_ratio", "i_sens_uV_per_A", "v_gain_ppm", "v_off_uV",
             "i_gain_ppm", "zero_tare_V", "zero_V", "period_ms", "oversample", "relay_en", "bt"]
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

    tipo: 'D' (dict de dados), 'E' (nome, detalhes), 'S' (dict de status), 'H' (cabecalho),
          'L' (dict de um registro do log em flash), 'R' (linha E reenviada pelo SYNC; payload =
          texto da linha E original) ou 'LOG' (qualquer outra linha).
    """
    line = line.strip()
    if line.startswith("D;"):
        parts = line.split(";")
        if len(parts) == len(D_COLUMNS) + 1:
            row = {}
            for name, value in zip(D_COLUMNS, parts[1:]):
                row[name] = value if name == "state" else to_float(value)
            return "D", row
        return "LOG", line
    if line.startswith("L;"):
        parts = line.split(";")
        if len(parts) == len(L_COLUMNS) + 1:
            row = {}
            for name, value in zip(L_COLUMNS, parts[1:]):
                row[name] = value if name == "state" else to_float(value)
            return "L", row
        return "LOG", line
    if line.startswith("R;E;"):
        return "R", line[2:]
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
        self.start = 0.0         # origem do eixo (tempo/capacidade): 1a amostra visivel

    def update(self, dmin, dmax, cutoff=None):
        """Retorna True se o limite mudou."""
        return getattr(self, "_" + self.kind)(dmin, dmax, cutoff)

    def _time(self, dmin, dmax, cutoff):
        start = 0.0 if dmin is None else max(dmin, 0.0)
        dmax = start if dmax is None else max(dmax, start)
        if self.hi is not None and self.start == start and dmax <= start + (self.hi - start) * 0.98:
            return False
        target = max((dmax - start) * 1.5, 60.0)
        step = next((s for s in TimeLocator.STEPS if target / s <= 6), TimeLocator.STEPS[-1])
        self.start = start
        self.hi = start + math.ceil(target / step) * step
        self.lo = start - 0.01 * (self.hi - start)
        return True

    def _cap(self, dmin, dmax, cutoff):
        start = 0.0 if dmin is None else max(dmin, 0.0)
        dmax = start if dmax is None else max(dmax, start)
        if self.hi is not None and self.start == start and dmax <= start + (self.hi - start) * 0.98:
            return False
        self.start = start
        self.hi = start + nice_ceil(max((dmax - start) * 1.5, 0.2))
        self.lo = start - 0.01 * (self.hi - start)
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
