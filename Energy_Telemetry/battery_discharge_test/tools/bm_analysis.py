"""
bm_analysis.py — aba "Análise" do battery_monitor.

Abre um ou mais CSVs de descarga (formato novo, com cabeçalho de metadados "#", ou o antigo),
mostra uma tabela com Ah/Wh até tensões fixas e sobrepõe as curvas para comparar baterias
ou o mesmo pack ao longo do tempo. A opção "compensada por R0" soma I·R0 à tensão, aproximando
a curva em circuito aberto (remove a queda ôhmica).
"""

import csv
import math
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from bm_core import STATE_ORDER, parse_kv, style_time_axis, to_float  # noqa: F401

DEFAULT_THRESHOLDS = "14, 13.5, 13, 12.5, 12.2"
CURVES = ["Tensão × Ah", "Tensão × Ah (compensada por R0)", "Tensão × tempo", "Corrente × tempo"]
COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#e377c2", "#17becf"]


def read_summary_file(csv_path):
    """Le o _resumo.txt ao lado do CSV (secao RESUMO). Retorna dict de strings."""
    path = os.path.splitext(csv_path)[0] + "_resumo.txt"
    out = {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            in_summary = False
            for line in f:
                s = line.rstrip("\n")
                if s.strip() == "RESUMO":
                    in_summary = True
                    continue
                if in_summary and s and not s.startswith(" "):
                    break
                if in_summary and "=" in s:
                    k, _, v = s.partition("=")
                    out[k.strip()] = v.strip()
    except OSError:
        pass
    return out


def load_run(path):
    """Carrega um CSV de descarga. Usa so as linhas em DISCHARGE para curvas e totais."""
    meta, lines = {}, []
    with open(path, "r", newline="", encoding="utf-8") as f:
        for raw in f:
            if raw.startswith("#"):
                k, _, v = raw[1:].strip().partition("=")
                if k:
                    meta[k.strip()] = v.strip()
            elif raw.strip():
                lines.append(raw)
    reader = csv.DictReader(lines)
    cols = {k: [] for k in ("t", "v", "i", "q", "e")}
    names = {"t": "t_test_s", "v": "v_batt_V", "i": "i_A", "q": "q_Ah", "e": "e_Wh"}
    for r in reader:
        if r.get("state") != "DISCHARGE":
            continue
        for k, col in names.items():
            cols[k].append(to_float(r.get(col, "") or "nan"))
    if not cols["t"]:
        raise ValueError("nenhuma linha em DISCHARGE no arquivo")
    arr = {k: np.array(v, dtype=float) for k, v in cols.items()}
    return {"name": os.path.basename(path), "path": path, "meta": meta,
            "summary": read_summary_file(path), **arr}


def run_from_rows(rows, summary, meta, name="Sessão atual"):
    d = [r for r in rows if r.get("state") == "DISCHARGE"]
    if not d:
        raise ValueError("a sessão atual ainda não tem linhas em DISCHARGE")
    arr = {k: np.array([r[c] for r in d], dtype=float)
           for k, c in (("t", "t_test_s"), ("v", "v_batt_V"), ("i", "i_A"),
                        ("q", "q_Ah"), ("e", "e_Wh"))}
    return {"name": name, "path": "", "meta": dict(meta or {}), "summary": dict(summary or {}), **arr}


def run_r0_mohm(run):
    for key in ("r0_on_mohm", "rint_mohm"):
        x = to_float(run["summary"].get(key, "nan"))
        if not math.isnan(x):
            return x
    return 0.0


def compute_metrics(run, thresholds):
    t, v, q, e = run["t"], run["v"], run["q"], run["e"]
    out = {"dur": float(t[-1] - t[0]), "q": float(q[-1]), "e": float(e[-1]),
           "vmean": float(e[-1] / q[-1]) if q[-1] > 0 else float("nan"),
           "vmin": float(np.nanmin(v)), "thr": {}}
    for thr in thresholds:
        below = np.nonzero(v < thr)[0]
        out["thr"][thr] = (float(q[below[0]]), float(e[below[0]])) if below.size else None
    return out


class AnalysisTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.runs = []
        self.curve = tk.StringVar(value=CURVES[0])
        self.thr_text = tk.StringVar(value=DEFAULT_THRESHOLDS)
        self.r0_text = tk.StringVar(value="")        # vazio = usa o R0 de cada resumo
        self._build()

    def _build(self):
        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Button(bar, text="Abrir CSV(s)...", command=self._open).pack(side="left", padx=2)
        ttk.Button(bar, text="Adicionar sessão atual", command=self._add_current).pack(side="left", padx=2)
        ttk.Button(bar, text="Remover selecionada", command=self._remove).pack(side="left", padx=2)
        ttk.Button(bar, text="Limpar", command=self._clear).pack(side="left", padx=2)
        ttk.Button(bar, text="Exportar tabela CSV...", command=self._export_table).pack(side="right", padx=2)

        opt = ttk.Frame(self)
        opt.pack(fill="x", padx=8, pady=2)
        ttk.Label(opt, text="Curva:").pack(side="left")
        cb = ttk.Combobox(opt, textvariable=self.curve, values=CURVES, state="readonly", width=32)
        cb.pack(side="left", padx=4)
        cb.bind("<<ComboboxSelected>>", lambda e: self._plot())
        ttk.Label(opt, text="R0 (mΩ, vazio = do resumo):").pack(side="left", padx=(12, 2))
        e1 = ttk.Entry(opt, textvariable=self.r0_text, width=7)
        e1.pack(side="left")
        e1.bind("<Return>", lambda e: self._plot())
        ttk.Label(opt, text="Tensões da tabela (V):").pack(side="left", padx=(12, 2))
        e2 = ttk.Entry(opt, textvariable=self.thr_text, width=26)
        e2.pack(side="left")
        e2.bind("<Return>", lambda e: self._refresh())
        ttk.Button(opt, text="Aplicar", command=self._refresh).pack(side="left", padx=4)

        self.tree = ttk.Treeview(self, show="headings", height=5, selectmode="browse")
        self.tree.pack(fill="x", padx=8, pady=4)

        plot = ttk.Frame(self)
        plot.pack(fill="both", expand=True, padx=8, pady=4)
        self.fig = Figure(figsize=(9, 4.5), dpi=100, layout="constrained")
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot)
        NavigationToolbar2Tk(self.canvas, plot, pack_toolbar=True).pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        self._refresh()

    # ----------------------------------------------------------- carregar

    def _open(self):
        paths = filedialog.askopenfilenames(
            title="CSV(s) de descarga", filetypes=[("CSV", "*.csv"), ("Todos", "*.*")],
            initialdir=self.app.out_dir.get() if os.path.isdir(self.app.out_dir.get()) else None)
        for p in paths:
            try:
                self.runs.append(load_run(p))
            except (OSError, ValueError, KeyError) as exc:
                messagebox.showerror("Análise", f"Não foi possível ler {os.path.basename(p)}:\n{exc}")
        self._refresh()

    def _add_current(self):
        try:
            self.runs.append(run_from_rows(self.app.rows, self.app.summary, self.app.meta))
        except ValueError as exc:
            messagebox.showinfo("Análise", str(exc))
            return
        self._refresh()

    def _remove(self):
        sel = self.tree.selection()
        if sel:
            idx = self.tree.index(sel[0])
            if 0 <= idx < len(self.runs):
                del self.runs[idx]
                self._refresh()

    def _clear(self):
        self.runs = []
        self._refresh()

    # -------------------------------------------------------------- tabela

    def _thresholds(self):
        out = []
        for tok in self.thr_text.get().replace(";", ",").split(","):
            x = to_float(tok.strip().replace(",", "."))
            if not math.isnan(x) and x > 0:
                out.append(x)
        return sorted(set(out), reverse=True)

    def _table_rows(self):
        thr = self._thresholds()
        header = ["Corrida", "Duração", "Ah", "Ah corr.", "Wh", "V médio", "V mín", "R0 (mΩ)"] + \
                 [f"Ah até {t:g} V" for t in thr]
        rows = []
        for run in self.runs:
            m = compute_metrics(run, thr)
            s = run["summary"]
            qc = to_float(s.get("q_corr_Ah", "nan"))
            r0 = run_r0_mohm(run)
            row = [run["name"], self.app._fmt_time(m["dur"]), f"{m['q']:.4f}",
                   "-" if math.isnan(qc) else f"{qc:.4f}", f"{m['e']:.3f}", f"{m['vmean']:.3f}",
                   f"{m['vmin']:.3f}", f"{r0:.1f}" if r0 else "-"]
            row += ["-" if m["thr"][t] is None else f"{m['thr'][t][0]:.3f}" for t in thr]
            rows.append(row)
        return header, rows

    def _refresh(self):
        header, rows = self._table_rows()
        self.tree["columns"] = list(range(len(header)))
        for i, h in enumerate(header):
            self.tree.heading(i, text=h)
            self.tree.column(i, width=150 if i == 0 else 90, anchor="w" if i == 0 else "e", stretch=True)
        self.tree.delete(*self.tree.get_children())
        for r in rows:
            self.tree.insert("", "end", values=r)
        self._plot()

    def _export_table(self):
        header, rows = self._table_rows()
        if not rows:
            messagebox.showinfo("Análise", "Nenhuma corrida carregada.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")],
                                            initialfile="comparacao_descargas.csv")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(rows)
        except OSError as exc:
            messagebox.showerror("Análise", f"Falha ao gravar:\n{exc}")

    # -------------------------------------------------------------- grafico

    def _plot(self):
        ax = self.ax
        ax.clear()
        ax.grid(True, alpha=0.3)
        kind = self.curve.get()
        manual = to_float(self.r0_text.get().strip().replace(",", ".")) if self.r0_text.get().strip() else None
        for n, run in enumerate(self.runs):
            color = COLORS[n % len(COLORS)]
            if kind == CURVES[2]:
                ax.plot(run["t"], run["v"], color=color, lw=1.2, label=run["name"])
            elif kind == CURVES[3]:
                ax.plot(run["t"], run["i"], color=color, lw=1.2, label=run["name"])
            else:
                v = run["v"]
                label = run["name"]
                if kind == CURVES[1]:
                    r0 = manual if (manual is not None and not math.isnan(manual)) else run_r0_mohm(run)
                    v = v + run["i"] * r0 / 1000.0
                    label = f"{run['name']} (R0 {r0:.0f} mΩ)"
                ax.plot(run["q"], v, color=color, lw=1.2, label=label)
        if kind in (CURVES[2], CURVES[3]):
            style_time_axis(ax)
            ax.set_xlabel("Tempo desde o início da descarga")
            ax.set_ylabel("V bateria (V)" if kind == CURVES[2] else "Corrente (A)")
        else:
            ax.set_xlabel("Capacidade descarregada (Ah)")
            ax.set_ylabel("V bateria (V)" if kind == CURVES[0] else "V + I·R0 (V)")
        ax.set_title(kind)
        if self.runs:
            ax.legend(loc="best", fontsize=8)
        else:
            ax.text(0.5, 0.5, "Abra um ou mais CSVs de descarga", transform=ax.transAxes,
                    ha="center", va="center", color="#777")
        self.canvas.draw_idle()
