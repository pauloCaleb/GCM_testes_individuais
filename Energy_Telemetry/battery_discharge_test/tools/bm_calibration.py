"""
bm_calibration.py — aba "Calibração" do battery_monitor.

Os coeficientes ficam no FIRMWARE (NVS) e valem para qualquer computador que se conecte.
Esta aba só envia os comandos CAL e mostra o resultado; o procedimento passo a passo está
em MANUAL_USO_E_CALIBRACAO.md.

  V_calibrada = (AIN0 x 5,7) x ganho_V + offset_V
  I_calibrada = ((AIN1 - zero) / 40 mV/A) x ganho_I
"""

import math
import tkinter as tk
from tkinter import messagebox, ttk

from bm_core import parse_kv, to_float

GAIN_MIN, GAIN_MAX = 0.8, 1.2
OFF_MAX_MV = 500.0


class CalibrationTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.cur = {"v_gain_ppm": 1000000, "v_off_uV": 0, "i_gain_ppm": 1000000}
        self.v_ratio = 5.7
        self._prefilled = False
        self.live = {k: tk.StringVar(value="-") for k in
                     ("state", "v_cal", "v_raw", "i_cal", "ain1", "zero")}
        self.coef_txt = {k: tk.StringVar(value="-") for k in ("vg", "vo", "ig")}
        self.ent = {k: tk.StringVar() for k in ("vg", "vo", "ig", "ref1", "ref1b", "ref2", "refi")}
        self.lbl_p1 = tk.StringVar(value="Ponto 1 ainda não capturado.")
        self._btns = {}
        self._build()

    # ------------------------------------------------------------------ UI

    def _build(self):
        left = ttk.Frame(self)
        left.pack(side="left", fill="both", expand=True, padx=(8, 4), pady=8)
        right = ttk.Frame(self)
        right.pack(side="left", fill="both", expand=True, padx=(4, 8), pady=8)

        # --- leituras ao vivo
        live = ttk.LabelFrame(left, text="Leituras ao vivo")
        live.pack(fill="x", pady=(0, 6))
        rows = [("Estado do firmware", "state"), ("Tensão calibrada (V)", "v_cal"),
                ("Tensão bruta do divisor (V)", "v_raw"), ("Corrente calibrada (A)", "i_cal"),
                ("AIN1 do ACS758 (V)", "ain1"), ("Zero em uso (V)", "zero")]
        for r, (label, key) in enumerate(rows):
            ttk.Label(live, text=label, foreground="#555").grid(row=r, column=0, sticky="w", padx=8, pady=1)
            ttk.Label(live, textvariable=self.live[key], font=("TkDefaultFont", 11, "bold")).grid(
                row=r, column=1, sticky="w", padx=8)

        # --- coeficientes
        coef = ttk.LabelFrame(left, text="Coeficientes gravados no firmware")
        coef.pack(fill="x", pady=6)
        hdr = ["", "Atual", "Novo valor"]
        for c, h in enumerate(hdr):
            ttk.Label(coef, text=h, foreground="#555").grid(row=0, column=c, padx=8, sticky="w")
        items = [("Ganho da tensão (fator)", "vg"), ("Offset da tensão (mV)", "vo"),
                 ("Ganho da corrente (fator)", "ig")]
        for r, (label, key) in enumerate(items, start=1):
            ttk.Label(coef, text=label).grid(row=r, column=0, sticky="w", padx=8, pady=2)
            ttk.Label(coef, textvariable=self.coef_txt[key]).grid(row=r, column=1, sticky="w", padx=8)
            ttk.Entry(coef, textvariable=self.ent[key], width=14).grid(row=r, column=2, padx=8)
        bar = ttk.Frame(coef)
        bar.grid(row=4, column=0, columnspan=3, sticky="w", padx=8, pady=6)
        self._btns["refresh"] = ttk.Button(bar, text="Atualizar", command=lambda: self.app._send("CAL SHOW"))
        self._btns["refresh"].pack(side="left", padx=2)
        ttk.Button(bar, text="Copiar atuais", command=self._copy_current).pack(side="left", padx=2)
        self._btns["apply"] = ttk.Button(bar, text="Aplicar valores digitados", command=self._apply_manual)
        self._btns["apply"].pack(side="left", padx=2)
        self._btns["reset"] = ttk.Button(bar, text="Restaurar padrão", command=self._reset)
        self._btns["reset"].pack(side="left", padx=2)

        # --- registro
        logf = ttk.LabelFrame(left, text="Registro da calibração")
        logf.pack(fill="both", expand=True, pady=(6, 0))
        self.log = tk.Text(logf, height=8, state="disabled", wrap="word", font=("TkFixedFont", 9))
        self.log.pack(fill="both", expand=True, padx=4, pady=4)
        self.log.tag_configure("ok", foreground="#064")
        self.log.tag_configure("err", foreground="#b00")

        # --- tensao 1 ponto
        v1 = ttk.LabelFrame(right, text="Tensão — 1 ponto (ajusta o ganho)")
        v1.pack(fill="x", pady=(0, 6))
        ttk.Label(v1, wraplength=420, justify="left", foreground="#555", text=(
            "Bateria ou fonte ligada ao pad, carga desligada (estado IDLE, READY, DONE ou FAULT). "
            "Meça a tensão no pad com um multímetro e digite o valor.")).pack(anchor="w", padx=8, pady=(4, 2))
        r1 = ttk.Frame(v1)
        r1.pack(anchor="w", padx=8, pady=4)
        ttk.Label(r1, text="Referência (V):").pack(side="left")
        ttk.Entry(r1, textvariable=self.ent["ref1"], width=9).pack(side="left", padx=4)
        self._btns["v1"] = ttk.Button(r1, text="Calibrar ganho", command=self._cal_v1)
        self._btns["v1"].pack(side="left", padx=4)

        # --- tensao 2 pontos
        v2 = ttk.LabelFrame(right, text="Tensão — 2 pontos (ganho e offset)")
        v2.pack(fill="x", pady=6)
        ttk.Label(v2, wraplength=420, justify="left", foreground="#555", text=(
            "Use a fonte de bancada no pad, em duas tensões afastadas (ex.: 12 V e 16 V; mínimo "
            "1,5 V de diferença). Em cada uma, digite a leitura do multímetro e capture.")).pack(
            anchor="w", padx=8, pady=(4, 2))
        p1 = ttk.Frame(v2)
        p1.pack(anchor="w", padx=8, pady=2)
        ttk.Label(p1, text="Ponto 1 — referência (V):").pack(side="left")
        ttk.Entry(p1, textvariable=self.ent["ref1b"], width=9).pack(side="left", padx=4)
        self._btns["vp1"] = ttk.Button(p1, text="Capturar ponto 1", command=self._cal_vp1)
        self._btns["vp1"].pack(side="left", padx=4)
        ttk.Label(v2, textvariable=self.lbl_p1, foreground="#a60").pack(anchor="w", padx=8)
        p2 = ttk.Frame(v2)
        p2.pack(anchor="w", padx=8, pady=(2, 6))
        ttk.Label(p2, text="Ponto 2 — referência (V):").pack(side="left")
        ttk.Entry(p2, textvariable=self.ent["ref2"], width=9).pack(side="left", padx=4)
        self._btns["vp2"] = ttk.Button(p2, text="Capturar ponto 2 e calcular", command=self._cal_vp2)
        self._btns["vp2"].pack(side="left", padx=4)

        # --- corrente
        ci = ttk.LabelFrame(right, text="Corrente (ajusta o ganho)")
        ci.pack(fill="x", pady=6)
        ttk.Label(ci, wraplength=420, justify="left", foreground="#555", text=(
            "A corrente só existe com a carga ligada, então esta calibração é feita DURANTE a "
            "descarga (estado DISCHARGE, ≥ 0,3 A): inicie um teste, espere ~1 min, meça a corrente "
            "com um alicate amperímetro DC ou multímetro em série e digite. O firmware faz a média "
            "de 3 s e ajusta o ganho. Depois interrompa o teste (STOP): ele não serve como medida "
            "de capacidade.")).pack(anchor="w", padx=8, pady=(4, 2))
        rc = ttk.Frame(ci)
        rc.pack(anchor="w", padx=8, pady=6)
        ttk.Label(rc, text="Corrente de referência (A):").pack(side="left")
        ttk.Entry(rc, textvariable=self.ent["refi"], width=9).pack(side="left", padx=4)
        self._btns["ci"] = ttk.Button(rc, text="Calibrar corrente", command=self._cal_i)
        self._btns["ci"].pack(side="left", padx=4)

        ttk.Label(right, foreground="#555", wraplength=440, justify="left", text=(
            "Os coeficientes ficam salvos no ESP32 (NVS), alteram o cutoff e valem nos testes seguintes. "
            "Procedimento detalhado em tools/MANUAL_USO_E_CALIBRACAO.md.")).pack(anchor="w", pady=(6, 0))

    # ---------------------------------------------------------- dados do app

    def refresh_state(self, connected, st):
        idle_like = st in ("IDLE", "READY", "DONE", "FAULT")

        def en(key, cond):
            self._btns[key].configure(state="normal" if (connected and cond) else "disabled")
        en("refresh", True)
        en("apply", idle_like)
        en("reset", idle_like)
        en("v1", idle_like)
        en("vp1", idle_like)
        en("vp2", idle_like)
        en("ci", st == "DISCHARGE")
        self.live["state"].set(st)

    def update_live(self, row):
        self.live["v_cal"].set(f"{row['v_batt_V']:.4f}")
        self.live["v_raw"].set(f"{row['ain0_V'] * self.v_ratio:.4f}")
        self.live["i_cal"].set(f"{row['i_A']:.4f}")
        self.live["ain1"].set(f"{row['ain1_V']:.5f}")

    def on_status(self, kv):
        try:
            self.cur["v_gain_ppm"] = int(kv["v_gain_ppm"])
            self.cur["v_off_uV"] = int(kv["v_off_uV"])
            self.cur["i_gain_ppm"] = int(kv["i_gain_ppm"])
        except (KeyError, ValueError):
            return
        if "v_ratio" in kv:
            r = to_float(kv["v_ratio"])
            if not math.isnan(r) and r > 0:
                self.v_ratio = r
        if "zero_V" in kv:
            self.live["zero"].set(kv["zero_V"])
        self.coef_txt["vg"].set(f"{self.cur['v_gain_ppm'] / 1e6:.6f}  ({self.cur['v_gain_ppm']} ppm)")
        self.coef_txt["vo"].set(f"{self.cur['v_off_uV'] / 1000.0:.3f} mV")
        self.coef_txt["ig"].set(f"{self.cur['i_gain_ppm'] / 1e6:.6f}  ({self.cur['i_gain_ppm']} ppm)")
        if not self._prefilled:
            self._copy_current()
            self._prefilled = True

    def on_event(self, name, details):
        if name == "CAL":
            kind, _, rest = details.partition(";")
            self._log(f"{kind}: {rest}", "ok")
            if kind == "VP1":
                kv = parse_kv(rest)
                self.lbl_p1.set(f"Ponto 1 capturado: bruto {kv.get('raw_V', '?')} V, "
                                f"referência {kv.get('ref_V', '?')} V.")
            elif kind in ("VP2", "RESET", "SET"):
                self.lbl_p1.set("Ponto 1 ainda não capturado.")
        elif name == "ERR" and details.startswith("CAL"):
            self._log(details, "err")

    # ------------------------------------------------------------- acoes

    @staticmethod
    def _volts_to_mv(text):
        try:
            v = float(text.strip().replace(",", "."))
        except ValueError:
            return None
        if v <= 0:
            return None
        return int(round(v * 1000.0))

    def _copy_current(self):
        self.ent["vg"].set(f"{self.cur['v_gain_ppm'] / 1e6:.6f}")
        self.ent["vo"].set(f"{self.cur['v_off_uV'] / 1000.0:.3f}")
        self.ent["ig"].set(f"{self.cur['i_gain_ppm'] / 1e6:.6f}")

    def _apply_manual(self):
        try:
            vg = float(self.ent["vg"].get().replace(",", "."))
            vo = float(self.ent["vo"].get().replace(",", "."))
            ig = float(self.ent["ig"].get().replace(",", "."))
        except ValueError:
            messagebox.showerror("Calibração", "Valores inválidos.")
            return
        if not (GAIN_MIN <= vg <= GAIN_MAX and GAIN_MIN <= ig <= GAIN_MAX and abs(vo) <= OFF_MAX_MV):
            messagebox.showerror(
                "Calibração",
                f"Fora da faixa aceita: ganhos entre {GAIN_MIN} e {GAIN_MAX}; offset até ±{OFF_MAX_MV:.0f} mV.")
            return
        if messagebox.askokcancel("Calibração", "Gravar estes coeficientes no firmware?\n"
                                  "Eles alteram a tensão usada no cutoff."):
            self.app._send(f"CAL SET {int(round(vg * 1e6))} {int(round(vo * 1000))} {int(round(ig * 1e6))}")

    def _reset(self):
        if messagebox.askokcancel("Calibração", "Restaurar ganhos 1,000000 e offset 0 no firmware?"):
            self.app._send("CAL RESET")

    def _cal_v1(self):
        mv = self._volts_to_mv(self.ent["ref1"].get())
        if mv is None:
            messagebox.showerror("Calibração", "Digite a tensão de referência em volts (ex.: 16.43).")
            return
        self.app._send(f"CAL V {mv}")

    def _cal_vp1(self):
        mv = self._volts_to_mv(self.ent["ref1b"].get())
        if mv is None:
            messagebox.showerror("Calibração", "Digite a tensão de referência do ponto 1 em volts.")
            return
        self.app._send(f"CAL VP1 {mv}")

    def _cal_vp2(self):
        mv = self._volts_to_mv(self.ent["ref2"].get())
        if mv is None:
            messagebox.showerror("Calibração", "Digite a tensão de referência do ponto 2 em volts.")
            return
        self.app._send(f"CAL VP2 {mv}")

    def _cal_i(self):
        try:
            a = float(self.ent["refi"].get().strip().replace(",", "."))
        except ValueError:
            a = 0.0
        if a <= 0.3:
            messagebox.showerror("Calibração", "Digite a corrente de referência em ampères (> 0,3 A).")
            return
        self.app._send(f"CAL I {int(round(a * 1000))}")

    def _log(self, text, tag):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")
