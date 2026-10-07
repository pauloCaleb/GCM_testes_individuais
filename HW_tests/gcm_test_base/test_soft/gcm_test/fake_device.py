"""
Simulador do firmware da GCM (protocolo v1) com a mesma interface mínima de
uma porta serial (read/write/close/in_waiting).

Serve para desenvolver e testar a GUI e scripts sem hardware: use a porta
"DEMO". As mensagens, campos e mensagens de erro seguem o firmware real
(ver PROTOCOL.md); a dinâmica dos motores é simplificada.
"""

from __future__ import annotations

import json
import math
import random
import threading
import time


def _dumps(obj) -> bytes:
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode()


class FakeSerial:
    def __init__(self, boot_delay: float = 0.4, seed: int = 1) -> None:
        self.is_open = True
        self._rng = random.Random(seed)
        self._t0 = time.monotonic()
        self._boot_delay = boot_delay
        self._cond = threading.Condition()
        self._tx = bytearray()
        self._rx = bytearray()
        self._lock = threading.RLock()

        # estado simulado
        self.tel_hz = 20
        self.cap = 30.0
        self.wd_ms = 500
        self.slew = 300.0
        self.en = 0
        self.led = [0, 1, 0]
        self.tgt = [0.0, 0.0]
        self.cur = [0.0, 0.0]
        self.failsafe = 0
        self.bs = [1, 1, 1, 1]
        self.start = 1
        self.tof_mm = [600.0, 900.0, 1500.0]
        self._tof_age = [0, 0, 0]
        self._last_cmd = time.monotonic()
        self._hello_sent = False

        self._thread = threading.Thread(target=self._run, name="fake-gcm", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # interface "serial"
    # ------------------------------------------------------------------
    @property
    def in_waiting(self) -> int:
        with self._cond:
            return len(self._tx)

    def read(self, n: int = 1) -> bytes:
        with self._cond:
            if not self._tx:
                self._cond.wait(0.05)
            out = bytes(self._tx[:n])
            del self._tx[:n]
            return out

    def write(self, data: bytes) -> int:
        with self._lock:
            self._rx.extend(data)
            while b"\n" in self._rx:
                i = self._rx.index(b"\n")
                line = bytes(self._rx[:i]).decode("utf-8", errors="replace").strip()
                del self._rx[: i + 1]
                if line.startswith("{"):
                    self._handle(line)
        return len(data)

    def close(self) -> None:
        self.is_open = False

    # ------------------------------------------------------------------
    def _out(self, obj) -> None:
        with self._cond:
            self._tx.extend(_dumps(obj))
            self._cond.notify_all()

    def _ms(self) -> int:
        return int((time.monotonic() - self._t0) * 1000)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _hello(self, seq=None):
        m = {"t": "hello", "fw": "gcm_test_base(sim)", "ver": "1.0.0", "proto": 1, "exp": 1,
             "tof": [1, 1, 1], "cap": self.cap, "wd_ms": self.wd_ms, "slew": self.slew,
             "tel_hz": self.tel_hz}
        if seq is not None:
            m["seq"] = seq
        self._out(m)

    def _err(self, cmd, msg, seq):
        safe = "".join(c if (c.islower() or c.isdigit() or c == "_") else "?" for c in (cmd or ""))[:19]
        m = {"t": "err", "cmd": safe, "msg": msg}
        if seq is not None:
            m["seq"] = seq
        self._out(m)

    def _ack(self, cmd, seq, **fields):
        m = {"t": "ack", "cmd": cmd, **fields}
        if seq is not None:
            m["seq"] = seq
        self._out(m)

    def _stop(self):
        self.cur = [0.0, 0.0]
        self.tgt = [0.0, 0.0]
        self.en = 0

    # ------------------------------------------------------------------
    def _handle(self, line: str) -> None:
        try:
            o = json.loads(line)
            if not isinstance(o, dict) or any(isinstance(v, (dict, list)) for v in o.values()):
                raise ValueError
        except ValueError:
            self._err("", "json_invalido", None)
            return
        seq = o.get("seq") if isinstance(o.get("seq"), (int, float)) and o.get("seq") >= 0 else None
        seq = int(seq) if seq is not None else None
        cmd = o.get("cmd")
        if not isinstance(cmd, str):
            self._err("", "sem_cmd", seq)
            return

        def num(key):
            v = o.get(key)
            return v if isinstance(v, (int, float)) and not isinstance(v, str) else None

        self._last_cmd = time.monotonic()

        if cmd == "ping":
            m = {"t": "pong", "ms": self._ms()}
            if seq is not None:
                m["seq"] = seq
            self._out(m)
        elif cmd == "hello":
            self._hello(seq)
        elif cmd == "led":
            on = num("on")
            if on is None:
                return self._err("led", "falta_on", seq)
            on = 1 if on else 0
            if o.get("idx") == "all":
                self.led = [on, on, on]
                return self._ack("led", seq, idx="all", on=on)
            idx = num("idx")
            if idx is None or idx < 1 or idx > 3:
                return self._err("led", "idx_invalido", seq)
            self.led[int(idx) - 1] = on
            self._ack("led", seq, idx=int(idx), on=on)
        elif cmd == "motor":
            ch, duty = num("ch"), num("duty")
            if ch not in (1, 2):
                return self._err("motor", "ch_invalido", seq)
            if duty is None or not (-100 <= duty <= 100):
                return self._err("motor", "duty_invalido", seq)
            v = self._clamp(float(duty), -self.cap, self.cap)
            self.tgt[int(ch) - 1] = v
            self._ack("motor", seq, ch=int(ch), duty=round(v, 1))
        elif cmd == "en":
            on = num("on")
            if on is None:
                return self._err("en", "falta_on", seq)
            if on and not self.en:
                self.tgt = [0.0, 0.0]
                self.failsafe = 0
            self.en = 1 if on else 0
            if not self.en:
                self.cur = [0.0, 0.0]
                self.tgt = [0.0, 0.0]
            self._ack("en", seq, on=self.en)
        elif cmd == "stop":
            self._stop()
            self._ack("stop", seq)
        elif cmd == "cfg":
            hz, cap, wd, slew = num("tel_hz"), num("max_duty"), num("wd_ms"), num("slew")
            if hz is not None and not 1 <= hz <= 50:
                return self._err("cfg", "tel_hz_invalido", seq)
            if cap is not None and not 0 <= cap <= 100:
                return self._err("cfg", "max_duty_invalido", seq)
            if wd is not None and not (wd == 0 or 50 <= wd <= 10000):
                return self._err("cfg", "wd_ms_invalido", seq)
            if slew is not None and not 0 <= slew <= 2000:
                return self._err("cfg", "slew_invalido", seq)
            if hz is not None:
                self.tel_hz = int(hz)
            if cap is not None:
                self.cap = float(cap)
            if wd is not None:
                self.wd_ms = int(wd)
            if slew is not None:
                self.slew = float(slew)
            self._ack("cfg", seq, tel_hz=self.tel_hz, max_duty=self.cap, wd_ms=self.wd_ms, slew=self.slew)
        else:
            self._err(cmd, "cmd_desconhecido", seq)

    # ------------------------------------------------------------------
    def _tel(self) -> dict:
        tof = []
        for i in range(3):
            mm = int(self.tof_mm[i])
            tof.append({"on": 1, "mm": mm, "st": 0 if 40 < mm < 3900 else 4,
                        "age": self._tof_age[i], "err": 0})
        return {
            "t": "tel", "ms": self._ms(), "bs": list(self.bs), "start": self.start, "tof": tof,
            "led": list(self.led), "en": self.en,
            "m": [round(v, 1) for v in self.cur], "mt": [round(v, 1) for v in self.tgt],
            "cap": self.cap, "fs": self.failsafe,
        }

    def _step(self, dt: float) -> None:
        # motores: rampa passando por zero ao inverter
        step_max = self.slew * dt if self.slew > 0 else 1000.0
        for i in range(2):
            tgt = self._clamp(self.tgt[i], -self.cap, self.cap) if self.en else 0.0
            cur = self.cur[i]
            eff = 0.0 if (cur != 0 and (tgt > 0) != (cur > 0)) or tgt == 0 else tgt
            diff = eff - cur
            self.cur[i] = eff if abs(diff) <= step_max else cur + math.copysign(step_max, diff)

        # watchdog
        active = self.en or any(self.cur) or any(self.tgt)
        if self.wd_ms > 0 and active and not self.failsafe and \
                (time.monotonic() - self._last_cmd) * 1000 > self.wd_ms:
            self._stop()
            self.failsafe = 1
            self._out({"t": "evt", "name": "watchdog", "idx": 0, "v": 1, "ms": self._ms()})

        # sensores de distância: oscilação lenta em torno de uma base + ruído
        base = (600.0, 1000.0, 1600.0)
        for i in range(3):
            self.tof_mm[i] = self._clamp(
                base[i] + 350 * math.sin(time.monotonic() * (0.5 + 0.17 * i)) + self._rng.uniform(-8, 8),
                30, 3900)
            self._tof_age[i] = self._rng.randint(0, 40)

        # entradas: alterna um sensor de borda de vez em quando, e o botão
        if self._rng.random() < 0.004:
            k = self._rng.randrange(4)
            self.bs[k] ^= 1
            self._out({"t": "evt", "name": "bs", "idx": k + 1, "v": self.bs[k], "ms": self._ms()})
        if self._rng.random() < 0.0015:
            self.start ^= 1
            self._out({"t": "evt", "name": "start", "idx": 0, "v": self.start, "ms": self._ms()})

    def _run(self) -> None:
        tick = 0.01
        next_tel = time.monotonic()
        while self.is_open:
            time.sleep(tick)
            now = time.monotonic()
            with self._lock:
                if not self._hello_sent and now - self._t0 >= self._boot_delay:
                    self._hello_sent = True
                    self._hello()
                if not self._hello_sent:
                    continue
                self._step(tick)
                if now >= next_tel:
                    self._out(self._tel())
                    next_tel = now + 1.0 / max(1, self.tel_hz)
