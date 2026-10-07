"""
Cliente do protocolo serial JSON-lines da GCM (sem dependência de Qt).

Pode ser usado direto em scripts de teste automatizados::

    from gcm_test.client import GcmClient

    with GcmClient() as gcm:
        gcm.connect("/dev/ttyUSB0")            # ou "DEMO" para o simulador
        gcm.wait_hello()
        gcm.led(1, True)
        tel = gcm.wait_tel()
        print(tel["tof"][0]["mm"])

Cada mensagem recebida vira um dict com o campo "t" (hello, tel, evt, ack,
err, pong) mais dois tipos gerados aqui: {"t": "log", "line": ...} para linhas
que não são JSON (logs do ESP-IDF) e {"t": "disconnect", "error": ...}.
"""

from __future__ import annotations

import collections
import json
import threading
import time
from typing import Any, Callable, Deque, Dict, List, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

DEFAULT_BAUD = 921600
DEMO_PORT = "DEMO"

Message = Dict[str, Any]


class GcmError(Exception):
    pass


def available_ports() -> List[str]:
    """Portas seriais do sistema (mais a porta simulada 'DEMO')."""
    ports: List[str] = []
    if list_ports is not None:
        ports = [p.device for p in sorted(list_ports.comports())]
    return ports + [DEMO_PORT]


class GcmClient:
    def __init__(self) -> None:
        self._ser = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._wlock = threading.Lock()
        self._cond = threading.Condition()
        self._queue: Deque[Message] = collections.deque(maxlen=20000)
        self._seq = 0
        self.connected = False
        self.port: Optional[str] = None
        self.last_tel: Optional[Message] = None
        self.last_hello: Optional[Message] = None
        self.rx_bytes = 0
        self.rx_lines = 0
        self.rx_json = 0

    # ------------------------------------------------------------------
    # conexão
    # ------------------------------------------------------------------
    def __enter__(self) -> "GcmClient":
        return self

    def __exit__(self, *exc) -> None:
        self.disconnect()

    def connect(self, port: str, baud: int = DEFAULT_BAUD) -> None:
        if self.connected:
            self.disconnect()

        if port == DEMO_PORT:
            from .fake_device import FakeSerial
            self._ser = FakeSerial()
        else:
            if serial is None:
                raise GcmError("pyserial não instalado (pip install pyserial)")
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = baud
            ser.timeout = 0.05
            ser.write_timeout = 0.5
            # DTR/RTS em nível inativo: o circuito de auto-download do CH340C
            # liga DTR/RTS ao EN/GPIO0 do ESP32, então abrir a porta pode
            # reiniciar a placa em alguns sistemas (a GUI espera o "hello").
            ser.dtr = False
            ser.rts = False
            try:
                ser.open()
            except (serial.SerialException, OSError) as e:
                raise GcmError(f"não foi possível abrir {port}: {e}") from e
            self._ser = ser

        self.port = port
        self._stop.clear()
        self.last_tel = None
        self.last_hello = None
        self.rx_bytes = self.rx_lines = self.rx_json = 0
        self._queue.clear()
        self.connected = True
        self._thread = threading.Thread(target=self._reader, name="gcm-reader", daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._stop.set()
        self.connected = False
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    # ------------------------------------------------------------------
    # recepção
    # ------------------------------------------------------------------
    def _push(self, msg: Message) -> None:
        with self._cond:
            self._queue.append(msg)
            self._cond.notify_all()

    def _handle_line(self, raw: bytes) -> None:
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return
        self.rx_lines += 1
        if line.startswith("{"):
            try:
                msg = json.loads(line)
                if isinstance(msg, dict) and "t" in msg:
                    msg["_rx"] = time.time()
                    self.rx_json += 1
                    if msg["t"] == "tel":
                        self.last_tel = msg
                    elif msg["t"] == "hello":
                        self.last_hello = msg
                    self._push(msg)
                    return
            except json.JSONDecodeError:
                pass
        self._push({"t": "log", "line": line, "_rx": time.time()})

    def _reader(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                n = max(1, getattr(self._ser, "in_waiting", 0))
                chunk = self._ser.read(n)
            except Exception as e:  # porta removida, erro de I/O...
                if not self._stop.is_set():
                    self.connected = False
                    self._push({"t": "disconnect", "error": str(e), "_rx": time.time()})
                return
            if not chunk:
                continue
            self.rx_bytes += len(chunk)
            buf.extend(chunk)
            while True:
                i = buf.find(b"\n")
                if i < 0:
                    break
                line = bytes(buf[:i])
                del buf[: i + 1]
                self._handle_line(line)
            if len(buf) > 65536:   # lixo sem quebra de linha
                buf.clear()

    def drain(self) -> List[Message]:
        """Retorna (e remove) todas as mensagens pendentes."""
        with self._cond:
            out = list(self._queue)
            self._queue.clear()
        return out

    def wait_message(self, pred: Callable[[Message], bool], timeout: float = 2.0) -> Message:
        """Espera (e consome) a primeira mensagem que satisfaça pred. Para scripts."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                for m in list(self._queue):
                    if pred(m):
                        self._queue.remove(m)
                        return m
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError("tempo esgotado esperando mensagem")
                self._cond.wait(left)

    def wait_hello(self, timeout: float = 20.0, retry_s: float = 0.5) -> Message:
        """Pede 'hello' repetidamente até o firmware responder (ele só responde
        depois do boot, que leva alguns segundos por causa do autoteste dos LEDs)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.send("hello")
            try:
                return self.wait_message(lambda m: m.get("t") == "hello",
                                         timeout=min(retry_s, max(0.01, deadline - time.monotonic())))
            except TimeoutError:
                continue
        raise TimeoutError("firmware não respondeu ao hello")

    def wait_tel(self, timeout: float = 2.0) -> Message:
        return self.wait_message(lambda m: m.get("t") == "tel", timeout)

    # ------------------------------------------------------------------
    # envio
    # ------------------------------------------------------------------
    def send(self, cmd: str, **fields: Any) -> int:
        """Envia um comando; retorna o 'seq' usado (ecoado no ack/err)."""
        if not self.connected or self._ser is None:
            raise GcmError("não conectado")
        self._seq += 1
        msg = {"cmd": cmd, **fields, "seq": self._seq}
        data = (json.dumps(msg, separators=(",", ":")) + "\n").encode()
        try:
            with self._wlock:
                self._ser.write(data)
        except Exception as e:
            self.connected = False
            self._push({"t": "disconnect", "error": str(e), "_rx": time.time()})
            raise GcmError(str(e)) from e
        return self._seq

    def ping(self) -> int:
        return self.send("ping")

    def hello(self) -> int:
        return self.send("hello")

    def led(self, idx, on: bool) -> int:
        """idx = 1..3 ou 'all'."""
        return self.send("led", idx=idx, on=1 if on else 0)

    def motor(self, ch: int, duty: float) -> int:
        """ch = 1..2; duty em % com sinal (-100..100), limitado pelo firmware a max_duty."""
        return self.send("motor", ch=ch, duty=round(float(duty), 1))

    def enable(self, on: bool) -> int:
        return self.send("en", on=1 if on else 0)

    def stop(self) -> int:
        return self.send("stop")

    def cfg(self, **fields: Any) -> int:
        """tel_hz, max_duty, wd_ms, slew."""
        return self.send("cfg", **fields)
