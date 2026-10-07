"""Testes do cliente + simulador (e compatibilidade com o formato real do firmware)."""

import json
import os
import time

import pytest

from gcm_test.client import GcmClient

DATA = os.path.join(os.path.dirname(__file__), "data", "firmware_tel_samples.txt")


@pytest.fixture()
def gcm():
    c = GcmClient()
    c.connect("DEMO")
    c.wait_hello(5)
    yield c
    c.disconnect()


def ack_for(c, seq, timeout=1.0):
    return c.wait_message(lambda m: m.get("seq") == seq and m["t"] in ("ack", "err", "pong"), timeout)


def test_hello_fields(gcm):
    h = gcm.last_hello
    for k in ("fw", "ver", "proto", "exp", "tof", "cap", "wd_ms", "slew", "tel_hz"):
        assert k in h
    assert h["proto"] == 1 and len(h["tof"]) == 3


def test_telemetry_stream(gcm):
    t1 = gcm.wait_tel()
    t2 = gcm.wait_tel()
    assert t2["ms"] >= t1["ms"]
    assert len(t1["bs"]) == 4 and len(t1["tof"]) == 3 and len(t1["led"]) == 3
    assert len(t1["m"]) == 2 and len(t1["mt"]) == 2


def test_led_commands(gcm):
    assert ack_for(gcm, gcm.led(3, True))["t"] == "ack"
    gcm.drain()
    assert gcm.wait_tel()["led"][2] == 1
    assert ack_for(gcm, gcm.led("all", False))["idx"] == "all"
    gcm.drain()
    assert gcm.wait_tel()["led"] == [0, 0, 0]


def test_motor_cap_and_enable(gcm):
    assert ack_for(gcm, gcm.motor(1, 90))["duty"] == 30.0      # limitado pelo cap de 30 %
    assert ack_for(gcm, gcm.cfg(max_duty=50))["max_duty"] == 50.0
    assert ack_for(gcm, gcm.motor(1, -90))["duty"] == -50.0
    # habilitar zera os alvos
    ack_for(gcm, gcm.enable(True))
    gcm.drain()
    t = gcm.wait_tel()
    assert t["en"] == 1 and t["mt"] == [0.0, 0.0]
    # com EN, o duty aplicado segue o alvo
    gcm.motor(2, 20)
    for _ in range(12):                 # mantém o watchdog alimentado enquanto a rampa acontece
        gcm.ping()
        time.sleep(0.05)
    gcm.drain()
    assert gcm.wait_tel()["m"][1] == 20.0


def test_stop(gcm):
    gcm.enable(True)
    gcm.motor(1, 20)
    time.sleep(0.2)
    assert ack_for(gcm, gcm.stop())["t"] == "ack"
    gcm.drain()
    t = gcm.wait_tel()
    assert t["en"] == 0 and t["m"] == [0.0, 0.0] and t["mt"] == [0.0, 0.0]


@pytest.mark.parametrize("cmd,fields,msg", [
    ("motor", {"ch": 3, "duty": 10}, "ch_invalido"),
    ("motor", {"ch": 1, "duty": 500}, "duty_invalido"),
    ("motor", {"ch": 1}, "duty_invalido"),
    ("led", {"idx": 9, "on": 1}, "idx_invalido"),
    ("led", {"idx": 1}, "falta_on"),
    ("en", {}, "falta_on"),
    ("cfg", {"tel_hz": 0}, "tel_hz_invalido"),
    ("cfg", {"max_duty": 101}, "max_duty_invalido"),
    ("cfg", {"wd_ms": 10}, "wd_ms_invalido"),
    ("cfg", {"slew": -1}, "slew_invalido"),
    ("nada", {}, "cmd_desconhecido"),
])
def test_validation_errors(gcm, cmd, fields, msg):
    r = ack_for(gcm, gcm.send(cmd, **fields))
    assert r["t"] == "err" and r["msg"] == msg


def test_invalid_json_line(gcm):
    gcm._ser.write(b'{"cmd": oops}\n')
    m = gcm.wait_message(lambda m: m["t"] == "err", 1.0)
    assert m["msg"] == "json_invalido"
    gcm._ser.write(b'{"foo":1}\n')
    assert gcm.wait_message(lambda m: m["t"] == "err", 1.0)["msg"] == "sem_cmd"


def test_non_json_lines_become_log(gcm):
    gcm._handle_line(b"I (123) gcm_main: boot ok")
    m = gcm.wait_message(lambda m: m["t"] == "log", 1.0)
    assert "boot ok" in m["line"]


def test_watchdog_stops_motors(gcm):
    assert ack_for(gcm, gcm.cfg(wd_ms=100))["wd_ms"] == 100
    gcm.enable(True)
    gcm.motor(1, 15)
    ev = gcm.wait_message(lambda m: m["t"] == "evt" and m.get("name") == "watchdog", 2.0)
    assert ev["v"] == 1
    gcm.drain()
    t = gcm.wait_tel()
    assert t["en"] == 0 and t["m"] == [0.0, 0.0] and t["fs"] == 1


def test_watchdog_fed_by_ping(gcm):
    gcm.cfg(wd_ms=200)
    gcm.enable(True)
    gcm.motor(1, 15)
    for _ in range(12):
        gcm.ping()
        time.sleep(0.05)
    gcm.drain()
    assert gcm.wait_tel()["fs"] == 0


def test_firmware_sample_lines_parse_and_match_simulator(gcm):
    """As linhas geradas pelo C real (host_tests/test_host.c) têm as mesmas chaves do simulador."""
    sim = gcm.wait_tel()
    with open(DATA, encoding="utf-8") as f:
        lines = [json.loads(l) for l in f if l.strip()]
    assert len(lines) >= 2
    for fw in lines:
        assert fw["t"] == "tel"
        assert set(fw) == {k for k in sim if not k.startswith("_")}
        on_sensors = [s for s in fw["tof"] if s["on"]]
        assert on_sensors
        for s in on_sensors:
            assert set(s) == set(sim["tof"][0])
        assert fw["tof"][1] == {"on": 0} or "mm" in fw["tof"][1]


def test_rx_counters(gcm):
    gcm.wait_tel()
    assert gcm.rx_bytes > 0 and gcm.rx_json > 0 and gcm.rx_lines >= gcm.rx_json
