#!/usr/bin/env python3
"""
Teste de fumaça sem GUI: conecta, imprime o hello, varre os LEDs, lê os
sensores por alguns segundos e (opcional) dá um pulso curto nos motores.

    python examples/smoke_test.py --port COM5
    python examples/smoke_test.py --demo
    python examples/smoke_test.py --port COM5 --motors     # CUIDADO: aciona os motores
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from gcm_test.client import DEFAULT_BAUD, DEMO_PORT, GcmClient  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--motors", action="store_true", help="aciona cada motor a 15 %% por 1 s")
    args = ap.parse_args()
    port = DEMO_PORT if args.demo else args.port
    if not port:
        ap.error("informe --port ou --demo")

    with GcmClient() as gcm:
        gcm.connect(port, args.baud)
        hello = gcm.wait_hello()
        print("firmware:", hello["fw"], hello["ver"], "| PCA9554A ok:", bool(hello["exp"]), "| ToF online:", hello["tof"])

        for idx in (1, 2, 3):
            gcm.led(idx, True)
            time.sleep(0.4)
            gcm.led(idx, False)

        end = time.time() + args.seconds
        while time.time() < end:
            t = gcm.wait_tel()
            tof = ["--" if not s["on"] else f'{s["mm"]}mm(st{s["st"]})' for s in t["tof"]]
            print(f'bs={t["bs"]} start={t["start"]} tof={tof}')
            gcm.ping()
            time.sleep(0.25)

        if args.motors:
            input("Motores livres e fora do chão? ENTER para continuar (Ctrl+C aborta)...")
            gcm.cfg(max_duty=20)
            gcm.enable(True)
            for ch in (1, 2):
                for sign in (+1, -1):
                    gcm.motor(ch, 15 * sign)
                    t_end = time.time() + 1.0
                    while time.time() < t_end:
                        gcm.ping()          # mantém o watchdog alimentado
                        time.sleep(0.1)
                    gcm.motor(ch, 0)
                    time.sleep(0.3)
            gcm.stop()
        gcm.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
