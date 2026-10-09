# test_soft/ - GUI e ferramentas de teste da GCM

GUI em PySide6 + pyqtgraph que conversa com o firmware de `firm/` (protocolo em `../PROTOCOL.md`).

## Instalação

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Uso

```bash
python run_gui.py                   # escolha a porta e clique em Conectar
python run_gui.py --port COM5       # conecta ao abrir (Windows)
python run_gui.py --port /dev/ttyUSB0 --baud 921600
python run_gui.py --demo            # simulador do firmware, sem hardware
```

A GUI mostra:

- **Entradas**: BS_1..BS_4 e START_BOT (nível cru; marque "ativo em nível baixo" para inverter a cor).
- **ToF**: distância e status dos 3 VL53L1X, com gráfico dos últimos 30 s.
- **LEDs**: um botão por LED da fita e "todos".
- **Motores**: EN_ALL, um slider por motor (faixa = limite de duty), "Zero" e "Inverter sentido" (inverte o sinal só na GUI).
- **Configuração**: limite de duty, rampa, watchdog e taxa de telemetria (botão Aplicar).
- **Log**: eventos, erros, logs do ESP-IDF e gravação da telemetria em CSV.

Segurança: o botão **PARAR MOTORES** (atalhos **Esc** e **Espaço**) está sempre visível; os painéis ficam desabilitados até o firmware responder o `hello`; a GUI manda um `ping` a cada 150 ms (heartbeat do watchdog de 500 ms do firmware) e envia `stop` ao desconectar/fechar. Se a GUI travar ou o cabo sair, o firmware para os motores sozinho.

Ao abrir a porta, alguns sistemas reiniciam o ESP32 (o DTR/RTS do CH340C está ligado ao EN/GPIO0); a GUI já abre com DTR/RTS inativos e, de qualquer forma, espera o boot (alguns segundos, por causa do autoteste dos LEDs) reenviando `hello`.

## Usando em scripts de teste

`gcm_test/client.py` não depende de Qt:

```python
from gcm_test.client import GcmClient

with GcmClient() as gcm:
    gcm.connect("COM5")             # "DEMO" usa o simulador
    gcm.wait_hello()
    gcm.led("all", True)
    print(gcm.wait_tel()["tof"])
    gcm.enable(True); gcm.motor(1, 15)
    ...                              # mande gcm.ping() a cada ~150 ms durante o movimento
    gcm.stop()
```

Veja `examples/smoke_test.py` (`--demo` para testar sem placa; `--motors` aciona cada motor a 15 % por 1 s).

## Testes

```bash
python -m pytest -q
```

24 testes: protocolo (cliente + simulador), compatibilidade com as linhas geradas pelo C real (`tests/data/firmware_tel_samples.txt`, gerado por `firm/host_tests`) e fumaça da GUI em modo offscreen.

## Desempenho

A recepção (`_drain`, 25 ms) só registra cada quadro de telemetria (histórico do gráfico e CSV); os widgets são atualizados a 10 Hz com o **último** quadro e o gráfico a 5 Hz, só com a janela visível de 30 s. Assim o custo da interface não depende da taxa de telemetria nem cresce com o tempo, e linhas de log em excesso são descartadas com um aviso. A barra de status mostra `telemetria: N quadros/s | GUI: M atualizações/s, atraso máx. X ms`: se a GUI voltar a ficar lenta, esse atraso é o número a observar (e o uso de CPU no Gerenciador de Tarefas).

## Problemas conhecidos

- **A GUI conecta mas não identifica o firmware** ("dados ilegíveis" / sem `hello`): o baud da GUI não bate com o do firmware. O padrão é 921600; se o firmware foi compilado com o `sdkconfig` antigo, o console fica em 115200 (veja `firm/README.md`). Escolha o mesmo baud nos dois lados; a 115200 funciona, mas a telemetria de 20 Hz ocupa mais da metade do enlace (reduza a taxa com o campo "Telemetria").
- **`ImportError: DLL load failed while importing QtCore` no Windows** com o PyQt6 também instalado: o pyqtgraph escolhia o PyQt6 antes do PySide6 e as DLLs do Qt conflitavam. Corrigido: o projeto fixa `PYQTGRAPH_QT_LIB=PySide6` e importa o PySide6 antes do pyqtgraph. Se o erro persistir, use um ambiente virtual limpo (`python -m venv .venv`) e atualize o Visual C++ Redistributable x64.
