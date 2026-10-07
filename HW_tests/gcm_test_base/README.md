# GCM-PI2-2026.2 - Base de testes da central eletrônica

Firmware + GUI para validar a GCM e os periféricos do robô sumô (PI2): sensores de borda, botão de start, 3 sensores de distância VL53L1X, LEDs da fita e 2 motores nas pontes H BTS7960.

```
gcm_test_base/
├── PROTOCOL.md      protocolo serial (JSON por linha, 921600 baud)
├── firm/            firmware ESP-IDF 5.4.2 (ESP32 WROOM)
└── test_soft/       GUI PySide6, cliente Python, simulador e testes
```

## Início rápido

1. **Firmware**: `cd firm && idf.py set-target esp32 && idf.py build && idf.py -p <porta> flash` (detalhes em `firm/README.md`).
2. **GUI**: `cd test_soft && pip install -r requirements.txt && python run_gui.py` (ou `--demo` para testar sem placa).
3. Escolha a porta, clique em **Conectar** e aguarde o `hello` (alguns segundos: o firmware faz o autoteste dos LEDs e a inicialização dos ToF).

## Pinout usado (CN15)

| Função | GPIO | Pino CN15 | Obs. |
|---|---|---|---|
| BS_1 / BS_2 | 34 / 35 | 21 / 9 | pull-up externo R23 / R22 |
| BS_3 / BS_4 | 16 / 14 | 19 / 18 | pull-up interno ligado |
| START_BOT | 4 | 20 | pull-up interno ligado |
| EN_ALL | 23 | 8 | fio roxo |
| PWM1 / DIR_1 | 13 / 17 | 7 / 6 | fios cinza / branco (motor 1) |
| PWM2 / DIR2 | 27 / 25 | 5 / 4 | fios preto / marrom (motor 2) |
| SDA1 / SCL1 | 18 / 19 | 16 / 3 | I2C1: PCA9554A (0x38) + 3x VL53L1X |

PCA9554A: IO0..IO2 = XSHUT dos ToF (S1..S3), IO7/IO6/IO5 = LEDs 1/2/3. VL53L1X em 0x30/0x31/0x32.

## Premissas e cuidados (leia antes de ligar os motores)

- **Interface das pontes H**: o firmware assume PWM + DIR + EN com **sinal e magnitude**. O módulo BTS7960 de prateleira (IBT-2) tem entradas **RPWM/LPWM** (uma por sentido) e **R_EN/L_EN**, não PWM/DIR. Se o seu módulo for assim, a lógica entre os pinos PWM/DIR da GCM e o módulo (placa intermediária, ligação direta etc.) define se esta premissa vale. **Confirme antes de ligar motores**; se a conversão não existir, a geração de PWM em dois canais por motor precisa ser ajustada em `motors.c`.
- `EN_ALL` é tratado como ativo em nível alto (INH do BTS7960), e "frente" como DIR em nível alto; ambos em `app_config.h`.
- Antes do firmware subir, os GPIOs ficam flutuando: use pull-down em `EN_ALL` (e nos PWM) no lado da ponte H.
- Primeiro teste: **motores sem carga/fora do chão**, limite de duty no padrão (30 %), mão no botão **PARAR** (Esc / Espaço).
- As entradas do CN15 não têm proteção: sinais externos acima de 3,3 V danificam o ESP32. Os pull-ups R26/R27 do I2C1 precisam estar montados para PCA9554A + 3 sensores a 400 kHz.
- O ESP32 usa o mesmo CH340C para gravar e para este protocolo: feche a GUI antes de gravar.

## Fora do escopo desta versão

Telemetria de potência (ADS1115), CAN e interrupção do PCA9554 (`PCA9554_INT`, GPIO26) ficam para a próxima fase.

## Estado da verificação

- Firmware: **não foi compilado com o ESP-IDF** (o toolchain não estava disponível onde foi gerado). Foi verificado com checagem de sintaxe contra headers simulados e com testes no PC (`make -C firm/host_tests run`): parser JSON, formatador de telemetria e simulação da lógica de segurança dos motores. Revise a saída do primeiro `idf.py build`.
- GUI/cliente: testados com o simulador do firmware (`python -m pytest -q`, 24 testes) e exercitados em modo offscreen. **Não** foram testados com a placa real.
