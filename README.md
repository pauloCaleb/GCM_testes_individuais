# GCM — Testes individuais

Repositório de testes de bancada (hardware e firmware) da **GCM (General Control Module)**, a central eletrônica do robô sumô da disciplina **Projeto Integrador 2** (FCTE – Universidade de Brasília).

Cada teste isola um bloco da placa (núcleo lógico, telemetria de potência, expansor de I/O, sensores ToF, entradas, descarga da bateria) para validar o hardware antes da integração no firmware final do robô. O repositório também documenta a investigação de uma falha no chicote entre a GCM e a placa auxiliar ToFaB-i2c.

> **Referência de hardware:** o conjunto firmware + software Python de [`HW_tests/gcm_test_base`](HW_tests/gcm_test_base), testado em bancada com sensores e atuadores. O pinout de todos os projetos segue o [`app_config.h`](HW_tests/gcm_test_base/firm/main/app_config.h) dele.

## Sobre a GCM

- **MCU:** ESP32-WROOM-32D (Wi-Fi + Bluetooth, 4 MB de flash)
- **Alimentação:** `V_SUPPLY` de 12–24 V (bateria 4S no robô) → buck LM2596 (+5 V) → LDO LM1117 (+3,3 V)
- **Comunicação:** USB isolado (ADuM3160 + CH340C com auto-download) e CAN (TJA1050). Sem o TJA1050, GPIO32/33 podem ir ao CN15 como GPIOs (R40–R43)
- **Telemetria de potência:** ADS1115 (16 bits, I2C0 `0x49`) lendo corrente e tensão da lógica (ACS712-05B) e de uma carga externa (ACS758, ±50 A), via level shifter I2C 5 V ↔ 3,3 V
- **Proteções:** MOSFET canal P contra inversão de polaridade e varistor S10K20V contra surtos
- **Conector principal:** CN15 (27 pinos) com alimentação, I2C1, pontes H (EN_ALL, PWM, DIR), sensores de borda e botão de start

A descrição completa está em [`HW_characteristics/`](HW_characteristics).

### Pinout do CN15

| Função | GPIO | Pino CN15 | Obs. |
|---|---|---|---|
| BS_1 / BS_2 | 34 / 35 | 21 / 9 | sensores de borda; pull-up externo R23 / R22 |
| BS_3 / BS_4 | 16 / 14 | 19 / 18 | sensores de borda; pull-up interno |
| START_BOT | 4 | 20 | pull-up interno |
| EN_ALL | 23 | 8 | habilita as pontes H, fio roxo |
| PWM1 / DIR_1 | 13 / 17 | 7 / 6 | motor 1, fios cinza / branco |
| PWM2 / DIR2 | 27 / 25 | 5 / 4 | motor 2, fios preto / marrom |
| SDA1 / SCL1 | 18 / 19 | 16 / 3 | I2C1: PCA9554A (`0x38`) + 3x VL53L1X (`0x30`/`0x31`/`0x32`) |
| PCA9554_INT | 26 | 17 | interrupção do expansor |
| CANL / CANH | 32 / 33 | 12 / 13 | CAN (TJA1050 montado) ou GPIO |

PCA9554A da ToFaB: IO0..IO2 = XSHUT dos ToF S1..S3, IO7/IO6/IO5 = LEDs 1/2/3 da fita (ativos em alto).

## Estrutura do repositório

```
GCM_testes_individuais/
├── HW_characteristics/              # Esquemático (PDF), visão geral e revisão bloco a bloco
├── HW_tests/
│   ├── gcm_test_base/               # REFERÊNCIA: firmware + GUI PySide6 (sensores, ToF, LEDs, motores)
│   └── relatorio_perda_gnd_tofab_gcm.md
├── Energy_Telemetry/
│   └── battery_discharge_test/      # Descarga da bateria: firmware + app Python (USB ou Bluetooth)
├── Firm_tests/                      # Testes isolados (ESP-IDF 5.4.2)
│   ├── gcm_core_test/
│   ├── gcm_telemetry_i2c0/
│   ├── gcm_gpio_poll_sensors/
│   ├── pca9554_test/
│   ├── vl53l1x_esp_idf/
│   └── gcm_tof_vl53l1x/
├── ROBOT_FREERTOS/                  # Início do firmware do robô (ToFaB + bordas com interrupção)
└── atom_sparring_bot_FW/            # Robô de sparring ATOM (outra placa, ver abaixo)
```

## Projetos

Todos usam **ESP-IDF 5.4.2**, alvo **ESP32** (WROOM clássico) e o driver novo `i2c_master`.

| Projeto | Barramento / pinos | O que valida |
|---|---|---|
| [`HW_tests/gcm_test_base`](HW_tests/gcm_test_base) | CN15 completo + I2C1 | Base de testes validada: entradas, 3 ToF, LEDs da fita e 2 motores (BTS7960), com GUI e protocolo JSON a 921600 baud. |
| [`Energy_Telemetry/battery_discharge_test`](Energy_Telemetry/battery_discharge_test) | I2C0 (ADS1115 @ `0x49`); GPIO23 = enable da carga | Descarga controlada da bateria com cutoff no firmware, calibração e análise. Comece pelo [manual](Energy_Telemetry/battery_discharge_test/tools/MANUAL_USO_E_CALIBRACAO.md). |
| `Firm_tests/gcm_core_test` | I2C0 + saídas/entradas do CN15 | Bring-up do núcleo lógico: lê os 4 canais do ADS1115, amostra as entradas e acende uma saída do CN15 por vez. Saída CSV (`;`). **Rode com as pontes H desconectadas.** |
| `Firm_tests/gcm_telemetry_i2c0` | I2C0 (ADS1115 @ `0x49`) | Telemetria de potência convertida para V e A. |
| `Firm_tests/gcm_gpio_poll_sensors` | BS_1..BS_4, START_BOT | Polling das entradas digitais com debounce opcional. |
| `Firm_tests/pca9554_test` | I2C1 (PCA9554A @ `0x38`) | Driver do expansor (`components/pca9554`) com blink das 8 GPIOs e readback. |
| `Firm_tests/vl53l1x_esp_idf` | I2C1 (VL53L1X @ `0x29`) | Driver mínimo do VL53L1X com **um** sensor: Model ID (`0xEACC`) e ranging contínuo. |
| `Firm_tests/gcm_tof_vl53l1x` | I2C1 (PCA9554A + 3x VL53L1X) | Os 3 ToF da ToFaB: XSHUT pelo expansor, reendereçamento e polling. |
| `ROBOT_FREERTOS` | I2C1 + BS_1..BS_4 | Primeiro esboço do firmware do robô: ToFaB e sensores de borda com interrupção e latch. |

### Escalas da telemetria (`gcm_telemetry_i2c0`)

| Canal | Sinal | Fórmula |
|---|---|---|
| AIN0 | `PWR_VOLTAGE_SENS` (divisor 4,7 kΩ / 1 kΩ) | `V_batt = V_AIN0 × 5,7` |
| AIN1 | `PWR_CURRENT_SENS` (ACS758LCB-050B, 40 mV/A) | `I = (V_AIN1 − 2,5) / 0,040` |
| AIN2 | `LOGIC_VOLTAGE_SENS` (divisor 4,7 kΩ / 1 kΩ) | `V_logic = V_AIN2 × 5,7` |
| AIN3 | `LOGIC_CURRENT_SENS` (ACS712-05B, 185 mV/A) | `I = (V_AIN3 − 2,5) / 0,185` |

O PGA do ADS1115 está em ±6,144 V e a leitura usa polling do bit `OS`, em vez de um delay fixo.

### Compilar e gravar

```bash
. $IDF_PATH/export.sh            # ambiente ESP-IDF 5.4.2
cd <projeto>
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSBx flash monitor
```

O `sdkconfig` e o `build/` **não** são versionados: cada projeto é configurado pelo seu `sdkconfig.defaults` (tick de 1 ms e flash de 4 MB, como na base de testes). Se você tem um `sdkconfig` antigo na pasta, apague-o antes do `set-target` para os defaults valerem.

## Testes e relatórios de hardware

### Perda de GND no chicote GCM ↔ ToFaB-i2c

[`HW_tests/relatorio_perda_gnd_tofab_gcm.md`](HW_tests/relatorio_perda_gnd_tofab_gcm.md) (rascunho 0.1, 30/09/2026)

Em bancada, a desconexão do GND da ToFaB-i2c por mau contato no chicote queimou o ESP32 da GCM, duas vezes. O isolamento USB e o restante da placa sobreviveram. O relatório descreve:

- a arquitetura e o mecanismo provável da falha (hipótese a confirmar em ensaio: o GND local da ToFaB flutua perto da tensão da bateria e leva ~16 V às linhas SDA/SCL, ligadas direto aos GPIOs do ESP32);
- três alternativas de correção: alimentar a ToFaB com 5 V, level shifter MOSFET (BSN20) e isolador I2C;
- um plano de ensaio com fonte limitada em corrente;
- a **proposta final**: combinar alimentação da ToFaB em 5 V (com boost para os 12 V da fita de LED) com o isolador I2C **ADuM1250**, além de reforçar as conexões de GND. O componente já foi comprado, com entrega prevista para 28/10/2026.

## ATOM — robô de sparring

[`atom_sparring_bot_FW`](atom_sparring_bot_FW) é o firmware do robô de sparring usado para treinar o algoritmo do ARES, controlado por BLE pelo app Dabble. Ele roda em **outra placa** (ponte H com IN1/IN2/EN12 em GPIO19/5/18 e IN3/IN4/EN34 em GPIO12/21/22), então o pinout da GCM não se aplica a ele.

## Observações para quem for usar a placa

- GPIO34 e GPIO35 são somente entrada e não têm pull-up interno (os pull-ups R23/R22 estão na placa).
- Os pinos CANH/CANL viram GPIO33/GPIO32 apenas com o TJA1050 **não** populado e R41/R43 montados; os dois modos são mutuamente exclusivos.
- O ESP32 não é tolerante a 5 V, e as entradas do CN15 não têm proteção. Na GCM, o RXD do TJA1050 passa por um divisor (R28/R29) antes de chegar ao GPIO33.
- Os pull-ups R26/R27 do I2C1 precisam estar montados para PCA9554A + 3 sensores a 400 kHz.
- Antes do firmware subir, os GPIOs ficam flutuando: use pull-down em `EN_ALL` (e nos PWM) no lado da ponte H.
- Não conecte o chicote energizado à GCM, e use fonte de bancada com limitação de corrente nos ensaios de falha.
- Confirme a variante do expansor montado: PCA9554 (`0x20–0x27`) e PCA9554A (`0x38–0x3F`) têm faixas de endereço diferentes.

## Autor

Paulo Caleb — Engenharia Eletrônica, UnB (FCTE).
