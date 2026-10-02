# GCM — Testes individuais

Repositório de testes de bancada (hardware e firmware) da **GCM (General Control Module)**, a central eletrônica do robô sumô da disciplina **Projeto Integrador 2** (FCTE – Universidade de Brasília).

Cada teste isola um bloco da placa (núcleo lógico, telemetria de potência, expansor de I/O, sensor ToF, descarga da bateria) para validar o hardware antes da integração no firmware final do robô. O repositório também documenta a investigação de uma falha no chicote entre a GCM e a placa auxiliar ToFaB-i2c.

## Sobre a GCM

- **MCU:** ESP32-WROOM-32D (Wi-Fi + Bluetooth)
- **Alimentação:** `V_SUPPLY` de 12–24 V (bateria 4S no robô) → buck LM2596 (+5 V) → LDO LM1117 (+3,3 V)
- **Comunicação:** USB isolado (ADuM3160 + CH340C com auto-download), CAN (TJA1050, opcional) ou, alternativamente, GPIOs puras no lugar do CAN
- **Telemetria de potência:** ADS1115 (16 bits, I2C `0x49`) lendo corrente e tensão da lógica (ACS712-05B) e de uma carga externa (ACS758, ±50 A), via level shifter I2C 5 V ↔ 3,3 V
- **Proteções:** MOSFET canal P contra inversão de polaridade e varistor S10K20V contra surtos
- **Conector principal:** CN15 (27 pinos) com alimentação, I2C1, shift register 74HC595, sinais de motor/atuador (DIR, EN, PWM) e GPIOs

A descrição completa está em [`HW_characteristics/`](HW_characteristics).

## Estrutura do repositório

```
GCM_testes_individuais/
├── Firm_tests/                     # Projetos ESP-IDF 5.4.2 (um por teste; em .zip, exceto o último)
│   ├── gcm_core_test.zip
│   ├── gcm_telemetry_i2c0.zip
│   ├── pca9554_test.zip
│   ├── vl53l1x_esp_idf.zip
│   └── battery_discharge_test/     # firmware + app Python (pasta, sem .zip)
├── HW_characteristics/
│   ├── descricao_gcm.txt           # Visão geral da GCM
│   └── resumo-hardware-placa.md    # Revisão bloco a bloco do esquemático
└── HW_tests/
    └── relatorio_perda_gnd_tofab_gcm.md   # Relatório: perda de GND no chicote GCM ↔ ToFaB
```

## Testes de firmware

Todos usam **ESP-IDF 5.4.2**, alvo **ESP32** (WROOM clássico) e o driver novo `i2c_master` do ESP-IDF 5.x. Extraia o `.zip` desejado antes de compilar (o `battery_discharge_test` já é uma pasta).

| Projeto | Barramento / pinos | O que valida |
|---|---|---|
| `gcm_core_test` | I2C0: SDA = GPIO21, SCL = GPIO22 (ADS1115 @ `0x49`) | Bring-up do núcleo lógico: lê os 4 canais do ADS1115, amostra GPIO34/35 (somente entrada) e percorre todas as saídas expostas no CN15 acendendo uma de cada vez. Saída serial em CSV (`;`). |
| `gcm_telemetry_i2c0` | I2C0: SDA = GPIO21, SCL = GPIO22 (ADS1115 @ `0x49`) | Telemetria de potência já convertida para unidades físicas (V e A). O I2C1 não é usado. |
| `pca9554_test` | I2C1: SDA = GPIO18, SCL = GPIO19 (PCA9554A @ `0x38`) | Valida o expansor de I/O que substituiu o PCF8574 e entrega um driver reutilizável (`components/pca9554`). Faz blink das 8 GPIOs com verificação por readback. |
| `vl53l1x_esp_idf` | I2C: SDA = GPIO21, SCL = GPIO22 (VL53L1X @ `0x29`) | Driver mínimo, em nível de registrador, do sensor ToF VL53L1X: lê o Model ID (`0xEACC`) e faz ranging contínuo por polling. |
| `battery_discharge_test` | I2C0: SDA = GPIO21, SCL = GPIO22 (ADS1115 @ `0x49`); GPIO23 = enable da carga | Descarga controlada da bateria do robô com cutoff ajustável (padrão 12 V). Ver a seção própria abaixo. |

### Escalas da telemetria (`gcm_telemetry_i2c0`)

| Canal | Sinal | Fórmula |
|---|---|---|
| AIN0 | `PWR_VOLTAGE_SENS` (divisor 4,7 kΩ / 1 kΩ) | `V_batt = V_AIN0 × 5,7` |
| AIN1 | `PWR_CURRENT_SENS` (ACS758LCB-050B, 40 mV/A) | `I = (V_AIN1 − 2,5) / 0,040` |
| AIN2 | `LOGIC_VOLTAGE_SENSE` (divisor 4,7 kΩ / 1 kΩ) | `V_logic = V_AIN2 × 5,7` |
| AIN3 | `LOGIC_CURRENT_SENS` (ACS712-05B, 185 mV/A) | `I = (V_AIN3 − 2,5) / 0,185` |

O PGA do ADS1115 está em ±6,144 V e a leitura usa polling do bit `OS`, em vez de um delay fixo (que causava vazamento entre canais por truncamento de `vTaskDelay` com tick de 10 ms).

### Compilar e gravar

```bash
. $IDF_PATH/export.sh            # ambiente ESP-IDF 5.4.2
cd <projeto>
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSBx flash monitor
```

Cada projeto traz mais detalhes (ligações, saída esperada, API do driver) no próprio `README.md`, quando existe.

> **Atenção:** os `.zip` de `Firm_tests/` incluem os diretórios `build/` (~32 MB cada). Ao extrair, o código-fonte útil está em `main/`, `components/`, `CMakeLists.txt` e `sdkconfig*`.
>
> **Atenção:** o `vl53l1x_esp_idf` não chegou a ser compilado no ambiente em que foi gerado; revise a saída do primeiro `idf.py build`.

## Teste de descarga da bateria

Pasta: [`Firm_tests/battery_discharge_test`](Firm_tests/battery_discharge_test) (detalhes completos no `README.md` dela).

Usa o sensor externo da GCM (ACS758 + divisor de tensão, via ADS1115) para monitorar a descarga da bateria do robô por meio de uma carga eletrônica fixa. O firmware amostra tensão e corrente (10 Hz), integra Ah e Wh, estima a resistência interna e **desliga a carga ao atingir o cutoff** (padrão 12 V, ajustável por `menuconfig` ou pelo comando serial `CUTOFF`). O cutoff é decidido no firmware, então o teste termina certo mesmo que o PC desconecte.

```
bateria (+) ──► PWR_PAD_BATT ──[ACS758]──► PWR_PAD_SENS ──► carga eletrônica ──► bateria (−)
```

- A GCM é alimentada por **fonte de bancada** (não pela bateria) e a ToFaB fica desconectada.
- O enable da carga é o **GPIO23** (CN15 pino 8), com polaridade configurável. Coloque um resistor de pull externo no enable, para a carga ficar desligada durante o boot e após reset.
- O app `tools/battery_monitor.py` (Python, Tkinter) envia **Tara** e **START**, mostra os gráficos ao vivo (tensão × tempo, corrente × tempo e tensão × Ah, selecionáveis) e salva o **CSV** a cada teste, mais um `_resumo.txt`. Também exporta os três gráficos como PNG separados. Os arquivos saem na mesma pasta do script.
- Proteções com a carga desligada e estado `FAULT`: sobrecorrente, ausência de corrente com a carga ligada, corrente com a carga desligada, bateria ausente ou abaixo do limite de partida, e falhas do ADS1115.

```bash
# firmware
cd Firm_tests/battery_discharge_test
idf.py set-target esp32 && idf.py menuconfig && idf.py build && idf.py -p /dev/ttyUSBx flash

# app de acompanhamento
pip install -r tools/requirements.txt
python tools/battery_monitor.py
```

> Ao abrir ou fechar a porta serial, alguns adaptadores reiniciam o ESP32 (DTR/RTS). Conecte o app *antes* do START e evite reconectar durante a descarga.

## Testes e relatórios de hardware

### Perda de GND no chicote GCM ↔ ToFaB-i2c

[`HW_tests/relatorio_perda_gnd_tofab_gcm.md`](HW_tests/relatorio_perda_gnd_tofab_gcm.md) (rascunho 0.1, 30/09/2026)

Em bancada, a desconexão do GND da ToFaB-i2c por mau contato no chicote queimou o ESP32 da GCM, duas vezes. O isolamento USB e o restante da placa sobreviveram. O relatório descreve:

- a arquitetura e o mecanismo provável da falha (hipótese a confirmar em ensaio: o GND local da ToFaB flutua perto da tensão da bateria e leva ~16 V às linhas SDA/SCL, ligadas direto aos GPIOs do ESP32);
- três alternativas de correção: alimentar a ToFaB com 5 V, level shifter MOSFET (BSN20) e isolador I2C;
- um plano de ensaio com fonte limitada em corrente;
- a **proposta final**: combinar alimentação da ToFaB em 5 V (com boost para os 12 V da fita de LED) com o isolador I2C **ADuM1250**, além de reforçar as conexões de GND. O componente já foi comprado, com entrega prevista para 28/10/2026.

## Observações para quem for usar a placa

- GPIO34 e GPIO35 são somente entrada e não aceitam ser dirigidos a partir do conector.
- Os pinos CANH/CANL viram GPIO32/GPIO33 apenas com o TJA1050 **não** populado e os resistores R40–R43 em 0 Ω; os dois modos são mutuamente exclusivos.
- O ESP32 não é tolerante a 5 V. Na GCM, o RXD do TJA1050 passa por um divisor (R28/R29) antes de chegar ao GPIO34.
- Não conecte o chicote energizado à GCM, e use fonte de bancada com limitação de corrente nos ensaios de falha.
- Confirme a variante do expansor montado: PCA9554 (`0x20–0x27`) e PCA9554A (`0x38–0x3F`) têm faixas de endereço diferentes.

## Autor

Paulo Caleb — Engenharia Eletrônica, UnB (FCTE).