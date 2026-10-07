# gcm_telemetry_i2c0

Projeto ESP-IDF 5.4.2 para leitura das tensões e correntes da GCM-PI2-2026.2
via I2C0 (SDA=GPIO21, SCL=GPIO22 → ADS1115 @0x49 através do level shifter).

O I2C1 (CN15 / SCL1-SDA1, barramento externo para sensores auxiliares) não é
usado neste firmware.

## Build

```bash
. $IDF_PATH/export.sh          # ambiente ESP-IDF 5.4.2
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSBx flash monitor
```

## Saída esperada (serial monitor)

```
PWR_VOLTAGE_SENS (batt) :  0.000 V  (raw AIN0 =  0.000 V)   # pads flutuando na bancada
PWR_CURRENT_SENS (ext)  :  0.050 A  (raw AIN1 =  2.502 V)   # ACS758 em repouso
LOGIC_VOLTAGE_SENSE     : 11.700 V  (raw AIN2 =  2.053 V)
LOGIC_CURRENT_SENS      :  0.108 A  (raw AIN3 =  2.520 V)   # ACS712 em repouso
--------------------------------------------------------
```

## Escalas aplicadas

| Canal | Origem física | Fórmula |
|---|---|---|
| `PWR_VOLTAGE_SENS` (AIN0) | Divisor R8(4,7k)/R15(1k) a partir de `Vl_PWR` | `V_batt = V_AIN0 * 5,7` |
| `PWR_CURRENT_SENS` (AIN1) | ACS758LCB-050B, ±50 A, 40 mV/A, Vcc=5V | `I = (V_AIN1 - 2,5) / 0,040` |
| `LOGIC_VOLTAGE_SENSE` (AIN2) | Divisor R4(4,7k)/R6(1k) na alimentação lógica protegida | `V_logic = V_AIN2 * 5,7` |
| `LOGIC_CURRENT_SENS` (AIN3) | ACS712-05B, ±5 A, 185 mV/A, Vcc=5V | `I = (V_AIN3 - 2,5) / 0,185` |

## Observações

- PGA do ADS1115 configurado em ±6,144 V (não o padrão ±4,096 V) para não
  saturar `PWR_VOLTAGE_SENS`, cuja escala foi dimensionada para até ~5,3 V.
- Polling do bit `OS` do registrador de config em passos de 1 ms / timeout
  50 ms — mesmo padrão já validado no `gcm_core_test` para evitar truncamento
  de `vTaskDelay` abaixo do tick do FreeRTOS.
- Pull-ups internos do ESP32 desativados (`enable_internal_pullup = false`),
  pois R9–R12 já cumprem esse papel no hardware.
- O sinal `IP+`/`IP-` do ACS758 assume corrente de `PWR_PAD_BATT` para
  `PWR_PAD_SENS`; corrente negativa reportada indica fluxo invertido (dentro
  da faixa bidirecional do LCB-050B).
