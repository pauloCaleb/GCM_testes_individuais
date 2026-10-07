# gcm_tof_vl53l1x

Teste de 3x VL53L1X no **I2C1 da GCM** (SDA=GPIO18, SCL=GPIO19), com os XSHUT
controlados pelo **PCA9554A** (0x38). ESP-IDF 5.4.2, alvo ESP32 (WROOM classic).

Baseado nos projetos `pca9554_test` (driver do expansor, copiado sem alteração) e
`vl53l1x_esp_idf` (driver do sensor, com 3 funções novas: `vl53l1x_wait_for_boot`,
`vl53l1x_set_i2c_address` e `vl53l1x_deinit`).

## Hardware assumido

| PCA9554A | Função |
|---|---|
| IO0 / IO1 / IO2 | XSHUT dos sensores S1 / S2 / S3 |
| IO7 / IO6 / IO5 | LEDs da fita (1º / 2º / 3º), ativos em nível alto |
| IO3 / IO4 | não usados (ficam como entrada) |

Endereços finais dos sensores: **S1 = 0x30, S2 = 0x31, S3 = 0x32**. Ficam fora de
0x29 (padrão do VL53L1X) e de toda a faixa do PCA9554 (0x20–0x27) e do PCA9554A
(0x38–0x3F).

## Sequência de boot

1. Sobe o I2C1 e confirma o PCA9554A por probe.
2. Escreve o Output Port (XSHUT em LOW, LEDs apagados) **antes** de configurar os
   pinos como saída, sem glitch no XSHUT/LED. Config final = `0x18`.
3. LEDs: cada um aceso sozinho por 1 s; depois um fica fixo aceso.
4. Atribuição de endereços: com todos os XSHUT em LOW (e um probe em 0x29 para
   checar a fiação), libera um sensor por vez, espera o boot, escreve o novo
   endereço no registrador `0x0001` e confere o Model ID (0xEACC) no endereço novo.
   Se um sensor falhar, o XSHUT dele volta a LOW para não colidir com o próximo.
5. `sensor_init` + modo de distância + timing budget + período + `start_ranging`
   em cada sensor.
6. Loop de polling (5 ms): checa data-ready dos três sem bloquear e imprime uma
   linha a cada 200 ms.

Saída esperada:

```
[   12345 ms]  S1(0x30):  523 mm  S2(0x31): 1021 mm  S3(0x32): ---- (st=4)
```
`OFFLINE` = sensor que não inicializou; `st=N` = leitura descartada (status do range).

## Ajustes (topo do `main/main.c`)

- `LED_FIXED_PIN`: qual LED fica aceso fixo (padrão: IO6).
- `LED_ACTIVE_HIGH`, `LED_ON_TIME_MS`.
- `TOF_ADDR_Sx`, `TOF_DISTANCE_MODE`, `TOF_TIMING_BUDGET_MS`, `TOF_INTER_MEAS_MS`.
- `PCA9554_I2C_ADDR` (0x38).

## Build

```
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSBx flash monitor
```

## Notas

- `sdkconfig.defaults` usa `CONFIG_FREERTOS_HZ=1000`: os drivers fazem
  `vTaskDelay(pdMS_TO_TICKS(2))` em loops de espera, que viram 0 ticks a 100 Hz.
  Os timeouts do `i2c_master` seguem em ms diretos.
- Os endereços dos VL53L1X são voláteis: voltam a 0x29 com qualquer reset via
  XSHUT. Por isso o firmware sempre derruba os três XSHUT antes de começar,
  inclusive após um reset do ESP32 com os sensores ainda energizados.
- Os pull-ups do I2C1 (R26/R27, 3,3 kΩ) só existem na placa se estiverem
  montados; o firmware também liga o pull-up interno (como nos projetos de
  referência), que sozinho é fraco para 400 kHz com 4 dispositivos.
- Não foi compilado com o toolchain do ESP-IDF; só passou checagem de sintaxe
  com headers simulados. Revise a saída do primeiro `idf.py build`.
