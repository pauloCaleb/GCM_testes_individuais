# VL53L1X + ESP-IDF 5.4.2

Firmware mínimo para ler distância de um sensor ToF ST VL53L1X via I2C,
usando o driver novo `i2c_master` do ESP-IDF 5.x (não o `driver/i2c.h` antigo).

## Estrutura

```
main/main.c                        -> inicializa I2C e faz o loop de leitura
components/vl53l1x/include/vl53l1x.h
components/vl53l1x/vl53l1x.c       -> driver próprio, register-level
```

## Ligação

| VL53L1X | ESP32           |
|---------|-----------------|
| VIN     | 3V3             |
| GND     | GND             |
| SDA     | GPIO18 = SDA1 (CN15-16, I2C1 da GCM) |
| SCL     | GPIO19 = SCL1 (CN15-3, I2C1 da GCM) |
| XSHUT   | não usado (deixar sempre em nível alto / não conectado, sensor sempre ativo) |

Endereço I2C padrão: `0x29`.

Pinos iguais aos da base de testes validada (`HW_tests/gcm_test_base`). Este projeto
fala com **um** sensor; para os 3 VL53L1X da ToFaB (XSHUT pelo PCA9554A) use
`Firm_tests/gcm_tof_vl53l1x`. O I2C0 (GPIO21/22) é do ADS1115 e não deve ser usado aqui.

## Compilar e gravar

```
idf.py set-target esp32          # ou esp32s3, esp32c3, etc.
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Não foi possível compilar este projeto no ambiente onde ele foi gerado
(sem o toolchain do ESP-IDF disponível) — revise a saída do `idf.py build`
com atenção na primeira compilação.

## O que o driver cobre

- Boot check + leitura do Model ID (deve ler `0xEACC`)
- Carga da tabela de configuração padrão de fábrica (registradores 0x2D–0x87)
- Seleção de modo de distância (curto/longo)
- Timing budget e período entre medições
- Ranging contínuo com polling de "dado pronto" via I2C (sem usar o pino de interrupção)
- Leitura de distância, status do range e taxas de sinal/ambiente (brutas, sem escala calibrada em MCPS)

## O que NÃO está implementado

- Múltiplos sensores no mesmo barramento (troca de endereço via XSHUT)
- Calibração de offset e crosstalk
- ROI customizada
- Uso do pino de interrupção física (GPIO1 do sensor) — aqui a leitura é sempre por polling

## Origem da lógica

A sequência de boot, a tabela de configuração padrão e os endereços de
registro reproduzem o comportamento descrito pela ST no UM2510 (VL53L1X
ULD API), replicado de forma equivalente em vários ports open-source
independentes (Pololu, SparkFun, ESPHome, mbed). Se algo se comportar de
forma inesperada, vale conferir esses valores contra o UM2510 antes de
mexer na lógica do driver.
