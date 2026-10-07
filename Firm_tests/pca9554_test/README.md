# pca9554_test

Projeto ESP-IDF 5.4.2 para validar o PCA9554A que substituiu o PCF8574
danificado na auxboard, e servir de base de driver reutilizável pelo time de
software.

## Contexto

- Barramento: **I2C1 da GCM** (SDA = GPIO18, SCL = GPIO19 — mesmo bus
  reservado a sensores externos/ToF, CN15, nets SCL1/SDA1).
- Endereço detectado: **0x38** (PCA9554A com A0=A1=A2=GND).
  - Atenção: PCA9554 (sem "A") tem endereço fixo diferente (0x20-0x27);
    PCA9554A é 0x38-0x3F. Confirme no seu chip qual variante está montada.
- Alvo: **ESP32 WROOM classic** (não S3) — `sdkconfig.defaults` já fixa
  `CONFIG_IDF_TARGET=esp32`.

## Estrutura

```
pca9554_test/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   └── main.c              <- app de teste (blink de todas as 8 GPIOs)
└── components/
    └── pca9554/
        ├── CMakeLists.txt
        ├── include/pca9554.h
        └── pca9554.c        <- driver completo, reutilizável
```

## Build e flash

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSBx flash monitor
```

## O que o teste faz

1. Sobe o barramento I2C1 (driver novo `i2c_master_bus`, 400kHz).
2. Anexa o PCA9554A no endereço 0x38.
3. Escreve `0x00` no Configuration register → todos os 8 pinos como saída.
4. Lê de volta o Configuration register e confere se bateu com `0x00`.
5. Em loop, escreve `0xFF`/`0x00` no Output Port a cada 500ms, e a cada
   escrita faz um readback do próprio registrador para confirmar que o valor
   programado é o esperado (detecta problema de barramento/solda mesmo sem
   osciloscópio — mas o ideal ainda é conferir os pinos fisicamente com
   multímetro/osciloscópio no primeiro teste).

## Driver `pca9554.h` / `pca9554.c`

Cobre os 4 registradores do PCA9554/PCA9554A (Table 4 do datasheet NXP,
Rev. 10):

| Registrador | Comando | Função | Acesso |
|---|---|---|---|
| Input Port | 0x00 | nível lógico atual dos pinos | leitura |
| Output Port | 0x01 | estado programado nos pinos configurados como saída | leitura/escrita |
| Polarity Inversion | 0x02 | inverte a leitura do Input Port bit a bit | leitura/escrita |
| Configuration | 0x03 | direção de cada pino (1=entrada, 0=saída; default=entrada) | leitura/escrita |

API por registrador completo (8 bits de uma vez):
- `pca9554_read_input_port`
- `pca9554_write_output_port` / `pca9554_read_output_port`
- `pca9554_write_polarity_inversion` / `pca9554_read_polarity_inversion`
- `pca9554_write_config` / `pca9554_read_config`

API de conveniência por pino individual (0-7), todas via read-modify-write
(seguro porque os registradores R/W retornam o valor programado, não o pino
físico — datasheet §6.1.3, §6.1.4, §6.1.5):
- `pca9554_set_pin_direction`
- `pca9554_set_pin_level`
- `pca9554_get_pin_level` (lê do Input Port)
- `pca9554_set_pin_polarity_inversion`

### Notas de implementação

- O PCA9554 **não tem auto-increment**: uma vez enviado o command byte, o
  mesmo registrador é acessado até um novo command byte ser enviado. O driver
  reenvia o command byte em toda leitura, então isso é transparente.
- Timeout de transação I2C é definido em milissegundos diretos
  (`PCA9554_I2C_XFER_TIMEOUT_MS`, 1000ms) — a API nova do IDF 5.4.x
  (`i2c_master_transmit`/`i2c_master_transmit_receive`) já recebe o timeout
  em ms, não em ticks, então não há risco do bug de truncamento em
  `pdMS_TO_TICKS` com tick rate baixo.
- Clock configurado a 400kHz (Fast-mode), suportado pelo PCA9554/PCA9554A
  (diferente do PCF8574 antigo, limitado a 100kHz).

## Próximos passos sugeridos para o time de software

- Se o teste confirmar o expander funcionando, portar o controle dos XSHUT
  dos VL53L0X para usar `pca9554_set_pin_direction` + `pca9554_set_pin_level`
  no lugar do driver antigo do PCF8574.
- Considerar usar o Polarity Inversion register em vez de inverter em
  software, se algum sinal de entrada vier invertido por hardware.
- Adicionar suporte a interrupção (pino INT, open-drain) se for necessário
  reagir a mudanças de entrada sem polling — não implementado neste driver
  ainda.
