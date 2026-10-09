# firm/ - firmware da base de testes (ESP-IDF 5.4.2, ESP32 WROOM)

Une os testes individuais (entradas, ToF + PCA9554A) e adiciona LEDs e motores controlados pelo protocolo da `PROTOCOL.md`.

## Build e gravação

```bash
idf.py set-target esp32
idf.py build
idf.py -p <porta> flash
idf.py -p <porta> monitor -b 921600     # console e protocolo em 921600 baud
```

**Baud (importante):** o `sdkconfig.defaults` coloca o console em *Custom UART* (UART0) a 921600. No ESP-IDF 5.4 o baud do console só é configurável nesse modo; com "Default: UART0" ele fica em 115200. O `sdkconfig.defaults` só é lido quando **não existe** `sdkconfig`: se você já compilou antes, apague `sdkconfig` e `build/` e rode `idf.py set-target esp32` de novo. Para conferir: `grep CONSOLE_UART sdkconfig` deve mostrar `CONFIG_ESP_CONSOLE_UART_CUSTOM=y` e `CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600`. As mensagens do ROM no reset continuam em 115200 (aparecem como lixo no monitor a 921600); o resto é legível.

O monitor do ESP-IDF mostra os logs e também as linhas JSON do protocolo (telemetria a 20 Hz). A GUI de `test_soft/` não deve ficar aberta ao mesmo tempo que o monitor (uma única porta).

## Sequência de boot (log)

1. `motors_init`: EN inativo, PWM = 0.
2. entradas + driver da UART.
3. I2C1 (GPIO18/19) + PCA9554A (0x38).
4. Autoteste dos LEDs: cada um 1 s, depois o LED 2 (IO6) fica aceso (`LED_FIXED_PIN`).
5. ToF: XSHUT um a um, endereços 0x30/0x31/0x32, ranging contínuo.
6. Tasks (entradas, ToF, protocolo) e mensagem `hello`.

Se o PCA9554A não responder, o firmware continua: entradas e motores funcionam, LEDs e ToF ficam fora (`"exp":0` no `hello`).

## Módulos (`main/`)

| Arquivo | Função |
|---|---|
| `app_config.h` | **Todos os pinos e parâmetros** (polaridades, PWM, endereços, limites padrão) |
| `main.c` | Ordem de boot e autoteste dos LEDs |
| `motors.c` | LEDC 20 kHz, EN/DIR, limite, rampa, inversão segura, watchdog |
| `inputs.c` | Polling com debounce de BS_1..4 e START_BOT, eventos |
| `expander.c` | PCA9554A com registrador-sombra + mutex (LEDs e XSHUT) |
| `tof.c` | 3x VL53L1X: atribuição de endereços via XSHUT e polling |
| `proto.c` | Tasks de RX/TX, comandos e mensagens JSON |
| `mini_json.c` | Parser JSON plano (sem dependências, testado no PC) |
| `proto_format.c` | Formatação do quadro de telemetria (testado no PC) |
| `components/pca9554`, `components/vl53l1x` | Drivers (o do VL53L1X ganhou troca de endereço I2C) |

## Premissas de hardware (ajustar em `app_config.h` se necessário)

- Interface das pontes H: **PWM + DIR por motor e EN_ALL comum**, com **sinal + magnitude** (DIR escolhe o sentido, PWM é o módulo). Se o seu módulo BTS7960 for do tipo RPWM/LPWM, essa premissa não vale; veja o README da raiz.
- `EN_ALL` ativo em nível alto (INH do BTS7960: alto = habilitado, baixo = sleep). `M1_DIR_FWD_LEVEL`/`M2_DIR_FWD_LEVEL` definem qual nível de DIR é "frente".
- PWM de 20 kHz (o BTS7960 aceita até 25 kHz).
- LEDs da fita ativos em nível alto (`LED_ACTIVE_HIGH`).

## Testes no PC (sem placa)

```bash
make -C host_tests run
```

Roda o parser JSON, o formatador de telemetria e uma **simulação da lógica de segurança dos motores** (EN inativo no boot, limite, rampa, DIR só troca com PWM = 0, STOP, watchdog) compilando `motors.c` com headers falsos.
