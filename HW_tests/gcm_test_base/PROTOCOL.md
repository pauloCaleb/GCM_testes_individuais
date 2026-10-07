# Protocolo serial da GCM (v1)

Transporte: UART0 do ESP32 -> CH340C -> USB isolado, **921600 baud, 8N1**.
Enquadramento: **uma mensagem JSON por linha** (terminada em `\n`), nos dois sentidos.

- Todas as mensagens do firmware têm o campo `"t"` (tipo).
- Linhas que **não** começam com `{` são logs do ESP-IDF (`I (1234) tag: ...`). A GUI mostra essas linhas no log e o cliente Python as devolve como `{"t":"log","line":...}`.
- O firmware só entende objetos JSON **planos** (sem objetos/arrays dentro dos comandos) e ignora linhas que não comecem com `{`. Linhas com mais de 255 bytes são rejeitadas (`linha_longa`).
- Qualquer comando pode levar `"seq": N` (inteiro >= 0); o firmware repete o mesmo `seq` no `ack`/`err`/`pong`/`hello` correspondente, para você casar pedido e resposta.

## Comandos (PC -> placa)

| Comando | Campos | Efeito |
|---|---|---|
| `hello` | - | Responde com a mensagem `hello`. Pode ser repetido à vontade. |
| `ping` | - | Responde `pong`. **Alimenta o watchdog** (heartbeat). |
| `led` | `idx`: 1..3 ou `"all"`, `on`: 0/1 | Liga/desliga LED da fita (1 = IO7, 2 = IO6, 3 = IO5 do PCA9554A). |
| `motor` | `ch`: 1..2, `duty`: -100..100 | Define o duty-alvo em % com sinal (positivo = frente). Limitado por `max_duty`. O `ack` traz o valor efetivamente aplicado. |
| `en` | `on`: 0/1 | Habilita/desabilita EN_ALL. **Ao habilitar, os alvos dos motores são zerados.** |
| `stop` | - | Parada imediata: duty 0, sem rampa, EN_ALL inativo. |
| `cfg` | qualquer combinação de `tel_hz` (1..50), `max_duty` (0..100), `wd_ms` (0 ou 50..10000), `slew` (0..2000 %/s) | Ajusta a configuração. É tudo-ou-nada: se um campo for inválido nada é alterado. |

Qualquer comando válido (inclusive `stop`) realimenta o watchdog.

Exemplos:

```
{"cmd":"led","idx":2,"on":1,"seq":7}
{"cmd":"en","on":1}
{"cmd":"motor","ch":1,"duty":-25.5}
{"cmd":"cfg","max_duty":50,"wd_ms":1000}
```

## Mensagens (placa -> PC)

### `hello`
Enviada no fim do boot e em resposta ao comando `hello`.

```json
{"t":"hello","fw":"gcm_test_base","ver":"1.0.0","proto":1,"exp":1,"tof":[1,1,1],"cap":30.0,"wd_ms":500,"slew":300.0,"tel_hz":20}
```
`exp` = PCA9554A respondeu (sem ele não há LEDs nem ToF). `tof` = sensores em operação. O firmware só responde depois de terminar o boot (autoteste dos LEDs + inicialização dos ToF, alguns segundos); por isso a GUI reenvia `hello` a cada 500 ms até receber a resposta.

### `tel` (telemetria periódica, `tel_hz` Hz, padrão 20)

```json
{"t":"tel","ms":123456,"bs":[1,0,1,1],"start":0,
 "tof":[{"on":1,"mm":523,"st":0,"age":12,"err":0},{"on":0},{"on":1,"mm":-1,"st":255,"age":0,"err":3}],
 "led":[0,1,0],"en":1,"m":[-12.5,0.0],"mt":[-30.0,0.0],"cap":30.0,"fs":0}
```

| Campo | Significado |
|---|---|
| `ms` | milissegundos desde o boot |
| `bs` | nível cru (0/1) de BS_1..BS_4 (GPIO34, 35, 16, 14) |
| `start` | nível cru de START_BOT (GPIO4) |
| `tof[i].on` | 1 = sensor em operação; se 0, o objeto só tem `on` |
| `tof[i].mm` | última distância (mm); -1 = ainda sem leitura |
| `tof[i].st` | status do range (0 = válida; 255 = sem leitura) |
| `tof[i].age` | ms desde a última leitura nova |
| `tof[i].err` | erros I2C acumulados |
| `led` | estado dos 3 LEDs (1 = aceso) |
| `en` | EN_ALL ativo |
| `m` | duty **aplicado** agora (%), com sinal |
| `mt` | duty **alvo** (%), com sinal |
| `cap` | limite de duty atual (%) |
| `fs` | 1 = failsafe (watchdog disparou e não foi limpo) |

Status do VL53L1X: 0 válida, 1 sigma alto, 2 sinal alto, 3 min. range, 4 sinal fraco, 5 sem sinal, 6 erro de HW, 7 warm-up, 9 merged pulse, 10 sinal muito fraco, 11 interferência, 12 erro de intervalo, 13 erro de calibração.

### `evt` (eventos imediatos)

```json
{"t":"evt","name":"bs","idx":2,"v":0,"ms":1234}        // BS_2 mudou para 0 (após debounce de 30 ms)
{"t":"evt","name":"start","idx":0,"v":1,"ms":1500}     // botão de start
{"t":"evt","name":"watchdog","idx":0,"v":1,"ms":9000}  // watchdog parou os motores
```

### `ack`, `err`, `pong`

```json
{"t":"ack","cmd":"motor","ch":1,"duty":30.0,"seq":4}
{"t":"ack","cmd":"cfg","tel_hz":20,"max_duty":50.0,"wd_ms":500,"slew":300.0}
{"t":"err","cmd":"motor","msg":"duty_invalido","seq":5}
{"t":"pong","ms":4321,"seq":6}
```

Mensagens de erro: `json_invalido`, `sem_cmd`, `cmd_desconhecido`, `linha_longa`, `falta_on`, `idx_invalido`, `ch_invalido`, `duty_invalido`, `tel_hz_invalido`, `max_duty_invalido`, `wd_ms_invalido`, `slew_invalido`, `pca9554_indisponivel`, `falha_i2c`.

## Segurança dos motores (comportamento do firmware)

1. **Boot seguro**: EN_ALL inativo e PWM = 0 antes de qualquer outra inicialização.
2. **Limite de duty** (`max_duty`, padrão **30 %**): nenhum alvo passa dele; se o limite for reduzido, o duty aplicado desce junto.
3. **Rampa** (`slew`, padrão 300 %/s) e **inversão segura**: para mudar de sentido o duty desce até 0, o pino DIR só troca com o PWM já em 0 e o motor espera 2 ticks (20 ms) antes de acelerar de novo.
4. **Watchdog** (`wd_ms`, padrão **500 ms**): se a ponte estiver ativa (EN ligado ou duty != 0) e nenhum comando válido chegar dentro de `wd_ms`, o firmware faz `stop`, emite o evento `watchdog` e liga `fs`. Para voltar a operar, envie `en` com `on:1`. Mande `ping` a cada ~150 ms (a GUI faz isso). Com `wd_ms: 0` o watchdog fica desligado.
5. `en` com `on:1` **zera os alvos**: ninguém sai girando com um valor antigo.

## Versionamento

`proto` no `hello` é a versão do protocolo (1). Mudanças incompatíveis incrementam esse número.
