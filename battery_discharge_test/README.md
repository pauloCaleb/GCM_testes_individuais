# battery_discharge_test

Teste de **descarga controlada da bateria do robô** usando a GCM como instrumento de medição. A carga eletrônica fixa descarrega a bateria; o ESP32 mede tensão e corrente (ACS758, ADS1115), integra Ah/Wh, corta a carga na tensão de **cutoff (padrão 12 V, ajustável)** e envia tudo pela serial. Um app Python (Tkinter) envia os comandos de tara/start, plota ao vivo e salva os dados em CSV.

```
battery_discharge_test/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild     # parâmetros do teste (idf.py menuconfig)
│   └── main.c                # firmware (ESP-IDF 5.4.2, ESP32)
└── tools/
    ├── battery_monitor.py    # app Tkinter: controle, gráficos e CSV
    └── requirements.txt
```

## Ligações

A GCM é alimentada por **fonte de bancada** (não pela bateria). A ToFaB fica **desconectada** neste teste.

```
bateria (+) ──► PWR_PAD_BATT ──[ACS758]──► PWR_PAD_SENS ──► carga eletrônica ──► bateria (−)
```

| Sinal | Origem | Uso |
|---|---|---|
| AIN0 `PWR_VOLTAGE_SENS` | divisor R8/R15 a partir de `PWR_PAD_BATT` (razão 5,7) | tensão da bateria sob carga |
| AIN1 `PWR_CURRENT_SENS` | ACS758LCB-050B (40 mV/A, zero ≈ Vcc/2) | corrente (positiva = descarga) |
| GPIO23 (CN15 pino 8) | saída do ESP32 | enable da carga (ativo em nível alto por padrão) |

- O **GND da bateria precisa estar ligado ao GND da GCM** (referência do divisor e do ACS758).
- O **retorno da corrente da carga deve ir direto ao negativo da bateria**, sem passar por trilhas da GCM.
- Coloque um **resistor de pull externo no enable da carga** (pull-down se ativo em alto, pull-up se ativo em baixo), na ordem de 10 kΩ. Durante o boot e após um reset do ESP32 o pino fica fora do controle do firmware, e o resistor garante a carga desligada.
- O GPIO23 é lógica 3,3 V: confirme que a entrada de enable da carga reconhece esse nível.

## Fluxo do teste

```
IDLE ──TARE──► READY ──START──► REST ──► DISCHARGE ──cutoff/STOP──► RECOVERY ──► DONE
                 ▲                 └──────────── qualquer erro ────────────────► FAULT
                 └───────── TARE ou RESET (a partir de DONE/FAULT) ◄────────────────┘
```

1. **Tara** (carga desligada): média de 200 leituras do ACS758 define o zero. É rejeitada se o zero estiver a mais de 150 mV de 2,5 V. Informa também o ruído em mA.
2. **START**: a carga fica desligada por 3 s (**REST**) para medir a tensão em repouso. Só segue se houver bateria e se a tensão em repouso for maior que `cutoff + 200 mV`.
3. **DISCHARGE**: GPIO23 liga a carga. O firmware integra Ah e Wh e estima a resistência interna `(V_repouso − V_carga) / I` numa janela de 1 s, 1,5 s depois de ligar.
4. **Cutoff**: 5 amostras consecutivas abaixo do cutoff desligam a carga (`CUTOFF`). `STOP` ou o tempo máximo (4 h) também encerram.
5. **RECOVERY**: carga desligada, registro por mais 60 s. A média dos últimos 5 s dá a tensão recuperada e o **zero final do ACS758**, que mostra a deriva do offset durante o teste.
6. **DONE**: o firmware imprime o resumo (`E;SUMMARY`) e continua transmitindo as medidas.

O cutoff é decidido **no firmware**: se o PC desconectar ou o Python for fechado, a descarga termina normalmente.

### Proteções (carga desligada e estado `FAULT`)

| Motivo | Condição |
|---|---|
| `OVERCURRENT` | corrente acima de 30 A por 3 amostras |
| `NO_CURRENT` | corrente abaixo de 100 mA, 1,5 s após ligar (carga desconectada, enable errado) |
| `CURRENT_WITH_LOAD_OFF` | corrente acima de 300 mA com a carga desligada, no REST |
| `NO_BATTERY` | tensão em repouso abaixo de 5 V |
| `BATTERY_BELOW_START_LIMIT` | tensão em repouso ≤ cutoff + 200 mV |
| `ADC_FAIL` | 5 falhas consecutivas do ADS1115 |

Task watchdog com panic: se a task travar, o ESP32 reinicia e o pull externo desliga a carga.

## Firmware

```bash
. $IDF_PATH/export.sh            # ESP-IDF 5.4.2
cd battery_discharge_test
idf.py set-target esp32
idf.py menuconfig                # "Battery discharge test": cutoff, limites, período...
idf.py build
idf.py -p /dev/ttyUSBx flash monitor
```

O `sdkconfig.defaults` fixa `CONFIG_FREERTOS_HZ=1000` (necessário para o polling do ADS1115, e o build falha com um `#error` se o tick for outro). Ele só vale numa configuração nova: se já existir um `sdkconfig`, rode `idf.py fullclean` e apague o `sdkconfig`.

### Principais parâmetros (`menuconfig`)

| Opção | Padrão | Descrição |
|---|---|---|
| `BATT_CUTOFF_MV` | 12000 | cutoff inicial |
| `BATT_CUTOFF_MIN_MV` / `MAX_MV` | 10000 / 16000 | faixa aceita pelo comando `CUTOFF` |
| `BATT_CUTOFF_CONSEC_SAMPLES` | 5 | amostras abaixo do cutoff para disparar |
| `BATT_SAMPLE_PERIOD_MS` | 100 | período de amostragem e de saída |
| `BATT_OVERSAMPLE` | 4 | conversões do ADS1115 por canal em cada amostra |
| `BATT_MAX_CURRENT_MA` | 30000 | proteção de sobrecorrente (ajuste à sua carga) |
| `BATT_MIN_CURRENT_MA` | 100 | corrente mínima esperada (0 desabilita) |
| `BATT_MAX_TEST_S` | 14400 | duração máxima (0 = sem limite) |
| `BATT_RECOVERY_S` | 60 | registro após o cutoff |
| `BATT_V_DIV_RATIO_X10000` | 57000 | razão do divisor ×10000, para ajuste fino com multímetro |
| `BATT_I_SENS_UV_PER_A` | 40000 | sensibilidade do ACS758 (µV/A) |
| `BATT_LOAD_GPIO` / `BATT_LOAD_ACTIVE_HIGH` | 23 / sim | enable da carga |

### Protocolo serial (115200 8N1)

Comandos (um por linha, sem distinção de maiúsculas): `TARE`, `START`, `STOP`, `RESET`, `CUTOFF <mV ou V>`, `STATUS`, `HELP`. O `CUTOFF` é bloqueado em `REST` e `DISCHARGE`.

Respostas:

```
H;t_boot_s;t_test_s;state;v_batt_V;i_A;p_W;q_Ah;e_Wh;ain0_V;ain1_V
D;1050.315;1041.200;DISCHARGE;13.1964;4.4070;58.156;1.408971;20.66074;2.31516;2.68627
E;LOAD_OFF;CUTOFF;v_V=11.9973,i_A=3.9980,q_Ah=1.772389,e_Wh=25.30301,t_s=1348.6
E;SUMMARY;end=CUTOFF,dur_s=1348.6,q_Ah=1.772389,e_Wh=25.30301,i_avg_A=4.7313,...
S;state=READY,cutoff_mV=12000,tared=1,zero_V=2.50999,...
```

`D` traz uma linha por amostra. As colunas `ain0_V`/`ain1_V` são as tensões cruas do ADS1115, e permitem recalcular tensão e corrente depois com outro ganho ou offset. Qualquer linha fora desse formato (boot, `ESP_LOG`) é ignorada pelo app.

## App Python

```bash
pip install -r tools/requirements.txt     # pyserial, matplotlib (Tkinter vem com o Python)
python tools/battery_monitor.py
```

- **Conectar**, depois **Tara**, depois **START**. Os botões habilitam conforme o estado do firmware. O campo de cutoff é em volts, e o valor que vale é o confirmado pelo firmware.
- **Gráficos selecionáveis:** três checkboxes (*Tensão × tempo*, *Corrente × tempo*, *Tensão × Ah*). Os marcados dividem a área lado a lado; com um só, ele ocupa tudo. O zoom no tempo de V(t) e I(t) é ligado quando os dois estão visíveis. A linha de cutoff aparece nos gráficos de tensão e o eixo de tempo usa marcas legíveis (45s, 12m30s, 1h30). A escolha dos checkboxes e o auto-save são lembrados em `~/.gcm_battery_monitor.json`.
- **Zoom e pan:** ao dar zoom ou pan o auto-ajuste dos eixos para (aparece o aviso "vista manual"); o botão *Home* da barra volta a acompanhar os dados.
- **Exportar imagens (3 PNG)...:** gera três arquivos separados, `<nome>_tensao_tempo.png`, `<nome>_corrente_tempo.png` e `<nome>_tensao_capacidade.png`, a 200 dpi, com todos os pontos da sessão e independentemente de quais gráficos estão visíveis. O título traz data, motivo do fim, Ah, Wh e cutoff.
- A janela abre maximizada (`START_MAXIMIZED` no início do script).
- **Auto-salvar CSV** (ligado por padrão): a cada START cria `descarga_AAAAMMDD_HHMMSS.csv` na **mesma pasta do `battery_monitor.py`** (dá para trocar em *Pasta...*, só vale para aquela execução), com `host_time` + as colunas do firmware, e `..._resumo.txt` com o resumo e os eventos.
- **Salvar CSV...** (canto direito da barra de controle, sempre visível): exporta a sessão em memória quando quiser, inclusive no meio da descarga ou depois do DONE. Com o auto-save desligado, o app avisa se houver dados não salvos ao iniciar outro teste ou fechar a janela.
- A sessão grava do REST até DONE/FAULT.

> **Aviso (reset ao abrir/fechar a porta):** em muitos adaptadores USB-serial, abrir ou fechar a porta pode pulsar DTR/RTS e reiniciar o ESP32. O app abre a porta com DTR/RTS desativados e, no circuito de auto-download da GCM, os dois no mesmo nível não aciona EN/GPIO0. Confirme isso uma vez em bancada, **sem a carga ligada**. Em todo caso, conecte *antes* do START e evite reconectar durante a descarga: um reset desliga a carga e perde o teste.

## Precisão e limitações

- Resolução do ADS1115 (±6,144 V): LSB de 187,5 µV, o que dá ≈ 1,07 mV na tensão da bateria e ≈ 4,7 mA na corrente (antes da média).
- O zero do ACS758 é **ratiométrico** ao Vcc: 1 % de variação no +5 V desloca o zero ≈ 25 mV (≈ 0,6 A). A tara no início compensa o valor estático, mas a deriva ao longo do teste entra no Ah. Use `i_zero_end_mA` do resumo para avaliar o erro e, se for relevante, refaça a integral offline com as colunas cruas.
- Tensão e corrente são amostradas de forma intercalada (não simultânea); com 100 ms de período e cargas fixas o efeito é desprezível.
- A tensão medida é a do terminal sob carga (queda no fio e no ACS758 inclusa); a resistência interna estimada inclui essa queda.
- O cutoff de 12 V equivale a 3,0 V/célula numa bateria 4S. Confirme o valor mínimo de descarga no datasheet da sua bateria.

## Validação realizada

- O firmware foi **compilado no PC com stubs das APIs do ESP-IDF** e executado contra um modelo simulado de bateria 4S, ADS1115, ACS758 e carga. Cobriu: descarga completa até o cutoff (Ah, Wh e resistência interna conferem com o modelo), `STOP`, `CUTOFF` e seus limites, `OVERCURRENT`, `NO_CURRENT`, `NO_BATTERY`, `BATTERY_BELOW_START_LIMIT`, `ADC_FAIL` e comando desconhecido.
- O app Python foi testado de forma headless contra uma serial virtual reproduzindo a saída simulada: parser, gráficos, CSV, resumo e estados dos botões.
- Testado em bancada pelo autor com a carga eletrônica. Em um primeiro uso do zero, repita a validação com a fonte no lugar da bateria e uma carga pequena: confira tara, tensão e corrente contra o multímetro, o nível de GPIO23 e o desligamento no cutoff antes de usar a bateria real.
