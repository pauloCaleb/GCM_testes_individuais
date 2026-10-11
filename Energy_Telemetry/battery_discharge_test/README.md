# battery_discharge_test (v2)

Teste de **descarga controlada da bateria do robô** usando a GCM como instrumento de medição. A carga eletrônica fixa descarrega a bateria; o ESP32 mede tensão e corrente (ACS758 + ADS1115), integra Ah/Wh, **desliga a carga no cutoff (padrão 12 V, ajustável)** e envia tudo por **cabo USB ou Bluetooth**. O app Python (Tkinter) controla o teste, plota ao vivo, salva CSV e tem abas de **calibração** e **análise**.

```
battery_discharge_test/
├── CMakeLists.txt
├── partitions.csv            # NVS + app + partição "blog" (log em flash)
├── sdkconfig.defaults        # Bluetooth Classic, tick 1 ms, watchdog, 4 MB de flash
├── main/
│   ├── main.c                # máquina de estados, aquisição, comandos
│   ├── cal_store.c/.h        # coeficientes de calibração em NVS
│   ├── flash_log.c/.h        # log a 1 Hz na partição "blog"
│   ├── bt_link.c/.h          # Bluetooth Classic SPP espelhando a serial
│   └── Kconfig.projbuild     # parâmetros (idf.py menuconfig)
└── tools/
    ├── battery_monitor.py    # app Tkinter (abas Teste, Calibração, Análise)
    ├── bm_core.py            # protocolo, limites de eixo estáveis, toolbar
    ├── bm_calibration.py     # aba Calibração
    ├── bm_analysis.py        # aba Análise
    ├── requirements.txt
    └── MANUAL_USO_E_CALIBRACAO.md   # manual completo (instalação, rotina, calibração)
```

**Comece pelo [`tools/MANUAL_USO_E_CALIBRACAO.md`](tools/MANUAL_USO_E_CALIBRACAO.md).**

## Ligações

```
bateria (+) ──► PWR_PAD_BATT ──[ACS758]──► PWR_PAD_SENS ──► [relé opcional] ──► carga eletrônica ──► bateria (−)
```

A GCM é alimentada por **fonte de bancada** (não pela bateria) e a ToFaB fica desconectada.

| Sinal | Origem | Uso |
|---|---|---|
| AIN0 `PWR_VOLTAGE_SENS` | divisor R8/R15 a partir de `PWR_PAD_BATT` (razão 5,7) | tensão da bateria sob carga |
| AIN1 `PWR_CURRENT_SENS` | ACS758LCB-050B (40 mV/A, zero ≈ Vcc/2) | corrente (positiva = descarga) |
| GPIO23 (CN15 pino 8) | saída do ESP32 | enable da carga (ativo em alto por padrão) |
| GPIO16 (opcional) | saída do ESP32 | relé NA em série com a carga (`BATT_RELAY_ENABLE`) |

- O **GND da bateria** vai ao GND da GCM e o **retorno da carga** direto ao negativo da bateria.
- Coloque um **resistor de pull externo** no enable da carga (pull-down se ativo em alto): durante o boot e após reset o pino fica fora do controle do firmware.
- Se usar relé, prefira **normalmente aberto**, com transistor e diodo de roda-livre. Com o relé aberto a corrente é zero de verdade, e a tara e o re-zero ficam corretos mesmo se a carga tiver vazamento.

## Fluxo do teste

```
IDLE ──TARE──► READY ──START──► REST ──► DISCHARGE ──cutoff/STOP──► RECOVERY ──► DONE
                 ▲                 └──────────── qualquer erro ────────────────► FAULT
                 └───────── TARE ou RESET (a partir de DONE/FAULT) ◄────────────────┘
```

1. **Tara** (carga desligada): média de 200 leituras do ACS758 define o zero (rejeitada se ficar a mais de 150 mV de 2,5 V).
2. **START → REST (3 s):** mede a tensão em repouso e **re-zera o ACS758** (desconta a deriva desde a tara, se a diferença for < 100 mA). Só segue se houver bateria e a tensão for maior que `cutoff + 200 mV`.
3. **DISCHARGE:** (relé fecha e) a carga liga. O firmware integra Ah e Wh e mede **R0** na primeira amostra com a carga ativa.
4. **Cutoff:** 5 amostras consecutivas abaixo do cutoff (**tensão já calibrada**) desligam a carga. `STOP` e o tempo máximo (4 h) também encerram.
5. **RECOVERY (60 s):** carga e relé abertos; mede **R0 no desligamento**, a tensão recuperada e o **zero final** do ACS758.
6. **DONE:** resumo com `q_Ah` e `q_corr_Ah` (corrigido pela deriva do zero), R0, zeros e metadados.

O cutoff é decidido **no firmware**: se o PC ou o link cair, a descarga termina normalmente.

### Proteções (carga e relé desligados, estado `FAULT`)

| Motivo | Condição |
|---|---|
| `OVERCURRENT` | corrente acima de 30 A por 3 amostras |
| `NO_CURRENT` | corrente abaixo de 100 mA, 1,5 s após ligar |
| `CURRENT_WITH_LOAD_OFF` | corrente acima de 300 mA com a carga desligada, no REST |
| `NO_BATTERY` | tensão em repouso abaixo de 5 V |
| `BATTERY_BELOW_START_LIMIT` | tensão em repouso ≤ cutoff + 200 mV |
| `ADC_FAIL` | 5 falhas consecutivas do ADS1115 |

Task watchdog com panic: se a task travar, o ESP32 reinicia e o pull externo desliga a carga.

## Novidades da v2

- **Calibração** de ganho e offset da tensão e ganho da corrente, gravada em NVS (comandos `CAL`, aba *Calibração*).
- **Re-zero** automático do ACS758 no REST e **Ah/Wh corrigidos pela deriva** (`q_corr_Ah`). Só vale para deriva: se o zero final passar de 100 mA o firmware assume corrente real e não corrige.
- **R0** no instante em que a carga liga e desliga.
- **Metadados** (versão, calibração, zeros) no resumo, nos eventos e no cabeçalho do CSV.
- **Bluetooth Classic (SPP):** aparece como porta COM; PIN de pareamento; um cliente por vez.
- **Número de sequência** nas linhas de dados (o app conta linhas perdidas) e comando `SYNC`: o app se **anexa a um teste em andamento** e reconecta sozinho depois de uma queda.
- **Log a 1 Hz na flash** (≈ 9 h): recupera os pontos perdidos na queda do link e sobrevive a reset do ESP32 (*Baixar log da flash*).
- **Relé opcional** em série com a carga.
- App com abas, exportação de CSV e de 3 imagens PNG, e eixos estáveis (não ficam "dançando").

## Firmware

```bash
. $IDF_PATH/export.sh            # ESP-IDF 5.4.2
cd battery_discharge_test
idf.py set-target esp32
idf.py menuconfig                # "Battery discharge test": cutoff, PIN do Bluetooth, relé...
idf.py build
idf.py -p COMx erase-flash       # só na primeira vez (a tabela de partições mudou)
idf.py -p COMx flash
```

- O `sdkconfig.defaults` só vale numa configuração nova: se existir um `sdkconfig` antigo, apague-o e rode `idf.py fullclean`. O build falha com um `#error` se o tick não for 1 ms.
- Exige **flash de 4 MB** (ESP32-WROOM-32D padrão). O `erase-flash` apaga também a calibração salva: anote os coeficientes antes.
- **Troque o PIN do Bluetooth** (`BATT_BT_PIN`, padrão `1234`).

Os parâmetros do `menuconfig` estão descritos no manual (seção 2.2).

## Protocolo serial (115200 8N1; o mesmo texto pelo Bluetooth)

Comandos: `TARE`, `START`, `STOP`, `RESET`, `CUTOFF <mV|V>`, `STATUS`, `SYNC`, `LOG`, `HELP`, `CAL SHOW|RESET|V|VP1|VP2|I|SET`. O `CUTOFF` e o `CAL` de tensão são bloqueados durante a descarga; `CAL I` só vale em `DISCHARGE`.

```
H;seq;t_boot_s;t_test_s;state;v_batt_V;i_A;p_W;q_Ah;e_Wh;ain0_V;ain1_V
D;4210;1050.315;1041.200;DISCHARGE;13.1964;4.4070;58.156;1.408971;20.66074;2.31516;2.68627
E;REZERO;tare_zero_V=2.51001,rest_zero_V=2.50989,delta_mA=-3.0,applied=1
E;R0;on_mohm=50.2,v_V=16.1301,i_A=5.3798
E;LOAD_OFF;CUTOFF;v_V=12.4932,i_A=4.1535,q_Ah=1.026181,e_Wh=14.72496,t_s=777.1
E;SUMMARY;end=CUTOFF,test_id=1,dur_s=777.1,q_Ah=1.026181,q_corr_Ah=1.028127,...
S;fw=2.0.0,state=READY,cutoff_mV=12000,...,v_gain_ppm=1000000,v_off_uV=0,i_gain_ppm=1000000,...
L;120;120.000;DISCHARGE;15.4012;3.9770;0.131200;2.0210      (log da flash, resposta ao LOG)
R;E;LOAD_ON;...                                                (evento reenviado pelo SYNC)
```

As colunas `ain0_V` e `ain1_V` são as tensões cruas do ADC: permitem refazer tensão e corrente depois com outra calibração.

## App Python

```bash
pip install -r tools/requirements.txt     # pyserial, matplotlib, numpy (Tkinter vem com o Python)
python tools/battery_monitor.py
```

- **Conexão:** cabo USB ou Bluetooth (porta COM), abertura sem travar a janela e **reconexão automática**.
- **Aba Teste:** Tara, START, STOP, Reset, cutoff; três gráficos selecionáveis (tensão × tempo, corrente × tempo, tensão × Ah) lado a lado; zoom/pan com *Home*; *Limpar gráficos* / *Mostrar tudo* (só a vista; os dados seguem no CSV e nas imagens); *Salvar CSV*, *Exportar imagens (3 PNG)* e *Baixar log da flash*. **Cada teste é salvo na sua própria pasta** `descarga_AAAAMMDD_HHMMSS/` (dentro da pasta do script): CSV com cabeçalho de metadados (`# chave=valor`; leia com `pd.read_csv(..., comment="#")`), `_resumo.txt` e as 3 imagens. Ao clicar em **STOP** tudo é salvo na hora, e o fim do teste regrava a versão completa. O STOP não abre uma nova sessão: o registro segue na recuperação até o DONE.
- **Aba Calibração:** coeficientes atuais, tensão em 1 e 2 pontos, corrente, aplicar valores manualmente e restaurar padrão.
- **Aba Análise:** abre vários CSVs (inclusive os do formato antigo), tabela de Ah e Wh até tensões fixas, curvas sobrepostas e curva **compensada por R0**.

## Precisão e limitações

- Resolução do ADS1115 (±6,144 V): LSB de 187,5 µV, ou ≈ 1,07 mV na tensão da bateria e ≈ 4,7 mA na corrente (antes da média). O ruído não limita a medida; **os erros sistemáticos limitam** (ganho do ACS758, tolerância do divisor, deriva do zero) — por isso a calibração contra instrumento.
- A tensão medida é a do terminal **no pad da GCM** e inclui a queda nos cabos e conectores.
- O cutoff olha o pack inteiro. Em packs sem BMS por grupo, meça os grupos no fim do teste.
- O re-zero e a correção por deriva assumem variação linear do offset: `q_corr_Ah` é uma estimativa.

## Validação realizada

- A lógica do firmware foi **compilada no PC e simulada** (bateria 4S, ADS1115, ACS758, carga, relé, NVS e partição de flash em RAM): descarga completa até o cutoff, `STOP`, `CUTOFF` e limites, calibração em 1 e 2 pontos e de corrente, re-zero, correção de Ah por deriva, caso de deriva grande (re-zero rejeitado e correção anulada), R0, log em flash (apagar antes de escrever, CRC), `LOG`, `SYNC`, comando por Bluetooth simulado, erros e `FAULT`.
- Os módulos foram conferidos (sintaxe e chamadas) contra os **cabeçalhos reais do ESP-IDF v5.4.2** para Bluetooth SPP/GAP, NVS e CRC.
- O app foi testado de forma headless contra a saída simulada, incluindo anexar a um teste em andamento com recuperação do log, aba Análise (com o CSV real do primeiro teste) e aba Calibração.
- **Não testado em hardware:** o rádio Bluetooth, o log em flash e o relé. O teste da v1 foi feito em bancada pelo autor, sem esses recursos. O manual (seção 8) traz os testes de aceitação para a primeira vez.
