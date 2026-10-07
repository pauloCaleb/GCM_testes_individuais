# Manual de uso e calibração — teste de descarga da bateria (GCM-PI2-2026.2)

Este manual cobre o firmware `battery_discharge_test` (v2) e o app `battery_monitor.py`. Para a
visão geral do projeto, veja o `README.md` da pasta acima.

> **Estado da validação.** A lógica do firmware foi simulada no PC (descarga completa, cutoff,
> calibração, re-zero, log em flash, SYNC) e os cabeçalhos do Bluetooth foram conferidos contra o
> ESP-IDF 5.4.2. **O rádio Bluetooth, o log em flash e o relé ainda não foram testados em
> hardware.** A seção 8 traz os testes de aceitação para fazer na primeira vez.

---

## 1. Visão geral

```
bateria (+) ──► PWR_PAD_BATT ──[ACS758]──► PWR_PAD_SENS ──► [relé opcional] ──► carga ──► bateria (−)
```

- A GCM é alimentada pela **fonte de bancada**, nunca pela bateria em teste. A ToFaB fica desconectada.
- O **GND da bateria** deve estar ligado ao GND da GCM (referência do divisor e do ACS758).
- O **retorno da corrente da carga** vai direto ao negativo da bateria, sem passar por trilhas da GCM.
- O enable da carga é o **GPIO23** (CN15 pino 8, 3,3 V). Use um **resistor de pull externo** no enable,
  para a carga ficar desligada durante o boot e depois de um reset.
- O firmware decide o **cutoff sozinho**, com a tensão já calibrada. Se o PC ou o link cair, o teste
  termina corretamente.

O firmware mede duas grandezas (ADS1115 @ 0x49):

| Canal | Sinal | Fórmula (já com calibração) |
|---|---|---|
| AIN0 | `PWR_VOLTAGE_SENS` (divisor 4,7 kΩ / 1 kΩ, razão 5,7) | `V = AIN0 × 5,7 × ganho_V + offset_V` |
| AIN1 | `PWR_CURRENT_SENS` (ACS758LCB-050B, 40 mV/A, zero ≈ 2,5 V) | `I = (AIN1 − zero) / 0,040 × ganho_I` |

Corrente positiva = fluxo de `PWR_PAD_BATT` para `PWR_PAD_SENS` = **descarga**.

---

## 2. Instalação

### 2.1 Firmware (ESP-IDF 5.4.2)

```bash
. $IDF_PATH/export.sh
cd battery_discharge_test
idf.py set-target esp32
idf.py menuconfig          # menu "Battery discharge test" (ver 2.2)
idf.py build
idf.py -p COMx erase-flash # só na PRIMEIRA vez: a tabela de partições mudou
idf.py -p COMx flash
```

- O `sdkconfig.defaults` ativa Bluetooth Classic, tick de 1 ms, tabela de partições própria
  (`partitions.csv`, flash de 4 MB) e a pilha de 8 KB da task principal. Ele só vale numa
  configuração nova: se já existir um `sdkconfig` antigo, apague-o e rode `idf.py fullclean`.
- O `erase-flash` apaga também a **calibração salva**. Anote os coeficientes antes (aba Calibração
  ou `E;CAL;SHOW`) e reaplique depois com *Aplicar valores digitados*.

### 2.2 Opções do `menuconfig` que você deve revisar

| Opção | Padrão | Observação |
|---|---|---|
| `BATT_CUTOFF_MV` | 12000 | cutoff inicial; mínimo/máximo aceitos em runtime: 10000–16000 |
| `BATT_MAX_CURRENT_MA` | 30000 | ajuste para logo acima da corrente da sua carga |
| `BATT_BT_PIN` | `1234` | **troque**: quem parear consegue enviar START |
| `BATT_BT_NAME` | `GCM-BATT` | nome que aparece no Windows |
| `BATT_RELAY_ENABLE` | desligado | habilite se houver relé NA em série; defina GPIO e polaridade |
| `BATT_REZERO_AT_REST` | ligado | re-zera o ACS758 durante o REST |
| `BATT_REZERO_MAX_MA` | 100 | deriva máxima aceita como "do sensor"; acima disso é tratada como corrente real |
| `BATT_RECOVERY_S` | 60 | aumente para 300–600 s para ver a tensão se aproximar do repouso |
| `BATT_LOG_INTERVAL_MS` | 1000 | intervalo do log em flash (1 Hz ≈ 9 h de teste na partição) |

### 2.3 App (Windows)

```bash
pip install -r tools/requirements.txt     # pyserial, matplotlib, numpy
python tools/battery_monitor.py
```

Os arquivos de saída vão para a **mesma pasta do script**, a não ser que você escolha outra em *Pasta...*;
cada teste ganha a sua própria subpasta (ver 4.3).

---

## 3. Conexão

O app fala com a **porta COM**; o cabo USB (CH340) e o Bluetooth aparecem do mesmo jeito.

### 3.1 Cabo USB

1. Escolha a porta, clique *Conectar*.
2. Em alguns adaptadores, abrir/fechar a porta reinicia o ESP32 (DTR/RTS). O app abre com DTR/RTS
   desativados, mas confirme isso uma vez, **sem a carga ligada**. Conecte antes do START e evite
   reconectar o cabo durante a descarga.

### 3.2 Bluetooth

1. No Windows: *Configurações → Bluetooth e dispositivos → Adicionar dispositivo → Bluetooth*,
   escolha `GCM-BATT` e digite o **PIN** (padrão `1234`).
2. Abra *Configurações de Bluetooth → Mais opções de Bluetooth → Portas COM* e anote a porta
   **"Saída"** (outgoing) de `GCM-BATT`. Ela aparece na lista do app depois de *Atualizar*.
3. *Conectar*. Abrir uma porta Bluetooth pode levar vários segundos; o rótulo mostra "conectando...".
4. Só **um cliente** conecta por vez. Um segundo é desconectado pelo firmware.
5. O Bluetooth é opcional: sem ele, nada muda no restante.

### 3.3 O que acontece ao conectar

O app envia `SYNC`. O firmware responde com o status, os eventos recentes e o cabeçalho. Se o
firmware já estiver em teste, o app **se anexa**: ele baixa o log da flash (1 Hz) para recuperar o
que perdeu, junta com os dados ao vivo (10 Hz) e segue gravando. Isso vale também para a **reconexão
automática** depois de uma queda de link (opção *Reconectar automaticamente*).

---

## 4. Rotina de teste

### 4.1 Antes de começar

- [ ] GCM ligada na fonte de bancada **há pelo menos 10 min** (o zero do ACS758 precisa estabilizar).
- [ ] Bateria no `PWR_PAD_BATT`, carga no `PWR_PAD_SENS`, GND comum, retorno direto ao negativo da bateria.
- [ ] Carga **desabilitada** (e relé aberto, se houver). Nenhuma corrente passando pelo ACS758.
- [ ] Bateria carregada e em repouso; local seguro (caixa/saco antichama) e temperatura monitorada.
- [ ] Cutoff conferido na tela (padrão 12,00 V = 3,0 V/célula em 4S; confirme o mínimo da sua célula).
- [ ] Para pack sem BMS por grupo: multímetro à mão para medir os 4 grupos no fim.

### 4.2 Passo a passo

1. **Conectar** (USB ou Bluetooth).
2. **Tara**: mede o zero do ACS758 com a carga desligada (200 amostras, ~1 s). O app avisa se a GCM foi
   ligada há menos de 10 min. A tara é rejeitada se o zero estiver a mais de 150 mV de 2,5 V.
3. **START**: o app pede confirmação. O firmware faz:
   - **REST (3 s):** carga desligada; mede a tensão de repouso e o zero (**re-zero**: o zero passa a ser
     o medido agora, se a diferença for < `BATT_REZERO_MAX_MA`);
   - confere bateria presente e acima do cutoff + 200 mV;
   - fecha o relé (se houver), habilita a carga e entra em **DISCHARGE**.
4. **Acompanhar:** os três gráficos têm checkboxes; zoom/pan para de reajustar os eixos (botão *Home*
   retoma). O cutoff aparece como linha vermelha. *Limpar gráficos* esvazia só a vista (por exemplo, para descartar o trecho do REST ou de uma calibração): o CSV, o resumo, as imagens exportadas e a aba *Análise* continuam com todos os pontos, e *Mostrar tudo* traz a vista de volta. Iniciar um novo teste zera a vista automaticamente.
5. **Fim:** 5 amostras consecutivas abaixo do cutoff desligam a carga (`CUTOFF`). Também encerram:
   `STOP` e o tempo máximo (4 h). Segue **RECOVERY** (padrão 60 s, carga e relé abertos) e **DONE**.
6. **Depois do DONE: desconecte a bateria do circuito.** Meça a tensão dos 4 grupos.
7. **Reset** volta ao estado pronto (a tara é mantida). Faça nova tara se mudou a bancada.

### 4.3 Onde ficam os arquivos: uma pasta por teste

Cada teste cria a sua própria pasta, nomeada pelo horário em que começou, dentro da pasta de saída
(por padrão, a do script). Tudo do teste fica ali dentro:

```
tools/
└── descarga_20261004_031239/
    ├── descarga_20261004_031239.csv
    ├── descarga_20261004_031239_resumo.txt
    ├── descarga_20261004_031239_tensao_tempo.png
    ├── descarga_20261004_031239_corrente_tempo.png
    └── descarga_20261004_031239_tensao_capacidade.png
```

| Arquivo | Conteúdo |
|---|---|
| `<pasta>.csv` | linhas de dados a 10 Hz; começa com um cabeçalho de **metadados** (`# chave=valor`) |
| `<pasta>_resumo.txt` | resumo, metadados, qualidade dos dados (linhas perdidas) e eventos |
| `<pasta>_*.png` | os três gráficos, gerados automaticamente |

**Quando é salvo.** O CSV é gravado durante o teste. O resumo e as imagens são gravados:

1. **ao clicar em STOP** (na hora, com o que já existe; o app pede confirmação);
2. **no fim do teste** (cutoff, FAULT, STOP na recuperação, desconexão ou fechamento do app), já
   completos: sobrescrevem os arquivos do passo anterior.

O STOP **não inicia uma nova sessão**: ele desliga a carga e o teste passa à recuperação
(padrão 60 s), registrando a tensão que volta a subir, tudo na mesma sessão e na mesma pasta. Um segundo
STOP encerra a recuperação na hora. Uma nova sessão só começa no próximo START (ou ao se anexar a um teste
em andamento). Se o teste for abortado ainda no REST, a pasta tem só o CSV e o resumo (sem imagens).

Com *Auto-salvar* desligado nada é gravado automaticamente (use *Salvar CSV* e *Exportar imagens*). O botão
*Abrir pasta do teste* abre a pasta no Explorer. O log baixado da flash vai para uma pasta com o sufixo
`_logflash`. Se dois testes começarem no mesmo segundo, o segundo ganha o sufixo `_2`.

Ler o CSV no pandas: `pd.read_csv(caminho, comment="#")`. Desligue *Metadados no CSV* se alguma
ferramenta não aceitar linhas `#`.

Colunas: `host_time, seq, t_boot_s, t_test_s, state, v_batt_V, i_A, p_W, q_Ah, e_Wh, ain0_V, ain1_V`.
`ain0_V` e `ain1_V` são as tensões **cruas** do ADC: com elas dá para refazer tensão e corrente
offline com outra calibração.

---

## 5. Entendendo o resultado

| Campo | Significado |
|---|---|
| `q_Ah`, `e_Wh` | capacidade e energia medidas (integração trapezoidal a cada amostra) |
| `q_corr_Ah`, `e_corr_Wh` | as mesmas, corrigidas pela **deriva do zero** (ver abaixo) |
| `i_zero_end_mA` | corrente lida com a carga desligada no fim (idealmente ~0) |
| `drift_mA` | erro médio de offset estimado durante o teste |
| `r0_on_mohm` | queda de tensão na 1ª amostra com a carga ligada ÷ corrente (parte ôhmica; **inclui cabos e conectores**) |
| `r0_off_mohm` | salto de tensão ao desligar ÷ corrente |
| `rint_mohm` | queda numa janela de ~1,5 s (já inclui polarização; serve só para comparar) |
| `v_rec_V` | tensão média dos últimos 5 s de RECOVERY; ainda sobe depois disso |

**Correção pela deriva.** Se o zero do ACS758 deriva, a corrente fica com um erro de offset que se acumula
no Ah. O firmware mede o offset no início (0 após o re-zero) e no fim (carga desligada) e, supondo
variação linear, subtrai `média × duração`. Exemplo do primeiro teste: offset final −21,6 mA →
correção ≈ +0,016 Ah (0,27 %). **Se o zero final passar de `BATT_REZERO_MAX_MA` (100 mA)**, o firmware
assume que há corrente real (vazamento da carga) e **não corrige**: `q_corr_Ah = nan` e aparece
`DRIFT_TOO_LARGE`. A correção só trata deriva de zero, **não** erro de ganho — para isso é a calibração.

**Linhas perdidas.** Cada linha tem um número de sequência. O app conta as lacunas (aparecem no resumo).
Via Bluetooth pode haver perdas; o `q_Ah` do firmware não é afetado, só a resolução do gráfico/CSV
naquele trecho. Pontos a 1 Hz recuperados da flash são marcados no resumo.

---

## 6. Calibração

### 6.1 Quando calibrar

- Na primeira montagem e sempre que trocar o ACS758, o ADS1115 ou os resistores do divisor (R8/R15).
- Pelo menos uma vez por semestre ou quando o multímetro divergir do app além do aceitável.
- **A calibração de tensão muda o ponto de cutoff**: confira o cutoff depois (6.5).

Os coeficientes ficam no **ESP32** (NVS) e valem para qualquer computador; sobrevivem a reinício e a
`idf.py flash` (mas não a `erase-flash`).

### 6.2 Instrumentos

- **Tensão:** multímetro de bancada (ou 4½ dígitos, se houver). Meça **nos próprios pads**, com as
  pontas no mesmo ponto que o divisor enxerga, para não incluir queda nos cabos.
- **Corrente:** alicate amperímetro **DC** (Hall) ou multímetro em série na escala de 10 A ou 20 A.
  Confirme a classe de exatidão do instrumento; ele limita a sua calibração.
- Fonte de bancada com limitação de corrente (para o método de 2 pontos).

A exatidão final nunca é melhor que a do instrumento de referência.

### 6.3 Tensão — 1 ponto (rápido)

Corrige o **ganho** mantendo o offset atual. Serve quando o erro é proporcional (por exemplo, resistores
do divisor com tolerância).

1. Estado `IDLE`, `READY`, `DONE` ou `FAULT` (carga desligada).
2. Aplique uma tensão estável no pad (bateria, ou fonte de bancada entre `PWR_PAD_BATT` e GND).
3. Meça no pad e digite o valor em **Referência (V)** na aba *Calibração*.
4. *Calibrar ganho*. O firmware mede 40 conversões do ADC, calcula
   `ganho = (V_ref − offset) / V_bruta` e grava.
5. Confira em **Leituras ao vivo**: a *Tensão calibrada* deve bater com o multímetro.

### 6.4 Tensão — 2 pontos (ganho e offset)

Use quando o erro **varia com a tensão** (1 ponto acerta em 16 V e erra em 12 V). Como o cutoff fica em
~12 V, é o método recomendado.

1. Fonte de bancada no pad (`PWR_PAD_BATT` e GND), carga desligada, limite de corrente baixo (≈ 0,1 A).
2. **Ponto 1:** ajuste para ~12 V; meça no pad; digite em *Ponto 1 — referência*; *Capturar ponto 1*.
3. **Ponto 2:** ajuste para ~16 V; meça; digite em *Ponto 2 — referência*; *Capturar ponto 2 e calcular*.
4. O firmware exige **pelo menos 1,5 V** de diferença entre os pontos (tanto na referência quanto na leitura
   bruta) e calcula:

   ```
   ganho  = (ref2 − ref1) / (bruta2 − bruta1)
   offset = ref1 − ganho × bruta1
   ```

5. Verifique num terceiro ponto (ex.: 14 V). O erro deve ficar em poucos mV.

### 6.5 Conferência do cutoff

Depois de calibrar a tensão, ajuste a fonte para **exatamente o cutoff** (ex.: 12,000 V no multímetro) e
veja a *Tensão calibrada*. Ela deve ler 12,00 V. Se o erro for relevante, repita 6.4.

### 6.6 Corrente (ganho)

A corrente só existe com a carga ligada, então a calibração é feita **durante a descarga**:

1. Inicie um teste normal (*START*) com uma bateria em bom estado e espere o estado `DISCHARGE`
   estabilizar (~1 min; a corrente sobe nos primeiros minutos).
2. Meça a corrente real com o alicate DC ou o multímetro em série. Anote o valor **no mesmo instante**.
3. Na aba *Calibração*, digite a corrente de referência (A) e clique *Calibrar corrente*. O firmware faz a
   média de 3 s (30 amostras) da corrente crua e ajusta `ganho_I = ref / medida`. Exige ≥ 0,3 A.
4. Confira: a *Corrente calibrada* deve bater com o instrumento. Depois **interrompa com STOP**. Esse teste
   não serve como medida de capacidade (o ganho mudou no meio).

> A calibração da corrente é de **ganho**. O offset (zero) é tratado pela tara e pelo re-zero do REST.
> Se puder, repita em duas correntes diferentes (se a carga for ajustável) para checar a linearidade.

### 6.7 Aplicar ou restaurar manualmente

Em *Coeficientes gravados no firmware*: *Atualizar* lê os valores, *Copiar atuais* preenche os campos,
*Aplicar valores digitados* grava (`CAL SET`) e *Restaurar padrão* volta a ganhos 1,000000 e offset 0.
Faixas aceitas: ganhos de 0,8 a 1,2; offset de −500 a +500 mV. Fora delas o firmware recusa.

**Guarde os coeficientes** (ganhos em ppm, offset em µV) junto com a data e o instrumento usado.

### 6.8 Tara e zero

- A **tara** mede o zero do ACS758 sem corrente. Faça com a GCM aquecida (≥ 10 min), carga desligada e
  relé aberto. O valor típico fica próximo de 2,5 V.
- O **re-zero** do REST repete a medição com a carga desligada no começo de cada teste, descontando a deriva
  desde a tara. A diferença aparece no evento `REZERO` (`delta_mA`). Se passar de `BATT_REZERO_MAX_MA`,
  o firmware **não** aplica (`REZERO_REJECTED`): investigue vazamento da carga ou relé colado.
- Se a carga tiver vazamento com o enable inativo, a tara embute essa corrente no zero e o Ah fica subestimado.
  O relé em série resolve; sem relé, tare com a **bateria desconectada**.

---

## 7. Problemas comuns

| Evento / sintoma | Causa provável | O que fazer |
|---|---|---|
| `FAULT: CURRENT_WITH_LOAD_OFF` | carga conduzindo com o enable inativo (> 300 mA) ou relé colado | verificar pull do enable, polaridade (`BATT_LOAD_ACTIVE_HIGH`), relé |
| `FAULT: NO_CURRENT` | carga desconectada, enable errado, relé aberto, fiação aberta | conferir enable/relé; `BATT_MIN_CURRENT_MA` |
| `FAULT: OVERCURRENT` | corrente acima de `BATT_MAX_CURRENT_MA` por 3 amostras | revisar a carga e o limite |
| `FAULT: NO_BATTERY` | tensão em repouso < 5 V | bateria desconectada ou GND aberto |
| `FAULT: BATTERY_BELOW_START_LIMIT` | tensão ≤ cutoff + 200 mV | recarregar a bateria ou baixar o cutoff |
| `FAULT: ADC_FAIL` | 5 falhas seguidas do ADS1115 | I2C, endereço 0x49, alimentação |
| `ERR: TARE_ZERO_OUT_OF_RANGE` | zero a mais de 150 mV de 2,5 V | sensor ausente/defeituoso ou corrente passando |
| `WARN: REZERO_REJECTED` / `DRIFT_TOO_LARGE` | diferença de zero acima de 100 mA | vazamento real; não confie em `q_corr_Ah` |
| `WARN: PERIOD_OVERRUN` | amostra mais lenta que 100 ms | reduzir `BATT_OVERSAMPLE` ou aumentar o período |
| `WARN: LOG_WRITE` | falha ao gravar na flash | o teste segue; o log é desativado |
| `ERR: CAL_*` | calibração recusada | ler o texto do erro no *Registro da calibração* |
| `ESP32 reiniciou` | brownout, watchdog ou reset por DTR/RTS | a carga é desligada; baixar o log da flash |
| Bluetooth não conecta | pareamento antigo, PIN, 2º cliente | remover `GCM-BATT` do Windows e parear de novo; usar a porta "Saída" |
| Janela do app sem dados | porta errada | testar `STATUS` com um terminal serial a 115200 |

Reinício durante o teste: o log a 1 Hz permanece na flash. Conecte, clique *Baixar log da flash* e
use *Salvar CSV*, *Exportar imagens* e a aba *Análise*.

---

## 8. Testes de aceitação do Bluetooth (fazer na primeira vez)

O Bluetooth e o log em flash foram escritos contra a API real do ESP-IDF 5.4.2, mas **não testados em
hardware**. Antes de confiar no link, faça (carga pequena, fonte no lugar da bateria):

1. **Ruído com e sem Bluetooth.** Rode 10 min em regime por USB e 10 min por Bluetooth (sem cabo). No CSV,
   compare o desvio padrão da corrente e da tensão em regime (`pd.read_csv(..., comment="#")`). Esperado
   (primeiro teste a 4 A, só USB): corrente ≈ 4 mA, tensão ≈ 0,4 mV. Não pode haver `PERIOD_OVERRUN`
   novo nem aumento relevante de ruído.
2. **Perda de link.** No meio de um teste, afaste-se ou desligue o Bluetooth do PC por 1–2 min e volte. O
   app deve reconectar sozinho, mostrar "pontos recuperados do log da flash" e manter o tempo contínuo no
   gráfico.
3. **Anexar.** Feche o app durante um teste e abra de novo: ele deve se anexar e recuperar o histórico.
4. **Reinício.** Com a carga pequena, force um reset (botão EN) durante um teste: a carga deve desligar,
   o app deve avisar, e *Baixar log da flash* deve trazer os pontos gravados.
5. **Fim a fim.** Compare `q_Ah` do resumo (firmware) com a integral do CSV: devem diferir menos que
   algumas partes por mil.

Critério de reprovação: ruído claramente maior com Bluetooth, perdas frequentes de linhas, ou qualquer
caso em que a carga permaneça ligada depois do cutoff. Se acontecer, desabilite `BATT_BT_ENABLE` e
use o cabo, e me avise para investigar.

---

## 9. Referência rápida de comandos

Todos terminam em `\n` e não diferenciam maiúsculas. Podem ser digitados num terminal serial (115200 8N1)
ou pelo Bluetooth.

| Comando | Efeito |
|---|---|
| `TARE` | mede o zero do ACS758 (carga desligada) |
| `START` / `STOP` / `RESET` | inicia, interrompe, volta ao estado pronto |
| `CUTOFF <valor>` | mV, ou V se < 100; bloqueado em REST e DISCHARGE; faixa 10–16 V |
| `STATUS` | estado, parâmetros, calibração, zeros, Bluetooth, log |
| `SYNC` | status + eventos recentes (o app envia ao conectar) |
| `LOG` | baixa o log da flash do último teste |
| `CAL SHOW` / `CAL RESET` | mostra / restaura os coeficientes |
| `CAL V <ref>` | calibra o ganho da tensão (1 ponto) |
| `CAL VP1 <ref>` e `CAL VP2 <ref>` | tensão em 2 pontos (ganho e offset) |
| `CAL I <ref>` | ganho da corrente (somente em DISCHARGE) |
| `CAL SET <ganhoV_ppm> <offsetV_uV> <ganhoI_ppm>` | grava coeficientes diretamente |

`<ref>` em mV/mA, ou em V/A se for menor que 100.

Linhas do firmware: `D;` dado (com número de sequência), `E;` evento, `S;` status, `H;` cabeçalho,
`L;` registro do log, `R;` evento reenviado pelo `SYNC`. Qualquer outra linha é ignorada pelo app.

---

## 10. Limitações conhecidas

- A tensão medida é a do terminal sob carga **no pad da GCM**: inclui a queda nos cabos e conectores.
- O cutoff olha o pack inteiro. Em pack 4S2P usado, um grupo fraco pode estar abaixo do limite quando o total
  chega ao cutoff: meça os 4 grupos no fim e suba o cutoff se algum estiver abaixo de ~2,8 V.
- Tensão e corrente são amostradas de forma intercalada (não simultânea); com carga estável o efeito é desprezível.
- O re-zero e a correção por deriva assumem **variação linear** do offset. Deriva térmica não é linear:
  `q_corr_Ah` é uma estimativa, não uma garantia.
- O log em flash guarda 1 Hz (≈ 9 h na partição). Testes mais longos que isso param de ser registrados na flash
  (o teste em si continua).
- Um erro de **ganho** do ACS758 não é visível nos dados; só a calibração contra um instrumento corrige.
