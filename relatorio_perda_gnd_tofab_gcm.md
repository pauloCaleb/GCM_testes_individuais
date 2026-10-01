# Falha por perda de GND no chicote GCM ↔ ToFaB-i2c

**Projeto:** Robô sumô — PI2 (UnB – FCTE)
**Placas envolvidas:** GCM (GeneralControlModule-PI2-2026.2) e ToFaB-i2c
**Autor:** Paulo Caleb
**Data:** 30/09/2026
**Versão:** rascunho 0.1

> Convenção: trechos em `[PREENCHER]` dependem de medição ou decisão. Itens marcados como **hipótese** devem ser confirmados no ensaio da seção 7 antes de serem afirmados como fato.

---

## 1. Resumo

Durante testes de bancada, a desconexão do GND da ToFaB-i2c (mau contato de chicote) causou a queima irreversível do ESP32 da GCM. O evento ocorreu duas vezes (uma acidental, outra durante a investigação). As demais proteções da GCM, inclusive o isolamento USB (ADuM3160), funcionaram: o computador conectado e os outros circuitos sobreviveram, e apenas o ESP32 foi danificado.

Este documento descreve a arquitetura, o mecanismo provável da falha, avalia três alternativas de correção (alimentação em 5V, level shifter MOSFET e isolador I2C) e propõe um ensaio para validar a escolha.

**Conclusão preliminar:** [PREENCHER após o ensaio]

---

## 2. Arquitetura do sistema

### 2.1 GCM

- Única central eletrônica do robô.
- MCU: ESP32-WROOM-32D.
- Alimentação pela bateria em `V_SUPPLY` (CN15, pinos 24/25), com proteção contra polaridade reversa (Q1 + D1) e varistor (U4).
- Buck LM2596 gera o +5V e LDO LM1117-3,3 gera o +3,3V.
- Barramento I2C1 (`SCL1`/`SDA1`, CN15 pinos 3/16): ligado **direto aos GPIOs do ESP32**, sem level shifter e sem resistor série, com pull-ups R26/R27 (3,3 kΩ) no +3,3V da GCM.
- Um level shifter separado (Q2/Q3, BSN20) atende apenas o ADS1115 (lado 5V).

### 2.2 ToFaB-i2c (Time of Flight Auxiliary Board)

- PCA9554: expansor de I/O que controla os pinos XSHUT dos 3 ToF VL53L1X e a base de 3 BJTs que acionam a fita de LED.
- Alimentada diretamente pela bateria, com dois reguladores internos:
  - 12 V para a fita de LED;
  - 3,3 V próprio para os ToF e o PCA9554.
- Os domínios de 3,3 V da GCM e da ToFaB **não** são interligados.

### 2.3 Interligação

- A ToFaB e a GCM são alimentadas em paralelo pela mesma bateria (GND comum pelo chicote).
- O I2C1 é o único elo de sinal. O ponto de contato entre os dois domínios de 3,3 V é o próprio barramento: o pull-up da GCM encontra os drivers do PCA9554 e dos ToF, alimentados pela ToFaB.

```
Bateria (~16 V) ─┬─► GCM (V_SUPPLY) ─► LM2596 ─► 5V ─► LM1117 ─► 3V3 ─► ESP32
                 │                                              │
                 │                                    R26/R27 (3,3k pull-up)
                 │                                              │
                 │                      SDA/SCL (CN15 3/16) ◄───┴──► ESP32 GPIO (direto)
                 │                              │
                 └─► ToFaB-i2c ─► Reg 12V (LED) │
                         │                      │
                         └─► Reg 3V3 ─► PCA9554 + 3× VL53L1X ◄── SDA/SCL
                         GND (chicote) ◄──────► GND (GCM)
```

*[Substituir por diagrama no relatório final.]*

---

## 3. Descrição do problema

### 3.1 Sintoma

Com o GND da ToFaB desconectado e a bateria ligada, o ESP32 da GCM queima. Falha irreversível, exigindo troca do módulo. Reproduzido duas vezes.

### 3.2 Mecanismo provável (**hipótese**)

1. Sem o GND, o GND local da ToFaB flutua. Reguladores, BJTs e fita de LED continuam conectados ao positivo da bateria, e as cargas puxam o GND local para perto de +bateria (~16 V, pack 4S).
2. Todos os pinos da ToFaB referenciados a esse GND local, entre eles SDA e SCL do PCA9554 e dos ToF, passam a "enxergar" ~16 V acima do GND da GCM.
3. Os diodos de proteção (ESD) desses pinos ficam polarizados diretamente, e o potencial elevado aparece nas linhas SDA/SCL do chicote.
4. SDA/SCL estão ligados direto aos GPIOs do ESP32. A tensão (~16 V) excede muito o máximo absoluto do pino (ver seção 9) e/ou a corrente injetada excede o que os diodos internos suportam. O GPIO é destruído.
5. O restante da GCM sobrevive porque o ponto exposto ao outro domínio de GND é só o I2C.

> **Importante:** os itens 2 a 4 são a reconstrução lógica do evento, não foram medidos. O ensaio da seção 7 deve registrar a tensão e a corrente reais em SDA/SCL.

### 3.3 Natureza da falha

É uma falha de **perda de referência**, não de sobretensão na alimentação. Qualquer mau contato no GND do chicote pode causá-la, e o dano é irreversível.

### 3.4 Evidências disponíveis

| Evidência | Situação |
|---|---|
| ESP32 queimado em 2 ocasiões | Observado |
| Demais circuitos e PC preservados | Observado |
| Tensão em SDA/SCL com GND aberto | aprox 16V |
| Corrente injetada nos pinos | [PREENCHER — medir] |
| Tensão no rail de 3,3 V durante o evento | Não será testado para preservar o ESP32 |

---

## 4. Requisitos da solução

- **R1.** Uma perda de GND na ToFaB não pode danificar a GCM (nem o ESP32).
- **R2.** O I2C deve operar a 400 kHz (velocidade máxima do VL53L1X), com tempos de subida dentro da especificação I2C.
- **R3.** Custo e complexidade compatíveis com PI2; componentes disponíveis ou importáveis em prazo viável.
- **R4.** Preferência por solução que atue na causa (caminho elétrico) e não só reduza a probabilidade do evento.

---

## 5. Alternativas avaliadas

### 5.1 Alternativa A — Alimentar a ToFaB com 5V da GCM (boost interno para 12 V)

**Ideia:** a ToFaB passa a ser alimentada pelo +5V da GCM. Um boost converte 5 V em 12 V para a fita LED. Com GND perdido, o potencial das linhas seria ~5 V em vez de ~16 V.

**Análise:**

- **Premissa descartada:** o ESP32 não é 5V tolerant (limite próximo a VDD + 0,3 V). A própria GCM usa um divisor (R28/R29) no RXD do TJA1050 para não levar 5 V ao GPIO34.
- Mesmo com um MCU 5V tolerant (por exemplo, pinos FT de alguns STM32), a tolerância cobre só a **tensão** no pino. A **corrente de retorno** da carga da ToFaB continuaria passando pelo I2C.
- O rail de 3,3 V pode ser elevado pela corrente injetada, pois o LM1117 não drena corrente.

**Veredito:** mitigação parcial. Reduz o nível de tensão, mas não elimina o modo de falha.

### 5.2 Alternativa B — Level shifter MOSFET (BSN20) na fronteira do chicote

**Ideia:** reaproveitar o circuito já usado na GCM (N-MOSFET bidirecional): gate em +3,3 V, source no lado do ESP32, dreno no lado da ToFaB, com pull-ups em cada lado.

**Comportamento esperado:**

- > Linha em nível alto, GND da ToFaB perdido
  -  Source preso em ~3,3 V pelo pull-up da GCM -> Vgs ≈ 0 → canal desligado; diodo de corpo reverso. Os ~16 V ficam sobre o dreno. Tensão no ESP32 preservada.
- > ESP32 (ou escravo) puxando a linha para baixo
  -  Source vai a 0 V → Vgs ≈ 3,3 V → canal liga e conecta o ESP32 ao lado B, que está a ~16 V. O pino em si fica em ~0 V, mas passa a **sinkar a corrente** que a ToFaB consegue injetar.

**Pontos a verificar:**

- Vds máx. do BSN20 (50 V) e Vgs máx. (±20 V). O gate (3,3 V) vê ~-12,7 V em relação ao dreno com o lado B a ~16 V. Dentro das capacidades do MOSFET
- Lado B precisa de pull-ups próprios para o 3,3 V da ToFaB (R26/R27 permanecem só no lado A).
- Tempo de subida a 400 kHz com pull-ups e capacitância do chicote.
- Glitch capacitivo na desconexão (queda/subida rápida do lado B). Resistor série (~100 Ω) no lado do ESP32 ajuda contra o pulso e contra o caso de sinking.
- Corrente máxima injetada com a linha em nível baixo. Possível problema para a saúde do pino do ESP32.

**Veredito:** melhoria significativa e barata. Elimina o caso estático de sobretensão, mas **não garante** a eliminação do caminho de corrente quando o ESP32 puxa a linha para baixo.

### 5.3 Alternativa C — Isolador I2C (ADuM1250, ISO1540 ou equivalente)

**Ideia:** isolar galvanicamente os dois lados do barramento. Cada lado é alimentado pelo seu próprio domínio (VDD1 da GCM, VDD2 da ToFaB). Sem caminho elétrico entre SDA/SCL dos dois lados, a perda de GND da ToFaB deixa de se refletir no ESP32.

**Pontos a verificar:**

- Tensão de isolamento especificada muito acima dos ~16 V esperados.
- Velocidade suportada (400 kHz para ADUm1250)
- Alimentação dos dois lados: VDD1 do +3,3 V da GCM, VDD2 do 3,3 V da ToFaB.
- Pull-ups em ambos os lados.
- Disponibilidade e custo (importação demorada, aproximadamente 14 dias para o componente chegar)
- Observação funcional: com o GND perdido, a ToFaB deixa de funcionar (sensores e LED), mas a falha passa de **destrutiva** para **funcional**. O firmware deve detectar a perda de comunicação e tratar o erro.

**Veredito:** única alternativa que remove o caminho elétrico por construção.


## 6. Plano de ensaio

**Objetivo:** medir tensão e corrente em SDA/SCL, e o efeito no rail de 3,3 V, ao perder o GND da ToFaB em cada configuração.

**Precauções:**

- Não conectar o chicote elétrico energizado na GCM.
- Alimentar com fonte de bancada com limitação de corrente (ordem de 50 mA no primeiro ensaio) no lugar da bateria.
- Medir com osciloscópio (captura de transiente na desconexão) e multímetro.


## 7. Resumo das alternativas levantadas

**Configurações:**

| ID | Configuração |
|---|---|
| E0 | Referência: arquitetura atual (I2C direto, ToFaB a ~16 V) |
| E1 | Alternativa A (ToFaB em 5 V) com reforço de GND|
| E2 | Alternativa B (level shifter MOSFET) |
| E3 | Alternativa C (isolador I2C) |


## 8. Proposta final

Mesclagem das alternativas E1 e E3, utilizando os 5V para alimentar a ToFaB (exigindo o design de um circuito boost de baixa corrente para alimentar as fitas de LED em 12V) somado a utilização de um isolador i2c para fazer o desacoplamento do chicote elétrico, além de reforçar as conexões de GND da ToFaB.

O isolador i2c escolhido para a implementação é o ADUm1250. O item já foi comprado e possui expectativa de entrega em 28/10/2026.
