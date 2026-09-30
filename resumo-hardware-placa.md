# Resumo de Hardware — Placa ESP32-WROOM-32D (motor / shift-register / CAN / USB isolado)

> Revisão completa: MCU, programação, isolamento USB, CAN, conector principal, estágio de alimentação, ADC/level-shifter I2C e sensores de corrente/tensão dos PADs externos.

## 1. MCU — ESP32-WROOM-32D

**Strapping pins** (todos com resistor de 10kΩ dedicado, nada depende só do pull interno):

| Pino | Resistor | Estado no boot |
|---|---|---|
| GPIO0 | pull-up 10kΩ, integrado ao circuito de auto-download | HIGH |
| GPIO2 | pull-down 10kΩ (R39) | LOW |
| GPIO5 | pull-up 10kΩ (R25) | HIGH |
| GPIO12 (MTDI, seleciona tensão da flash) | pull-down 10kΩ (R38) | LOW — o mais crítico, flash é 3,3V |
| GPIO15 (MTDO, boot log) | pull-up 10kΩ (R24) + jumper 0Ω não populado | HIGH (LOW opcional via jumper, silencia o boot log) |
| EN | pull-up 10kΩ (R20) + capacitor 100nF pra GND (C21) | HIGH |

**Pinos reservados/intocados:**
- SD2, SD3, CMD, SD0, SD1, CLK (pinos 17–22, flash SPI interna) — marcados *No Connect*
- NC (pino 32) — sem conexão

**Entradas-somente sem pull interno (GPIO34-39):**
- SENSOR_VP / SENSOR_VN — aterrados diretamente (não usados)
- GPIO34 / GPIO35 — usados como entradas normais com pull-up externo (R22/R23, 10kΩ), expostos no conector principal (CN15). Qualquer coisa ligada ali precisa ser fonte de sinal — nunca tentar dirigir esses pinos a partir do conector.

## 2. Programação — CH340C + circuito de auto-download

- CH340C alimentado em 5V, com C14/C20 de decoupling.
- Auto-download com 2 transistores BC547-B (Q4 → EN, Q5 → GPIO0), topologia com **emissores cruzados**: emissor de Q4 no RTS, emissor de Q5 no DTR (em vez do clássico "emissor no GND + capacitor de delay"). Garante a sequência correta (EN sobe exatamente quando GPIO0 está LOW) via lógica diferencial; nenhum dos dois transistores liga se RTS/DTR estiverem no mesmo nível — sem estado ambíguo.
- R17 (10kΩ): pull-up de GPIO0 pra +3,3V.

## 3. Isolador USB — ADuM3160 (U10)

- **Lado 1** (host/PC, via conector USB-Micro U9): VBUS1 ← 5V do cabo. PDEN + SPU amarrados em VDD1 (modo padrão + *full speed*). Pino GND1 do Adum3160 num domínio isolado próprio e rotulado como "GND2" no esquema, separado do GND geral da placa.
- **Lado 2** (isolado, vai pro CH340 via ESP_D-/ESP_D+): VBUS2 ← +5V da placa. Pino GND2 do Adum3160 = GND geral da placa. VDD2, SPD e PIN corretamente amarrados entre si.

## 4. Transceiver CAN — TJA1050 (U13)

- VCC = 5V, GND, pino S → GND (modo *high-speed*, recomendado), VREF NC (ok pra RXD digital).
- CANH/CANL → conector H1, terminação 120Ω (R21), ativo quando e se o módulo for terminação da rede CAN, em outros casos o conector fica aberto.
- **RXD com divisor de nível** (R28 10kΩ série + R29 22kΩ pull-down): reduz o RXD de 5V pra ~3,44V antes de chegar no GPIO34 (entrada-somente, sem proteção ESD pra VDD) — proteção necessária e bem dimensionada.
- TXD direto, sem divisor (é entrada do TJA1050, aceita 3,3V).
- **Modo alternativo GPIO-direto**: R40–R43 (0Ω) permitem, quando o TJA1050 **não é populado**, usar os pinos físicos de CANH/CANL como GPIO33/GPIO32 puros, acessados direto pelo CN15 — mutuamente exclusivo com o modo transceiver, sem risco de conflito.

## 5. Conector principal — CN15 (27 pinos)

| Pinos | Função |
|---|---|
| 1, 2, 14, 15, 26, 27 | GND (6 pinos) |
| 22, 10 | +3,3V — saída (2 pinos) |
| 23, 11 | +5V — saída regulada da placa para circuitos externos (2 pinos) |
| 24, 25 | V_SUPPLY — entrada de alimentação bruta da placa (12–24V), vai pro regulador que gera o +5V (2 pinos, ligados juntos) |
| 3, 16 | SCL1, SDA1 (I2C) |
| 4, 17, 5 | HC595_DS, HC595_CLK, HC595_LATCH (shift register) |
| 18, 6, 19, 7, 20 | DIR_2, DIR_1, EN_ALL, PWM1, PWM2 (controle de motor/atuador) |
| 8, 21, 9 | GPIO23, GPIO34, GPIO35 (uso geral / entrada ou saída) |
| 12, 13 | CANL/GPIO32, CANH/GPIO33 (depende da população do TJA1050 e da configuração aplicada aos resistores R40–R43 (0Ω)) |

## 6. Estágio de alimentação (V_SUPPLY 12–24V → +5V → +3,3V)

**Proteção de entrada:**
- L4 (varistor S10K20V): supressão de surto/transiente na entrada.
- Q1 (IRF9530, MOSFET canal-P) + D1 (1N4740A, zener 10V): proteção de polaridade reversa. D1 limita o gate em 10V, com boa margem dentro do ±20V máximo do IRF9530 mesmo com picos acima dos 24V nominais.

**Sensor de corrente da alimentação lógica:**
- U3 (ACS712-05B): sensor de corrente Hall em série no caminho da alimentação protegida → VIN do buck. Pinagem e capacitores (C7 100nF em VCC, C8 1nF em FILTER) batem exatamente com o circuito de referência da Allegro. VIOUT → `LOGIC_CURRENT_SENS`.
- Divisor R4 (4,7kΩ) / R6 (1kΩ) próximo ao Q1: gera `LOGIC_VOLTAGE_SENSE`, monitorando a tensão da alimentação lógica protegida.

**Buck — U1 (LM2596S-ADJ/TR) → +5V:**
- Pinagem confirmada: VIN, OUT, GND, FB, ON_OFF.
- ON_OFF → GND (prática padrão recomendada quando o shutdown não é usado).
- Realimentação: R1 (trimmer 10kΩ, topo, FB→VOUT) + R5 (1kΩ, base, FB→GND) + C1 (10nF, feedforward em paralelo com R1) — divisor completo, sensando a saída **regulada** (depois do indutor L1), calibrável para 5V exatos (R1 ≈ 3,07kΩ na posição de ajuste).
- D2 (SS14, Schottky): diodo de roda-livre no nó de chaveamento — correto para topologia buck não-síncrona.
- L1 (47µH) + C3 (220µF) + R3 (0Ω, jumper): filtro de saída.
- C2 (100µF): capacitor de entrada do buck.

**LDO — U2 (LM1117-3,3) → +3,3V:**
- Entrada a partir do +5V regulado, com C4/C5 (100nF) de decoupling na entrada/saída.

**Test points:** CN1 (VIN), CN2 (5V_REG), CN3 (3V3_REG), CN4 (VL_LOGIC), CN5 (LOGIC_CURRENT_SENS).

## 7. ADC — ADS1115 (U8) + level shifter I2C bidirecional

**ADS1115IDGSR:**
- ADDR (pino 1): pull-up 10kΩ (R14) para +5V → endereço I2C 0x49 (uma das 4 opções válidas do datasheet).
- ALERT/RDY (pino 2): pull-up 10kΩ (R13) para +5V — necessário porque o pino é open-drain.
- C11 (100nF): decoupling em VDD, posicionado propositalmente perto dos pinos de alimentação (nota no próprio esquemático).
- 4 canais de entrada monitorados:
  - AIN0 → `PWR_VOLTAGE_SENS`
  - AIN1 → `PWR_CURRENT_SENS`
  - AIN2 → `LOGIC_VOLTAGE_SENSE`
  - AIN3 → `LOGIC_CURRENT_SENS`

**Level shifter I2C 5V↔3,3V (Q2/Q3, BSN20, topologia N-MOSFET bidirecional padrão):**
- R9/R10 (4,7kΩ): pull-up do lado 5V (SDA, SCL).
- R11/R12 (3,3kΩ): pull-up do lado 3,3V (SDA3V3, SCL3V3).
- Gate de Q2 e Q3 amarrado em +3,3V (rail mais baixo) — correto.
- Dreno no lado 5V, Fonte no lado 3,3V em ambos os transistores — polaridade correta (evita condução indevida do diodo de body intrínseco).
- Test points: CN8 (SDA), CN9 (SDA_3V3), CN10 (SCL), CN11 (SCL_3V3).

## 8. Sensores de PAD externo — ACS758 (U7)

Mede a potência consumida por um circuito externo conectado a dois PADs expostos na placa.

- U7 (ACS758): pinagem confirmada (1-VCC, 2-GND, 3-VIOUT, 4-IP+, 5-IP-).
- IP+ ← PWR_PAD_BATT (U5) | IP- → PWR_PAD_SENS (U6): corrente flui do pad de bateria para o pad monitorado, dando slope positivo em VIOUT conforme 0 datasheet.
- VCC = +5V (C13 100nF) | GND = GND geral | VIOUT → `PWR_CURRENT_SENS` (mesmo net que alimenta o AIN1 do ADS1115).
- Divisor R8 (4,7kΩ) / R15 (1kΩ), a partir do mesmo nó de PWR_PAD_BATT (`Vl_PWR`, test point CN7): gera `PWR_VOLTAGE_SENS`, razão ~1:5,7 — escala até ~30V de entrada pra caber nos 5,3V máximos de entrada do ADS1115, com boa folga acima da faixa de 12–24V do V_SUPPLY.
- C12 (100nF): filtro no nó de sense.

## Status geral

Todos os blocos da placa foram revisados e validados de ponta a ponta — MCU, programação/auto-download, isolamento USB, CAN, estágio de alimentação, ADC/level-shifter e os dois pares de sensores de corrente/tensão (ACS712 na alimentação lógica, ACS758 nos PADs externos). Nenhuma pendência em aberto.
