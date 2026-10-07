# Resumo de Hardware — GCM (General Control Module) — GCM-PI2-2026.2

> Base: esquemático `GCM-PI2-2026.2-211062820` (3 folhas: PWR_Supply, Logic_core, Annotations). Folha Logic_core atualizada em 2026-10-07; PWR_Supply em 2026-08-27.
> Placa: ESP32-WROOM-32D · UnB/FCTE · Projeto Integrador 2 (robô sumô).
> Legenda de confiança: **[folha]** = lido direto do esquemático desta revisão · **[herdado]** = vem da revisão anterior validada, em trecho não alterado · **[inferido]** = deduzido, confirmar.

---

## 1. Visão geral

Central de controle modular baseada no ESP32-WROOM-32D (Wi-Fi + Bluetooth), com:

- Alimentação bruta de 12–24 V (V_SUPPLY) → buck LM2596 → +5 V → LDO LM1117-3.3 → +3,3 V.
- USB isolado (ADuM3160) + ponte USB-serial CH340C com auto-download (Q4/Q5).
- Transceiver CAN TJA1050 (configuração atual: **CAN habilitado**).
- Telemetria de potência com ADS1115 (I2C0, 16 bits): 2 pares tensão/corrente (lógica e PADs externos).
- Conector principal CN15 (chamado de "DB25" na descrição do projeto; no esquemático são 27 pinos) com interface para pontes H, sensores de borda, botão de start e expansor PCA9554 externo (I2C1).

---

## 2. Mapa de GPIOs do ESP32 (U12) [folha]

| GPIO | Net | Destino | Observação |
|---|---|---|---|
| 0 | ESP_IO0 | Q5 (auto-download) | Strapping; R17 10 kΩ pull-up p/ 3V3 |
| 2 | — | R39 10 kΩ pull-down | Strapping; não exposto |
| 4 | START_BOT | CN15-20 | Botão de start; resistor em série (ver §2.1) |
| 5 | — | R25 10 kΩ pull-up | Strapping; não exposto |
| 12 | — | R38 10 kΩ pull-down | Strapping (VDD_SDIO = 3,3 V); não exposto |
| 13 | PWM1 | CN15-7 | Ponte H 1, fio **cinza**; R37 em série |
| 14 | BS_4 | CN15-18 | Sensor de borda 4; resistor em série |
| 15 | — | R24 10 kΩ pull-up + pad BOOT_LOG_DIS p/ GND | Strapping (boot log) |
| 16 | BS_3 | CN15-19 | Sensor de borda 3; resistor em série |
| 17 | DIR_1 | CN15-6 | Ponte H 1, fio **branco**; resistor em série |
| 18 | SDA1 | CN15-16 | I2C1 (PCA9554 externo) |
| 19 | SCL1 | CN15-3 | I2C1 (PCA9554 externo) |
| 21 | SDA3V3 | Level shifter Q2 → ADS1115 | I2C0 |
| 22 | SCL3V3 | Level shifter Q3 → ADS1115 | I2C0 |
| 23 | EN_ALL | CN15-8 | Habilita pontes H, fio **roxo**; R19 em série |
| 25 | DIR2 | CN15-4 | Ponte H 2, fio **marrom**; resistor em série |
| 26 | PCA9554_INT | CN15-17 | Interrupção do PCA9554; resistor em série |
| 27 | PWM2 | CN15-5 | Ponte H 2, fio **preto**; resistor em série |
| 32 | TX_CAN | TXD do TJA1050 (e CN16) | Compartilha o pino CN15-12 só no modo GPIO (ver §5) |
| 33 | RX_CAN | RXD do TJA1050 via divisor (e CN17) | Compartilha o pino CN15-13 só no modo GPIO (ver §5) |
| 34 | BS_1 | CN15-21 | Sensor de borda 1; **R23 10 kΩ pull-up p/ 3V3** |
| 35 | BS_2 | CN15-9 | Sensor de borda 2; **R22 10 kΩ pull-up p/ 3V3** |
| 1 / 3 | ESP_TX0 / ESP_RX0 | CH340C | UART0 (programação/log) |
| VP / VN | — | GND | Aterrados, não usados |

Pinos da flash SPI interna (SD0–SD3, CMD, CLK) e o NC (pino 32 do módulo) ficam sem conexão [folha].

### 2.1 Resistores em série nas saídas/entradas do CN15
Há resistores em série (sem valor legível no PDF exportado) em GPIO25, 26, 27, 14, 13, 17, 16 e 4 (designadores R30–R37, exceto R19) e o R19 em GPIO23 (EN_ALL). GPIO34, 35, 32 e 33 não têm resistor em série. **Valores e a atribuição individual R↔sinal não são legíveis na exportação; ver "Itens a confirmar".**

---

## 3. Pinout do CN15 (27 pinos) [folha]

| Pino | Sinal | GPIO | Obs. |
|---|---|---|---|
| 1, 2, 14, 15, 26, 27 | GND | — | 6 pinos |
| 3 | SCL1 | 19 | I2C1 |
| 16 | SDA1 | 18 | I2C1 |
| 4 | DIR2 | 25 | fio marrom |
| 17 | PCA9554_INT | 26 | |
| 5 | PWM2 | 27 | fio preto |
| 18 | BS_4 | 14 | |
| 6 | DIR_1 | 17 | fio branco |
| 19 | BS_3 | 16 | |
| 7 | PWM1 | 13 | fio cinza |
| 20 | START_BOT | 4 | |
| 8 | EN_ALL | 23 | fio roxo |
| 21 | BS_1 | 34 | pull-up externo R23 |
| 9 | BS_2 | 35 | pull-up externo R22 |
| 10, 22 | +3,3 V | — | saída |
| 11, 23 | +5 V | — | saída |
| 24, 25 | V_SUPPLY | — | entrada bruta 12–24 V |
| 12 | CANL / GPIO32 | 32 | ver §5 |
| 13 | CANH / GPIO33 | 33 | ver §5 |

---

## 4. Sistema de potência

### 4.1 Entrada e proteções [folha, inalterado]
- **U4 (varistor S10K20V)** de V_SUPPLY para GND: supressão de surto.
- **Q1 (IRF9530, P-MOSFET)**: proteção de polaridade reversa, dreno em V_SUPPLY e fonte em VIN+. **R4 (10 kΩ)** do gate ao GND e **D1 (1N4740A, zener 10 V)** entre gate e fonte limitam |Vgs| a 10 V.
- **U3 (ACS712)** em série em VIN+ → alimentação do buck. VCC = +5 V (C7 100 nF), FILTER com C8 1 nF. VIOUT → `LOGIC_CURRENT_SENS` (CN5). A variante (5 A/20 A/30 A) **não** consta no esquemático; a revisão anterior citava ACS712-05B.
- **R2 (4,7 kΩ) / R6 (1 kΩ)** + C6 100 nF em VIN+ → `LOGIC_VOLTAGE_SENS` (CN4). Razão 1:5,7.

### 4.2 Buck U1 (LM2596S-ADJ) → +5 V [folha / herdado]
- C2 100 µF na entrada; D2 SS14 (roda-livre); L1 47 µH; C3 220 µF; R3 0 Ω; CN2 = 5V_REG.
- Realimentação (R1 trimmer 10 kΩ ligado como reostato + R5 1 kΩ + C1 10 nF em paralelo com a parte superior): sensa a saída **após** L1. Para 5 V: R1 ≈ 3,07 kΩ (Vref ≈ 1,23 V).
- ON_OFF → GND [herdado: validado na revisão anterior; trecho de fiação inalterado na folha 1].

### 4.3 LDO U2 (LM1117-3.3) → +3,3 V [folha]
- Entrada no +5 V; C4 100 nF na entrada, C5 100 nF na saída; CN3 = 3V3_REG.
- **Ver item de atenção A1** (capacitor de saída).

### 4.4 Filtro pi da alimentação do ADS1115 [folha, documentado na folha 3]
- +5 V → C9 10 µF → L2 22 µH → R7 2,2 Ω → C10 10 µF → rail `+5V_ADS1115` (também em CN6 = 5V_AVDD).
- Teórico: fc ≈ 10,7 kHz; atenuação ≈ 45,8 dB em 150 kHz (frequência do LM2596). Simulação LTspice: ≈ 56 dB.
- O rail `+5V_ADS1115` alimenta: VDD do ADS1115 (C11 100 nF perto dos pinos), pull-ups R13/R14 e pull-ups R9/R10 do lado 5 V do I2C0.

---

## 5. CAN — TJA1050 (U13) [folha]

- Pinagem: 1 TXD, 2 GND, 3 VCC (+5 V), 4 RXD, 5 VREF (NC), 6 CANL, 7 CANH, 8 S (→ GND, modo high-speed).
- TXD ← `TX_CAN` (GPIO32) direto (lógica de 3,3 V).
- RXD → R28 10 kΩ (série) → nó `RX_CAN` (GPIO33) com R29 22 kΩ para GND: saída de 5 V reduzida a ≈ 3,44 V.
- H1 (jumper de 2 pinos) + R21 120 Ω: terminação CAN ativa quando H1 está fechado.
- CN16 = TX_CAN e CN17 = RX_CAN: pontos de teste.

### 5.1 Seleção de modo (R40–R43, 0 Ω)
- R40: CANH ↔ nó `CANH/GPIO33` · R41: nó `CANH/GPIO33` ↔ RX_CAN
- R42: CANL ↔ nó `CANL/GPIO32` · R43: nó `CANL/GPIO32` ↔ TX_CAN
- **Modo CAN (atual, "CAN enabled")**: U13 montado; R40 e R42 montados; R41 e R43 **não** montados. CN15-13 = CANH e CN15-12 = CANL. GPIO32/33 ficam dedicados a TX/RX. **[inferido a partir de "CAN enabled": confirmar quais resistores estão montados]**
- **Modo GPIO**: U13 **não** montado; R41 e R43 montados; R40 e R42 não montados. CN15-13 = GPIO33 e CN15-12 = GPIO32. Nunca montar R40/R41 (ou R42/R43) juntos com o TJA1050 presente.

---

## 6. USB isolado e programação [folha / herdado]

- **U9** (USB-Micro) → **D3 (SS14)** em série na linha VBUS → VBUS1 do **U10 (ADuM3160)**; CN13 = VBUS (teste). D3 é item novo em relação ao resumo anterior.
- U10 lado 1 (host): GND no domínio isolado `GND2` (CN14); PDEN e SPU em VDD1 (full speed). Lado 2: VBUS2 em +5 V da placa, GND2 do chip = GND geral, VDD2/SPD/PIN amarrados.
- **U11 (CH340C)**: VCC +5 V, V3 com capacitor para GND (C20), TXD → `ESP_RX0` e RXD → `ESP_TX0` (cruzamento correto).
- **Auto-download**: Q4 (→ EN) e Q5 (→ GPIO0), BC547-B, R16/R18 10 kΩ, topologia com emissores cruzados em RTS/DTR [herdado: validado na revisão anterior; fiação RTS/DTR não reavaliada em pixel nesta revisão].
- Botão RST: `ESP_EN` para GND.
- LED1 (TLHG4605) + R44 470 Ω em +5 V: indicador de energia.

### Strapping [folha]
| Pino | Rede | Estado no boot |
|---|---|---|
| GPIO0 | R17 pull-up 10 kΩ + Q5 | HIGH |
| GPIO2 | R39 pull-down 10 kΩ | LOW |
| GPIO5 | R25 pull-up 10 kΩ | HIGH |
| GPIO12 | R38 pull-down 10 kΩ | LOW (flash a 3,3 V) |
| GPIO15 | R24 pull-up 10 kΩ + pad BOOT_LOG_DIS p/ GND | HIGH (LOW silencia o boot log) |
| EN | R20 10 kΩ pull-up p/ 3V3 + botão RST | HIGH |

---

## 7. Telemetria — ADS1115 (U8) + level shifter I2C0 [folha / herdado]

- **ADS1115IDGSR**: ADDR → +5V_ADS1115 via R14 (endereço **0x49**), ALERT/RDY com pull-up R13 10 kΩ (open-drain), VDD = +5V_ADS1115.
- Canais: **AIN0** `PWR_VOLTAGE_SENS` · **AIN1** `PWR_CURRENT_SENS` · **AIN2** `LOGIC_VOLTAGE_SENS` · **AIN3** `LOGIC_CURRENT_SENS`.
- **Level shifter** Q2/Q3 (BSN20): gate em +3,3 V; lado 5 V com pull-ups R9/R10 4,7 kΩ (+5V_ADS1115), lado 3,3 V com R11/R12 3,3 kΩ. SDA/SCL (5 V) ↔ SDA3V3/SCL3V3 (GPIO21/22).
- Testpoints: CN8 SDA, CN9 SDA_3V3, CN10 SCL, CN11 SCL_3V3.

### Sensor externo — ACS758 (U7)
- Pinagem: 1 VCC (+5 V, C13 100 nF), 2 GND, 3 VIOUT, 4 IP+, 5 IP−.
- IP+ ← `PWR_PAD_BATT` (U5) · IP− → `PWR_PAD_SENS` (U6). VIOUT → `PWR_CURRENT_SENS` (CN12). A variante do ACS758 (faixa de corrente, uni/bidirecional) **não** consta no esquemático.
- Divisor **R8 4,7 kΩ / R15 1 kΩ** a partir de `PWR_PAD_BATT` (CN7 = Vi_PWR) + C12 100 nF → `PWR_VOLTAGE_SENS`.

---

## 8. I2C1 (PCA9554 externo)

- GPIO18 (SDA1) e GPIO19 (SCL1) vão ao CN15 (pinos 16 e 3).
- **R26/R27 (3,3 kΩ, pull-ups para 3V3)** só devem ser montados se o I2C1 for usado (nota no esquemático). Esses pull-ups ficam no lado do ESP; o PCA9554 está fora da placa (chicote).
- PCA9554_INT em GPIO26 (CN15-17).

---

## 9. O que mudou em relação ao resumo anterior

1. **CN15 totalmente remapeado**: saiu o shift register HC595 (DS/CLK/LATCH); entram `BS_1..4`, `START_BOT`, `PCA9554_INT`, `DIR_1/DIR2`, `EN_ALL`, `PWM1/PWM2` e o I2C1. Os GPIOs 23/34/35 deixaram de ser "uso geral": são EN_ALL, BS_1 e BS_2.
2. **GPIO34/35**: antes com pull-up externo genérico; agora são as entradas dos sensores de borda BS_1/BS_2 (R23 → BS_1, R22 → BS_2).
3. **CAN**: o RXD (via divisor R28/R29) vai agora ao **GPIO33** (antes ia ao GPIO34), e TXD ao GPIO32. Isso liberou GPIO34/35 para os sensores de borda.
4. **EN**: C21 (100 nF) aparece agora como bypass do +3,3 V; não há capacitor dedicado no EN na folha.
5. **Rail do ADS1115**: filtro pi (L2/R7/C9/C10) com rail `+5V_ADS1115` documentado na folha 3.
6. **USB**: D3 (SS14) na linha VBUS.
7. **Designadores renumerados** em parte da folha de potência (varistor agora **U4**; divisor da lógica R2/R6; ACS712 = U3). Use os designadores desta revisão.

---

## 10. Itens de atenção

**A1. LM1117-3.3 com apenas 100 nF na saída (C5).** O datasheet da TI exige no mínimo 10 µF (tantalo) na saída para estabilidade, com ESR entre 0,3 Ω e 22 Ω, e recomenda 10 µF tantalo na entrada. O +5 V à montante tem C3 220 µF, então a entrada está coberta; a saída não. Recomendo incluir ≥10 µF no +3,3 V (ou usar LDO estável com cerâmica) — o ESP32 gera picos de corrente no Wi-Fi/BT e é o caso mais crítico.

**A2. Faixa do divisor `LOGIC_VOLTAGE_SENS` / `PWR_VOLTAGE_SENS` (1:5,7).** Em ±4,096 V de fundo de escala do ADS1115 (PGA padrão, ganho 1), satura em ≈ 23,3 V de entrada. Para medir até 24–25 V, configurar o PGA em ±6,144 V (a tensão no pino do ADS1115 não pode passar de VDD + 0,3 V ≈ 5,3 V, ou seja, ≈ 30 V no divisor).

**A3. Varistor S10K20V.** Pelo que lembro da série S10K20 (≈ 20 Vrms / 26 Vdc), a margem sobre 24 V é pequena. Confirmar no datasheet do fabricante se pretende operar V_SUPPLY próximo de 24 V contínuos.

**A4. Entradas do CN15 sem proteção.** BS_1…BS_4 e START_BOT entram direto no ESP32 (3,3 V). Qualquer sensor com saída de 5 V ligado ali danifica o pino. BS_3 (GPIO16) e BS_4 (GPIO14) não têm pull-up externo; dependem do sensor ou de pull-up interno configurado no firmware.

**A5. EN sem RC dedicado.** Não é problema com o botão RST e o auto-download, mas vale saber caso apareçam resets na energização.

---

## 11. Itens a confirmar

1. **Valores** dos resistores em série R19 e R30–R37, e a atribuição de cada um ao sinal.
2. Resistores **R40–R43** realmente montados (assumi R40/R42 montados e R41/R43 não, por "CAN enabled").
3. Variante exata do **ACS712** e do **ACS758**.
4. Valor/estado do pad **BOOT_LOG_DIS** (a revisão anterior dizia 0 Ω não montado).
5. Se o **H1** (terminação 120 Ω) fica fechado ou aberto na placa de uso.
