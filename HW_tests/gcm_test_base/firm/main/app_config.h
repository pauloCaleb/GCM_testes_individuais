/**
 * @file app_config.h
 * @brief Configuração central do firmware de testes da GCM-PI2-2026.2.
 *
 * Tudo que depende da fiação/hardware fica aqui, para a equipe ajustar sem
 * mexer na lógica dos módulos.
 */
#pragma once

#include "driver/gpio.h"

#define FW_NAME        "gcm_test_base"
#define FW_VERSION     "1.0.0"
#define PROTO_VERSION  1

/* ------------------------------------------------------------------ */
/* Entradas digitais (conector CN15)                                   */
/* ------------------------------------------------------------------ */
#define PIN_BS1            GPIO_NUM_34   /* CN15-21, pull-up externo R23 */
#define PIN_BS2            GPIO_NUM_35   /* CN15-9,  pull-up externo R22 */
#define PIN_BS3            GPIO_NUM_16   /* CN15-19 */
#define PIN_BS4            GPIO_NUM_14   /* CN15-18 */
#define PIN_START          GPIO_NUM_4    /* CN15-20 */

#define INPUT_POLL_MS            10
/* Amostras iguais consecutivas para aceitar uma mudança (3 x 10 ms = 30 ms). */
#define INPUT_DEBOUNCE_SAMPLES   3
/* Pull-up interno em GPIO16/14/4 (GPIO34/35 não têm pull interno). */
#define INPUT_USE_INTERNAL_PULLUP 1

/* ------------------------------------------------------------------ */
/* Pontes H (BTS7960) - interface PWM + DIR + EN                       */
/* ------------------------------------------------------------------ */
#define PIN_EN_ALL         GPIO_NUM_23   /* CN15-8,  fio roxo   */
#define PIN_M1_PWM         GPIO_NUM_13   /* CN15-7,  fio cinza  */
#define PIN_M1_DIR         GPIO_NUM_17   /* CN15-6,  fio branco */
#define PIN_M2_PWM         GPIO_NUM_27   /* CN15-5,  fio preto  */
#define PIN_M2_DIR         GPIO_NUM_25   /* CN15-4,  fio marrom */

/* INH do BTS7960: nível alto habilita, nível baixo = sleep. */
#define EN_ACTIVE_LEVEL    1
/* Nível de DIR que corresponde a duty POSITIVO ("frente") em cada motor. */
#define M1_DIR_FWD_LEVEL   1
#define M2_DIR_FWD_LEVEL   1

/* PWM: o BTS7960 aceita até 25 kHz. 20 kHz fica fora da faixa audível. */
#define MOTOR_PWM_FREQ_HZ        20000
#define MOTOR_TICK_MS            10
/* Ticks parado em duty 0 depois de trocar DIR, antes de voltar a acelerar. */
#define MOTOR_DIR_SETTLE_TICKS   2

/* Valores iniciais (a GUI pode alterá-los com o comando "cfg"). */
#define MOTOR_DEFAULT_MAX_DUTY   30.0f   /* % - limite de segurança */
#define MOTOR_DEFAULT_SLEW       300.0f  /* %/s - 0 = sem rampa */
#define MOTOR_DEFAULT_WD_MS      500     /* ms - 0 = watchdog desligado */

/* ------------------------------------------------------------------ */
/* I2C1 (PCA9554A + 3x VL53L1X)                                        */
/* ------------------------------------------------------------------ */
#define I2C1_SDA_GPIO      GPIO_NUM_18
#define I2C1_SCL_GPIO      GPIO_NUM_19
#define I2C1_PORT          I2C_NUM_1

/* PCA9554A com A2=A1=A0=GND. */
#define PCA9554_I2C_ADDR   0x38

#define PCA_PIN_XSHUT_S1   0
#define PCA_PIN_XSHUT_S2   1
#define PCA_PIN_XSHUT_S3   2
#define PCA_PIN_LED_1      7
#define PCA_PIN_LED_2      6
#define PCA_PIN_LED_3      5

/* 1 = nível alto acende o LED. */
#define LED_ACTIVE_HIGH    1
#define LED_ON_TIME_MS     1000
/* LED que fica aceso depois do autoteste de boot (um dos PCA_PIN_LED_x). */
#define LED_FIXED_PIN      PCA_PIN_LED_2
/* 1 = executa o autoteste dos LEDs no boot. */
#define BOOT_LED_SELFTEST  1

/* Endereços I2C finais dos VL53L1X (fora de 0x29 e das faixas do PCA9554/A). */
#define TOF_ADDR_S1        0x30
#define TOF_ADDR_S2        0x31
#define TOF_ADDR_S3        0x32

#define TOF_DISTANCE_MODE     VL53L1X_DISTANCE_LONG
#define TOF_TIMING_BUDGET_MS  100
#define TOF_INTER_MEAS_MS     200
#define TOF_POLL_MS           5

/* ------------------------------------------------------------------ */
/* Protocolo serial                                                    */
/* ------------------------------------------------------------------ */
#define PROTO_UART_NUM     UART_NUM_0
#define PROTO_LINE_MAX     256
#define PROTO_TX_MAX       256
#define PROTO_TEL_MAX      640
#define PROTO_DEFAULT_TEL_HZ 20
