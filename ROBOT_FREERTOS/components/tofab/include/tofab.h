/**
 * @file tofab.h
 * @brief Placa auxiliar ToFaB_i2c-PI2-2026.2: expansor PCA9554A + 3x VL53L1X no I2C1.
 *
 * Os três VL53L1X sobem no mesmo endereço (0x29). O expansor controla o XSHUT
 * de cada um, então no boot o driver:
 *   1. coloca todos os XSHUT em nível baixo (sensores em reset);
 *   2. libera um sensor por vez, espera o boot e grava o endereço definitivo;
 *   3. configura e inicia o ranging contínuo nos três.
 *
 * Mapeamento do expansor:
 *   P0 = ToF_XSHUT1 (sensor esquerdo, 0x30)
 *   P1 = ToF_XSHUT2 (sensor central,  0x31)
 *   P2 = ToF_XSHUT3 (sensor direito,  0x32)
 *   P3 = header H5 (configurado como entrada)
 *   P4 = header H5 (configurado como entrada)
 *   P7 / P6 / P5 = LEDs 1 / 2 / 3 da fita (via BC337, ativos em alto).
 *   Os nomes RED/GREEN/BLUE abaixo são só apelidos de LED 1/2/3.
 *
 * Cada XSHUT tem pull-up de 10k para +3,3V (R13-R15) na placa. Como o
 * PCA9554 liga com todos os pinos como entrada, os sensores ficam ativos
 * até tofab_init() assumir o controle.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "pca9554.h"
#include "vl53l1x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PCA9554A com A2=A1=A0=GND: 0111_000 = 0x38. */
#ifndef TOFAB_EXPANDER_ADDR
#define TOFAB_EXPANDER_ADDR     0x38
#endif

#define TOFAB_NUM_TOF           3

/* Endereços atribuídos aos ToF no boot. */
#define TOFAB_TOF_ADDR_LEFT     0x30
#define TOFAB_TOF_ADDR_CENTER   0x31
#define TOFAB_TOF_ADDR_RIGHT    0x32

/* Pinos do expansor. */
#define TOFAB_PIN_XSHUT1        0
#define TOFAB_PIN_XSHUT2        1
#define TOFAB_PIN_XSHUT3        2
#define TOFAB_PIN_H5_A          3
#define TOFAB_PIN_H5_B          4
#define TOFAB_PIN_BLUE_LED      5
#define TOFAB_PIN_GREEN_LED     6
#define TOFAB_PIN_RED_LED       7

/* Distância reportada quando não há medida válida. */
#define TOFAB_DIST_INVALID_MM   8190

typedef enum {
    TOFAB_TOF_LEFT   = 0, /*!< XSHUT1, 0x30, cone esquerdo */
    TOFAB_TOF_CENTER = 1, /*!< XSHUT2, 0x31, cone central  */
    TOFAB_TOF_RIGHT  = 2, /*!< XSHUT3, 0x32, cone direito  */
} tofab_tof_id_t;

/** LEDs da fita, combináveis com OR. */
typedef enum {
    TOFAB_LED_OFF   = 0,
    TOFAB_LED_1     = 1u << TOFAB_PIN_RED_LED,   /*!< P7 */
    TOFAB_LED_2     = 1u << TOFAB_PIN_GREEN_LED, /*!< P6 */
    TOFAB_LED_3     = 1u << TOFAB_PIN_BLUE_LED,  /*!< P5 */
    TOFAB_LED_RED   = 1u << TOFAB_PIN_RED_LED,
    TOFAB_LED_GREEN = 1u << TOFAB_PIN_GREEN_LED,
    TOFAB_LED_BLUE  = 1u << TOFAB_PIN_BLUE_LED,
} tofab_led_t;

typedef struct {
    vl53l1x_distance_mode_t distance_mode;
    uint16_t timing_budget_ms;      /*!< valores discretos aceitos pelo driver vl53l1x */
    uint16_t inter_measurement_ms;  /*!< >= timing_budget_ms */
} tofab_config_t;

/*
 * Modo curto (alcance ~1,3 m) cobre o limiar de arena de 1120 mm e é mais
 * imune à luz ambiente. Budget de 33 ms com período de 40 ms: 25 Hz.
 */
#define TOFAB_CONFIG_DEFAULT() {                    \
    .distance_mode        = VL53L1X_DISTANCE_SHORT, \
    .timing_budget_ms     = 33,                     \
    .inter_measurement_ms = 40,                     \
}

typedef struct {
    uint16_t distance_mm;   /*!< TOFAB_DIST_INVALID_MM se !valid */
    uint8_t  range_status;  /*!< 0 = medida válida (ver vl53l1x_range_status_str) */
    bool     valid;         /*!< range_status == 0 */
    bool     novo;          /*!< true se esta chamada leu uma amostra nova do sensor */
    int64_t  timestamp_us;  /*!< esp_timer_get_time() da última amostra nova */
} tofab_reading_t;

typedef struct {
    i2c_master_bus_handle_t bus;
    pca9554_t         expander;
    SemaphoreHandle_t lock;          /*!< protege out_shadow e escritas no expansor */
    uint8_t           out_shadow;    /*!< cópia local do Output Port */
    tofab_config_t    cfg;
    vl53l1x_t         tof[TOFAB_NUM_TOF];
    bool              tof_ok[TOFAB_NUM_TOF];
    tofab_reading_t   last[TOFAB_NUM_TOF];
} tofab_t;

/**
 * @brief Anexa o expansor, sequencia os XSHUT, reendereça e inicia os 3 ToF.
 *
 * Retorna ESP_OK se o expansor respondeu e pelo menos um ToF subiu. Sensores
 * que falharem ficam com XSHUT em baixo e tof_ok[i] = false; consulte
 * tofab_tof_ok() antes de confiar na leitura.
 *
 * @param cfg NULL usa TOFAB_CONFIG_DEFAULT().
 */
esp_err_t tofab_init(tofab_t *b, i2c_master_bus_handle_t bus, const tofab_config_t *cfg);

/**
 * @brief Refaz a sequência de boot dos ToF (reset via XSHUT e reendereçamento).
 *
 * Usar após travamento do I2C1, depois de recuperar o barramento.
 */
esp_err_t tofab_restart_tofs(tofab_t *b);

/** Remove os handles I2C do expansor e dos ToF. */
esp_err_t tofab_deinit(tofab_t *b);

bool tofab_tof_ok(const tofab_t *b, tofab_tof_id_t id);

/**
 * @brief Leitura não bloqueante de um ToF.
 *
 * Se houver amostra nova, lê, rearma o sensor e retorna com out->novo = true.
 * Caso contrário devolve a última amostra conhecida com out->novo = false.
 */
esp_err_t tofab_read(tofab_t *b, tofab_tof_id_t id, tofab_reading_t *out);

/** Lê os três ToF com tofab_read(). Retorna o primeiro erro encontrado. */
esp_err_t tofab_read_all(tofab_t *b, tofab_reading_t out[TOFAB_NUM_TOF]);

/** Define os LEDs da fita (combinação de tofab_led_t). */
esp_err_t tofab_set_led(tofab_t *b, uint8_t leds);

#ifdef __cplusplus
}
#endif
