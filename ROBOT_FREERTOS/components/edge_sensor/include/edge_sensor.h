/**
 * @file edge_sensor.h
 * @brief Sensores de borda LM393 com interrupção e latch.
 *
 * Ligação via CN15 (GCM-PI2-2026.2, igual a HW_tests/gcm_test_base):
 *   BS_1 GPIO34 (CN15-21) -> borda frontal esquerda  (pull-up externo R23)
 *   BS_2 GPIO35 (CN15-9)  -> borda frontal direita   (pull-up externo R22)
 *   BS_3 GPIO16 (CN15-19) -> borda traseira esquerda (pull-up interno)
 *   BS_4 GPIO14 (CN15-18) -> borda traseira direita  (pull-up interno)
 *
 * Confirme na montagem qual sensor físico vai em cada BS_x; as macros abaixo
 * podem ser redefinidas antes do include.
 *
 * Nível lógico: 0 = linha branca, 1 = piso preto.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EDGE_GPIO_FRONT_LEFT
#define EDGE_GPIO_FRONT_LEFT    GPIO_NUM_34   /* BS_1 */
#endif
#ifndef EDGE_GPIO_FRONT_RIGHT
#define EDGE_GPIO_FRONT_RIGHT   GPIO_NUM_35   /* BS_2 */
#endif
#ifndef EDGE_GPIO_REAR_LEFT
#define EDGE_GPIO_REAR_LEFT     GPIO_NUM_16   /* BS_3 */
#endif
#ifndef EDGE_GPIO_REAR_RIGHT
#define EDGE_GPIO_REAR_RIGHT    GPIO_NUM_14   /* BS_4 */
#endif

typedef enum {
    EDGE_FRONT_LEFT  = 0,
    EDGE_FRONT_RIGHT = 1,
    EDGE_REAR_LEFT   = 2,
    EDGE_REAR_RIGHT  = 3,
    EDGE_COUNT
} edge_sensor_id_t;

#define EDGE_MASK(id)   ((uint8_t)(1u << (id)))
#define EDGE_MASK_ALL   ((uint8_t)((1u << EDGE_COUNT) - 1u))
#define EDGE_MASK_FRONT (EDGE_MASK(EDGE_FRONT_LEFT) | EDGE_MASK(EDGE_FRONT_RIGHT))
#define EDGE_MASK_REAR  (EDGE_MASK(EDGE_REAR_LEFT) | EDGE_MASK(EDGE_REAR_RIGHT))

/** Retrato consistente do estado dos 4 sensores. */
typedef struct {
    uint8_t  latched;                     /*!< bit i = sensor i travado desde o último clear */
    uint8_t  level;                       /*!< bit i = sensor i vendo linha agora */
    uint32_t count[EDGE_COUNT];           /*!< total de disparos (bordas de descida) */
    int64_t  latch_time_us[EDGE_COUNT];   /*!< esp_timer_get_time() do disparo que travou */
} edge_snapshot_t;

/**
 * Callback chamado DENTRO da ISR a cada borda de descida. A ISR roda da IRAM,
 * então o callback precisa ser IRAM_ATTR, curto e usar apenas APIs *FromISR.
 * Retorne true se acordou uma task de prioridade maior.
 */
typedef bool (*edge_sensor_isr_cb_t)(edge_sensor_id_t id, void *arg);

/**
 * @brief Configura os 4 GPIOs com interrupção na borda de descida.
 *
 * Sensores que já estão sobre a linha na inicialização começam travados.
 * @param cb Pode ser NULL; o latch funciona sem callback.
 */
esp_err_t edge_sensor_init(edge_sensor_isr_cb_t cb, void *arg);

esp_err_t edge_sensor_deinit(void);

/**
 * Máscara usada pela lógica de decisão: sensores travados OU vendo linha
 * agora. Este é o valor a consultar a cada tick do laço de controle.
 */
uint8_t edge_sensor_line_mask(void);

/** Lê estado travado, nível atual, contadores e instantes de uma vez. */
void edge_sensor_snapshot(edge_snapshot_t *snap);

/**
 * @brief Destrava os sensores de `mask` que já saíram da linha.
 *
 * Um sensor só é destravado se (a) não está vendo linha agora e (b) não
 * houve disparo novo depois de `snap` (contador igual). Assim um toque novo
 * na linha entre o snapshot e o clear nunca é apagado.
 *
 * @param snap Snapshot que a lógica usou para decidir. NULL ignora o teste (b).
 * @return Máscara dos sensores que continuam travados.
 */
uint8_t edge_sensor_clear(const edge_snapshot_t *snap, uint8_t mask);

/** Nível instantâneo do pino (true = linha branca agora). Não usa o latch. */
bool edge_sensor_is_line(edge_sensor_id_t id);

/** Nível instantâneo dos 4 pinos. Não usa o latch. */
uint8_t edge_sensor_read_level_mask(void);

gpio_num_t edge_sensor_gpio(edge_sensor_id_t id);

#ifdef __cplusplus
}
#endif
