/**
 * @file inputs.h
 * @brief Polling dos sensores de borda (BS_1..BS_4) e do botão de start.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint8_t bs[4];   /* nível lógico cru do pino (0/1) */
    uint8_t start;
} inputs_state_t;

/** Chamado na task de polling a cada mudança debounced.
 *  name = "bs" (idx 1..4) ou "start" (idx 0). */
typedef void (*inputs_event_cb_t)(const char *name, int idx, int level);

esp_err_t inputs_init(void);
void inputs_set_event_cb(inputs_event_cb_t cb);
void inputs_start_task(void);
void inputs_get(inputs_state_t *out);
