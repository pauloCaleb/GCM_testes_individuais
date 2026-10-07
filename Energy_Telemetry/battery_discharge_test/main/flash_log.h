#pragma once
/* Log decimado do teste na particao "blog" (registros de 32 bytes, tipicamente 1 Hz).
 * Sobrevive a reset do ESP32 e a queda do link; pode ser baixado pelo comando LOG. */
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct __attribute__((packed)) {
    uint32_t idx;
    uint16_t test_id;
    uint8_t  state;
    uint8_t  flags;
    float    t_test_s;
    float    v_batt_v;
    float    i_a;
    float    q_ah;
    float    e_wh;
    uint32_t crc;
} flog_rec_t;

esp_err_t flog_init(void);
bool flog_available(void);
uint32_t flog_capacity(void);                 /* registros */
void flog_begin(uint16_t test_id);            /* novo teste: recomeca do inicio da particao */
esp_err_t flog_append(uint8_t state, float t_test_s, float v, float i, float q, float e);
uint32_t flog_count(void);                    /* registros gravados no teste atual (RAM) */
bool flog_read(uint16_t test_id, uint32_t idx, flog_rec_t *out);   /* true se valido */
