#pragma once
/* Coeficientes de calibracao (persistidos em NVS) e contador de testes. */
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    int32_t v_gain_ppm;   /* ganho da tensao, 1000000 = 1,000000 */
    int32_t v_off_uv;     /* offset da tensao em uV, somado apos o ganho */
    int32_t i_gain_ppm;   /* ganho da corrente, 1000000 = 1,000000 */
} cal_t;

#define CAL_GAIN_NOM_PPM 1000000
#define CAL_GAIN_MIN_PPM 800000
#define CAL_GAIN_MAX_PPM 1200000
#define CAL_OFF_MAX_UV   500000

esp_err_t cal_store_init(void);          /* nvs_flash_init + leitura dos valores salvos */
const cal_t *cal_get(void);
bool cal_valid(const cal_t *c);
esp_err_t cal_set(const cal_t *c);       /* valida e salva */
esp_err_t cal_reset(void);               /* volta ao padrao (ganho 1, offset 0) e salva */
uint16_t cal_last_test_id(void);
uint16_t cal_next_test_id(void);         /* incrementa e persiste */
