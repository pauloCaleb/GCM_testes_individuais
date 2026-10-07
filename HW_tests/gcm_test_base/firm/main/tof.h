/**
 * @file tof.h
 * @brief 3x VL53L1X no I2C1, com endereços atribuídos via XSHUT (PCA9554A).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define TOF_COUNT 3

typedef struct {
    bool     online;       /* ranging ativo */
    bool     has_result;   /* já houve ao menos uma leitura */
    uint16_t mm;
    uint8_t  status;       /* 0 = leitura válida */
    uint32_t age_ms;       /* ms desde a última leitura nova */
    uint32_t i2c_errors;
} tof_snapshot_t;

/**
 * Sequência completa: todos os XSHUT em LOW, libera um sensor por vez, troca o
 * endereço (0x29 -> TOF_ADDR_Sx), confere o Model ID, configura e inicia o
 * ranging contínuo. Sensores que falharem ficam offline (XSHUT em LOW).
 * Requer expander_ok(). Retorna a quantidade de sensores em operação.
 */
int tof_init_all(i2c_master_bus_handle_t bus);

void tof_start_task(void);
void tof_get(int idx, tof_snapshot_t *out);
