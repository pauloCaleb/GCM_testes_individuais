/**
 * @file expander.h
 * @brief Acesso compartilhado ao PCA9554A (XSHUT dos ToF e LEDs da fita).
 *
 * Usa um registrador-sombra do Output Port protegido por mutex: cada escrita
 * é uma única transação I2C, sem read-modify-write, então LEDs (GUI) e XSHUT
 * (inicialização dos sensores) podem ser usados de tasks diferentes.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** Probe + configuração do PCA9554A. Output Port é escrito ANTES de Config,
 *  então XSHUT/LEDs já nascem no nível correto. */
esp_err_t expander_init(i2c_master_bus_handle_t bus);

bool expander_ok(void);

/** Escreve um pino (0..7) do Output Port. */
esp_err_t expander_set_pin(uint8_t pin, bool level);

/** Nível programado (sombra) de um pino (0..7). */
bool expander_get_pin(uint8_t pin);

/** LED 1..3 (IO7, IO6, IO5), respeitando LED_ACTIVE_HIGH. */
esp_err_t expander_led_set(int idx, bool on);
bool expander_led_get(int idx);
