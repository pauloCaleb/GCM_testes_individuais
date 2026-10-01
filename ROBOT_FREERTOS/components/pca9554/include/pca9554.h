#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Timeout de transação I2C em milissegundos (API nova do IDF 5.4.x usa ms, não ticks). */
#define PCA9554_I2C_XFER_TIMEOUT_MS 1000

/** Faixa de endereços do PCA9554 (fixo 0100xxx): 0x20–0x27, conforme A2:A1:A0. */
#define PCA9554_BASE_ADDR   0x20
/** Faixa de endereços do PCA9554A (fixo 0111xxx): 0x38–0x3F, conforme A2:A1:A0. */
#define PCA9554A_BASE_ADDR  0x38

/** Comandos (endereços) de registrador, Table 4 do datasheet. */
typedef enum {
    PCA9554_REG_INPUT_PORT   = 0x00, /*!< Somente leitura */
    PCA9554_REG_OUTPUT_PORT  = 0x01, /*!< Leitura/escrita */
    PCA9554_REG_POLARITY_INV = 0x02, /*!< Leitura/escrita */
    PCA9554_REG_CONFIG       = 0x03, /*!< Leitura/escrita */
} pca9554_reg_t;

/** Direção de um pino individual, usada em pca9554_set_pin_direction(). */
typedef enum {
    PCA9554_PIN_OUTPUT = 0, /*!< Bit 0 no Configuration register */
    PCA9554_PIN_INPUT  = 1, /*!< Bit 1 no Configuration register (default) */
} pca9554_pin_dir_t;

/** Handle de uma instância do dispositivo. */
typedef struct {
    i2c_master_dev_handle_t i2c_dev; /*!< Handle do dispositivo I2C (driver novo do IDF) */
    uint8_t address;                 /*!< Endereço I2C 7 bits em uso */
} pca9554_t;

/**
 * @brief Anexa um PCA9554/PCA9554A a um barramento I2C já inicializado (i2c_master_bus_handle_t).
 *
 * Não faz nenhuma escrita no dispositivo — apenas registra o device handle.
 * Chame pca9554_set_config() em seguida para definir as direções dos pinos.
 *
 * @param bus_handle Handle do barramento I2C obtido via i2c_new_master_bus().
 * @param address     Endereço I2C 7 bits do dispositivo (ex.: 0x38 para PCA9554A com A0=A1=A2=GND).
 * @param out_dev     Estrutura a ser preenchida.
 */
esp_err_t pca9554_init(i2c_master_bus_handle_t bus_handle, uint8_t address, pca9554_t *out_dev);

/** Remove o device handle do barramento I2C. */
esp_err_t pca9554_deinit(pca9554_t *dev);

/* ---------------------- Acesso a registrador completo (8 bits) ---------------------- */

/** Register 0 — Input Port: reflete o nível lógico atual dos pinos, independente da direção configurada. */
esp_err_t pca9554_read_input_port(pca9554_t *dev, uint8_t *value);

/** Register 1 — Output Port: escreve o byte de saída completo (afeta apenas os pinos configurados como saída). */
esp_err_t pca9554_write_output_port(pca9554_t *dev, uint8_t value);

/** Register 1 — Output Port: lê de volta o valor programado no flip-flop de saída (não o pino físico). */
esp_err_t pca9554_read_output_port(pca9554_t *dev, uint8_t *value);

/** Register 2 — Polarity Inversion: 1 = inverte a leitura do Input Port naquele bit, 0 = mantém (default). */
esp_err_t pca9554_write_polarity_inversion(pca9554_t *dev, uint8_t mask);

/** Register 2 — Polarity Inversion: lê a máscara programada atualmente. */
esp_err_t pca9554_read_polarity_inversion(pca9554_t *dev, uint8_t *mask);

/** Register 3 — Configuration: 1 = pino como entrada (default), 0 = pino como saída. */
esp_err_t pca9554_write_config(pca9554_t *dev, uint8_t mask);

/** Register 3 — Configuration: lê a máscara de direção programada atualmente. */
esp_err_t pca9554_read_config(pca9554_t *dev, uint8_t *mask);

/* ---------------------------- Conveniência por pino individual ---------------------------- */

/**
 * @brief Configura a direção de um único pino (0-7) via read-modify-write no Configuration register.
 */
esp_err_t pca9554_set_pin_direction(pca9554_t *dev, uint8_t pin, pca9554_pin_dir_t direction);

/**
 * @brief Escreve o nível lógico de um único pino de saída (0-7) via read-modify-write no Output Port register.
 *
 * Não altera a direção do pino; se o pino estiver configurado como entrada, o bit fica
 * armazenado no flip-flop mas não tem efeito no pino físico até que ele seja reconfigurado como saída.
 */
esp_err_t pca9554_set_pin_level(pca9554_t *dev, uint8_t pin, bool level_high);

/**
 * @brief Lê o nível lógico atual de um único pino (0-7) a partir do Input Port register.
 */
esp_err_t pca9554_get_pin_level(pca9554_t *dev, uint8_t pin, bool *level_high);

/**
 * @brief Habilita/desabilita a inversão de polaridade de um único pino de entrada (0-7).
 */
esp_err_t pca9554_set_pin_polarity_inversion(pca9554_t *dev, uint8_t pin, bool inverted);

#ifdef __cplusplus
}
#endif
