/**
 * @file main.c
 * @brief Teste de bring-up do PCA9554A na auxboard, via barramento I2C1 da GCM.
 *
 * Substitui o PCF8574 (que parou de responder no barramento, suspeita de dano
 * físico). O PCA9554A está montado com A0=A1=A2=GND, portanto endereço fixo
 * 0x38 (0111_000), confirmado por varredura no I2C1.
 *
 * Este teste:
 *  1. Inicializa o barramento I2C1 da GCM (SDA=GPIO18, SCL=GPIO19 — mesmo
 *     barramento usado pelos VL53L0X, conforme overview do board).
 *  2. Configura todos os 8 pinos do PCA9554A como saída (Configuration
 *     register = 0x00).
 *  3. Pisca os pinos de forma sequencial (um por vez, em varredura 0-7) em
 *     loop, validando o driver completo (escrita) e, a cada pino, faz uma
 *     leitura de volta do Output Port register para confirmar que o dado
 *     programado bate com o esperado.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "pca9554.h"

static const char *TAG = "pca9554_blink_test";

/* I2C1 da GCM: mesmo barramento reservado a sensores externos/ToF (CN15, nets SCL1/SDA1). */
#define I2C1_SDA_GPIO      GPIO_NUM_18
#define I2C1_SCL_GPIO      GPIO_NUM_19
#define I2C1_PORT          I2C_NUM_1

/* PCA9554A com A2=A1=A0=GND -> endereço fixo 0111_000 = 0x38 */
#define PCA9554A_ADDR      0x38

#define BLINK_PERIOD_MS    500

static i2c_master_bus_handle_t s_i2c1_bus = NULL;
static pca9554_t s_expander;

static void i2c1_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C1_PORT,
        .sda_io_num        = I2C1_SDA_GPIO,
        .scl_io_num        = I2C1_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c1_bus));
    ESP_LOGI(TAG, "I2C1 inicializado (SDA=GPIO%d, SCL=GPIO%d)", I2C1_SDA_GPIO, I2C1_SCL_GPIO);
}

void app_main(void)
{
    i2c1_bus_init();

    ESP_ERROR_CHECK(pca9554_init(s_i2c1_bus, PCA9554A_ADDR, &s_expander));

    /* Configuration register: 0x00 = todos os 8 pinos como saída */
    ESP_ERROR_CHECK(pca9554_write_config(&s_expander, 0x00));

    uint8_t readback_config = 0xFF;
    ESP_ERROR_CHECK(pca9554_read_config(&s_expander, &readback_config));
    ESP_LOGI(TAG, "Configuration register lido de volta: 0x%02X (esperado 0x00)", readback_config);
    if (readback_config != 0x00) {
        ESP_LOGE(TAG, "Configuration register não bateu com o esperado — verifique o barramento/endereço");
    }

    while (1) {
        for (uint8_t pin = 0; pin < 8; pin++) {
            uint8_t output_value = (uint8_t)(1u << pin);

            esp_err_t err = pca9554_write_output_port(&s_expander, output_value);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Falha ao escrever Output Port (0x%02X, pino %d): %s",
                         output_value, pin, esp_err_to_name(err));
            } else {
                uint8_t readback = 0;
                err = pca9554_read_output_port(&s_expander, &readback);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Pino %d ON -> Output Port = 0x%02X (readback = 0x%02X)%s",
                             pin, output_value, readback,
                             (readback == output_value) ? "" : "  <-- DIVERGENTE");
                } else {
                    ESP_LOGW(TAG, "Falha ao ler de volta o Output Port: %s", esp_err_to_name(err));
                }
            }

            vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));
        }
    }
}