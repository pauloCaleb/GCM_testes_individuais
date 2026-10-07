/**
 * @file main.c
 * @brief GCM-PI2-2026.2 - base de testes da placa (firmware).
 *
 * Une os testes individuais (entradas, ToF VL53L1X + PCA9554A) e adiciona
 * LEDs e motores controláveis por um protocolo JSON na UART0 (USB isolado).
 * A GUI em test_soft/ conversa com este firmware; o protocolo está em PROTOCOL.md.
 *
 * Ordem de boot:
 *   1. pontes H em estado seguro (EN inativo, PWM = 0)
 *   2. entradas digitais e UART
 *   3. I2C1 + PCA9554A
 *   4. autoteste dos LEDs (1 s cada; depois um fica aceso)
 *   5. sensores ToF: endereços via XSHUT + início do ranging
 *   6. tasks (entradas, ToF, protocolo) + mensagem "hello"
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "app_config.h"
#include "expander.h"
#include "inputs.h"
#include "motors.h"
#include "proto.h"
#include "tof.h"

static const char *TAG = "gcm_main";

static const uint8_t s_led_idx[3] = { 1, 2, 3 };

static void led_selftest(void)
{
#if BOOT_LED_SELFTEST
    for (size_t i = 0; i < sizeof(s_led_idx); i++) {
        ESP_LOGI(TAG, "LED %d ligado por %d ms", s_led_idx[i], LED_ON_TIME_MS);
        expander_led_set(s_led_idx[i], true);
        vTaskDelay(pdMS_TO_TICKS(LED_ON_TIME_MS));
        expander_led_set(s_led_idx[i], false);
    }
    /* LED_FIXED_PIN é um pino do PCA9554A (IO7/IO6/IO5); converte p/ índice 1..3. */
    int fixed = (LED_FIXED_PIN == PCA_PIN_LED_1) ? 1 : (LED_FIXED_PIN == PCA_PIN_LED_2) ? 2 : 3;
    ESP_LOGI(TAG, "LED %d fixo aceso", fixed);
    expander_led_set(fixed, true);
#endif
}

void app_main(void)
{
    /* 1) Antes de qualquer coisa: pontes H em estado seguro. */
    ESP_ERROR_CHECK(motors_init());
    motors_start_task();

    ESP_LOGI(TAG, "%s v%s - base de testes da GCM-PI2-2026.2", FW_NAME, FW_VERSION);

    /* 2) Entradas e UART (a UART já aceita comandos que cheguem durante o boot). */
    ESP_ERROR_CHECK(inputs_init());
    ESP_ERROR_CHECK(proto_init());

    /* 3) I2C1 + PCA9554A */
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C1_PORT,
        .sda_io_num        = I2C1_SDA_GPIO,
        .scl_io_num        = I2C1_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    ESP_LOGI(TAG, "I2C1 inicializado (SDA=GPIO%d, SCL=GPIO%d)", (int)I2C1_SDA_GPIO, (int)I2C1_SCL_GPIO);

    if (expander_init(bus) != ESP_OK) {
        ESP_LOGE(TAG, "PCA9554A indisponivel: LEDs e ToF ficam fora; entradas e motores seguem funcionando");
    } else {
        /* 4) Autoteste dos LEDs */
        led_selftest();
    }

    /* 5) Sensores ToF */
    tof_init_all(bus);

    /* 6) Tasks e hello */
    inputs_start_task();
    tof_start_task();
    proto_start_tasks();
    proto_send_hello();

    ESP_LOGI(TAG, "pronto: aguardando comandos (JSON-lines, 921600 baud)");
}
