#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "vl53l1x.h"

/* Ajuste esses pinos para a sua placa. Nos ESP32-DevKit "genéricos" os
 * pinos abaixo costumam estar livres, mas confira o pinout da sua placa
 * específica antes de ligar o sensor. */
#define I2C_SDA_GPIO   18
#define I2C_SCL_GPIO   19
#define I2C_PORT       I2C_NUM_0

static const char *TAG = "app_main";

static i2c_master_bus_handle_t s_i2c_bus;
static vl53l1x_t s_tof;

static void i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
}

void app_main(void)
{
    i2c_bus_init();

    ESP_ERROR_CHECK(vl53l1x_init_bus_device(s_i2c_bus, VL53L1X_DEFAULT_I2C_ADDR, &s_tof));

    esp_err_t err = vl53l1x_sensor_init(&s_tof);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "falha ao inicializar o VL53L1X (0x%x). Confira alimentacao e fiacao I2C.", err);
        return;
    }
    ESP_LOGI(TAG, "VL53L1X inicializado com sucesso");

    /* Configuração de medição: alcance longo (até ~4 m), timing budget de
     * 100 ms e período entre medições de 200 ms (deve ser >= timing budget). */
    ESP_ERROR_CHECK(vl53l1x_set_distance_mode(&s_tof, VL53L1X_DISTANCE_LONG));
    ESP_ERROR_CHECK(vl53l1x_set_timing_budget_ms(&s_tof, 100));
    ESP_ERROR_CHECK(vl53l1x_set_inter_measurement_ms(&s_tof, 200));

    ESP_ERROR_CHECK(vl53l1x_start_ranging(&s_tof));

    while (1) {
        vl53l1x_result_t result;
        esp_err_t r = vl53l1x_wait_for_result(&s_tof, &result, 1000);
        if (r == ESP_OK) {
            if (result.range_status == 0) {
                ESP_LOGI(TAG, "distancia: %u mm  (status: %s)",
                         result.distance_mm, vl53l1x_range_status_str(result.range_status));
            } else {
                ESP_LOGW(TAG, "leitura descartada, status: %s (%u)",
                         vl53l1x_range_status_str(result.range_status), result.range_status);
            }
        } else {
            ESP_LOGE(TAG, "timeout/erro aguardando medicao (0x%x)", r);
        }
    }
}
