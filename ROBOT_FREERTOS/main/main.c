/**
 * main.c - ROBOT (GCM-PI2-2026.2)
 *
 * Bring-up dos sensores do robô:
 *   - ToFaB no I2C1 (SDA=GPIO18, SCL=GPIO19): PCA9554A + 3x VL53L1X;
 *   - 4 sensores de borda LM393 (BS_1..BS_4) por GPIO com interrupção e latch.
 *
 * A task de borda acorda pela ISR; a task de ToF faz polling não bloqueante.
 * No robô, quem destrava a borda é a máquina de estados ao fim do escape;
 * aqui a task de teste destrava assim que o sensor sai da linha.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "tofab.h"
#include "edge_sensor.h"

static const char *TAG = "robot";

/* I2C1 da GCM: nets SCL1/SDA1 no CN15 (pinos 3/16), exclusivo da ToFaB.
 * Os pull-ups R26/R27 precisam estar montados (PCA9554A + 3 ToF a 400 kHz). */
#define I2C1_PORT       I2C_NUM_1
#define I2C1_SDA_GPIO   GPIO_NUM_18
#define I2C1_SCL_GPIO   GPIO_NUM_19

#define TOF_POLL_MS     10
#define TOF_LOG_EVERY   50   /* imprime a cada 50 ciclos de polling (~500 ms) */

static i2c_master_bus_handle_t s_i2c1;
static tofab_t s_tofab;
static TaskHandle_t s_edge_task;

static const char *const kEdgeName[EDGE_COUNT] = {
    "frontal esq", "frontal dir", "traseira esq", "traseira dir",
};

static void i2c1_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C1_PORT,
        .sda_io_num        = I2C1_SDA_GPIO,
        .scl_io_num        = I2C1_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &s_i2c1));
}

/* Roda na ISR (IRAM): só acorda a task de borda. */
static bool IRAM_ATTR edge_cb(edge_sensor_id_t id, void *arg)
{
    (void)id;
    (void)arg;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_edge_task, &woken);
    return woken == pdTRUE;
}

static void edge_task(void *arg)
{
    (void)arg;
    uint8_t reported = 0;

    while (1) {
        /* Acorda pela ISR ou a cada 100 ms para tentar destravar. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        edge_snapshot_t s;
        edge_sensor_snapshot(&s);

        for (int i = 0; i < EDGE_COUNT; i++) {
            uint8_t bit = EDGE_MASK(i);
            if ((s.latched & bit) && !(reported & bit)) {
                ESP_LOGW(TAG, "BORDA travada: %s (GPIO%d, %lu disparos)",
                         kEdgeName[i], edge_sensor_gpio((edge_sensor_id_t)i),
                         (unsigned long)s.count[i]);
            }
        }

        uint8_t still = edge_sensor_clear(&s, s.latched);
        for (int i = 0; i < EDGE_COUNT; i++) {
            uint8_t bit = EDGE_MASK(i);
            if ((s.latched & bit) && !(still & bit)) {
                ESP_LOGI(TAG, "borda liberada: %s", kEdgeName[i]);
            }
        }
        reported = still;
    }
}

static void tof_task(void *arg)
{
    (void)arg;
    tofab_reading_t r[TOFAB_NUM_TOF];
    uint32_t n = 0;

    while (1) {
        tofab_read_all(&s_tofab, r);

        if (++n % TOF_LOG_EVERY == 0) {
            ESP_LOGI(TAG, "ToF esq=%4u (st %3u)  centro=%4u (st %3u)  dir=%4u (st %3u)  borda=0x%X",
                     r[TOFAB_TOF_LEFT].distance_mm,   r[TOFAB_TOF_LEFT].range_status,
                     r[TOFAB_TOF_CENTER].distance_mm, r[TOFAB_TOF_CENTER].range_status,
                     r[TOFAB_TOF_RIGHT].distance_mm,  r[TOFAB_TOF_RIGHT].range_status,
                     edge_sensor_line_mask());
        }
        vTaskDelay(pdMS_TO_TICKS(TOF_POLL_MS));
    }
}

void app_main(void)
{
    i2c1_init();

    esp_err_t err = tofab_init(&s_tofab, s_i2c1, NULL);
    if (err == ESP_OK) {
        tofab_set_led(&s_tofab, TOFAB_LED_GREEN);
    } else {
        ESP_LOGE(TAG, "ToFaB não inicializou (%s)", esp_err_to_name(err));
        tofab_set_led(&s_tofab, TOFAB_LED_RED);
    }

    xTaskCreate(edge_task, "edge", 3072, NULL, 10, &s_edge_task);
    ESP_ERROR_CHECK(edge_sensor_init(edge_cb, NULL));

    xTaskCreate(tof_task, "tof", 4096, NULL, 5, NULL);
}
