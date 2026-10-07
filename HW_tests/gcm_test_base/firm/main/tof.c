#include "tof.h"
#include "app_config.h"
#include "expander.h"
#include "vl53l1x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "tof";

typedef enum {
    TOF_OFFLINE = 0,
    TOF_ADDRESSED,
    TOF_RANGING,
} tof_state_t;

typedef struct {
    const char       *name;
    uint8_t           xshut_pin;
    uint8_t           addr;
    tof_state_t       state;
    vl53l1x_t         dev;
    vl53l1x_result_t  last;
    bool              has_result;
    uint32_t          t_last_ms;
    uint32_t          i2c_errors;
} tof_t;

static tof_t s_tof[TOF_COUNT] = {
    { .name = "S1", .xshut_pin = PCA_PIN_XSHUT_S1, .addr = TOF_ADDR_S1 },
    { .name = "S2", .xshut_pin = PCA_PIN_XSHUT_S2, .addr = TOF_ADDR_S2 },
    { .name = "S3", .xshut_pin = PCA_PIN_XSHUT_S3, .addr = TOF_ADDR_S3 },
};

static i2c_master_bus_handle_t s_bus;
static SemaphoreHandle_t s_mtx;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void xshut_set(const tof_t *s, bool released)
{
    esp_err_t err = expander_set_pin(s->xshut_pin, released);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s: falha ao escrever XSHUT (IO%u): %s", s->name, s->xshut_pin, esp_err_to_name(err));
    }
}

/* Fase A: todos os XSHUT em LOW; libera um por vez e troca o endereço. */
static void assign_addresses(void)
{
    /* Reset de todos (e retorno a 0x29, caso o ESP32 tenha reiniciado com os
     * sensores ainda energizados). */
    for (int i = 0; i < TOF_COUNT; i++) xshut_set(&s_tof[i], false);
    vTaskDelay(pdMS_TO_TICKS(20));

    if (i2c_master_probe(s_bus, VL53L1X_DEFAULT_I2C_ADDR, 50) == ESP_OK) {
        ESP_LOGW(TAG, "Algo responde em 0x%02X com todos os XSHUT em LOW. "
                      "Confira a fiacao do XSHUT (ou ha outro dispositivo nesse endereco).",
                 VL53L1X_DEFAULT_I2C_ADDR);
    }

    for (int i = 0; i < TOF_COUNT; i++) {
        tof_t *s = &s_tof[i];
        s->state = TOF_OFFLINE;

        ESP_LOGI(TAG, "%s: liberando XSHUT (IO%u)", s->name, s->xshut_pin);
        xshut_set(s, true);
        vTaskDelay(pdMS_TO_TICKS(5));

        esp_err_t err = vl53l1x_init_bus_device(s_bus, VL53L1X_DEFAULT_I2C_ADDR, &s->dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: falha ao criar handle em 0x29: %s", s->name, esp_err_to_name(err));
            xshut_set(s, false);
            continue;
        }

        err = vl53l1x_wait_for_boot(&s->dev, 500);
        if (err == ESP_OK) err = vl53l1x_set_i2c_address(s_bus, &s->dev, s->addr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: nao respondeu/mudou de endereco (%s). Mantendo XSHUT em LOW.",
                     s->name, esp_err_to_name(err));
            if (s->dev.i2c_dev) vl53l1x_deinit(&s->dev);
            xshut_set(s, false);   /* não pode ficar em 0x29 e colidir com o próximo */
            continue;
        }

        uint16_t id = 0;
        err = vl53l1x_get_sensor_id(&s->dev, &id);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: sem resposta em 0x%02X apos trocar o endereco (%s)",
                     s->name, s->addr, esp_err_to_name(err));
            vl53l1x_deinit(&s->dev);
            xshut_set(s, false);
            continue;
        }
        ESP_LOGI(TAG, "%s: endereco 0x%02X ok, Model ID = 0x%04X%s", s->name, s->addr, id,
                 id == 0xEACC ? "" : "  (esperado 0xEACC!)");
        s->state = TOF_ADDRESSED;
    }
}

/* Fase B: configura e inicia ranging nos sensores já endereçados. */
static int start_all(void)
{
    int n_ok = 0;
    for (int i = 0; i < TOF_COUNT; i++) {
        tof_t *s = &s_tof[i];
        if (s->state != TOF_ADDRESSED) continue;

        esp_err_t err = vl53l1x_sensor_init(&s->dev);
        if (err == ESP_OK) err = vl53l1x_set_distance_mode(&s->dev, TOF_DISTANCE_MODE);
        if (err == ESP_OK) err = vl53l1x_set_timing_budget_ms(&s->dev, TOF_TIMING_BUDGET_MS);
        if (err == ESP_OK) err = vl53l1x_set_inter_measurement_ms(&s->dev, TOF_INTER_MEAS_MS);
        if (err == ESP_OK) err = vl53l1x_start_ranging(&s->dev);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: falha na inicializacao/configuracao (%s)", s->name, esp_err_to_name(err));
            s->state = TOF_OFFLINE;
            continue;
        }
        s->state = TOF_RANGING;
        n_ok++;
        ESP_LOGI(TAG, "%s (0x%02X): ranging iniciado", s->name, s->addr);
    }
    return n_ok;
}

int tof_init_all(i2c_master_bus_handle_t bus)
{
    s_bus = bus;
    if (!s_mtx) s_mtx = xSemaphoreCreateMutex();

    if (!expander_ok() || !s_mtx) {
        ESP_LOGE(TAG, "PCA9554A indisponivel: sensores ToF offline");
        for (int i = 0; i < TOF_COUNT; i++) s_tof[i].state = TOF_OFFLINE;
        return 0;
    }

    assign_addresses();
    int n = start_all();
    ESP_LOGI(TAG, "%d de %d sensores em operacao", n, TOF_COUNT);
    return n;
}

void tof_get(int idx, tof_snapshot_t *out)
{
    *out = (tof_snapshot_t){ 0 };
    if (idx < 0 || idx >= TOF_COUNT || !s_mtx) return;

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    const tof_t *s = &s_tof[idx];
    out->online     = (s->state == TOF_RANGING);
    out->has_result = s->has_result;
    out->mm         = s->last.distance_mm;
    out->status     = s->last.range_status;
    out->age_ms     = s->has_result ? (now_ms() - s->t_last_ms) : 0;
    out->i2c_errors = s->i2c_errors;
    xSemaphoreGive(s_mtx);
}

static void poll_once(void)
{
    for (int i = 0; i < TOF_COUNT; i++) {
        tof_t *s = &s_tof[i];
        if (s->state != TOF_RANGING) continue;

        bool ready = false;
        esp_err_t err = vl53l1x_check_data_ready(&s->dev, &ready);
        if (err != ESP_OK) {
            xSemaphoreTake(s_mtx, portMAX_DELAY);
            s->i2c_errors++;
            xSemaphoreGive(s_mtx);
            continue;
        }
        if (!ready) continue;

        vl53l1x_result_t r;
        err = vl53l1x_get_result(&s->dev, &r);
        if (err == ESP_OK) err = vl53l1x_clear_interrupt(&s->dev);

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        if (err != ESP_OK) {
            s->i2c_errors++;
        } else {
            s->last = r;
            s->has_result = true;
            s->t_last_ms = now_ms();
        }
        xSemaphoreGive(s_mtx);
    }
}

static void tof_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        poll_once();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TOF_POLL_MS));
    }
}

void tof_start_task(void)
{
    xTaskCreate(tof_task, "tof", 4096, NULL, 5, NULL);
}
