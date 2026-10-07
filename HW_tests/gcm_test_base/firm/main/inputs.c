#include "inputs.h"
#include <stdbool.h>
#include "app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "inputs";

typedef struct {
    const char *name;
    int         idx;
    gpio_num_t  gpio;
    bool        has_pull;   /* GPIO34-39 não têm pull interno */
    int         level;      /* nível aceito (debounced) */
    int         cand;       /* último nível visto */
    int         count;      /* amostras consecutivas do candidato */
} input_t;

static input_t s_in[] = {
    { "bs",    1, PIN_BS1,   false, -1, -1, 0 },
    { "bs",    2, PIN_BS2,   false, -1, -1, 0 },
    { "bs",    3, PIN_BS3,   true,  -1, -1, 0 },
    { "bs",    4, PIN_BS4,   true,  -1, -1, 0 },
    { "start", 0, PIN_START, true,  -1, -1, 0 },
};
#define N_IN (sizeof(s_in) / sizeof(s_in[0]))

static SemaphoreHandle_t s_mtx;
static inputs_event_cb_t s_cb;

esp_err_t inputs_init(void)
{
    s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return ESP_ERR_NO_MEM;

    for (size_t i = 0; i < N_IN; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << s_in[i].gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = (INPUT_USE_INTERNAL_PULLUP && s_in[i].has_pull)
                            ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&cfg);
        if (err != ESP_OK) return err;

        int lvl = gpio_get_level(s_in[i].gpio);
        s_in[i].level = s_in[i].cand = lvl;
        s_in[i].count = INPUT_DEBOUNCE_SAMPLES;
    }
    ESP_LOGI(TAG, "entradas configuradas");
    return ESP_OK;
}

void inputs_set_event_cb(inputs_event_cb_t cb)
{
    s_cb = cb;
}

void inputs_get(inputs_state_t *out)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (int i = 0; i < 4; i++) out->bs[i] = (uint8_t)s_in[i].level;
    out->start = (uint8_t)s_in[4].level;
    xSemaphoreGive(s_mtx);
}

static void inputs_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        for (size_t i = 0; i < N_IN; i++) {
            input_t *in = &s_in[i];
            int lvl = gpio_get_level(in->gpio);

            if (lvl == in->cand) {
                if (in->count < INPUT_DEBOUNCE_SAMPLES) in->count++;
            } else {
                in->cand = lvl;
                in->count = 1;
            }

            if (in->count >= INPUT_DEBOUNCE_SAMPLES && in->level != in->cand) {
                xSemaphoreTake(s_mtx, portMAX_DELAY);
                in->level = in->cand;
                xSemaphoreGive(s_mtx);
                if (s_cb) s_cb(in->name, in->idx, in->level);
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(INPUT_POLL_MS));
    }
}

void inputs_start_task(void)
{
    xTaskCreate(inputs_task, "inputs", 3072, NULL, 5, NULL);
}
