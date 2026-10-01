#include "edge_sensor.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "edge_sensor";

/* Linha branca = nível 0 na saída do LM393. */
#define EDGE_LINE_LEVEL 0

static const gpio_num_t kEdgeGpio[EDGE_COUNT] = {
    EDGE_GPIO_FRONT_LEFT,
    EDGE_GPIO_FRONT_RIGHT,
    EDGE_GPIO_REAR_LEFT,
    EDGE_GPIO_REAR_RIGHT,
};

/* Estado compartilhado com a ISR; acesso sempre dentro de s_mux. */
static DRAM_ATTR edge_sensor_isr_cb_t s_cb;
static DRAM_ATTR void *s_cb_arg;
static DRAM_ATTR volatile uint8_t  s_latched;
static DRAM_ATTR volatile uint32_t s_count[EDGE_COUNT];
static DRAM_ATTR volatile int64_t  s_latch_time_us[EDGE_COUNT];
static DRAM_ATTR portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_initialized;

/*
 * Roda da IRAM para não ser adiada quando o cache da flash está desligado
 * (escrita em NVS, por exemplo). Repiques do LM393 na transição só
 * incrementam o contador; o instante de travamento é o do primeiro disparo.
 */
static void IRAM_ATTR edge_isr(void *arg)
{
    edge_sensor_id_t id = (edge_sensor_id_t)(uintptr_t)arg;
    uint8_t bit = EDGE_MASK(id);
    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&s_mux);
    s_count[id]++;
    if ((s_latched & bit) == 0) {
        s_latched |= bit;
        s_latch_time_us[id] = now;
    }
    portEXIT_CRITICAL_ISR(&s_mux);

    if (s_cb != NULL && s_cb(id, s_cb_arg)) {
        portYIELD_FROM_ISR();
    }
}

/* GPIO34-39 do ESP32 são somente entrada e não têm pull-up/pull-down interno. */
static bool gpio_has_internal_pull(gpio_num_t gpio)
{
    return gpio < GPIO_NUM_34;
}

static uint8_t level_mask(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < EDGE_COUNT; i++) {
        if (gpio_get_level(kEdgeGpio[i]) == EDGE_LINE_LEVEL) {
            mask |= EDGE_MASK(i);
        }
    }
    return mask;
}

esp_err_t edge_sensor_init(edge_sensor_isr_cb_t cb, void *arg)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < EDGE_COUNT; i++) {
        gpio_num_t gpio = kEdgeGpio[i];
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = gpio_has_internal_pull(gpio) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_NEGEDGE,
        };
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_config GPIO%d: %s", gpio, esp_err_to_name(err));
            return err;
        }
    }

    portENTER_CRITICAL(&s_mux);
    s_cb = cb;
    s_cb_arg = arg;
    memset((void *)s_count, 0, sizeof(s_count));
    memset((void *)s_latch_time_us, 0, sizeof(s_latch_time_us));
    s_latched = 0;
    portEXIT_CRITICAL(&s_mux);

    /*
     * ESP_INTR_FLAG_IRAM: a ISR continua atendendo com o cache desligado.
     * Se outro módulo já instalou o serviço, ESP_ERR_INVALID_STATE é aceito.
     */
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < EDGE_COUNT; i++) {
        err = gpio_isr_handler_add(kEdgeGpio[i], edge_isr, (void *)(uintptr_t)i);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_isr_handler_add GPIO%d: %s", kEdgeGpio[i], esp_err_to_name(err));
            for (int j = 0; j < i; j++) {
                gpio_isr_handler_remove(kEdgeGpio[j]);
            }
            return err;
        }
    }

    /* Sensor já sobre a linha não gera borda de descida: trava pelo nível. */
    uint8_t on_line = level_mask();
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < EDGE_COUNT; i++) {
        if ((on_line & EDGE_MASK(i)) && !(s_latched & EDGE_MASK(i))) {
            s_latched |= EDGE_MASK(i);
            s_latch_time_us[i] = now;
        }
    }
    portEXIT_CRITICAL(&s_mux);

    s_initialized = true;
    ESP_LOGI(TAG, "borda em GPIO%d/%d/%d/%d, sobre a linha no init: 0x%X",
             kEdgeGpio[0], kEdgeGpio[1], kEdgeGpio[2], kEdgeGpio[3], on_line);
    return ESP_OK;
}

esp_err_t edge_sensor_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    for (int i = 0; i < EDGE_COUNT; i++) {
        gpio_intr_disable(kEdgeGpio[i]);
        gpio_isr_handler_remove(kEdgeGpio[i]);
    }
    portENTER_CRITICAL(&s_mux);
    s_cb = NULL;
    s_cb_arg = NULL;
    portEXIT_CRITICAL(&s_mux);
    s_initialized = false;
    return ESP_OK;
}

uint8_t edge_sensor_line_mask(void)
{
    uint8_t lvl = level_mask();
    portENTER_CRITICAL(&s_mux);
    uint8_t latched = s_latched;
    portEXIT_CRITICAL(&s_mux);
    return latched | lvl;
}

void edge_sensor_snapshot(edge_snapshot_t *snap)
{
    if (snap == NULL) {
        return;
    }
    snap->level = level_mask();
    portENTER_CRITICAL(&s_mux);
    snap->latched = s_latched;
    for (int i = 0; i < EDGE_COUNT; i++) {
        snap->count[i] = s_count[i];
        snap->latch_time_us[i] = s_latch_time_us[i];
    }
    portEXIT_CRITICAL(&s_mux);
}

uint8_t edge_sensor_clear(const edge_snapshot_t *snap, uint8_t mask)
{
    /*
     * O nível é lido fora da seção crítica. Os contadores são capturados antes,
     * para detectar um disparo que caia entre a leitura do pino e o destrave.
     */
    uint32_t count_before[EDGE_COUNT];
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < EDGE_COUNT; i++) {
        count_before[i] = s_count[i];
    }
    portEXIT_CRITICAL(&s_mux);

    uint8_t on_line = level_mask();

    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < EDGE_COUNT; i++) {
        uint8_t bit = EDGE_MASK(i);
        if (!(mask & bit) || (on_line & bit)) {
            continue;
        }
        if (count_before[i] != s_count[i]) {
            continue; /* disparou enquanto o pino era lido */
        }
        if (snap != NULL && snap->count[i] != s_count[i]) {
            continue; /* disparou de novo depois do snapshot */
        }
        s_latched &= (uint8_t)~bit;
    }
    uint8_t still = s_latched;
    portEXIT_CRITICAL(&s_mux);
    return still;
}

bool edge_sensor_is_line(edge_sensor_id_t id)
{
    if ((unsigned)id >= EDGE_COUNT) {
        return false;
    }
    return gpio_get_level(kEdgeGpio[id]) == EDGE_LINE_LEVEL;
}

uint8_t edge_sensor_read_level_mask(void)
{
    return level_mask();
}

gpio_num_t edge_sensor_gpio(edge_sensor_id_t id)
{
    return ((unsigned)id < EDGE_COUNT) ? kEdgeGpio[id] : GPIO_NUM_NC;
}
