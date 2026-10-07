#include "expander.h"
#include "app_config.h"
#include "pca9554.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "expander";

#define XSHUT_MASK ((uint8_t)((1u << PCA_PIN_XSHUT_S1) | (1u << PCA_PIN_XSHUT_S2) | (1u << PCA_PIN_XSHUT_S3)))
#define LED_MASK   ((uint8_t)((1u << PCA_PIN_LED_1) | (1u << PCA_PIN_LED_2) | (1u << PCA_PIN_LED_3)))
/* Configuration: 0 = saída. IO3/IO4 não são usados e ficam como entrada. */
#define PCA_CONFIG_VALUE ((uint8_t)(~(XSHUT_MASK | LED_MASK)))
/* Estado inicial: XSHUT em LOW, LEDs desligados. */
#define PCA_OUTPUT_IDLE  ((uint8_t)(LED_ACTIVE_HIGH ? 0x00 : LED_MASK))

static const uint8_t s_led_pins[3] = { PCA_PIN_LED_1, PCA_PIN_LED_2, PCA_PIN_LED_3 };

static pca9554_t s_pca;
static SemaphoreHandle_t s_mtx;
static uint8_t s_out = PCA_OUTPUT_IDLE;
static bool s_ok;

esp_err_t expander_init(i2c_master_bus_handle_t bus)
{
    s_ok = false;
    if (!s_mtx) s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return ESP_ERR_NO_MEM;

    esp_err_t err = i2c_master_probe(bus, PCA9554_I2C_ADDR, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9554A nao respondeu em 0x%02X (%s)", PCA9554_I2C_ADDR, esp_err_to_name(err));
        return err;
    }
    err = pca9554_init(bus, PCA9554_I2C_ADDR, &s_pca);
    if (err != ESP_OK) return err;

    s_out = PCA_OUTPUT_IDLE;
    err = pca9554_write_output_port(&s_pca, s_out);
    if (err != ESP_OK) return err;
    err = pca9554_write_config(&s_pca, PCA_CONFIG_VALUE);
    if (err != ESP_OK) return err;

    uint8_t cfg = 0xFF, out = 0xFF;
    err = pca9554_read_config(&s_pca, &cfg);
    if (err == ESP_OK) err = pca9554_read_output_port(&s_pca, &out);
    if (err != ESP_OK) return err;
    if (cfg != PCA_CONFIG_VALUE || out != PCA_OUTPUT_IDLE) {
        ESP_LOGE(TAG, "readback divergente: Config=0x%02X (esp. 0x%02X) Output=0x%02X (esp. 0x%02X)",
                 cfg, PCA_CONFIG_VALUE, out, PCA_OUTPUT_IDLE);
        return ESP_FAIL;
    }

    s_ok = true;
    ESP_LOGI(TAG, "PCA9554A ok em 0x%02X (Config=0x%02X, Output=0x%02X)", PCA9554_I2C_ADDR, cfg, out);
    return ESP_OK;
}

bool expander_ok(void)
{
    return s_ok;
}

esp_err_t expander_set_pin(uint8_t pin, bool level)
{
    if (!s_ok || pin > 7) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    uint8_t next = level ? (uint8_t)(s_out | (1u << pin)) : (uint8_t)(s_out & ~(1u << pin));
    esp_err_t err = pca9554_write_output_port(&s_pca, next);
    if (err == ESP_OK) s_out = next;
    xSemaphoreGive(s_mtx);
    return err;
}

bool expander_get_pin(uint8_t pin)
{
    if (pin > 7 || !s_mtx) return false;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    bool v = (s_out >> pin) & 1u;
    xSemaphoreGive(s_mtx);
    return v;
}

esp_err_t expander_led_set(int idx, bool on)
{
    if (idx < 1 || idx > 3) return ESP_ERR_INVALID_ARG;
    bool level = LED_ACTIVE_HIGH ? on : !on;
    return expander_set_pin(s_led_pins[idx - 1], level);
}

bool expander_led_get(int idx)
{
    if (idx < 1 || idx > 3) return false;
    bool level = expander_get_pin(s_led_pins[idx - 1]);
    return LED_ACTIVE_HIGH ? level : !level;
}
