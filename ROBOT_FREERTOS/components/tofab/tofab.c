#include "tofab.h"
#include <string.h>
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "tofab";

#define TOFAB_PROBE_TIMEOUT_MS   50

#define TOFAB_XSHUT_MASK  ((1u << TOFAB_PIN_XSHUT1) | (1u << TOFAB_PIN_XSHUT2) | (1u << TOFAB_PIN_XSHUT3))
#define TOFAB_LED_MASK    (TOFAB_LED_RED | TOFAB_LED_GREEN | TOFAB_LED_BLUE)
/* Configuration: 1 = entrada. P0-P2 e P5-P7 saída; P3/P4 (H5) entrada. */
#define TOFAB_CONFIG_REG  ((uint8_t)~(TOFAB_XSHUT_MASK | TOFAB_LED_MASK))

/* XSHUT de cada sensor e endereço definitivo, na ordem de tofab_tof_id_t. */
static const uint8_t kXshutPin[TOFAB_NUM_TOF] = {
    TOFAB_PIN_XSHUT1, TOFAB_PIN_XSHUT2, TOFAB_PIN_XSHUT3,
};
static const uint8_t kTofAddr[TOFAB_NUM_TOF] = {
    TOFAB_TOF_ADDR_LEFT, TOFAB_TOF_ADDR_CENTER, TOFAB_TOF_ADDR_RIGHT,
};
static const char *const kTofName[TOFAB_NUM_TOF] = { "esquerdo", "central", "direito" };

/*
 * Com CONFIG_FREERTOS_HZ=100 um tick vale 10 ms. Garante pelo menos 1 tick
 * para que o atraso nunca vire vTaskDelay(0).
 */
static void delay_ms(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    vTaskDelay(t > 0 ? t : 1);
}

/* ------------------------------ Expansor ------------------------------ */

/*
 * Atualiza bits do Output Port a partir da cópia local: uma transação I2C por
 * mudança, sem read-modify-write pelo barramento.
 */
static esp_err_t expander_update(tofab_t *b, uint8_t clear_mask, uint8_t set_mask)
{
    xSemaphoreTake(b->lock, portMAX_DELAY);
    uint8_t next = (uint8_t)((b->out_shadow & ~clear_mask) | set_mask);
    esp_err_t err = ESP_OK;
    if (next != b->out_shadow) {
        err = pca9554_write_output_port(&b->expander, next);
        if (err == ESP_OK) {
            b->out_shadow = next;
        }
    }
    xSemaphoreGive(b->lock);
    return err;
}

static esp_err_t xshut_set(tofab_t *b, tofab_tof_id_t id, bool enable)
{
    uint8_t bit = (uint8_t)(1u << kXshutPin[id]);
    return enable ? expander_update(b, 0, bit) : expander_update(b, bit, 0);
}

static esp_err_t expander_setup(tofab_t *b)
{
    esp_err_t err = i2c_master_probe(b->bus, TOFAB_EXPANDER_ADDR, TOFAB_PROBE_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "expansor não respondeu em 0x%02X (%s)", TOFAB_EXPANDER_ADDR, esp_err_to_name(err));
        return err;
    }

    err = pca9554_init(b->bus, TOFAB_EXPANDER_ADDR, &b->expander);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Escreve o Output Port antes do Configuration: quando os pinos virarem
     * saída, já saem em nível baixo (todos os ToF em reset, LED apagado).
     */
    b->out_shadow = 0x00;
    err = pca9554_write_output_port(&b->expander, b->out_shadow);
    if (err != ESP_OK) {
        return err;
    }
    err = pca9554_write_polarity_inversion(&b->expander, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    err = pca9554_write_config(&b->expander, TOFAB_CONFIG_REG);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t readback = 0;
    err = pca9554_read_config(&b->expander, &readback);
    if (err == ESP_OK && readback != TOFAB_CONFIG_REG) {
        ESP_LOGE(TAG, "Configuration lido 0x%02X, esperado 0x%02X", readback, TOFAB_CONFIG_REG);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (err != ESP_OK) {
        return err;
    }

    /* Input Port reflete o pino real: com XSHUT em baixo, P0-P2 devem ler 0. */
    uint8_t pins = 0xFF;
    err = pca9554_read_input_port(&b->expander, &pins);
    if (err == ESP_OK && (pins & TOFAB_XSHUT_MASK) != 0) {
        ESP_LOGE(TAG, "XSHUT não foi para nível baixo (pinos = 0x%02X)", pins);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}

/* ------------------------------ ToF ------------------------------ */

static void tof_release(tofab_t *b, tofab_tof_id_t id)
{
    if (b->tof[id].i2c_dev != NULL) {
        vl53l1x_deinit_bus_device(&b->tof[id]);
    }
    b->tof_ok[id] = false;
}

/*
 * Sobe um único sensor: só ele está fora de reset, então responde em 0x29.
 * Em caso de falha o XSHUT volta para baixo, para não ocupar 0x29 durante o
 * boot dos sensores seguintes.
 */
static esp_err_t tof_boot_one(tofab_t *b, tofab_tof_id_t id)
{
    vl53l1x_t *dev = &b->tof[id];
    memset(dev, 0, sizeof(*dev));

    esp_err_t err = xshut_set(b, id, true);
    if (err != ESP_OK) {
        return err;
    }
    /* tBOOT do VL53L1X é 1,2 ms; 20 ms (2 ticks) dá folga com tick de 10 ms. */
    delay_ms(20);

    err = vl53l1x_init_bus_device(b->bus, VL53L1X_DEFAULT_I2C_ADDR, dev);
    if (err != ESP_OK) goto fail;

    /* sensor_init espera o firmware do sensor ficar pronto antes de configurar. */
    err = vl53l1x_sensor_init(dev);
    if (err != ESP_OK) goto fail;

    err = vl53l1x_set_i2c_address(dev, kTofAddr[id]);
    if (err != ESP_OK) goto fail;

    err = vl53l1x_set_distance_mode(dev, b->cfg.distance_mode);
    if (err != ESP_OK) goto fail;
    err = vl53l1x_set_timing_budget_ms(dev, b->cfg.timing_budget_ms);
    if (err != ESP_OK) goto fail;
    err = vl53l1x_set_inter_measurement_ms(dev, b->cfg.inter_measurement_ms);
    if (err != ESP_OK) goto fail;
    err = vl53l1x_start_ranging(dev);
    if (err != ESP_OK) goto fail;

    b->tof_ok[id] = true;
    ESP_LOGI(TAG, "ToF %s pronto em 0x%02X", kTofName[id], kTofAddr[id]);
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "falha ao subir ToF %s (XSHUT P%u): %s",
             kTofName[id], kXshutPin[id], esp_err_to_name(err));
    tof_release(b, id);
    xshut_set(b, id, false);
    return err;
}

static esp_err_t tof_boot_all(tofab_t *b)
{
    /* Reset de todos: o endereço do VL53L1X é volátil e volta a 0x29. */
    esp_err_t err = expander_update(b, TOFAB_XSHUT_MASK, 0);
    if (err != ESP_OK) {
        return err;
    }
    delay_ms(20);

    int ok = 0;
    for (int i = 0; i < TOFAB_NUM_TOF; i++) {
        b->last[i] = (tofab_reading_t){ .distance_mm = TOFAB_DIST_INVALID_MM, .range_status = 255 };
        if (tof_boot_one(b, (tofab_tof_id_t)i) == ESP_OK) {
            ok++;
        }
    }

    ESP_LOGI(TAG, "%d de %d ToF ativos", ok, TOFAB_NUM_TOF);
    return ok > 0 ? ESP_OK : ESP_FAIL;
}

/* ------------------------------ API pública ------------------------------ */

esp_err_t tofab_init(tofab_t *b, i2c_master_bus_handle_t bus, const tofab_config_t *cfg)
{
    if (b == NULL || bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(b, 0, sizeof(*b));
    b->bus = bus;
    if (cfg != NULL) {
        b->cfg = *cfg;
    } else {
        b->cfg = (tofab_config_t)TOFAB_CONFIG_DEFAULT();
    }
    if (b->cfg.inter_measurement_ms < b->cfg.timing_budget_ms) {
        ESP_LOGE(TAG, "inter_measurement_ms (%u) menor que timing_budget_ms (%u)",
                 b->cfg.inter_measurement_ms, b->cfg.timing_budget_ms);
        return ESP_ERR_INVALID_ARG;
    }

    b->lock = xSemaphoreCreateMutex();
    if (b->lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = expander_setup(b);
    if (err != ESP_OK) {
        tofab_deinit(b);
        return err;
    }

    return tof_boot_all(b);
}

esp_err_t tofab_restart_tofs(tofab_t *b)
{
    if (b == NULL || b->lock == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < TOFAB_NUM_TOF; i++) {
        tof_release(b, (tofab_tof_id_t)i);
    }
    return tof_boot_all(b);
}

esp_err_t tofab_deinit(tofab_t *b)
{
    if (b == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < TOFAB_NUM_TOF; i++) {
        if (b->tof_ok[i]) {
            vl53l1x_stop_ranging(&b->tof[i]);
        }
        tof_release(b, (tofab_tof_id_t)i);
    }
    if (b->expander.i2c_dev != NULL) {
        pca9554_deinit(&b->expander);
    }
    if (b->lock != NULL) {
        vSemaphoreDelete(b->lock);
        b->lock = NULL;
    }
    return ESP_OK;
}

bool tofab_tof_ok(const tofab_t *b, tofab_tof_id_t id)
{
    return b != NULL && (unsigned)id < TOFAB_NUM_TOF && b->tof_ok[id];
}

esp_err_t tofab_read(tofab_t *b, tofab_tof_id_t id, tofab_reading_t *out)
{
    if (b == NULL || out == NULL || (unsigned)id >= TOFAB_NUM_TOF) {
        return ESP_ERR_INVALID_ARG;
    }

    tofab_reading_t *last = &b->last[id];
    if (!b->tof_ok[id]) {
        *out = *last;
        out->novo = false;
        return ESP_ERR_INVALID_STATE;
    }

    bool ready = false;
    esp_err_t err = vl53l1x_check_data_ready(&b->tof[id], &ready);
    if (err == ESP_OK && ready) {
        vl53l1x_result_t r;
        err = vl53l1x_get_result(&b->tof[id], &r);
        if (err == ESP_OK) {
            err = vl53l1x_clear_interrupt(&b->tof[id]);
        }
        if (err == ESP_OK) {
            last->range_status = r.range_status;
            last->valid        = (r.range_status == 0);
            last->distance_mm  = last->valid ? r.distance_mm : TOFAB_DIST_INVALID_MM;
            last->timestamp_us = esp_timer_get_time();
        }
    }

    *out = *last;
    out->novo = (err == ESP_OK) && ready;
    return err;
}

esp_err_t tofab_read_all(tofab_t *b, tofab_reading_t out[TOFAB_NUM_TOF])
{
    esp_err_t first_err = ESP_OK;
    for (int i = 0; i < TOFAB_NUM_TOF; i++) {
        esp_err_t err = tofab_read(b, (tofab_tof_id_t)i, &out[i]);
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
        }
    }
    return first_err;
}

esp_err_t tofab_set_led(tofab_t *b, uint8_t leds)
{
    if (b == NULL || b->lock == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return expander_update(b, TOFAB_LED_MASK, leds & TOFAB_LED_MASK);
}
