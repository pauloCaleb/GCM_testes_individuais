#include "cal_store.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "CAL";
static cal_t s_cal = { CAL_GAIN_NOM_PPM, 0, CAL_GAIN_NOM_PPM };
static uint16_t s_test_id;

bool cal_valid(const cal_t *c)
{
    return c->v_gain_ppm >= CAL_GAIN_MIN_PPM && c->v_gain_ppm <= CAL_GAIN_MAX_PPM &&
           c->i_gain_ppm >= CAL_GAIN_MIN_PPM && c->i_gain_ppm <= CAL_GAIN_MAX_PPM &&
           c->v_off_uv >= -CAL_OFF_MAX_UV && c->v_off_uv <= CAL_OFF_MAX_UV;
}

static esp_err_t save_cal(const cal_t *c)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open("battcal", NVS_READWRITE, &h);
    if (e != ESP_OK) {
        return e;
    }
    e = nvs_set_i32(h, "vg", c->v_gain_ppm);
    if (e == ESP_OK) {
        e = nvs_set_i32(h, "vo", c->v_off_uv);
    }
    if (e == ESP_OK) {
        e = nvs_set_i32(h, "ig", c->i_gain_ppm);
    }
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);
    return e;
}

esp_err_t cal_store_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_init: %s (usando padroes)", esp_err_to_name(e));
        return e;
    }
    nvs_handle_t h;
    if (nvs_open("battcal", NVS_READWRITE, &h) != ESP_OK) {
        return ESP_FAIL;
    }
    cal_t c = s_cal;
    int32_t v;
    if (nvs_get_i32(h, "vg", &v) == ESP_OK) { c.v_gain_ppm = v; }
    if (nvs_get_i32(h, "vo", &v) == ESP_OK) { c.v_off_uv = v; }
    if (nvs_get_i32(h, "ig", &v) == ESP_OK) { c.i_gain_ppm = v; }
    uint16_t tid = 0;
    if (nvs_get_u16(h, "tid", &tid) == ESP_OK) { s_test_id = tid; }
    nvs_close(h);
    if (cal_valid(&c)) {
        s_cal = c;
    } else {
        ESP_LOGW(TAG, "coeficientes salvos invalidos; usando padroes");
    }
    return ESP_OK;
}

const cal_t *cal_get(void)
{
    return &s_cal;
}

esp_err_t cal_set(const cal_t *c)
{
    if (!cal_valid(c)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t e = save_cal(c);
    if (e == ESP_OK) {
        s_cal = *c;
    }
    return e;
}

esp_err_t cal_reset(void)
{
    cal_t d = { CAL_GAIN_NOM_PPM, 0, CAL_GAIN_NOM_PPM };
    return cal_set(&d);
}

uint16_t cal_last_test_id(void)
{
    return s_test_id;
}

uint16_t cal_next_test_id(void)
{
    s_test_id++;
    if (s_test_id == 0) {
        s_test_id = 1;
    }
    nvs_handle_t h;
    if (nvs_open("battcal", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u16(h, "tid", s_test_id);
        nvs_commit(h);
        nvs_close(h);
    }
    return s_test_id;
}
