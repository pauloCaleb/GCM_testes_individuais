/*
 * Driver mínimo VL53L1X para ESP-IDF 5.4.2.
 *
 * A sequência de inicialização e os endereços de registro reproduzem o
 * comportamento documentado pela ST no UM2510 (VL53L1X ULD API) e replicado
 * de forma equivalente em diversos ports open-source independentes
 * (Pololu, SparkFun, ESPHome, mbed). Se o sensor se comportar de forma
 * inesperada, vale conferir esses endereços contra o UM2510 antes de mexer
 * na lógica.
 *
 * Não implementado:
 *   - calibração de offset e crosstalk
 *   - ROI customizada
 *   - modo por interrupção via pino GPIO1 (aqui só há polling por I2C)
 */
#include "vl53l1x.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "vl53l1x";

/* ---- Endereços de registro (16 bits) usados neste driver ---- */
#define REG_SOFT_RESET                              0x0000
#define REG_I2C_SLAVE_DEVICE_ADDRESS                0x0001
#define REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND     0x0008
#define REG_GPIO_HV_MUX_CTRL                        0x0030
#define REG_GPIO_TIO_HV_STATUS                      0x0031
#define REG_PHASECAL_CONFIG_TIMEOUT_MACROP          0x004B
#define REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI        0x005E
#define REG_RANGE_CONFIG_VCSEL_PERIOD_A             0x0060
#define REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI        0x0061
#define REG_RANGE_CONFIG_VCSEL_PERIOD_B             0x0063
#define REG_RANGE_CONFIG_VALID_PHASE_HIGH           0x0069
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD          0x006C   /* 32 bits */
#define REG_SD_CONFIG_WOI_SD0                       0x0078
#define REG_SD_CONFIG_INITIAL_PHASE_SD0             0x007A
#define REG_SYSTEM_INTERRUPT_CLEAR                  0x0086
#define REG_SYSTEM_MODE_START                       0x0087
#define REG_RESULT_RANGE_STATUS                     0x0089
#define REG_RESULT_PEAK_SIGNAL_RATE                 0x0098
#define REG_RESULT_FINAL_RANGE_MM                   0x0096
#define REG_RESULT_AMBIENT_RATE                     0x0090
#define REG_RESULT_OSC_CALIBRATE_VAL                0x00DE
#define REG_FIRMWARE_SYSTEM_STATUS                  0x00E5
#define REG_IDENTIFICATION_MODEL_ID                 0x010F

#define VL53L1X_MODEL_ID_EXPECTED                   0xEACC

#define I2C_TIMEOUT_MS 100

/* Com CONFIG_FREERTOS_HZ=100, pdMS_TO_TICKS(2) vale 0 e vTaskDelay(0) só
 * cede a CPU. Garante pelo menos 1 tick de espera nos laços de polling. */
static inline void delay_ms_min_tick(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    vTaskDelay(t > 0 ? t : 1);
}

/* Tabela de configuração de fábrica: registradores 0x2D..0x87 (91 bytes),
 * carregada de uma vez logo após o boot do sensor. */
static const uint8_t kDefaultConfig[] = {
    0x00, /* 0x2D */ 0x00, /* 0x2E */ 0x00, /* 0x2F */ 0x01, /* 0x30 */
    0x02, /* 0x31 */ 0x00, /* 0x32 */ 0x02, /* 0x33 */ 0x08, /* 0x34 */
    0x00, /* 0x35 */ 0x08, /* 0x36 */ 0x10, /* 0x37 */ 0x01, /* 0x38 */
    0x01, /* 0x39 */ 0x00, /* 0x3A */ 0x00, /* 0x3B */ 0x00, /* 0x3C */
    0x00, /* 0x3D */ 0xFF, /* 0x3E */ 0x00, /* 0x3F */ 0x0F, /* 0x40 */
    0x00, /* 0x41 */ 0x00, /* 0x42 */ 0x00, /* 0x43 */ 0x00, /* 0x44 */
    0x00, /* 0x45 */ 0x20, /* 0x46 */ 0x0B, /* 0x47 */ 0x00, /* 0x48 */
    0x00, /* 0x49 */ 0x02, /* 0x4A */ 0x0A, /* 0x4B */ 0x21, /* 0x4C */
    0x00, /* 0x4D */ 0x00, /* 0x4E */ 0x05, /* 0x4F */ 0x00, /* 0x50 */
    0x00, /* 0x51 */ 0x00, /* 0x52 */ 0x00, /* 0x53 */ 0xC8, /* 0x54 */
    0x00, /* 0x55 */ 0x00, /* 0x56 */ 0x38, /* 0x57 */ 0xFF, /* 0x58 */
    0x01, /* 0x59 */ 0x00, /* 0x5A */ 0x08, /* 0x5B */ 0x00, /* 0x5C */
    0x00, /* 0x5D */ 0x01, /* 0x5E */ 0xCC, /* 0x5F */ 0x0F, /* 0x60 */
    0x01, /* 0x61 */ 0xF1, /* 0x62 */ 0x0D, /* 0x63 */ 0x01, /* 0x64 */
    0x68, /* 0x65 */ 0x00, /* 0x66 */ 0x80, /* 0x67 */ 0x08, /* 0x68 */
    0xB8, /* 0x69 */ 0x00, /* 0x6A */ 0x00, /* 0x6B */ 0x00, /* 0x6C */
    0x00, /* 0x6D */ 0x0F, /* 0x6E */ 0x89, /* 0x6F */ 0x00, /* 0x70 */
    0x00, /* 0x71 */ 0x00, /* 0x72 */ 0x00, /* 0x73 */ 0x00, /* 0x74 */
    0x00, /* 0x75 */ 0x00, /* 0x76 */ 0x01, /* 0x77 */ 0x0F, /* 0x78 */
    0x0D, /* 0x79 */ 0x0E, /* 0x7A */ 0x0E, /* 0x7B */ 0x00, /* 0x7C */
    0x00, /* 0x7D */ 0x02, /* 0x7E */ 0xC7, /* 0x7F */ 0xFF, /* 0x80 */
    0x9B, /* 0x81 */ 0x00, /* 0x82 */ 0x00, /* 0x83 */ 0x00, /* 0x84 */
    0x01, /* 0x85 */ 0x00, /* 0x86 : SYSTEM__INTERRUPT_CLEAR */
    0x00, /* 0x87 : SYSTEM__MODE_START (fica parado até start_ranging) */
};
#define DEFAULT_CONFIG_FIRST_REG 0x2D

/* Tabela de conversão do RangeStatus bruto do sensor para o código
 * "amigável" que a maioria das libs (Pololu/ULD API) reporta. 255 = reservado. */
static const uint8_t kRangeStatusMap[24] = {
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255,
    9,   13,  255, 255, 255, 255, 10, 6, 255, 255, 11, 12
};

/* ---------------- helpers de I2C ---------------- */

static esp_err_t reg_write_bytes(vl53l1x_t *dev, uint16_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[2 + 32];
    if (len > sizeof(buf) - 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    memcpy(&buf[2], data, len);
    return i2c_master_transmit(dev->i2c_dev, buf, 2 + len, I2C_TIMEOUT_MS);
}

static esp_err_t reg_write8(vl53l1x_t *dev, uint16_t reg, uint8_t val)
{
    return reg_write_bytes(dev, reg, &val, 1);
}

static esp_err_t reg_write16(vl53l1x_t *dev, uint16_t reg, uint16_t val)
{
    uint8_t data[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return reg_write_bytes(dev, reg, data, 2);
}

static esp_err_t reg_write32(vl53l1x_t *dev, uint16_t reg, uint32_t val)
{
    uint8_t data[4] = {
        (uint8_t)(val >> 24), (uint8_t)(val >> 16),
        (uint8_t)(val >> 8),  (uint8_t)(val & 0xFF)
    };
    return reg_write_bytes(dev, reg, data, 4);
}

static esp_err_t reg_read_bytes(vl53l1x_t *dev, uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    return i2c_master_transmit_receive(dev->i2c_dev, addr, sizeof(addr),
                                        data, len, I2C_TIMEOUT_MS);
}

static esp_err_t reg_read8(vl53l1x_t *dev, uint16_t reg, uint8_t *val)
{
    return reg_read_bytes(dev, reg, val, 1);
}

static esp_err_t reg_read16(vl53l1x_t *dev, uint16_t reg, uint16_t *val)
{
    uint8_t data[2];
    esp_err_t err = reg_read_bytes(dev, reg, data, 2);
    if (err == ESP_OK) {
        *val = ((uint16_t)data[0] << 8) | data[1];
    }
    return err;
}

/* ---------------- API pública ---------------- */

esp_err_t vl53l1x_init_bus_device(i2c_master_bus_handle_t bus_handle,
                                   uint8_t i2c_addr,
                                   vl53l1x_t *dev)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->i2c_dev);
    if (err == ESP_OK) {
        dev->bus = bus_handle;
        dev->addr = i2c_addr;
    }
    return err;
}

esp_err_t vl53l1x_deinit_bus_device(vl53l1x_t *dev)
{
    if (dev == NULL || dev->i2c_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = i2c_master_bus_rm_device(dev->i2c_dev);
    dev->i2c_dev = NULL;
    return err;
}

esp_err_t vl53l1x_set_i2c_address(vl53l1x_t *dev, uint8_t new_addr)
{
    if (dev == NULL || dev->i2c_dev == NULL || new_addr > 0x7F) {
        return ESP_ERR_INVALID_ARG;
    }
    if (new_addr == dev->addr) {
        return ESP_OK;
    }

    /* O sensor responde no novo endereço logo após esta escrita. */
    esp_err_t err = reg_write8(dev, REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    if (err != ESP_OK) {
        return err;
    }

    i2c_master_bus_handle_t bus = dev->bus;
    err = vl53l1x_deinit_bus_device(dev);
    if (err != ESP_OK) {
        return err;
    }
    return vl53l1x_init_bus_device(bus, new_addr, dev);
}

esp_err_t vl53l1x_get_sensor_id(vl53l1x_t *dev, uint16_t *model_id)
{
    return reg_read16(dev, REG_IDENTIFICATION_MODEL_ID, model_id);
}

static esp_err_t wait_for_boot(vl53l1x_t *dev, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    uint8_t state = 0;
    while (1) {
        esp_err_t err = reg_read8(dev, REG_FIRMWARE_SYSTEM_STATUS, &state);
        if (err == ESP_OK && (state & 0x01)) {
            return ESP_OK;
        }
        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        delay_ms_min_tick(2);
    }
}

esp_err_t vl53l1x_sensor_init(vl53l1x_t *dev)
{
    esp_err_t err;

    /* Sensor pode levar alguns ms para ficar pronto após power-on/reset. */
    err = wait_for_boot(dev, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timeout esperando boot do sensor (verifique fiação/alimentação)");
        return err;
    }

    uint16_t model_id;
    err = vl53l1x_get_sensor_id(dev, &model_id);
    if (err != ESP_OK) {
        return err;
    }
    if (model_id != VL53L1X_MODEL_ID_EXPECTED) {
        ESP_LOGW(TAG, "model ID lido 0x%04X, esperado 0x%04X (endereco/fiacao ok?)",
                 model_id, VL53L1X_MODEL_ID_EXPECTED);
    }

    /* Carrega a configuração padrão de fábrica, registrador a registrador. */
    for (size_t i = 0; i < sizeof(kDefaultConfig); i++) {
        err = reg_write8(dev, DEFAULT_CONFIG_FIRST_REG + i, kDefaultConfig[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    /* Dispara uma medição "descartável" para completar a calibração inicial
     * de VHV, igual à sequência oficial do ULD API. */
    err = vl53l1x_start_ranging(dev);
    if (err != ESP_OK) return err;

    bool ready = false;
    TickType_t start = xTaskGetTickCount();
    while (!ready) {
        err = vl53l1x_check_data_ready(dev, &ready);
        if (err != ESP_OK) return err;
        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > 1000) {
            return ESP_ERR_TIMEOUT;
        }
        delay_ms_min_tick(2);
    }
    err = vl53l1x_clear_interrupt(dev);
    if (err != ESP_OK) return err;
    err = vl53l1x_stop_ranging(dev);
    if (err != ESP_OK) return err;

    err = reg_write8(dev, REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    if (err != ESP_OK) return err;
    /* Começa VHV a partir da temperatura anterior (registrador interno, sem nome oficial). */
    err = reg_write8(dev, 0x000B, 0x00);
    return err;
}

esp_err_t vl53l1x_start_ranging(vl53l1x_t *dev)
{
    return reg_write8(dev, REG_SYSTEM_MODE_START, 0x40);
}

esp_err_t vl53l1x_stop_ranging(vl53l1x_t *dev)
{
    return reg_write8(dev, REG_SYSTEM_MODE_START, 0x00);
}

esp_err_t vl53l1x_clear_interrupt(vl53l1x_t *dev)
{
    return reg_write8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

static esp_err_t get_interrupt_polarity(vl53l1x_t *dev, uint8_t *pol)
{
    uint8_t tmp;
    esp_err_t err = reg_read8(dev, REG_GPIO_HV_MUX_CTRL, &tmp);
    if (err != ESP_OK) return err;
    tmp = (tmp & 0x10) >> 4;
    *pol = 1 - tmp;
    return ESP_OK;
}

esp_err_t vl53l1x_check_data_ready(vl53l1x_t *dev, bool *ready)
{
    uint8_t pol, status;
    esp_err_t err = get_interrupt_polarity(dev, &pol);
    if (err != ESP_OK) return err;
    err = reg_read8(dev, REG_GPIO_TIO_HV_STATUS, &status);
    if (err != ESP_OK) return err;
    *ready = ((status & 0x01) == pol);
    return ESP_OK;
}

esp_err_t vl53l1x_get_result(vl53l1x_t *dev, vl53l1x_result_t *result)
{
    esp_err_t err;
    uint8_t raw_status;

    err = reg_read8(dev, REG_RESULT_RANGE_STATUS, &raw_status);
    if (err != ESP_OK) return err;
    raw_status &= 0x1F;
    result->range_status = (raw_status < 24) ? kRangeStatusMap[raw_status] : 255;

    err = reg_read16(dev, REG_RESULT_FINAL_RANGE_MM, &result->distance_mm);
    if (err != ESP_OK) return err;

    err = reg_read16(dev, REG_RESULT_PEAK_SIGNAL_RATE, &result->signal_rate_mcps);
    if (err != ESP_OK) return err;

    err = reg_read16(dev, REG_RESULT_AMBIENT_RATE, &result->ambient_rate_mcps);
    return err;
}

esp_err_t vl53l1x_wait_for_result(vl53l1x_t *dev, vl53l1x_result_t *result, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    bool ready = false;
    while (!ready) {
        esp_err_t err = vl53l1x_check_data_ready(dev, &ready);
        if (err != ESP_OK) return err;
        if (!ready) {
            if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > timeout_ms) {
                return ESP_ERR_TIMEOUT;
            }
            delay_ms_min_tick(2);
        }
    }
    esp_err_t err = vl53l1x_get_result(dev, result);
    if (err != ESP_OK) return err;
    return vl53l1x_clear_interrupt(dev);
}

esp_err_t vl53l1x_set_distance_mode(vl53l1x_t *dev, vl53l1x_distance_mode_t mode)
{
    esp_err_t err;
    if (mode == VL53L1X_DISTANCE_SHORT) {
        err = reg_write8(dev, REG_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14);
        if (err) return err;
        err = reg_write8(dev, REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x07);
        if (err) return err;
        err = reg_write8(dev, REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x05);
        if (err) return err;
        err = reg_write8(dev, REG_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        if (err) return err;
        err = reg_write16(dev, REG_SD_CONFIG_WOI_SD0, 0x0705);
        if (err) return err;
        err = reg_write16(dev, REG_SD_CONFIG_INITIAL_PHASE_SD0, 0x0606);
    } else {
        err = reg_write8(dev, REG_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A);
        if (err) return err;
        err = reg_write8(dev, REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);
        if (err) return err;
        err = reg_write8(dev, REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);
        if (err) return err;
        err = reg_write8(dev, REG_RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8);
        if (err) return err;
        err = reg_write16(dev, REG_SD_CONFIG_WOI_SD0, 0x0F0D);
        if (err) return err;
        err = reg_write16(dev, REG_SD_CONFIG_INITIAL_PHASE_SD0, 0x0E0E);
    }
    return err;
}

/* Pares (timing budget em ms -> valores de TIMEOUT_MACROP_A/B) para modo LONGO.
 * Só estes valores discretos são suportados, igual à API oficial da ST. */
typedef struct { uint16_t budget_ms; uint16_t a; uint16_t b; } budget_entry_t;

static const budget_entry_t kBudgetLong[] = {
    { 20,  0x001E, 0x0022 },
    { 33,  0x0060, 0x006E },
    { 50,  0x00AD, 0x00C6 },
    { 100, 0x01CC, 0x01EA },
    { 200, 0x02D9, 0x02F8 },
    { 500, 0x048F, 0x04A4 },
};

static const budget_entry_t kBudgetShort[] = {
    { 15,  0x01D,  0x0027 }, /* só disponível no modo curto */
    { 20,  0x0051, 0x006E },
    { 33,  0x00D6, 0x006E },
    { 50,  0x01AE, 0x01E8 },
    { 100, 0x02E1, 0x0388 },
    { 200, 0x03E1, 0x0496 },
    { 500, 0x0591, 0x05C1 },
};

esp_err_t vl53l1x_set_timing_budget_ms(vl53l1x_t *dev, uint16_t budget_ms)
{
    /* Descobre o modo atual olhando o VCSEL period A já configurado
     * (0x07 -> curto, 0x0F -> longo, default de fábrica é longo). */
    uint8_t vcsel_a;
    esp_err_t err = reg_read8(dev, REG_RANGE_CONFIG_VCSEL_PERIOD_A, &vcsel_a);
    if (err != ESP_OK) return err;

    const budget_entry_t *table = (vcsel_a == 0x07) ? kBudgetShort : kBudgetLong;
    size_t table_len = (vcsel_a == 0x07) ? (sizeof(kBudgetShort) / sizeof(kBudgetShort[0]))
                                          : (sizeof(kBudgetLong) / sizeof(kBudgetLong[0]));

    for (size_t i = 0; i < table_len; i++) {
        if (table[i].budget_ms == budget_ms) {
            err = reg_write16(dev, REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, table[i].a);
            if (err != ESP_OK) return err;
            return reg_write16(dev, REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, table[i].b);
        }
    }
    ESP_LOGE(TAG, "timing budget %u ms nao suportado nesse modo", budget_ms);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t vl53l1x_set_inter_measurement_ms(vl53l1x_t *dev, uint16_t period_ms)
{
    uint16_t clock_pll;
    esp_err_t err = reg_read16(dev, REG_RESULT_OSC_CALIBRATE_VAL, &clock_pll);
    if (err != ESP_OK) return err;
    clock_pll &= 0x3FF;
    uint32_t value = (uint32_t)(clock_pll * (float)period_ms * 1.075f);
    return reg_write32(dev, REG_SYSTEM_INTERMEASUREMENT_PERIOD, value);
}

const char *vl53l1x_range_status_str(uint8_t status)
{
    switch (status) {
        case 0:  return "range valido";
        case 1:  return "sigma acima do limite";
        case 2:  return "taxa de sinal acima do limite";
        case 3:  return "min range clipping";
        case 4:  return "sinal fraco (out of bounds)";
        case 5:  return "sem sinal detectado (fase invalida)";
        case 6:  return "erro de hardware";
        case 7:  return "warmup / recalibracao interna";
        case 9:  return "vizinhanca de merged pulse";
        case 10: return "sinal muito fraco";
        case 11: return "sinal isolado (interference)";
        case 12: return "erro de intervalo entre medicoes";
        case 13: return "erro de calibracao interna (nao usar leitura)";
        default: return "status desconhecido/reservado";
    }
}
