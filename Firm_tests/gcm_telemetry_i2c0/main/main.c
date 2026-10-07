/**
 * main.c — GCM-PI2-2026.2 telemetry (I2C0 only)
 *
 * Le os 4 canais do ADS1115 (U8) da GCM via I2C0 do ESP32
 * (SDA=GPIO21, SCL=GPIO22 -> nets SDA3V3/SCL3V3 -> level shifter BSN20 ->
 * ADS1115, endereco 0x49) e imprime tensao/corrente convertidas no serial
 * monitor.
 *
 * Usa SOMENTE o controlador I2C0 do ESP32-WROOM-32D. O I2C1 (nets SCL1/SDA1,
 * exposto no CN15 para o barramento externo/ToF) nao e tocado por este
 * codigo.
 *
 * Canais do ADS1115:
 *   AIN0 -> PWR_VOLTAGE_SENS    (divisor R8/R15 = 4,7k/1k, razao ~5,7 -> Vbatt)
 *   AIN1 -> PWR_CURRENT_SENS    (ACS758LCB-050B, +-50A, 40 mV/A, Vcc=5V)
 *   AIN2 -> LOGIC_VOLTAGE_SENSE (divisor R4/R6 = 4,7k/1k, razao ~5,7 -> Vlogic)
 *   AIN3 -> LOGIC_CURRENT_SENS  (ACS712-05B, +-5A, 185 mV/A, Vcc=5V)
 *
 * Toolchain: ESP-IDF v5.4.2, driver I2C master novo (driver/i2c_master.h).
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "GCM_TELEMETRY";

/* ---------- Pinagem / barramento I2C0 ---------- */
#define I2C0_SDA_GPIO       21   /* net SDA3V3 */
#define I2C0_SCL_GPIO       22   /* net SCL3V3 */
#define I2C0_FREQ_HZ        400000
#define ADS1115_ADDR        0x49 /* ADDR -> +5V (R14) */

/* ---------- Registradores ADS1115 ---------- */
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

/*
 * Config word (16 bits):
 *   OS=1 (dispara conversao) | MUX (canal single-ended) | PGA=000 (+-6,144V)
 *   | MODE=1 (single-shot) | DR=100 (128 SPS) | COMP_QUE=11 (comparador off)
 *
 * PGA em +-6,144V para cobrir a faixa de ate ~5,3V prevista no divisor
 * PWR_VOLTAGE_SENS (ver Descricao_de_hardware, secao 8).
 */
#define ADS1115_CFG_BASE     0x8183u  /* OS=1, PGA=000, MODE=1, DR=100, COMP_QUE=11 */
#define ADS1115_MUX_AIN0     (0x4u << 12)
#define ADS1115_MUX_AIN1     (0x5u << 12)
#define ADS1115_MUX_AIN2     (0x6u << 12)
#define ADS1115_MUX_AIN3     (0x7u << 12)

#define ADS1115_FSR_V        6.144f
#define ADS1115_LSB_V        (ADS1115_FSR_V / 32768.0f)

/* ---------- Escalas dos canais (ver Descricao_de_hardware) ---------- */
#define VOLTAGE_DIVIDER_RATIO   5.7f   /* (R8+R15)/R15 = (R4+R6)/R6 = 5,7 */

#define ACS_SUPPLY_V             5.0f  /* VCC dos ACS712/ACS758 = +5V */
#define ACS_MIDPOINT_V           (ACS_SUPPLY_V / 2.0f)

#define ACS712_SENSITIVITY_V_PER_A   0.185f  /* ACS712-05B, +-5A: 185 mV/A */
#define ACS758_SENSITIVITY_V_PER_A   0.040f  /* ACS758LCB-050B, +-50A bidirecional: 40 mV/A */

/* IMPORTANTE: o ultimo parametro de i2c_master_transmit/receive/transmit_receive
 * e "xfer_timeout_ms" em MILISSEGUNDOS PUROS, e nao ticks do FreeRTOS. Usar
 * pdMS_TO_TICKS() aqui (como na primeira versao deste arquivo) reduz o timeout
 * real para uma fracao do pretendido (com tick padrao de 100Hz, pdMS_TO_TICKS(100)
 * vira 10 -> timeout real de 10ms), causando "I2C software timeout" esporadico
 * assim que qualquer coisa (ex.: o proprio printf do log) atrasa o scheduler. */
#define I2C_XFER_TIMEOUT_MS   1000

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_ads1115 = NULL;

/* Tenta reinicializar o periferico I2C apos uma falha de transacao. O
 * ESP32 (silicio classico) tem um comportamento conhecido em que, apos um
 * timeout de hardware, a maquina de estados do I2C pode ficar presa ate
 * um reset explicito do barramento — sem isso, as proximas transacoes
 * continuam falhando mesmo com timeout correto. */
static void ads1115_recover_bus(void)
{
    esp_err_t err = i2c_master_bus_reset(s_bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_reset falhou: %s", esp_err_to_name(err));
    } else {
        ESP_LOGW(TAG, "Barramento I2C0 resetado apos falha de transacao");
    }
}

static esp_err_t ads1115_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    esp_err_t err = i2c_master_transmit(s_ads1115, buf, sizeof(buf), I2C_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ads1115_recover_bus();
    }
    return err;
}

static esp_err_t ads1115_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t rx[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(s_ads1115, &reg, 1, rx, sizeof(rx),
                                                 I2C_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ads1115_recover_bus();
        return err;
    }
    *value = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

/* Dispara uma conversao single-shot no canal indicado (mux ja deslocado para
 * bits 14:12) e espera o bit OS voltar a 1 (fim de conversao), com polling
 * por passos de 1ms e timeout de 50ms — mesmo padrao ja validado no
 * gcm_core_test, que evita o vTaskDelay(3ms) ser truncado para 0 tick. */
static esp_err_t ads1115_read_channel(uint16_t mux, float *out_volts)
{
    uint16_t cfg = ADS1115_CFG_BASE | mux;
    esp_err_t err = ads1115_write_reg(ADS1115_REG_CONFIG, cfg);
    if (err != ESP_OK) {
        return err;
    }

    const int step_ms = 1;
    const int timeout_ms = 50;
    int waited = 0;
    uint16_t status = 0;

    do {
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        waited += step_ms;
        err = ads1115_read_reg(ADS1115_REG_CONFIG, &status);
        if (err != ESP_OK) {
            return err;
        }
    } while (((status & 0x8000u) == 0) && (waited < timeout_ms));

    if ((status & 0x8000u) == 0) {
        ESP_LOGW(TAG, "Timeout aguardando conversao (mux=0x%04X)", mux);
        return ESP_ERR_TIMEOUT;
    }

    uint16_t raw;
    err = ads1115_read_reg(ADS1115_REG_CONVERSION, &raw);
    if (err != ESP_OK) {
        return err;
    }

    int16_t signed_raw = (int16_t)raw; /* entradas single-ended sao sempre >=0 */
    *out_volts = ((float)signed_raw) * ADS1115_LSB_V;
    return ESP_OK;
}

static void i2c0_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C0_SDA_GPIO,
        .scl_io_num = I2C0_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, /* pull-ups ja existem no hardware (R9-R12) */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = I2C0_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_ads1115));
}

void app_main(void)
{
    i2c0_init();
    ESP_LOGI(TAG, "I2C0 iniciado (SDA=%d, SCL=%d), ADS1115 @0x%02X",
             I2C0_SDA_GPIO, I2C0_SCL_GPIO, ADS1115_ADDR);

    while (1) {
        float v_ain0, v_ain1, v_ain2, v_ain3;
        esp_err_t e0 = ads1115_read_channel(ADS1115_MUX_AIN0, &v_ain0);
        esp_err_t e1 = ads1115_read_channel(ADS1115_MUX_AIN1, &v_ain1);
        esp_err_t e2 = ads1115_read_channel(ADS1115_MUX_AIN2, &v_ain2);
        esp_err_t e3 = ads1115_read_channel(ADS1115_MUX_AIN3, &v_ain3);

        if (e0 == ESP_OK && e1 == ESP_OK && e2 == ESP_OK && e3 == ESP_OK) {
            float pwr_voltage_batt = v_ain0 * VOLTAGE_DIVIDER_RATIO;               /* V */
            float pwr_current_ext  = (v_ain1 - ACS_MIDPOINT_V)
                                      / ACS758_SENSITIVITY_V_PER_A;                /* A */
            float logic_voltage    = v_ain2 * VOLTAGE_DIVIDER_RATIO;               /* V */
            float logic_current    = (v_ain3 - ACS_MIDPOINT_V)
                                      / ACS712_SENSITIVITY_V_PER_A;                /* A */

            printf("PWR_VOLTAGE_SENS (batt) : %6.3f V  (raw AIN0 = %6.3f V)\n",
                   pwr_voltage_batt, v_ain0);
            printf("PWR_CURRENT_SENS (ext)  : %6.3f A  (raw AIN1 = %6.3f V)\n",
                   pwr_current_ext, v_ain1);
            printf("LOGIC_VOLTAGE_SENSE     : %6.3f V  (raw AIN2 = %6.3f V)\n",
                   logic_voltage, v_ain2);
            printf("LOGIC_CURRENT_SENS      : %6.3f A  (raw AIN3 = %6.3f V)\n",
                   logic_current, v_ain3);
            printf("--------------------------------------------------------\n");
        } else {
            ESP_LOGE(TAG, "Falha na leitura: e0=%d e1=%d e2=%d e3=%d", e0, e1, e2, e3);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}