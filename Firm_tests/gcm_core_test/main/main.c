/*
 * GCM-PI2-2026.2 — Firmware de bring-up do "Logic Core"
 * ------------------------------------------------------
 * Objetivo: validar o core da placa (sem o TJA1050 populado) antes dos
 * testes de CAN em outro projeto.
 *
 *  1) Le continuamente as 4 tensoes do ADS1115 (AIN0..AIN3) e envia via
 *     serial (USB isolado -> CH340 -> UART0).
 *  2) Amostra os dois GPIOs input-only (GPIO34/GPIO35) e envia o estado.
 *  3) Aciona sequencialmente todos os demais GPIOs de saida expostos no
 *     CN15 (um "chase" com LED em cada saida), para confirmar que cada
 *     trilha/pino esta funcional.
 *
 * Mapeamento de pinos (conferido diretamente no esquematico
 * GCM-PI2-2026.2-211062820, folha Logic_core — pontos de juncao, nao
 * proximidade de trilhas):
 *
 *   Sinal (net)          GPIO   CN15   Observacao
 *   -------------------- ------ ------ ------------------------------
 *   HC595_DS             25     4      shift register
 *   HC595_CLK            26     17     shift register
 *   HC595_LATCH          27     5      shift register
 *   DIR_2                14     18     motor/atuador
 *   DIR_1                17     6      motor/atuador
 *   EN_ALL               16     19     motor/atuador
 *   PWM1                 13     7      motor/atuador
 *   PWM2                 4      20     motor/atuador
 *   GPIO23                23     8      uso geral
 *   CANL/GPIO32           32     12     GPIO puro (R40-R43 = 0R, TJA1050 NAO populado)
 *   CANH/GPIO33            33     13     GPIO puro (R40-R43 = 0R, TJA1050 NAO populado)
 *   GPIO34 (input-only)   34     21     pull-up externo R22, ja na placa
 *   GPIO35 (input-only)   35     9      pull-up externo R23, ja na placa
 *   SDA3V3 (I2C p/ ADS1115) 21   -      level-shiftado (Q2/Q3) p/ 5V
 *   SCL3V3 (I2C p/ ADS1115) 22   -      level-shiftado (Q2/Q3) p/ 5V
 *
 * OBS: SDA1/SCL1 (GPIO18/GPIO19, CN15 pinos 16/3) entram na varredura
 * de saidas como teste ELETRICO de continuidade ate o conector — o
 * firmware nunca fala protocolo I2C1 neles, so aciona/desaciona como
 * GPIO digital comum (vence os pull-ups de 3,3kOhm R26/R27).
 *
 * Canais do ADS1115 (endereco 0x49, ADDR->+5V, confirmado no doc):
 *   AIN0 -> PWR_VOLTAGE_SENS
 *   AIN1 -> PWR_CURRENT_SENS
 *   AIN2 -> LOGIC_VOLTAGE_SENSE
 *   AIN3 -> LOGIC_CURRENT_SENS
 *
 * PGA usado: +-6.144V (2/3x) — cobre 0-5V de qualquer um dos 4 sinais
 * (todos os dividores/sensores da placa saem no maximo perto de 5V).
 *
 * --------------------------------------------------------------------
 * FIX (2026-08-29): leitura do ADS1115 trocada de "delay fixo + read"
 * para "poll do bit OS no registro de config + read". O delay fixo de
 * 3ms usava vTaskDelay(pdMS_TO_TICKS(3)), que trunca para 0 ticks no
 * CONFIG_FREERTOS_HZ=100 padrao do IDF (tick de 10ms) — o tempo real de
 * espera variava entre ~0 e ~10ms de forma nao deterministica. Quando o
 * delay efetivo caia abaixo do tempo de conversao (~1,16ms a 860 SPS),
 * a leitura pegava o valor da conversao ANTERIOR ainda latched no
 * registro, causando "vazamento" de valores entre canais adjacentes.
 * O polling do bit OS (1 = conversao concluida) elimina essa race
 * condition de forma determinista, independente do tick rate ou do DR
 * escolhido.
 *
 * Build:
 *   idf.py set-target esp32
 *   idf.py build flash monitor
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GCM_CORE_TEST";

/* ------------------------- I2C / ADS1115 ------------------------- */

#define I2C_SDA_GPIO   GPIO_NUM_21     /* SDA3V3 */
#define I2C_SCL_GPIO   GPIO_NUM_22     /* SCL3V3 */
#define I2C_FREQ_HZ    100000
#define ADS1115_ADDR   0x49

#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01

/* LSB para PGA = +-6.144V (2/3x) */
#define ADS1115_LSB_VOLT (6.144f / 32768.0f)

/* Taxa de amostragem do ADS1115: 860 SPS (mais rapido, suficiente p/ bring-up) */
#define ADS1115_DR_FIELD 0x7

/* Polling do bit OS (conversao concluida) em vez de delay fixo — ver
 * nota de FIX no cabecalho do arquivo. */
#define ADS1115_POLL_STEP_MS   1
#define ADS1115_POLL_TIMEOUT   50   /* ~50ms de guarda, folgado sobre os ~1,16ms @ 860 SPS */

typedef enum {
    ADS_CH_AIN0 = 0, /* PWR_VOLTAGE_SENS    */
    ADS_CH_AIN1 = 1, /* PWR_CURRENT_SENS    */
    ADS_CH_AIN2 = 2, /* LOGIC_VOLTAGE_SENSE */
    ADS_CH_AIN3 = 3, /* LOGIC_CURRENT_SENS  */
    ADS_CH_COUNT = 4,
} ads_channel_t;

static const char *g_ads_names[ADS_CH_COUNT] = {
    "PWR_VOLTAGE_SENS",
    "PWR_CURRENT_SENS",
    "LOGIC_VOLTAGE_SENSE",
    "LOGIC_CURRENT_SENS",
};

static i2c_master_bus_handle_t g_i2c_bus;
static i2c_master_dev_handle_t g_ads1115;

/* ------------------------------ GPIO ------------------------------ */

#define GPIO_INPUT_34 GPIO_NUM_34
#define GPIO_INPUT_35 GPIO_NUM_35

typedef struct {
    gpio_num_t  pin;
    const char *net_name;
} gpio_out_t;

static const gpio_out_t g_outputs[] = {
    { GPIO_NUM_25, "HC595_DS"     },
    { GPIO_NUM_26, "HC595_CLK"    },
    { GPIO_NUM_27, "HC595_LATCH"  },
    { GPIO_NUM_14, "DIR_2"        },
    { GPIO_NUM_17, "DIR_1"        },
    { GPIO_NUM_16, "EN_ALL"       },
    { GPIO_NUM_13, "PWM1"         },
    { GPIO_NUM_4,  "PWM2"         },
    { GPIO_NUM_23, "GPIO23"       },
    { GPIO_NUM_32, "CANL/GPIO32"  }, /* requer R40-R43=0R e TJA1050 NAO populado */
    { GPIO_NUM_33, "CANH/GPIO33"  }, /* requer R40-R43=0R e TJA1050 NAO populado */
    { GPIO_NUM_19, "SCL1"         }, /* teste eletrico de continuidade ate CN15 pino 3 */
    { GPIO_NUM_18, "SDA1"         }, /* teste eletrico de continuidade ate CN15 pino 16 */
};
#define NUM_OUTPUTS (sizeof(g_outputs) / sizeof(g_outputs[0]))

/* Quantas leituras de status sao impressas com cada saida ligada, e
 * o intervalo entre elas (ex.: 4 * 500ms = 2s de LED aceso por vez). */
#define SAMPLES_PER_OUTPUT   4
#define SAMPLE_PERIOD_MS     500

/* --------------------------------------------------------------------- */

static void i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, /* pull-ups dedicados ja existem no level shifter */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &g_i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ADS1115_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, &g_ads1115));
}

static esp_err_t ads1115_write_config(uint16_t cfg)
{
    uint8_t buf[3] = { ADS1115_REG_CONFIG, (uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF) };
    return i2c_master_transmit(g_ads1115, buf, sizeof(buf), 100 /* ms */);
}

static esp_err_t ads1115_read_config(uint16_t *out_cfg)
{
    uint8_t reg = ADS1115_REG_CONFIG;
    uint8_t rx[2];
    esp_err_t err = i2c_master_transmit_receive(g_ads1115, &reg, 1, rx, sizeof(rx), 100 /* ms */);
    if (err != ESP_OK) {
        return err;
    }
    *out_cfg = (uint16_t)((rx[0] << 8) | rx[1]);
    return ESP_OK;
}

static esp_err_t ads1115_read_conversion(int16_t *out_raw)
{
    uint8_t reg = ADS1115_REG_CONVERSION;
    uint8_t rx[2];
    esp_err_t err = i2c_master_transmit_receive(g_ads1115, &reg, 1, rx, sizeof(rx), 100 /* ms */);
    if (err != ESP_OK) {
        return err;
    }
    *out_raw = (int16_t)((rx[0] << 8) | rx[1]);
    return ESP_OK;
}

static esp_err_t ads1115_read_single_ended(ads_channel_t ch, float *out_volts)
{
    /* MUX single-ended: 100=AIN0/GND, 101=AIN1/GND, 110=AIN2/GND, 111=AIN3/GND */
    uint16_t mux = (uint16_t)(0x4 + ch) & 0x7;

    uint16_t cfg = 0;
    cfg |= (1u << 15);              /* OS = 1 -> inicia conversao single-shot   */
    cfg |= (mux << 12);             /* MUX                                      */
    cfg |= (0x0u << 9);             /* PGA = 000 -> +-6.144V                    */
    cfg |= (1u << 8);               /* MODE = 1 -> single-shot                  */
    cfg |= (ADS1115_DR_FIELD << 5); /* DR = 111 -> 860 SPS                      */
    cfg |= (0x3u << 0);             /* COMP_QUE = 11 -> comparador desabilitado */

    esp_err_t err = ads1115_write_config(cfg);
    if (err != ESP_OK) {
        return err;
    }

    /* Poll do bit OS (bit 15) em vez de delay fixo: OS=1 = conversao
     * concluida. Ver nota de FIX no cabecalho do arquivo. */
    uint16_t status = 0;
    int tries = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(ADS1115_POLL_STEP_MS));
        err = ads1115_read_config(&status);
        if (err != ESP_OK) {
            return err;
        }
        tries++;
    } while (!(status & 0x8000) && tries < ADS1115_POLL_TIMEOUT);

    if (!(status & 0x8000)) {
        ESP_LOGW(TAG, "Timeout aguardando conversao do ADS1115 (canal %d)", ch);
        return ESP_ERR_TIMEOUT;
    }

    int16_t raw;
    err = ads1115_read_conversion(&raw);
    if (err != ESP_OK) {
        return err;
    }

    *out_volts = raw * ADS1115_LSB_VOLT;
    return ESP_OK;
}

static void gpio_init_all(void)
{
    /* Entradas input-only — pull-up ja existe na placa (R22/R23), nao ha
     * pull interno disponivel de qualquer forma para GPIO34/35. */
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << GPIO_INPUT_34) | (1ULL << GPIO_INPUT_35),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));

    uint64_t out_mask = 0;
    for (size_t i = 0; i < NUM_OUTPUTS; i++) {
        out_mask |= (1ULL << g_outputs[i].pin);
    }
    gpio_config_t out_cfg = {
        .pin_bit_mask = out_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    for (size_t i = 0; i < NUM_OUTPUTS; i++) {
        gpio_set_level(g_outputs[i].pin, 0);
    }
}

/* --------------------------------------------------------------------- */

static void print_header(void)
{
    printf("t_ms;active_output;GPIO34;GPIO35;"
           "PWR_VOLTAGE_SENS_V;PWR_CURRENT_SENS_V;"
           "LOGIC_VOLTAGE_SENSE_V;LOGIC_CURRENT_SENS_V\n");
}

static void print_status_line(const char *active_output)
{
    int64_t t_ms = esp_timer_get_time() / 1000;

    int in34 = gpio_get_level(GPIO_INPUT_34);
    int in35 = gpio_get_level(GPIO_INPUT_35);

    float volts[ADS_CH_COUNT] = { 0 };
    char  cell[ADS_CH_COUNT][8];

    for (int ch = 0; ch < ADS_CH_COUNT; ch++) {
        esp_err_t err = ads1115_read_single_ended((ads_channel_t)ch, &volts[ch]);
        if (err == ESP_OK) {
            snprintf(cell[ch], sizeof(cell[ch]), "%.3f", volts[ch]);
        } else {
            snprintf(cell[ch], sizeof(cell[ch]), "ERR");
            ESP_LOGW(TAG, "Falha lendo %s: %s", g_ads_names[ch], esp_err_to_name(err));
        }
    }

    printf("%lld;%s;%d;%d;%s;%s;%s;%s\n",
           (long long)t_ms,
           active_output ? active_output : "-",
           in34, in35,
           cell[0], cell[1], cell[2], cell[3]);
}

/* --------------------------------------------------------------------- */

void app_main(void)
{
    gpio_init_all();
    i2c_bus_init();

    ESP_LOGI(TAG, "GCM core test iniciado: %d saidas sequenciais, "
                  "2 entradas input-only, ADS1115 @ 0x%02X",
             (int)NUM_OUTPUTS, ADS1115_ADDR);
    ESP_LOGI(TAG, "TJA1050 assumido NAO populado (R40-R43 em 0R p/ GPIO32/33 diretos)");
    ESP_LOGI(TAG, "SDA1/SCL1 incluidos na varredura como teste eletrico (nao usam protocolo I2C1)");
    ESP_LOGI(TAG, "Leitura do ADS1115 via polling do bit OS (sem delay fixo)");

    print_header();

    while (1) {
        for (size_t i = 0; i < NUM_OUTPUTS; i++) {
            gpio_set_level(g_outputs[i].pin, 1);

            for (int s = 0; s < SAMPLES_PER_OUTPUT; s++) {
                print_status_line(g_outputs[i].net_name);
                vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            }

            gpio_set_level(g_outputs[i].pin, 0);
        }
    }
}