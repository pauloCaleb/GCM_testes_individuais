/**
 * @file main.c
 * @brief GCM-PI2-2026.2 - teste de 3x VL53L1X no I2C1 com XSHUT via PCA9554A.
 *
 * Sequencia de boot:
 *   1. Sobe o I2C1 (SDA=GPIO18, SCL=GPIO19) e anexa o PCA9554A (0x38).
 *   2. Configura o PCA9554A: XSHUT (IO0..IO2) em LOW e LEDs (IO7/IO6/IO5)
 *      apagados ANTES de virarem saida (nao ha glitch no XSHUT/LED).
 *   3. Teste dos LEDs: acende cada LED sozinho por 1 s; depois deixa um
 *      deles aceso fixo (LED_FIXED_PIN).
 *   4. Atribuicao de enderecos: com todos os XSHUT em LOW, libera um sensor
 *      por vez (todos nascem em 0x29), troca o endereco dele via registrador
 *      0x0001 e so entao libera o proximo.
 *   5. Configura e inicia o ranging continuo de cada sensor.
 *   6. Loop de polling dos tres sensores, imprimindo as leituras no serial.
 *
 * Hardware (PCA9554A):
 *   IO0 -> XSHUT sensor 1     IO5 -> LED (terceiro)
 *   IO1 -> XSHUT sensor 2     IO6 -> LED (segundo)
 *   IO2 -> XSHUT sensor 3     IO7 -> LED (primeiro)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "pca9554.h"
#include "vl53l1x.h"

static const char *TAG = "gcm_tof";

/* ------------------------------------------------------------------ */
/* Configuracao                                                        */
/* ------------------------------------------------------------------ */

/* I2C1 da GCM (nets SCL1/SDA1, CN15) */
#define I2C1_SDA_GPIO        GPIO_NUM_18
#define I2C1_SCL_GPIO        GPIO_NUM_19
#define I2C1_PORT            I2C_NUM_1

/* PCA9554A com A2=A1=A0=GND -> 0x38 (ver pca9554_test). */
#define PCA9554_I2C_ADDR     0x38

/* Pinos do PCA9554A */
#define PCA_PIN_XSHUT_S1     0
#define PCA_PIN_XSHUT_S2     1
#define PCA_PIN_XSHUT_S3     2
#define PCA_PIN_LED_1        7
#define PCA_PIN_LED_2        6
#define PCA_PIN_LED_3        5

/* Sinal da fita: 1 = nivel alto liga o LED. Mude para 0 se for ativo em baixo. */
#define LED_ACTIVE_HIGH      1
#define LED_ON_TIME_MS       1000
/* LED que fica aceso fixo depois do teste (um dos PCA_PIN_LED_x). */
#define LED_FIXED_PIN        PCA_PIN_LED_2

/* Enderecos I2C finais dos sensores. Evitam 0x29 (padrao do VL53L1X) e toda a
 * faixa do PCA9554 (0x20-0x27) e do PCA9554A (0x38-0x3F). */
#define TOF_ADDR_S1          0x30
#define TOF_ADDR_S2          0x31
#define TOF_ADDR_S3          0x32

/* Parametros de medicao (iguais ao projeto de referencia vl53l1x_esp_idf).
 * Budgets validos: LONG 20/33/50/100/200/500 ms; SHORT 15/20/33/50/100/200/500 ms.
 * O periodo entre medicoes deve ser >= timing budget. */
#define TOF_DISTANCE_MODE    VL53L1X_DISTANCE_LONG
#define TOF_TIMING_BUDGET_MS 100
#define TOF_INTER_MEAS_MS    200

/* Loop de polling */
#define POLL_PERIOD_MS       5
#define PRINT_PERIOD_MS      200

/* ------------------------------------------------------------------ */
/* Estado                                                              */
/* ------------------------------------------------------------------ */

#define N_SENSORS 3

typedef enum {
    TOF_OFFLINE = 0,   /* nao respondeu / falhou na inicializacao */
    TOF_ADDRESSED,     /* endereco unico atribuido, ainda nao configurado */
    TOF_RANGING,       /* ranging continuo ativo */
} tof_state_t;

typedef struct {
    const char       *name;
    uint8_t           xshut_pin;   /* pino do PCA9554A */
    uint8_t           addr;        /* endereco I2C final */
    tof_state_t       state;
    vl53l1x_t         dev;
    vl53l1x_result_t  last;
    bool              has_result;
    uint32_t          i2c_errors;
} tof_t;

static tof_t s_tof[N_SENSORS] = {
    { .name = "S1", .xshut_pin = PCA_PIN_XSHUT_S1, .addr = TOF_ADDR_S1 },
    { .name = "S2", .xshut_pin = PCA_PIN_XSHUT_S2, .addr = TOF_ADDR_S2 },
    { .name = "S3", .xshut_pin = PCA_PIN_XSHUT_S3, .addr = TOF_ADDR_S3 },
};

static const uint8_t s_led_pins[3] = { PCA_PIN_LED_1, PCA_PIN_LED_2, PCA_PIN_LED_3 };

static i2c_master_bus_handle_t s_bus;
static pca9554_t s_pca;

/* Mascaras do PCA9554A */
#define XSHUT_MASK ((uint8_t)((1u << PCA_PIN_XSHUT_S1) | (1u << PCA_PIN_XSHUT_S2) | (1u << PCA_PIN_XSHUT_S3)))
#define LED_MASK   ((uint8_t)((1u << PCA_PIN_LED_1) | (1u << PCA_PIN_LED_2) | (1u << PCA_PIN_LED_3)))
/* Configuration register: 0 = saida. IO3/IO4 nao sao usados e ficam como entrada. */
#define PCA_CONFIG_VALUE ((uint8_t)(~(XSHUT_MASK | LED_MASK)))
/* Estado inicial do Output Port: XSHUT em LOW, LEDs desligados. */
#define PCA_OUTPUT_IDLE  ((uint8_t)(LED_ACTIVE_HIGH ? 0x00 : LED_MASK))

/* ------------------------------------------------------------------ */
/* Barramento e expansor                                               */
/* ------------------------------------------------------------------ */

static void i2c1_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C1_PORT,
        .sda_io_num        = I2C1_SDA_GPIO,
        .scl_io_num        = I2C1_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
    ESP_LOGI(TAG, "I2C1 inicializado (SDA=GPIO%d, SCL=GPIO%d)", (int)I2C1_SDA_GPIO, (int)I2C1_SCL_GPIO);
}

static esp_err_t expander_init(void)
{
    esp_err_t err = i2c_master_probe(s_bus, PCA9554_I2C_ADDR, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9554A nao respondeu em 0x%02X (%s). Confira o endereco e o I2C1.",
                 PCA9554_I2C_ADDR, esp_err_to_name(err));
        return err;
    }

    err = pca9554_init(s_bus, PCA9554_I2C_ADDR, &s_pca);
    if (err != ESP_OK) return err;

    /* Ordem importa: o Output Port e escrito enquanto os pinos ainda sao
     * entradas (default de POR); so depois o Configuration vira saida. Assim
     * XSHUT e LEDs ja nascem no nivel correto, sem pulso indevido. */
    err = pca9554_write_output_port(&s_pca, PCA_OUTPUT_IDLE);
    if (err != ESP_OK) return err;
    err = pca9554_write_config(&s_pca, PCA_CONFIG_VALUE);
    if (err != ESP_OK) return err;

    uint8_t cfg = 0xFF, out = 0xFF;
    ESP_ERROR_CHECK(pca9554_read_config(&s_pca, &cfg));
    ESP_ERROR_CHECK(pca9554_read_output_port(&s_pca, &out));
    ESP_LOGI(TAG, "PCA9554A: Config=0x%02X (esperado 0x%02X), Output=0x%02X (esperado 0x%02X)",
             cfg, PCA_CONFIG_VALUE, out, PCA_OUTPUT_IDLE);
    if (cfg != PCA_CONFIG_VALUE || out != PCA_OUTPUT_IDLE) {
        ESP_LOGE(TAG, "Readback do PCA9554A divergente");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* LEDs                                                                */
/* ------------------------------------------------------------------ */

static void led_set(uint8_t pin, bool on)
{
    bool level = LED_ACTIVE_HIGH ? on : !on;
    esp_err_t err = pca9554_set_pin_level(&s_pca, pin, level);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "falha ao escrever LED no IO%u: %s", pin, esp_err_to_name(err));
    }
}

static void led_test_sequence(void)
{
    for (size_t i = 0; i < sizeof(s_led_pins); i++) {
        ESP_LOGI(TAG, "LED em IO%u ligado por %d ms", s_led_pins[i], LED_ON_TIME_MS);
        led_set(s_led_pins[i], true);
        vTaskDelay(pdMS_TO_TICKS(LED_ON_TIME_MS));
        led_set(s_led_pins[i], false);
    }
    ESP_LOGI(TAG, "LED em IO%u fixo aceso", LED_FIXED_PIN);
    led_set(LED_FIXED_PIN, true);
}

/* ------------------------------------------------------------------ */
/* Sensores VL53L1X                                                    */
/* ------------------------------------------------------------------ */

static void xshut_set(const tof_t *s, bool released)
{
    esp_err_t err = pca9554_set_pin_level(&s_pca, s->xshut_pin, released);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s: falha ao escrever XSHUT (IO%u): %s", s->name, s->xshut_pin, esp_err_to_name(err));
    }
}

/* Fase A: todos os XSHUT estao em LOW; libera um por vez e troca o endereco. */
static void tof_assign_addresses(void)
{
    /* Garante o reset de todos (e o retorno a 0x29, caso o ESP32 tenha
     * reiniciado com os sensores ainda energizados). */
    for (int i = 0; i < N_SENSORS; i++) {
        xshut_set(&s_tof[i], false);
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    if (i2c_master_probe(s_bus, VL53L1X_DEFAULT_I2C_ADDR, 50) == ESP_OK) {
        ESP_LOGW(TAG, "Algo responde em 0x%02X com todos os XSHUT em LOW. "
                      "Confira a fiacao do XSHUT (ou ha outro dispositivo nesse endereco).",
                 VL53L1X_DEFAULT_I2C_ADDR);
    }

    for (int i = 0; i < N_SENSORS; i++) {
        tof_t *s = &s_tof[i];
        s->state = TOF_OFFLINE;

        ESP_LOGI(TAG, "%s: liberando XSHUT (IO%u)", s->name, s->xshut_pin);
        xshut_set(s, true);
        vTaskDelay(pdMS_TO_TICKS(5));   /* margem para o boot do sensor */

        esp_err_t err = vl53l1x_init_bus_device(s_bus, VL53L1X_DEFAULT_I2C_ADDR, &s->dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: falha ao criar handle em 0x29: %s", s->name, esp_err_to_name(err));
            xshut_set(s, false);
            continue;
        }

        err = vl53l1x_wait_for_boot(&s->dev, 500);
        if (err == ESP_OK) {
            err = vl53l1x_set_i2c_address(s_bus, &s->dev, s->addr);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: nao respondeu/mudou de endereco (%s). Mantendo XSHUT em LOW.",
                     s->name, esp_err_to_name(err));
            if (s->dev.i2c_dev) vl53l1x_deinit(&s->dev);
            xshut_set(s, false);   /* nao pode ficar em 0x29 e colidir com o proximo */
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

/* Fase B: configura e inicia ranging em cada sensor ja enderecado. */
static void tof_start_all(void)
{
    for (int i = 0; i < N_SENSORS; i++) {
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
        ESP_LOGI(TAG, "%s (0x%02X): ranging iniciado", s->name, s->addr);
    }
}

static void tof_poll_once(void)
{
    for (int i = 0; i < N_SENSORS; i++) {
        tof_t *s = &s_tof[i];
        if (s->state != TOF_RANGING) continue;

        bool ready = false;
        esp_err_t err = vl53l1x_check_data_ready(&s->dev, &ready);
        if (err != ESP_OK) {
            s->i2c_errors++;
            continue;
        }
        if (!ready) continue;

        vl53l1x_result_t r;
        err = vl53l1x_get_result(&s->dev, &r);
        if (err == ESP_OK) err = vl53l1x_clear_interrupt(&s->dev);
        if (err != ESP_OK) {
            s->i2c_errors++;
            continue;
        }
        s->last = r;
        s->has_result = true;
    }
}

static void tof_print_line(void)
{
    printf("[%8lld ms]", (long long)(esp_timer_get_time() / 1000));
    for (int i = 0; i < N_SENSORS; i++) {
        const tof_t *s = &s_tof[i];
        printf("  %s(0x%02X): ", s->name, s->addr);
        if (s->state != TOF_RANGING) {
            printf("OFFLINE");
        } else if (!s->has_result) {
            printf("aguardando");
        } else if (s->last.range_status == 0) {
            printf("%4u mm", s->last.distance_mm);
        } else {
            printf("---- (st=%u)", s->last.range_status);
        }
        if (s->i2c_errors) printf(" [errI2C=%lu]", (unsigned long)s->i2c_errors);
    }
    printf("\n");
}

/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "GCM-PI2-2026.2: teste de 3x VL53L1X (XSHUT via PCA9554A)");

    i2c1_bus_init();

    if (expander_init() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar o PCA9554A - abortando.");
        return;
    }

    /* 1) Teste dos LEDs, um por vez, e depois um fixo */
    led_test_sequence();

    /* 2) Atribuicao de enderecos via XSHUT */
    tof_assign_addresses();

    /* 3) Configuracao e inicio do ranging */
    tof_start_all();

    int n_ok = 0;
    for (int i = 0; i < N_SENSORS; i++) n_ok += (s_tof[i].state == TOF_RANGING);
    ESP_LOGI(TAG, "%d de %d sensores em operacao", n_ok, N_SENSORS);

    /* 4) Loop de polling */
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_print_us = 0;
    for (;;) {
        tof_poll_once();

        int64_t now_us = esp_timer_get_time();
        if (now_us - last_print_us >= (int64_t)PRINT_PERIOD_MS * 1000) {
            tof_print_line();
            last_print_us = now_us;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
