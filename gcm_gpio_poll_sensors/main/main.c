/*
 * GCM-PI2-2026.2 - Teste de entradas digitais por polling
 * ESP-IDF v5.4.2 / ESP32-WROOM-32D
 *
 * Amostra BS_1..BS_4 (sensores de borda) e START_BOT (botão de start)
 * e imprime o nível lido de cada GPIO no monitor serial.
 *
 * Pinout (CN15):
 *   BS_1      GPIO34  (CN15-21)  entrada-somente, pull-up EXTERNO R23 (3V3)
 *   BS_2      GPIO35  (CN15-9)   entrada-somente, pull-up EXTERNO R22 (3V3)
 *   BS_3      GPIO16  (CN15-19)  sem pull-up externo
 *   BS_4      GPIO14  (CN15-18)  sem pull-up externo
 *   START_BOT GPIO4   (CN15-20)  sem pull-up externo
 *
 * GPIO34/35 não possuem pull interno; só GPIO16/14/4 podem usar pull-up interno.
 * ATENÇÃO: as entradas do CN15 não têm proteção; sinais externos devem ser <= 3,3 V.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "gcm_gpio";

/* ---------------- Configuração ---------------- */
#define POLL_PERIOD_MS      10    /* período de amostragem                          */
#define PRINT_PERIOD_MS     500   /* período da linha de status periódica           */
#define DEBOUNCE_SAMPLES    1     /* amostras iguais consecutivas p/ aceitar mudança
                                     (1 = sem debounce, mostra o nível cru)         */
#define USE_INTERNAL_PULLUP 1     /* 1: pull-up interno em GPIO16/14/4 (34/35 não têm) */

typedef struct {
    const char *name;
    gpio_num_t  gpio;
    bool        has_internal_pull;   /* GPIO34-39 não têm */
    int         level;               /* nível aceito (estável) */
    int         candidate;           /* último nível visto */
    int         count;               /* amostras consecutivas do candidato */
} input_t;

static input_t inputs[] = {
    { "BS_1",      GPIO_NUM_34, false, -1, -1, 0 },
    { "BS_2",      GPIO_NUM_35, false, -1, -1, 0 },
    { "BS_3",      GPIO_NUM_16, true,  -1, -1, 0 },
    { "BS_4",      GPIO_NUM_14, true,  -1, -1, 0 },
    { "START_BOT", GPIO_NUM_4,  true,  -1, -1, 0 },
};
#define N_INPUTS (sizeof(inputs) / sizeof(inputs[0]))

static void inputs_init(void)
{
    for (size_t i = 0; i < N_INPUTS; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << inputs[i].gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = (USE_INTERNAL_PULLUP && inputs[i].has_internal_pull)
                            ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&cfg));

        int lvl = gpio_get_level(inputs[i].gpio);
        inputs[i].level = inputs[i].candidate = lvl;
        inputs[i].count = DEBOUNCE_SAMPLES;
        ESP_LOGI(TAG, "%-9s GPIO%-2d pull-up %s, nivel inicial = %d",
                 inputs[i].name, (int)inputs[i].gpio,
                 inputs[i].has_internal_pull
                     ? (USE_INTERNAL_PULLUP ? "interno" : "desligado")
                     : "externo (placa)",
                 lvl);
    }
}

static void print_status(void)
{
    printf("[%8lld ms]", (long long)(esp_timer_get_time() / 1000));
    for (size_t i = 0; i < N_INPUTS; i++) {
        printf("  %s(GPIO%d)=%d", inputs[i].name, (int)inputs[i].gpio, inputs[i].level);
    }
    printf("\n");
}

static void poll_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_print_us = 0;

    for (;;) {
        bool changed = false;

        for (size_t i = 0; i < N_INPUTS; i++) {
            int lvl = gpio_get_level(inputs[i].gpio);

            if (lvl == inputs[i].candidate) {
                if (inputs[i].count < DEBOUNCE_SAMPLES) inputs[i].count++;
            } else {
                inputs[i].candidate = lvl;
                inputs[i].count = 1;
            }

            if (inputs[i].count >= DEBOUNCE_SAMPLES && inputs[i].level != inputs[i].candidate) {
                printf("[%8lld ms] MUDANCA  %s (GPIO%d): %d -> %d\n",
                       (long long)(esp_timer_get_time() / 1000),
                       inputs[i].name, (int)inputs[i].gpio,
                       inputs[i].level, inputs[i].candidate);
                inputs[i].level = inputs[i].candidate;
                changed = true;
            }
        }

        int64_t now_us = esp_timer_get_time();
        if (changed || (now_us - last_print_us) >= (int64_t)PRINT_PERIOD_MS * 1000) {
            print_status();
            last_print_us = now_us;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "GCM-PI2-2026.2: polling de BS_1..BS_4 e START_BOT");
    inputs_init();
    xTaskCreate(poll_task, "poll_task", 4096, NULL, 5, NULL);
}
