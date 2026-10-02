/*
 * ATOM - robô de sparring (referência ao robô de "Gigantes de Aço")
 * Controle por BLE via app Dabble (módulo Gamepad) - ESP32 WROOM + ESP-IDF.
 *
 * Migração do firmware Arduino "Robozito/Minion":
 *   - Dabble (Arduino)  -> NimBLE + parser do protocolo (dabble_gamepad.c)
 *   - ledcSetup/Write   -> driver LEDC do ESP-IDF (motor.c)
 *   - loop()            -> task FreeRTOS de controle a cada CONTROL_PERIOD_MS
 *
 * Nota de hardware: IN3 está no GPIO12 (MTDI, seleciona a tensão da flash) e
 * IN2 no GPIO5. Funciona porque a ponte H não deve puxar essas entradas para
 * HIGH durante o boot; se o robô não bootar ou travar ao ligar, suspeite disso.
 */
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "config.h"
#include "motor.h"
#include "dabble_gamepad.h"
#include "ble_gamepad.h"

static const char *TAG = "ATOM";

typedef enum { MODE_IDLE, MODE_SPIN_SQUARE, MODE_SPIN_CIRCLE, MODE_JOYSTICK, MODE_DISCONNECTED } drive_mode_t;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Curva do joystick: zona morta + cubo (idêntico ao firmware antigo). */
static float shape_axis(float v)
{
    if (fabsf(v) < DEAD_ZONE) v = 0.0f;
    return v * v * v;
}

static void on_ble_connection(bool connected)
{
    if (!connected) {
        motor_stop_all();       /* failsafe imediato; o laço de controle mantém parado */
    }
}

static void control_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    drive_mode_t last_mode = MODE_DISCONNECTED;

    for (;;) {
        drive_mode_t mode;
        float a = 0.0f, b = 0.0f;

        if (!ble_gamepad_is_connected()) {
            mode = MODE_DISCONNECTED;
        } else {
            dabble_gp_state_t s;
            dabble_gp_get(&s);

            if (s.buttons & DABBLE_BTN_SQUARE) {
                /* Mesmos níveis de pino do firmware antigo: A=(LOW,HIGH) B=(HIGH,LOW).
                 * Se na prática o quadrado girar para o lado errado, troque os sinais. */
                mode = MODE_SPIN_SQUARE;
                a = +1.0f; b = -1.0f;
            } else if (s.buttons & DABBLE_BTN_CIRCLE) {
                mode = MODE_SPIN_CIRCLE;
                a = -1.0f; b = +1.0f;
            } else {
                mode = MODE_JOYSTICK;
                float x, y;
                dabble_gp_axes(&s, &x, &y);
#if JOY_NORMALIZE
                x /= 7.0f;
                y /= 7.0f;
#endif
                x = shape_axis(x);
                y = shape_axis(y);
                a = clampf(y + x * STEER_GAIN, -1.0f, 1.0f);
                b = clampf(y - x * STEER_GAIN, -1.0f, 1.0f);
            }
        }

        motor_set(MOTOR_A, a);
        motor_set(MOTOR_B, b);

        if (mode != last_mode) {            /* loga só nas transições */
            switch (mode) {
            case MODE_SPIN_SQUARE:  ESP_LOGI(TAG, "Giro rápido: quadrado"); break;
            case MODE_SPIN_CIRCLE:  ESP_LOGI(TAG, "Giro rápido: círculo");  break;
            case MODE_JOYSTICK:     ESP_LOGI(TAG, "Modo joystick");         break;
            case MODE_DISCONNECTED: ESP_LOGW(TAG, "Sem controle - motores parados"); break;
            default: break;
            }
            last_mode = mode;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void app_main(void)
{
    /* NVS é exigida pelo controlador BT (calibração de PHY) */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    motor_init();                                   /* motores parados */
    ble_gamepad_start(ROBOT_NAME, on_ble_connection);

    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "ATOM pronto - conecte pelo Dabble em \"%s\"", ROBOT_NAME);
}
