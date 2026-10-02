#include "motor.h"
#include "config.h"

#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"

#define LEDC_MODE   LEDC_LOW_SPEED_MODE
#define LEDC_TIMER  LEDC_TIMER_0

typedef struct {
    gpio_num_t in_a;     /* nível "frente": in_a = LOW  */
    gpio_num_t in_b;     /* nível "frente": in_b = HIGH */
    gpio_num_t en;
    ledc_channel_t ch;
} motor_hw_t;

static const motor_hw_t s_motor[MOTOR_COUNT] = {
    [MOTOR_A] = { PIN_IN1, PIN_IN2, PIN_EN12, LEDC_CHANNEL_0 },
    [MOTOR_B] = { PIN_IN3, PIN_IN4, PIN_EN34, LEDC_CHANNEL_1 },
};

static void set_duty(const motor_hw_t *hw, uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, hw->ch, duty);
    ledc_update_duty(LEDC_MODE, hw->ch);
}

void motor_init(void)
{
    /* Direção: tudo em LOW antes de qualquer coisa (GPIO12 e GPIO5 são
     * strapping pins - nunca devem ser forçados a HIGH durante o boot). */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        const motor_hw_t *hw = &s_motor[i];
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << hw->in_a) | (1ULL << hw->in_b),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(hw->in_a, 0);
        gpio_set_level(hw->in_b, 0);
    }

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = s_motor[i].en,
            .speed_mode = LEDC_MODE,
            .channel = s_motor[i].ch,
            .timer_sel = LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
}

void motor_set(motor_id_t m, float speed)
{
    if (m >= MOTOR_COUNT) return;
    const motor_hw_t *hw = &s_motor[m];

    if (speed > 1.0f)  speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    if (speed > 0.0f) {
        gpio_set_level(hw->in_a, 0);
        gpio_set_level(hw->in_b, 1);
    } else if (speed < 0.0f) {
        gpio_set_level(hw->in_a, 1);
        gpio_set_level(hw->in_b, 0);
    } else {
        gpio_set_level(hw->in_a, 0);
        gpio_set_level(hw->in_b, 0);
    }

    set_duty(hw, (uint32_t)(fabsf(speed) * PWM_MAX_DUTY));
}

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set((motor_id_t)i, 0.0f);
    }
}
