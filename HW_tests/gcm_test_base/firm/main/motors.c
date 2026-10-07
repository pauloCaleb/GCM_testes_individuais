#include "motors.h"
#include "app_config.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "motors";

#define PWM_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define PWM_TIMER        LEDC_TIMER_0
#define PWM_RES          LEDC_TIMER_10_BIT
#define PWM_MAX_RAW      1023u

#define EN_INACTIVE_LEVEL (EN_ACTIVE_LEVEL ? 0 : 1)

typedef struct {
    gpio_num_t     pwm_pin;
    gpio_num_t     dir_pin;
    int            dir_fwd_level;
    ledc_channel_t channel;
    float          cur;      /* duty aplicado (%) */
    float          tgt;      /* alvo (%) */
    int            dir;      /* +1 frente, -1 ré, 0 = ainda não definido */
    int            settle;   /* ticks restantes parado após trocar DIR */
} motor_t;

static motor_t s_m[MOTOR_COUNT] = {
    { PIN_M1_PWM, PIN_M1_DIR, M1_DIR_FWD_LEVEL, LEDC_CHANNEL_0, 0, 0, 0, 0 },
    { PIN_M2_PWM, PIN_M2_DIR, M2_DIR_FWD_LEVEL, LEDC_CHANNEL_1, 0, 0, 0, 0 },
};

static SemaphoreHandle_t s_mtx;
static bool     s_en;
static float    s_max_duty = MOTOR_DEFAULT_MAX_DUTY;
static float    s_slew     = MOTOR_DEFAULT_SLEW;
static uint32_t s_wd_ms    = MOTOR_DEFAULT_WD_MS;
static bool     s_failsafe;
static uint32_t s_last_feed_ms;
static motors_failsafe_cb_t s_failsafe_cb;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void write_pwm(const motor_t *m, float duty_abs_pct)
{
    uint32_t raw = (uint32_t)(clampf(duty_abs_pct, 0.0f, 100.0f) / 100.0f * (float)PWM_MAX_RAW + 0.5f);
    ledc_set_duty(PWM_SPEED_MODE, m->channel, raw);
    ledc_update_duty(PWM_SPEED_MODE, m->channel);
}

static void write_dir(const motor_t *m, int dir)
{
    int fwd = (dir >= 0);
    gpio_set_level(m->dir_pin, fwd ? m->dir_fwd_level : !m->dir_fwd_level);
}

static void write_en(bool en)
{
    gpio_set_level(PIN_EN_ALL, en ? EN_ACTIVE_LEVEL : EN_INACTIVE_LEVEL);
}

/* Parada imediata. Chamar com o mutex tomado. */
static void stop_locked(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_m[i].cur = 0;
        s_m[i].tgt = 0;
        s_m[i].settle = 0;
        write_pwm(&s_m[i], 0);
    }
    s_en = false;
    write_en(false);
}

esp_err_t motors_init(void)
{
    s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return ESP_ERR_NO_MEM;

    /* 1) EN inativo primeiro. */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_EN_ALL,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;
    write_en(false);

    /* 2) DIR e PWM em nível baixo antes de entregar o PWM ao LEDC. */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        io.pin_bit_mask = (1ULL << s_m[i].dir_pin) | (1ULL << s_m[i].pwm_pin);
        err = gpio_config(&io);
        if (err != ESP_OK) return err;
        gpio_set_level(s_m[i].pwm_pin, 0);
        write_dir(&s_m[i], +1);
    }

    /* 3) LEDC: 20 kHz, 10 bits, duty 0. */
    ledc_timer_config_t tcfg = {
        .speed_mode      = PWM_SPEED_MODE,
        .duty_resolution = PWM_RES,
        .timer_num       = PWM_TIMER,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) return err;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ccfg = {
            .gpio_num   = s_m[i].pwm_pin,
            .speed_mode = PWM_SPEED_MODE,
            .channel    = s_m[i].channel,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = PWM_TIMER,
            .duty       = 0,
            .hpoint     = 0,
        };
        err = ledc_channel_config(&ccfg);
        if (err != ESP_OK) return err;
    }

    s_last_feed_ms = now_ms();
    ESP_LOGI(TAG, "pontes H em estado seguro (EN inativo, PWM=0, %d Hz)", MOTOR_PWM_FREQ_HZ);
    return ESP_OK;
}

void motors_set_failsafe_cb(motors_failsafe_cb_t cb)
{
    s_failsafe_cb = cb;
}

float motors_set_target(int ch, float duty_pct)
{
    if (ch < 0 || ch >= MOTOR_COUNT) return 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    float v = clampf(duty_pct, -s_max_duty, s_max_duty);
    s_m[ch].tgt = v;
    xSemaphoreGive(s_mtx);
    return v;
}

void motors_set_enable(bool en)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (en && !s_en) {
        /* Ao habilitar a ponte, ninguém sai girando com um alvo antigo. */
        for (int i = 0; i < MOTOR_COUNT; i++) {
            s_m[i].tgt = 0;
        }
        s_failsafe = false;
    }
    s_en = en;
    write_en(en);
    if (!en) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            s_m[i].cur = 0;
            s_m[i].tgt = 0;
            s_m[i].settle = 0;
            write_pwm(&s_m[i], 0);
        }
    }
    s_last_feed_ms = now_ms();
    xSemaphoreGive(s_mtx);
}

void motors_stop(void)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    stop_locked();
    xSemaphoreGive(s_mtx);
}

void motors_feed_watchdog(void)
{
    s_last_feed_ms = now_ms();
}

void motors_set_max_duty(float pct)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_max_duty = clampf(pct, 0.0f, 100.0f);
    xSemaphoreGive(s_mtx);
}

void motors_set_slew(float pct_per_s)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_slew = pct_per_s < 0 ? 0 : pct_per_s;
    xSemaphoreGive(s_mtx);
}

void motors_set_wd_ms(uint32_t ms)
{
    s_wd_ms = ms;
    s_last_feed_ms = now_ms();
}

void motors_get(motors_snapshot_t *out)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        out->cur[i] = s_m[i].cur;
        out->tgt[i] = s_m[i].tgt;
    }
    out->en       = s_en;
    out->max_duty = s_max_duty;
    out->slew     = s_slew;
    out->wd_ms    = s_wd_ms;
    out->failsafe = s_failsafe;
    xSemaphoreGive(s_mtx);
}

static int sign_of(float v)
{
    return v > 0.0f ? +1 : (v < 0.0f ? -1 : 0);
}

/* Um passo de controle de um motor (mutex tomado).
 *
 * Regras de segurança:
 *   - DIR só muda com o motor parado (duty 0) E com o PWM já escrito em 0 no
 *     tick anterior; depois fica MOTOR_DIR_SETTLE_TICKS ticks parado.
 *   - Para inverter o sentido (ou parar), o duty desce primeiro até 0.
 */
static void step_motor(motor_t *m)
{
    const float step_max = (s_slew > 0.0f) ? s_slew * ((float)MOTOR_TICK_MS / 1000.0f) : 1000.0f;

    /* O limite pode ter sido reduzido depois de definido o alvo. */
    float tgt = clampf(m->tgt, -s_max_duty, s_max_duty);
    if (!s_en) tgt = 0;

    if (m->settle > 0) {
        /* Parado em duty 0 enquanto o DIR assenta. */
        m->settle--;
        m->cur = 0;
        write_pwm(m, 0);
        return;
    }

    int want = sign_of(tgt);
    int have = sign_of(m->cur);

    /* Parado e com alvo em outro sentido: troca DIR antes de acelerar. */
    if (have == 0 && want != 0 && m->dir != want) {
        write_dir(m, want);
        m->dir = want;
        m->settle = MOTOR_DIR_SETTLE_TICKS;
        write_pwm(m, 0);
        return;
    }

    float eff = (have != 0 && want != have) ? 0.0f : tgt;
    float diff = eff - m->cur;
    if (fabsf(diff) <= step_max) m->cur = eff;
    else                         m->cur += (diff > 0 ? step_max : -step_max);

    write_pwm(m, fabsf(m->cur));
}

static void motors_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        bool fire_failsafe = false;

        xSemaphoreTake(s_mtx, portMAX_DELAY);

        for (int i = 0; i < MOTOR_COUNT; i++) step_motor(&s_m[i]);

        bool active = s_en || s_m[0].cur != 0 || s_m[1].cur != 0 || s_m[0].tgt != 0 || s_m[1].tgt != 0;
        if (s_wd_ms > 0 && active && !s_failsafe && (now_ms() - s_last_feed_ms) > s_wd_ms) {
            stop_locked();
            s_failsafe = true;
            fire_failsafe = true;
        }

        xSemaphoreGive(s_mtx);

        if (fire_failsafe) {
            ESP_LOGW(TAG, "WATCHDOG: sem comandos por %lu ms -> STOP", (unsigned long)s_wd_ms);
            if (s_failsafe_cb) s_failsafe_cb();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_TICK_MS));
    }
}

void motors_start_task(void)
{
    xTaskCreate(motors_task, "motors", 4096, NULL, 7, NULL);
}
