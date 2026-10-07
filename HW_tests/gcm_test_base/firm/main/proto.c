#include "proto.h"
#include "app_config.h"
#include "expander.h"
#include "inputs.h"
#include "mini_json.h"
#include "motors.h"
#include "proto_format.h"
#include "tof.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "proto";

static volatile int s_tel_hz = PROTO_DEFAULT_TEL_HZ;
static long s_seq = -1;   /* seq do comando em tratamento (-1 = sem) */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Formata e envia UMA linha (uma única chamada de fputs: linhas não se misturam
 * com os logs do ESP-IDF, que usam o mesmo stdout). */
static void emit(const char *fmt, ...)
{
    char buf[PROTO_TX_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 3) n = (int)sizeof(buf) - 3;
    buf[n++] = '\n';
    buf[n] = '\0';
    fputs(buf, stdout);
}

static const char *seq_suffix(char *tmp, size_t n)
{
    if (s_seq < 0) return "";
    snprintf(tmp, n, ",\"seq\":%ld", s_seq);
    return tmp;
}

static void emit_err(const char *cmd, const char *msg)
{
    /* Só ecoa o nome do comando se for seguro dentro de uma string JSON. */
    char safe[20];
    size_t k = 0;
    for (; cmd && cmd[k] && k < sizeof(safe) - 1; k++) {
        char c = cmd[k];
        safe[k] = ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_') ? c : '?';
    }
    safe[k] = '\0';
    char sq[24];
    emit("{\"t\":\"err\",\"cmd\":\"%s\",\"msg\":\"%s\"%s}", safe, msg, seq_suffix(sq, sizeof(sq)));
}

/* ------------------------------------------------------------------ */
/* Mensagens de saída                                                  */
/* ------------------------------------------------------------------ */

void proto_send_hello(void)
{
    motors_snapshot_t m;
    motors_get(&m);
    tof_snapshot_t t[TOF_COUNT];
    for (int i = 0; i < TOF_COUNT; i++) tof_get(i, &t[i]);
    char sq[24];

    emit("{\"t\":\"hello\",\"fw\":\"%s\",\"ver\":\"%s\",\"proto\":%d,\"exp\":%d,"
         "\"tof\":[%d,%d,%d],\"cap\":%.1f,\"wd_ms\":%lu,\"slew\":%.1f,\"tel_hz\":%d%s}",
         FW_NAME, FW_VERSION, PROTO_VERSION, expander_ok() ? 1 : 0,
         t[0].online, t[1].online, t[2].online,
         (double)m.max_duty, (unsigned long)m.wd_ms, (double)m.slew, s_tel_hz,
         seq_suffix(sq, sizeof(sq)));
}

static void on_input_event(const char *name, int idx, int level)
{
    emit("{\"t\":\"evt\",\"name\":\"%s\",\"idx\":%d,\"v\":%d,\"ms\":%lu}",
         name, idx, level, (unsigned long)now_ms());
}

static void on_failsafe(void)
{
    emit("{\"t\":\"evt\",\"name\":\"watchdog\",\"idx\":0,\"v\":1,\"ms\":%lu}", (unsigned long)now_ms());
}

/* ------------------------------------------------------------------ */
/* Comandos                                                            */
/* ------------------------------------------------------------------ */

static void cmd_led(const mj_obj_t *o)
{
    double on_d;
    if (!mj_num(o, "on", &on_d)) { emit_err("led", "falta_on"); return; }
    bool on = (on_d != 0);
    char sq[24];

    if (!expander_ok()) { emit_err("led", "pca9554_indisponivel"); return; }

    const char *s = mj_str(o, "idx");
    if (s && strcmp(s, "all") == 0) {
        for (int i = 1; i <= 3; i++) {
            if (expander_led_set(i, on) != ESP_OK) { emit_err("led", "falha_i2c"); return; }
        }
        emit("{\"t\":\"ack\",\"cmd\":\"led\",\"idx\":\"all\",\"on\":%d%s}", on, seq_suffix(sq, sizeof(sq)));
        return;
    }

    double idx_d;
    if (!mj_num(o, "idx", &idx_d) || idx_d < 1 || idx_d > 3) { emit_err("led", "idx_invalido"); return; }
    int idx = (int)idx_d;
    if (expander_led_set(idx, on) != ESP_OK) { emit_err("led", "falha_i2c"); return; }
    emit("{\"t\":\"ack\",\"cmd\":\"led\",\"idx\":%d,\"on\":%d%s}", idx, on, seq_suffix(sq, sizeof(sq)));
}

static void cmd_motor(const mj_obj_t *o)
{
    double ch_d, duty;
    if (!mj_num(o, "ch", &ch_d) || (ch_d != 1 && ch_d != 2)) { emit_err("motor", "ch_invalido"); return; }
    if (!mj_num(o, "duty", &duty) || !(duty >= -100.0 && duty <= 100.0)) { emit_err("motor", "duty_invalido"); return; }

    float applied = motors_set_target((int)ch_d - 1, (float)duty);
    char sq[24];
    emit("{\"t\":\"ack\",\"cmd\":\"motor\",\"ch\":%d,\"duty\":%.1f%s}", (int)ch_d, (double)applied, seq_suffix(sq, sizeof(sq)));
}

static void cmd_en(const mj_obj_t *o)
{
    double on_d;
    if (!mj_num(o, "on", &on_d)) { emit_err("en", "falta_on"); return; }
    motors_set_enable(on_d != 0);
    char sq[24];
    emit("{\"t\":\"ack\",\"cmd\":\"en\",\"on\":%d%s}", on_d != 0, seq_suffix(sq, sizeof(sq)));
}

static void cmd_cfg(const mj_obj_t *o)
{
    double hz = 0, cap = 0, wd = 0, slew = 0;
    bool has_hz = mj_num(o, "tel_hz", &hz);
    bool has_cap = mj_num(o, "max_duty", &cap);
    bool has_wd = mj_num(o, "wd_ms", &wd);
    bool has_slew = mj_num(o, "slew", &slew);

    /* Valida tudo antes de aplicar qualquer coisa. */
    if (has_hz && !(hz >= 1 && hz <= 50))                      { emit_err("cfg", "tel_hz_invalido"); return; }
    if (has_cap && !(cap >= 0 && cap <= 100))                  { emit_err("cfg", "max_duty_invalido"); return; }
    if (has_wd && !(wd == 0 || (wd >= 50 && wd <= 10000)))     { emit_err("cfg", "wd_ms_invalido"); return; }
    if (has_slew && !(slew >= 0 && slew <= 2000))              { emit_err("cfg", "slew_invalido"); return; }

    if (has_hz)   s_tel_hz = (int)hz;
    if (has_cap)  motors_set_max_duty((float)cap);
    if (has_wd)   motors_set_wd_ms((uint32_t)wd);
    if (has_slew) motors_set_slew((float)slew);

    motors_snapshot_t m;
    motors_get(&m);
    char sq[24];
    emit("{\"t\":\"ack\",\"cmd\":\"cfg\",\"tel_hz\":%d,\"max_duty\":%.1f,\"wd_ms\":%lu,\"slew\":%.1f%s}",
         s_tel_hz, (double)m.max_duty, (unsigned long)m.wd_ms, (double)m.slew, seq_suffix(sq, sizeof(sq)));
}

static void handle_line(char *line)
{
    mj_obj_t o;
    s_seq = -1;

    if (!mj_parse(line, &o)) { emit_err("", "json_invalido"); return; }

    double seq_d;
    if (mj_num(&o, "seq", &seq_d) && seq_d >= 0 && seq_d < 2147483647.0) s_seq = (long)seq_d;

    const char *cmd = mj_str(&o, "cmd");
    if (!cmd) { emit_err("", "sem_cmd"); return; }

    char sq[24];
    if (strcmp(cmd, "ping") == 0) {
        motors_feed_watchdog();
        emit("{\"t\":\"pong\",\"ms\":%lu%s}", (unsigned long)now_ms(), seq_suffix(sq, sizeof(sq)));
    } else if (strcmp(cmd, "hello") == 0) {
        motors_feed_watchdog();
        proto_send_hello();
    } else if (strcmp(cmd, "led") == 0) {
        motors_feed_watchdog();
        cmd_led(&o);
    } else if (strcmp(cmd, "motor") == 0) {
        motors_feed_watchdog();
        cmd_motor(&o);
    } else if (strcmp(cmd, "en") == 0) {
        motors_feed_watchdog();
        cmd_en(&o);
    } else if (strcmp(cmd, "stop") == 0) {
        motors_stop();
        motors_feed_watchdog();
        emit("{\"t\":\"ack\",\"cmd\":\"stop\"%s}", seq_suffix(sq, sizeof(sq)));
    } else if (strcmp(cmd, "cfg") == 0) {
        motors_feed_watchdog();
        cmd_cfg(&o);
    } else {
        emit_err(cmd, "cmd_desconhecido");
    }
}

/* ------------------------------------------------------------------ */
/* Tasks                                                               */
/* ------------------------------------------------------------------ */

static void rx_task(void *arg)
{
    (void)arg;
    char line[PROTO_LINE_MAX];
    size_t n = 0;
    bool overflow = false;
    uint8_t buf[64];

    for (;;) {
        int r = uart_read_bytes(PROTO_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < r; i++) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (overflow) {
                    emit_err("", "linha_longa");
                } else if (n > 0) {
                    line[n] = '\0';
                    /* Só processa linhas JSON; qualquer outra coisa é ignorada. */
                    char *p = line;
                    while (*p == ' ' || *p == '\t') p++;
                    if (*p == '{') handle_line(p);
                }
                n = 0;
                overflow = false;
            } else if (n < sizeof(line) - 1) {
                line[n++] = c;
            } else {
                overflow = true;
            }
        }
    }
}

static void build_tel(proto_tel_t *t)
{
    memset(t, 0, sizeof(*t));
    t->ms = now_ms();

    inputs_state_t in;
    inputs_get(&in);
    for (int i = 0; i < 4; i++) t->bs[i] = in.bs[i];
    t->start = in.start;

    for (int i = 0; i < TOF_COUNT; i++) {
        tof_snapshot_t s;
        tof_get(i, &s);
        t->tof[i].on = s.online ? 1 : 0;
        t->tof[i].mm = s.has_result ? (int)s.mm : -1;
        t->tof[i].st = s.has_result ? s.status : 255;
        t->tof[i].age_ms = s.age_ms;
        t->tof[i].err = s.i2c_errors;
    }

    for (int i = 0; i < 3; i++) t->led[i] = expander_ok() ? (expander_led_get(i + 1) ? 1 : 0) : 0;

    motors_snapshot_t m;
    motors_get(&m);
    t->en = m.en ? 1 : 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        t->m_cur[i] = m.cur[i];
        t->m_tgt[i] = m.tgt[i];
    }
    t->cap = m.max_duty;
    t->failsafe = m.failsafe ? 1 : 0;
}

static void tx_task(void *arg)
{
    (void)arg;
    char buf[PROTO_TEL_MAX];
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        proto_tel_t t;
        build_tel(&t);
        int n = proto_format_tel(buf, sizeof(buf) - 2, &t);
        if (n > 0) {
            buf[n++] = '\n';
            buf[n] = '\0';
            fputs(buf, stdout);
        }

        int hz = s_tel_hz;
        if (hz < 1) hz = 1;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / hz));
    }
}

esp_err_t proto_init(void)
{
    /* O console (printf/logs) continua em modo polling; o driver é usado só
     * para receber os comandos. */
    esp_err_t err = uart_driver_install(PROTO_UART_NUM, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install falhou: %s", esp_err_to_name(err));
        return err;
    }
    inputs_set_event_cb(on_input_event);
    motors_set_failsafe_cb(on_failsafe);
    return ESP_OK;
}

void proto_start_tasks(void)
{
    xTaskCreate(rx_task, "proto_rx", 4096, NULL, 6, NULL);
    xTaskCreate(tx_task, "proto_tx", 4096, NULL, 4, NULL);
}
