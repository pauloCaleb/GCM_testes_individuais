#include "dabble_gamepad.h"

#include <math.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define START_OF_FRAME   0xFF
#define END_OF_FRAME     0x00
#define GAMEPAD_ID       0x01
#define GP_DIGITAL       0x01
#define GP_ANALOG        0x02
#define GP_ACCL          0x03
#define FRAME_TIMEOUT_US (2000 * 1000)   /* mesmo timeout de 2 s da lib original */

typedef enum { S_IDLE, S_MODULE, S_FUNC, S_ARGC, S_ARGLEN, S_ARGDATA, S_END } pstate_t;

static struct {
    pstate_t st;
    uint8_t  module, func, argc, argi, arglen, got;
    uint8_t  arg0[2];       /* só guardamos os 2 bytes do 1º argumento */
    int64_t  last_byte_us;
} s_p;

static dabble_gp_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static void parser_reset(void)
{
    s_p.st = S_IDLE;
}

static void frame_done(void)
{
    if (s_p.module != GAMEPAD_ID) return;
    if (s_p.func < GP_DIGITAL || s_p.func > GP_ACCL) return;
    if (s_p.argc != 1) return;

    portENTER_CRITICAL(&s_lock);
    s_state.buttons = s_p.arg0[0];
    s_state.joy_raw = s_p.arg0[1];
    s_state.analog  = (s_p.func != GP_DIGITAL);
    portEXIT_CRITICAL(&s_lock);
}

static void feed_byte(uint8_t b)
{
    int64_t now = esp_timer_get_time();
    if (s_p.st != S_IDLE && (now - s_p.last_byte_us) > FRAME_TIMEOUT_US) {
        parser_reset();
    }
    s_p.last_byte_us = now;

    switch (s_p.st) {
    case S_IDLE:
        if (b == START_OF_FRAME) s_p.st = S_MODULE;
        break;
    case S_MODULE:
        s_p.module = b;
        s_p.st = S_FUNC;
        break;
    case S_FUNC:
        s_p.func = b;
        s_p.st = S_ARGC;
        break;
    case S_ARGC:
        s_p.argc = b;
        s_p.argi = 0;
        s_p.arg0[0] = s_p.arg0[1] = 0;
        s_p.st = (b == 0) ? S_END : S_ARGLEN;
        break;
    case S_ARGLEN:
        s_p.arglen = b;
        s_p.got = 0;
        s_p.st = S_ARGDATA;
        if (b == 0) {                       /* argumento vazio */
            if (++s_p.argi >= s_p.argc) s_p.st = S_END;
            else s_p.st = S_ARGLEN;
        }
        break;
    case S_ARGDATA:
        if (s_p.argi == 0 && s_p.got < sizeof(s_p.arg0)) {
            s_p.arg0[s_p.got] = b;
        }
        s_p.got++;
        if (s_p.got >= s_p.arglen) {
            if (++s_p.argi >= s_p.argc) s_p.st = S_END;
            else s_p.st = S_ARGLEN;
        }
        break;
    case S_END:
        if (b == END_OF_FRAME) frame_done();
        parser_reset();
        break;
    }
}

void dabble_gp_feed(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) feed_byte(data[i]);
}

void dabble_gp_get(dabble_gp_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}

void dabble_gp_reset(void)
{
    portENTER_CRITICAL(&s_lock);
    memset(&s_state, 0, sizeof(s_state));
    portEXIT_CRITICAL(&s_lock);
    parser_reset();
}

void dabble_gp_axes(const dabble_gp_state_t *s, float *x, float *y)
{
    if (!s->analog) {            /* em modo digital 'joy_raw' é o D-pad, não joystick */
        *x = 0.0f; *y = 0.0f;
        return;
    }
    float angle_deg = (float)((s->joy_raw >> 3) * 15);
    float radius    = (float)(s->joy_raw & 0x07);
    float rad       = angle_deg * (float)M_PI / 180.0f;
    *x = radius * cosf(rad);
    *y = radius * sinf(rad);
}
