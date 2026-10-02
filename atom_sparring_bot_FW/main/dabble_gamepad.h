#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Parser mínimo do protocolo do app Dabble (módulo Gamepad).
 *
 * Frame:  0xFF | module | function | argc | [len | data...]*argc | 0x00
 * Gamepad: module = 0x01, function = 0x01 (digital) / 0x02 (analógico) /
 *          0x03 (acelerômetro), argc = 1, len = 2, data = { botões, joy/dpad }.
 */

/* Bits do byte "botões" (value0 na biblioteca original) */
#define DABBLE_BTN_START     (1u << 0)
#define DABBLE_BTN_SELECT    (1u << 1)
#define DABBLE_BTN_TRIANGLE  (1u << 2)
#define DABBLE_BTN_CIRCLE    (1u << 3)
#define DABBLE_BTN_CROSS     (1u << 4)
#define DABBLE_BTN_SQUARE    (1u << 5)

typedef struct {
    uint8_t buttons;     /* máscara DABBLE_BTN_* */
    uint8_t joy_raw;     /* modo analógico: (ângulo/15)<<3 | raio(0..7) */
    bool    analog;      /* true se o último frame foi analógico/acelerômetro */
} dabble_gp_state_t;

/* Alimenta o parser com bytes recebidos (pode fragmentar frames à vontade). */
void dabble_gp_feed(const uint8_t *data, size_t len);

/* Cópia atômica do estado atual. */
void dabble_gp_get(dabble_gp_state_t *out);

/* Zera estado (ex.: ao desconectar) e descarta frame parcial. */
void dabble_gp_reset(void);

/* Eixos do joystick em unidades do Dabble (raio 0..7), como
 * getXaxisData()/getYaxisData() da biblioteca original.
 * Em modo digital (D-pad) retorna 0,0. */
void dabble_gp_axes(const dabble_gp_state_t *s, float *x, float *y);
