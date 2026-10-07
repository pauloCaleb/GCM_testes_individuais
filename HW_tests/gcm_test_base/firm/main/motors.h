/**
 * @file motors.h
 * @brief Controle de 2 pontes H BTS7960 (PWM + DIR por motor, EN_ALL comum).
 *
 * Convenção: duty em % com sinal. Positivo = "frente" (DIR no nível
 * Mx_DIR_FWD_LEVEL), negativo = "ré". O módulo garante:
 *   - estado seguro no boot (EN em nível inativo, PWM = 0);
 *   - limite de duty (max_duty) aplicado a qualquer alvo;
 *   - rampa (slew, %/s) e passagem obrigatória por duty 0 ao inverter DIR;
 *   - watchdog: sem "feed" por wd_ms com a ponte ativa -> STOP + failsafe.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define MOTOR_COUNT 2

typedef struct {
    float    cur[MOTOR_COUNT];   /* duty aplicado agora (%) */
    float    tgt[MOTOR_COUNT];   /* duty alvo (%), já limitado por max_duty */
    bool     en;
    float    max_duty;
    float    slew;               /* %/s; 0 = sem rampa */
    uint32_t wd_ms;              /* 0 = watchdog desligado */
    bool     failsafe;           /* true após um disparo do watchdog */
} motors_snapshot_t;

/** Chamado (no contexto da task dos motores) quando o watchdog dispara. */
typedef void (*motors_failsafe_cb_t)(void);

/** Deve ser a PRIMEIRA coisa do app_main: coloca EN e PWM em estado seguro. */
esp_err_t motors_init(void);
void motors_start_task(void);
void motors_set_failsafe_cb(motors_failsafe_cb_t cb);

/** Define o alvo (ch 0..1). Retorna o valor efetivamente aplicado (limitado por max_duty). */
float motors_set_target(int ch, float duty_pct);

/** Habilita/desabilita EN_ALL. Ao habilitar, os alvos são zerados (segurança). */
void motors_set_enable(bool en);

/** Parada imediata: duty 0 nos dois canais, sem rampa, e EN_ALL inativo. */
void motors_stop(void);

/** Reinicia o watchdog (chamar a cada comando válido recebido). */
void motors_feed_watchdog(void);

void motors_set_max_duty(float pct);
void motors_set_slew(float pct_per_s);
void motors_set_wd_ms(uint32_t ms);

void motors_get(motors_snapshot_t *out);
