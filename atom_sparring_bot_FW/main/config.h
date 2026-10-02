#pragma once

/* ------------------------------------------------------------------ */
/*  ATOM - configuração do robô                                        */
/* ------------------------------------------------------------------ */

/* Nome anunciado por BLE (o app Dabble lista por este nome).
 * Limite prático: o nome + UUID 128 bits + flags precisam caber nos 31 bytes
 * do pacote de advertising -> até ~ 10 caracteres. */
#define ROBOT_NAME          "ATOM"

/* ---- Pinos da ponte H (mesmo hardware do projeto antigo) ---- */
/* Motor A "Esquerda" */
#define PIN_IN1             19
#define PIN_IN2             5     /* strapping pin (GPIO5) */
#define PIN_EN12            18    /* PWM */
/* Motor B "Direita" */
#define PIN_IN3             12    /* strapping pin (GPIO12 / MTDI) - ver nota no main.c */
#define PIN_IN4             21
#define PIN_EN34            22    /* PWM */

/* ---- PWM (LEDC) ---- */
#define PWM_FREQ_HZ         5000
#define PWM_MAX_DUTY        255   /* 8 bits */

/* ---- Joystick ---- */
#define DEAD_ZONE           0.12f
#define STEER_GAIN          0.7f  /* "sensitividade_curva" do firmware antigo */

/* O Dabble entrega o raio do joystick como inteiro 0..7 (não -1..1).
 *
 * JOY_NORMALIZE = 0 -> comportamento IDÊNTICO ao firmware Arduino antigo:
 *                      eixos em -7..+7, o cubo estoura e o clamp satura em
 *                      +-1 já a partir de raio 1 (na prática, "liga/desliga").
 * JOY_NORMALIZE = 1 -> eixos divididos por 7 (-1..+1): a curva cúbica passa a
 *                      realmente suavizar o controle (intenção original).     */
#define JOY_NORMALIZE       0

/* Período do laço de controle dos motores */
#define CONTROL_PERIOD_MS   20
