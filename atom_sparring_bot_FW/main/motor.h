#pragma once
#include <stdbool.h>

typedef enum {
    MOTOR_A = 0,   /* esquerda (IN1/IN2/EN12) */
    MOTOR_B,       /* direita  (IN3/IN4/EN34) */
    MOTOR_COUNT
} motor_id_t;

/* Configura GPIOs e PWM; motores começam parados. */
void motor_init(void);

/* speed em [-1.0, +1.0].
 *   > 0 : "frente" na convenção do joystick do firmware antigo
 *         (Motor A: IN1=LOW  IN2=HIGH | Motor B: IN3=LOW  IN4=HIGH)
 *   < 0 : "trás"
 *   = 0 : roda livre (IN=LOW/LOW, PWM=0) - igual ao firmware antigo */
void motor_set(motor_id_t m, float speed);

void motor_stop_all(void);
