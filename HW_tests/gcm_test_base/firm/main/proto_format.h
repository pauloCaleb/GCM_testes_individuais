/**
 * @file proto_format.h
 * @brief Formatação do quadro de telemetria (JSON de uma linha).
 *
 * Sem dependências do ESP-IDF: testável no PC (ver host_tests/).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define PF_BS_COUNT   4
#define PF_TOF_COUNT  3
#define PF_LED_COUNT  3
#define PF_MOTOR_COUNT 2

/** Tamanho de buffer suficiente para um quadro de telemetria. */
#define PROTO_TEL_BUF 640

typedef struct {
    uint8_t  on;      /* 1 = sensor em operação */
    int      mm;      /* -1 = ainda sem leitura */
    uint8_t  st;      /* range status (0 = válido) */
    uint32_t age_ms;  /* ms desde a última leitura nova */
    uint32_t err;     /* erros I2C acumulados */
} pf_tof_t;

typedef struct {
    uint32_t ms;
    uint8_t  bs[PF_BS_COUNT];
    uint8_t  start;
    pf_tof_t tof[PF_TOF_COUNT];
    uint8_t  led[PF_LED_COUNT];
    uint8_t  en;
    float    m_cur[PF_MOTOR_COUNT];   /* duty aplicado (%), com sinal */
    float    m_tgt[PF_MOTOR_COUNT];   /* duty alvo (%), com sinal */
    float    cap;                     /* limite de duty (%) */
    uint8_t  failsafe;
} proto_tel_t;

/** Escreve uma linha JSON (sem '\n') em buf. Retorna o tamanho ou -1 se não coube. */
int proto_format_tel(char *buf, size_t n, const proto_tel_t *t);
