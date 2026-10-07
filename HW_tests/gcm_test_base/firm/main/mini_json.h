/**
 * @file mini_json.h
 * @brief Parser mínimo de objetos JSON PLANOS ({"chave": valor, ...}).
 *
 * Suporta valores string, número, true/false/null. Não suporta objetos ou
 * arrays aninhados (o protocolo de comandos da GCM não precisa deles).
 * O parse é feito IN-PLACE: a linha de entrada é modificada e deve continuar
 * válida enquanto os ponteiros retornados forem usados.
 *
 * Sem dependências do ESP-IDF: pode ser testado no PC (ver host_tests/).
 */
#pragma once

#include <stdbool.h>

#define MJ_MAX_FIELDS 12

typedef struct {
    const char *key;
    const char *str;   /* != NULL se o valor era string */
    double      num;   /* número; true=1, false/null=0 */
} mj_field_t;

typedef struct {
    mj_field_t f[MJ_MAX_FIELDS];
    int n;
} mj_obj_t;

/** Faz o parse de uma linha terminada em '\0'. Retorna false se inválida. */
bool mj_parse(char *line, mj_obj_t *out);

/** Valor string da chave, ou NULL se ausente / não for string. */
const char *mj_str(const mj_obj_t *o, const char *key);

/** Valor numérico da chave. false se ausente ou se for string. */
bool mj_num(const mj_obj_t *o, const char *key, double *out);

bool mj_has(const mj_obj_t *o, const char *key);
