#include "mini_json.h"
#include <string.h>
#include <stdlib.h>

static char *skip_ws(char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return p;
}

/* Lê uma string JSON cuja aspa de abertura está em p. Desescapa in-place.
 * Retorna o início do conteúdo e *end aponta para depois da aspa de fechamento. */
static char *read_string(char *p, char **end)
{
    if (*p != '"') return NULL;
    char *start = ++p;
    char *w = start;
    while (*p && *p != '"') {
        if (*p == '\\') {
            p++;
            switch (*p) {
                case '"':  *w++ = '"';  break;
                case '\\': *w++ = '\\'; break;
                case '/':  *w++ = '/';  break;
                case 'n':  *w++ = '\n'; break;
                case 't':  *w++ = '\t'; break;
                case 'r':  *w++ = '\r'; break;
                default:   return NULL;
            }
            p++;
        } else {
            *w++ = *p++;
        }
    }
    if (*p != '"') return NULL;
    *end = p + 1;   /* antes de escrever o terminador (w pode ser == p) */
    *w = '\0';
    return start;
}

bool mj_parse(char *s, mj_obj_t *o)
{
    o->n = 0;
    char *p = skip_ws(s);
    if (*p != '{') return false;
    p = skip_ws(p + 1);
    if (*p == '}') {
        p = skip_ws(p + 1);
        return *p == '\0';
    }

    for (;;) {
        p = skip_ws(p);
        char *end = NULL;
        char *key = read_string(p, &end);
        if (!key) return false;
        p = skip_ws(end);
        if (*p != ':') return false;
        p = skip_ws(p + 1);

        if (o->n >= MJ_MAX_FIELDS) return false;
        mj_field_t *f = &o->f[o->n];
        f->key = key;
        f->str = NULL;
        f->num = 0;

        if (*p == '"') {
            char *v = read_string(p, &end);
            if (!v) return false;
            f->str = v;
            p = end;
        } else if (strncmp(p, "true", 4) == 0) {
            f->num = 1; p += 4;
        } else if (strncmp(p, "false", 5) == 0) {
            f->num = 0; p += 5;
        } else if (strncmp(p, "null", 4) == 0) {
            f->num = 0; p += 4;
        } else if (*p == '-' || (*p >= '0' && *p <= '9')) {
            char *e = NULL;
            f->num = strtod(p, &e);
            if (e == p) return false;
            p = e;
        } else {
            return false;
        }
        o->n++;

        p = skip_ws(p);
        if (*p == ',') { p++; continue; }
        if (*p == '}') {
            p = skip_ws(p + 1);
            return *p == '\0';
        }
        return false;
    }
}

static const mj_field_t *find(const mj_obj_t *o, const char *key)
{
    for (int i = 0; i < o->n; i++) {
        if (strcmp(o->f[i].key, key) == 0) return &o->f[i];
    }
    return NULL;
}

const char *mj_str(const mj_obj_t *o, const char *key)
{
    const mj_field_t *f = find(o, key);
    return f ? f->str : NULL;
}

bool mj_num(const mj_obj_t *o, const char *key, double *out)
{
    const mj_field_t *f = find(o, key);
    if (!f || f->str) return false;
    *out = f->num;
    return true;
}

bool mj_has(const mj_obj_t *o, const char *key)
{
    return find(o, key) != NULL;
}
