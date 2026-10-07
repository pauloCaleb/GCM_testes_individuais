#include "proto_format.h"
#include <stdio.h>

#define APPEND(...)                                                          \
    do {                                                                     \
        int _r = snprintf(buf + len, (len < (int)n) ? (n - (size_t)len) : 0, \
                          __VA_ARGS__);                                      \
        if (_r < 0) return -1;                                               \
        len += _r;                                                           \
    } while (0)

int proto_format_tel(char *buf, size_t n, const proto_tel_t *t)
{
    int len = 0;

    APPEND("{\"t\":\"tel\",\"ms\":%lu,\"bs\":[%d,%d,%d,%d],\"start\":%d,\"tof\":[",
           (unsigned long)t->ms, t->bs[0], t->bs[1], t->bs[2], t->bs[3], t->start);

    for (int i = 0; i < PF_TOF_COUNT; i++) {
        const pf_tof_t *s = &t->tof[i];
        if (i) APPEND(",");
        if (!s->on) {
            APPEND("{\"on\":0}");
        } else {
            APPEND("{\"on\":1,\"mm\":%d,\"st\":%u,\"age\":%lu,\"err\":%lu}",
                   s->mm, (unsigned)s->st, (unsigned long)s->age_ms, (unsigned long)s->err);
        }
    }

    APPEND("],\"led\":[%d,%d,%d],\"en\":%d,\"m\":[%.1f,%.1f],\"mt\":[%.1f,%.1f],\"cap\":%.1f,\"fs\":%d}",
           t->led[0], t->led[1], t->led[2], t->en,
           (double)t->m_cur[0], (double)t->m_cur[1],
           (double)t->m_tgt[0], (double)t->m_tgt[1],
           (double)t->cap, t->failsafe);

    return (len < (int)n) ? len : -1;
}
