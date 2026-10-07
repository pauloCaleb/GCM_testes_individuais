/* Testes no PC para o parser JSON e o formatador de telemetria.
 * Uso: make -C host_tests run
 * Também gera tests_sample/tel_samples.txt, consumido pelos testes Python da GUI. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "../main/mini_json.h"
#include "../main/proto_format.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FALHOU: %s (linha %d)\n", #c, __LINE__); fails++; } } while (0)

static void test_parse_ok(void)
{
    char l[] = " {\"cmd\":\"motor\", \"ch\":2,\"duty\":-37.5 , \"seq\":12} ";
    mj_obj_t o; double d;
    CHECK(mj_parse(l, &o));
    CHECK(strcmp(mj_str(&o, "cmd"), "motor") == 0);
    CHECK(mj_num(&o, "ch", &d) && d == 2);
    CHECK(mj_num(&o, "duty", &d) && fabs(d + 37.5) < 1e-9);
    CHECK(mj_num(&o, "seq", &d) && d == 12);
    CHECK(!mj_num(&o, "cmd", &d));      /* string não é número */
    CHECK(mj_str(&o, "ch") == NULL);    /* número não é string */
    CHECK(!mj_has(&o, "nada"));
}

static void test_parse_types(void)
{
    char l[] = "{\"a\":true,\"b\":false,\"c\":null,\"idx\":\"all\",\"e\":1e2,\"s\":\"x\\\"y\"}";
    mj_obj_t o; double d;
    CHECK(mj_parse(l, &o));
    CHECK(mj_num(&o, "a", &d) && d == 1);
    CHECK(mj_num(&o, "b", &d) && d == 0);
    CHECK(mj_num(&o, "c", &d) && d == 0);
    CHECK(strcmp(mj_str(&o, "idx"), "all") == 0);
    CHECK(mj_num(&o, "e", &d) && d == 100);
    CHECK(strcmp(mj_str(&o, "s"), "x\"y") == 0);
}

static void test_parse_bad(void)
{
    const char *bad[] = {
        "", "abc", "{", "{\"a\"}", "{\"a\":}", "{\"a\":1,}", "{\"a\":1} lixo",
        "{\"a\":nan}", "{\"a\":[1]}", "{\"a\":{\"b\":1}}", "{a:1}", "{\"a\":\"x}",
        "{\"a\":1 \"b\":2}", "{\"a\":inf}", "{\"a\":truex}",
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        char buf[128];
        strncpy(buf, bad[i], sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
        mj_obj_t o;
        if (mj_parse(buf, &o)) { printf("FALHOU: aceitou invalido: %s\n", bad[i]); fails++; }
    }
    char empty[] = "{}"; mj_obj_t o;
    CHECK(mj_parse(empty, &o) && o.n == 0);
}

static void test_too_many_fields(void)
{
    char l[256] = "{";
    for (int i = 0; i < MJ_MAX_FIELDS + 1; i++) {
        char t[16]; snprintf(t, sizeof t, "%s\"k%d\":1", i ? "," : "", i); strcat(l, t);
    }
    strcat(l, "}");
    mj_obj_t o;
    CHECK(!mj_parse(l, &o));
}

static void test_format(void)
{
    proto_tel_t t = {
        .ms = 123456, .bs = {1, 0, 1, 1}, .start = 0,
        .tof = { { 1, 523, 0, 12, 0 }, { 0 }, { 1, -1, 255, 0, 3 } },
        .led = {0, 1, 0}, .en = 1, .m_cur = {-12.5f, 0}, .m_tgt = {-30.0f, 0}, .cap = 30.0f, .failsafe = 0,
    };
    char buf[PROTO_TEL_BUF];
    int n = proto_format_tel(buf, sizeof buf, &t);
    CHECK(n > 0 && (size_t)n == strlen(buf));
    printf("%s\n", buf);
    CHECK(strstr(buf, "\"tof\":[{\"on\":1,\"mm\":523,\"st\":0,\"age\":12,\"err\":0},{\"on\":0},{\"on\":1,\"mm\":-1,\"st\":255,\"age\":0,\"err\":3}]"));
    CHECK(strstr(buf, "\"m\":[-12.5,0.0],\"mt\":[-30.0,0.0],\"cap\":30.0,\"fs\":0}"));

    char small[40];
    CHECK(proto_format_tel(small, sizeof small, &t) == -1);   /* não coube */

    /* amostras para a suíte Python */
    FILE *f = fopen("tel_samples.txt", "w");
    if (f) {
        fprintf(f, "%s\n", buf);
        t.failsafe = 1; t.en = 0; t.m_cur[1] = 30.0f; t.m_tgt[1] = 30.0f;
        t.tof[1] = (pf_tof_t){ 1, 1999, 4, 7, 0 };
        proto_format_tel(buf, sizeof buf, &t);
        fprintf(f, "%s\n", buf);
        fclose(f);
    }
}

int main(void)
{
    test_parse_ok();
    test_parse_types();
    test_parse_bad();
    test_too_many_fields();
    test_format();
    if (fails) { printf("%d falha(s)\n", fails); return 1; }
    printf("OK: todos os testes passaram\n");
    return 0;
}
