/**
 * main.c — GCM-PI2-2026.2 / Teste de descarga controlada da bateria do robo (v2)
 * ------------------------------------------------------------------------
 * Descarrega a bateria atraves de uma carga eletronica fixa e registra
 * tensao, corrente, potencia, Ah e Wh ate a tensao de cutoff (padrao 12 V).
 *
 * Topologia de medicao (GCM alimentada por fonte de bancada, NAO pela bateria):
 *
 *     bateria (+) --> PWR_PAD_BATT --[ACS758]--> PWR_PAD_SENS --> [rele] --> carga --> bateria (-)
 *
 *   AIN0 = PWR_VOLTAGE_SENS (divisor R8/R15 a partir de PWR_PAD_BATT) -> V_batt
 *   AIN1 = PWR_CURRENT_SENS (ACS758LCB-050B, 40 mV/A, zero em ~Vcc/2) -> I
 *   Corrente positiva = fluxo de BATT para SENS = DESCARGA.
 *
 *   O GND da bateria precisa estar ligado ao GND da GCM (referencia do divisor
 *   e do ACS758). O retorno da corrente da carga deve ir DIRETO ao negativo da
 *   bateria, sem passar por trilhas da GCM.
 *
 * Enable da carga: GPIO23 (CN15 pino 8). Inicia SEMPRE em nivel inativo.
 * Rele opcional (menuconfig): fecha antes de habilitar a carga e abre depois de desabilita-la.
 *
 * Novidades da v2:
 *   - Calibracao de ganho/offset da tensao e ganho da corrente, guardada em NVS (comandos CAL).
 *   - Re-zero automatico do ACS758 no REST e Ah/Wh corrigidos pela deriva do zero.
 *   - R0 (queda ohmica) no instante em que a carga liga e desliga.
 *   - Metadados (versao, calibracao, zeros) em eventos e no resumo.
 *   - Bluetooth Classic SPP espelhando a serial (aparece como porta COM no PC).
 *   - Numero de sequencia nas linhas D e comando SYNC para um app se anexar a um teste em andamento.
 *   - Log decimado (1 Hz) na flash, recuperavel pelo comando LOG.
 *
 * Maquina de estados (um unico task, sem race entre comandos e aquisicao):
 *
 *   IDLE --TARE--> READY --START--> REST --> DISCHARGE --cutoff/STOP--> RECOVERY --> DONE
 *                    ^                  \\__________ qualquer erro ___________> FAULT
 *                    |__________ TARE / RESET (a partir de DONE/FAULT) ___________|
 *
 * Protocolo (115200 8N1 na UART e o mesmo texto pelo Bluetooth; linhas terminadas em \n):
 *
 *   PC -> ESP32 (comandos, case-insensitive):
 *     TARE | START | STOP | RESET | CUTOFF <mV ou V> | STATUS | SYNC | LOG | HELP
 *     CAL SHOW | CAL RESET | CAL V <ref> | CAL VP1 <ref> | CAL VP2 <ref> | CAL I <ref>
 *     CAL SET <v_gain_ppm> <v_off_uV> <i_gain_ppm>
 *       (<ref> em mV/mA, ou em V/A se < 100)
 *
 *   ESP32 -> PC:
 *     H;seq;t_boot_s;t_test_s;state;v_batt_V;i_A;p_W;q_Ah;e_Wh;ain0_V;ain1_V   cabecalho
 *     D;<mesmos campos>                  uma linha por amostra
 *     E;<EVENTO>;<detalhes>              eventos (TARE_OK, REZERO, LOAD_ON, R0, FAULT, SUMMARY, CAL...)
 *     S;<chave=valor,...>                status e metadados
 *     R;<linha E original>               replay de eventos recentes (resposta ao SYNC)
 *     L;idx;t_test_s;state;v;i;q;e       registro do log em flash (resposta ao LOG)
 *   Qualquer outra linha (ESP_LOG, boot) e informativa e ignorada pelo parser.
 *
 * Seguranca:
 *   - O cutoff e decidido AQUI, no firmware, com a tensao ja calibrada: o teste termina
 *     corretamente mesmo que o PC/Bluetooth desconecte ou o Python seja fechado.
 *   - Carga desligada em: cutoff, STOP, FAULT, reset do chip (pull externo no enable).
 *   - FAULT em: sobrecorrente, ausencia de corrente com a carga ligada, falhas repetidas
 *     do ADS1115, corrente com a carga desligada (REST), bateria ausente ou abaixo do
 *     limite de partida.
 *   - Task watchdog com panic (sdkconfig.defaults).
 *
 * Toolchain: ESP-IDF v5.4.2, alvo ESP32.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "cal_store.h"
#include "flash_log.h"
#include "bt_link.h"

#if CONFIG_FREERTOS_HZ < 1000
#error "Este projeto exige CONFIG_FREERTOS_HZ=1000 (ver sdkconfig.defaults). Rode 'idf.py fullclean' e reconfigure."
#endif

#define FW_VERSION "2.0.0"

static const char *TAG = "BATT_TEST";

/* ------------------------------------------------------------------ */
/* Pinagem / barramento                                                */
/* ------------------------------------------------------------------ */

#define I2C0_SDA_GPIO        21   /* net SDA3V3 -> level shifter -> ADS1115 */
#define I2C0_SCL_GPIO        22   /* net SCL3V3 */
#define ADS1115_ADDR         0x49 /* ADDR -> +5V (R14) */
#define I2C_XFER_TIMEOUT_MS  100  /* ms puros (nao ticks) */

#define LOAD_GPIO            ((gpio_num_t)CONFIG_BATT_LOAD_GPIO)
#ifdef CONFIG_BATT_LOAD_ACTIVE_HIGH
#define LOAD_ACTIVE_LEVEL    1
#else
#define LOAD_ACTIVE_LEVEL    0
#endif

#ifdef CONFIG_BATT_RELAY_ENABLE
#define RELAY_EN             1
#define RELAY_GPIO           ((gpio_num_t)CONFIG_BATT_RELAY_GPIO)
#ifdef CONFIG_BATT_RELAY_ACTIVE_HIGH
#define RELAY_ACTIVE_LEVEL   1
#else
#define RELAY_ACTIVE_LEVEL   0
#endif
#else
#define RELAY_EN             0
#endif

#ifdef CONFIG_BATT_REZERO_AT_REST
#define REZERO_EN            1
#else
#define REZERO_EN            0
#endif

#ifdef CONFIG_BATT_BT_ENABLE
#define BT_EN                1
#else
#define BT_EN                0
#endif

#define CMD_UART             UART_NUM_0

/* ------------------------------------------------------------------ */
/* ADS1115                                                             */
/* ------------------------------------------------------------------ */

#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01

/* OS=1 | MUX (OR) | PGA=000 (+-6,144 V) | MODE=1 (single-shot) | DR=111 (860 SPS)
 * | COMP_QUE=11 (comparador desabilitado) */
#define ADS1115_CFG_BASE       0x81E3u
#define ADS1115_MUX_AIN0       (0x4u << 12)
#define ADS1115_MUX_AIN1       (0x5u << 12)
#define ADS1115_LSB_V          (6.144 / 32768.0)
#define ADS1115_POLL_STEP_MS   1
#define ADS1115_POLL_TIMEOUT   50

/* ------------------------------------------------------------------ */
/* Escalas                                                             */
/* ------------------------------------------------------------------ */

#define V_RATIO         ((double)CONFIG_BATT_V_DIV_RATIO_X10000 / 10000.0)
#define I_SENS_V_PER_A  ((double)CONFIG_BATT_I_SENS_UV_PER_A / 1e6)
#define ACS_NOMINAL_ZERO_V 2.5   /* Vcc/2 com Vcc = 5 V */

#define CAL_I_SAMPLES   30       /* amostras (3 s a 10 Hz) para calibrar a corrente */
#define EV_N            16       /* eventos guardados para o SYNC */
#define EV_LEN          704

/* ------------------------------------------------------------------ */
/* Estado do teste                                                     */
/* ------------------------------------------------------------------ */

typedef enum {
    ST_IDLE = 0,   /* sem tara */
    ST_READY,      /* tara feita, aguardando START */
    ST_REST,       /* carga desligada, medindo a tensao e o zero em repouso */
    ST_DISCHARGE,  /* carga ligada */
    ST_RECOVERY,   /* carga desligada apos cutoff/STOP, registrando a recuperacao */
    ST_DONE,       /* teste encerrado */
    ST_FAULT,      /* erro: carga desligada */
} test_state_t;

static const char *const STATE_NAME[] = {
    "IDLE", "READY", "REST", "DISCHARGE", "RECOVERY", "DONE", "FAULT"
};
#define N_STATES ((int)(sizeof(STATE_NAME) / sizeof(STATE_NAME[0])))

static test_state_t s_state = ST_IDLE;

static int    s_cutoff_mv = CONFIG_BATT_CUTOFF_MV;
static bool   s_tared = false;
static double s_zero_v = ACS_NOMINAL_ZERO_V;     /* zero em uso */
static double s_zero_tare_v = ACS_NOMINAL_ZERO_V;
static double s_zero_rest_v = NAN;
static bool   s_load_on = false;
static bool   s_relay_closed = false;
static int    s_adc_fail = 0;
static bool   s_resync = false;   /* a tara bloqueia ~1 s: reancora o periodo do loop */
static uint32_t s_seq = 0;
static uint16_t s_test_id = 0;

/* Tempos (esp_timer, us) */
static int64_t s_last_us;       /* ultima amostra valida */
static int64_t s_rest_t0_us;
static int64_t s_load_on_us;
static int64_t s_rec_t0_us;

/* Medidas e acumuladores */
static double s_last_v, s_last_i;
static double s_t_test_s;       /* tempo desde que a carga foi ligada (congela no fim) */
static double s_q_ah, s_e_wh, s_vt_vh;          /* Ah, Wh e integral de V (V*h) */
static double s_prev_i, s_prev_p, s_prev_v;
static double s_v_rest, s_v_min, s_v_cut, s_v_rec;
static double s_i_zero_end_a, s_off_start_a;
static double s_q_corr, s_e_corr, s_drift_a;
static double s_dur_s;
static double s_rint_mohm, s_r0_on_mohm, s_r0_off_mohm;
static const char *s_end_reason = "-";

static double s_rest_sv, s_rest_zsum; static int s_rest_n; static int s_rest_bad;
static double s_rint_sv, s_rint_si; static int s_rint_n; static bool s_rint_done;
static double s_rec_sv, s_rec_si; static int s_rec_n;
static int s_below_cnt, s_oc_cnt, s_nc_cnt;
static bool s_r0_on_pending, s_r0_off_pending; static int s_r0_off_try;
static double s_v_before_off, s_i_before_off;

/* Calibracao da corrente em andamento (CAL I) e ponto 1 da tensao (CAL VP1) */
static bool   s_cal_i_active; static double s_cal_i_ref_a, s_cal_i_sum; static int s_cal_i_n;
static bool   s_vp1_set; static double s_vp1_raw, s_vp1_ref;

/* Log em flash / download */
static bool     s_log_run;
static int64_t  s_log_last_us;
static bool     s_dump_active;
static uint32_t s_dump_idx;
static uint16_t s_dump_test_id;

/* Eventos recentes (resposta ao SYNC) */
static char s_ev[EV_N][EV_LEN];
static int  s_ev_head, s_ev_count;

/* Linhas de comando (uma por origem, para nao misturar bytes de UART e Bluetooth) */
typedef struct { char buf[96]; size_t len; } line_asm_t;
static line_asm_t s_rx_uart, s_rx_bt;

/* I2C */
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_ads;

/* ------------------------------------------------------------------ */
/* Saida (UART + Bluetooth)                                            */
/* ------------------------------------------------------------------ */

static char s_line[820];
static char s_kv[720];

static void ring_store(const char *line)
{
    if (strncmp(line, "E;", 2) != 0 || !strncmp(line, "E;WARN;ADC_ERR", 14) || !strncmp(line, "E;WARN;PERIOD", 13) ||
        !strncmp(line, "E;SYNC_", 7) || !strncmp(line, "E;LOGDUMP", 9) ||
        !strncmp(line, "E;BOOT", 6)) {
        return;
    }
    snprintf(s_ev[s_ev_head], EV_LEN, "%s", line);
    s_ev_head = (s_ev_head + 1) % EV_N;
    if (s_ev_count < EV_N) {
        s_ev_count++;
    }
}

static void out_line(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_line, sizeof(s_line) - 2, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n > (int)sizeof(s_line) - 3) {
        n = (int)sizeof(s_line) - 3;
    }
    s_line[n] = '\0';
    ring_store(s_line);
    s_line[n++] = '\n';
    s_line[n] = '\0';
    fputs(s_line, stdout);
    fflush(stdout);
    bt_link_send(s_line, (size_t)n, s_line[0] == 'D' || s_line[0] == 'L');
}

static void print_header(void)
{
    out_line("H;seq;t_boot_s;t_test_s;state;v_batt_V;i_A;p_W;q_Ah;e_Wh;ain0_V;ain1_V");
}

static void build_kv(void)
{
    const cal_t *c = cal_get();
    snprintf(s_kv, sizeof(s_kv),
             "fw=%s,state=%s,cutoff_mV=%d,cutoff_min_mV=%d,cutoff_max_mV=%d,tared=%d,"
             "zero_V=%.5f,zero_tare_V=%.5f,zero_rest_V=%.5f,load=%d,relay_en=%d,relay_closed=%d,"
             "period_ms=%d,oversample=%d,max_current_mA=%d,v_ratio=%.4f,i_sens_uV_per_A=%d,"
             "v_gain_ppm=%d,v_off_uV=%d,i_gain_ppm=%d,bt=%d,bt_conn=%d,bt_drop=%u,"
             "log=%d,log_n=%u,test_id=%u,up_s=%u",
             FW_VERSION, STATE_NAME[s_state], s_cutoff_mv,
             CONFIG_BATT_CUTOFF_MIN_MV, CONFIG_BATT_CUTOFF_MAX_MV, s_tared ? 1 : 0,
             s_zero_v, s_zero_tare_v, s_zero_rest_v, s_load_on ? 1 : 0, RELAY_EN,
             s_relay_closed ? 1 : 0, CONFIG_BATT_SAMPLE_PERIOD_MS, CONFIG_BATT_OVERSAMPLE,
             CONFIG_BATT_MAX_CURRENT_MA, V_RATIO, CONFIG_BATT_I_SENS_UV_PER_A,
             (int)c->v_gain_ppm, (int)c->v_off_uv, (int)c->i_gain_ppm, BT_EN,
             bt_link_connected() ? 1 : 0, (unsigned)bt_link_dropped(),
             flog_available() ? 1 : 0, (unsigned)flog_count(), (unsigned)s_test_id,
             (unsigned)(esp_timer_get_time() / 1000000));
}

static void print_status(void)
{
    build_kv();
    out_line("S;%s", s_kv);
}

static void print_meta(void)
{
    build_kv();
    out_line("E;META;%s", s_kv);
}

/* ------------------------------------------------------------------ */
/* Carga eletronica (GPIO23) e rele opcional                           */
/* ------------------------------------------------------------------ */

static void load_set(bool on)
{
    gpio_set_level(LOAD_GPIO, on ? LOAD_ACTIVE_LEVEL : !LOAD_ACTIVE_LEVEL);
    s_load_on = on;
}

static void out_pin_init(gpio_num_t pin, int inactive_level)
{
    /* Grava o nivel inativo no latch ANTES de configurar o pino como saida,
     * para nao gerar pulso ativo na inicializacao. */
    gpio_set_level(pin, inactive_level);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << (int)pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(pin, inactive_level);
}

static void relay_set(bool closed)
{
#if RELAY_EN
    gpio_set_level(RELAY_GPIO, closed ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
#endif
    s_relay_closed = closed;
}

/* Liga: fecha o rele, espera o ricochete, habilita a carga. */
static void power_path_on(void)
{
#if RELAY_EN
    relay_set(true);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_BATT_RELAY_CLOSE_DELAY_MS));
    s_resync = true;
#endif
    load_set(true);
}

/* Desliga: desabilita a carga e so depois abre o rele (nunca interrompe corrente alta). */
static void power_path_off(void)
{
    load_set(false);
#if RELAY_EN
    if (s_relay_closed) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BATT_RELAY_OPEN_DELAY_MS));
        relay_set(false);
        s_resync = true;
    }
#endif
}

/* ------------------------------------------------------------------ */
/* ADS1115                                                             */
/* ------------------------------------------------------------------ */

static void i2c_recover_bus(void)
{
    esp_err_t err = i2c_master_bus_reset(s_bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_reset falhou: %s", esp_err_to_name(err));
    }
}

static esp_err_t ads_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    esp_err_t err = i2c_master_transmit(s_ads, buf, sizeof(buf), I2C_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        i2c_recover_bus();
    }
    return err;
}

static esp_err_t ads_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t rx[2] = { 0 };
    esp_err_t err = i2c_master_transmit_receive(s_ads, &reg, 1, rx, sizeof(rx),
                                                I2C_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        i2c_recover_bus();
        return err;
    }
    *value = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

/* Conversao single-shot com polling do bit OS (sem delay fixo). */
static esp_err_t ads_read_channel(uint16_t mux, double *out_volts)
{
    esp_err_t err = ads_write_reg(ADS1115_REG_CONFIG, ADS1115_CFG_BASE | mux);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t status = 0;
    int waited = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(ADS1115_POLL_STEP_MS));
        waited += ADS1115_POLL_STEP_MS;
        err = ads_read_reg(ADS1115_REG_CONFIG, &status);
        if (err != ESP_OK) {
            return err;
        }
    } while (!(status & 0x8000u) && waited < ADS1115_POLL_TIMEOUT);

    if (!(status & 0x8000u)) {
        return ESP_ERR_TIMEOUT;
    }

    uint16_t raw;
    err = ads_read_reg(ADS1115_REG_CONVERSION, &raw);
    if (err != ESP_OK) {
        return err;
    }
    *out_volts = (double)(int16_t)raw * ADS1115_LSB_V;
    return ESP_OK;
}

/* Media de BATT_OVERSAMPLE conversoes por canal (canais intercalados). */
static esp_err_t measure(double *ain0, double *ain1)
{
    double s0 = 0.0, s1 = 0.0, v;
    for (int k = 0; k < CONFIG_BATT_OVERSAMPLE; k++) {
        esp_err_t err = ads_read_channel(ADS1115_MUX_AIN0, &v);
        if (err != ESP_OK) {
            return err;
        }
        s0 += v;
        err = ads_read_channel(ADS1115_MUX_AIN1, &v);
        if (err != ESP_OK) {
            return err;
        }
        s1 += v;
    }
    *ain0 = s0 / CONFIG_BATT_OVERSAMPLE;
    *ain1 = s1 / CONFIG_BATT_OVERSAMPLE;
    return ESP_OK;
}

/* Tensao BRUTA do divisor (sem ganho/offset de calibracao), media de 40 conversoes. */
static bool measure_raw_v(double *raw_v)
{
    double sum = 0.0, v;
    int n = 0;
    for (int k = 0; k < 40; k++) {
        esp_task_wdt_reset();
        if (ads_read_channel(ADS1115_MUX_AIN0, &v) == ESP_OK) {
            sum += v;
            n++;
        }
    }
    s_resync = true;
    if (n < 30) {
        return false;
    }
    *raw_v = sum / n * V_RATIO;
    return true;
}

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C0_SDA_GPIO,
        .scl_io_num = I2C0_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, /* pull-ups R9-R12 ja existem na placa */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = CONFIG_BATT_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_ads));
}

/* ------------------------------------------------------------------ */
/* Transicoes de estado                                                */
/* ------------------------------------------------------------------ */

static void clear_test_data(void)
{
    s_t_test_s = 0.0;
    s_q_ah = s_e_wh = s_vt_vh = 0.0;
    s_prev_i = s_prev_p = s_prev_v = 0.0;
    s_v_rest = s_v_min = s_v_cut = s_v_rec = NAN;
    s_i_zero_end_a = NAN;
    s_off_start_a = 0.0;
    s_q_corr = s_e_corr = s_drift_a = NAN;
    s_rint_mohm = s_r0_on_mohm = s_r0_off_mohm = NAN;
    s_zero_rest_v = NAN;
    s_dur_s = 0.0;
    s_end_reason = "-";
    s_rest_sv = s_rest_zsum = 0.0; s_rest_n = 0; s_rest_bad = 0;
    s_rint_sv = s_rint_si = 0.0; s_rint_n = 0; s_rint_done = false;
    s_rec_sv = s_rec_si = 0.0; s_rec_n = 0;
    s_below_cnt = s_oc_cnt = s_nc_cnt = 0;
    s_r0_on_pending = s_r0_off_pending = false; s_r0_off_try = 0;
    s_cal_i_active = false;
}

static void print_summary(void)
{
    const cal_t *c = cal_get();
    double i_avg = (s_dur_s > 0.0) ? (s_q_ah * 3600.0 / s_dur_s) : 0.0;
    out_line("E;SUMMARY;end=%s,test_id=%u,dur_s=%.1f,q_Ah=%.6f,q_corr_Ah=%.6f,e_Wh=%.5f,"
             "e_corr_Wh=%.5f,i_avg_A=%.4f,v_rest_V=%.4f,v_min_V=%.4f,v_cut_V=%.4f,v_rec_V=%.4f,"
             "r0_on_mohm=%.1f,r0_off_mohm=%.1f,rint_mohm=%.1f,drift_mA=%.2f,"
             "i_zero_start_mA=%.1f,i_zero_end_mA=%.1f,zero_tare_V=%.5f,zero_rest_V=%.5f,"
             "cutoff_mV=%d,v_gain_ppm=%d,v_off_uV=%d,i_gain_ppm=%d,fw=%s",
             s_end_reason, (unsigned)s_test_id, s_dur_s, s_q_ah, s_q_corr, s_e_wh, s_e_corr,
             i_avg, s_v_rest, s_v_min, s_v_cut, s_v_rec, s_r0_on_mohm, s_r0_off_mohm,
             s_rint_mohm, s_drift_a * 1000.0, s_off_start_a * 1000.0, s_i_zero_end_a * 1000.0,
             s_zero_tare_v, s_zero_rest_v, s_cutoff_mv, (int)c->v_gain_ppm, (int)c->v_off_uv,
             (int)c->i_gain_ppm, FW_VERSION);
}

static void enter_fault(const char *reason)
{
    bool was_discharging = (s_state == ST_DISCHARGE);
    bool test_ran = was_discharging || (s_state == ST_RECOVERY);

    power_path_off();
    if (was_discharging) {
        s_dur_s = (double)(esp_timer_get_time() - s_load_on_us) / 1e6;
        s_v_cut = s_last_v;
    }
    s_cal_i_active = false;
    s_end_reason = reason;
    s_state = ST_FAULT;
    out_line("E;FAULT;%s", reason);
    if (test_ran) {
        print_summary();
    }
}

static void begin_recovery(const char *reason, int64_t now_us)
{
    power_path_off();
    s_dur_s = (double)(now_us - s_load_on_us) / 1e6;
    s_v_cut = s_last_v;
    s_end_reason = reason;
    s_rec_t0_us = now_us;
    s_rec_sv = s_rec_si = 0.0;
    s_rec_n = 0;
    s_cal_i_active = false;
    s_r0_off_pending = true;
    s_r0_off_try = 0;
    s_v_before_off = s_last_v;
    s_i_before_off = s_last_i;
    s_state = ST_RECOVERY;
    out_line("E;LOAD_OFF;%s;v_V=%.4f,i_A=%.4f,q_Ah=%.6f,e_Wh=%.5f,t_s=%.1f",
             reason, s_last_v, s_last_i, s_q_ah, s_e_wh, s_dur_s);
}

static void finish_test(void)
{
    if (s_rec_n > 0) {
        s_v_rec = s_rec_sv / s_rec_n;
        s_i_zero_end_a = s_rec_si / s_rec_n;
    } else {
        s_v_rec = s_last_v;
        s_i_zero_end_a = s_last_i;
    }
    /* Deriva do zero: erro de offset entre o inicio (off_start) e o fim (medido com a carga
     * desligada). Supondo variacao linear, o erro medio vale a media dos dois extremos. */
    if (fabs(s_i_zero_end_a) * 1000.0 <= CONFIG_BATT_REZERO_MAX_MA) {
        s_drift_a = 0.5 * (s_off_start_a + s_i_zero_end_a);
        s_q_corr = s_q_ah - s_drift_a * s_dur_s / 3600.0;
        s_e_corr = s_e_wh - s_drift_a * s_vt_vh;
    } else {
        /* corrente residual grande com a carga desligada: nao e deriva, nao corrige */
        out_line("E;WARN;DRIFT_TOO_LARGE;i_zero_end_mA=%.1f,max_mA=%d",
                 s_i_zero_end_a * 1000.0, CONFIG_BATT_REZERO_MAX_MA);
    }
    s_state = ST_DONE;
    out_line("E;DONE;v_rec_V=%.4f", s_v_rec);
    print_summary();
}

static void start_discharge(int64_t now_us)
{
    (void)now_us;
    power_path_on();
    s_load_on_us = esp_timer_get_time();
    s_last_us = s_load_on_us;
    s_prev_i = s_prev_p = 0.0;
    s_prev_v = s_v_rest;
    s_q_ah = s_e_wh = s_vt_vh = 0.0;
    s_v_min = s_v_rest;
    s_below_cnt = s_oc_cnt = s_nc_cnt = 0;
    s_rint_sv = s_rint_si = 0.0; s_rint_n = 0; s_rint_done = false;
    s_r0_on_pending = true;
    s_t_test_s = 0.0;
    s_state = ST_DISCHARGE;
    out_line("E;LOAD_ON;v_rest_V=%.4f,cutoff_mV=%d", s_v_rest, s_cutoff_mv);
}

/* ------------------------------------------------------------------ */
/* Processamento de uma amostra                                        */
/* ------------------------------------------------------------------ */

static void process_sample(double ain0, double ain1, int64_t now_us)
{
    const cal_t *cal = cal_get();
    const double v = ain0 * V_RATIO * ((double)cal->v_gain_ppm / 1e6) + (double)cal->v_off_uv / 1e6;
    const double i_raw = (ain1 - s_zero_v) / I_SENS_V_PER_A;          /* A, sem ganho */
    const double i = i_raw * ((double)cal->i_gain_ppm / 1e6);         /* A (+ = descarga) */
    const double p = v * i;                                           /* W */
    s_last_v = v;
    s_last_i = i;

    switch (s_state) {

    case ST_REST: {
        if (fabs(i) * 1000.0 > CONFIG_BATT_REST_MAX_CURRENT_MA) {
            if (++s_rest_bad >= 5) {
                enter_fault("CURRENT_WITH_LOAD_OFF");
            }
            break;
        }
        s_rest_bad = 0;
        s_rest_sv += v;
        s_rest_zsum += ain1;
        s_rest_n++;
        if ((now_us - s_rest_t0_us) / 1000 >= CONFIG_BATT_REST_MS) {
            s_v_rest = s_rest_sv / s_rest_n;
            s_zero_rest_v = s_rest_zsum / s_rest_n;
            const double vr_mv = s_v_rest * 1000.0;
            if (vr_mv < CONFIG_BATT_PRESENT_MIN_MV) {
                enter_fault("NO_BATTERY");
            } else if (vr_mv <= (double)s_cutoff_mv + CONFIG_BATT_START_MARGIN_MV) {
                enter_fault("BATTERY_BELOW_START_LIMIT");
            } else {
                /* Zero medido agora, com a carga desligada, contra o zero em uso (tara). */
                const double delta_a = (s_zero_rest_v - s_zero_v) / I_SENS_V_PER_A;
                int applied = 0;
                s_off_start_a = 0.0;
                if (fabs(delta_a) * 1000.0 <= CONFIG_BATT_REZERO_MAX_MA) {
                    /* diferenca pequena: e deriva do sensor, nao corrente real */
#if REZERO_EN
                    s_zero_v = s_zero_rest_v;
                    applied = 1;
#else
                    s_off_start_a = delta_a * ((double)cal->i_gain_ppm / 1e6);
#endif
                } else {
                    out_line("E;WARN;REZERO_REJECTED;delta_mA=%.1f,max_mA=%d",
                             delta_a * 1000.0, CONFIG_BATT_REZERO_MAX_MA);
                }
                out_line("E;REZERO;tare_zero_V=%.5f,rest_zero_V=%.5f,delta_mA=%.1f,applied=%d",
                         s_zero_tare_v, s_zero_rest_v, delta_a * 1000.0, applied);
                start_discharge(now_us);
            }
        }
        break;
    }

    case ST_DISCHARGE: {
        const double dt = (double)(now_us - s_last_us) / 1e6;
        s_q_ah += 0.5 * (i + s_prev_i) * dt / 3600.0;
        s_e_wh += 0.5 * (p + s_prev_p) * dt / 3600.0;
        s_vt_vh += 0.5 * (v + s_prev_v) * dt / 3600.0;
        s_prev_i = i;
        s_prev_p = p;
        s_prev_v = v;
        s_t_test_s = (double)(now_us - s_load_on_us) / 1e6;
        if (v < s_v_min) {
            s_v_min = v;
        }
        const int64_t t_on_ms = (now_us - s_load_on_us) / 1000;

        /* R0 no instante em que a carga liga (primeira amostra com a carga ativa) */
        if (s_r0_on_pending) {
            s_r0_on_pending = false;
            if (i > 0.5) {
                s_r0_on_mohm = (s_v_rest - v) / i * 1000.0;
                out_line("E;R0;on_mohm=%.1f,v_V=%.4f,i_A=%.4f", s_r0_on_mohm, v, i);
            }
        }

        /* Calibracao da corrente (CAL I): media de CAL_I_SAMPLES amostras */
        if (s_cal_i_active) {
            s_cal_i_sum += i_raw;
            if (++s_cal_i_n >= CAL_I_SAMPLES) {
                s_cal_i_active = false;
                const double avg = s_cal_i_sum / s_cal_i_n;
                cal_t c = *cal_get();
                if (avg < 0.3) {
                    out_line("E;ERR;CAL_I_CURRENT_TOO_LOW;measured_A=%.3f", avg);
                } else {
                    c.i_gain_ppm = (int32_t)lround(s_cal_i_ref_a / avg * 1e6);
                    if (cal_set(&c) == ESP_OK) {
                        out_line("E;CAL;I;raw_A=%.4f,ref_A=%.4f,i_gain_ppm=%d",
                                 avg, s_cal_i_ref_a, (int)c.i_gain_ppm);
                        print_status();
                    } else {
                        out_line("E;ERR;CAL_OUT_OF_RANGE;i_gain_ppm=%d", (int)c.i_gain_ppm);
                    }
                }
            }
        }

        /* Sobrecorrente */
        if (i * 1000.0 > CONFIG_BATT_MAX_CURRENT_MA) {
            if (++s_oc_cnt >= 3) {
                enter_fault("OVERCURRENT");
                break;
            }
        } else {
            s_oc_cnt = 0;
        }

        /* Sem corrente com a carga ligada */
        if (CONFIG_BATT_MIN_CURRENT_MA > 0 && t_on_ms >= CONFIG_BATT_LOAD_SETTLE_MS) {
            if (i * 1000.0 < CONFIG_BATT_MIN_CURRENT_MA) {
                if (++s_nc_cnt >= 5) {
                    enter_fault("NO_CURRENT");
                    break;
                }
            } else {
                s_nc_cnt = 0;
            }
        }

        /* Resistencia aparente (janela de 1 s apos o assentamento; inclui polarizacao) */
        if (!s_rint_done && t_on_ms >= CONFIG_BATT_LOAD_SETTLE_MS) {
            if (t_on_ms < CONFIG_BATT_LOAD_SETTLE_MS + 1000) {
                s_rint_sv += v;
                s_rint_si += i;
                s_rint_n++;
            } else {
                s_rint_done = true;
                if (s_rint_n > 0 && (s_rint_si / s_rint_n) > 0.1) {
                    const double vl = s_rint_sv / s_rint_n;
                    const double il = s_rint_si / s_rint_n;
                    s_rint_mohm = (s_v_rest - vl) / il * 1000.0;
                    out_line("E;RINT;rint_mohm=%.1f,v_load_V=%.4f,i_load_A=%.4f",
                             s_rint_mohm, vl, il);
                }
            }
        }

        /* Cutoff (tensao ja calibrada) */
        if (v * 1000.0 < (double)s_cutoff_mv) {
            if (++s_below_cnt >= CONFIG_BATT_CUTOFF_CONSEC_SAMPLES) {
                begin_recovery("CUTOFF", now_us);
                break;
            }
        } else {
            s_below_cnt = 0;
        }

        /* Duracao maxima */
        if (CONFIG_BATT_MAX_TEST_S > 0 && s_t_test_s >= CONFIG_BATT_MAX_TEST_S) {
            begin_recovery("TIMEOUT", now_us);
        }
        break;
    }

    case ST_RECOVERY: {
        s_t_test_s = (double)(now_us - s_load_on_us) / 1e6;

        /* R0 no desligamento: salto de tensao na primeira amostra sem corrente */
        if (s_r0_off_pending) {
            if (s_i_before_off > 0.5 && i < 0.1 * s_i_before_off) {
                s_r0_off_pending = false;
                s_r0_off_mohm = (v - s_v_before_off) / s_i_before_off * 1000.0;
                out_line("E;R0;off_mohm=%.1f,v_V=%.4f,i_before_A=%.4f",
                         s_r0_off_mohm, v, s_i_before_off);
            } else if (++s_r0_off_try > 10) {
                s_r0_off_pending = false;
            }
        }

        const int64_t el_ms = (now_us - s_rec_t0_us) / 1000;
        const int64_t rec_ms = (int64_t)CONFIG_BATT_RECOVERY_S * 1000;
        const int64_t win_start = (rec_ms > 5000) ? (rec_ms - 5000) : 0;
        if (el_ms >= win_start) {          /* media dos ultimos 5 s: tensao e zero final */
            s_rec_sv += v;
            s_rec_si += i;
            s_rec_n++;
        }
        if (el_ms >= rec_ms) {
            finish_test();
        }
        break;
    }

    default:
        break;   /* IDLE, READY, DONE, FAULT: so monitora */
    }

    s_last_us = now_us;

    out_line("D;%u;%.3f;%.3f;%s;%.4f;%.4f;%.3f;%.6f;%.5f;%.5f;%.5f",
             (unsigned)s_seq++, (double)now_us / 1e6, s_t_test_s, STATE_NAME[s_state],
             v, i, p, s_q_ah, s_e_wh, ain0, ain1);
}

/* ------------------------------------------------------------------ */
/* Log em flash e download                                             */
/* ------------------------------------------------------------------ */

static void log_tick(int64_t now_us)
{
    if (!s_log_run) {
        return;
    }
    const bool final = (s_state == ST_DONE || s_state == ST_FAULT);
    if (!final && (now_us - s_log_last_us) < (int64_t)CONFIG_BATT_LOG_INTERVAL_MS * 1000) {
        return;
    }
    s_log_last_us = now_us;
    esp_err_t e = flog_append((uint8_t)s_state, (float)s_t_test_s, (float)s_last_v,
                              (float)s_last_i, (float)s_q_ah, (float)s_e_wh);
    if (e != ESP_OK) {
        out_line("E;WARN;LOG_WRITE;%s", esp_err_to_name(e));
        s_log_run = false;
        return;
    }
    if (final) {
        s_log_run = false;
    }
}

static void dump_tick(void)
{
    if (!s_dump_active) {
        return;
    }
    for (int k = 0; k < CONFIG_BATT_LOG_DUMP_PER_TICK; k++) {
        if (bt_link_free_slots() < 24) {
            return;                      /* link ocupado: continua no proximo ciclo */
        }
        flog_rec_t r;
        if (!flog_read(s_dump_test_id, s_dump_idx, &r)) {
            out_line("E;LOGDUMP_END;n=%u,test_id=%u", (unsigned)s_dump_idx, (unsigned)s_dump_test_id);
            s_dump_active = false;
            return;
        }
        out_line("L;%u;%.3f;%s;%.4f;%.4f;%.6f;%.5f", (unsigned)r.idx, r.t_test_s,
                 r.state < N_STATES ? STATE_NAME[r.state] : "?", r.v_batt_v, r.i_a,
                 r.q_ah, r.e_wh);
        s_dump_idx++;
    }
}

static void do_sync(void)
{
    out_line("E;SYNC_BEGIN");
    print_header();
    print_status();
    for (int k = 0; k < s_ev_count; k++) {
        int idx = (s_ev_head - s_ev_count + k + EV_N) % EV_N;
        out_line("R;%s", s_ev[idx]);
    }
    out_line("E;SYNC_END");
}

/* ------------------------------------------------------------------ */
/* Comandos                                                            */
/* ------------------------------------------------------------------ */

static void upper(char *s)
{
    for (; *s; s++) {
        *s = (char)toupper((unsigned char)*s);
    }
}

static bool idle_like(void)
{
    return s_state == ST_IDLE || s_state == ST_READY || s_state == ST_DONE || s_state == ST_FAULT;
}

/* Aceita mV/mA (>= 100) ou V/A (< 100) e devolve em V/A. */
static bool parse_unit(const char *s, double *out)
{
    if (s == NULL) {
        return false;
    }
    char *end = NULL;
    double x = strtod(s, &end);
    if (end == s || !isfinite(x) || x <= 0.0) {
        return false;
    }
    *out = (x >= 100.0) ? x / 1000.0 : x;
    return true;
}

static void cmd_tare(void)
{
    if (s_state == ST_REST || s_state == ST_DISCHARGE || s_state == ST_RECOVERY) {
        out_line("E;ERR;TARE_NOT_ALLOWED_IN_%s", STATE_NAME[s_state]);
        return;
    }
    power_path_off();
    out_line("E;TARE_START;samples=%d", CONFIG_BATT_TARE_SAMPLES);

    double sum = 0.0, sum2 = 0.0, v;
    int n = 0;
    for (int k = 0; k < CONFIG_BATT_TARE_SAMPLES; k++) {
        esp_task_wdt_reset();
        if (ads_read_channel(ADS1115_MUX_AIN1, &v) == ESP_OK) {
            sum += v;
            sum2 += v * v;
            n++;
        }
    }
    if (n < (CONFIG_BATT_TARE_SAMPLES * 9) / 10) {
        out_line("E;ERR;TARE_ADC_FAILED;valid=%d/%d", n, CONFIG_BATT_TARE_SAMPLES);
        return;
    }
    const double mean = sum / n;
    double var = sum2 / n - mean * mean;
    if (var < 0.0) {
        var = 0.0;
    }
    const double noise_ma = sqrt(var) / I_SENS_V_PER_A * 1000.0;

    if (fabs(mean - ACS_NOMINAL_ZERO_V) * 1000.0 > CONFIG_BATT_TARE_ZERO_TOL_MV) {
        out_line("E;ERR;TARE_ZERO_OUT_OF_RANGE;zero_V=%.5f,nominal_V=%.3f,tol_mV=%d",
                 mean, ACS_NOMINAL_ZERO_V, CONFIG_BATT_TARE_ZERO_TOL_MV);
        return;
    }

    s_zero_v = mean;
    s_zero_tare_v = mean;
    s_tared = true;
    s_resync = true;
    s_log_run = false;
    clear_test_data();
    s_state = ST_READY;
    out_line("E;TARE_OK;zero_V=%.5f,noise_mA=%.1f,samples=%d", mean, noise_ma, n);
    print_status();
}

static void cmd_start(void)
{
    if (s_state != ST_READY) {
        out_line("E;ERR;START_REQUIRES_READY;state=%s", STATE_NAME[s_state]);
        return;
    }
    clear_test_data();
    s_test_id = cal_next_test_id();
    s_dump_active = false;
    s_log_run = false;
    if (flog_available()) {
        flog_begin(s_test_id);
        s_log_run = true;
        s_log_last_us = 0;
    }
    s_rest_t0_us = esp_timer_get_time();
    s_adc_fail = 0;
    power_path_off();
    s_state = ST_REST;
    out_line("E;REST_START;rest_ms=%d,cutoff_mV=%d", CONFIG_BATT_REST_MS, s_cutoff_mv);
    print_meta();
}

static void cmd_stop(void)
{
    switch (s_state) {
    case ST_REST:
        power_path_off();
        s_state = ST_READY;
        s_log_run = false;
        out_line("E;STOP;aborted_in_rest");
        break;
    case ST_DISCHARGE:
        begin_recovery("USER_STOP", esp_timer_get_time());
        break;
    case ST_RECOVERY:
        finish_test();
        break;
    default:
        out_line("E;ERR;STOP_NOTHING_RUNNING;state=%s", STATE_NAME[s_state]);
        break;
    }
}

static void cmd_reset(void)
{
    if (s_state != ST_DONE && s_state != ST_FAULT) {
        out_line("E;ERR;RESET_ONLY_FROM_DONE_OR_FAULT;state=%s", STATE_NAME[s_state]);
        return;
    }
    power_path_off();
    clear_test_data();
    s_log_run = false;
    s_state = s_tared ? ST_READY : ST_IDLE;
    out_line("E;RESET;state=%s", STATE_NAME[s_state]);
    print_status();
}

static void cmd_cutoff(const char *arg)
{
    if (s_state == ST_REST || s_state == ST_DISCHARGE) {
        out_line("E;ERR;CUTOFF_LOCKED_IN_%s", STATE_NAME[s_state]);
        return;
    }
    if (arg == NULL) {
        out_line("E;ERR;CUTOFF_NEEDS_VALUE");
        return;
    }
    char *end = NULL;
    double val = strtod(arg, &end);
    if (end == arg || !isfinite(val)) {
        out_line("E;ERR;CUTOFF_BAD_VALUE;%s", arg);
        return;
    }
    if (val < 100.0) {
        val *= 1000.0;   /* aceita volts */
    }
    const int mv = (int)(val + 0.5);
    if (mv < CONFIG_BATT_CUTOFF_MIN_MV || mv > CONFIG_BATT_CUTOFF_MAX_MV) {
        out_line("E;ERR;CUTOFF_OUT_OF_RANGE;requested_mV=%d,min_mV=%d,max_mV=%d",
                 mv, CONFIG_BATT_CUTOFF_MIN_MV, CONFIG_BATT_CUTOFF_MAX_MV);
        return;
    }
    s_cutoff_mv = mv;
    out_line("E;CUTOFF_SET;cutoff_mV=%d", s_cutoff_mv);
    print_status();
}

static void cmd_log(void)
{
    if (!flog_available()) {
        out_line("E;ERR;LOG_UNAVAILABLE");
        return;
    }
    s_dump_test_id = s_test_id;
    s_dump_idx = 0;
    s_dump_active = true;
    out_line("E;LOGDUMP_START;test_id=%u", (unsigned)s_dump_test_id);
}

/* CAL <sub> ...: calibracao de ganho/offset (tensao) e ganho (corrente) */
static void cmd_cal(char **t, int nt)
{
    if (nt < 1) {
        out_line("E;ERR;CAL_NEEDS_SUBCOMMAND");
        return;
    }
    upper(t[0]);
    const char *sub = t[0];
    cal_t c = *cal_get();

    if (!strcmp(sub, "SHOW")) {
        print_status();
        out_line("E;CAL;SHOW;v_gain_ppm=%d,v_off_uV=%d,i_gain_ppm=%d",
                 (int)c.v_gain_ppm, (int)c.v_off_uv, (int)c.i_gain_ppm);
        return;
    }
    if (!strcmp(sub, "RESET")) {
        if (!idle_like()) {
            out_line("E;ERR;CAL_NOT_ALLOWED_IN_%s", STATE_NAME[s_state]);
            return;
        }
        if (cal_reset() == ESP_OK) {
            out_line("E;CAL;RESET;v_gain_ppm=%d,v_off_uV=%d,i_gain_ppm=%d",
                     CAL_GAIN_NOM_PPM, 0, CAL_GAIN_NOM_PPM);
            print_status();
        } else {
            out_line("E;ERR;CAL_SAVE_FAILED");
        }
        return;
    }
    if (!strcmp(sub, "SET")) {
        if (!idle_like()) {
            out_line("E;ERR;CAL_NOT_ALLOWED_IN_%s", STATE_NAME[s_state]);
            return;
        }
        if (nt < 4) {
            out_line("E;ERR;CAL_SET_NEEDS_3_VALUES");
            return;
        }
        c.v_gain_ppm = (int32_t)strtol(t[1], NULL, 10);
        c.v_off_uv = (int32_t)strtol(t[2], NULL, 10);
        c.i_gain_ppm = (int32_t)strtol(t[3], NULL, 10);
        if (!cal_valid(&c)) {
            out_line("E;ERR;CAL_OUT_OF_RANGE;v_gain_ppm=%d,v_off_uV=%d,i_gain_ppm=%d",
                     (int)c.v_gain_ppm, (int)c.v_off_uv, (int)c.i_gain_ppm);
            return;
        }
        if (cal_set(&c) == ESP_OK) {
            out_line("E;CAL;SET;v_gain_ppm=%d,v_off_uV=%d,i_gain_ppm=%d",
                     (int)c.v_gain_ppm, (int)c.v_off_uv, (int)c.i_gain_ppm);
            print_status();
        } else {
            out_line("E;ERR;CAL_SAVE_FAILED");
        }
        return;
    }
    if (!strcmp(sub, "V") || !strcmp(sub, "VP1") || !strcmp(sub, "VP2")) {
        if (!idle_like()) {
            out_line("E;ERR;CAL_NOT_ALLOWED_IN_%s", STATE_NAME[s_state]);
            return;
        }
        double ref;
        if (!parse_unit(nt > 1 ? t[1] : NULL, &ref)) {
            out_line("E;ERR;CAL_NEEDS_REFERENCE");
            return;
        }
        double raw;
        if (!measure_raw_v(&raw)) {
            out_line("E;ERR;CAL_ADC_FAILED");
            return;
        }
        if (raw < 2.0) {
            out_line("E;ERR;CAL_V_NO_SIGNAL;raw_V=%.3f", raw);
            return;
        }
        if (!strcmp(sub, "VP1")) {
            s_vp1_set = true;
            s_vp1_raw = raw;
            s_vp1_ref = ref;
            out_line("E;CAL;VP1;raw_V=%.4f,ref_V=%.4f", raw, ref);
            return;
        }
        if (!strcmp(sub, "VP2")) {
            if (!s_vp1_set) {
                out_line("E;ERR;CAL_VP2_NEEDS_VP1");
                return;
            }
            if (fabs(raw - s_vp1_raw) < 1.5 || fabs(ref - s_vp1_ref) < 1.5) {
                out_line("E;ERR;CAL_POINTS_TOO_CLOSE;raw_delta_V=%.3f,ref_delta_V=%.3f",
                         fabs(raw - s_vp1_raw), fabs(ref - s_vp1_ref));
                return;
            }
            const double g = (ref - s_vp1_ref) / (raw - s_vp1_raw);
            const double off = s_vp1_ref - g * s_vp1_raw;
            c.v_gain_ppm = (int32_t)lround(g * 1e6);
            c.v_off_uv = (int32_t)lround(off * 1e6);
            if (!cal_valid(&c)) {
                out_line("E;ERR;CAL_OUT_OF_RANGE;v_gain_ppm=%d,v_off_uV=%d",
                         (int)c.v_gain_ppm, (int)c.v_off_uv);
                return;
            }
            if (cal_set(&c) == ESP_OK) {
                s_vp1_set = false;
                out_line("E;CAL;VP2;raw1_V=%.4f,ref1_V=%.4f,raw2_V=%.4f,ref2_V=%.4f,"
                         "v_gain_ppm=%d,v_off_uV=%d",
                         s_vp1_raw, s_vp1_ref, raw, ref, (int)c.v_gain_ppm, (int)c.v_off_uv);
                print_status();
            } else {
                out_line("E;ERR;CAL_SAVE_FAILED");
            }
            return;
        }
        /* V: um ponto, ajusta o ganho mantendo o offset atual */
        const double g = (ref - (double)c.v_off_uv / 1e6) / raw;
        c.v_gain_ppm = (int32_t)lround(g * 1e6);
        if (!cal_valid(&c)) {
            out_line("E;ERR;CAL_OUT_OF_RANGE;v_gain_ppm=%d", (int)c.v_gain_ppm);
            return;
        }
        if (cal_set(&c) == ESP_OK) {
            out_line("E;CAL;V;raw_V=%.4f,ref_V=%.4f,v_gain_ppm=%d,v_off_uV=%d",
                     raw, ref, (int)c.v_gain_ppm, (int)c.v_off_uv);
            print_status();
        } else {
            out_line("E;ERR;CAL_SAVE_FAILED");
        }
        return;
    }
    if (!strcmp(sub, "I")) {
        if (s_state != ST_DISCHARGE) {
            out_line("E;ERR;CAL_I_ONLY_IN_DISCHARGE;state=%s", STATE_NAME[s_state]);
            return;
        }
        double ref;
        if (!parse_unit(nt > 1 ? t[1] : NULL, &ref)) {
            out_line("E;ERR;CAL_NEEDS_REFERENCE");
            return;
        }
        s_cal_i_ref_a = ref;
        s_cal_i_sum = 0.0;
        s_cal_i_n = 0;
        s_cal_i_active = true;
        out_line("E;CAL;I_START;ref_A=%.4f,samples=%d", ref, CAL_I_SAMPLES);
        return;
    }
    out_line("E;ERR;CAL_UNKNOWN_SUBCOMMAND;%s", sub);
}

static void handle_command(char *line)
{
    char *tok[5];
    int nt = 0;
    char *save = NULL;
    for (char *p = strtok_r(line, " \t,;=", &save); p != NULL && nt < 5;
         p = strtok_r(NULL, " \t,;=", &save)) {
        tok[nt++] = p;
    }
    if (nt == 0) {
        return;
    }
    upper(tok[0]);
    const char *cmd = tok[0];

    if (!strcmp(cmd, "TARE")) {
        cmd_tare();
    } else if (!strcmp(cmd, "START")) {
        cmd_start();
    } else if (!strcmp(cmd, "STOP")) {
        cmd_stop();
    } else if (!strcmp(cmd, "RESET")) {
        cmd_reset();
    } else if (!strcmp(cmd, "CUTOFF")) {
        cmd_cutoff(nt > 1 ? tok[1] : NULL);
    } else if (!strcmp(cmd, "CAL")) {
        cmd_cal(&tok[1], nt - 1);
    } else if (!strcmp(cmd, "STATUS")) {
        print_status();
    } else if (!strcmp(cmd, "SYNC")) {
        do_sync();
    } else if (!strcmp(cmd, "LOG")) {
        cmd_log();
    } else if (!strcmp(cmd, "HELP")) {
        out_line("E;HELP;TARE | START | STOP | RESET | CUTOFF <mV|V> | STATUS | SYNC | LOG | "
                 "CAL SHOW|RESET|V|VP1|VP2|I|SET");
    } else {
        out_line("E;ERR;UNKNOWN_COMMAND;%s", cmd);
    }
}

static void feed(line_asm_t *a, char c)
{
    if (c == '\n' || c == '\r') {
        if (a->len > 0) {
            a->buf[a->len] = '\0';
            a->len = 0;
            handle_command(a->buf);
        }
    } else if (a->len < sizeof(a->buf) - 1) {
        a->buf[a->len++] = c;
    } else {
        a->len = 0;   /* linha longa demais: descarta */
    }
}

static void poll_commands(void)
{
    uint8_t buf[64];
    int n = uart_read_bytes(CMD_UART, buf, sizeof(buf), 0);
    for (int k = 0; k < n; k++) {
        feed(&s_rx_uart, (char)buf[k]);
    }
    size_t m;
    while ((m = bt_link_read(buf, sizeof(buf))) > 0) {
        for (size_t k = 0; k < m; k++) {
            feed(&s_rx_bt, (char)buf[k]);
        }
    }
}

static void uart_init(void)
{
    /* Driver so para RX (os printf continuam pelo console padrao). */
    ESP_ERROR_CHECK(uart_driver_install(CMD_UART, 256, 0, 0, NULL, 0));
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* 1) Carga e rele SEMPRE desligados primeiro. */
    out_pin_init(LOAD_GPIO, !LOAD_ACTIVE_LEVEL);
    load_set(false);
#if RELAY_EN
    out_pin_init(RELAY_GPIO, !RELAY_ACTIVE_LEVEL);
    relay_set(false);
#endif

    cal_store_init();
    s_test_id = cal_last_test_id();
    uart_init();
    i2c_init();
    flog_init();
    if (BT_EN) {
        bt_link_init();
    }
    clear_test_data();

    if (s_cutoff_mv < CONFIG_BATT_CUTOFF_MIN_MV || s_cutoff_mv > CONFIG_BATT_CUTOFF_MAX_MV) {
        int clamped = s_cutoff_mv < CONFIG_BATT_CUTOFF_MIN_MV ? CONFIG_BATT_CUTOFF_MIN_MV
                                                              : CONFIG_BATT_CUTOFF_MAX_MV;
        out_line("E;WARN;DEFAULT_CUTOFF_OUT_OF_RANGE;%d->%d", s_cutoff_mv, clamped);
        s_cutoff_mv = clamped;
    }

    esp_err_t werr = esp_task_wdt_add(NULL);
    if (werr != ESP_OK) {
        ESP_LOGW(TAG, "esp_task_wdt_add: %s", esp_err_to_name(werr));
    }

    out_line("E;BOOT;fw=%s,reset=%d,load_gpio=%d,active_level=%d,relay=%d,bt=%d,log=%d,test_id=%u",
             FW_VERSION, (int)esp_reset_reason(), CONFIG_BATT_LOAD_GPIO, LOAD_ACTIVE_LEVEL,
             RELAY_EN, BT_EN, flog_available() ? 1 : 0, (unsigned)s_test_id);
    print_header();
    print_status();

    const TickType_t period = pdMS_TO_TICKS(CONFIG_BATT_SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();
    int overruns = 0;

    while (1) {
        esp_task_wdt_reset();
        poll_commands();
        if (bt_link_take_new_connection()) {
            do_sync();               /* cliente novo: manda cabecalho, status e eventos recentes */
        }

        double ain0, ain1;
        esp_err_t err = measure(&ain0, &ain1);
        const int64_t now_us = esp_timer_get_time();

        if (err == ESP_OK) {
            s_adc_fail = 0;
            process_sample(ain0, ain1, now_us);
            log_tick(now_us);
        } else {
            s_adc_fail++;
            out_line("E;WARN;ADC_ERR;%s,consecutive=%d", esp_err_to_name(err), s_adc_fail);
            if (s_adc_fail >= CONFIG_BATT_ADC_FAIL_LIMIT &&
                (s_state == ST_REST || s_state == ST_DISCHARGE)) {
                enter_fault("ADC_FAIL");
            }
        }
        dump_tick();

        if (s_resync) {              /* tara, CAL e atrasos do rele bloqueiam por projeto */
            last_wake = xTaskGetTickCount();
            s_resync = false;
        }
        if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
            /* Aquisicao mais lenta que o periodo: reduza BATT_OVERSAMPLE ou aumente o periodo. */
            if (++overruns <= 5 || overruns % 100 == 0) {
                out_line("E;WARN;PERIOD_OVERRUN;count=%d", overruns);
            }
        }
    }
}
