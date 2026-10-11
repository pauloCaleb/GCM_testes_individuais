/**
 * main.c — GCM-PI2-2026.2 / Teste de descarga controlada da bateria do robo
 * ------------------------------------------------------------------------
 * Descarrega a bateria atraves de uma carga eletronica fixa e registra
 * tensao, corrente, potencia, Ah e Wh ate a tensao de cutoff (padrao 12 V).
 *
 * Topologia de medicao (GCM alimentada por fonte de bancada, NAO pela bateria):
 *
 *     bateria (+) --> PWR_PAD_BATT --[ACS758]--> PWR_PAD_SENS --> carga --> bateria (-)
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
 *
 * Maquina de estados (um unico task, sem race entre comandos e aquisicao):
 *
 *   IDLE --TARE--> READY --START--> REST --> DISCHARGE --cutoff/STOP--> RECOVERY --> DONE
 *                    ^                  \__________ qualquer erro ___________> FAULT
 *                    |__________ TARE / RESET (a partir de DONE/FAULT) ___________|
 *
 * Protocolo serial (115200 8N1, linhas terminadas em \n):
 *
 *   PC -> ESP32 (comandos, case-insensitive):
 *     TARE            mede o zero do ACS758 com a carga desligada
 *     START           repouso -> liga a carga -> descarga ate o cutoff
 *     STOP            interrompe (DISCHARGE -> RECOVERY; RECOVERY -> DONE)
 *     RESET           volta a READY/IDLE a partir de DONE ou FAULT
 *     CUTOFF <valor>  ajusta o cutoff (mV, ou V se < 100). Bloqueado em REST/DISCHARGE
 *     STATUS          imprime o estado e os parametros
 *     HELP            lista os comandos
 *
 *   ESP32 -> PC (uma linha por registro; campos separados por ';'):
 *     H;<colunas>                       cabecalho do CSV
 *     D;t_boot_s;t_test_s;state;v_batt_V;i_A;p_W;q_Ah;e_Wh;ain0_V;ain1_V
 *     E;<EVENTO>;<detalhes>             eventos (TARE_OK, LOAD_ON, LOAD_OFF, FAULT, SUMMARY...)
 *     S;<chave=valor,...>               status
 *   Qualquer outra linha (ESP_LOG, boot) e informativa e ignorada pelo parser.
 *
 * Seguranca:
 *   - O cutoff e decidido AQUI, no firmware: o teste termina corretamente mesmo
 *     que o PC desconecte ou o Python seja fechado.
 *   - Carga desligada em: cutoff, STOP, FAULT, reset do chip (pull externo).
 *   - FAULT em: sobrecorrente, ausencia de corrente com a carga ligada,
 *     falhas repetidas do ADS1115, corrente com a carga desligada (REST),
 *     bateria ausente ou abaixo do limite de partida.
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
#include "esp_timer.h"
#include "esp_task_wdt.h"

#if CONFIG_FREERTOS_HZ < 1000
#error "Este projeto exige CONFIG_FREERTOS_HZ=1000 (ver sdkconfig.defaults). Rode 'idf.py fullclean' e reconfigure."
#endif

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

/* ------------------------------------------------------------------ */
/* Estado do teste                                                     */
/* ------------------------------------------------------------------ */

typedef enum {
    ST_IDLE = 0,   /* sem tara */
    ST_READY,      /* tara feita, aguardando START */
    ST_REST,       /* carga desligada, medindo a tensao em repouso */
    ST_DISCHARGE,  /* carga ligada */
    ST_RECOVERY,   /* carga desligada apos cutoff/STOP, registrando a recuperacao */
    ST_DONE,       /* teste encerrado */
    ST_FAULT,      /* erro: carga desligada */
} test_state_t;

static const char *const STATE_NAME[] = {
    "IDLE", "READY", "REST", "DISCHARGE", "RECOVERY", "DONE", "FAULT"
};

static test_state_t s_state = ST_IDLE;

static int    s_cutoff_mv = CONFIG_BATT_CUTOFF_MV;
static bool   s_tared = false;
static double s_zero_v = ACS_NOMINAL_ZERO_V;
static bool   s_load_on = false;
static int    s_adc_fail = 0;
static bool   s_resync = false;   /* a tara bloqueia ~1 s: reancora o periodo do loop */

/* Tempos (esp_timer, us) */
static int64_t s_last_us;       /* ultima amostra valida */
static int64_t s_rest_t0_us;
static int64_t s_load_on_us;
static int64_t s_rec_t0_us;

/* Medidas e acumuladores */
static double s_last_v, s_last_i;
static double s_t_test_s;       /* tempo desde que a carga foi ligada (congela no fim) */
static double s_q_ah, s_e_wh;
static double s_prev_i, s_prev_p;
static double s_v_rest, s_v_min, s_v_cut, s_v_rec;
static double s_i_zero_end_a;
static double s_dur_s;
static double s_rint_mohm;
static const char *s_end_reason = "-";

static double s_rest_sv; static int s_rest_n; static int s_rest_bad;
static double s_rint_sv, s_rint_si; static int s_rint_n; static bool s_rint_done;
static double s_rec_sv, s_rec_si; static int s_rec_n;
static int s_below_cnt, s_oc_cnt, s_nc_cnt;

/* Linha de comando */
static char s_rx[64];
static size_t s_rx_len;

/* I2C */
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_ads;

/* ------------------------------------------------------------------ */
/* Saida serial                                                        */
/* ------------------------------------------------------------------ */

static void out_line(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
    fflush(stdout);
}

static void print_header(void)
{
    out_line("H;t_boot_s;t_test_s;state;v_batt_V;i_A;p_W;q_Ah;e_Wh;ain0_V;ain1_V");
}

static void print_status(void)
{
    out_line("S;state=%s,cutoff_mV=%d,cutoff_min_mV=%d,cutoff_max_mV=%d,tared=%d,"
             "zero_V=%.5f,load=%d,period_ms=%d,oversample=%d,max_current_mA=%d",
             STATE_NAME[s_state], s_cutoff_mv,
             CONFIG_BATT_CUTOFF_MIN_MV, CONFIG_BATT_CUTOFF_MAX_MV,
             s_tared ? 1 : 0, s_zero_v, s_load_on ? 1 : 0,
             CONFIG_BATT_SAMPLE_PERIOD_MS, CONFIG_BATT_OVERSAMPLE,
             CONFIG_BATT_MAX_CURRENT_MA);
}

/* ------------------------------------------------------------------ */
/* Carga eletronica (GPIO23)                                           */
/* ------------------------------------------------------------------ */

static void load_set(bool on)
{
    gpio_set_level(LOAD_GPIO, on ? LOAD_ACTIVE_LEVEL : !LOAD_ACTIVE_LEVEL);
    s_load_on = on;
}

static void load_init(void)
{
    /* Grava o nivel inativo no latch ANTES de configurar o pino como saida,
     * para nao gerar pulso ativo na inicializacao. */
    gpio_set_level(LOAD_GPIO, !LOAD_ACTIVE_LEVEL);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CONFIG_BATT_LOAD_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    load_set(false);
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
    s_q_ah = s_e_wh = 0.0;
    s_prev_i = s_prev_p = 0.0;
    s_v_rest = s_v_min = s_v_cut = s_v_rec = NAN;
    s_i_zero_end_a = NAN;
    s_rint_mohm = NAN;
    s_dur_s = 0.0;
    s_end_reason = "-";
    s_rest_sv = 0.0; s_rest_n = 0; s_rest_bad = 0;
    s_rint_sv = s_rint_si = 0.0; s_rint_n = 0; s_rint_done = false;
    s_rec_sv = s_rec_si = 0.0; s_rec_n = 0;
    s_below_cnt = s_oc_cnt = s_nc_cnt = 0;
}

static void print_summary(void)
{
    double i_avg = (s_dur_s > 0.0) ? (s_q_ah * 3600.0 / s_dur_s) : 0.0;
    out_line("E;SUMMARY;end=%s,dur_s=%.1f,q_Ah=%.6f,e_Wh=%.5f,i_avg_A=%.4f,"
             "v_rest_V=%.4f,v_min_V=%.4f,v_cut_V=%.4f,v_rec_V=%.4f,"
             "rint_mohm=%.1f,i_zero_end_mA=%.1f,cutoff_mV=%d",
             s_end_reason, s_dur_s, s_q_ah, s_e_wh, i_avg,
             s_v_rest, s_v_min, s_v_cut, s_v_rec,
             s_rint_mohm, s_i_zero_end_a * 1000.0, s_cutoff_mv);
}

static void enter_fault(const char *reason)
{
    bool was_discharging = (s_state == ST_DISCHARGE);
    bool test_ran = was_discharging || (s_state == ST_RECOVERY);

    load_set(false);
    if (was_discharging) {
        s_dur_s = (double)(esp_timer_get_time() - s_load_on_us) / 1e6;
        s_v_cut = s_last_v;
    }
    s_end_reason = reason;
    s_state = ST_FAULT;
    out_line("E;FAULT;%s", reason);
    if (test_ran) {
        print_summary();
    }
}

static void begin_recovery(const char *reason, int64_t now_us)
{
    load_set(false);
    s_dur_s = (double)(now_us - s_load_on_us) / 1e6;
    s_v_cut = s_last_v;
    s_end_reason = reason;
    s_rec_t0_us = now_us;
    s_rec_sv = s_rec_si = 0.0;
    s_rec_n = 0;
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
    s_state = ST_DONE;
    out_line("E;DONE;v_rec_V=%.4f", s_v_rec);
    print_summary();
}

static void start_discharge(int64_t now_us)
{
    s_load_on_us = now_us;
    s_last_us = now_us;
    s_prev_i = s_prev_p = 0.0;
    s_q_ah = s_e_wh = 0.0;
    s_v_min = s_v_rest;
    s_below_cnt = s_oc_cnt = s_nc_cnt = 0;
    s_rint_sv = s_rint_si = 0.0; s_rint_n = 0; s_rint_done = false;
    s_t_test_s = 0.0;
    load_set(true);
    s_state = ST_DISCHARGE;
    out_line("E;LOAD_ON;v_rest_V=%.4f,cutoff_mV=%d", s_v_rest, s_cutoff_mv);
}

/* ------------------------------------------------------------------ */
/* Processamento de uma amostra                                        */
/* ------------------------------------------------------------------ */

static void process_sample(double ain0, double ain1, int64_t now_us)
{
    const double v = ain0 * V_RATIO;                       /* V */
    const double i = (ain1 - s_zero_v) / I_SENS_V_PER_A;   /* A (+ = descarga) */
    const double p = v * i;                                /* W */
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
        s_rest_n++;
        if ((now_us - s_rest_t0_us) / 1000 >= CONFIG_BATT_REST_MS) {
            s_v_rest = s_rest_sv / s_rest_n;
            const double vr_mv = s_v_rest * 1000.0;
            if (vr_mv < CONFIG_BATT_PRESENT_MIN_MV) {
                enter_fault("NO_BATTERY");
            } else if (vr_mv <= (double)s_cutoff_mv + CONFIG_BATT_START_MARGIN_MV) {
                enter_fault("BATTERY_BELOW_START_LIMIT");
            } else {
                start_discharge(now_us);
            }
        }
        break;
    }

    case ST_DISCHARGE: {
        const double dt = (double)(now_us - s_last_us) / 1e6;
        s_q_ah += 0.5 * (i + s_prev_i) * dt / 3600.0;
        s_e_wh += 0.5 * (p + s_prev_p) * dt / 3600.0;
        s_prev_i = i;
        s_prev_p = p;
        s_t_test_s = (double)(now_us - s_load_on_us) / 1e6;
        if (v < s_v_min) {
            s_v_min = v;
        }
        const int64_t t_on_ms = (now_us - s_load_on_us) / 1000;

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

        /* Resistencia interna: janela de 1 s apos o assentamento */
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

        /* Cutoff */
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

    out_line("D;%.3f;%.3f;%s;%.4f;%.4f;%.3f;%.6f;%.5f;%.5f;%.5f",
             (double)now_us / 1e6, s_t_test_s, STATE_NAME[s_state],
             v, i, p, s_q_ah, s_e_wh, ain0, ain1);
}

/* ------------------------------------------------------------------ */
/* Comandos                                                            */
/* ------------------------------------------------------------------ */

static void cmd_tare(void)
{
    if (s_state == ST_REST || s_state == ST_DISCHARGE || s_state == ST_RECOVERY) {
        out_line("E;ERR;TARE_NOT_ALLOWED_IN_%s", STATE_NAME[s_state]);
        return;
    }
    load_set(false);
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
    s_tared = true;
    s_resync = true;
    clear_test_data();
    s_end_reason = "-";
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
    s_rest_t0_us = esp_timer_get_time();
    s_adc_fail = 0;
    load_set(false);
    s_state = ST_REST;
    out_line("E;REST_START;rest_ms=%d,cutoff_mV=%d", CONFIG_BATT_REST_MS, s_cutoff_mv);
}

static void cmd_stop(void)
{
    switch (s_state) {
    case ST_REST:
        load_set(false);
        s_state = ST_READY;
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
    load_set(false);
    clear_test_data();
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

static void handle_command(char *line)
{
    char *save = NULL;
    char *cmd = strtok_r(line, " \t,;=", &save);
    if (cmd == NULL) {
        return;
    }
    for (char *p = cmd; *p; p++) {
        *p = (char)toupper((unsigned char)*p);
    }
    char *arg = strtok_r(NULL, " \t,;=", &save);

    if (!strcmp(cmd, "TARE")) {
        cmd_tare();
    } else if (!strcmp(cmd, "START")) {
        cmd_start();
    } else if (!strcmp(cmd, "STOP")) {
        cmd_stop();
    } else if (!strcmp(cmd, "RESET")) {
        cmd_reset();
    } else if (!strcmp(cmd, "CUTOFF")) {
        cmd_cutoff(arg);
    } else if (!strcmp(cmd, "STATUS")) {
        print_status();
    } else if (!strcmp(cmd, "HELP")) {
        out_line("E;HELP;TARE | START | STOP | RESET | CUTOFF <mV|V> | STATUS");
    } else {
        out_line("E;ERR;UNKNOWN_COMMAND;%s", cmd);
    }
}

static void poll_commands(void)
{
    uint8_t buf[64];
    int n = uart_read_bytes(CMD_UART, buf, sizeof(buf), 0);
    for (int k = 0; k < n; k++) {
        char c = (char)buf[k];
        if (c == '\n' || c == '\r') {
            if (s_rx_len > 0) {
                s_rx[s_rx_len] = '\0';
                s_rx_len = 0;
                handle_command(s_rx);
            }
        } else if (s_rx_len < sizeof(s_rx) - 1) {
            s_rx[s_rx_len++] = c;
        } else {
            s_rx_len = 0;   /* linha longa demais: descarta */
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
    /* 1) Carga SEMPRE desligada primeiro. */
    load_init();

    uart_init();
    i2c_init();
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

    out_line("E;BOOT;fw=battery_discharge_test,load_gpio=%d,active_level=%d",
             CONFIG_BATT_LOAD_GPIO, LOAD_ACTIVE_LEVEL);
    print_header();
    print_status();

    const TickType_t period = pdMS_TO_TICKS(CONFIG_BATT_SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();
    int overruns = 0;

    while (1) {
        esp_task_wdt_reset();
        poll_commands();
        if (s_resync) {
            last_wake = xTaskGetTickCount();
            s_resync = false;
        }

        double ain0, ain1;
        esp_err_t err = measure(&ain0, &ain1);
        const int64_t now_us = esp_timer_get_time();

        if (err == ESP_OK) {
            s_adc_fail = 0;
            process_sample(ain0, ain1, now_us);
        } else {
            s_adc_fail++;
            out_line("E;WARN;ADC_ERR;%s,consecutive=%d", esp_err_to_name(err), s_adc_fail);
            if (s_adc_fail >= CONFIG_BATT_ADC_FAIL_LIMIT &&
                (s_state == ST_REST || s_state == ST_DISCHARGE)) {
                enter_fault("ADC_FAIL");
            }
        }

        if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
            /* Aquisicao mais lenta que o periodo: reduza BATT_OVERSAMPLE ou aumente o periodo. */
            if (++overruns <= 5 || overruns % 100 == 0) {
                out_line("E;WARN;PERIOD_OVERRUN;count=%d", overruns);
            }
        }
    }
}
