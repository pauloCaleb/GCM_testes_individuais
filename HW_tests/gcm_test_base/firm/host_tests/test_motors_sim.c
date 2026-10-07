/* Simulacao no PC da logica de seguranca dos motores (motors.c com stubs de hardware).
 * Verifica: EN inativo no boot, limite de duty, rampa, DIR so troca com PWM=0,
 * STOP imediato e watchdog. Uso: make -C host_tests run */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
/* --- stubs de hardware que gravam o estado dos pinos --- */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
static int g_gpio[64]; static uint32_t g_duty[2]; static int64_t g_us;
static int g_dir_flips_with_duty;  /* DIR mudando com PWM != 0 (proibido) */
esp_err_t gpio_config(const gpio_config_t*c){(void)c;return 0;}
esp_err_t gpio_set_level(gpio_num_t p,uint32_t l){
    if ((p==17||p==25) && g_gpio[p]!=(int)l) { int ch=(p==17)?0:1; if (g_duty[ch]!=0) g_dir_flips_with_duty++; }
    g_gpio[p]=l; return 0;}
int gpio_get_level(gpio_num_t p){return g_gpio[p];}
esp_err_t ledc_timer_config(const ledc_timer_config_t*c){(void)c;return 0;}
esp_err_t ledc_channel_config(const ledc_channel_config_t*c){(void)c;return 0;}
esp_err_t ledc_set_duty(ledc_mode_t m,ledc_channel_t c,uint32_t d){(void)m;g_duty[c]=d;return 0;}
esp_err_t ledc_update_duty(ledc_mode_t m,ledc_channel_t c){(void)m;(void)c;return 0;}
SemaphoreHandle_t xSemaphoreCreateMutex(void){return (void*)1;}
BaseType_t xSemaphoreTake(SemaphoreHandle_t s,TickType_t t){(void)s;(void)t;return 1;}
BaseType_t xSemaphoreGive(SemaphoreHandle_t s){(void)s;return 1;}
void vTaskDelay(TickType_t t){(void)t;} TickType_t xTaskGetTickCount(void){return 0;}
void vTaskDelayUntil(TickType_t*p,TickType_t t){(void)p;(void)t;}
BaseType_t xTaskCreate(void(*f)(void*),const char*n,uint32_t s,void*a,int p,TaskHandle_t*h){(void)f;(void)n;(void)s;(void)a;(void)p;(void)h;return 1;}
const char *esp_err_to_name(esp_err_t e){(void)e;return "x";}
#include "esp_timer.h"
int64_t esp_timer_get_time(void){return g_us;}
#undef ESP_LOGI
#define ESP_LOGI(t,f,...) ((void)0)
#define ESP_LOGW(t,f,...) ((void)0)
#include "../main/motors.c"

static int fails;
#define CHECK(c) do{ if(!(c)){printf("FALHOU: %s (linha %d)\n",#c,__LINE__);fails++;} }while(0)
static void tick(int n){ for(int i=0;i<n;i++){ g_us += MOTOR_TICK_MS*1000; step_motor(&s_m[0]); step_motor(&s_m[1]);
   bool active = s_en || s_m[0].cur!=0||s_m[1].cur!=0||s_m[0].tgt!=0||s_m[1].tgt!=0;
   if (s_wd_ms>0 && active && !s_failsafe && (now_ms()-s_last_feed_ms) > s_wd_ms) { stop_locked(); s_failsafe=true; } } }

int main(void){
    CHECK(motors_init()==0);
    CHECK(g_gpio[23]==0);                       /* EN inativo no boot */
    CHECK(g_duty[0]==0 && g_duty[1]==0);

    /* Sem EN: o alvo é ignorado */
    motors_set_target(0, 20); tick(10);
    CHECK(s_m[0].cur==0 && g_duty[0]==0);

    s_wd_ms = 0;                                /* watchdog fora nos testes de movimento */
    motors_set_enable(true);
    CHECK(g_gpio[23]==1);
    CHECK(s_m[0].tgt==0);                       /* habilitar zera alvos */

    /* Limite de duty: 90 -> 30 */
    float ap = motors_set_target(0, 90);
    CHECK(fabsf(ap-30.0f)<1e-3);

    /* Rampa: 300 %/s e tick de 10 ms = 3 %/tick; settle de 2 ticks pós-DIR */
    tick(1); CHECK(s_m[0].cur==0 && g_gpio[17]==1);   /* DIR definido, ainda parado */
    tick(2); CHECK(s_m[0].cur==0);                     /* settle */
    tick(1); CHECK(fabsf(s_m[0].cur-3.0f)<1e-3);
    tick(20); CHECK(fabsf(s_m[0].cur-30.0f)<1e-3);
    CHECK(g_duty[0]==(uint32_t)(30.0f/100*1023+0.5f));

    /* Inversão: -30 -> passa por 0, só então troca DIR, nunca com PWM != 0 */
    motors_set_target(0, -30);
    int saw_zero=0, saw_neg=0, dir_changed_at_zero=0;
    for(int i=0;i<60;i++){ tick(1);
        if (s_m[0].cur==0) saw_zero=1;
        if (s_m[0].cur<0) { saw_neg=1; CHECK(g_gpio[17]==0); }   /* ré => DIR no nível oposto */
        if (g_gpio[17]==0 && s_m[0].cur==0) dir_changed_at_zero=1; }
    CHECK(saw_zero && saw_neg && dir_changed_at_zero);
    CHECK(fabsf(s_m[0].cur+30.0f)<1e-3);
    CHECK(g_dir_flips_with_duty==0);

    /* Sem rampa (slew=0): inverter ainda passa por zero */
    s_slew = 0; motors_set_target(0, 30);
    tick(1); CHECK(s_m[0].cur==0);
    for(int i=0;i<10;i++) tick(1);
    CHECK(fabsf(s_m[0].cur-30.0f)<1e-3 && g_dir_flips_with_duty==0);
    s_slew = 300;

    /* Reduzir o limite reduz o duty aplicado */
    motors_set_max_duty(10); tick(30); CHECK(fabsf(s_m[0].cur-10.0f)<1e-3);
    motors_set_max_duty(30);

    /* STOP imediato */
    motors_stop(); CHECK(s_m[0].cur==0 && g_duty[0]==0 && g_gpio[23]==0 && !s_en);

    /* Watchdog: ativo -> dispara após wd_ms sem feed */
    s_wd_ms = 500;
    motors_set_enable(true); motors_set_target(1, 20); tick(30);
    CHECK(s_m[1].cur>0);
    motors_feed_watchdog();
    tick(40);                                    /* 400 ms < 500: ainda ativo */
    CHECK(!s_failsafe && s_m[1].cur>0);
    tick(20);                                    /* passa de 500 ms */
    CHECK(s_failsafe && s_m[1].cur==0 && g_duty[1]==0 && g_gpio[23]==0);
    /* Re-habilitar limpa o failsafe */
    motors_set_enable(true); CHECK(!s_failsafe);
    /* Inativo não dispara watchdog */
    motors_stop(); s_failsafe=false; tick(200); CHECK(!s_failsafe);

    if(fails){printf("%d falha(s)\n",fails);return 1;}
    printf("OK: simulacao dos motores passou\n"); return 0; }
