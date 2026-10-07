#include "bt_link.h"
#include "sdkconfig.h"

#if CONFIG_BATT_BT_ENABLE

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define BT_MSG_MAX   160
#define BT_QUEUE_LEN 96
#define BT_RESERVE   16      /* vagas reservadas a linhas que nao podem ser descartadas */

typedef struct {
    uint8_t len;
    char data[BT_MSG_MAX];
} bt_msg_t;

static const char *TAG = "BT_LINK";
static QueueHandle_t s_txq;
static StreamBufferHandle_t s_rx_sb;
static SemaphoreHandle_t s_wr_sem;
static volatile uint32_t s_handle;
static volatile bool s_conn;
static volatile bool s_new_conn;
static volatile bool s_cong;
static volatile int s_wr_len;
static volatile uint32_t s_dropped;

static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, "GCM_BATT_SPP");
        } else {
            ESP_LOGE(TAG, "SPP init falhou: %d", param->init.status);
        }
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            esp_bt_gap_set_device_name(CONFIG_BATT_BT_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(TAG, "SPP start falhou: %d", param->start.status);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        if (s_conn) {                                   /* um cliente por vez */
            esp_spp_disconnect(param->srv_open.handle);
        } else {
            s_handle = param->srv_open.handle;
            s_cong = false;
            s_conn = true;
            s_new_conn = true;
            xQueueReset(s_txq);
        }
        break;
    case ESP_SPP_CLOSE_EVT:
        if (param->close.handle == s_handle) {
            s_conn = false;
            s_handle = 0;
            xSemaphoreGive(s_wr_sem);                   /* destrava a task de TX */
        }
        break;
    case ESP_SPP_DATA_IND_EVT:
        if (param->data_ind.handle == s_handle) {
            xStreamBufferSend(s_rx_sb, param->data_ind.data, param->data_ind.len, 0);
        }
        break;
    case ESP_SPP_CONG_EVT:
        s_cong = param->cong.cong;
        break;
    case ESP_SPP_WRITE_EVT:
        s_cong = param->write.cong;
        s_wr_len = (param->write.status == ESP_SPP_SUCCESS) ? param->write.len : 0;
        xSemaphoreGive(s_wr_sem);
        break;
    default:
        break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    if (event == ESP_BT_GAP_AUTH_CMPL_EVT) {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "pareado: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGW(TAG, "falha de autenticacao: %d", param->auth_cmpl.stat);
        }
    }
}

static void tx_task(void *arg)
{
    bt_msg_t m;
    for (;;) {
        if (xQueueReceive(s_txq, &m, portMAX_DELAY) != pdTRUE || !s_conn) {
            continue;
        }
        int off = 0, fails = 0;
        while (off < m.len && s_conn) {
            int waited = 0;
            while (s_cong && s_conn && waited < 5000) {      /* congestionado: espera liberar */
                vTaskDelay(pdMS_TO_TICKS(10));
                waited += 10;
            }
            xSemaphoreTake(s_wr_sem, 0);
            s_wr_len = 0;
            if (esp_spp_write(s_handle, m.len - off, (uint8_t *)m.data + off) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (++fails > 50) {
                    break;
                }
                continue;
            }
            if (xSemaphoreTake(s_wr_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
                break;
            }
            if (s_wr_len <= 0) {
                if (++fails > 50) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            off += s_wr_len;
        }
    }
}

void bt_link_init(void)
{
    s_txq = xQueueCreate(BT_QUEUE_LEN, sizeof(bt_msg_t));
    s_rx_sb = xStreamBufferCreate(512, 1);
    s_wr_sem = xSemaphoreCreateBinary();
    xTaskCreate(tx_task, "bt_tx", 3072, NULL, 4, NULL);

    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mem_release(BLE): %s", esp_err_to_name(ret));
    }
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK ||
        (ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "controlador BT: %s", esp_err_to_name(ret));
        return;
    }
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = false;                           /* pareamento legado com PIN fixo */
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK ||
        (ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid: %s", esp_err_to_name(ret));
        return;
    }
    esp_bt_gap_register_callback(gap_cb);
    esp_spp_register_callback(spp_cb);
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    if ((ret = esp_spp_enhanced_init(&spp_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "spp init: %s", esp_err_to_name(ret));
        return;
    }
    esp_bt_pin_code_t pin = { 0 };
    size_t n = strlen(CONFIG_BATT_BT_PIN);
    if (n > 16) {
        n = 16;
    }
    memcpy(pin, CONFIG_BATT_BT_PIN, n);
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, (uint8_t)n, pin);
}

bool bt_link_connected(void)
{
    return s_conn;
}

bool bt_link_send(const char *s, size_t len, bool droppable)
{
    if (!s_conn || s_txq == NULL) {
        return false;
    }
    const size_t chunks = (len + BT_MSG_MAX - 1) / BT_MSG_MAX;
    const UBaseType_t freeq = uxQueueSpacesAvailable(s_txq);
    if ((droppable && freeq < chunks + BT_RESERVE) || (!droppable && freeq < chunks)) {
        s_dropped += (uint32_t)chunks;
        return false;
    }
    bt_msg_t m;
    for (size_t off = 0; off < len; off += BT_MSG_MAX) {
        size_t n = len - off > BT_MSG_MAX ? BT_MSG_MAX : len - off;
        m.len = (uint8_t)n;
        memcpy(m.data, s + off, n);
        xQueueSend(s_txq, &m, 0);
    }
    return true;
}

int bt_link_free_slots(void)
{
    return (s_conn && s_txq) ? (int)uxQueueSpacesAvailable(s_txq) : 99;
}

size_t bt_link_read(uint8_t *buf, size_t max)
{
    if (s_rx_sb == NULL) {
        return 0;
    }
    return xStreamBufferReceive(s_rx_sb, buf, max, 0);
}

bool bt_link_take_new_connection(void)
{
    if (s_new_conn) {
        s_new_conn = false;
        return true;
    }
    return false;
}

uint32_t bt_link_dropped(void)
{
    return s_dropped;
}

#else  /* Bluetooth desabilitado no menuconfig */

void bt_link_init(void) {}
bool bt_link_connected(void) { return false; }
bool bt_link_send(const char *s, size_t len, bool droppable) { (void)s; (void)len; (void)droppable; return false; }
int bt_link_free_slots(void) { return 99; }
size_t bt_link_read(uint8_t *buf, size_t max) { (void)buf; (void)max; return 0; }
bool bt_link_take_new_connection(void) { return false; }
uint32_t bt_link_dropped(void) { return 0; }

#endif
