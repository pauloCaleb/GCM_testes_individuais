#include "ble_gamepad.h"
#include "dabble_gamepad.h"

#include <assert.h>
#include <string.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_gp";

/* UUIDs (little-endian) do serviço usado pela biblioteca DabbleESP32 */
static const ble_uuid128_t UUID_SERVICE =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E);
static const ble_uuid128_t UUID_RX =   /* app -> robô (write) */
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E);
static const ble_uuid128_t UUID_TX =   /* robô -> app (notify) */
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E);

static uint8_t s_own_addr_type;
static volatile bool s_connected;
static ble_gamepad_conn_cb_t s_cb;
static uint16_t s_tx_handle;

static void start_advertising(void);

static int rx_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[64];
        uint16_t out_len = 0;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &out_len);
        if (rc == 0) {
            dabble_gp_feed(buf, out_len);
        }
    }
    return 0;
}

static int tx_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;   /* característica só de notify; nada a ler/escrever */
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SERVICE.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &UUID_RX.u,
                .access_cb = rx_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &UUID_TX.u,
                .access_cb = tx_access_cb,
                .val_handle = &s_tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        },
    },
    { 0 }
};

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "conexão %s (status=%d)",
                 event->connect.status == 0 ? "estabelecida" : "falhou",
                 event->connect.status);
        if (event->connect.status == 0) {
            s_connected = true;
            if (s_cb) s_cb(true);
        } else {
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "desconectado (reason=%d)", event->disconnect.reason);
        s_connected = false;
        dabble_gp_reset();
        if (s_cb) s_cb(false);
        start_advertising();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;

    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    const char *name = ble_svc_gap_device_name();

    /* Tudo no pacote principal (flags 3 + UUID128 18 + nome <= 31 bytes), como o
     * firmware antigo fazia com o nome: o app Dabble encontra o robô pelo nome. */
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;
    fields.uuids128 = &UUID_SERVICE;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d (nome grande demais?)", rc);
        return;
    }

    struct ble_gap_adv_params params;
    memset(&params, 0, sizeof(params));
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "anunciando como \"%s\"", name);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    assert(rc == 0);
    (void)rc;
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "host reset, reason=%d", reason);
}

static void host_task(void *param)
{
    nimble_port_run();                 /* só retorna em nimble_port_stop() */
    nimble_port_freertos_deinit();
}

void ble_gamepad_start(const char *name, ble_gamepad_conn_cb_t cb)
{
    s_cb = cb;

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set(name);
    assert(rc == 0);
    (void)rc;

    nimble_port_freertos_init(host_task);
}

bool ble_gamepad_is_connected(void)
{
    return s_connected;
}
