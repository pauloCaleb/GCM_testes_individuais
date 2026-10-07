#include "flash_log.h"
#include <string.h>
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "esp_log.h"

#define REC_SZ    32u
#define SECTOR_SZ 4096u

_Static_assert(sizeof(flog_rec_t) == REC_SZ, "flog_rec_t deve ter 32 bytes");

static const char *TAG = "FLOG";
static const esp_partition_t *s_part;
static uint16_t s_tid;
static uint32_t s_next_idx;

static uint32_t rec_crc(const flog_rec_t *r)
{
    return esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)r, REC_SZ - 4);
}

esp_err_t flog_init(void)
{
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "blog");
    if (s_part == NULL) {
        ESP_LOGW(TAG, "particao 'blog' nao encontrada; log em flash desabilitado");
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

bool flog_available(void)
{
    return s_part != NULL;
}

uint32_t flog_capacity(void)
{
    return s_part ? s_part->size / REC_SZ : 0;
}

void flog_begin(uint16_t test_id)
{
    s_tid = test_id;
    s_next_idx = 0;
}

uint32_t flog_count(void)
{
    return s_next_idx;
}

esp_err_t flog_append(uint8_t state, float t_test_s, float v, float i, float q, float e)
{
    if (s_part == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint32_t off = s_next_idx * REC_SZ;
    if (off + REC_SZ > s_part->size) {
        return ESP_ERR_NO_MEM;
    }
    if (off % SECTOR_SZ == 0) {                       /* apaga o setor so quando for usa-lo */
        esp_err_t er = esp_partition_erase_range(s_part, off, SECTOR_SZ);
        if (er != ESP_OK) {
            return er;
        }
    }
    flog_rec_t r;
    memset(&r, 0, sizeof(r));
    r.idx = s_next_idx;
    r.test_id = s_tid;
    r.state = state;
    r.t_test_s = t_test_s;
    r.v_batt_v = v;
    r.i_a = i;
    r.q_ah = q;
    r.e_wh = e;
    r.crc = rec_crc(&r);
    esp_err_t er = esp_partition_write(s_part, off, &r, REC_SZ);
    if (er == ESP_OK) {
        s_next_idx++;
    }
    return er;
}

bool flog_read(uint16_t test_id, uint32_t idx, flog_rec_t *out)
{
    if (s_part == NULL) {
        return false;
    }
    const uint32_t off = idx * REC_SZ;
    if (off + REC_SZ > s_part->size) {
        return false;
    }
    if (esp_partition_read(s_part, off, out, REC_SZ) != ESP_OK) {
        return false;
    }
    return out->crc == rec_crc(out) && out->idx == idx && out->test_id == test_id;
}
