/*
 * Driver mínimo (não-oficial) para o sensor ToF ST VL53L1X, escrito para
 * ESP-IDF 5.4.2 usando o driver novo i2c_master (driver/i2c_master.h).
 *
 * Cobre apenas o essencial para medir distância em modo contínuo:
 *   - boot check / identificação do sensor
 *   - carga da configuração padrão de fábrica (registradores 0x2D..0x87)
 *   - start/stop de ranging contínuo
 *   - polling de dado pronto + leitura de distância e status de range
 *
 * Multi-sensor: vl53l1x_set_i2c_address() troca o endereço I2C do sensor
 * (volátil: volta a 0x29 a cada reset via XSHUT ou power-cycle).
 *
 * Não implementa: calibração de offset/crosstalk, ROI customizada,
 * modo de interrupção via GPIO. Esses pontos ficam marcados como TODO.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L1X_DEFAULT_I2C_ADDR   0x29

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t address;               // endereço I2C 7 bits atualmente em uso
} vl53l1x_t;

typedef enum {
    VL53L1X_DISTANCE_SHORT = 0,   // até ~1.3 m, mais imune à luz ambiente
    VL53L1X_DISTANCE_LONG  = 1,   // até ~4 m (default de fábrica)
} vl53l1x_distance_mode_t;

typedef struct {
    uint16_t distance_mm;      // distância filtrada (crosstalk corrigido)
    uint8_t  range_status;     // 0 = "range valid" (ver vl53l1x_range_status_str)
    uint16_t signal_rate_mcps; // taxa de sinal, unidade 1/128 MCPS
    uint16_t ambient_rate_mcps;// taxa de luz ambiente, unidade 1/128 MCPS
} vl53l1x_result_t;

/**
 * Cria o handle I2C do dispositivo no barramento já inicializado
 * (bus_handle vem de i2c_new_master_bus() feito pelo chamador).
 */
esp_err_t vl53l1x_init_bus_device(i2c_master_bus_handle_t bus_handle,
                                   uint8_t i2c_addr,
                                   vl53l1x_t *dev);

/**
 * Espera o boot do sensor (bit firmware ready em 0xE5). Deve ser chamada
 * depois de soltar o XSHUT e antes de qualquer outra escrita.
 */
esp_err_t vl53l1x_wait_for_boot(vl53l1x_t *dev, uint32_t timeout_ms);

/**
 * Troca o endereço I2C do sensor (registrador 0x0001, valor de 7 bits) e
 * recria o handle do dispositivo no novo endereço. O sensor deve estar
 * bootado e ser o único respondendo no endereço atual. A troca é volátil.
 * Em caso de sucesso dev->i2c_dev/dev->address já refletem o novo endereço.
 */
esp_err_t vl53l1x_set_i2c_address(i2c_master_bus_handle_t bus_handle,
                                   vl53l1x_t *dev,
                                   uint8_t new_addr);

/** Remove o handle do dispositivo do barramento. */
esp_err_t vl53l1x_deinit(vl53l1x_t *dev);

/**
 * Espera o boot do sensor (bit firmware ready) e carrega a configuração
 * padrão de fábrica. Deve ser chamado uma vez após o power-up (ou após
 * pulsar XSHUT), antes de start_ranging.
 */
esp_err_t vl53l1x_sensor_init(vl53l1x_t *dev);

/** Lê o Model ID (deve retornar 0xEACC para VL53L1X). Útil para checar fiação. */
esp_err_t vl53l1x_get_sensor_id(vl53l1x_t *dev, uint16_t *model_id);

/** Define o timing budget da medição em ms (20-500 ms). Chamar antes de start_ranging. */
esp_err_t vl53l1x_set_timing_budget_ms(vl53l1x_t *dev, uint16_t budget_ms);

/** Define o período entre medições no modo contínuo (deve ser >= timing budget). */
esp_err_t vl53l1x_set_inter_measurement_ms(vl53l1x_t *dev, uint16_t period_ms);

/** Modo curto (mais imune a luz ambiente) ou longo (alcance máximo ~4 m, default). */
esp_err_t vl53l1x_set_distance_mode(vl53l1x_t *dev, vl53l1x_distance_mode_t mode);

/** Inicia medições contínuas. */
esp_err_t vl53l1x_start_ranging(vl53l1x_t *dev);

/** Para as medições contínuas. */
esp_err_t vl53l1x_stop_ranging(vl53l1x_t *dev);

/** true se há um novo dado de distância pronto para leitura. */
esp_err_t vl53l1x_check_data_ready(vl53l1x_t *dev, bool *ready);

/** Lê o resultado da medição atual. Chamar clear_interrupt depois. */
esp_err_t vl53l1x_get_result(vl53l1x_t *dev, vl53l1x_result_t *result);

/** Rearma o sensor para a próxima medição (chamar após ler o resultado). */
esp_err_t vl53l1x_clear_interrupt(vl53l1x_t *dev);

/** Bloqueia (com timeout) até o dado ficar pronto e já retorna o resultado. */
esp_err_t vl53l1x_wait_for_result(vl53l1x_t *dev, vl53l1x_result_t *result,
                                   uint32_t timeout_ms);

/** Texto legível para o campo range_status (RangeStatus da ST, já traduzido). */
const char *vl53l1x_range_status_str(uint8_t status);

#ifdef __cplusplus
}
#endif
