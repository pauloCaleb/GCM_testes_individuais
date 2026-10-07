#include "pca9554.h"
#include "esp_log.h"

static const char *TAG = "pca9554";

/* ---------------------------- Primitivas de barramento ---------------------------- */

static esp_err_t pca9554_write_reg(pca9554_t *dev, pca9554_reg_t reg, uint8_t value)
{
    uint8_t buf[2] = { (uint8_t)reg, value };
    return i2c_master_transmit(dev->i2c_dev, buf, sizeof(buf), PCA9554_I2C_XFER_TIMEOUT_MS);
}

/*
 * O PCA9554 não possui auto-increment: o command byte seleciona o registrador
 * e todas as leituras subsequentes acessam esse mesmo registrador até que um
 * novo command byte seja enviado. Por isso toda leitura aqui é feita como uma
 * transação "write command byte" + "read 1 byte" (Fig. 13 do datasheet).
 */
static esp_err_t pca9554_read_reg(pca9554_t *dev, pca9554_reg_t reg, uint8_t *value)
{
    uint8_t cmd = (uint8_t)reg;
    return i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, value, 1, PCA9554_I2C_XFER_TIMEOUT_MS);
}

/* ------------------------------------ Init / deinit ------------------------------------ */

esp_err_t pca9554_init(i2c_master_bus_handle_t bus_handle, uint8_t address, pca9554_t *out_dev)
{
    if (bus_handle == NULL || out_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = address,
        .scl_speed_hz    = 400000, /* PCA9554/PCA9554A suportam até 400kHz (Fast-mode) */
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao anexar dispositivo 0x%02X ao barramento: %s", address, esp_err_to_name(err));
        return err;
    }

    out_dev->i2c_dev = dev_handle;
    out_dev->address = address;

    ESP_LOGI(TAG, "PCA9554/PCA9554A anexado no endereço 0x%02X", address);
    return ESP_OK;
}

esp_err_t pca9554_deinit(pca9554_t *dev)
{
    if (dev == NULL || dev->i2c_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = i2c_master_bus_rm_device(dev->i2c_dev);
    dev->i2c_dev = NULL;
    return err;
}

/* ---------------------------- Acesso a registrador completo ---------------------------- */

esp_err_t pca9554_read_input_port(pca9554_t *dev, uint8_t *value)
{
    return pca9554_read_reg(dev, PCA9554_REG_INPUT_PORT, value);
}

esp_err_t pca9554_write_output_port(pca9554_t *dev, uint8_t value)
{
    return pca9554_write_reg(dev, PCA9554_REG_OUTPUT_PORT, value);
}

esp_err_t pca9554_read_output_port(pca9554_t *dev, uint8_t *value)
{
    return pca9554_read_reg(dev, PCA9554_REG_OUTPUT_PORT, value);
}

esp_err_t pca9554_write_polarity_inversion(pca9554_t *dev, uint8_t mask)
{
    return pca9554_write_reg(dev, PCA9554_REG_POLARITY_INV, mask);
}

esp_err_t pca9554_read_polarity_inversion(pca9554_t *dev, uint8_t *mask)
{
    return pca9554_read_reg(dev, PCA9554_REG_POLARITY_INV, mask);
}

esp_err_t pca9554_write_config(pca9554_t *dev, uint8_t mask)
{
    return pca9554_write_reg(dev, PCA9554_REG_CONFIG, mask);
}

esp_err_t pca9554_read_config(pca9554_t *dev, uint8_t *mask)
{
    return pca9554_read_reg(dev, PCA9554_REG_CONFIG, mask);
}

/* ---------------------------- Conveniência por pino individual ---------------------------- */

static esp_err_t pca9554_rmw_bit(pca9554_t *dev, pca9554_reg_t reg, uint8_t pin, bool bit_value)
{
    if (pin > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t current = 0;
    esp_err_t err = pca9554_read_reg(dev, reg, &current);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t updated = bit_value ? (current | (1 << pin)) : (current & ~(1 << pin));
    if (updated == current) {
        return ESP_OK; /* nada a fazer, evita transação I2C desnecessária */
    }

    return pca9554_write_reg(dev, reg, updated);
}

esp_err_t pca9554_set_pin_direction(pca9554_t *dev, uint8_t pin, pca9554_pin_dir_t direction)
{
    return pca9554_rmw_bit(dev, PCA9554_REG_CONFIG, pin, direction == PCA9554_PIN_INPUT);
}

esp_err_t pca9554_set_pin_level(pca9554_t *dev, uint8_t pin, bool level_high)
{
    return pca9554_rmw_bit(dev, PCA9554_REG_OUTPUT_PORT, pin, level_high);
}

esp_err_t pca9554_get_pin_level(pca9554_t *dev, uint8_t pin, bool *level_high)
{
    if (pin > 7 || level_high == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t input_port = 0;
    esp_err_t err = pca9554_read_input_port(dev, &input_port);
    if (err != ESP_OK) {
        return err;
    }

    *level_high = (input_port & (1 << pin)) != 0;
    return ESP_OK;
}

esp_err_t pca9554_set_pin_polarity_inversion(pca9554_t *dev, uint8_t pin, bool inverted)
{
    return pca9554_rmw_bit(dev, PCA9554_REG_POLARITY_INV, pin, inverted);
}
