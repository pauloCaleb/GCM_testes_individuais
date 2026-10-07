#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef int gpio_num_t;
#define GPIO_NUM_4 4
#define GPIO_NUM_13 13
#define GPIO_NUM_14 14
#define GPIO_NUM_16 16
#define GPIO_NUM_17 17
#define GPIO_NUM_18 18
#define GPIO_NUM_19 19
#define GPIO_NUM_23 23
#define GPIO_NUM_25 25
#define GPIO_NUM_27 27
#define GPIO_NUM_34 34
#define GPIO_NUM_35 35
typedef enum { GPIO_MODE_INPUT=1, GPIO_MODE_OUTPUT=2 } gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE, GPIO_PULLUP_ENABLE } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE, GPIO_PULLDOWN_ENABLE } gpio_pulldown_t;
typedef enum { GPIO_INTR_DISABLE } gpio_int_type_t;
typedef struct { uint64_t pin_bit_mask; gpio_mode_t mode; gpio_pullup_t pull_up_en; gpio_pulldown_t pull_down_en; gpio_int_type_t intr_type; } gpio_config_t;
esp_err_t gpio_config(const gpio_config_t*);
esp_err_t gpio_set_level(gpio_num_t, uint32_t);
int gpio_get_level(gpio_num_t);
