#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define portTICK_PERIOD_MS 1
#define portMAX_DELAY 0xFFFFFFFFu
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
