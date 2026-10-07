#pragma once
#include <stdio.h>
#define ESP_LOGI(t,f,...) printf(f,##__VA_ARGS__)
#define ESP_LOGW(t,f,...) printf(f,##__VA_ARGS__)
#define ESP_LOGE(t,f,...) printf(f,##__VA_ARGS__)
