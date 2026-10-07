#pragma once
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
void vTaskDelay(TickType_t t);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *p, TickType_t t);
BaseType_t xTaskCreate(void (*fn)(void*), const char *name, uint32_t stack, void *arg, int prio, TaskHandle_t *h);
