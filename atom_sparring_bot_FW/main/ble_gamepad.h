#pragma once
#include <stdbool.h>

typedef void (*ble_gamepad_conn_cb_t)(bool connected);

/* Sobe o stack NimBLE, registra o serviço compatível com o Dabble
 * (Nordic UART: 6E400001/2/3-B5A3-F393-E0A9-E50E24DCCA9E) e começa a anunciar
 * com o nome informado. Reanuncia automaticamente após desconexão. */
void ble_gamepad_start(const char *name, ble_gamepad_conn_cb_t cb);

bool ble_gamepad_is_connected(void);
