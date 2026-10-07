#pragma once
/* Link Bluetooth Classic (SPP) espelhando a saida serial. Aparece no PC como porta COM. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void bt_link_init(void);
bool bt_link_connected(void);
bool bt_link_send(const char *s, size_t len, bool droppable);   /* nao bloqueia */
int  bt_link_free_slots(void);                                  /* 99 se nao ha cliente */
size_t bt_link_read(uint8_t *buf, size_t max);                  /* bytes recebidos do cliente */
bool bt_link_take_new_connection(void);                         /* true uma vez por conexao */
uint32_t bt_link_dropped(void);
