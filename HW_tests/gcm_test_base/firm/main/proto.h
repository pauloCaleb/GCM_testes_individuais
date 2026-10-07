/**
 * @file proto.h
 * @brief Protocolo serial JSON-lines da GCM (ver PROTOCOL.md na raiz do projeto).
 */
#pragma once

#include "esp_err.h"

/** Instala o driver da UART (RX) e registra os callbacks de eventos. */
esp_err_t proto_init(void);

/** Cria as tasks de RX (comandos) e TX (telemetria periódica). */
void proto_start_tasks(void);

/** Envia a mensagem "hello" (chamada no fim do boot e sob o comando "hello"). */
void proto_send_hello(void);
