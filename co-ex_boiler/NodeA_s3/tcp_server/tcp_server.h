#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "coex_packet.h"

void tcp_server_start(void);
void tcp_server_get_stats(uint32_t *rx, uint32_t *tx);
bool tcp_server_session_complete(void);
const test_session_t *tcp_server_get_session(void);
