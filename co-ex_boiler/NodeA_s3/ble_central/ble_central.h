#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#include <stdbool.h>
#include <stdint.h>
#include "coex_packet.h"

void ble_central_init(void);
void ble_central_get_stats(uint32_t *rx, uint32_t *tx);
bool ble_central_session_complete_b(void);
bool ble_central_session_complete_c(void);
const test_session_t *ble_central_get_session_b(void);
const test_session_t *ble_central_get_session_c(void);

#endif
