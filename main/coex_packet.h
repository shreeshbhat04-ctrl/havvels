#pragma once

#include <stdint.h>

#define MODE_WIFI 1
#define MODE_ZIGBEE 2
#define MODE_BLE 3

typedef struct __attribute__((packed)) {
  uint16_t header; // 0xAA55
  uint16_t seq;
  uint64_t tx_timestamp; // us
  uint8_t mode;
  int8_t rssi;
  uint8_t lqi;
  int8_t tx_power; // dBm — transmit power used
  uint8_t channel; // Wi-Fi channel (1-13) or 802.15.4 channel (11-26)
  uint16_t payload_len;
  uint8_t payload[64];
  uint16_t crc;
} coex_packet_t;

// CRC16 (implemented in coex_packet.c)
uint16_t crc16(uint8_t *data, uint16_t len);
