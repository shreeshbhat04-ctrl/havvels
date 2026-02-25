#include "coex_packet.h"

// Simple CRC16 implementation
uint16_t crc16(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}
