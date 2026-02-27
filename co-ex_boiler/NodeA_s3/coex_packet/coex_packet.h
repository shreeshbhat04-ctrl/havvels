#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ================= RADIO MODES ================= */
#define MODE_WIFI 1
#define MODE_ZIGBEE 2
#define MODE_BLE 3

/* Packet direction (master <-> node) */
#define DIR_REQUEST 0x01  /* Master (Node A) -> Node */
#define DIR_RESPONSE 0x02 /* Node -> Master (echo/ack) */

/* ================= TEST CONFIG ================= */
#define TEST_PACKET_COUNT 1000

/* ================= COEX PACKET ================= */

typedef struct __attribute__((packed)) {
  uint16_t header;       // 0xAA55
  uint16_t seq;
  uint64_t tx_timestamp; // us
  uint8_t mode;
  uint8_t direction;     // DIR_REQUEST or DIR_RESPONSE
  int8_t rssi;
  uint8_t lqi;
  int8_t tx_power;       // dBm
  uint8_t channel;       // Wi-Fi (1-13) or 802.15.4 (11-26)
  uint16_t payload_len;
  uint8_t payload[64];
  uint16_t crc;
} coex_packet_t; // 86 bytes (packed)

/* ================= TEST SESSION ================= */

typedef struct {
  uint32_t target_count;   /* Packets per test window (1000) */
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t lost_count;     /* Detected via seq gaps */
  uint16_t last_seq;       /* Last seen sequence number */
  int64_t start_time_us;
  int64_t end_time_us;
  int64_t last_arrival_us; /* For jitter calculation */
  int64_t jitter_sum_us;   /* Sum of |delta_i - delta_i-1| */
  int64_t latency_sum_us;  /* Sum of one-way latencies */
  int32_t rssi_sum;
  int8_t rssi_min;
  int8_t rssi_max;
  uint32_t total_bytes;
  bool complete;
} test_session_t;

/* Per-radio packet loss statistics (legacy) */
typedef struct {
  uint32_t tx_count;
  uint32_t rx_count;
} radio_stats_t;

/* CRC16 (implemented in coex_packet.c) */
uint16_t crc16(uint8_t *data, uint16_t len);
