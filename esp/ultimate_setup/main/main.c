/**
 * ultimate_setup/main.c — Coexistence Test Firmware for ESP32-H2 (Nodes C & D)
 *
 * Node D (IS_NODE_D=1): Zigbee Coordinator + BLE Peripheral "BLE_NODE_D"
 *   - Sends coex_packet_t over 802.15.4 (broadcast) to Node C
 *   - Receives echoed ACKs → tracks Zigbee packet loss
 *   - BLE advertises so external A/B can connect and create interference
 *
 * Node C (IS_NODE_D=0): Zigbee End Device + BLE Peripheral "BLE_NODE_C"
 *   - Receives 802.15.4 packets from Node D, echoes back as ACK
 *   - BLE advertises so external A/B can connect and create interference
 *
 * Based on Haavvvels/main/main.c boilerplate structure.
 */

#include "sdkconfig.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "coex_packet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ============ ROLE & RADIO TOGGLES ============ */
#define IS_NODE_D 0  // 1 = Zigbee Coordinator (sends), 0 = End Device (echoes)
#define ENABLE_BLE 1 // 1 = BLE Peripheral advertising ON, 0 = OFF
#define ENABLE_ZIGBEE 1 // 1 = 802.15.4 link ON, 0 = OFF

/* ============ Channel Planning Config ============ */
// Wi-Fi CH1 (from Node A S3): 2412 MHz, occupies 2401–2423 MHz
// Zigbee CH20: 2450 MHz — AWAY from Wi-Fi to minimize overlap
// Zigbee CH13: 2415 MHz — MAXIMUM overlap with Wi-Fi CH1 (for stress testing)
// BLE: Hops across 37 data channels (2402-2480 MHz), 3 adv channels (37,38,39)
#define ZIGBEE_CHANNEL 20 // Change to 13 for maximum Wi-Fi overlap test

/* ============ Interference Duty Cycle Config ============ */
// Lower = more aggressive packet rate, must be >= 10ms (1 FreeRTOS tick)
#define ZIGBEE_SEND_INTERVAL_MS 100 // 10 packets/sec (same as boilerplate)

/* ============ Dashboard Config ============ */
#define DASHBOARD_EVERY_N 750 // Print analysis report every N packets

static const char *TAG = "COEX_TEST";

#if IS_NODE_D
#define DEVICE_MODE "NODE_D_COORD"
#else
#define DEVICE_MODE "NODE_C_ENDDEV"
#endif

/* =================================================================
 *  802.15.4 (Zigbee) Section — Raw frame TX/RX between C & D
 * ================================================================= */
#if ENABLE_ZIGBEE

#include "esp_ieee802154.h"

#define ZB_FRAME_HEADER_LEN 7 // FCF(2) + Seq(1) + PAN(2) + Dest(2)

/* Per-radio packet loss statistics */
static radio_stats_t zigbee_stats = {0};

/* Shared RX buffer (set from ISR callback) */
static volatile bool ieee_rx_pending = false;
static uint8_t ieee_rx_buf[128];
static int ieee_rx_len = 0;

/* 802.15.4 receive callback (ISR context — keep short!) */
void esp_ieee802154_receive_done(uint8_t *frame,
                                 esp_ieee802154_frame_info_t *frame_info) {
  if (frame && frame[0] > ZB_FRAME_HEADER_LEN) {
    int payload_len = frame[0] - ZB_FRAME_HEADER_LEN;
    if (payload_len <= (int)sizeof(ieee_rx_buf)) {
      memcpy(ieee_rx_buf, &frame[1 + ZB_FRAME_HEADER_LEN], payload_len);
      ieee_rx_len = payload_len;
      ieee_rx_pending = true;
    }
  }
}

/* ISR callbacks — NO printf/ESP_LOG allowed here (uses interrupted task's
 * stack!) */
void esp_ieee802154_transmit_done(const uint8_t *frame, const uint8_t *ack,
                                  esp_ieee802154_frame_info_t *ack_frame_info) {
  esp_ieee802154_receive(); /* re-arm receiver after TX */
}

static volatile uint32_t tx_fail_count = 0;
void esp_ieee802154_transmit_failed(const uint8_t *frame,
                                    esp_ieee802154_tx_error_t error) {
  tx_fail_count++;          /* safe from ISR — log from task context instead */
  esp_ieee802154_receive(); /* re-arm receiver after TX failure too */
}

/* Build and transmit a raw 802.15.4 frame carrying coex_packet_t */
static void zigbee_send_packet(coex_packet_t *pkt) {
  static uint8_t frame[128]; /* static: esp_ieee802154_transmit is async/DMA */
  int pkt_len = sizeof(coex_packet_t);

  if (pkt_len + ZB_FRAME_HEADER_LEN > 125) {
    ESP_LOGE(TAG, "Packet too large for 802.15.4 frame");
    return;
  }

  frame[0] = ZB_FRAME_HEADER_LEN + pkt_len; // PHR: length
  frame[1] = 0x01;                          // FCF low: data frame
  frame[2] = 0x08;                          // FCF high: no ack request
  frame[3] = (uint8_t)(pkt->seq & 0xFF);    // Sequence
  frame[4] = 0xFF;
  frame[5] = 0xFF; // PAN ID: broadcast
  frame[6] = 0xFF;
  frame[7] = 0xFF; // Dest addr: broadcast
  memcpy(&frame[1 + ZB_FRAME_HEADER_LEN], pkt, pkt_len);

  esp_ieee802154_transmit(frame, false);
}

static void zigbee_init(void) {
  esp_ieee802154_enable();
  esp_ieee802154_set_channel(ZIGBEE_CHANNEL);
  esp_ieee802154_set_txpower(20);
  esp_ieee802154_set_panid(0xFFFF);
  esp_ieee802154_set_promiscuous(true);
  esp_ieee802154_receive(); // Start listening
  ESP_LOGI(TAG, "802.15.4 initialized: CH=%d, TX=20dBm, promiscuous=ON",
           ZIGBEE_CHANNEL);
}

/* =================================================================
 *  NODE D (Coordinator) — Sends packets, receives ACKs, tracks loss
 * ================================================================= */
#if IS_NODE_D

static uint32_t zb_last_report_tx = 0;

void zigbee_tx_task(void *pvParameters) {
  ESP_LOGI(TAG, "Starting Zigbee TX Task (CH=%d, interval=%dms)",
           ZIGBEE_CHANNEL, ZIGBEE_SEND_INTERVAL_MS);

  static coex_packet_t pkt; /* static: keep off stack */
  uint16_t seq = 0;
  int64_t last_log = 0;

  while (1) {
    /* Build coex_packet_t — same struct as boilerplate main.c */
    pkt.header = 0xAA55;
    pkt.seq = seq++;
    pkt.tx_timestamp = esp_timer_get_time();
    pkt.mode = MODE_ZIGBEE;
    pkt.direction = DIR_REQUEST;
    pkt.rssi = 0; // TX side — no RX RSSI available
    pkt.lqi = 0;
    pkt.tx_power = 20; // dBm
    pkt.channel = ZIGBEE_CHANNEL;
    pkt.payload_len = snprintf((char *)pkt.payload, sizeof(pkt.payload),
                               "D_ZB_%u", (unsigned)pkt.seq);
    pkt.crc = crc16((uint8_t *)&pkt, sizeof(coex_packet_t) - 2);

    zigbee_send_packet(&pkt);
    zigbee_stats.tx_count++;

    /* Per-second log (same style as boilerplate) */
    if (esp_timer_get_time() - last_log > 1000000) {
      ESP_LOGI(TAG, "Sent seq %d, mode=%d, ch=%d", pkt.seq, pkt.mode,
               pkt.channel);
      last_log = esp_timer_get_time();
    }

    /* Dashboard report every 500 TX packets */
    if (zigbee_stats.tx_count - zb_last_report_tx >= DASHBOARD_EVERY_N) {
      zb_last_report_tx = zigbee_stats.tx_count;
      float loss = zigbee_stats.tx_count > 0
                       ? 100.0f * (1.0f - (float)zigbee_stats.rx_count /
                                              zigbee_stats.tx_count)
                       : 0.0f;
      ESP_LOGI(TAG, "");
      ESP_LOGI(TAG, "=== ZIGBEE ANALYSIS (NODE D) — %lu TX milestone ===",
               (unsigned long)zigbee_stats.tx_count);
      ESP_LOGI(TAG, "  TX:       %lu packets sent",
               (unsigned long)zigbee_stats.tx_count);
      ESP_LOGI(TAG, "  RX ACKs:  %lu received",
               (unsigned long)zigbee_stats.rx_count);
      ESP_LOGI(TAG, "  LOSS:     %.1f%% (%lu dropped)", loss,
               (unsigned long)(zigbee_stats.tx_count - zigbee_stats.rx_count));
      ESP_LOGI(TAG, "  Channel:  %d (%.0f MHz)", ZIGBEE_CHANNEL,
               2405.0 + 5.0 * (ZIGBEE_CHANNEL - 11));
      ESP_LOGI(TAG, "  BLE:      %s", ENABLE_BLE ? "ON (interference)" : "OFF");
      ESP_LOGI(TAG, "=================================================");
      ESP_LOGI(TAG, "");
    }

    vTaskDelay(pdMS_TO_TICKS(ZIGBEE_SEND_INTERVAL_MS));
  }
}

/* Task: receive ACKs (DIR_RESPONSE) from Node C */
void zigbee_rx_task(void *pvParameters) {
  while (1) {
    if (ieee_rx_pending && ieee_rx_len >= (int)sizeof(coex_packet_t)) {
      coex_packet_t *pkt = (coex_packet_t *)ieee_rx_buf;
      if (pkt->header == 0xAA55 && pkt->direction == DIR_RESPONSE) {
        zigbee_stats.rx_count++;
      }
      ieee_rx_pending = false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

#else
/* =================================================================
 *  NODE C (End Device) — Receives packets, echoes ACKs
 * ================================================================= */

static uint32_t zb_last_report_rx = 0;

void zigbee_echo_task(void *pvParameters) {
  ESP_LOGI(TAG, "Starting Zigbee Echo Task (CH=%d)", ZIGBEE_CHANNEL);

  while (1) {
    if (ieee_rx_pending && ieee_rx_len >= (int)sizeof(coex_packet_t)) {
      coex_packet_t *pkt = (coex_packet_t *)ieee_rx_buf;
      if (pkt->header == 0xAA55 && pkt->direction == DIR_REQUEST) {
        zigbee_stats.rx_count++;

        /* Echo back as response (same pattern as boilerplate) */
        static coex_packet_t ack; /* static: keep off stack */
        ack = *pkt;
        ack.direction = DIR_RESPONSE;
        ack.tx_timestamp = esp_timer_get_time();
        ack.payload_len = snprintf((char *)ack.payload, sizeof(ack.payload),
                                   "C_ACK_%u", (unsigned)ack.seq);
        ack.crc = crc16((uint8_t *)&ack, sizeof(coex_packet_t) - 2);
        zigbee_send_packet(&ack);
        zigbee_stats.tx_count++;

        /* Dashboard report every 500 RX packets */
        if (zigbee_stats.rx_count - zb_last_report_rx >= DASHBOARD_EVERY_N) {
          zb_last_report_rx = zigbee_stats.rx_count;
          ESP_LOGI(TAG, "");
          ESP_LOGI(TAG, "=== ZIGBEE ANALYSIS (NODE C) — %lu RX milestone ===",
                   (unsigned long)zigbee_stats.rx_count);
          ESP_LOGI(TAG, "  RX:       %lu packets received",
                   (unsigned long)zigbee_stats.rx_count);
          ESP_LOGI(TAG, "  ACKs Out: %lu sent back to D",
                   (unsigned long)zigbee_stats.tx_count);
          ESP_LOGI(TAG, "  Channel:  %d (%.0f MHz)", ZIGBEE_CHANNEL,
                   2405.0 + 5.0 * (ZIGBEE_CHANNEL - 11));
          ESP_LOGI(TAG, "  BLE:      %s",
                   ENABLE_BLE ? "ON (interference)" : "OFF");
          ESP_LOGI(TAG, "==================================================");
          ESP_LOGI(TAG, "");
        }
      }
      ieee_rx_pending = false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

#endif /* IS_NODE_D */
#endif /* ENABLE_ZIGBEE */

/* =================================================================
 *  BLE Peripheral — Advertises for external connections from A/B
 *  Creates real 2.4 GHz RF interference on the shared spectrum
 * ================================================================= */
#if ENABLE_BLE

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static radio_stats_t ble_stats = {0};
static uint16_t ble_conn_handle = 0xFFFF;
static uint32_t ble_last_report = 0;

/* Custom GATT Service UUID: 59462F12-9543-9999-12C8-58B459A2712D */
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x2D, 0x71, 0xA2, 0x59, 0xB4, 0x58, 0xC8, 0x12, 0x99, 0x99,
                     0x43, 0x95, 0x12, 0x2F, 0x46, 0x59);

/* Custom Characteristic UUID: 5C3A659E-... */
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0x9E, 0x65, 0x3A, 0x5C, 0xB4, 0x58, 0xC8, 0x12, 0x99, 0x99,
                     0x43, 0x95, 0x12, 0x2F, 0x46, 0x59);

static uint16_t chr_val_handle = 0;

/* GATT write handler: receive packets from external nodes A/B, echo back */
static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    if (OS_MBUF_PKTLEN(ctxt->om) >= sizeof(coex_packet_t)) {
      coex_packet_t pkt;
      os_mbuf_copydata(ctxt->om, 0, sizeof(pkt), &pkt);
      if (pkt.header == 0xAA55) {
        ble_stats.rx_count++;

        /* Echo back via notification */
        pkt.direction = DIR_RESPONSE;
        pkt.crc = crc16((uint8_t *)&pkt, sizeof(pkt) - 2);
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt));
        ble_gatts_notify_custom(conn_handle, chr_val_handle, om);
        ble_stats.tx_count++;

        /* BLE analysis every 500 packets */
        if (ble_stats.rx_count - ble_last_report >= DASHBOARD_EVERY_N) {
          ble_last_report = ble_stats.rx_count;
          ESP_LOGI(TAG, "");
          ESP_LOGI(TAG, "=== BLE ANALYSIS (%s) — %lu RX milestone ===",
                   IS_NODE_D ? "NODE D" : "NODE C",
                   (unsigned long)ble_stats.rx_count);
          ESP_LOGI(TAG, "  RX:       %lu from external (A/B)",
                   (unsigned long)ble_stats.rx_count);
          ESP_LOGI(TAG, "  ACKs Out: %lu notified back",
                   (unsigned long)ble_stats.tx_count);
          ESP_LOGI(TAG, "==========================================");
          ESP_LOGI(TAG, "");
        }
      }
    }
  }
  return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &gatt_svr_chr_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &chr_val_handle,
                },
                {0}},
    },
    {0}};

static void ble_app_advertise(void);

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      ble_conn_handle = event->connect.conn_handle;
      ESP_LOGI(TAG, "BLE: External device connected (handle=%d)",
               ble_conn_handle);
    } else {
      ble_app_advertise();
    }
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "BLE: External device disconnected");
    ble_conn_handle = 0xFFFF;
    ble_app_advertise();
    break;
  default:
    break;
  }
  return 0;
}

static void ble_app_advertise(void) {
  struct ble_hs_adv_fields fields;
  memset(&fields, 0, sizeof fields);
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

  const char *name = IS_NODE_D ? "BLE_NODE_D" : "BLE_NODE_C";
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
  fields.name_is_complete = 1;

  ble_gap_adv_set_fields(&fields);

  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                    ble_gap_event_cb, NULL);
  ESP_LOGI(TAG, "BLE advertising started as '%s'", name);
}

static void ble_on_sync(void) { ble_app_advertise(); }

static void ble_host_task(void *param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void ble_init_and_start(void) {
  ESP_LOGI(TAG, "Initializing NimBLE...");
  ESP_ERROR_CHECK(nimble_port_init());

  ble_gatts_count_cfg(gatt_svr_svcs);
  ble_gatts_add_svcs(gatt_svr_svcs);
  ble_hs_cfg.sync_cb = ble_on_sync;
  ble_svc_gap_device_name_set(IS_NODE_D ? "BLE_NODE_D" : "BLE_NODE_C");
  nimble_port_freertos_init(ble_host_task);

  ESP_LOGI(TAG,
           "NimBLE initialized — advertising for external A/B connections");
}

#endif /* ENABLE_BLE */

/* =================================================================
 *  app_main — entry point (same structure as boilerplate main.c)
 * ================================================================= */
void app_main(void) {
  ESP_LOGI(TAG, "Running Device Mode: %s", DEVICE_MODE);
  ESP_LOGI(TAG, "  Zigbee: %s (CH=%d, %.0f MHz)", ENABLE_ZIGBEE ? "ON" : "OFF",
           ZIGBEE_CHANNEL, 2405.0 + 5.0 * (ZIGBEE_CHANNEL - 11));
  ESP_LOGI(TAG, "  BLE:    %s", ENABLE_BLE ? "ON" : "OFF");
  ESP_LOGI(TAG, "  Report: Every %d packets", DASHBOARD_EVERY_N);

  /* Initialize NVS (required for BLE) */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* --- Zigbee 802.15.4 --- */
#if ENABLE_ZIGBEE
  zigbee_init();
#if IS_NODE_D
  xTaskCreate(zigbee_tx_task, "zb_tx", 16384, NULL, 5, NULL);
  xTaskCreate(zigbee_rx_task, "zb_rx", 8192, NULL, 5, NULL);
#else
  xTaskCreate(zigbee_echo_task, "zb_echo", 16384, NULL, 5, NULL);
#endif
#endif

  /* --- BLE Peripheral (external interference from A/B) --- */
#if ENABLE_BLE
  ble_init_and_start();
#endif
}
