/*
 * ============================================================
 *  BLE Central — Starter Boilerplate (NimBLE, ESP-IDF)
 * ============================================================
 *
 *  What this code does:
 *   - Initializes BLE stack (NimBLE)
 *   - Scans for peripherals
 *   - Connects to target devices
 *   - Discovers services & characteristics
 *   - Subscribes to notifications
 *   - Sends 1000-packet batches with seq tracking
 *   - Tracks per-node BLE session stats
 *
 * ============================================================
 */

#include <assert.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "coex_packet.h"

/* ================= COMPAT ================= */

#ifndef BLE_HS_CONN_HANDLE_NONE
#define BLE_HS_CONN_HANDLE_NONE 0xFFFF
#endif

#ifndef BLE_HS_EDONE
#define BLE_HS_EDONE 14
#endif

/* ================= CONFIG ================= */

#define TAG "BLE_CENTRAL"

#define DEVICE_NAME_1 "NODE-B(classic)"
#define DEVICE_NAME_2 "NODE-C(H2)"

/* ================= GLOBAL STATE ================= */

static uint16_t conn_nodeb = BLE_HS_CONN_HANDLE_NONE;
static uint16_t conn_nodec = BLE_HS_CONN_HANDLE_NONE;
static uint16_t char_nodeb = 0;
static uint16_t char_nodec = 0;

static volatile uint32_t g_ble_rx = 0;
static volatile uint32_t g_ble_tx = 0;

static bool connect_in_progress = false;
static int last_found_device = 0; // 1 = NodeB, 2 = NodeC

/* Per-node session tracking */
static test_session_t g_ble_session_b;
static test_session_t g_ble_session_c;

/* Forward declaration */
static void ble_app_scan(void);

/* ================= SESSION HELPERS ================= */

static void ble_session_reset(test_session_t *s) {
  memset(s, 0, sizeof(*s));
  s->target_count = TEST_PACKET_COUNT;
  s->rssi_min = 0;
  s->rssi_max = -127;
  s->complete = false;
}

static void print_ble_session_report(const char *node, test_session_t *s) {
  double duration = (s->end_time_us - s->start_time_us) / 1000000.0;
  double throughput_kbps =
      (duration > 0.001) ? (s->total_bytes * 8.0) / (duration * 1000.0) : 0.0;
  double avg_jitter =
      (s->rx_count > 1) ? (double)s->jitter_sum_us / (s->rx_count - 1) : 0.0;
  double pdr =
      s->target_count > 0 ? 100.0 * s->rx_count / s->target_count : 0.0;

  ESP_LOGI(TAG, "+--------------------------------------+");
  ESP_LOGI(TAG, "|  BLE %s SESSION REPORT", node);
  ESP_LOGI(TAG, "+--------------------------------------+");
  ESP_LOGI(TAG, "|  TX:         %lu", (unsigned long)s->tx_count);
  ESP_LOGI(TAG, "|  RX:         %lu / %lu",
           (unsigned long)s->rx_count, (unsigned long)s->target_count);
  ESP_LOGI(TAG, "|  Lost:       %lu", (unsigned long)s->lost_count);
  ESP_LOGI(TAG, "|  PDR:        %.1f%%", pdr);
  ESP_LOGI(TAG, "|  Bytes RX:   %lu", (unsigned long)s->total_bytes);
  ESP_LOGI(TAG, "|  Duration:   %.2f s", duration);
  ESP_LOGI(TAG, "|  Throughput: %.1f kbps", throughput_kbps);
  ESP_LOGI(TAG, "|  Avg Jitter: %.1f us", avg_jitter);
  ESP_LOGI(TAG, "+--------------------------------------+");
}

/* ================= HELPER FUNCTIONS ================= */

static bool is_my_device(const struct ble_gap_event *event) {
  uint8_t i = 0;
  char name[32] = {0};

  while (i < event->disc.length_data) {

    uint8_t len = event->disc.data[i++];
    if (len == 0)
      break;

    uint8_t type = event->disc.data[i];

    if (type == 0x08 || type == 0x09) {

      uint8_t nlen = len - 1;
      if (nlen > 31)
        nlen = 31;

      memcpy(name, &event->disc.data[i + 1], nlen);
      name[nlen] = '\0';

      if (!strcmp(name, DEVICE_NAME_1)) {
        last_found_device = 1;
        return true;
      }

      if (!strcmp(name, DEVICE_NAME_2)) {
        last_found_device = 2;
        return true;
      }
    }

    i += len;
  }

  return false;
}

/* ================= NOTIFICATION SUBSCRIPTION ================= */

static void subscribe_notifications(uint16_t conn_handle,
                                    uint16_t val_handle) {
  uint16_t cccd_handle = val_handle + 1;
  uint8_t cccd_val[2] = {0x01, 0x00}; /* Enable notifications */
  int rc = ble_gattc_write_flat(conn_handle, cccd_handle, cccd_val,
                                sizeof(cccd_val), NULL, NULL);
  ESP_LOGI(TAG, "Subscribe notifications (CCCD handle=%d) rc=%d", cccd_handle,
           rc);
}

/* ================= GATT DISCOVERY CALLBACKS ================= */

static int char_disc_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg) {
  if (error->status != 0 && error->status != BLE_HS_EDONE)
    return error->status;

  if (error->status == BLE_HS_EDONE) {
    ESP_LOGI(TAG, "Characteristic discovery complete");
    return 0;
  }

  /* Check for our custom characteristic UUID */
  if (chr->uuid.u.type == BLE_UUID_TYPE_128) {
    if (conn_handle == conn_nodeb) {
      char_nodeb = chr->val_handle;
      ESP_LOGI(TAG, "NodeB custom characteristic found: %d", char_nodeb);
      subscribe_notifications(conn_handle, char_nodeb);
    } else if (conn_handle == conn_nodec) {
      char_nodec = chr->val_handle;
      ESP_LOGI(TAG, "NodeC custom characteristic found: %d", char_nodec);
      subscribe_notifications(conn_handle, char_nodec);
    }
  }

  return 0;
}

/* ================= GAP EVENT CALLBACK ================= */

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {

  /* ---------- DEVICE DISCOVERED ---------- */
  case BLE_GAP_EVENT_DISC:

    if (connect_in_progress)
      return 0;

    if (!is_my_device(event))
      return 0;

    if (last_found_device == 1 && conn_nodeb != BLE_HS_CONN_HANDLE_NONE)
      return 0;

    if (last_found_device == 2 && conn_nodec != BLE_HS_CONN_HANDLE_NONE)
      return 0;

    ESP_LOGI(TAG, "Connecting to target device...");
    connect_in_progress = true;

    ble_gap_disc_cancel();

    /* Custom connect params: WiFi coex needs a generous supervision timeout
       so the connection survives WiFi bursts on the S3. */
    struct ble_gap_conn_params conn_params = {
        .scan_itvl = 0x0060,           /* 96 * 0.625ms = 60ms */
        .scan_window = 0x0030,         /* 48 * 0.625ms = 30ms */
        .itvl_min = 0x0018,            /* 24 * 1.25ms = 30ms */
        .itvl_max = 0x0030,            /* 48 * 1.25ms = 60ms */
        .latency = 0,
        .supervision_timeout = 0x0258, /* 600 * 10ms = 6.0s */
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000,
                    &conn_params, gap_event_cb, NULL);
    return 0;/// important for coexxistence

  /* ---------- CONNECTION COMPLETE ---------- */
  case BLE_GAP_EVENT_CONNECT:

    connect_in_progress = false;

    if (event->connect.status != 0) {
      ESP_LOGE(TAG, "Connection failed");
      ble_app_scan();
      return 0;
    }

    if (last_found_device == 1) {
      conn_nodeb = event->connect.conn_handle;
      ESP_LOGI(TAG, "Connected to NodeB (handle %d)", conn_nodeb);
    } else {
      conn_nodec = event->connect.conn_handle;
      ESP_LOGI(TAG, "Connected to NodeC (handle %d)", conn_nodec);
    }

    ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);

    ble_app_scan();
    return 0;

  /* ---------- DISCONNECTED ---------- */
  case BLE_GAP_EVENT_DISCONNECT:

    if (event->disconnect.conn.conn_handle == conn_nodeb) {
      conn_nodeb = BLE_HS_CONN_HANDLE_NONE;
      char_nodeb = 0;
      ESP_LOGW(TAG, "NodeB disconnected (reason=0x%02x)",
               event->disconnect.reason);
    }

    if (event->disconnect.conn.conn_handle == conn_nodec) {
      conn_nodec = BLE_HS_CONN_HANDLE_NONE;
      char_nodec = 0;
      ESP_LOGW(TAG, "NodeC disconnected (reason=0x%02x)",
               event->disconnect.reason);
    }

    ble_app_scan();
    return 0;

  /* ---------- MTU UPDATE ---------- */
  case BLE_GAP_EVENT_MTU:

    ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);

    /* Request connection parameters that work with WiFi coex on S3 */
    {
      struct ble_gap_upd_params conn_params = {
          .itvl_min = 0x0018,          /* 24 * 1.25ms = 30ms min */
          .itvl_max = 0x0030,          /* 48 * 1.25ms = 60ms max */
          .latency = 0,                /* no skipping — need throughput */
          .supervision_timeout = 0x0258 /* 600 * 10ms = 6.0s */
      };
      ble_gap_update_params(event->mtu.conn_handle, &conn_params);
      ESP_LOGI(TAG, "Requested conn interval 30-60ms, timeout 6s for handle %d",
               event->mtu.conn_handle);
    }

    ble_gattc_disc_all_chrs(event->mtu.conn_handle, 1, 0xFFFF, char_disc_cb,
                            NULL);
    return 0;

  /* ---------- NOTIFICATION RECEIVED ---------- */
  case BLE_GAP_EVENT_NOTIFY_RX: {
    g_ble_rx++;
    int64_t now = esp_timer_get_time();

    test_session_t *session = NULL;
    const char *node_name = NULL;

    if (event->notify_rx.conn_handle == conn_nodeb) {
      session = &g_ble_session_b;
      node_name = "NodeB";
    } else if (event->notify_rx.conn_handle == conn_nodec) {
      session = &g_ble_session_c;
      node_name = "NodeC";
    }

    if (session) {
      /* Extract seq from echoed packet [0xAA, 0x55, seq_lo, seq_hi] */
      uint16_t seq = 0;
      uint16_t pktlen = OS_MBUF_PKTLEN(event->notify_rx.om);
      if (pktlen >= 4) {
        uint8_t *buf = event->notify_rx.om->om_data;
        if (buf[0] == 0xAA && buf[1] == 0x55) {
          seq = buf[2] | (buf[3] << 8);
        }
      }

      /* Start session on first RX */
      if (session->rx_count == 0 && session->start_time_us == 0) {
        ble_session_reset(session);
        session->start_time_us = now;
      }

      /* Jitter */
      if (session->last_arrival_us > 0) {
        int64_t delta = now - session->last_arrival_us;
        if (delta < 0)
          delta = -delta;
        session->jitter_sum_us += delta;
      }
      session->last_arrival_us = now;

      /* Loss detection via seq gap */
      if (session->rx_count > 0) {
        int gap = (int)seq - (int)session->last_seq - 1;
        if (gap > 0)
          session->lost_count += gap;
      }
      session->last_seq = seq;

      session->rx_count++;
      session->total_bytes += pktlen;

      /* Session complete? */
      if (session->rx_count >= session->target_count) {
        session->end_time_us = now;
        session->complete = true;
        print_ble_session_report(node_name, session);
        ble_session_reset(session);
        session->start_time_us = esp_timer_get_time();
      }
    }
    return 0;
  }

  case BLE_GAP_EVENT_SUBSCRIBE:
    ESP_LOGI(TAG, "Subscribe event: cur_notify=%d",
             event->subscribe.cur_notify);
    return 0;

  default:
    return 0;
  }
}

/* ================= SCANNING ================= */

static void ble_app_scan(void) {
  /* Stop scanning when both nodes are connected — saves radio for WiFi */
  if (conn_nodeb != BLE_HS_CONN_HANDLE_NONE &&
      conn_nodec != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG, "Both nodes connected, scanning stopped");
    return;
  }

  struct ble_gap_disc_params params = {
      .itvl = 0x0060,   /* 96 * 0.625ms = 60ms (was 10ms) */
      .window = 0x0010,  /* 16 * 0.625ms = 10ms (~17% duty) */
      .passive = 0,
      .filter_duplicates = 1};

  ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_event_cb,
               NULL);

  ESP_LOGI(TAG, "Scanning started...");
}

/* ================= BLE STACK READY ================= */

static void ble_on_sync(void) {
  uint8_t addr_type;

  ble_hs_util_ensure_addr(0);
  ble_hs_id_infer_auto(0, &addr_type);

  ESP_LOGI(TAG, "BLE stack synced");

  ble_app_scan();
}

/* ================= HOST TASK ================= */

static void ble_host_task(void *param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

/* ================= PUBLIC FUNCTIONS ================= */

void ble_central_get_stats(uint32_t *rx, uint32_t *tx) {
  if (rx)
    *rx = g_ble_rx;
  if (tx)
    *tx = g_ble_tx;
}

bool ble_central_session_complete_b(void) {
  return g_ble_session_b.complete;
}

bool ble_central_session_complete_c(void) {
  return g_ble_session_c.complete;
}

const test_session_t *ble_central_get_session_b(void) {
  return &g_ble_session_b;
}

const test_session_t *ble_central_get_session_c(void) {
  return &g_ble_session_c;
}

void ble_central_init(void) {

  nimble_port_init();

  ble_hs_cfg.sync_cb = ble_on_sync;

  nimble_port_freertos_init(ble_host_task);

  ESP_LOGI(TAG, "BLE Central initialized (RX-only)");
}
