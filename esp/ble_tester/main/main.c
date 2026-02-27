#include "coex_packet.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <stdio.h>
#include <string.h>

// ==========================================================
// ROLE SELECTOR
// Change IS_CENTRAL to 1 for the Master (Access Point equivalent)
// Change IS_CENTRAL to 0 for the Peripheral (Node equivalent)
// ==========================================================
#define IS_CENTRAL 0

static const char *TAG = "BLE_TEST";

// Custom Service UUID: 59462F12-9543-9999-12C8-58B459A2712D
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x2D, 0x71, 0xA2, 0x59, 0xB4, 0x58, 0xC8, 0x12, 0x99, 0x99,
                     0x43, 0x95, 0x12, 0x2F, 0x46, 0x59);

// Custom Characteristic UUID (for TX/RX): 5C3A659E-...
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0x9E, 0x65, 0x3A, 0x5C, 0xB4, 0x58, 0xC8, 0x12, 0x99, 0x99,
                     0x43, 0x95, 0x12, 0x2F, 0x46, 0x59);

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static radio_stats_t ble_stats = {0};

void ble_app_advertise(void);
void ble_app_scan(void);

// ================= PERIPHERAL LOGIC (IS_CENTRAL == 0) =================
#if !IS_CENTRAL

static uint16_t chr_val_handle = 0;

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    if (OS_MBUF_PKTLEN(ctxt->om) >= sizeof(coex_packet_t)) {
      coex_packet_t pkt;
      os_mbuf_copydata(ctxt->om, 0, sizeof(pkt), &pkt);
      if (pkt.header == 0xAA55 && pkt.direction == DIR_REQUEST) {
        ble_stats.rx_count++;

        // Echo back as response
        pkt.direction = DIR_RESPONSE;
        pkt.crc = crc16((uint8_t *)&pkt, sizeof(pkt) - 2);

        // Send notification
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt));
        ble_gatts_notify_custom(conn_handle, chr_val_handle, om);
        ble_stats.tx_count++;
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
                    .access_cb = gatt_svr_chr_access,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &chr_val_handle,
                },
                {0}},
    },
    {0}};

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      conn_handle = event->connect.conn_handle;
      ESP_LOGI(TAG, "Connected as Peripheral");
    } else {
      ble_app_advertise();
    }
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "Disconnected");
    conn_handle = BLE_HS_CONN_HANDLE_NONE;
    ble_app_advertise();
    break;
  }
  return 0;
}

void ble_app_advertise(void) {
  struct ble_hs_adv_fields fields;
  memset(&fields, 0, sizeof fields);
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  const char *name = "BLE_TESTER";
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
  fields.name_is_complete = 1;

  ble_gap_adv_set_fields(&fields);

  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                    ble_gap_event, NULL);
  ESP_LOGI(TAG, "Advertising...");
}

#endif // !IS_CENTRAL

// ================= CENTRAL LOGIC (IS_CENTRAL == 1) =================
#if IS_CENTRAL

static uint16_t peer_chr_val_handle = 0;

static int on_disc_c(uint16_t conn_h, const struct ble_gatt_error *error,
                     const struct ble_gatt_chr *chr, void *arg) {
  if (error->status == 0) {
    if (ble_uuid_cmp(&chr->uuid.u, &gatt_svr_chr_uuid.u) == 0) {
      peer_chr_val_handle = chr->val_handle;
      ESP_LOGI(TAG, "Discovered characteristic handle: %d",
               peer_chr_val_handle);
      // Subscribe to notifications by writing 0x01 to CCCD
      uint8_t enable_notify[2] = {1, 0};
      ble_gattc_write_no_rsp_flat(conn_h, peer_chr_val_handle + 1,
                                  enable_notify, sizeof(enable_notify));
    }
  }
  return 0;
}

static int on_disc_s(uint16_t conn_h, const struct ble_gatt_error *error,
                     const struct ble_gatt_svc *service, void *arg) {
  if (error->status == 0) {
    if (ble_uuid_cmp(&service->uuid.u, &gatt_svr_svc_uuid.u) == 0) {
      ESP_LOGI(TAG, "Discovered service, discovering characteristics...");
      ble_gattc_disc_all_chrs(conn_h, service->start_handle,
                              service->end_handle, on_disc_c, NULL);
    }
  }
  return 0;
}

static int ble_gap_event_central(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_EXT_DISC:
  case BLE_GAP_EVENT_DISC: {
    // Find "BLE_TESTER"
    struct ble_hs_adv_fields fields;
    ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
    if (fields.name_len > 0 &&
        strncmp((char *)fields.name, "BLE_TESTER", fields.name_len) == 0) {
      ESP_LOGI(TAG, "Found peripheral, connecting...");
      ble_gap_disc_cancel();
      ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, NULL,
                      ble_gap_event_central, NULL);
    }
    break;
  }
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      conn_handle = event->connect.conn_handle;
      ESP_LOGI(TAG, "Connected to Peripheral!");
      ble_gattc_exchange_mtu(conn_handle, NULL, NULL);
    } else {
      ble_app_scan();
    }
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "Disconnected");
    conn_handle = BLE_HS_CONN_HANDLE_NONE;
    peer_chr_val_handle = 0;
    ble_app_scan();
    break;
  case BLE_GAP_EVENT_MTU:
    ESP_LOGI(TAG, "MTU exchanged, discovering services...");
    ble_gattc_disc_all_svcs(conn_handle, on_disc_s, NULL);
    break;
  case BLE_GAP_EVENT_NOTIFY_RX:
    if (event->notify_rx.attr_handle == peer_chr_val_handle) {
      if (OS_MBUF_PKTLEN(event->notify_rx.om) >= sizeof(coex_packet_t)) {
        coex_packet_t pkt;
        os_mbuf_copydata(event->notify_rx.om, 0, sizeof(pkt), &pkt);
        if (pkt.header == 0xAA55 && pkt.direction == DIR_RESPONSE) {
          ble_stats.rx_count++;
          uint32_t rtt = (uint32_t)(esp_timer_get_time() - pkt.tx_timestamp);
          // ESP_LOGI(TAG, "ACK RX seq=%d RTT=%lu us", pkt.seq, rtt);
        }
      }
    }
    break;
  }
  return 0;
}

void ble_app_scan(void) {
  struct ble_gap_disc_params disc_params;
  memset(&disc_params, 0, sizeof disc_params);
  disc_params.filter_duplicates = 1;
  disc_params.passive = 0;
  disc_params.itvl = 0;
  disc_params.window = 0;
  ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
               ble_gap_event_central, NULL);
  ESP_LOGI(TAG, "Scanning...");
}

// Task to periodically send packets
static void central_tx_task(void *arg) {
  uint16_t seq = 0;
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(100)); // 10 packets per second
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE && peer_chr_val_handle != 0) {
      coex_packet_t pkt = {
          .header = 0xAA55,
          .seq = seq++,
          .tx_timestamp = esp_timer_get_time(),
          .mode = MODE_BLE,
          .direction = DIR_REQUEST,
      };
      pkt.crc = crc16((uint8_t *)&pkt, sizeof(pkt) - 2);
      // Write without response
      ble_gattc_write_no_rsp_flat(conn_handle, peer_chr_val_handle, &pkt,
                                  sizeof(pkt));
      ble_stats.tx_count++;

      // Periodically log TX progress
      if (ble_stats.tx_count % 50 == 0) {
        ESP_LOGI(TAG, "Sent %lu packets...", ble_stats.tx_count);
      }
    }
  }
}
#endif // IS_CENTRAL

// ================= SHARED LOGIC =================
static void dashboard_task(void *arg) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(5000));
#if IS_CENTRAL
    float loss =
        ble_stats.tx_count > 0
            ? 100.0f * (1.0f - (float)ble_stats.rx_count / ble_stats.tx_count)
            : 0.0f;
    ESP_LOGI(TAG,
             "============= PURE BLE COEX DASHBOARD (MASTER) =============");
    ESP_LOGI(TAG, "TX: %lu | RX ACKs: %lu | Loss: %.1f%%", ble_stats.tx_count,
             ble_stats.rx_count, loss);
    ESP_LOGI(TAG,
             "============================================================");
#else
    ESP_LOGI(TAG,
             "============= PURE BLE COEX DASHBOARD (NODE) ===============");
    ESP_LOGI(TAG, "Packets Received: %lu | ACKs Sent: %lu", ble_stats.rx_count,
             ble_stats.tx_count);
    ESP_LOGI(TAG,
             "============================================================");
#endif
  }
}

void ble_hs_cfg_sync(void) {
#if IS_CENTRAL
  ble_app_scan();
#else
  ble_app_advertise();
#endif
}

void nimble_host_task(void *param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "Initializing NimBLE (Role: %s)",
           IS_CENTRAL ? "Central/Master" : "Peripheral/Node");

  ESP_ERROR_CHECK(nimble_port_init());

#if !IS_CENTRAL
  ble_gatts_count_cfg(gatt_svr_svcs);
  ble_gatts_add_svcs(gatt_svr_svcs);
#endif

  ble_hs_cfg.sync_cb = ble_hs_cfg_sync;
  // Set device name
  ble_svc_gap_device_name_set("BLE_TESTER");
  nimble_port_freertos_init(nimble_host_task);

#if IS_CENTRAL
  xTaskCreate(central_tx_task, "ctx", 4096, NULL, 5, NULL);
#endif

  xTaskCreate(dashboard_task, "dash", 4096, NULL, 3, NULL);
}
