/*
 * ============================================================
 *  BLE Peripheral — Project Boilerplate (NimBLE, ESP-IDF)
 * ============================================================
 *
 *  This file provides:
 *   ✔ BLE stack initialization
 *   ✔ One custom GATT service + characteristic
 *   ✔ Advertising with device name and UUID
 *   ✔ Connection handling
 *
 *  Modify the CONFIG section as required.
 *  Application-specific behavior should be added inside
 *  the marked callback sections.
 *
 * ============================================================
 */

#include <assert.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_uuid.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ================= CONFIGURATION ================= */

#define TAG "BLE_PERIPHERAL"

/* Device name shown during advertising */
#define DEVICE_NAME "NODE-C(H2)"

/* 16-bit UUID included in advertisement packet */
#define ADV_UUID16 0xABCD

#define BLE_SEND_INTERVAL_MS  60
#define TEST_PACKET_COUNT    1000

/* ================= CUSTOM UUIDs ================= */

/* Replace with project-specific UUIDs if needed */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t chr_uuid =
    BLE_UUID128_INIT(0xac, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* Handle used for notifications */
static uint16_t char_handle;

/* Connection handle for sending notifications back */
static uint16_t g_conn_handle = 0xFFFF;

/* Set to true when central subscribes to notifications */
static volatile bool g_subscribed = false;

/* Forward declaration */
static void ble_app_advertise(void);

/* ============================================================
 * CHARACTERISTIC ACCESS CALLBACK
 * ============================================================
 *
 * Called whenever the central device performs
 * a READ or WRITE on the characteristic.
 *
 * Add application logic here.
 */
static int char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  switch (ctxt->op) {
  case BLE_GATT_ACCESS_OP_READ_CHR:
    os_mbuf_append(ctxt->om, "DATA", 4);
    return 0;
  default:
    return BLE_ATT_ERR_UNLIKELY;
  }
}

/* ============================================================
 * GATT SERVICE DEFINITION
 * ============================================================ */

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &svc_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){{.uuid = &chr_uuid.u,
                                      .access_cb = char_access_cb,
                                      .flags = BLE_GATT_CHR_F_READ |
                                               BLE_GATT_CHR_F_NOTIFY,
                                      .val_handle = &char_handle},
                                     {0}}},
    {0}};

/* ============================================================
 * GAP EVENT HANDLER
 * ============================================================
 *
 * Handles connection and disconnection events.
 */
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {

  case BLE_GAP_EVENT_CONNECT:

    if (event->connect.status == 0) {
      g_conn_handle = event->connect.conn_handle;
      ESP_LOGI(TAG, "Central connected (handle %d)", g_conn_handle);
    } else {
      ESP_LOGE(TAG, "Connection failed");
      ble_app_advertise();
    }
    return 0;

  case BLE_GAP_EVENT_DISCONNECT:

    ESP_LOGI(TAG, "Central disconnected");
    g_conn_handle = 0xFFFF;
    g_subscribed = false;

    /* Restart advertising so new devices can connect */
    ble_app_advertise();

    return 0;

  case BLE_GAP_EVENT_SUBSCRIBE:
    ESP_LOGI(TAG, "Subscribe event: cur_notify=%d",
             event->subscribe.cur_notify);
    g_subscribed = (event->subscribe.cur_notify != 0);
    return 0;

  default:
    return 0;
  }
}

/* ============================================================
 * ADVERTISING SETUP
 * ============================================================ */

static void ble_app_advertise(void) {
  struct ble_hs_adv_fields fields = {0};
  struct ble_gap_adv_params params = {0};

  /* Advertised device name */
  fields.name = (uint8_t *)DEVICE_NAME;
  fields.name_len = strlen(DEVICE_NAME);
  fields.name_is_complete = 1;

  /* Advertised 16-bit UUID */
  ble_uuid16_t uuid16 = BLE_UUID16_INIT(ADV_UUID16);
  fields.uuids16 = &uuid16;
  fields.num_uuids16 = 1;
  fields.uuids16_is_complete = 1;

  ble_gap_adv_set_fields(&fields);

  /* Advertising parameters */
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &params,
                             gap_event_cb, NULL);

  if (rc == 0) {
    ESP_LOGI(TAG, "Advertising started: %s", DEVICE_NAME);
  } else if (rc != BLE_HS_EALREADY) {
    ESP_LOGE(TAG, "Advertising failed; rc=%d", rc);
  }
}

/* ============================================================
 * BLE STACK READY CALLBACK
 * ============================================================ */

static void ble_on_sync(void) {
  uint8_t addr_type;

  /* Ensure device has a valid address */
  ble_hs_util_ensure_addr(0);
  ble_hs_id_infer_auto(0, &addr_type);

  ESP_LOGI(TAG, "BLE stack ready");

  /* Start advertising */
  ble_app_advertise();
}

/* ============================================================
 * BLE HOST TASK
 * ============================================================ */

static void ble_host_task(void *param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

/* ============================================================
 * NOTIFICATION TX TASK — sends 1000 packets to central
 * ============================================================ */

static void ble_notify_task(void *pvParameters) {
  /* Wait for central to connect and subscribe */
  while (!g_subscribed || g_conn_handle == 0xFFFF) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI(TAG, "Central subscribed — starting TX (1000 pkts)");

  uint16_t seq = 0;
  uint32_t fail_count = 0;

  while (seq < TEST_PACKET_COUNT) {
    vTaskDelay(pdMS_TO_TICKS(BLE_SEND_INTERVAL_MS));

    if (g_conn_handle == 0xFFFF || !g_subscribed) {
      ESP_LOGW(TAG, "Disconnected during TX at seq=%u, waiting...", seq);
      while (!g_subscribed || g_conn_handle == 0xFFFF) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
      ESP_LOGI(TAG, "Reconnected, resuming TX at seq=%u", seq);
    }

    uint8_t pkt[4] = {0xAA, 0x55, (uint8_t)(seq & 0xFF),
                       (uint8_t)(seq >> 8)};
    struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, sizeof(pkt));
    if (!om) {
      fail_count++;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    int rc = ble_gatts_notify_custom(g_conn_handle, char_handle, om);
    if (rc == 0) {
      seq++;
      fail_count = 0;
      if (seq % 100 == 0) {
        ESP_LOGI(TAG, "BLE TX: %u/%d", seq, TEST_PACKET_COUNT);
      }
    } else {
      fail_count++;
      if (fail_count % 50 == 1) {
        ESP_LOGW(TAG, "Notify fail (rc=%d, streak=%lu)", rc,
                 (unsigned long)fail_count);
      }
      vTaskDelay(pdMS_TO_TICKS(100)); /* back-off */
    }
  }

  ESP_LOGI(TAG, "BLE TX complete — %d notifications sent. Task stopping.",
           TEST_PACKET_COUNT);
  vTaskDelete(NULL);
}

/* ============================================================
 * PUBLIC INITIALIZATION FUNCTION
 * ============================================================
 *
 * Call this once from app_main() to start BLE.
 */
void ble_peripheral_init(void) {
  /* Initialize NimBLE stack */
  nimble_port_init();

  /* Initialize standard GAP and GATT services */
  ble_svc_gap_init();
  ble_svc_gatt_init();

  /* Register custom services */
  ble_gatts_count_cfg(gatt_svcs);
  ble_gatts_add_svcs(gatt_svcs);

  /* Set callback for when stack becomes ready */
  ble_hs_cfg.sync_cb = ble_on_sync;

  /* Start BLE host task */
  nimble_port_freertos_init(ble_host_task);

  /* Start notification TX task */
  xTaskCreate(ble_notify_task, "ble_ntf_tx", 4096, NULL, 5, NULL);

  ESP_LOGI(TAG, "BLE Peripheral initialized");
}