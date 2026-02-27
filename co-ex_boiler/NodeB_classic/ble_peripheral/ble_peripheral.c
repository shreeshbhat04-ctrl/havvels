/*
 * ============================================================
 *  BLE Peripheral — Starter Boilerplate (NimBLE, ESP-IDF)
 * ============================================================
 *
 *  What this code does:
 *   ✔ Initializes BLE stack
 *   ✔ Creates a custom GATT service
 *   ✔ Advertises device name + UUID
 *   ✔ Accepts connections from a central device
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

/* ================= CONFIG ================= */

#define TAG "BLE_PERIPHERAL"
#define DEVICE_NAME "NODE-B(classic)"
#define ADV_UUID16 0xABCD // Advertised 16-bit UUID

/* 128-bit Service & Characteristic UUIDs */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t chr_uuid =
    BLE_UUID128_INIT(0xac, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* Characteristic handle (for notifications) */
static uint16_t char_handle;

/* Connection handle for sending notifications back */
static uint16_t g_conn_handle = 0xFFFF;

/* Forward declaration */
static void ble_app_advertise(void);

/* ============================================================
 * GATT ACCESS CALLBACK
 * ============================================================ */

/*
 * Handles read/write operations on the characteristic
 */
static int char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  switch (ctxt->op) {

  case BLE_GATT_ACCESS_OP_READ_CHR:
    /*
     * Example: return static data
     */
    os_mbuf_append(ctxt->om, "HELLO", 5);
    return 0;

  case BLE_GATT_ACCESS_OP_WRITE_CHR: {
    ESP_LOGI(TAG, "Write received (%d bytes)", OS_MBUF_PKTLEN(ctxt->om));

    /* Echo back as notification so central can count RX */
    if (g_conn_handle != 0xFFFF && char_handle != 0) {
      struct os_mbuf *om = ble_hs_mbuf_from_flat(ctxt->om->om_data,
                                                  OS_MBUF_PKTLEN(ctxt->om));
      if (om) {
        ble_gatts_notify_custom(g_conn_handle, char_handle, om);
      }
    }
    return 0;
  }

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
                                               BLE_GATT_CHR_F_WRITE |
                                               BLE_GATT_CHR_F_NOTIFY,
                                      .val_handle = &char_handle},
                                     {0}}},
    {0}};

/* ============================================================
 * GAP EVENTS
 * ============================================================ */

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
    ble_app_advertise();
    return 0;

  case BLE_GAP_EVENT_SUBSCRIBE:
    ESP_LOGI(TAG, "Subscribe event: cur_notify=%d",
             event->subscribe.cur_notify);
    return 0;

  default:
    return 0;
  }
}

/* ============================================================
 * ADVERTISING
 * ============================================================ */

static void ble_app_advertise(void) {
  struct ble_hs_adv_fields fields = {0};
  struct ble_gap_adv_params params = {0};

  /* Device name */
  fields.name = (uint8_t *)DEVICE_NAME;
  fields.name_len = strlen(DEVICE_NAME);
  fields.name_is_complete = 1;

  /* Advertise a 16-bit UUID */
  ble_uuid16_t uuid16 = BLE_UUID16_INIT(ADV_UUID16);
  fields.uuids16 = &uuid16;
  fields.num_uuids16 = 1;
  fields.uuids16_is_complete = 1;

  ble_gap_adv_set_fields(&fields);

  /* Advertising parameters */
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &params,
                    gap_event_cb, NULL);

  ESP_LOGI(TAG, "Advertising as: %s", DEVICE_NAME);
}

/* ============================================================
 * BLE READY CALLBACK
 * ============================================================ */

static void ble_on_sync(void) {
  uint8_t addr_type;

  ble_hs_util_ensure_addr(0);
  ble_hs_id_infer_auto(0, &addr_type);

  ESP_LOGI(TAG, "BLE stack ready");

  ble_app_advertise();
}

/* ============================================================
 * HOST TASK
 * ============================================================ */

static void ble_host_task(void *param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

/* ============================================================
 * PUBLIC INITIALIZATION FUNCTION
 * ============================================================ */

void ble_peripheral_init(void) {
  /* Initialize NimBLE */
  nimble_port_init();

  ble_svc_gap_init();
  ble_svc_gatt_init();

  /* Register GATT services */
  ble_gatts_count_cfg(gatt_svcs);
  ble_gatts_add_svcs(gatt_svcs);

  /* Set sync callback */
  ble_hs_cfg.sync_cb = ble_on_sync;

  /* Start BLE host task */
  nimble_port_freertos_init(ble_host_task);

  ESP_LOGI(TAG, "BLE Peripheral initialized");
}