/*
 * ============================================================
 *  Zigbee End Device — Boilerplate (ESP Zigbee, ESP-IDF)
 * ============================================================
 *
 *  Node C (ESP32-H2)
 *
 *  What this code does:
 *   ✔ Initializes Zigbee as End Device
 *   ✔ Joins an existing network (formed by Node D Coordinator)
 *   ✔ Provides hooks for receiving data
 *   ✔ Ready for custom application logic
 *
 *  Replace sections marked:
 *      // ===== ADD YOUR CODE HERE =====
 *
 * ============================================================
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_log.h"

#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_command.h"

/* ================= CONFIG ================= */

#define TAG "ZB_END_DEVICE"
#define ENDPOINT 1
#define ZIGBEE_CHANNEL 14

#define CUSTOM_CLUSTER_ID 0xFF00
#define CUSTOM_ATTR_ID 0xFFF0
#define DEVICE_ID_CUSTOM 0xFFF0

#define TEST_PKT_COUNT 1000

/* Semaphore signalled when the single Zigbee test batch completes */
static SemaphoreHandle_t s_batch_done_sem = NULL;

/* Buffer for custom attribute */
static uint8_t g_custom_attr_val[128] = {0};

/* Helper: send one 1000-packet batch and return */
static void send_zb_batch(uint32_t batch_num) {
  uint16_t seq = 0;
  ESP_LOGI(TAG, "ZB Batch %lu: sending %d pkts @ 10 pkt/s",
           (unsigned long)batch_num, TEST_PKT_COUNT);

  while (seq < TEST_PKT_COUNT) {
    vTaskDelay(pdMS_TO_TICKS(100)); /* 10 packets/sec */

    /* Encode: [len_byte][seq_lo][seq_hi] */
    g_custom_attr_val[0] = 2;
    g_custom_attr_val[1] = (uint8_t)(seq & 0xFF);
    g_custom_attr_val[2] = (uint8_t)((seq >> 8) & 0xFF);

    esp_zb_zcl_set_attribute_val(ENDPOINT, CUSTOM_CLUSTER_ID,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, CUSTOM_ATTR_ID,
                                 g_custom_attr_val, false);

    esp_zb_zcl_report_attr_cmd_t report_cmd;
    memset(&report_cmd, 0, sizeof(report_cmd));
    report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    report_cmd.zcl_basic_cmd.dst_endpoint = 1;
    report_cmd.zcl_basic_cmd.src_endpoint = ENDPOINT;
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report_cmd.clusterID = CUSTOM_CLUSTER_ID;
    report_cmd.attributeID = CUSTOM_ATTR_ID;
    report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    esp_zb_zcl_report_attr_cmd_req(&report_cmd);

    seq++;
    if (seq % 100 == 0) {
      ESP_LOGI(TAG, "ZB Batch %lu: %u / %d",
               (unsigned long)batch_num, seq, TEST_PKT_COUNT);
    }
  }

  ESP_LOGI(TAG, "ZB Batch %lu complete (%d pkts)",
           (unsigned long)batch_num, TEST_PKT_COUNT);
}

static void zb_tx_task(void *pvParameters) {

  /* Send exactly 1000 Zigbee packets then stop */
  send_zb_batch(1);

  /* Signal main() to start BLE peripheral */
  ESP_LOGI(TAG, "Zigbee test done — 1000 packets sent. Waking BLE phase.");
  if (s_batch_done_sem) {
    xSemaphoreGive(s_batch_done_sem);
  }

  vTaskDelete(NULL);
}

/* ============================================================
 * ATTRIBUTE RECEIVE HANDLER
 * ============================================================ */

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message) {
  if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {

    esp_zb_zcl_set_attr_value_message_t *msg =
        (esp_zb_zcl_set_attr_value_message_t *)message;

    if (msg->info.cluster == CUSTOM_CLUSTER_ID &&
        msg->attribute.id == CUSTOM_ATTR_ID) {

      /* ===== ADD YOUR CODE HERE =====
         This block executes when data is received
         from Coordinator (Node D).

         Example:
         - Parse payload
         - Control hardware
         - Update variables
         - Trigger events
      */

      ESP_LOGI(TAG, "Custom attribute received");
    }
  }

  return ESP_OK;
}

/* ============================================================
 * COMMISSIONING CALLBACK
 * ============================================================ */

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

/* ============================================================
 * ZIGBEE SIGNAL HANDLER
 * ============================================================ */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *p_sg_p;

  switch (sig_type) {

  case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:

    /* Start initialization */
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
    break;

  case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:

    if (err_status == ESP_OK) {
      /* Begin network join */
      ESP_LOGI(TAG, "Steering to join network...");
      esp_zb_bdb_start_top_level_commissioning(
          ESP_ZB_BDB_MODE_NETWORK_STEERING);
    } else {
      /* Stale network info or init failure — force re-steering */
      ESP_LOGW(TAG, "Init/reboot failed (0x%x), re-steering...", err_status);
      esp_zb_bdb_start_top_level_commissioning(
          ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
    break;

  case ESP_ZB_BDB_SIGNAL_STEERING:

    if (err_status == ESP_OK) {

      ESP_LOGI(TAG, "Joined network successfully!");

      /* Start periodic transmissions */
      xTaskCreate(zb_tx_task, "zb_tx", 4096, NULL, 5, NULL);

    } else {

      /* Retry join */
      ESP_LOGW(TAG, "Join failed, retrying...");
      esp_zb_scheduler_alarm(
          (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
          ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
    }
    break;

  case ESP_ZB_ZDO_SIGNAL_LEAVE:

    /* Rejoin network automatically */
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    break;

  default:
    break;
  }
}

/* ============================================================
 * ZIGBEE MAIN TASK
 * ============================================================ */

static void zigbee_task(void *pvParameters) {
  /* End Device configuration */
  esp_zb_cfg_t zb_nwk_cfg = {
      .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
      .install_code_policy = false,
      .nwk_cfg.zed_cfg = {.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
                          .keep_alive = 3000},
  };

  esp_zb_init(&zb_nwk_cfg);

  /* ===== BASIC CLUSTER ===== */

  esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);

  esp_zb_basic_cluster_add_attr(
      basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "HAVVELS-C");

  /* ===== CUSTOM CLUSTER ===== */

  esp_zb_attribute_list_t *custom_attr =
      esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);

  esp_zb_custom_cluster_add_custom_attr(
      custom_attr, CUSTOM_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      g_custom_attr_val);

  /* ===== CLUSTER LIST ===== */

  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

  esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster,
                                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_attr,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  /* ===== ENDPOINT ===== */

  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

  esp_zb_endpoint_config_t ep_cfg = {.endpoint = ENDPOINT,
                                     .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                                     .app_device_id = DEVICE_ID_CUSTOM,
                                     .app_device_version = 0};

  esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg);

  esp_zb_device_register(ep_list);

  /* Register receive handler */
  esp_zb_core_action_handler_register(zb_action_handler);

  /* Channel selection */
  esp_zb_set_primary_network_channel_set(1 << ZIGBEE_CHANNEL);

  /* Erase stale network info so we always do a fresh join.
     This is critical when the coordinator (NodeD) was re-flashed
     and formed a new network — old credentials are invalid. */
  esp_zb_nvram_erase_at_start(true);

  /* Start Zigbee stack */
  ESP_ERROR_CHECK(esp_zb_start(false));

  /* Main Zigbee loop (must not return) */
  esp_zb_stack_main_loop();
}

/* ============================================================
 * PUBLIC INITIALIZATION FUNCTION
 * ============================================================ */

void zigbee_zed_init(void) {
  /* Create semaphore before Zigbee task starts */
  s_batch_done_sem = xSemaphoreCreateBinary();

  /* Zigbee platform configuration */
  esp_zb_platform_config_t config = {
      .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
      .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE}};

  esp_zb_platform_config(&config);

  /* Start Zigbee task */
  xTaskCreate(zigbee_task, "zigbee_main", 4096, NULL, 5, NULL);

  ESP_LOGI(TAG, "Zigbee End Device initialized");
}

/**
 * Block until the single 1000-packet Zigbee test batch is done.
 * Call this from main before starting BLE.
 */
void zigbee_zed_wait_batch_done(void) {
  if (s_batch_done_sem) {
    ESP_LOGI(TAG, "Waiting for Zigbee batch to finish...");
    xSemaphoreTake(s_batch_done_sem, portMAX_DELAY);
    ESP_LOGI(TAG, "Zigbee batch done.");
  }
}