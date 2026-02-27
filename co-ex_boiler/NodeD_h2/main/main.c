/*
 * ============================================================
 *  Node D — ESP32-H2: Zigbee Coordinator
 * ============================================================
 *
 *  Responsibilities:
 *   ✔ Zigbee Coordinator (forms network, Node C joins)
 *   ✔ Prints Zigbee session analysis locally (no UART bridge)
 *
 * ============================================================
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_command.h"

#if !defined ZB_COORDINATOR_ROLE
#error Define ZB_COORDINATOR_ROLE in idf.py menuconfig
#endif

/* ================= CONFIG ================= */

#define TAG "ZB_COORDINATOR"
#define ENDPOINT 1
#define ZIGBEE_CHANNEL 14

#define CUSTOM_CLUSTER_ID 0xFF00
#define CUSTOM_ATTR_ID 0xFFF0
#define DEVICE_ID_CUSTOM 0xFFF0
#define TEST_PACKET_COUNT 1000

/* ============================================================
 * SESSION TRACKING
 * ============================================================ */

typedef struct {
  uint32_t rx_count;
  uint32_t lost_count;
  uint32_t target_count;
  uint16_t last_seq;
  int64_t  start_time_us;
  int64_t  end_time_us;
  int64_t  last_arrival_us;
  int64_t  jitter_sum_us;
  int32_t  rssi_sum;
  int8_t   rssi_min;
  int8_t   rssi_max;
  bool     complete;
} zb_session_t;

static zb_session_t g_session;
static volatile uint32_t g_total_rx = 0;

static void session_reset(zb_session_t *s) {
  memset(s, 0, sizeof(*s));
  s->target_count = TEST_PACKET_COUNT;
  s->rssi_min = 0;
  s->rssi_max = -127;
  s->complete = false;
}

static void session_print_report(zb_session_t *s) {
  double duration = (s->end_time_us - s->start_time_us) / 1000000.0;
  double pdr = s->target_count > 0
                   ? 100.0 * s->rx_count / s->target_count
                   : 0.0;
  double avg_jitter =
      s->rx_count > 1 ? (double)s->jitter_sum_us / (s->rx_count - 1) : 0.0;
  double avg_rssi =
      s->rx_count > 0 ? (double)s->rssi_sum / s->rx_count : 0.0;

  ESP_LOGI(TAG, "+------------------------------------+");
  ESP_LOGI(TAG, "|  ZIGBEE SESSION REPORT (COORD)     |");
  ESP_LOGI(TAG, "+------------------------------------+");
  ESP_LOGI(TAG, "|  RX:         %lu / %lu",
           (unsigned long)s->rx_count, (unsigned long)s->target_count);
  ESP_LOGI(TAG, "|  Lost:       %lu", (unsigned long)s->lost_count);
  ESP_LOGI(TAG, "|  PDR:        %.1f%%", pdr);
  ESP_LOGI(TAG, "|  Duration:   %.2f s", duration);
  ESP_LOGI(TAG, "|  Avg Jitter: %.1f us", avg_jitter);
  ESP_LOGI(TAG, "|  RSSI:       min=%d max=%d avg=%.1f dBm",
           s->rssi_min, s->rssi_max, avg_rssi);
  ESP_LOGI(TAG, "+------------------------------------+");
}

/* ============================================================
 * GLOBALS
 * ============================================================ */

/* Stores last joined device short address */
static uint16_t g_target_short_addr = 0xFFFF;

/* Buffer for custom attribute */
static uint8_t g_custom_attr_val[128] = {0};

/* ============================================================
 * DASHBOARD TASK
 * ============================================================ */

static void dashboard_task(void *pvParameters) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "--- ZB Coordinator Dashboard ---");
    ESP_LOGI(TAG, "  ZB Total RX: %lu", (unsigned long)g_total_rx);
    ESP_LOGI(TAG, "  Session RX:  %lu / %lu",
             (unsigned long)g_session.rx_count,
             (unsigned long)g_session.target_count);
    ESP_LOGI(TAG, "  Device:      0x%04X", g_target_short_addr);
  }
}
/* ============================================================
 * ZIGBEE ACTION HANDLER
 * ============================================================ */

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message) {
  esp_err_t ret = ESP_OK;

  if (callback_id == ESP_ZB_CORE_REPORT_ATTR_CB_ID) {
    esp_zb_zcl_report_attr_message_t *msg =
        (esp_zb_zcl_report_attr_message_t *)message;

    if (msg->cluster == CUSTOM_CLUSTER_ID) {
      /* Extract seq from attribute data: [len][seq_lo][seq_hi] */
      uint16_t zb_seq = 0;
      if (msg->attribute.data.value && msg->attribute.data.size >= 3) {
        uint8_t *d = (uint8_t *)msg->attribute.data.value;
        zb_seq = d[1] | (d[2] << 8);
      }

      g_total_rx++;
      int64_t now = esp_timer_get_time();

      ESP_LOGI(TAG, "ZB RX: addr=0x%04X seq=%u total=%lu",
               msg->src_address.u.short_addr, zb_seq,
               (unsigned long)g_total_rx);

      /* Session tracking */
      if (g_session.rx_count == 0 && g_session.start_time_us == 0) {
        session_reset(&g_session);
        g_session.start_time_us = now;
      }

      /* Jitter */
      if (g_session.last_arrival_us > 0) {
        int64_t delta = now - g_session.last_arrival_us;
        if (delta < 0) delta = -delta;
        g_session.jitter_sum_us += delta;
      }
      g_session.last_arrival_us = now;

      /* Loss detection */
      if (g_session.rx_count > 0) {
        int gap = (int)zb_seq - (int)g_session.last_seq - 1;
        if (gap > 0)
          g_session.lost_count += gap;
      }
      g_session.last_seq = zb_seq;

      g_session.rx_count++;

      /* Session complete? */
      if (g_session.rx_count >= g_session.target_count) {
        g_session.end_time_us = now;
        g_session.complete = true;
        session_print_report(&g_session);
        session_reset(&g_session);
      }
    }
  }
  return ret;
}

/* ============================================================
 * ZIGBEE SIGNAL HANDLER
 * ============================================================ */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal) {
  uint32_t *p_sg = signal->p_app_signal;
  esp_err_t err_status = signal->esp_err_status;
  esp_zb_app_signal_type_t type = *p_sg;

  switch (type) {

  case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:

    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
    break;

  case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:

    if (err_status == ESP_OK) {

      ESP_LOGI(TAG, "Network forming...");
      esp_zb_bdb_start_top_level_commissioning(
          ESP_ZB_BDB_MODE_NETWORK_FORMATION);
    }
    break;

  case ESP_ZB_BDB_SIGNAL_FORMATION:

    if (err_status == ESP_OK) {

      ESP_LOGI(TAG, "Network formed, opening for joining");

      /* Open network for devices to join (180s) */
      esp_zb_bdb_start_top_level_commissioning(
          ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
    break;

  case ESP_ZB_BDB_SIGNAL_STEERING:

    if (err_status == ESP_OK) {
      ESP_LOGI(TAG, "Network open for joining");
    }
    break;

  case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {

    /* Device joined network */
    esp_zb_zdo_signal_device_annce_params_t *dev =
        (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(
            p_sg);

    g_target_short_addr = dev->device_short_addr;

    ESP_LOGI(TAG, "Device joined: 0x%04X", g_target_short_addr);

    /* ===== ADD YOUR CODE HERE =====
       Example:
       - Store device info
       - Start communication
       - Perform binding
    */

    break;
  }

  case ESP_ZB_ZDO_SIGNAL_LEAVE:

    ESP_LOGI(TAG, "Device left network");
    g_target_short_addr = 0xFFFF;
    break;

  default:
    ESP_LOGD(TAG, "Unhandled Zigbee signal: 0x%X (status=%d)", type,
             err_status);
    break;
  }
}

/* ============================================================
 * ZIGBEE MAIN TASK
 * ============================================================ */

static void zigbee_task(void *pvParameters) {
  /* Coordinator configuration */
  esp_zb_cfg_t cfg = {
      .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
      .install_code_policy = false,
      .nwk_cfg.zczr_cfg = {.max_children = 10},
  };

  esp_zb_init(&cfg);

  /* ===== BASIC CLUSTER ===== */

  esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);

  esp_zb_basic_cluster_add_attr(
      basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "HAVVELS-D");

  /* ===== CUSTOM CLUSTER ===== */

  esp_zb_attribute_list_t *custom_attr =
      esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);

  esp_zb_custom_cluster_add_custom_attr(
      custom_attr, CUSTOM_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, g_custom_attr_val);

  /* ===== CLUSTER LIST ===== */

  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

  esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster,
                                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_attr,
                                         ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

  /* ===== ENDPOINT ===== */

  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

  esp_zb_endpoint_config_t ep_cfg = {.endpoint = ENDPOINT,
                                     .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                                     .app_device_id = DEVICE_ID_CUSTOM,
                                     .app_device_version = 0};

  esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg);

  esp_zb_device_register(ep_list);

  /* Channel selection */
  esp_zb_set_primary_network_channel_set(1 << ZIGBEE_CHANNEL);

  /* Register action handler for attribute reports */
  esp_zb_core_action_handler_register(zb_action_handler);

  /* Start Zigbee stack */
  ESP_ERROR_CHECK(esp_zb_start(false));

  /* Start dashboard task */
  xTaskCreate(dashboard_task, "dashboard", 4096, NULL, 3, NULL);

  /* Zigbee main loop (must not return) */
  esp_zb_stack_main_loop();
}

/* ============================================================
 * APPLICATION ENTRY
 * ============================================================ */

void app_main(void) {
  ESP_LOGI(TAG, "=== NODE D (ESP32-H2) — Zigbee Coordinator ===");

  /* Initialize NVS */
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  /* Initialize session tracking */
  session_reset(&g_session);

  /* Zigbee platform configuration */
  esp_zb_platform_config_t config = {
      .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
      .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE}};

  esp_zb_platform_config(&config);

  /* Start Zigbee task */
  xTaskCreate(zigbee_task, "zigbee_main", 4096, NULL, 5, NULL);
}