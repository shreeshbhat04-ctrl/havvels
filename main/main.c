#include "sdkconfig.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "esp_wifi.h"
#include "lwip/sockets.h"
#endif

#else
#include <time.h>
#define esp_timer_get_time() ((int64_t)0)
#endif

#include "coex_packet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_IDF_TARGET_ESP32H2
#include "esp_ieee802154.h"
#endif
#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#endif

static const char *TAG = "COEX_TEST";

#define WIFI_SSID "Sridhar-Maruthi"
#define WIFI_PASS "maruthi1979"
#define SERVER_IP "192.168.0.108" // Your laptop IP
#define SERVER_PORT 5000

/* ============ Channel Planning Config ============ */
// Wi-Fi: 1, 6, or 11  |  Zigbee 802.15.4: 15, 20, or 25
// Channel 1 (Wi-Fi) and Channel 25 (Zigbee) = ZERO spectral overlap
#define WIFI_CHANNEL 1
#define ZIGBEE_CHANNEL 25

/* ============ Interference Duty Cycle Config ============ */
// Adjustable: 1 = aggressive (near-continuous), 100 = light interference
#define INTERFERENCE_DELAY_MS 1

static TaskHandle_t wifi_task_handle = NULL;
static volatile bool wifi_connected = false;

#ifdef CONFIG_IDF_TARGET_ESP32S3
static volatile int active_sock = -1; // Global socket ref for graceful shutdown
#endif

/* ================= Target Detection ================= */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define DEVICE_MODE "S3_WIFI"
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#define DEVICE_MODE "H2_ZIGBEE"
#elif defined(CONFIG_IDF_TARGET_ESP32)
#define DEVICE_MODE "ESP32_BLE"
#else
#define DEVICE_MODE "UNKNOWN"
#endif

/* ================= Wi-Fi Client (S3) ================= */
#ifdef CONFIG_IDF_TARGET_ESP32S3
void wifi_tcp_task(void *pvParameters) {
  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(SERVER_PORT);

  int64_t last_log = 0;
  uint16_t seq = 0;

  while (wifi_connected) {
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    ESP_LOGI(TAG, "Connecting to %s:%d", SERVER_IP, SERVER_PORT);
    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
      ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
      close(sock);
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    active_sock = sock; // Store for graceful shutdown
    ESP_LOGI(TAG, "Successfully connected");

    while (wifi_connected) {
      coex_packet_t pkt;
      pkt.header = 0xAA55;
      pkt.seq = seq++;
      pkt.tx_timestamp = esp_timer_get_time();
      pkt.mode = MODE_WIFI;

      // Get AP info for RSSI and channel
      wifi_ap_record_t ap;
      esp_wifi_sta_get_ap_info(&ap);
      pkt.rssi = ap.rssi;
      pkt.lqi = 0;              // WiFi doesn't use LQI
      pkt.tx_power = 20;        // ESP32-S3 default TX power
      pkt.channel = ap.primary; // Current Wi-Fi channel

      pkt.payload_len = sprintf((char *)pkt.payload, "DATA_%d", pkt.seq);
      pkt.crc = crc16((uint8_t *)&pkt, sizeof(coex_packet_t) - 2);

      int err = send(sock, &pkt, sizeof(coex_packet_t), 0);
      if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        break;
      }

      if (esp_timer_get_time() - last_log > 1000000) {
        ESP_LOGI(TAG, "Sent seq %d, RSSI %d, CH %d", pkt.seq, pkt.rssi,
                 pkt.channel);
        last_log = esp_timer_get_time();
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Clean up socket
    if (sock != -1) {
      ESP_LOGI(TAG, "Shutting down socket...");
      shutdown(sock, 0);
      close(sock);
      active_sock = -1;
    }
  }

  // Task exiting cleanly
  ESP_LOGI(TAG, "Wi-Fi TCP task exiting");
  wifi_task_handle = NULL;
  vTaskDelete(NULL);
}
#endif

/* ================= 802.15.4 Transmitter (H2) ================= */
#ifdef CONFIG_IDF_TARGET_ESP32H2
void ieee802154_task(void *pvParameters) {
  ESP_LOGI(TAG, "Starting 802.15.4 Interference Task (CH=%d, delay=%dms)",
           ZIGBEE_CHANNEL, INTERFERENCE_DELAY_MS);
  esp_ieee802154_enable();
  esp_ieee802154_set_channel(ZIGBEE_CHANNEL);
  esp_ieee802154_set_txpower(20);

  uint8_t frame[] = {
      0x01, 0x08,            // FCF
      0x01,                  // Sequence
      0xFF, 0xFF,            // PAN
      0xFF, 0xFF,            // DestAddr
      0xAA, 0xBB, 0xCC, 0xDD // Payload (Interference)
  };

  while (1) {
    esp_ieee802154_transmit(frame, false);
    vTaskDelay(pdMS_TO_TICKS(INTERFERENCE_DELAY_MS));
  }
}
#endif

/* ================= Wi-Fi Init (S3 Only) ================= */
#ifdef CONFIG_IDF_TARGET_ESP32S3
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_connected = false;

    // Close the active socket so the task's send() unblocks and exits
    if (active_sock != -1) {
      shutdown(active_sock, 0);
      close(active_sock);
      active_sock = -1;
    }

    // Wait for task to exit gracefully (it checks wifi_connected flag)
    if (wifi_task_handle != NULL) {
      // Give the task a moment to notice the flag and clean up
      vTaskDelay(pdMS_TO_TICKS(100));
      // If still running, force delete as fallback
      if (wifi_task_handle != NULL) {
        vTaskDelete(wifi_task_handle);
        wifi_task_handle = NULL;
      }
    }

    esp_wifi_connect();
    ESP_LOGI(TAG, "retry to connect to the AP");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    wifi_connected = true;

    // Start socket task only after WiFi is connected
    if (wifi_task_handle == NULL) {
      xTaskCreate(wifi_tcp_task, "wifi_tcp_task", 4096, NULL, 5,
                  &wifi_task_handle);
    }
  }
}
#endif

/* ================= BLE Interference (ESP32) ================= */
#ifdef CONFIG_IDF_TARGET_ESP32

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x20,              // Min interval: 20ms (fastest allowed)
    .adv_int_max = 0x20,              // Max interval: 20ms
    .adv_type = ADV_TYPE_NONCONN_IND, // Non-connectable, just broadcast
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL, // Advertise on all 3 BLE adv channels
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  switch (event) {
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(TAG, "BLE advertising started successfully");
    } else {
      ESP_LOGE(TAG, "BLE advertising start failed, status %d",
               param->adv_start_cmpl.status);
    }
    break;
  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    ESP_LOGI(TAG, "BLE advertising stopped");
    break;
  default:
    break;
  }
}

static void ble_init_and_start(void) {
  ESP_LOGI(TAG, "Initializing BLE for interference...");

  // Release classic BT memory (we only need BLE)
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  // Initialize BT controller
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

  // Initialize Bluedroid stack
  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());

  // Register GAP callback
  ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

  // Set max TX power for maximum interference
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  ESP_LOGI(TAG, "BLE TX power set to max (+9 dBm)");

  // Configure advertising data (interference payload)
  uint8_t adv_raw_data[] = {
      0x02, 0x01, 0x06, // Flags: General Discoverable
      0x0B, 0xFF,       // Manufacturer Specific Data
      0xFF, 0xFF,       // Company ID (0xFFFF = test)
      0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22 // Interference payload
  };
  ESP_ERROR_CHECK(
      esp_ble_gap_config_adv_data_raw(adv_raw_data, sizeof(adv_raw_data)));

  // Start advertising
  ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
  ESP_LOGI(TAG, "BLE interference advertising started");
}

#endif

void app_main(void) {
  ESP_LOGI(TAG, "Running Device Mode: %s", DEVICE_MODE);

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

#ifdef CONFIG_IDF_TARGET_ESP32S3
  // Wi-Fi S3 side
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event_handler, NULL));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASS,
              .channel = WIFI_CHANNEL, // Pin to fixed channel for coexistence
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Waiting for WiFi on CH %d...", WIFI_CHANNEL);
#endif

#ifdef CONFIG_IDF_TARGET_ESP32H2
  // Zigbee/802.15.4 H2 side
  xTaskCreate(ieee802154_task, "ieee802154_task", 4096, NULL, 5, NULL);
#endif
#ifdef CONFIG_IDF_TARGET_ESP32
  // BLE ESP32 side — init and start advertising
  ble_init_and_start();
#endif
}
