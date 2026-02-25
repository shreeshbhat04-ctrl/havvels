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

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
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
#include "driver/uart.h"
#include "esp_ieee802154.h"

#endif

#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#endif

static const char *TAG = "COEX_TEST";

#define WIFI_SSID "QRG-RnD-5G"
#define WIFI_PASS "havells!@#$%"
#define SERVER_IP "10.220.4.127" // Your laptop Wi-Fi IP (same network as ESP32)
#define SERVER_PORT 5000

/* ============ Channel Planning Config ============ */
// Wi-Fi channel is decided by the AP, not us (WIFI_CHANNEL is just a scan hint)
// AP uses CH1 (2412 MHz, 2401-2423 MHz)
// Zigbee CH13 (2415 MHz) = MAXIMUM overlap with Wi-Fi CH1
#define WIFI_CHANNEL 1
#define ZIGBEE_CHANNEL 13

/* ============ Interference Duty Cycle Config ============ */
// Minimum 10ms to guarantee at least 1 FreeRTOS tick (100Hz default)
// Lower = more aggressive interference, but must be >= 10
#define INTERFERENCE_DELAY_MS 10

/* ============ Shared State ============ */
static TaskHandle_t wifi_task_handle = NULL;
static volatile bool wifi_connected = false;

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
static volatile int active_sock = -1;
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

/* =================================================================
 *  Wi-Fi TCP Task — shared by ESP32-S3 (WIFI) and ESP32 (BLE)
 * ================================================================= */
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)

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

    active_sock = sock;
    ESP_LOGI(TAG, "Successfully connected");

    while (wifi_connected) {
      coex_packet_t pkt;
      pkt.header = 0xAA55;
      pkt.seq = seq++;
      pkt.tx_timestamp = esp_timer_get_time();

#ifdef CONFIG_IDF_TARGET_ESP32S3
      pkt.mode = MODE_WIFI;
      // Get AP info for RSSI and channel
      wifi_ap_record_t ap;
      esp_wifi_sta_get_ap_info(&ap);
      pkt.rssi = ap.rssi;
      pkt.lqi = 0;
      pkt.tx_power = 20;
      pkt.channel = ap.primary;
#elif defined(CONFIG_IDF_TARGET_ESP32)
      pkt.mode = MODE_BLE;
      // Get Wi-Fi RSSI (BLE doesn't report RSSI for advertising)
      wifi_ap_record_t ap;
      esp_wifi_sta_get_ap_info(&ap);
      pkt.rssi = ap.rssi;
      pkt.lqi = 0;
      pkt.tx_power = 9; // BLE max TX power +9 dBm
      pkt.channel = 0;  // BLE hops across 37 data channels
#endif

      pkt.payload_len = sprintf((char *)pkt.payload, "DATA_%d", pkt.seq);
      pkt.crc = crc16((uint8_t *)&pkt, sizeof(coex_packet_t) - 2);

      int err = send(sock, &pkt, sizeof(coex_packet_t), 0);
      if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        break;
      }

      if (esp_timer_get_time() - last_log > 1000000) {
        ESP_LOGI(TAG, "Sent seq %d, RSSI %d, mode %d", pkt.seq, pkt.rssi,
                 pkt.mode);
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

  ESP_LOGI(TAG, "Wi-Fi TCP task exiting");
  wifi_task_handle = NULL;
  vTaskDelete(NULL);
}

/* ================= Wi-Fi Event Handler (S3 + ESP32) ================= */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_connected = false;

    if (active_sock != -1) {
      shutdown(active_sock, 0);
      close(active_sock);
      active_sock = -1;
    }

    if (wifi_task_handle != NULL) {
      vTaskDelay(pdMS_TO_TICKS(100));
      if (wifi_task_handle != NULL) {
        vTaskDelete(wifi_task_handle);
        wifi_task_handle = NULL;
      }
    }

    esp_wifi_connect();
    ESP_LOGI(TAG, "Retrying Wi-Fi connection...");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    wifi_connected = true;

    if (wifi_task_handle == NULL) {
      xTaskCreate(wifi_tcp_task, "wifi_tcp_task", 4096, NULL, 5,
                  &wifi_task_handle);
    }
  }
}

static void wifi_init_sta(void) {
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
              .channel = WIFI_CHANNEL,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Wi-Fi STA init done, waiting for connection...");
}

#endif /* CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32 */

/* =================================================================
 *  802.15.4 Transmitter (H2) — sends coex_packet_t via USB Serial
 * ================================================================= */
#ifdef CONFIG_IDF_TARGET_ESP32H2

#define UART_PORT_NUM UART_NUM_0
#define UART_BAUD_RATE 115200

void ieee802154_task(void *pvParameters) {
  ESP_LOGI(TAG, "Starting 802.15.4 Task (CH=%d, delay=%dms)", ZIGBEE_CHANNEL,
           INTERFERENCE_DELAY_MS);
  esp_ieee802154_enable();
  esp_ieee802154_set_channel(ZIGBEE_CHANNEL);
  esp_ieee802154_set_txpower(20);

  uint8_t frame[] = {
      0x01, 0x08,            // FCF
      0x01,                  // Sequence
      0xFF, 0xFF,            // PAN
      0xFF, 0xFF,            // DestAddr
      0xAA, 0xBB, 0xCC, 0xDD // Payload
  };

  uint16_t seq = 0;

  while (1) {
    // Transmit 802.15.4 interference frame
    frame[2] = (uint8_t)(seq & 0xFF); // Update sequence in frame
    esp_ieee802154_transmit(frame, false);

    // Build coex_packet_t and send over USB serial
    coex_packet_t pkt;
    pkt.header = 0xAA55;
    pkt.seq = seq++;
    pkt.tx_timestamp = esp_timer_get_time();
    pkt.mode = MODE_ZIGBEE;
    pkt.rssi = 0; // TX side, no RX RSSI
    pkt.lqi = 0;
    pkt.tx_power = 20;
    pkt.channel = ZIGBEE_CHANNEL;
    pkt.payload_len = sprintf((char *)pkt.payload, "ZB_%d", pkt.seq);
    pkt.crc = crc16((uint8_t *)&pkt, sizeof(coex_packet_t) - 2);

    // Write raw struct to UART0 (USB serial → COM10 on laptop)
    uart_write_bytes(UART_PORT_NUM, (const char *)&pkt, sizeof(coex_packet_t));

    vTaskDelay(pdMS_TO_TICKS(INTERFERENCE_DELAY_MS));
  }
}

#endif

/* =================================================================
 *  BLE Advertising (ESP32 Classic) — runs alongside Wi-Fi
 * ================================================================= */
#ifdef CONFIG_IDF_TARGET_ESP32

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x20, // 20ms
    .adv_int_max = 0x20,
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
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
  ESP_LOGI(TAG, "Initializing BLE...");

  // Release classic BT memory (we only need BLE)
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());

  ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  ESP_LOGI(TAG, "BLE TX power set to max (+9 dBm)");

  uint8_t adv_raw_data[] = {0x02, 0x01, 0x06, 0x0B, 0xFF, 0xFF, 0xFF, 0xAA,
                            0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
  ESP_ERROR_CHECK(
      esp_ble_gap_config_adv_data_raw(adv_raw_data, sizeof(adv_raw_data)));

  ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
  ESP_LOGI(TAG, "BLE interference advertising started");
}

#endif

/* =================================================================
 *  app_main — entry point for all targets
 * ================================================================= */
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
  /* --- S3: Wi-Fi STA → TCP (mode=WIFI) --- */
  wifi_init_sta();
#endif

#ifdef CONFIG_IDF_TARGET_ESP32H2
  /* --- H2: Zigbee TX → USB serial (mode=ZIGBEE) --- */
  xTaskCreate(ieee802154_task, "ieee802154_task", 4096, NULL, 5, NULL);
#endif

#ifdef CONFIG_IDF_TARGET_ESP32
  /* --- ESP32: Wi-Fi STA + BLE → TCP (mode=BLE) --- */
  wifi_init_sta();      // Start Wi-Fi first
  ble_init_and_start(); // Then start BLE advertising
#endif
}
