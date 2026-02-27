#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

/* ================= CONFIG ================= */

#define AP_SSID "ESP32_TEST_AP"
#define AP_PASS "12345678" // Minimum 8 chars
#define AP_CHANNEL 6
#define MAX_STA_CONN 4

static const char *TAG = "WIFI_AP";

/* ================= WIFI AP INIT ================= */

void wifi_ap_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t ap_config = {.ap = {.ssid = AP_SSID,
                                    .ssid_len = strlen(AP_SSID),
                                    .password = AP_PASS,
                                    .channel = AP_CHANNEL,
                                    .max_connection = MAX_STA_CONN,
                                    .authmode = WIFI_AUTH_WPA2_PSK}};

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Wi-Fi AP started");
  ESP_LOGI(TAG, "SSID: %s | Channel: %d | Max STA: %d", AP_SSID, AP_CHANNEL,
           MAX_STA_CONN);
}
