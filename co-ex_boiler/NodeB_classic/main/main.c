/*
 * ============================================================
 *  Node B — ESP32 Classic: Spoke
 * ============================================================
 *
 *  Responsibilities:
 *   ✔ Wi-Fi STA (connects to Node A's AP)
 *   ✔ BLE Peripheral (advertises for Node A to scan)
 *   ✔ TCP Client (sends coex_packet_t to Node A)
 *
 * ============================================================
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "ble_peripheral.h"
#include "tcp_client.h"
#include "wifi_sta.h"


static const char *TAG = "NODE_B";

/* ================= APP MAIN ================= */

void app_main(void) {
  ESP_LOGI(TAG, "=== NODE B (ESP32) — Spoke ===");

  /* Initialize NVS (required for Wi-Fi and BLE) */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* 1. Start Wi-Fi STA (connect to Node A's AP) */
  wifi_sta_init();

  /* 2. Start BLE Peripheral (advertise for Node A) */
  ble_peripheral_init();

  /* 3. Start TCP Client (send coex packets to Node A) */
  tcp_client_start();

  ESP_LOGI(TAG, "All subsystems initialized");
}
