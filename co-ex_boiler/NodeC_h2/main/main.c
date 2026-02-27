/*
 * ============================================================
 *  Node C — ESP32-H2: Zigbee End Device + BLE Peripheral
 * ============================================================
 *
 *  Responsibilities:
 *   ✔ Zigbee End Device (joins Node D's coordinator network)
 *   ✔ BLE Peripheral (optional, advertises for Node A)
 *
 * ============================================================
 */

#include "esp_log.h"
#include "nvs_flash.h"

#include "ble_peripheral.h"
#include "zigbee_zed.h"

static const char *TAG = "NODE_C";

void app_main(void) {
  ESP_LOGI(TAG, "=== NODE C (ESP32-H2) — ZB End Device + BLE ===");

  /* Initialize NVS */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* 1. Phase 1: Zigbee-only — join network and send 1000-packet batch */
  ESP_LOGI(TAG, "Phase 1: Zigbee test starting...");
  zigbee_zed_init();

  /* Block here until all 1000 Zigbee packets are sent */
  zigbee_zed_wait_batch_done();
  ESP_LOGI(TAG, "Phase 1 complete. Starting Phase 2: BLE.");

  /* 2. Phase 2: BLE-only — advertise and connect to Node A */
  ble_peripheral_init();

  ESP_LOGI(TAG, "All subsystems initialized");
}
