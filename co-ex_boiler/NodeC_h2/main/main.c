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

  /*
   * Phase 1 — Zigbee-only (no BLE on this chip yet)
   *   NodeC joins NodeD's Zigbee network and sends a 1000-packet batch.
   *   Meanwhile NodeA+B are busy with their own WiFi+BLE test.
   *
   *   After batch 1 the semaphore fires, but the ZB task does NOT stop —
   *   it continues sending batch 2, 3, ... indefinitely in a loop.
   *   Zigbee runs all the time; the semaphore just unlocks BLE start.
   */
  ESP_LOGI(TAG, "Phase 1: Zigbee-only batch starting (A+B WiFi/BLE running in parallel)");
  zigbee_zed_init();
  zigbee_zed_wait_batch_done();
  ESP_LOGI(TAG, "Phase 1 done. Starting Phase 2: BLE + ZB coexistence.");

  /*
   * Phase 2 — Full coexistence stress test
   *
   *   WiFi (NodeA AP <-> NodeB STA) is always running — it is the
   *   constant baseline throughout both phases.
   *
   *   BLE alternates by phase:
   *     Phase 1 (above): NodeB BLE Peripheral <-> NodeA BLE Central
   *     Phase 2  (here): NodeC BLE Peripheral <-> NodeA BLE Central
   *   NodeA naturally connects to whichever peripheral is advertising;
   *   NodeC's BLE only starts now, so NodeA will pick it up in Phase 2.
   *
   *   Simultaneous radios in Phase 2 on this chip (H2):
   *     - 802.15.4 Zigbee (batch 2+) + BLE Peripheral
   *
   *   This is the intended coexistence measurement window.
   */
  ble_peripheral_init();

  ESP_LOGI(TAG, "Phase 2 active: ZB+BLE coex running");
}
