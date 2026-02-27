/*
 * ============================================================
 *  Node A — ESP32-S3: Master Hub
 * ============================================================
 *
 *  Responsibilities:
 *   - Wi-Fi Access Point (other nodes connect as STA)
 *   - BLE Central (scans & connects to Node B and Node C)
 *   - TCP Server (receives coex_packet_t from Node B)
 *   - Dashboard task (periodic coex statistics)
 *
 * ============================================================
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "ble_central.h"
#include "tcp_server.h"
#include "wifi_ap.h"

static const char *TAG = "NODE_A";

/* ================= COEX DASHBOARD ================= */

static void dashboard_task(void *pvParameters) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Fetch live Wi-Fi stats from TCP server */
    uint32_t wifi_rx = 0, wifi_tx = 0;
    tcp_server_get_stats(&wifi_rx, &wifi_tx);

    uint32_t ble_rx = 0, ble_tx = 0;
    ble_central_get_stats(&ble_rx, &ble_tx);

    ESP_LOGI(TAG, "============= COEX DASHBOARD (HUB) =============");
    ESP_LOGI(TAG, "  Wi-Fi:   TX=%-6lu  RX=%-6lu",
             (unsigned long)wifi_tx, (unsigned long)wifi_rx);
    ESP_LOGI(TAG, "  BLE:     TX=%-6lu  RX=%-6lu",
             (unsigned long)ble_tx, (unsigned long)ble_rx);
    ESP_LOGI(TAG, "=================================================");
  }
}

/* ================= APP MAIN ================= */

void app_main(void) {
  ESP_LOGI(TAG, "=== NODE A (ESP32-S3) — Master Hub ===");

  /* Initialize NVS (required for Wi-Fi and BLE) */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* 1. Start Wi-Fi Access Point */
  wifi_ap_init();

  /* 2. Start BLE Central (scanner + connector) */
  ble_central_init();

  /* 3. Start TCP Server (receives data from Node B) */
  tcp_server_start();

  /* 4. Periodic coex dashboard */
  xTaskCreate(dashboard_task, "dashboard", 4096, NULL, 3, NULL);

  ESP_LOGI(TAG, "All subsystems initialized");
}
