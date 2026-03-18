#include "dll_metrics.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "DLL_METRICS";

static const char *dll_scenario_name(dll_scenario_t scenario) {
  switch (scenario) {
  case SCENARIO_HOME_IOT:
    return "HOME_IOT";
  case SCENARIO_OFFICE_HIGH_DENSITY:
    return "OFFICE_HIGH_DENSITY";
  default:
    return "UNKNOWN";
  }
}

void dll_metrics_reset(dll_metrics_t *m) {
  if (!m) {
    return;
  }
  memset(m, 0, sizeof(*m));
}

void dll_metrics_report(const dll_metrics_t *m, dll_scenario_t scenario) {
  if (!m) {
    return;
  }

  ESP_LOGI(TAG, "+--------------------------------------+");
  ESP_LOGI(TAG, "|  DLL SESSION REPORT (%s)             |",
           dll_scenario_name(scenario));
  ESP_LOGI(TAG, "+--------------------------------------+");
  ESP_LOGI(TAG, "|  BLE LL Retrans: %lu",
           (unsigned long)m->ble_ll_retransmissions);
  ESP_LOGI(TAG, "|  Wi-Fi MAC Retry: %lu",
           (unsigned long)m->wifi_mac_retry_frames);
  ESP_LOGI(TAG, "|  Wi-Fi MAC Drop:  %lu",
           (unsigned long)m->wifi_mac_drop_retry_limit);
  ESP_LOGI(TAG, "|  Zigbee ACK TO:   %lu",
           (unsigned long)m->zigbee_mac_ack_timeouts);
  ESP_LOGI(TAG, "|  Zigbee CCA Fail: %lu",
           (unsigned long)m->zigbee_cca_failures);
  ESP_LOGI(TAG, "|  Wi-Fi RSSI Avg:  %d dBm", m->wifi_rssi_avg_dbm);
  ESP_LOGI(TAG, "|  Wi-Fi MCS Rate:  %u", (unsigned)m->wifi_mcs_rate);
  ESP_LOGI(TAG, "+--------------------------------------+");
}
