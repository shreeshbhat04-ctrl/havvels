#ifndef DLL_METRICS_H
#define DLL_METRICS_H

#include <stdint.h>

typedef struct {
  uint32_t ble_ll_retransmissions;
  uint32_t wifi_mac_retry_frames;
  uint32_t wifi_mac_drop_retry_limit;
  uint32_t zigbee_mac_ack_timeouts;
  uint32_t zigbee_cca_failures;
  int8_t wifi_rssi_avg_dbm;
  uint8_t wifi_mcs_rate;
} dll_metrics_t;

typedef enum {
  SCENARIO_HOME_IOT = 0,
  SCENARIO_OFFICE_HIGH_DENSITY,
} dll_scenario_t;

void dll_metrics_reset(dll_metrics_t *m);
void dll_metrics_report(const dll_metrics_t *m, dll_scenario_t scenario);

#endif
