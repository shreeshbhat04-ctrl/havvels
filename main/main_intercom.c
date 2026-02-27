/**
 * main_intercom.c — Bidirectional Inter-Communication Firmware
 *
 * Node A (ESP32-S3) — Master: Wi-Fi AP + BLE Scanner + UART bridge to Node D
 *                     Sends packets, receives acks, tracks packet loss.
 * Node B (ESP32)    — Wi-Fi STA → Node A AP, BLE Advertiser.
 *                     Echoes back Wi-Fi packets from Node A.
 * Node D (ESP32-H2) — Zigbee Coordinator, UART relay ↔ Node A.
 *                     Relays packets between UART and 802.15.4.
 * Node C (ESP32-H2) — Zigbee End Device → echoes back to Node D.
 *
 * Build flags:
 *   idf.py -DUSE_INTERCOM=ON build
 *   (H2 only) -DIS_COORDINATOR=ON  → Node D
 *             -DIS_COORDINATOR=OFF → Node C
 */

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
#else
#include <time.h>
#define esp_timer_get_time() ((int64_t)0)
#endif

#include "coex_packet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"

static const char *TAG = "INTERCOM";

/* ============ UART Bridge Config (S3 ↔ H2) ============ */
#define UART_BRIDGE_PORT UART_NUM_1
#define UART_BRIDGE_BAUD 115200
#define UART_BRIDGE_TX_PIN 17 /* S3 GPIO17 → H2 RX */
#define UART_BRIDGE_RX_PIN 18 /* S3 GPIO18 ← H2 TX */
#define UART_BUF_SIZE 256

/* H2 side: use UART1 with its own pins */
#define H2_UART_PORT UART_NUM_1
#define H2_UART_TX_PIN 4 /* H2 GPIO4  → S3 RX (GPIO18) */
#define H2_UART_RX_PIN 5 /* H2 GPIO5  ← S3 TX (GPIO17) */

/* ============ Wi-Fi Config (S3 = AP, Others = STA) ============ */
#define HUB_SSID "ESP32-COEX-HUB"
#define HUB_PASS "" /* Open network for simplicity */
#define HUB_IP "192.168.4.1"
#define HUB_PORT 5000

/* ============ Channel / Timing Config ============ */
#define ZIGBEE_CHANNEL 20     /* 2450 MHz */
#define INTERCOM_DELAY_MS 100 /* Packet send interval */

/* ============ Target Detection ============ */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define DEVICE_ROLE "NODE_A_S3"
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#if IS_ZIGBEE_COORDINATOR
#define DEVICE_ROLE "NODE_D_H2_COORD"
#else
#define DEVICE_ROLE "NODE_C_H2_END"
#endif
#elif defined(CONFIG_IDF_TARGET_ESP32)
#define DEVICE_ROLE "NODE_B_ESP32"
#else
#define DEVICE_ROLE "UNKNOWN"
#endif

/* ========================================================================
 *  NODE A — ESP32-S3: Master Hub
 *  Wi-Fi AP + BLE Scanner + UART bridge + packet loss tracker
 * ======================================================================== */
#if defined(CONFIG_IDF_TARGET_ESP32S3)

/* ---- Packet loss statistics ---- */
static radio_stats_t wifi_stats = {0};
static radio_stats_t ble_stats = {0};
static radio_stats_t zigbee_stats = {0};

/* ---- Sequence counter (Zigbee) ---- */
static uint16_t zigbee_tx_seq = 0;

/* ---- UART bridge to Node D (H2) ---- */
static void uart_bridge_init(void) {
  uart_config_t cfg = {
      .baud_rate = UART_BRIDGE_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(
      uart_driver_install(UART_BRIDGE_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_BRIDGE_PORT, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_BRIDGE_PORT, UART_BRIDGE_TX_PIN,
                               UART_BRIDGE_RX_PIN, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ESP_LOGI(TAG, "UART bridge init: TX=%d, RX=%d", UART_BRIDGE_TX_PIN,
           UART_BRIDGE_RX_PIN);
}

/* Task: read coex_packet_t responses from H2 via UART */
static void uart_rx_task(void *pvParameters) {
  uint8_t buf[sizeof(coex_packet_t)];
  while (1) {
    int len = uart_read_bytes(UART_BRIDGE_PORT, buf, sizeof(coex_packet_t),
                              pdMS_TO_TICKS(100));
    if (len == sizeof(coex_packet_t)) {
      coex_packet_t *pkt = (coex_packet_t *)buf;
      if (pkt->header == 0xAA55 && pkt->direction == DIR_RESPONSE) {
        zigbee_stats.rx_count++;
        ESP_LOGI(TAG, "[ZB ACK←H2] seq=%d ch=%d rssi=%d (rx=%lu)", pkt->seq,
                 pkt->channel, pkt->rssi, (unsigned long)zigbee_stats.rx_count);
      }
    }
  }
}

/* Task: periodically send coex_packet_t to Node D via UART (→Zigbee→C) */
static void zigbee_tx_task(void *pvParameters) {
  while (1) {
    coex_packet_t pkt = {
        .header = 0xAA55,
        .seq = zigbee_tx_seq++,
        .tx_timestamp = esp_timer_get_time(),
        .mode = MODE_ZIGBEE,
        .direction = DIR_REQUEST,
        .rssi = 0,
        .lqi = 0,
        .tx_power = 20,
        .channel = ZIGBEE_CHANNEL,
    };
    pkt.payload_len = sprintf((char *)pkt.payload, "A_ZB_%d", pkt.seq);
    pkt.crc = crc16((uint8_t *)&pkt, sizeof(coex_packet_t) - 2);

    uart_write_bytes(UART_BRIDGE_PORT, (const char *)&pkt,
                     sizeof(coex_packet_t));
    zigbee_stats.tx_count++;
  }
}

/* ---- Wi-Fi AP Initialization (S3) ---- */
static void wifi_ap_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = HUB_SSID,
              .ssid_len = strlen(HUB_SSID),
              .channel = 1,
              .password = HUB_PASS,
              .max_connection = 4,
              .authmode = WIFI_AUTH_WPA2_PSK,
          },
  };
  if (strlen(HUB_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Wi-Fi AP started. SSID:%s", HUB_SSID);
}

/* Task: TCP Server on Hub (S3) — Receive stats from Node B */
static void tcp_server_task(void *pvParameters) {
  char addr_str[128];
  int addr_family = AF_INET;
  int ip_protocol = IPPROTO_IP;
  struct sockaddr_storage dest_addr;

  struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
  dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr_ip4->sin_family = AF_INET;
  dest_addr_ip4->sin_port = htons(HUB_PORT);

  int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
  if (listen_sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }

  int opt = 1;
  setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) !=
      0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    goto CLEAN_UP;
  }

  if (listen(listen_sock, 1) != 0) {
    ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
    goto CLEAN_UP;
  }

  while (1) {
    ESP_LOGI(TAG, "TCP Server listening on port %d", HUB_PORT);
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
      break;
    }

    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str,
                sizeof(addr_str) - 1);
    ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

    while (1) {
      coex_packet_t pkt;
      int len = recv(sock, &pkt, sizeof(pkt), 0);
      if (len < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        break;
      } else if (len == 0) {
        ESP_LOGI(TAG, "Connection closed");
        break;
      } else {
        if (pkt.header == 0xAA55) {
          wifi_stats.rx_count++;
          /* We assume the client tracks its own TX and sends it or we estimate
           */
          wifi_stats.tx_count = pkt.seq + 1;
        }
      }
    }

    if (sock != -1) {
      shutdown(sock, 0);
      close(sock);
    }
  }

CLEAN_UP:
  close(listen_sock);
  vTaskDelete(NULL);
}

/* ---- BLE Scanner (receives advertisements from Node B) ---- */
static void ble_scan_result_handler(esp_gap_ble_cb_event_t event,
                                    esp_ble_gap_cb_param_t *param) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  if (event == ESP_GAP_BLE_EXT_ADV_REPORT_EVT) {
    uint8_t *adv_data = param->ext_adv_report.params.adv_data;
    uint8_t adv_len = param->ext_adv_report.params.adv_data_len;

    /* Filter for our custom manufacturer data prefix 0xFFFF + 0xAA marker */
    for (int i = 0; i < adv_len - 4; i++) {
      if (adv_data[i] == 0xFF && adv_data[i + 1] == 0xFF &&
          adv_data[i + 2] == 0xFF && adv_data[i + 3] == 0xAA) {
        ble_stats.rx_count++;
        break;
      }
    }
  }
#else
  if (event == ESP_GAP_BLE_SCAN_RESULT_EVT &&
      param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
    uint8_t *adv_data = param->scan_rst.ble_adv;
    uint8_t adv_len = param->scan_rst.adv_data_len;

    /* Filter for our custom manufacturer data prefix 0xFFFF + 0xAA marker */
    for (int i = 0; i < adv_len - 4; i++) {
      if (adv_data[i] == 0xFF && adv_data[i + 1] == 0xFF &&
          adv_data[i + 2] == 0xFF && adv_data[i + 3] == 0xAA) {
        ble_stats.rx_count++;
        break;
      }
    }
  }
#endif
}

static void ble_scanner_init(void) {
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());

  ESP_ERROR_CHECK(esp_ble_gap_register_callback(ble_scan_result_handler));

#ifdef CONFIG_IDF_TARGET_ESP32S3
  esp_ble_ext_scan_params_t scan_params = {
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
      .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK,
      .uncoded_cfg = {BLE_SCAN_TYPE_ACTIVE, 0x50, 0x30},
  };
  ESP_ERROR_CHECK(esp_ble_gap_set_ext_scan_params(&scan_params));
  ESP_ERROR_CHECK(esp_ble_gap_start_ext_scan(0, 0)); /* scan forever */
#else
  esp_ble_scan_params_t scan_params = {
      .scan_type = BLE_SCAN_TYPE_ACTIVE,
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_interval = 0x50, /* 50ms */
      .scan_window = 0x30,   /* 30ms */
      .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
  };
  ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
  ESP_ERROR_CHECK(esp_ble_gap_start_scanning(0)); /* scan forever */
#endif
  ESP_LOGI(TAG, "BLE scanner started");
}

/* ---- Dashboard task: periodic summary to Serial ---- */
static void dashboard_task(void *pvParameters) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    float wifi_loss =
        wifi_stats.tx_count > 0
            ? 100.0f * (1.0f - (float)wifi_stats.rx_count / wifi_stats.tx_count)
            : 0.0f;
    float ble_loss =
        ble_stats.tx_count > 0
            ? 100.0f * (1.0f - (float)ble_stats.rx_count / ble_stats.tx_count)
            : 0.0f;
    float zb_loss = zigbee_stats.tx_count > 0
                        ? 100.0f * (1.0f - (float)zigbee_stats.rx_count /
                                               zigbee_stats.tx_count)
                        : 0.0f;

    ESP_LOGI(TAG, "============= GLOBAL COEX DASHBOARD (HUB) =============");
    ESP_LOGI(TAG, "  Wi-Fi:  TX=%-6lu RX=%-6lu Loss=%.1f%% (B->A TCP)",
             (unsigned long)wifi_stats.tx_count,
             (unsigned long)wifi_stats.rx_count, wifi_loss);
    ESP_LOGI(TAG, "  BLE:    TX=%-6lu RX=%-6lu Loss=%.1f%%",
             (unsigned long)ble_stats.tx_count,
             (unsigned long)ble_stats.rx_count, ble_loss);
    ESP_LOGI(TAG, "  Zigbee: TX=%-6lu RX=%-6lu Loss=%.1f%%",
             (unsigned long)zigbee_stats.tx_count,
             (unsigned long)zigbee_stats.rx_count, zb_loss);
    ESP_LOGI(TAG, "=======================================================");
  }
}

/* ---- BLE "TX" counter task (estimates expected BLE adverts) ---- */
static void ble_tx_counter_task(void *pvParameters) {
  /* BLE advertising is broadcast — we estimate expected adverts
     based on Node B's advertising interval (~30ms) */
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(30)); /* Match Node B's ~30ms adv interval */
    ble_stats.tx_count++;
  }
}

#endif /* CONFIG_IDF_TARGET_ESP32S3 */

/* ========================================================================
 *  NODE B — ESP32 Classic: Wi-Fi STA → Node A AP + BLE Advertiser
 *  Receives requests from Node A, echoes back as responses.
 * ======================================================================== */
#if defined(CONFIG_IDF_TARGET_ESP32)

static volatile bool wifi_connected = false;

/* ---- Wi-Fi STA mode (Node B) ---- */
static void wifi_sta_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_connected = false;
    ESP_LOGI(TAG, "Disconnected from Hub, retrying...");
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    wifi_connected = true;
    ESP_LOGI(TAG, "Connected to Hub AP");
  }
}

static void wifi_sta_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_sta_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_sta_event_handler, NULL));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = HUB_SSID,
              .password = HUB_PASS,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

/* Task: TCP client on Spoke (Node B) — Send stats to Hub (S3) */
static void wifi_client_task(void *pvParameters) {
  struct sockaddr_in dest_addr = {
      .sin_addr.s_addr = inet_addr(HUB_IP),
      .sin_family = AF_INET,
      .sin_port = htons(HUB_PORT),
  };
  uint16_t seq = 0;

  while (1) {
    if (!wifi_connected) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
      close(sock);
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    while (wifi_connected) {
      coex_packet_t pkt = {
          .header = 0xAA55,
          .seq = seq++,
          .tx_timestamp = esp_timer_get_time(),
          .mode = MODE_WIFI,
          .direction = DIR_REQUEST,
      };
      pkt.payload_len = sprintf((char *)pkt.payload, "B_CLIENT_%d", pkt.seq);
      pkt.crc = crc16((uint8_t *)&pkt, sizeof(coex_packet_t) - 2);

      if (send(sock, &pkt, sizeof(coex_packet_t), 0) < 0) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(INTERCOM_DELAY_MS));
    }
    close(sock);
  }
}

/* ---- BLE Advertiser ---- */
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x20, /* 20ms */
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT) {
    if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(TAG, "BLE advertising started");
    } else {
      ESP_LOGE(TAG, "BLE advertising failed, status=%d",
               param->adv_start_cmpl.status);
    }
  }
}

static void ble_advertiser_init(void) {
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());
  ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);

  /* Custom adv data with our 0xFFFF 0xAA marker for Node A scanner */
  uint8_t adv_raw[] = {
      0x02, 0x01, 0x06,                        /* Flags */
      0x0B, 0xFF, 0xFF, 0xFF, 0xAA,            /* Manufacturer: our marker */
      0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22 /* Payload */
  };
  ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(adv_raw, sizeof(adv_raw)));
  ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
  ESP_LOGI(TAG, "BLE advertiser started (+9 dBm)");
}

#endif /* CONFIG_IDF_TARGET_ESP32 */

/* ========================================================================
 *  ESP32-H2: Shared between Node D (Coordinator) and Node C (End Device)
 *  Selected via IS_ZIGBEE_COORDINATOR compile flag.
 * ======================================================================== */
#ifdef CONFIG_IDF_TARGET_ESP32H2

#include "driver/uart.h"
#include "esp_ieee802154.h"

/* ---- 802.15.4 frame format for carrying coex_packet_t ---- */
#define ZB_FRAME_HEADER_LEN 7 /* FCF(2) + Seq(1) + PAN(2) + Dest(2) */
#define ZB_MAX_PAYLOAD (127 - ZB_FRAME_HEADER_LEN - 2) /* minus FCS */

/* ---- Shared 802.15.4 RX buffer ---- */
static volatile bool ieee_rx_pending = false;
static uint8_t ieee_rx_buf[128];
static int ieee_rx_len = 0;

/* 802.15.4 receive callback */
void esp_ieee802154_receive_done(uint8_t *frame,
                                 esp_ieee802154_frame_info_t *frame_info) {
  if (frame && frame[0] > ZB_FRAME_HEADER_LEN) {
    int payload_len = frame[0] - ZB_FRAME_HEADER_LEN;
    if (payload_len <= (int)sizeof(ieee_rx_buf)) {
      memcpy(ieee_rx_buf, &frame[1 + ZB_FRAME_HEADER_LEN], payload_len);
      ieee_rx_len = payload_len;
      ieee_rx_pending = true;
    }
  }
}

/* Called on TX done */
void esp_ieee802154_transmit_done(const uint8_t *frame, const uint8_t *ack,
                                  esp_ieee802154_frame_info_t *ack_frame_info) {
  /* No action needed, fire-and-forget */
}

/* Called on TX failure */
void esp_ieee802154_transmit_failed(const uint8_t *frame,
                                    esp_ieee802154_tx_error_t error) {
  ESP_LOGW(TAG, "802.15.4 TX failed, err=%d", error);
}

/* Build and transmit an 802.15.4 frame containing a coex_packet_t */
static void zigbee_send_packet(coex_packet_t *pkt) {
  uint8_t frame[128];
  int pkt_len = sizeof(coex_packet_t);

  if (pkt_len + ZB_FRAME_HEADER_LEN > 125) {
    ESP_LOGE(TAG, "Packet too large for 802.15.4 frame");
    return;
  }

  frame[0] = ZB_FRAME_HEADER_LEN + pkt_len; /* PHR: frame length */
  frame[1] = 0x01;                          /* FCF low: data frame */
  frame[2] = 0x08;                          /* FCF high: no ack request */
  frame[3] = (uint8_t)(pkt->seq & 0xFF);    /* Sequence */
  frame[4] = 0xFF;
  frame[5] = 0xFF; /* PAN ID: broadcast */
  frame[6] = 0xFF;
  frame[7] = 0xFF; /* Dest addr: broadcast */
  memcpy(&frame[1 + ZB_FRAME_HEADER_LEN], pkt, pkt_len);

  esp_ieee802154_transmit(frame, false);
}

#if IS_ZIGBEE_COORDINATOR
/* ========================================================================
 *  NODE D — Zigbee Coordinator + UART relay to/from Node A
 * ======================================================================== */

/* UART to S3 */
static void uart_bridge_init_h2(void) {
  uart_config_t cfg = {
      .baud_rate = UART_BRIDGE_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(
      uart_driver_install(H2_UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(H2_UART_PORT, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(H2_UART_PORT, H2_UART_TX_PIN, H2_UART_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_LOGI(TAG, "H2 UART bridge init: TX=%d, RX=%d", H2_UART_TX_PIN,
           H2_UART_RX_PIN);
}

/* Task: UART RX from Node A → 802.15.4 TX to Node C */
static void coord_uart_to_zigbee_task(void *pvParameters) {
  uint8_t buf[sizeof(coex_packet_t)];
  while (1) {
    int len = uart_read_bytes(H2_UART_PORT, buf, sizeof(coex_packet_t),
                              pdMS_TO_TICKS(50));
    if (len == sizeof(coex_packet_t)) {
      coex_packet_t *pkt = (coex_packet_t *)buf;
      if (pkt->header == 0xAA55 && pkt->direction == DIR_REQUEST) {
        /* Relay to Node C over 802.15.4 */
        zigbee_send_packet(pkt);
        ESP_LOGI(TAG, "[RELAY A→C] seq=%d ch=%d via 802.15.4", pkt->seq,
                 pkt->channel);
      }
    }
  }
}

/* Task: 802.15.4 RX from Node C → UART TX to Node A */
static void coord_zigbee_to_uart_task(void *pvParameters) {
  while (1) {
    if (ieee_rx_pending && ieee_rx_len >= (int)sizeof(coex_packet_t)) {
      coex_packet_t *pkt = (coex_packet_t *)ieee_rx_buf;
      if (pkt->header == 0xAA55 && pkt->direction == DIR_RESPONSE) {
        /* Relay ack from Node C back to Node A via UART */
        uart_write_bytes(H2_UART_PORT, (const char *)pkt,
                         sizeof(coex_packet_t));
        ESP_LOGI(TAG, "[RELAY C→A] seq=%d ack via UART", pkt->seq);
      }
      ieee_rx_pending = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#else /* !IS_ZIGBEE_COORDINATOR → Node C (End Device) */
/* ========================================================================
 *  NODE C — Zigbee End Device: receives from Node D, echoes back
 * ======================================================================== */

/* Task: process received 802.15.4 frames and echo back */
static void end_device_task(void *pvParameters) {
  uint16_t rx_count = 0;
  int64_t last_log = 0;

  while (1) {
    if (ieee_rx_pending && ieee_rx_len >= (int)sizeof(coex_packet_t)) {
      coex_packet_t *pkt = (coex_packet_t *)ieee_rx_buf;
      if (pkt->header == 0xAA55 && pkt->direction == DIR_REQUEST) {
        rx_count++;

        /* Echo back as response */
        coex_packet_t ack = *pkt;
        ack.direction = DIR_RESPONSE;
        ack.tx_timestamp = esp_timer_get_time();
        ack.payload_len = sprintf((char *)ack.payload, "C_ACK_%d", ack.seq);
        ack.crc = crc16((uint8_t *)&ack, sizeof(coex_packet_t) - 2);

        zigbee_send_packet(&ack);

        if (esp_timer_get_time() - last_log > 2000000) {
          ESP_LOGI(TAG, "[END_DEV] RX=%u, echoed seq=%d → Node D", rx_count,
                   ack.seq);
          last_log = esp_timer_get_time();
        }
      }
      ieee_rx_pending = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#endif /* IS_ZIGBEE_COORDINATOR */

#endif /* CONFIG_IDF_TARGET_ESP32H2 */

/* ========================================================================
 *  app_main — entry point for all targets
 * ======================================================================== */
void app_main(void) {
  ESP_LOGI(TAG, "=== INTER-COMMUNICATION MODE ===");
  ESP_LOGI(TAG, "Device Role: %s", DEVICE_ROLE);

  /* NVS init (required for Wi-Fi, BLE) */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

/* ---- Node A (S3): Master Hub — AP + TCP Server + BLE scanner + UART Bridge
 * ---- */
#ifdef CONFIG_IDF_TARGET_ESP32S3
  wifi_ap_init();
  uart_bridge_init();
  ble_scanner_init();
  xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
  xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL);
  xTaskCreate(zigbee_tx_task, "zb_tx", 4096, NULL, 5, NULL);
  xTaskCreate(dashboard_task, "dashboard", 4096, NULL, 3, NULL);
  xTaskCreate(ble_tx_counter_task, "ble_cnt", 2048, NULL, 2, NULL);
#endif

/* ---- Node B (ESP32): STA + TCP Client + BLE advertiser ---- */
#ifdef CONFIG_IDF_TARGET_ESP32
  wifi_sta_init();
  ble_advertiser_init();
  xTaskCreate(wifi_client_task, "wifi_client", 4096, NULL, 5, NULL);
#endif

/* ---- Node D or Node C (H2) ---- */
#ifdef CONFIG_IDF_TARGET_ESP32H2
  ESP_LOGI(TAG, "Initializing 802.15.4 on CH=%d", ZIGBEE_CHANNEL);
  esp_ieee802154_enable();
  esp_ieee802154_set_channel(ZIGBEE_CHANNEL);
  esp_ieee802154_set_txpower(20);
  esp_ieee802154_set_panid(0xFFFF);
  esp_ieee802154_set_promiscuous(true);
  esp_ieee802154_receive(); /* Start receiving */

#if IS_ZIGBEE_COORDINATOR
  /* Node D: UART relay ↔ Zigbee */
  uart_bridge_init_h2();
  xTaskCreate(coord_uart_to_zigbee_task, "u2z", 4096, NULL, 5, NULL);
  xTaskCreate(coord_zigbee_to_uart_task, "z2u", 4096, NULL, 5, NULL);
#else
  /* Node C: End device — echo back */
  xTaskCreate(end_device_task, "end_dev", 4096, NULL, 5, NULL);
#endif

#endif /* CONFIG_IDF_TARGET_ESP32H2 */
}
