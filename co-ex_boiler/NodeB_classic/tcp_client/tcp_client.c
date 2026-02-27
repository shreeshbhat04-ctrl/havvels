/*
 * ============================================================
 *  TCP Client — Coexistence Test (ESP-IDF)
 * ============================================================
 *
 *  What this code does:
 *   Waits for Wi-Fi connection (STA mode)
 *   Connects to Node A's TCP server
 *   Sends TEST_PACKET_COUNT coex_packet_t frames per batch
 *   Tracks sequence numbers for loss detection
 *   Auto-reconnects on failure
 *
 * ============================================================
 */

#include "tcp_client.h"
#include "coex_packet.h"
#include <errno.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

/* ================= CONFIGURATION ================= */

#define SERVER_IP "192.168.4.1" /* Node A AP default gateway */
#define TCP_PORT 5001
#define SEND_INTERVAL_MS 20 /* Interval between packets */

static const char *TAG = "TCP_CLIENT";

/* ------------------------------------------------- */
/* Check if Wi-Fi STA interface has a valid IP       */
/* ------------------------------------------------- */
static bool is_wifi_connected(void) {
  esp_netif_ip_info_t ip_info;
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

  if (netif) {
    esp_netif_get_ip_info(netif, &ip_info);
    return ip_info.ip.addr != 0;
  }
  return false;
}

/* ------------------------------------------------- */
/* TCP Client Task                                   */
/* ------------------------------------------------- */
static void tcp_client_task(void *arg) {
  uint16_t seq = 0;
  uint32_t batch = 0;

  /* Wait for Wi-Fi connection */
  ESP_LOGI(TAG, "Waiting for Wi-Fi IP...");
  while (!is_wifi_connected()) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  ESP_LOGI(TAG, "Wi-Fi connected");

  struct sockaddr_in server_addr = {
      .sin_family = AF_INET,
      .sin_port = htons(TCP_PORT),
      .sin_addr.s_addr = inet_addr(SERVER_IP),
  };

  /* -------- CONNECT + SEND LOOP (auto-reconnect) -------- */

  while (1) {

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
      ESP_LOGE(TAG, "Socket creation failed: %d", errno);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    ESP_LOGI(TAG, "Connecting to %s:%d...", SERVER_IP, TCP_PORT);

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) !=
        0) {
      ESP_LOGE(TAG, "Connection failed: %d", errno);
      close(sock);
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    ESP_LOGI(TAG, "Connected to server");

    /* Reset sequence for this batch */
    seq = 0;
    batch++;

    /* -------- SEND 1000 PACKETS -------- */

    while (seq < TEST_PACKET_COUNT) {

      /* Build coex packet */
      coex_packet_t pkt;
      memset(&pkt, 0, sizeof(pkt));
      pkt.header = 0xAA55;
      pkt.seq = seq;
      pkt.tx_timestamp = esp_timer_get_time();
      pkt.mode = MODE_WIFI;
      pkt.direction = DIR_REQUEST;
      pkt.rssi = 0;
      pkt.lqi = 0;
      pkt.tx_power = 20;
      pkt.channel = 6; /* Match AP channel */

      pkt.payload_len = snprintf((char *)pkt.payload, sizeof(pkt.payload),
                                 "B_%lu_%u", (unsigned long)batch, pkt.seq);

      pkt.crc = crc16((uint8_t *)&pkt, sizeof(coex_packet_t) - 2);

      int sent = send(sock, &pkt, sizeof(pkt), 0);
      if (sent < 0) {
        ESP_LOGE(TAG, "Send failed at seq %u", pkt.seq);
        break;
      }

      seq++;

      if (seq % 100 == 0) {
        ESP_LOGI(TAG, "Batch %lu: sent %u/%d packets",
                 (unsigned long)batch, seq, TEST_PACKET_COUNT);
      }

      vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }

    if (seq >= TEST_PACKET_COUNT) {
      ESP_LOGI(TAG, "Batch %lu complete: %d packets sent",
               (unsigned long)batch, TEST_PACKET_COUNT);
    }

    /* Short pause before next batch */
    close(sock);
    ESP_LOGI(TAG, "Disconnected, next batch in 5s...");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

/* ------------------------------------------------- */
/* Public function to start client task              */
/* ------------------------------------------------- */
void tcp_client_start(void) {
  xTaskCreatePinnedToCore(tcp_client_task, "tcp_client", 8192, NULL, 2, NULL,
                          1);
}
