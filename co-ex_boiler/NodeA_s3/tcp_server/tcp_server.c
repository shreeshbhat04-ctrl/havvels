#include "tcp_server.h"
#include "coex_packet.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include <errno.h>
#include <string.h>

/* ================= Configuration ================= */

#define TCP_PORT 5001
#define RX_BUF_SIZE 4096

static const char *TAG = "TCP_SERVER";

/* ================= Global Counters ================= */

static volatile uint32_t g_wifi_rx = 0;
static volatile uint32_t g_wifi_tx = 0;

/* 1000-packet session tracker */
static test_session_t g_wifi_session;

/* ================= Session Helpers ================= */

static void session_reset(test_session_t *s) {
  memset(s, 0, sizeof(*s));
  s->target_count = TEST_PACKET_COUNT;
  s->rssi_min = 0;
  s->rssi_max = -127;
  s->complete = false;
}

/* Retrieve current RSSI (STA or AP mode) */
static int8_t get_current_rssi(void) {
  wifi_ap_record_t ap_info;
  wifi_sta_list_t sta_list;

  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    return ap_info.rssi;
  }
  if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK && sta_list.num > 0) {
    return sta_list.sta[0].rssi;
  }
  return 0;
}

static void print_wifi_session_report(test_session_t *s) {
  double duration = (s->end_time_us - s->start_time_us) / 1000000.0;
  double throughput_mbps =
      (duration > 0.001) ? (s->total_bytes * 8.0) / (duration * 1e6) : 0.0;
  double avg_jitter =
      (s->rx_count > 1) ? (double)s->jitter_sum_us / (s->rx_count - 1) : 0.0;
  double pdr =
      s->target_count > 0 ? 100.0 * s->rx_count / s->target_count : 0.0;
  double avg_latency =
      s->rx_count > 0 ? (double)s->latency_sum_us / s->rx_count : 0.0;
  double avg_rssi =
      s->rx_count > 0 ? (double)s->rssi_sum / s->rx_count : 0.0;

  ESP_LOGI(TAG, "+--------------------------------------+");
  ESP_LOGI(TAG, "|  WI-FI SESSION REPORT                |");
  ESP_LOGI(TAG, "+--------------------------------------+");
  ESP_LOGI(TAG, "|  RX:         %lu / %lu",
           (unsigned long)s->rx_count, (unsigned long)s->target_count);
  ESP_LOGI(TAG, "|  Lost:       %lu", (unsigned long)s->lost_count);
  ESP_LOGI(TAG, "|  PDR:        %.1f%%", pdr);
  ESP_LOGI(TAG, "|  Bytes:      %lu", (unsigned long)s->total_bytes);
  ESP_LOGI(TAG, "|  Duration:   %.2f s", duration);
  ESP_LOGI(TAG, "|  Throughput: %.3f Mbps", throughput_mbps);
  ESP_LOGI(TAG, "|  Avg Jitter: %.1f us", avg_jitter);
  ESP_LOGI(TAG, "|  Avg Latency:%.1f us", avg_latency);
  ESP_LOGI(TAG, "|  RSSI:       min=%d max=%d avg=%.1f dBm",
           s->rssi_min, s->rssi_max, avg_rssi);
  ESP_LOGI(TAG, "+--------------------------------------+");
}

/* ================= Public Accessors ================= */

void tcp_server_get_stats(uint32_t *rx, uint32_t *tx) {
  if (rx)
    *rx = g_wifi_rx;
  if (tx)
    *tx = g_wifi_tx;
}

bool tcp_server_session_complete(void) {
  return g_wifi_session.complete;
}

const test_session_t *tcp_server_get_session(void) {
  return &g_wifi_session;
}

/* ============================================================
 *  TCP Server Task
 * ============================================================ */

static void tcp_server_task(void *arg) {
  int listen_sock = -1;
  int client_sock = -1;

  uint8_t *rx_buf = (uint8_t *)malloc(RX_BUF_SIZE);

  if (!rx_buf) {
    ESP_LOGE(TAG, "Memory allocation failed");
    vTaskDelete(NULL);
  }

  struct sockaddr_in server_addr = {
      .sin_family = AF_INET,
      .sin_port = htons(TCP_PORT),
      .sin_addr.s_addr = htonl(INADDR_ANY),
  };

  listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (listen_sock < 0) {
    ESP_LOGE(TAG, "Socket creation failed: %d", errno);
    goto cleanup;
  }

  int opt = 1;
  setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) !=
      0) {
    ESP_LOGE(TAG, "Bind failed: %d", errno);
    goto cleanup;
  }

  listen(listen_sock, 1);

  ESP_LOGI(TAG, "TCP server listening on port %d", TCP_PORT);

  /* ================= MAIN SERVER LOOP ================= */

  while (1) {

    client_sock = accept(listen_sock, NULL, NULL);
    if (client_sock < 0) {
      continue;
    }

    ESP_LOGI(TAG, "Client connected");

    /* Reset session for this connection */
    session_reset(&g_wifi_session);
    g_wifi_session.start_time_us = esp_timer_get_time();

    /* -------- CLIENT RECEIVE LOOP -------- */
    while (1) {

      int len = recv(client_sock, rx_buf, RX_BUF_SIZE, 0);

      if (len <= 0) {
        break;
      }

      /* Process complete coex_packet_t frames */
      int offset = 0;
      while (offset + (int)sizeof(coex_packet_t) <= len) {

        coex_packet_t *pkt = (coex_packet_t *)&rx_buf[offset];
        offset += sizeof(coex_packet_t);

        /* Validate header */
        if (pkt->header != 0xAA55) {
          continue;
        }

        int64_t now = esp_timer_get_time();

        /* Track jitter (inter-packet arrival time variation) */
        if (g_wifi_session.last_arrival_us > 0) {
          int64_t delta = now - g_wifi_session.last_arrival_us;
          if (delta < 0)
            delta = -delta;
          g_wifi_session.jitter_sum_us += delta;
        }
        g_wifi_session.last_arrival_us = now;

        /* Latency (one-way estimate from tx_timestamp) */
        if (pkt->tx_timestamp > 0) {
          int64_t lat = now - (int64_t)pkt->tx_timestamp;
          if (lat < 0)
            lat = -lat;
          g_wifi_session.latency_sum_us += lat;
        }

        /* Detect lost packets via sequence gap */
        if (g_wifi_session.rx_count > 0) {
          int gap = (int)pkt->seq - (int)g_wifi_session.last_seq - 1;
          if (gap > 0) {
            g_wifi_session.lost_count += gap;
          }
        }
        g_wifi_session.last_seq = pkt->seq;

        /* Update RSSI range */
        int8_t rssi = get_current_rssi();
        g_wifi_session.rssi_sum += rssi;
        if (rssi < g_wifi_session.rssi_min)
          g_wifi_session.rssi_min = rssi;
        if (rssi > g_wifi_session.rssi_max)
          g_wifi_session.rssi_max = rssi;

        g_wifi_session.rx_count++;
        g_wifi_session.total_bytes += sizeof(coex_packet_t);
        g_wifi_rx++;

        /* Check if 1000-packet session is complete */
        if (g_wifi_session.rx_count >= g_wifi_session.target_count) {
          g_wifi_session.end_time_us = now;
          g_wifi_session.complete = true;
          print_wifi_session_report(&g_wifi_session);

          /* Auto-reset for next batch */
          session_reset(&g_wifi_session);
          g_wifi_session.start_time_us = esp_timer_get_time();
        }
      }
    }

    /* Client disconnected — print partial session if any data received */
    if (g_wifi_session.rx_count > 0 && !g_wifi_session.complete) {
      g_wifi_session.end_time_us = esp_timer_get_time();
      print_wifi_session_report(&g_wifi_session);
    }

    ESP_LOGI(TAG, "Client disconnected");
    close(client_sock);
  }

cleanup:

  if (listen_sock >= 0) {
    close(listen_sock);
  }

  if (rx_buf) {
    free(rx_buf);
  }

  vTaskDelete(NULL);
}

void tcp_server_start(void) {
  xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 12288, NULL, 2, NULL,
                          1);
}
