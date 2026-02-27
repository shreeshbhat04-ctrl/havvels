/*
 * log_file.h - SPIFFS-based logging for ESP32 nodes
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SPIFFS and open log file
 * @param node_name Name of the node (e.g., "NodeC")
 * @return ESP_OK on success
 */
esp_err_t log_file_init(const char *node_name);

/**
 * @brief Write a formatted log line to the log file
 * @param format printf-style format string
 * @return Number of bytes written, or -1 on error
 */
int log_file_write(const char *format, ...);

/**
 * @brief Flush and close the log file
 */
void log_file_close(void);

/**
 * @brief Get current log file size in bytes
 */
size_t log_file_get_size(void);

/**
 * @brief Clear all logs and start fresh
 */
esp_err_t log_file_clear(void);

/**
 * @brief Print log file contents to serial (for debugging/extraction)
 */
void log_file_dump(void);

/**
 * @brief Redirect ESP_LOG output to also write to log file
 * Call after log_file_init()
 */
void log_file_redirect_esp_log(void);

#ifdef __cplusplus
}
#endif
