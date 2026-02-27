/*
 * log_file.c - SPIFFS-based logging for ESP32 nodes
 */

#include "log_file.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define LOG_MOUNT_POINT "/spiffs"
#define LOG_FILE_PATH   "/spiffs/coex_log.txt"
#define LOG_MAX_SIZE    (64 * 1024)  /* 64KB max log size for H2 (limited flash) */

static const char *TAG = "LOG_FILE";
static FILE *s_log_file = NULL;
static SemaphoreHandle_t s_log_mutex = NULL;
static char s_node_name[32] = "NODE";
static vprintf_like_t s_original_vprintf = NULL;

/* Forward declaration */
static int log_file_vprintf(const char *fmt, va_list args);

esp_err_t log_file_init(const char *node_name) {
    if (node_name) {
        strncpy(s_node_name, node_name, sizeof(s_node_name) - 1);
        s_node_name[sizeof(s_node_name) - 1] = '\0';
    }

    /* Create mutex for thread safety */
    s_log_mutex = xSemaphoreCreateMutex();
    if (!s_log_mutex) {
        ESP_LOGE(TAG, "Failed to create log mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Configure SPIFFS */
    esp_vfs_spiffs_conf_t conf = {
        .base_path = LOG_MOUNT_POINT,
        .partition_label = "storage",
        .max_files = 2,
        .format_if_mount_failed = true,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount SPIFFS");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "SPIFFS partition not found - add 'storage' partition");
        }
        return ret;
    }

    /* Check partition info */
    size_t total = 0, used = 0;
    ret = esp_spiffs_info("storage", &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS: total=%d, used=%d bytes", total, used);
    }

    /* Open log file for appending */
    s_log_file = fopen(LOG_FILE_PATH, "a");
    if (!s_log_file) {
        ESP_LOGE(TAG, "Failed to open log file");
        return ESP_FAIL;
    }

    /* Write session header */
    int64_t uptime = esp_timer_get_time();
    fprintf(s_log_file, "\n========== %s SESSION START (uptime: %lld us) ==========\n",
            s_node_name, uptime);
    fflush(s_log_file);

    ESP_LOGI(TAG, "Log file initialized: %s", LOG_FILE_PATH);
    return ESP_OK;
}

int log_file_write(const char *format, ...) {
    if (!s_log_file || !s_log_mutex) {
        return -1;
    }

    if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return -1;
    }

    /* Check file size and rotate if needed */
    struct stat st;
    if (stat(LOG_FILE_PATH, &st) == 0 && st.st_size > LOG_MAX_SIZE) {
        fclose(s_log_file);
        /* Rotate: delete old, create new */
        remove(LOG_FILE_PATH);
        s_log_file = fopen(LOG_FILE_PATH, "w");
        if (s_log_file) {
            fprintf(s_log_file, "=== LOG ROTATED ===\n");
        }
    }

    int ret = 0;
    if (s_log_file) {
        /* Write timestamp prefix */
        int64_t uptime_ms = esp_timer_get_time() / 1000;
        fprintf(s_log_file, "[%lld] ", uptime_ms);

        /* Write formatted message */
        va_list args;
        va_start(args, format);
        ret = vfprintf(s_log_file, format, args);
        va_end(args);

        /* Add newline if not present */
        size_t len = strlen(format);
        if (len > 0 && format[len - 1] != '\n') {
            fprintf(s_log_file, "\n");
        }

        fflush(s_log_file);
    }

    xSemaphoreGive(s_log_mutex);
    return ret;
}

void log_file_close(void) {
    if (s_log_mutex) {
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    }

    if (s_log_file) {
        fprintf(s_log_file, "========== SESSION END ==========\n");
        fflush(s_log_file);
        fclose(s_log_file);
        s_log_file = NULL;
    }

    if (s_log_mutex) {
        xSemaphoreGive(s_log_mutex);
        vSemaphoreDelete(s_log_mutex);
        s_log_mutex = NULL;
    }

    esp_vfs_spiffs_unregister("storage");
    ESP_LOGI(TAG, "Log file closed");
}

size_t log_file_get_size(void) {
    struct stat st;
    if (stat(LOG_FILE_PATH, &st) == 0) {
        return st.st_size;
    }
    return 0;
}

esp_err_t log_file_clear(void) {
    if (s_log_mutex) {
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    }

    if (s_log_file) {
        fclose(s_log_file);
    }

    remove(LOG_FILE_PATH);

    s_log_file = fopen(LOG_FILE_PATH, "w");
    if (!s_log_file) {
        if (s_log_mutex) {
            xSemaphoreGive(s_log_mutex);
        }
        return ESP_FAIL;
    }

    fprintf(s_log_file, "=== LOG CLEARED ===\n");
    fflush(s_log_file);

    if (s_log_mutex) {
        xSemaphoreGive(s_log_mutex);
    }

    ESP_LOGI(TAG, "Log file cleared");
    return ESP_OK;
}

void log_file_dump(void) {
    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (!f) {
        ESP_LOGW(TAG, "No log file to dump");
        return;
    }

    ESP_LOGI(TAG, "=== LOG FILE DUMP START ===");
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        printf("%s", line);
    }
    ESP_LOGI(TAG, "=== LOG FILE DUMP END ===");

    fclose(f);
}

/* Custom vprintf to redirect ESP_LOG to file */
static int log_file_vprintf(const char *fmt, va_list args) {
    /* First, call original to print to serial */
    int ret = 0;
    if (s_original_vprintf) {
        va_list args_copy;
        va_copy(args_copy, args);
        ret = s_original_vprintf(fmt, args_copy);
        va_end(args_copy);
    }

    /* Then write to log file */
    if (s_log_file && s_log_mutex) {
        if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            vfprintf(s_log_file, fmt, args);
            /* Flush every 10th write to avoid excessive I/O */
            static int flush_counter = 0;
            if (++flush_counter >= 10) {
                fflush(s_log_file);
                flush_counter = 0;
            }
            xSemaphoreGive(s_log_mutex);
        }
    }

    return ret;
}

void log_file_redirect_esp_log(void) {
    if (!s_log_file) {
        ESP_LOGW(TAG, "Call log_file_init() before redirecting logs");
        return;
    }

    s_original_vprintf = esp_log_set_vprintf(log_file_vprintf);
    ESP_LOGI(TAG, "ESP_LOG output now redirected to file");
}
