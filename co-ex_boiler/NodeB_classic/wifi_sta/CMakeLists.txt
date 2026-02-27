idf_component_register(
    SRCS "wifi_sta.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif nvs_flash lwip
    PRIV_REQUIRES esp_timer
)
