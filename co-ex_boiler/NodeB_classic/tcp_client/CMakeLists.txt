idf_component_register(
    SRCS "tcp_client.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif nvs_flash lwip coex_packet
    PRIV_REQUIRES esp_timer
)
