idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES wifi_sta
                             tcp_client
                             ble_peripheral
                             coex_packet
                    PRIV_REQUIRES nvs_flash esp_event)
