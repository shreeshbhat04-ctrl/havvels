idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        esp_wifi
                        esp_netif
                        bt
                        wifi_ap
                        tcp_server
                        ble_central
                    PRIV_REQUIRES
                        nvs_flash
                        esp_event)