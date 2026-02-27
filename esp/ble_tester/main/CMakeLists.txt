idf_component_register(SRCS "main.c" "../../../main/coex_packet.c"
                    INCLUDE_DIRS "." "../../../main"
                    REQUIRES nvs_flash bt esp_timer)
