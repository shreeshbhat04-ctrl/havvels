idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES ble_peripheral zigbee_zed
                    PRIV_REQUIRES nvs_flash bt)

