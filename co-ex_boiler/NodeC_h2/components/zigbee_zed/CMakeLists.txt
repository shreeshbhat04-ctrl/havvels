idf_component_register(
    SRCS "zigbee_zed.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_timer espressif__esp-zigbee-lib
)