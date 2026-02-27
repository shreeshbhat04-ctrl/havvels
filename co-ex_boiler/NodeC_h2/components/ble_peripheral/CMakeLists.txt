idf_component_register(
    SRCS "ble_peripheral.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_phy driver bt espressif__esp-zigbee-lib  # Add 'bt' and 'esp_phy'
)