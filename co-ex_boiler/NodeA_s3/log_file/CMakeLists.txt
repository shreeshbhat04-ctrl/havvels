idf_component_register(
    SRCS "log_file.c"
    INCLUDE_DIRS "."
    REQUIRES spiffs esp_partition spi_flash
    PRIV_REQUIRES esp_timer freertos
)
