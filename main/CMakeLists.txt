set(common_requires esp_timer nvs_flash esp_event)

if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND common_requires esp_wifi)
endif()

if(IDF_TARGET STREQUAL "esp32h2")
    list(APPEND common_requires ieee802154)
endif()

if(IDF_TARGET STREQUAL "esp32")
    list(APPEND common_requires bt)
endif()

idf_component_register(SRCS "main.c" "coex_packet.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${common_requires})
