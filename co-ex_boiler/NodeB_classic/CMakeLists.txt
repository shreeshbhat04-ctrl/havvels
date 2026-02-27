# Top-level CMakeLists for NodeB_classic
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/wifi_sta"
    "${CMAKE_CURRENT_LIST_DIR}/ble_peripheral"
    "${CMAKE_CURRENT_LIST_DIR}/tcp_client"
    "${CMAKE_CURRENT_LIST_DIR}/coex_packet"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(NodeB_classic)
