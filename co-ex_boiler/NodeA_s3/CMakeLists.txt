# Top-level CMakeLists for NodeA_s3
cmake_minimum_required(VERSION 3.16)

# Extra component directories (sibling folders to main/)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/wifi_ap"
    "${CMAKE_CURRENT_LIST_DIR}/ble_central"
    "${CMAKE_CURRENT_LIST_DIR}/tcp_server"
    "${CMAKE_CURRENT_LIST_DIR}/coex_packet"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(NodeA_s3)
