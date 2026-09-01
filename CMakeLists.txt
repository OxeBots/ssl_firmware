cmake_minimum_required(VERSION 3.16.0)

# Vendor libraries live inside the embedded-drivers submodule; register them
# as IDF components so their Kconfig options and include dirs stay available.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/embedded-drivers/esp-idf-mirf"
    "${CMAKE_CURRENT_LIST_DIR}/components/embedded-drivers/VL53L5CX-Library"
    ${EXTRA_COMPONENT_DIRS})
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ssl_firmware)
