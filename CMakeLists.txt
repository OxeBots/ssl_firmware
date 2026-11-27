cmake_minimum_required(VERSION 3.16.0)

if(SSL_ESP_LINK)
    set(EXCLUDE_COMPONENTS QuickPID sTune)
else()
    # Register the robot drivers as components for their Kconfig options.
    list(APPEND EXTRA_COMPONENT_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/components/embedded-drivers/esp-idf-mirf"
        "${CMAKE_CURRENT_LIST_DIR}/components/embedded-drivers/VL53L5CX-Library")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Coverage builds (env test_coverage) instrument project sources with gcov.
# -fprofile-filter-files keeps vendor components out of the counters so the
# app image and the report stay small. Set by scripts/coverage_flags.py.
if(DEFINED ENV{COVERAGE_BUILD})
    idf_build_set_property(COMPILE_OPTIONS "--coverage" APPEND)
    # Restrict counters to project sources; the regex must stay free of shell
    # metacharacters because PlatformIO replays it into a shell when compiling
    # src/ outside of ninja.
    idf_build_set_property(COMPILE_OPTIONS
        "-fprofile-filter-files=/ssl_firmware/" APPEND)
    idf_build_set_property(LINK_OPTIONS "--coverage" APPEND)
endif()

project(ssl_firmware)

# The -O2 performance build (encoder acquisition benchmark) exposes a
# pre-existing uninitialized-member warning inside the vendored sTune component.
# components/sTune is a submodule and is left untouched, so keep that warning
# visible but not fatal for the performance build.
if(NOT SSL_ESP_LINK)
    idf_build_set_property(COMPILE_OPTIONS "-Wno-error=uninitialized" APPEND)
endif()
