cmake_minimum_required(VERSION 3.16.0)

# Vendor libraries live inside the embedded-drivers submodule; register them
# as IDF components so their Kconfig options and include dirs stay available.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/embedded-drivers/esp-idf-mirf"
    "${CMAKE_CURRENT_LIST_DIR}/components/embedded-drivers/VL53L5CX-Library"
    ${EXTRA_COMPONENT_DIRS})
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
