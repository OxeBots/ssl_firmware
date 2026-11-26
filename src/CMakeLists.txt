if(CONFIG_FIRMWARE_ROLE_BASE_STATION)
    set(app_sources "${CMAKE_SOURCE_DIR}/src/base_station_main.cpp")
else()
    set(app_sources "${CMAKE_SOURCE_DIR}/src/main.cpp")
endif()

idf_component_register(SRCS ${app_sources})
