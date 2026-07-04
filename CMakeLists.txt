# Copyright 2025-2026 Zorxx Software. All rights reserved.
if(IDF_TARGET)
    # Starting from esp-idf v5.3, the peripheral drivers are in separate components
    list(APPEND priv_requires "esp_timer")
    if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
        list(APPEND priv_requires "esp_driver_i2c")
    else()
        list(APPEND priv_requires "driver")
    endif()
    idf_component_register(SRCS "lib/ina219.c" "lib/esp-idf.c"
                           INCLUDE_DIRS "lib" "include"
                           PRIV_INCLUDE_DIRS "lib" "include/ina219"
                           PRIV_REQUIRES ${priv_requires})
    return()
endif()

cmake_minimum_required(VERSION 3.5)
set(project ina219)
project(${project} LANGUAGES C VERSION 1.0.1)

add_library(ina219 STATIC lib/ina219.c lib/linux.c)
target_include_directories(ina219 PUBLIC include)
target_include_directories(ina219 PRIVATE lib include/ina219)
install(TARGETS ina219 LIBRARY DESTINATION lib)
install(DIRECTORY include/ina219 DESTINATION include)

add_subdirectory(example/linux)
