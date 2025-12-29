# Copyright 2025 Zorxx Software. All rights reserved.
if(IDF_TARGET)
    idf_component_register(SRCS "lib/ina219.c" "lib/esp-idf.c"
                           INCLUDE_DIRS "lib" "include"
                           PRIV_INCLUDE_DIRS "lib" "include/ina219"
                           PRIV_REQUIRES "driver" "esp_timer")
    return()
endif()

cmake_minimum_required(VERSION 3.5)
set(project ina219)
project(${project} LANGUAGES C VERSION 1.0.0)

add_library(ina219 STATIC lib/ina219.c lib/linux.c)
target_include_directories(ina219 PUBLIC include)
target_include_directories(ina219 PRIVATE lib include/ina219)
install(TARGETS ina219 LIBRARY DESTINATION lib)
install(DIRECTORY include/ina219 DESTINATION include)

add_subdirectory(example/linux)
