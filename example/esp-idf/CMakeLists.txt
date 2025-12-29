# Copyright 2025 Zorxx Software
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../..)
add_compile_definitions(SYS_DEBUG_ENABLE=1)
project(bmp180)
