cmake_minimum_required(VERSION 3.12)

# Set board type for RP2350 (e.g. pico2)
set(PICO_BOARD seeed_xiao_rp2350 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform type")

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(rpauto C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.2.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.2.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

add_compile_options(-Wall)

if (PICO_PLATFORM STREQUAL "host")
  add_subdirectory(test)
else ()
  add_subdirectory(src)
endif ()
