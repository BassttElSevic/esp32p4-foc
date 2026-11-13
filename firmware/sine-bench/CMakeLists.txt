cmake_minimum_required(VERSION 3.16)
# Use the drive project's P4, UART0, flash-size and RTOS tick defaults.
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../foc/sdkconfig.defaults")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4_sine_bench)
