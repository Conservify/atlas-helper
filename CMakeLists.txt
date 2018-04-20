project(ARDUINO_PROJECTS)
cmake_minimum_required(VERSION 2.6)

set(ARDUINO_IDE "${CMAKE_CURRENT_SOURCE_DIR}/../../conservify/arduino-1.8.3")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../../conservify/arduino-cmake-minimal/cmake/")

include(ArduinoMinimal)

add_subdirectory(firmware)
