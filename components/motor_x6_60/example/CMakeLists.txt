cmake_minimum_required(VERSION 3.20)

set(IDF_TARGET esp32p4 CACHE STRING "ESP-IDF target")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../"
  "../../motor_actuator/example/managed_components"
)

set(
  COMPONENTS
  "main motor_x6_60 motor_actuator esp_driver_twai espp__cli"
  CACHE STRING
  "List of components to include"
)

project(motor_x6_60_example)

set(CMAKE_CXX_STANDARD 20)