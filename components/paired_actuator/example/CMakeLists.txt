cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../"
  "../../motor_actuator"
)

set(
  COMPONENTS
  "main paired_actuator motor_actuator esp_driver_twai espp__cli"
  CACHE STRING
  "List of components to include"
)

project(paired_actuator_example)

set(CMAKE_CXX_STANDARD 20)
