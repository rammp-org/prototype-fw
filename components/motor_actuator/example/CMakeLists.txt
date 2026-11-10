cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../"
  "../../can_bus"
)

set(
  COMPONENTS
  "main can_bus motor_actuator espp__cli"
  CACHE STRING
  "List of components to include"
)

project(motor_actuator_example)

set(CMAKE_CXX_STANDARD 20)