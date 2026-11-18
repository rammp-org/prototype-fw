cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../"
  "../../can_bus"
  "../../eyou_motor"
)

set(
  COMPONENTS
  "main can_bus eyou_motor paired_eyou_actuator espp__cli"
  CACHE STRING
  "List of components to include"
)

project(paired_eyou_actuator_example)

set(CMAKE_CXX_STANDARD 20)
