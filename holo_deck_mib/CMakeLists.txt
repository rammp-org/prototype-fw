# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# The shared RTPS spec (messages, topics, tables): the rammp-rtps git submodule.
set(RAMMP_RTPS_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../external/rammp-rtps/components")
if(NOT EXISTS "${RAMMP_RTPS_COMPONENTS}/rammp_rtps_messages")
  message(FATAL_ERROR "Missing the rammp-rtps submodule: run git submodule update --init")
endif()

# the sibling in-repo components (motor_actuator, holo_deck_platform) + the spec
set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../components"
  "${RAMMP_RTPS_COMPONENTS}"
)

set(
  COMPONENTS
  "main esptool_py esp_driver_twai esp32-p4-eth twai rtps cdr timer task cli logger format motor_actuator holo_deck_platform rammp_rtps_messages"
  CACHE STRING
  "List of components to include"
  )

project(holo_deck_mib)

set(CMAKE_CXX_STANDARD 20)
