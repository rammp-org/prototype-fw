# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# The shared RTPS spec (messages, topics, tables): the rammp-rtps git submodule.
# Until the PEAR extension lands upstream it needs spec/rammp-rtps-pear.patch
# applied to the submodule working copy (see README.md).
set(RAMMP_RTPS_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../external/rammp-rtps/components")
if(NOT EXISTS "${RAMMP_RTPS_COMPONENTS}/rammp_rtps_messages")
  message(FATAL_ERROR "Missing the rammp-rtps submodule: run git submodule update --init")
endif()

# the sibling in-repo components (can_bus, eyou_motor, ds402_velocity_motor,
# holo_deck_platform) + the spec
set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../components"
  "${RAMMP_RTPS_COMPONENTS}"
)

set(
  COMPONENTS
  "main esptool_py esp_driver_twai esp_driver_gpio esp_timer esp32-p4-eth rtps cdr nvs timer task cli logger format can_bus eyou_motor ds402_velocity_motor holo_deck_platform rammp_rtps_messages"
  CACHE STRING
  "List of components to include"
  )

project(pear)

set(CMAKE_CXX_STANDARD 20)
