# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# the sibling in-repo components (motor_actuator, holo_deck_platform)
set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../components"
)

set(
  COMPONENTS
  "main esptool_py esp_driver_twai m5stack-tab5 cli joystick adc timer holo_deck_platform"
  CACHE STRING
  "List of components to include"
  )

project(holo_deck)

set(CMAKE_CXX_STANDARD 20)
