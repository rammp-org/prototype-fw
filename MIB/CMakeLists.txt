# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../components"
)

set(
  COMPONENTS
  "main esptool_py esp_driver_twai esp32-p4-eth twai canopen mcp266 logger"
  CACHE STRING
  "List of components to include"
  )

project(MIB)

set(CMAKE_CXX_STANDARD 20)
