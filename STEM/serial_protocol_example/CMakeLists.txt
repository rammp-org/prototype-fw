cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../../components"
)

set(
  COMPONENTS
  "main esptool_py esp_driver_twai esp_driver_uart"
  CACHE STRING
  "List of components to include"
)

project(STEM_SERIAL)

set(CMAKE_CXX_STANDARD 20)
