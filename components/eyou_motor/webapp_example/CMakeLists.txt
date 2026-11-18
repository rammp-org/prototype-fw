# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# the sibling in-repo components: the Eyou motor and the CAN bus it rides on
set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../../"
)

# The espp/* and esp_tinyusb components come from the registry (main/idf_component.yml).
set(
  COMPONENTS
  "main esptool_py esp_driver_twai esp_driver_gpio esp_timer nvs_flash app_update espcoredump esp_psram can_bus eyou_motor nvs usb_device dispatcher stream_frame system monitor ota coredump task timer logger format base_component esp_tinyusb"
  CACHE STRING
  "List of components to include"
  )

project(eyou_webapp_example)

set(CMAKE_CXX_STANDARD 23)
