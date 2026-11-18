# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../components"
)

set(
  COMPONENTS
  "main esptool_py esp_driver_twai esp32-p4-eth"
  CACHE STRING
  "List of components to include"
  )

project(STEM)

set(CMAKE_CXX_STANDARD 20)

# ESP-IDF 6.1 added `on_vsync` to esp_lcd_dpi_panel_event_callbacks_t; the
# released espp esp32-p4-eth component (<= 1.3.1) only initializes the older
# fields, which -Werror=missing-field-initializers turns into a build failure.
# The omitted field is zero-initialized by the designated initializer, so this
# is warning-only. Remove once espp/esp32-p4-eth ships an IDF 6.1 fix.
idf_component_get_property(p4_eth_lib espp__esp32-p4-eth COMPONENT_LIB)
if(p4_eth_lib)
  target_compile_options(${p4_eth_lib} PRIVATE -Wno-error=missing-field-initializers)
endif()