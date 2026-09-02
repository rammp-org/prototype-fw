idf_component_register(SRCS "main.cpp" "mcp266_controller.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp32-p4-eth twai canopen mcp266 logger esp_driver_twai
                                esp_driver_gpio)

# Forward build-time feature flags (set e.g. `idf.py -DMIB_USE_ESPP_MCP266=1 build`)
# as compile definitions so the #if guards in main.cpp see them.
foreach(flag MIB_USE_ESPP_MCP266 MIB_SCAN_OD)
  if(DEFINED ${flag})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${flag}=${${flag}})
  endif()
endforeach()
