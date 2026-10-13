idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp32-p4-eth twai canopen mcp266 logger esp_driver_twai
                                esp_driver_gpio)

# Forward the optional MIB_SCAN_OD build flag (set e.g. `idf.py -DMIB_SCAN_OD=1 build`)
# as a compile definition so the #if guard in main.cpp sees it.
if(DEFINED MIB_SCAN_OD)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE MIB_SCAN_OD=${MIB_SCAN_OD})
endif()
