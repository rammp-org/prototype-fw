idf_component_register(SRCS "main.cpp" "mcp266_controller.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp32-p4-eth twai canopen logger esp_driver_twai esp_driver_gpio)
