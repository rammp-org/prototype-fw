idf_component_register(SRCS "main.cpp" "holo_deck_controller.cpp" "twai_motor_bus.cpp" "rtps_link.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp32-p4-eth twai rtps cdr timer task cli logger format
                                motor_actuator holo_deck_platform rammp_rtps_messages
                                esp_driver_gpio)
