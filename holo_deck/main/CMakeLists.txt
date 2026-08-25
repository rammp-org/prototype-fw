idf_component_register(SRCS "main.cpp" "gui.cpp" "holo_deck_controller.cpp" "joystick_input.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES motor_actuator cli holo_deck_platform m5stack-tab5 joystick adc
                                math timer task esp_adc esp_driver_gpio)
