idf_component_register(SRCS "main.cpp" "ik_5bar.cpp" "stem_controller.cpp" "stem_module.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES motor_actuator paired_actuator cli esp32-p4-eth
                                coredump dispatcher ota stream_frame task usb_device
                                esp_tinyusb espcoredump app_update esp_timer)
