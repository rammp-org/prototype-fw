idf_component_register(SRCS "main.cpp" "drive_controller.cpp" "linkage_controller.cpp" "rtps_link.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp32-p4-eth rtps cdr nvs timer task cli logger format
                                can_bus eyou_motor ds402_velocity_motor holo_deck_platform
                                rammp_rtps_messages esp_driver_gpio esp_timer)
