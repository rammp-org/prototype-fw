idf_component_register(SRCS "main.cpp" "five_bar.cpp" "linkage_controller.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES eyou_motor can_bus nvs nvs_flash usb_device dispatcher stream_frame system monitor ota coredump task logger format base_component)
