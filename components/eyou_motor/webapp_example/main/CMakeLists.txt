idf_component_register(SRCS "eyou_webapp_example.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES eyou_motor can_bus nvs usb_device dispatcher stream_frame system monitor ota coredump task logger format base_component)
