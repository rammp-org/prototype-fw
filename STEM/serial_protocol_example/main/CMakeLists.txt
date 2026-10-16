idf_component_register(
  SRCS "main.cpp" "../../main/ik_5bar.cpp"
  INCLUDE_DIRS "." "../../main"
  REQUIRES motor_actuator paired_actuator esp_driver_uart
)
