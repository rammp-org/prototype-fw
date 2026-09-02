idf_component_register(SRCS "main.cpp" "ik_5bar.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES motor_actuator paired_actuator cli)