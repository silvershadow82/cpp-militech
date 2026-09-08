idf_component_register(SRCS "main.cpp" "zones.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES buzzer esp_driver_gpio esp_driver_gptimer esp_driver_uart esp_timer)
