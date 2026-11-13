idf_component_register(SRCS "bench.c" "bench_io.c"
    INCLUDE_DIRS "." "../../foc/main"
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_adc esp_timer console)
