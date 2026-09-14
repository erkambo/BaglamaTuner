idf_component_register(SRCS "adc_cal.c" "hello_world_main.c"
                       PRIV_REQUIRES spi_flash esp_adc
                       INCLUDE_DIRS "")
