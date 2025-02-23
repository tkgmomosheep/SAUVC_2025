idf_component_register(SRCS "main.cpp"
                    PRIV_REQUIRES spi_flash
                    REQUIRES MS5837
                    REQUIRES LEDControl
                    REQUIRES esp_driver_ledc
                    REQUIRES driver
                    INCLUDE_DIRS "")
