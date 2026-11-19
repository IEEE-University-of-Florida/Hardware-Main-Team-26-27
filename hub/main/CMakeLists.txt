idf_component_register(
    SRCS
        "../src/main.cpp"
        "../src/otos_localization.cpp"
        "../src/otos_esp_idf_bus.cpp"
        "../otos_driver/src/otos.cpp"
    INCLUDE_DIRS "../include" "../otos_driver/src"
    PRIV_REQUIRES esp_driver_i2c esp_timer
)
