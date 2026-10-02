idf_component_register(
    SRCS "main.c" "motor.c" "dabble_gamepad.c" "ble_gamepad.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash driver esp_timer
)
