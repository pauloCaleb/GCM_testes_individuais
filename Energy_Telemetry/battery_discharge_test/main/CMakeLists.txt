idf_component_register(
    SRCS "main.c" "cal_store.c" "flash_log.c" "bt_link.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_system esp_rom esp_partition spi_flash nvs_flash bt
)
