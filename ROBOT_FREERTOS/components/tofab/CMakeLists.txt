idf_component_register(
    SRCS "tofab.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer pca9554 vl53l1x
)
