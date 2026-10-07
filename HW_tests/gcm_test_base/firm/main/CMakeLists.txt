idf_component_register(
    SRCS "main.c"
         "inputs.c"
         "expander.c"
         "tof.c"
         "motors.c"
         "mini_json.c"
         "proto_format.c"
         "proto.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer pca9554 vl53l1x
)
