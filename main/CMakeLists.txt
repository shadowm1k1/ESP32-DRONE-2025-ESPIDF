idf_component_register(SRCS "main.c" "motor.c" "mpu.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer)