idf_component_register(SRCS "pid.c" "main.c" "motor.c" "mpu.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer)