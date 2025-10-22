idf_component_register(
    SRCS "wifi.c" "pid.c" "main.c" "motor.c" "mpu.c"
    INCLUDE_DIRS "." "../include"
    REQUIRES driver esp_timer esp_wifi esp_event nvs_flash esp_netif lwip esp_system 
)
