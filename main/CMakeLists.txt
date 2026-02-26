idf_component_register(
    SRCS "flow.c" "pid.c" "wifi.c" "main.c" "motor.c" "mpu.c"
    INCLUDE_DIRS "." "../include"
    REQUIRES driver esp_timer esp_wifi esp_event nvs_flash esp_netif lwip esp_system 
)
