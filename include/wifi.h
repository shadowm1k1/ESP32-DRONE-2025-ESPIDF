#pragma once
#include <stdio.h>  // for basic printf commands
#include <string.h> // for handling strings
#include "freertos/FreeRTOS.h" // for delay, mutexes, semaphores, RTOS operations
#include "esp_system.h" // esp_init functions esp_err_t 
#include "esp_wifi.h" // esp_wifi_init functions and WiFi operations
#include "esp_log.h" // for showing logs
#include "esp_event.h" // for WiFi events
#include "nvs_flash.h" // non-volatile storage
#include "lwip/err.h" // lightweight IP packets error handling
#include "lwip/sys.h" // system applications for lightweight IP apps
#include "lwip/sockets.h"   // for creating and managing TCP/UDP sockets (send/receive data over WiFi)
#include "lwip/netdb.h"     // for network database operations like hostname/IP lookup and address handling
#include "mpu.h" 

extern const char *ssid;
extern const char *pass;
extern uint32_t retry_num;


extern mpu_angles_t angles; // Initialize filtered angles

void wifi_connection(void);
void send_integers_continuously(void *pvParameters);

