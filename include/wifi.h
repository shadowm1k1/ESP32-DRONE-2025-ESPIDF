#ifndef _WIFI_H_
#define _WIFI_H_

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_timer.h"

/* ---------- public API ---------- */
esp_err_t wifi_start(void);

/* ---------- ports ---------- */
enum {
    UDP_TELEM_PORT = 8080,   /* drone → PC binary telemetry */
    UDP_RX_PORT    = 8081,   /* PC  → drone control */
    UDP_ERROR_PORT = 8082    /* drone → PC text errors   */
};

/* ---------- error codes ---------- */
enum {
    ERR_NONE = 0, 
    ERR_LOW_STACK_TX,
    ERR_LOW_STACK_RX,
    ERR_UDP_TX_SOCK,
    ERR_UDP_RX_SOCK,
};

/* ---------- packed telemetry frame ---------- */
typedef struct __attribute__((packed)) {
    float r_p, r_r, r_y;   /* rates  */
    float m0, m1, m2, m3;  /* motors */
    float a_p, a_r, a_y;   /* angles */
    uint8_t err_id;        /* 1-byte code */
} tel_t;

/* ---------- whitelist ---------- */
#define TRUSTED_IP  "10.136.12.81"

/* ---------- task entries ---------- */
void send_integers_continuously(void *pvParameters);
void udp_receiver_task          (void *pvParameters);
void send_errors_task           (void *pvParameters);

#endif