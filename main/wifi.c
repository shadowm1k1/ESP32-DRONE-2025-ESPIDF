#include "wifi.h"
#include "mpu.h" //   ----->   /* angles, rates, m0…m3 */

/*----- wifi socks and data -------*/
static const char *WIFI_SSID = "Netvorkes";
static const char *WIFI_PASS = "marka1234";

static uint8_t retry_cnt = 0;
static int udp_tx_sock = -1;
static int udp_rx_sock = -1;
static int udp_err_sock = -1;


/*------ variables from main to be chagned --------*/
extern float m0, m1, m2, m3;
extern mpu_angles_t angles;
extern mpu_rates_t  rates;

extern float baseThrottle, contthrottle, controll, contpitch, contyaw;
extern float rollp, rolli, rolld, pitchp, pitchi, pitchd, yawp, yawi, yawd;
extern volatile bool killswitch;


/* ---------- circular error buffer ---------- */
#define ERR_BUF_LEN 40
static char err_buf[ERR_BUF_LEN] = "OK";
static SemaphoreHandle_t err_mux;

/* ---------- event handler ---------- */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        udp_tx_sock = -1;
        udp_rx_sock = -1;
        udp_err_sock = -1;
        if (retry_cnt < 5) {
            esp_wifi_connect();
            retry_cnt++;
        }
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        retry_cnt = 0;
        if (udp_tx_sock < 0)
            xTaskCreate(send_integers_continuously, "udp_tx", 4096, NULL, 5, NULL);
        if (udp_rx_sock < 0)
            xTaskCreate(udp_receiver_task,          "udp_rx", 4096, NULL, 5, NULL);
        if (udp_err_sock < 0)
            xTaskCreate(send_errors_task,           "udp_err", 4096, NULL, 5, NULL);
    }
}

/* ---------- Wi-Fi init ---------- */
esp_err_t wifi_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,&wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wc = { 0 };
    strcpy((char *)wc.sta.ssid, WIFI_SSID);
    strcpy((char *)wc.sta.password, WIFI_PASS);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    err_mux = xSemaphoreCreateMutex();
    return ESP_OK;
}

/* ---------- 5 Hz telemetry ---------- */
void send_integers_continuously(void *pvParameters)
{
    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_port   = htons(UDP_TELEM_PORT),
        .sin_addr.s_addr = inet_addr(TRUSTED_IP)
    };

    udp_tx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_tx_sock < 0) return;

    static tel_t t;
    TickType_t xLastWake = xTaskGetTickCount();

    while (1) {
        /* pack data */
        t.r_p = rates.rate_pitch;
        t.r_r = rates.rate_roll;
        t.r_y = rates.rate_yaw;
        t.m0  = m0;
        t.m1  = m1;
        t.m2  = m2;
        t.m3  = m3;
        t.a_p = angles.pitch;
        t.a_r = angles.roll;
        t.a_y = angles.yaw;

        /* error code */
        t.err_id = ERR_OK;
        if (uxTaskGetStackHighWaterMark(NULL) < 100)      t.err_id = ERR_LOW_STACK_TX;
        else if (udp_tx_sock < 0)                           t.err_id = ERR_UDP_TX_SOCK;

        sendto(udp_tx_sock, &t, sizeof(t), 0, (struct sockaddr *)&dst, sizeof(dst));
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(50));   /* 20 Hz */
    }
}

/* ---------- 5 Hz human-readable errors ---------- */
void send_errors_task(void *pvParameters)
{
    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_port   = htons(UDP_ERROR_PORT),
        .sin_addr.s_addr = inet_addr(TRUSTED_IP)
    };

    udp_err_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_err_sock < 0) return;

    static char msg[ERR_BUF_LEN];
    TickType_t xLastWake = xTaskGetTickCount();

    while (1) {
        xSemaphoreTake(err_mux, portMAX_DELAY);
        strncpy(msg, err_buf, sizeof(msg));
        xSemaphoreGive(err_mux);

        sendto(udp_err_sock, msg, strlen(msg), 0, (struct sockaddr *)&dst, sizeof(dst));
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(200));   /* 5 Hz */
    }
}

void udp_receiver_task(void *pvParameters)
{
    struct sockaddr_in server = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_RX_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    udp_rx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_rx_sock < 0) return;
    if (bind(udp_rx_sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
        close(udp_rx_sock);
        udp_rx_sock = -1;
        return;
    }

    static struct {
        int   ks;
        float throttle, roll, pitch, yaw;
        float rp, ri, rd, pp, pi, pd, yp, yi, yd, base;
    } rx;

    char buf[300];
    while (1) {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int len = recvfrom(udp_rx_sock, buf, sizeof(buf)-1, 0, (struct sockaddr *)&from, &flen);
        if (len <= 0) continue;
        buf[len] = 0;
        if (from.sin_addr.s_addr != inet_addr(TRUSTED_IP)) continue;

        int n = sscanf(buf, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                       &rx.ks, &rx.throttle, &rx.roll, &rx.pitch, &rx.yaw,
                       &rx.rp, &rx.ri, &rx.rd,
                       &rx.pp, &rx.pi, &rx.pd,
                       &rx.yp, &rx.yi, &rx.yd, &rx.base);

        if (n == 15) {
            killswitch   = (rx.ks != 0);
            contthrottle = rx.throttle;
            controll     = rx.roll;
            contpitch    = rx.pitch;
            contyaw      = rx.yaw;
            rollp = rx.rp; rolli = rx.ri; rolld = rx.rd;
            pitchp = rx.pp; pitchi = rx.pi; pitchd = rx.pd;
            yawp = rx.yp; yawi = rx.yi; yawd = rx.yd;
            baseThrottle = rx.base;
        } else {
            /* bad frame – zero everything */
            killswitch = false;
            xSemaphoreTake(err_mux, portMAX_DELAY);
            snprintf(err_buf, sizeof(err_buf), "BAD_FRAME");
            xSemaphoreGive(err_mux);
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}