#include "wifi.h"

const char *ssid = "Netvorkes";
const char *pass = "marka1234";
uint32_t retry_num = 0;
int temp_ks_int;


extern bool killswitch;
extern float baseThrottle;
extern float contthrottle,controll,contpitch,contyaw;
extern float rollp,rolli,rolld,pitchp,pitchi,pitchd,yawp,yawi,yawd;

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        
        //ESP_LOGI(WIFITAG, "WIFI CONNECTING....");
        esp_wifi_connect();
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        
        //ESP_LOGI(WIFITAG, "WiFi CONNECTED");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        
        //ESP_LOGE(WIFITAG, "WiFi lost connection");
        if (retry_num < 5)
        {
            esp_wifi_connect();
            retry_num++;
            //ESP_LOGI(WIFITAG, "Retrying to Connect...");
        }
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        //ESP_LOGI(WIFITAG, "Wifi got IP...");
        
        xTaskCreate(send_integers_continuously, "send_integers_continuously", 6144, NULL, 5, NULL);
        xTaskCreate(udp_receiver_task,"udp_recv", 6144, NULL, 5, NULL);
    }
}

void wifi_connection(void)
{   
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init(); // network interface initialization
    esp_event_loop_create_default(); // responsible for handling and dispatching events
    esp_netif_create_default_wifi_sta(); // sets up necessary data structs for WiFi station interface

    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT(); // sets up wifi_init_config struct with default values
    esp_wifi_init(&wifi_initiation); // WiFi initialized with default wifi_initiation

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL); // register WiFi events
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL); // register IP events

    wifi_config_t wifi_configuration = { 0 }; // zero initialize struct
    strcpy((char *)wifi_configuration.sta.ssid, ssid); // copy chars from hardcoded configs to struct
    strcpy((char *)wifi_configuration.sta.password, pass);

    esp_wifi_set_mode(WIFI_MODE_STA); // station mode selected
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration); // setting up configs
    esp_wifi_start(); // start WiFi
    
    
    //ESP_LOGI(WIFITAG, "wifi_init_softap finished. SSID:%s  password:%s\n", ssid, pass);
}

void send_integers_continuously(void *pvParameters)
{
    const char *dst_ip   = "10.25.132.81";   // PC / drone IP
    const uint16_t dst_port = 8080;

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(dst_port),
        .sin_addr.s_addr = inet_addr(dst_ip)
    };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        //ESP_LOGE(WIFITAG, "Unable to create UDP socket");
        vTaskDelete(NULL);
    }

    char msg[128];
    while (1) {
        snprintf(msg, sizeof(msg), "pitch: %.2f , roll: %.2f , yaw: %.2f, m0: %.2f, m1: %.2f , m2: %.2f, m3: %.2f",
                 angles.pitch, angles.roll ,angles.yaw,m0,m1,m2,m3);
        int err = sendto(sock, msg, strlen(msg), 0,
                         (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            
        //ESP_LOGE(WIFITAG, "UDP send error:  %d", errno);
        } else {
            //printf("Sent UDP: %s\n", msg);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void udp_receiver_task(void *pvParameters)
{
    const uint16_t listen_port = 8081;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        
        //ESP_LOGE(WIFITAG, "Unable to create UDP receiver socket");
        vTaskDelete(NULL);
    }

    struct sockaddr_in dest_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(listen_port),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        
        //ESP_LOGE(WIFITAG, "UDP bind failed");
        
        close(sock);
        vTaskDelete(NULL);
    }

    char rx_buffer[256];
    while (1) {
        struct sockaddr_in6 source_addr; // large enough for both IPv4/6
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            
            //ESP_LOGE(WIFITAG, "Failed to recive data");
            break;
        }
        rx_buffer[len] = 0;
        //printf("Received UDP: %s\n", rx_buffer);  <----raw udp

        int count = sscanf(rx_buffer, "%d,%f,%f, %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &temp_ks_int,&contthrottle,&controll,&contpitch,&contyaw, &rollp, &rolli, &rolld, &pitchp, &pitchi, &pitchd, &yawp, &yawi, &yawd, &baseThrottle);
        killswitch = (temp_ks_int != 0);

        if (count == 15) {
            //printf("Received floats: %d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",temp_ks_int ,contthrottle, controll, contpitch, contyaw,  rollp, rolli, rolld, pitchp, pitchi, pitchd, yawp, yawi, yawd);
        } else {
            
            //ESP_LOGE(WIFITAG, "Error parsing floats!");
        }
        
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    close(sock);
    vTaskDelete(NULL);
}
