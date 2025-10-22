#include "wifi.h"

const char *ssid = "Netvorkes";
const char *pass = "marka1234";
uint32_t retry_num = 0;

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        printf("WIFI CONNECTING....\n");
        esp_wifi_connect();
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        printf("WiFi CONNECTED\n");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("WiFi lost connection\n");
        if (retry_num < 5)
        {
            esp_wifi_connect();
            retry_num++;
            printf("Retrying to Connect...\n");
        }
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        printf("Wifi got IP...\n\n");
        xTaskCreate(send_integers_continuously, "send_integers_continuously", 4096, NULL, 5, NULL);
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

    printf("wifi_init_softap finished. SSID:%s  password:%s\n", ssid, pass);
}

void send_integers_continuously(void *pvParameters)
{
    const char *pc_ip = "10.42.68.163";
    const int pc_port = 8080;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(pc_ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(pc_port);

    while (1)
    {
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0)
        {
            printf("Unable to create socket: errno %d\n", errno);
            vTaskDelay(pdMS_TO_TICKS(2000)); // wait before retrying
            continue;
        }

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0)
        {
            printf("Socket unable to connect: errno %d\n", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        printf("Connected to PC at %s:%d\n", pc_ip, pc_port);

        // Now continuously send two integers every second
        while (1)
        {

            char message[32];
            snprintf(message, sizeof(message), "pitch: %f, roll: %f", angles.pitch, angles.roll);
            

            int err_send = send(sock, message, strlen(message), 0);
            if (err_send < 0)
            {
                printf("Error sending data: errno %d\n", errno);
                break; // connection lost → break inner loop and reconnect
            }
            else
            {
                printf("Sent: %s\n", message);
            }

            vTaskDelay(pdMS_TO_TICKS(100)); // send every 0.1 second
        }

        close(sock);
        printf("Reconnecting...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}