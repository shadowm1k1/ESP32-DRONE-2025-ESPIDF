#include "flow.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#define TAG "FLOW"

#define FLOW_UART_NUM       UART_NUM_1
#define FLOW_UART_BAUD_RATE 19200
#define FLOW_UART_TX_PIN    41
#define FLOW_UART_RX_PIN    40
#define FLOW_BUF_SIZE       256

#define PACKET_HEADER_1     0xFE
#define PACKET_HEADER_2     0x04
#define PACKET_END          0xAA
#define PACKET_SIZE         11

static flow_data_t flow_data = {0};

static uint8_t rx_buffer[FLOW_BUF_SIZE];
static uint8_t parse_buffer[PACKET_SIZE];
static uint8_t parse_idx = 0;
static uint8_t header_found = 0;


/* ================= UART INIT ================= */

void flow_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = FLOW_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(FLOW_UART_NUM, FLOW_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(FLOW_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(FLOW_UART_NUM,
                                 FLOW_UART_TX_PIN,
                                 FLOW_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "Flow UART initialized");
}


/* ================= PACKET PARSER ================= */

static void flow_parse_byte(uint8_t byte)
{
    if (!header_found) {
        if (byte == PACKET_HEADER_1) {
            parse_buffer[0] = byte;
            parse_idx = 1;
            header_found = 1;
        }
    } else {

        parse_buffer[parse_idx++] = byte;

        if (parse_idx == 2 && byte != PACKET_HEADER_2) {
            header_found = 0;
            parse_idx = 0;
            return;
        }

        if (parse_idx >= PACKET_SIZE) {

            if (parse_buffer[10] == PACKET_END) {

                uint8_t calc_sum = 0;
                for (int i = 2; i <= 7; i++) {
                    calc_sum += parse_buffer[i];
                }

                if (calc_sum == parse_buffer[8]) {

                    flow_data.flow_x =
                        (int16_t)((parse_buffer[3] << 8) | parse_buffer[2]);

                    flow_data.flow_y =
                        (int16_t)((parse_buffer[5] << 8) | parse_buffer[4]);

                    flow_data.height =
                        (int16_t)((parse_buffer[7] << 8) | parse_buffer[6]);

                    flow_data.quality = parse_buffer[9];

                    flow_data.total_x -= flow_data.flow_x;
                    flow_data.total_y -= flow_data.flow_y;

                    flow_data.data_valid = 1;
                }
            }

            header_found = 0;
            parse_idx = 0;
        }
    }
}

void flow_reset_val(void)
{
    flow_data.flow_x = 0;
    flow_data.flow_y = 0;
    flow_data.height = 0;
    flow_data.quality = 0;
    flow_data.total_x = 0;
    flow_data.total_y = 0;
    flow_data.data_valid = 0;
}

/* ================= UPDATE FUNCTION ================= */

void flow_update(void)
{
    int len = uart_read_bytes(FLOW_UART_NUM,
                              rx_buffer,
                              sizeof(rx_buffer),
                              0);

    for (int i = 0; i < len; i++) {
        flow_parse_byte(rx_buffer[i]);
    }
}


/* ================= GETTER FUNCTION ================= */

bool flow_get_data(flow_data_t *out_data)
{
    if (flow_data.data_valid) {
        *out_data = flow_data;
        flow_data.data_valid = 0;
        return true;
    }
    return false;
}
