#ifndef FLOW_H
#define FLOW_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t flow_x;
    int16_t flow_y;
    int16_t height;
    uint8_t quality;
    int32_t total_x;
    int32_t total_y;
    uint8_t data_valid;
} flow_data_t;

void flow_uart_init(void);
void flow_update(void);
void flow_reset_val(void);
bool flow_get_data(flow_data_t *out_data);

#endif