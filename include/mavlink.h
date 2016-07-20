#pragma once

#include "board.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-pedantic"
#include <mavlink/v1.0/mavlink_types.h>
#pragma GCC diagnostic pop

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES send_uart_bytes
extern mavlink_system_t mavlink_system;

// this needs to be include after the above declarations
#include <mavlink/v1.0/rosflight/mavlink.h>

// function declarations
void init_mavlink(void);
