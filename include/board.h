#pragma once

#include <stdint.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-pedantic"
#include <mavlink/v1.0/mavlink_types.h>
#pragma GCC diagnostic pop

// serial
void init_serial(uint32_t baud);
void send_uart_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len);

// sensors
//bool init_imu();
//bool update_imu(float accel[3], float gyro[3], float *temperature);

// actuators


// memory
//bool init_memory(void);
//bool read_memory(void *dst, uint32_t len);
//bool write_memory(void *src, uint32_t len);
