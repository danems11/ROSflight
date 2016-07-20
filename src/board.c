#include <breezystm32/breezystm32.h>

#include "board.h"

#define BOARD_REV 2

#ifndef BOARD_REV
#error Must specify BOARD_REV
#endif

serialPort_t *Serial1;

void init_serial(uint32_t baud)
{
  Serial1 = uartOpen(USART1, NULL, baud, MODE_RXTX);
}

void send_uart_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len)
{
  if (chan == MAVLINK_COMM_0)
  {
    for (uint16_t i = 0; i < len; i++)
    {
      serialWrite(Serial1, (uint8_t)buf[i]);
    }
  }
}
