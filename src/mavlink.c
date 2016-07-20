#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "mavlink_receive.h"
#include "param.h"

#include "mavlink.h"

// global variable definitions
mavlink_system_t mavlink_system;

// function definitions
void init_mavlink(void)
{
  // Initialize Serial ports
  mavlink_system.sysid = _params.values[PARAM_SYSTEM_ID];
  mavlink_system.compid = 250;

  _offboard_control_time = 0;
}
