#ifndef RC_H_
#define RC_H_

#include <breezystm32/breezystm32.h>

// new
#include <stdbool.h>

#define RC_MAX_CHANNELS 8

// raw RC inputs
extern uint16_t _rc_raw[RC_MAX_CHANNELS];

// scaled values
extern float _rc_chan_x; // -1.0 to 1.0
extern float _rc_chan_y; // -1.0 to 1.0
extern float _rc_chan_z; // -1.0 to 1.0
extern float _rc_chan_F; //  0.0 to 1.0

// switch values
extern bool _rc_switch_armed;
extern bool _rc_switch_attitude_override;
extern bool _rc_switch_throttle_override;
extern bool _rc_switch_attitude_type;
extern bool _rc_switch_F_type;

typedef enum {
  RC_STICK_X,
  RC_STICK_Y,
  RC_STICK_Z,
  RC_STICK_F,
  NUM_RC_STICKS
} rc_stick_t;

typedef enum {
  RC_SWITCH_ARMED,
  RC_SWITCH_ATTITUDE_OVERRIDE,
  RC_SWITCH_THROTTLE_OVERRIDE,
  RC_SWITCH_ATTITUDE_TYPE,
  RC_SWITCH_F_TYPE,
  NUM_RC_SWITCHES
} rc_switch_t;

extern float _rc_stick[NUM_RC_STICKS];
extern bool _rc_switch[NUM_RC_SWITCHES];
extern bool _rc_switch_assigned[NUM_RC_SWITCHES];

// old
#include "mux.h"

//typedef struct
//{
//  int16_t channel;
//  int16_t direction;
//} rc_switch_t;

typedef enum
{
  PARALLEL_PWM,
  CPPM,
} rc_type_t;

extern bool _calibrate_rc;
void init_rc(void);
bool rc_switch(int16_t channel);
bool receive_rc(uint32_t now);

#endif
