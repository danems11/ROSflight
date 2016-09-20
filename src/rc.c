#include <stdint.h>
#include <stdbool.h>

#include <breezystm32/breezystm32.h>

#include "param.h"
#include "rc.h"
#include "mux.h"
#include "mode.h"

#include "mavlink_util.h"
#include "mavlink_log.h"

//***************************
// NEW
//***************************

float _rc_stick[NUM_RC_STICKS];
bool _rc_switch[NUM_RC_SWITCHES];
bool _rc_switch_assigned[NUM_RC_SWITCHES];

static struct
{
  uint8_t channel;
  uint16_t center;
  uint16_t range;
  bool positive; // true if 0 to 1 instead of -1 to 1
} sticks[NUM_RC_STICKS];

static struct
{
  bool assigned;
  uint8_t channel;
  int8_t direction;
} switches[NUM_RC_SWITCHES];

//***************************

//static rc_switch_t switches[NUM_RC_SWITCH];

bool _calibrate_rc;
void calibrate_rc();

void init_rc()
{
  _calibrate_rc = false;
  _rc_control.x.type = ANGLE;
  _rc_control.y.type = ANGLE;
  _rc_control.z.type = RATE;
  _rc_control.F.type = THROTTLE;

  _rc_control.x.value = 0;
  _rc_control.y.value = 0;
  _rc_control.z.value = 0;
  _rc_control.F.value = 0;

  _offboard_control.x.active = false;
  _offboard_control.y.active = false;
  _offboard_control.z.active = false;
  _offboard_control.F.active = false;

  switches[0].channel = 4;
  switches[0].direction = get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
  switches[1].channel = 5;
  switches[1].direction = get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
  switches[2].channel = 6;
  switches[2].direction = get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
  switches[3].channel = 7;
  switches[3].direction = get_param_int(PARAM_RC_SWITCH_8_DIRECTION);

  //***************************
  // NEW
  //***************************

  sticks[RC_STICK_X].channel = (uint8_t) get_param_int(PARAM_RC_X_CHANNEL);
  sticks[RC_STICK_X].center = (uint16_t) get_param_int(PARAM_RC_X_CENTER);
  sticks[RC_STICK_X].range = (uint16_t) get_param_int(PARAM_RC_X_RANGE);
  sticks[RC_STICK_X].positive = false;

  sticks[RC_STICK_Y].channel = (uint8_t) get_param_int(PARAM_RC_Y_CHANNEL);
  sticks[RC_STICK_Y].center = (uint16_t) get_param_int(PARAM_RC_Y_CENTER);
  sticks[RC_STICK_Y].range = (uint16_t) get_param_int(PARAM_RC_Y_RANGE);
  sticks[RC_STICK_Y].positive = false;

  sticks[RC_STICK_Z].channel = (uint8_t) get_param_int(PARAM_RC_Z_CHANNEL);
  sticks[RC_STICK_Z].center = (uint16_t) get_param_int(PARAM_RC_Z_CENTER);
  sticks[RC_STICK_Z].range = (uint16_t) get_param_int(PARAM_RC_Z_RANGE);
  sticks[RC_STICK_Z].positive = false;

  sticks[RC_STICK_F].channel = (uint8_t) get_param_int(PARAM_RC_F_CHANNEL);
  sticks[RC_STICK_F].center = (uint16_t) get_param_int(PARAM_RC_F_BOTTOM);
  sticks[RC_STICK_F].range = (uint16_t) get_param_int(PARAM_RC_F_RANGE);
  sticks[RC_STICK_F].positive = true;

  // TODO directions (probably need to rename some parameters)
  switches[RC_SWITCH_ARMED].channel = (uint8_t) get_param_int(PARAM_ARM_CHANNEL);
  switches[RC_SWITCH_ATTITUDE_OVERRIDE].channel = (uint8_t) get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL);
  switches[RC_SWITCH_THROTTLE_OVERRIDE].channel = (uint8_t) get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL);
  switches[RC_SWITCH_ATTITUDE_TYPE].channel = (uint8_t) get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL);
  switches[RC_SWITCH_F_TYPE].channel = (uint8_t) get_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL);

  for (int i = 0; i < NUM_RC_SWITCHES; i++)
  {
    switches[i].assigned = switches[i].channel >= 0;
    _rc_switch_assigned[i] = switches[i].assigned;
  }

  // TODO initialize values?

  //***************************
}

bool rc_switch(int16_t channel)
{
  if(channel < 4 || channel > 8)
  {
    return false;
  }
  if(switches[channel - 4].direction < 0)
  {
    return pwmRead(channel) < 1500;
  }
  else
  {
    return pwmRead(channel) > 1500;
  }
}

static void convertPWMtoRad()
{
  // Get Roll control command out of RC
  if (_rc_control.x.type == ANGLE)
  {
    _rc_control.x.value = (float)((pwmRead(get_param_int(PARAM_RC_X_CHANNEL)) - 1500)
                           *2.0f*get_param_float(PARAM_RC_MAX_ROLL))/(float)get_param_int(PARAM_RC_X_RANGE);
  }
  else if (_rc_control.x.type == RATE)
  {
    _rc_control.x.value = (float)((pwmRead(get_param_int(PARAM_RC_X_CHANNEL)) - 1500)
                            *2.0f*get_param_float(PARAM_RC_MAX_ROLLRATE))/(float)get_param_int(PARAM_RC_X_RANGE);
  }
  else if (_rc_control.x.type == PASSTHROUGH)
  {
    _rc_control.x.value = pwmRead(get_param_int(PARAM_RC_X_CHANNEL)) - get_param_int(PARAM_RC_X_CENTER);
  }

  // Get Pitch control command out of RC
  if (_rc_control.y.type == ANGLE)
  {
    _rc_control.y.value = ((pwmRead(get_param_int(PARAM_RC_Y_CHANNEL)) - 1500)
                            *2.0f*get_param_float(PARAM_RC_MAX_PITCH))/(float)get_param_int(PARAM_RC_Y_RANGE);
  }
  else if (_rc_control.y.type == RATE)
  {
    _rc_control.y.value = (float)((pwmRead(get_param_int(PARAM_RC_Y_CHANNEL)) - 1500)
                            *2.0f*get_param_float(PARAM_RC_MAX_PITCHRATE))/(float)get_param_int(PARAM_RC_Y_RANGE);
  }
  else if (_rc_control.y.type == PASSTHROUGH)
  {
    _rc_control.y.value = pwmRead(get_param_int(PARAM_RC_Y_CHANNEL)) - 1500;
  }

  // Get the Yaw control command type out of RC
  if (_rc_control.z.type == RATE)
  {
    _rc_control.z.value = ((pwmRead(get_param_int(PARAM_RC_Z_CHANNEL)) - 1500)
                           *2.0f*get_param_float(PARAM_RC_MAX_YAWRATE))/(float)get_param_int(PARAM_RC_Z_RANGE);
  }
  else if (_rc_control.z.type == PASSTHROUGH)
  {
    _rc_control.z.value = pwmRead(get_param_int(PARAM_RC_Z_CHANNEL)) - 1500;
  }

  // Finally, the throttle command
  _rc_control.F.value = (float)((pwmRead(get_param_int(PARAM_RC_F_CHANNEL)) - get_param_int(PARAM_RC_F_BOTTOM)))
                        / (float)get_param_int(PARAM_RC_F_RANGE);
}

//********************************************
// NEW
//********************************************
bool receive_rc(uint32_t now)
{
  static uint32_t last_rc_receive_time = 0;

  if (now - last_rc_receive_time < 20000)
  {
    return false;
  }
  last_rc_receive_time = now;

  // raw RC values
  //  for (int i = 0; i < RC_MAX_CHANNELS; i++)
  for (uint8_t i = 0; i < get_param_int(PARAM_RC_NUM_CHANNELS); i++)
  {
    _rc_raw[i] = pwmRead(i);
  }

  // normalized channels
//  _rc_chan[RC_STICK_X] = 2.0f * (_rc_raw[get_param_int(PARAM_RC_X_CHANNEL)] - get_param_int(PARAM_RC_X_CENTER)) / (float) get_param_int(PARAM_RC_X_RANGE);
//  _rc_chan[RC_STICK_Y] = 2.0f * (_rc_raw[get_param_int(PARAM_RC_Y_CHANNEL)] - get_param_int(PARAM_RC_Y_CENTER)) / (float) get_param_int(PARAM_RC_Y_RANGE);
//  _rc_chan[RC_STICK_Z] = 2.0f * (_rc_raw[get_param_int(PARAM_RC_Z_CHANNEL)] - get_param_int(PARAM_RC_Z_CENTER)) / (float) get_param_int(PARAM_RC_Z_RANGE);
//  _rc_chan[RC_STICK_F] = (_rc_raw[get_param_int(PARAM_RC_F_CHANNEL)] - get_param_int(PARAM_RC_F_BOTTOM)) / (float) get_param_int(PARAM_RC_F_RANGE);
  for (uint8_t i = 0; i < NUM_RC_STICKS; i++)
  {
    if (sticks[i].positive)
    {
      _rc_stick[i] = (_rc_raw[sticks[i].channel] - sticks[i].center) / (float) sticks[i].range;
    }
    else
    {
      _rc_stick[i] = 2.0f * (_rc_raw[sticks[i].channel] - sticks[i].center) / (float) sticks[i].range;
    }
  }

  // switches
  for (uint8_t i = 0; i < NUM_RC_SWITCHES; i++)
  {
    if (switches[i].assigned)
    {
      if (switches[i].direction < 0)
      {
        _rc_switch[i] = (_rc_raw[switches[i].channel] < 1500);
      }
      else
      {
        _rc_switch[i] = (_rc_raw[switches[i].channel] > 1500);
      }
    }
  }
}
//********************************************

bool receive_rc(uint32_t now)
{
  if(_calibrate_rc)
  {
    calibrate_rc();
  }
  // if it has been more than 20ms then look for new RC values and parse them
  static uint32_t last_rc_receive_time = 0;
  static uint32_t time_of_last_stick_deviation = 0;

  if (now - last_rc_receive_time < 20000)
  {
    return false;
  }
  last_rc_receive_time = now;
  // Get timestamp for deadband control lag


  // Figure out the desired control type from the switches and params
  if (get_param_int(PARAM_FIXED_WING))
  {
    // for using fixedwings
    _rc_control.x.type = _rc_control.y.type = _rc_control.z.type = PASSTHROUGH;
    _rc_control.F.type = THROTTLE;
  }
  else
  {
    _rc_control.x.type = _rc_control.y.type = rc_switch(get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)) ? ANGLE : RATE;
    _rc_control.z.type = RATE;
    _rc_control.F.type = rc_switch(get_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL)) ? ALTITUDE : THROTTLE;
  }

  // Interpret PWM Values from RC
  convertPWMtoRad();

  // Set flags for attitude channels
  if (rc_switch(get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL))
      || now - time_of_last_stick_deviation < (uint32_t)(get_param_int(PARAM_OVERRIDE_LAG_TIME))*1000)
  {
    // Pilot is in full control
    _rc_control.x.active = true;
    _rc_control.y.active = true;
    _rc_control.z.active = true;
  }
  else
  {
    // Check for stick deviation - if so, then the channel is active
    _rc_control.x.active = _rc_control.y.active  = _rc_control.z.active =
                             abs(pwmRead(get_param_int(PARAM_RC_X_CHANNEL)) - get_param_int(PARAM_RC_X_CENTER)) >
                             get_param_int(PARAM_RC_OVERRIDE_DEVIATION)
                             || abs(pwmRead(get_param_int(PARAM_RC_Y_CHANNEL)) - get_param_int(PARAM_RC_Y_CENTER)) >
                             get_param_int(PARAM_RC_OVERRIDE_DEVIATION)
                             || abs(pwmRead(get_param_int(PARAM_RC_Z_CHANNEL)) - get_param_int(PARAM_RC_Z_CENTER)) >
                             get_param_int(PARAM_RC_OVERRIDE_DEVIATION);
    if (_rc_control.x.active)
    {
      // reset override lag
      time_of_last_stick_deviation = now;
    }
  }


  // Set flags for throttle channel
  if (rc_switch(get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL)))
  {
    // RC Pilot is in full control
    _rc_control.F.active = true;
  }
  else
  {
    // Onboard Control - min throttle Checking will be done in mux and in the controller.
    _rc_control.F.active = false;
  }

  _new_command = true;
  return true;
}

void calibrate_rc()
{
  if(_armed_state == ARMED)
  {
    mavlink_log_error("Cannot calibrate RC when FCU is armed", NULL);
  }
  else
  {
    // Calibrate Extents of RC Transmitter
    mavlink_log_warning("Calibrating RC, move sticks to full extents", NULL);
    mavlink_log_warning("in the next 10s", NULL);
    uint32_t now = micros();
    static int32_t max[4] = {0, 0, 0, 0};
    static int32_t min[4] = {10000, 10000, 10000, 10000};
    while(micros() - now < 1e7)
    {
      for(int16_t i = 0; i < 4; i++)
      {
        int32_t read_value = (int32_t)pwmRead(i);
        if(read_value > max[i])
        {
          max[i] = read_value;
        }
        if(read_value < min[i])
        {
          min[i] = read_value;
        }
      }
      delay(10);
    }
    set_param_int(PARAM_RC_X_RANGE, max[get_param_int(PARAM_RC_X_CHANNEL)] - min[get_param_int(PARAM_RC_X_CHANNEL)]);
    set_param_int(PARAM_RC_Y_RANGE, max[get_param_int(PARAM_RC_Y_CHANNEL)] - min[get_param_int(PARAM_RC_Y_CHANNEL)]);
    set_param_int(PARAM_RC_Z_RANGE, max[get_param_int(PARAM_RC_Z_CHANNEL)] - min[get_param_int(PARAM_RC_Z_CHANNEL)]);
    set_param_int(PARAM_RC_F_RANGE, max[get_param_int(PARAM_RC_F_CHANNEL)] - min[get_param_int(PARAM_RC_F_CHANNEL)]);

    // Calibrate Trimmed Centers
    mavlink_log_warning("Calibrating RC, leave sticks at center", NULL);
    mavlink_log_warning("and throttle low for next 10 seconds", NULL);
    delay(5000);
    now = micros();
    static int32_t sum[4] = {0, 0, 0, 0};
    static int32_t count[4] = {0, 0, 0, 0};

    while(micros() - now < 5e6)
    {
      for(int16_t i = 0; i < 4; i++)
      {
        int32_t read_value = (int32_t)pwmRead(i);
        sum[i] = sum[i] + read_value;
        count[i] = count[i] + 1;
      }
      delay(20); // RC is updated at 50 Hz
    }

    set_param_int(PARAM_RC_X_CENTER, sum[get_param_int(PARAM_RC_X_CHANNEL)]/count[get_param_int(PARAM_RC_X_CHANNEL)]);
    set_param_int(PARAM_RC_Y_CENTER, sum[get_param_int(PARAM_RC_Y_CHANNEL)]/count[get_param_int(PARAM_RC_Y_CHANNEL)]);
    set_param_int(PARAM_RC_Z_CENTER, sum[get_param_int(PARAM_RC_Z_CHANNEL)]/count[get_param_int(PARAM_RC_Z_CHANNEL)]);
    set_param_int(PARAM_RC_F_BOTTOM, sum[get_param_int(PARAM_RC_F_CHANNEL)]/count[get_param_int(PARAM_RC_F_CHANNEL)]);
  }

  // calculate Trim values (in terms of SI units)
  if(rc_switch(get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)))
  {
    // in angle mode
    set_param_float(PARAM_ROLL_TRIM, (float)(get_param_int(PARAM_RC_X_CENTER) - 1500)*2.0f*get_param_float(PARAM_RC_MAX_ROLL)
                          /(float)get_param_int(PARAM_RC_X_RANGE));
    set_param_float(PARAM_PITCH_TRIM, (float)(get_param_int(PARAM_RC_Y_CENTER) - 1500)*2.0f*get_param_float(PARAM_RC_MAX_PITCH)
                          /(float)get_param_int(PARAM_RC_Y_RANGE));
  }
  else
  {
    // in rate mode
    set_param_float(PARAM_ROLLRATE_TRIM, (float)(get_param_int(PARAM_RC_X_CENTER) - 1500)*2.0f*get_param_float(PARAM_RC_MAX_ROLLRATE)
                          /(float)get_param_int(PARAM_RC_X_RANGE));
    set_param_float(PARAM_PITCHRATE_TRIM, (float)(get_param_int(PARAM_RC_Y_CENTER) - 1500)*2.0f*get_param_float(PARAM_RC_MAX_PITCHRATE)
                          /(float)get_param_int(PARAM_RC_Y_RANGE));
  }
  set_param_float(PARAM_YAWRATE_TRIM, (float)(get_param_int(PARAM_RC_Z_CENTER) - 1500)*2.0f*get_param_float(PARAM_RC_MAX_YAWRATE)
                        /(float)get_param_int(PARAM_RC_Z_RANGE));

  mavlink_log_warning("Completed RC calibration", NULL);
  _calibrate_rc = false;
}

