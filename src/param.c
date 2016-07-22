#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wswitch"

#include <stdbool.h>
#include <stdint.h>

#include "flash.h"
#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_stream.h"

#include "param.h"
#include "mixer.h"

//TODO temporary
#include <stdio.h>

// global variable definitions
params_t _params;

// local function definitions
static void init_param_int(param_id_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
  _params.values[id] = value;
  _params.types[id] = PARAM_TYPE_INT32;
}

static void init_param_float(param_id_t id, char name[PARAMS_NAME_LENGTH], float value)
{
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
  _params.values[id] = *((int32_t *) &value);
  _params.types[id] = PARAM_TYPE_FLOAT;
}

// function definitions
void init_params(void)
{
  initEEPROM();
  if (!read_params())
  {
    set_param_defaults();
    write_params();
  }

  for (uint16_t id = 0; id < PARAMS_COUNT; id++)
    param_change_callback((param_id_t) id);
}

void set_param_defaults(void)
{
  // temporary: replace with actual initialisation of rest of params
  char temp_name[PARAMS_NAME_LENGTH];
  for (uint16_t id = 0; id < PARAMS_COUNT; id++)
  {
    sprintf(temp_name, "TEMP_%c%c", 'A' + id/10, 'A' + id%10);
    init_param_int((param_id_t) id, temp_name, id);
  }
  init_param_int(PARAM_BOARD_REVISION, "BOARD_REV", 5);

  init_param_int(PARAM_BAUD_RATE, "BAUD_RATE", 921600);

  init_param_int(PARAM_SYSTEM_ID, "SYS_ID", 1);
  init_param_int(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1);
  init_param_int(PARAM_STREAM_IMU_RATE, "STRM_IMU", 1000);

  init_param_int(PARAM_CAMERA_TRIGGER_RATE, "CAMERA_RATE", 27);

  init_param_int(PARAM_STREAM_ADJUSTED_GYRO, "STRM_ADJ_GYRO", 0);

  init_param_int(PARAM_INIT_TIME, "FILTER_INIT_T", 3000); // ms
  init_param_float(PARAM_FILTER_KP, "FILTER_KP", 10000); // munits
  init_param_float(PARAM_FILTER_KI, "FILTER_KI", 1000);  // munits
  init_param_float(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0);
  init_param_float(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0);
  init_param_float(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0);
  init_param_float(PARAM_ACC_X_BIAS,  "ACC_X_BIAS", 0);
  init_param_float(PARAM_ACC_Y_BIAS,  "ACC_Y_BIAS", 0);
  init_param_float(PARAM_ACC_Z_BIAS,  "ACC_Z_BIAS", 0);
  init_param_float(PARAM_ACC_X_TEMP_COMP,  "ACC_X_TEMP_COMP", 0);
  init_param_float(PARAM_ACC_Y_TEMP_COMP,  "ACC_Y_TEMP_COMP", 0);
  init_param_float(PARAM_ACC_Z_TEMP_COMP,  "ACC_Z_TEMP_COMP", 0);
}

bool read_params(void)
{
  return readEEPROM();
}

bool write_params(void)
{
  return writeEEPROM(true);
}

void param_change_callback(param_id_t id)
{
  switch (id)
  {
  case PARAM_SYSTEM_ID:
    mavlink_system.sysid = _params.values[PARAM_SYSTEM_ID];
    break;
  case PARAM_STREAM_HEARTBEAT_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_HEARTBEAT, _params.values[PARAM_STREAM_HEARTBEAT_RATE]);
    break;

  case PARAM_STREAM_ATTITUDE_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_ATTITUDE, _params.values[PARAM_STREAM_ATTITUDE_RATE]);
    break;

  case PARAM_STREAM_IMU_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_IMU, _params.values[PARAM_STREAM_IMU_RATE]);
    // no action needed for this parameter
    break;
  }
}

param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH])
{
  for (uint16_t id = 0; id < PARAMS_COUNT; id++)
  {
    bool match = true;
    for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++)
    {
      // compare each character
      if (name[i] != _params.names[id][i])
      {
        match = false;
        break;
      }

      // stop comparing if end of string is reached
      if (_params.names[id][i] == '\0')
        break;
    }

    if (match)
      return (param_id_t) id;
  }

  return PARAMS_COUNT;
}

bool set_param_by_id(param_id_t id, int32_t value)
{
  if (id < PARAMS_COUNT && value != _params.values[id])
  {
    _params.values[id] = value;
    param_change_callback(id);
    mavlink_send_param(id);
    return true;
  }
  return false;
}

bool set_param_by_name(const char name[PARAMS_NAME_LENGTH], int32_t value)
{
  param_id_t id = lookup_param_id(name);
  return set_param_by_id(id, value);
}

bool set_param_by_id_float(param_id_t id, float value)
{
  return set_param_by_id(id, *(int32_t *) &value);
}

bool set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value)
{
  return set_param_by_name(name, *(int32_t *) &value);
}

float get_param_float(param_id_t id)
{
  return *(float *) &_params.values[id];
}
