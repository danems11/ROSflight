#include <stdbool.h>
#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "mavlink_util.h"
#include "param.h"
#include "sensors.h"

// global variable definitions
int16_t _accel_data[3];
int16_t _gyro_data[3];
int32_t _accel_scale;
int32_t _gyro_scale;
int16_t _imu_temperature;
uint32_t _imu_time;
bool _imu_ready;
bool _image_taken = false;

void imu_ISR(void)
{
  static int16_t count = 0;
  _imu_time = micros();
  _imu_ready = true;
  if (count > 1000/_params.values[PARAM_CAMERA_TRIGGER_RATE])
  {
//    TRIG_HIGH;
    count=0;
    _image_taken=true;
  }
  else
  {
//    TRIG_LOW;
    count++;
    //_image_taken=false;
  }
}


// local function definitions
static bool update_imu(void)
{
  if (_imu_ready)
  {
    _imu_ready = false;
    mpu6050_read_accel(_accel_data);
    mpu6050_read_gyro(_gyro_data);
    mpu6050_read_temperature(&_imu_temperature);

    // correct according to known biases and temperature compensation
    _accel_data[0] -= (_params.values[PARAM_ACC_X_TEMP_COMP]*_imu_temperature)/1000 + _params.values[PARAM_ACC_X_BIAS];
    _accel_data[1] -= (_params.values[PARAM_ACC_Y_TEMP_COMP]*_imu_temperature)/1000 + _params.values[PARAM_ACC_Y_BIAS];
    _accel_data[2] -= (_params.values[PARAM_ACC_Z_TEMP_COMP]*_imu_temperature)/1000 + _params.values[PARAM_ACC_X_BIAS];
    _gyro_data[0] -= _params.values[PARAM_GYRO_X_BIAS];
    _gyro_data[1] -= _params.values[PARAM_GYRO_Y_BIAS];
    _gyro_data[2] -= _params.values[PARAM_GYRO_Z_BIAS];
    return true;
  }
  else
  {
    return false;
  }
}

// function definitions
void init_sensors(void)
{
  // IMU
  _imu_ready = false;
  uint16_t acc1G;
  float gyro_scale;

  mpu6050_register_interrupt_cb(&imu_ISR);
  mpu6050_init(true, &acc1G, &gyro_scale, _params.values[PARAM_BOARD_REVISION]);

  _accel_scale = (1000*9807)/acc1G; // convert to um/s^2
  _gyro_scale = (int32_t)(gyro_scale*1000000000.0f); // convert to mrad/s
}

bool update_sensors(uint32_t time_us)
{
  return update_imu();
}

