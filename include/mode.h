#ifdef __cplusplus
extern "C" {
#endif

#pragma once

typedef enum
{
  ARMED,
  DISARMED,
  FAILSAFE_ARMED,
  FAILSAFE_DISARMED
} armed_state_t;
extern armed_state_t _armed_state;

typedef enum
{
  INVALID_CONTROL_MODE,
  INVALID_ARMED_STATE,
} error_state_t;
error_state_t _error_state;

void init_mode(void);
bool check_mode(uint32_t now);

#ifdef __cplusplus
}
#endif

