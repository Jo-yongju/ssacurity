#ifndef APP_TASK_CONTROL_H
#define APP_TASK_CONTROL_H

#include "main.h"

typedef enum
{
  CONTROL_DISABLED = 0,
  CONTROL_OPEN_LOOP = 1,
  CONTROL_SPEED_PID = 2
} ControlMode_t;

typedef struct
{
  ControlMode_t mode;
  float pwm_percent;
  float target_speed_mps;
  float target_steering_deg;
} ControlCommand_t;

typedef struct
{
  int32_t encoder_diff;
  float measured_speed_mps;
  float raw_speed_mps;
  float applied_pwm_percent;
  float target_steering_deg;
  int64_t total_encoder_ticks;
} ControlState_t;

void Control_Init(void);
void Control_Run10ms(void);
void Control_SetCommand(const ControlCommand_t *command);
void Control_GetState(ControlState_t *state);

#endif
