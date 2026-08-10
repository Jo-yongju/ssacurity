#include "task_control.h"

#include "encoder.h"
#include "motor.h"
#include "pid.h"

#include <stddef.h>
#include <string.h>

#define CONTROL_DT_SEC             0.010f
#define CONTROL_PID_KP             120.0f
#define CONTROL_PID_KI             20.0f
#define CONTROL_PID_KD             0.0f
#define CONTROL_MIN_RUNNING_PWM    22.0f
#define CONTROL_MAX_RUNNING_PWM    95.0f
#define CONTROL_PID_OUTPUT_MIN     \
  (-(CONTROL_MAX_RUNNING_PWM - CONTROL_MIN_RUNNING_PWM))
#define CONTROL_PID_OUTPUT_MAX     \
  (CONTROL_MAX_RUNNING_PWM - CONTROL_MIN_RUNNING_PWM)

static PIDController speed_pid;
static ControlCommand_t current_command;
static ControlState_t current_state;

static float ClampPercent(float percent)
{
  if (percent > 100.0f)
  {
    return 100.0f;
  }

  if (percent < -100.0f)
  {
    return -100.0f;
  }

  return percent;
}

static float ApplySpeedFeedforward(float target_speed_mps,
                                   float pid_correction)
{
  float output;

  /* Fixed 22% dead-zone compensation used before linear feedforward. */
  if (target_speed_mps > 0.0f)
  {
    output = CONTROL_MIN_RUNNING_PWM + pid_correction;
    if (output < 0.0f)
    {
      return 0.0f;
    }
    if (output > CONTROL_MAX_RUNNING_PWM)
    {
      return CONTROL_MAX_RUNNING_PWM;
    }
    return output;
  }

  output = -CONTROL_MIN_RUNNING_PWM + pid_correction;
  if (output > 0.0f)
  {
    return 0.0f;
  }
  if (output < -CONTROL_MAX_RUNNING_PWM)
  {
    return -CONTROL_MAX_RUNNING_PWM;
  }
  return output;
}

void Control_Init(void)
{
  bool motor_ok;
  bool encoder_ok;

  memset(&current_command, 0, sizeof(current_command));
  memset(&current_state, 0, sizeof(current_state));
  current_command.mode = CONTROL_DISABLED;

  motor_ok = Motor_Init();
  encoder_ok = Encoder_Init();
  PID_Init(&speed_pid,
           CONTROL_PID_KP,
           CONTROL_PID_KI,
           CONTROL_PID_KD,
           CONTROL_PID_OUTPUT_MIN,
           CONTROL_PID_OUTPUT_MAX);

  if (motor_ok && encoder_ok)
  {
    Motor_Disable();
  }
  else
  {
    current_command.mode = CONTROL_DISABLED;
    Motor_Disable();
  }
}

void Control_SetCommand(const ControlCommand_t *command)
{
  if (command == NULL)
  {
    return;
  }

  if (command->mode != current_command.mode)
  {
    PID_Reset(&speed_pid);
  }

  current_command = *command;
  current_command.pwm_percent = ClampPercent(current_command.pwm_percent);
  if ((current_command.mode < CONTROL_DISABLED) ||
      (current_command.mode > CONTROL_SPEED_PID))
  {
    current_command.mode = CONTROL_DISABLED;
  }
}

void Control_GetState(ControlState_t *state)
{
  if (state != NULL)
  {
    *state = current_state;
  }
}

void Control_Run10ms(void)
{
  EncoderSample encoder_sample;
  float pwm_percent = 0.0f;

  Encoder_Update(CONTROL_DT_SEC, &encoder_sample);
  current_state.encoder_diff = encoder_sample.delta_count;
  current_state.total_encoder_ticks = encoder_sample.total_count;
  current_state.raw_speed_mps = encoder_sample.speed_mps;
  current_state.measured_speed_mps = encoder_sample.speed_mps;
  current_state.target_steering_deg = current_command.target_steering_deg;

  switch (current_command.mode)
  {
    case CONTROL_OPEN_LOOP:
      pwm_percent = current_command.pwm_percent;
      Motor_Enable();
      Motor_SetPercent(pwm_percent);
      break;

    case CONTROL_SPEED_PID:
      if (!encoder_sample.calibrated)
      {
        PID_Reset(&speed_pid);
        Motor_Disable();
      }
      else if (current_command.target_speed_mps == 0.0f)
      {
        PID_Reset(&speed_pid);
        Motor_Enable();
        Motor_SetPercent(0.0f);
      }
      else
      {
        float pid_correction = PID_Update(
            &speed_pid,
            current_command.target_speed_mps,
            current_state.measured_speed_mps,
            CONTROL_DT_SEC);

        pwm_percent = ApplySpeedFeedforward(
            current_command.target_speed_mps,
            pid_correction);
        Motor_Enable();
        Motor_SetPercent(pwm_percent);
      }
      break;

    case CONTROL_DISABLED:
    default:
      PID_Reset(&speed_pid);
      Motor_Disable();
      break;
  }

  current_state.applied_pwm_percent = Motor_GetAppliedPercent();
}
