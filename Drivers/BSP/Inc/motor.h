#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>

bool Motor_Init(void);
void Motor_Enable(void);
void Motor_Disable(void);
bool Motor_IsEnabled(void);
void Motor_EmergencyDisable(void);
bool Motor_ClearEmergencyDisable(void);
bool Motor_IsEmergencyDisabled(void);
void Motor_SetPercent(float percent);
float Motor_GetAppliedPercent(void);
void Motor_SetDirectionInverted(bool inverted);

#endif
