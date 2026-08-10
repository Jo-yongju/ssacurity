#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Bidirectional wheel calibration:
 *   forward: +2473 counts / 3 revolutions
 *   reverse: -2464 counts / 3 revolutions
 *   rounded average: 823 counts / revolution
 */
#define ENCODER_DEFAULT_COUNTS_PER_WHEEL_REV 823.0f
#define ENCODER_DEFAULT_WHEEL_CIRCUMFERENCE_M 0.20106f

typedef struct
{
  int32_t delta_count;
  int64_t total_count;
  float delta_distance_m;
  float total_distance_m;
  float speed_mps;
  bool calibrated;
} EncoderSample;

bool Encoder_Init(void);
void Encoder_Reset(void);
void Encoder_Update(float dt_seconds, EncoderSample *sample);
bool Encoder_SetCalibration(float counts_per_wheel_revolution,
                            float wheel_circumference_m);
bool Encoder_IsCalibrated(void);
void Encoder_SetDirectionSign(int8_t direction_sign);

#endif
