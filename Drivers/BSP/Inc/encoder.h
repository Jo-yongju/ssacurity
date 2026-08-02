/*
 * encoder.h
 */


#ifndef BSP_INC_ENCODER_H_
#define BSP_INC_ENCODER_H_

#include "main.h"


void Encoder_Init(void);
int32_t Encoder_GetDiff(void);
float Encoder_GetSpeedMps(int32_t diff_ticks);


#endif /* BSP_INC_ENCODER_H_ */
