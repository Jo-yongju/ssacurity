





#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
	float kp;
	float ki;
	float kd;

	float integral; //적분 누적값
	float previous_error; //이전 오차

    // 출력 최소·최대,최종 PWM 출력 제한
	float output_min;
	float output_max;

    // 적분 최소·최대,적분 누적값 자체의 제한
	float integral_min;
	float integral_max;

    // 첫 실행 여부
	uint8_t first_run;
} PID_t;

//PID 구조체 초기화
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max,
              float integral_min,
              float integral_max);

// PID 게인과 제한값은 유지하고 누적된 제어 상태만 초기화
void PID_Reset(PID_t *pid);

// 목표값과 측정값을 이용해 PID 출력 계산
float PID_Compute(PID_t *pid,
                  float target,
                  float measurement,
                  float dt);

#endif







