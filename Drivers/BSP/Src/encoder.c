/*
 * encoder.c
 *
 */

#define WHEEL_DIAMETER_M       0.064f			//바퀴 지름 m 단위
#define CONTROL_DT_SEC         0.01f			//
#define COUNTS_PER_WHEEL_REV	823.0f				// 회전당 엔코더 카운트 수
#define PI 3.14159265359f

#include "encoder.h"
extern TIM_HandleTypeDef htim4;


/*
 * TIM4 Encoder Mode를 시작하고
 * 정·역방향 카운트를 위해 CNT를 중앙값으로 초기화한다.
 */

void Encoder_Init(void){
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

	TIM4 ->CNT = 32768U;
}




/*
 * TIM4의 현재 엔코더 카운트와 중앙값의 차이를 반환하고,
 * 다음 측정을 위해 CNT를 다시 중앙값으로 초기화한다.
 */

/*
 * 32768로 시작
      ↓
* 10ms 동안 증가/감소
      ↓
* CNT - 32768로 이동량 계산
      ↓
* 다시 32768로 초기화
      ↓
* 다음 10ms 측정
 */

int32_t Encoder_GetDiff(void)
{
    // 현재 CNT 읽기
	uint32_t current_cnt = TIM4->CNT;
    // 현재값 - 32768 계산
	int32_t diff =  (int32_t) current_cnt - 32768;
    // CNT를 32768로 재설정
	TIM4->CNT = 32768;
    // 차이값 반환
	return diff;
}


/*
 * 일정 시간 동안 측정한 엔코더 차이값을
 * 바퀴의 실제 선속도(m/s)로 변환한다.
 */
float Encoder_GetSpeedMps(int32_t diff_ticks)
{
    // 바퀴 둘레 계산
	float wheel_circumference = PI* WHEEL_DIAMETER_M ;

    // 엔코더 1틱당 이동거리 계산
	float distance_per_tick = wheel_circumference / COUNTS_PER_WHEEL_REV;

    // diff_ticks로 이동거리 계산
	float distance = diff_ticks * distance_per_tick;
    // 0.01초로 나누어 속도 계산
	float v = distance / CONTROL_DT_SEC;
    // 속도 반환
	return v;
}





