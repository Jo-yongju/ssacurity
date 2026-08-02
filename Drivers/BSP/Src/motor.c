/*
  motor.c
 */

#define ABS(x) (((x) >= 0) ? (x) : -(x))


#include "motor.h"
extern TIM_HandleTypeDef htim5;


//초기화 함수
/*
 1. BTS_R_EN, BTS_L_EN을 Low
2. TIM5 PWM Channel 1 시작
3. TIM5 PWM Channel 2 시작
4. 두 PWM Compare 값을 0으로 설정
5. BTS_R_EN, BTS_L_EN을 High
 */

void DriveMotor_Init(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);


    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

    TIM5->CCR1 = 0;
    TIM5->CCR4 = 0;


    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

}


//
/*
 1. 입력값을 -100~100으로 제한
2. PWM 절댓값을 CCR 값으로 변환
3. 양수이면 CH1만 출력, CH2는 0
4. 음수이면 CH1은 0, CH2만 출력
5. 0이면 CH1과 CH2 모두 0
 */
void Motor_SetPWM(float pwm_percent)
{
	if(pwm_percent < -100){
		pwm_percent = -100;
	}
	if(pwm_percent > 100){
		pwm_percent = 100;
	}

	uint32_t ccr = (uint32_t)(ABS(pwm_percent) * TIM5->ARR / 100.0f);//

	if(pwm_percent > 0){
		TIM5->CCR4 = 0;
		TIM5->CCR1 = ccr;
	}
	else if(pwm_percent < 0){
		TIM5->CCR1 = 0;
		TIM5->CCR4 = ccr;
	}
	else{
		TIM5->CCR1 = 0;
		TIM5->CCR4 = 0;
	}
}
