
#include "pid.h"




void PID_Reset(PID_t *pid){
	pid -> integral = 0.0f;
	pid -> previous_error = 0.0f;
	pid -> first_run = 1U;
}

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max,
              float integral_min,
              float integral_max)
{
    // pid 구조체에 kp, ki, kd 저장
	pid -> kp = kp;
	pid -> ki = ki;
	pid -> kd = kd;

    // 출력 제한값 저장
	pid -> output_min = output_min;
	pid -> output_max = output_max;

    // 적분 제한값 저장
	pid -> integral_min = integral_min;
	pid -> integral_max = integral_max;

    // 누적 상태 초기화
	PID_Reset(pid);
}


//값 제한 함수
static float Clamp(float value, float min, float max)
{
    // value가 max보다 크면 max 반환
	if(value > max) return max;
    // value가 min보다 작으면 min 반환
	if(value < min) return min;
    // 그 외에는 value 반환
	return value;
}



float PID_Compute(PID_t *pid,
                  float target,
                  float measurement,
                  float dt){

	if(dt <= 0.0f) return 0.0f;
	float error = target -measurement;// 오차

	float p = pid -> kp * error; // p항 계산

	float d = 0.0f;
	//첫 실행이 아닐 때만 d 계산
	if(pid -> first_run == 0U){
		d =  pid -> kd * (error - pid -> previous_error)/ dt; // d항 계산
	}

	// 적분 업데이트 전에 현재 적분값 기준으로 잠정 출력 계산
	float u     = p + pid -> ki * pid -> integral + d;
	float u_sat = Clamp(u, pid -> output_min, pid -> output_max);

	// 포화됐고 error가 그 포화를 더 밀어붙이는 방향일 때만 적분 중단
	//if((u - u_sat) * error <= 0.0f){
		pid -> integral += error * dt; // 누적 오차 계산
		pid -> integral = Clamp(pid -> integral, pid -> integral_min, pid -> integral_max); // 누적오차 범위 초과 방지
	//}


	float i = pid ->ki * pid -> integral; //i항 계산


	pid -> previous_error = error;
	pid -> first_run = 0U;

	return Clamp(p + i + d, pid -> output_min,pid -> output_max);
}




