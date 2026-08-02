

// 주행 제어 모듈
// 주행 제어 기능과 상태값 저장

#define CONTROL_DT_SEC 0.01f
#define SPEED_FILTER_ALPHA  0.2f  //필터


#include "task_control.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"


static PID_t speed_pid;   // 속도 PID 구조체
static const float SPEED_PID_KP = 150.0f;
static const float SPEED_PID_KI = 100.0f;
static const float SPEED_PID_KD = 0.0f;

static const float SPEED_PID_OUTPUT_MIN = -100.0f;
static const float SPEED_PID_OUTPUT_MAX = 100.0f;

static const float SPEED_PID_INTEGRAL_MIN = -1.0f;
static const float SPEED_PID_INTEGRAL_MAX = 1.0f;


static ControlCommand_t current_command;   //현재 명령
static ControlState_t current_state;		//현재 제어 상태


static float filtered_speed_mps = 0.0f;    			//필터링된 속도
static uint8_t speed_filter_initialized = 0U;		// 속도 필터 초기화

// 모터, 엔코더, 속도 PID와 제어 명령·상태를 안전한 초기 상태로 설정
void Control_Init(void){

	DriveMotor_Init(); //모터 초기화
	Encoder_Init(); //엔코더 초기화
	//pid 초기화
	PID_Init(&speed_pid,
	         SPEED_PID_KP,
	         SPEED_PID_KI,
	         SPEED_PID_KD,
	         SPEED_PID_OUTPUT_MIN,
	         SPEED_PID_OUTPUT_MAX,
	         SPEED_PID_INTEGRAL_MIN,
	         SPEED_PID_INTEGRAL_MAX);
	Motor_SetPWM(0.0f); //초기 모터 정지

	//명령 구조체 초기화
	current_command.mode = CONTROL_DISABLED;
	current_command.pwm_percent = 0.0f;
	current_command.target_speed_mps = 0.0f;
	current_command.target_steering_deg = 0.0f;

	//상태 구조체 초기화
	current_state.encoder_diff = 0;
	current_state.measured_speed_mps = 0.0f;
	current_state.applied_pwm_percent = 0.0f;
	current_state.target_steering_deg = 0.0f;
	current_state.total_encoder_ticks = 0.0f;

}



void Control_SetCommand(const ControlCommand_t *command){
	//유효한 명령 주소인지 확인
	if(command != NULL){
		// 제어 모드가 변경되면 이전 PID 누적 상태 초기화
		if(command -> mode != current_command.mode){
			PID_Reset(&speed_pid);
		}
		current_command = *command;  //새로운 주행명령 저장, 호출한 곳의 명령 받아들임.
	}
}




void Control_GetState(ControlState_t *state){
	//유효한 명령 주소인지 확인
	if(state != NULL){
		*state = current_state; //상태를 호출한 곳으로 전달
	}
}





/*
 * 10ms 주기로 모터 속도 제어를 수행한다.
 *
 * 엔코더 변화량을 읽어 현재 주행 속도를 계산하고,
 * 설정된 제어 모드에 따라 모터에 적용할 PWM을 결정한다.
 *
 * - CONTROL_DISABLED  : 모터 정지 및 PID 상태 초기화
 * - CONTROL_OPEN_LOOP : 입력받은 PWM 값을 그대로 출력
 * - CONTROL_SPEED_PID : 목표 속도와 측정 속도의 오차로 PWM 계산
 *
 * 계산된 PWM을 모터에 출력하고, 측정 속도와 적용 PWM을
 * current_state에 저장한다.
 */

void Control_Run10ms(void){
	 current_state.encoder_diff = Encoder_GetDiff(); //엔코더 변화량 저장
	 current_state.total_encoder_ticks += current_state.encoder_diff;

	 float raw = Encoder_GetSpeedMps(current_state.encoder_diff);
	 current_state.measured_speed_mps = raw; //측정된 속도 저장


	 /* 1. 엔코더로 계산한 원본 속도 */
	 current_state.measured_speed_mps+= 0.2f * (raw - current_state.measured_speed_mps);  // 필터 후

	  /* 2. 저역통과 필터 적용 */
	  if (speed_filter_initialized == 0U)
	  {
	      filtered_speed_mps = raw_speed_mps;
	      speed_filter_initialized = 1U;
	  }
	  else
	  {
	      filtered_speed_mps +=
	          SPEED_FILTER_ALPHA *
	          (raw_speed_mps - filtered_speed_mps);
	  }

	  /* 3. PID에는 필터링된 속도를 전달 */
	  current_state.measured_speed_mps = filtered_speed_mps;



	 float pwm_percent = 0.0f; // pwm 출력값 변수

	 //주행 모드에 따라 처리
	 /*
	  *DISABLED
	  → PWM 0
      → PID 리셋

       OPEN_LOOP
       → 명령받은 PWM 그대로 출력

      SPEED_PID + 목표 속도 0
      → PWM 0
      → PID 리셋

      SPEED_PID + 목표 속도 있음
	  → 목표 속도와 측정 속도로 PID 계산
	  → 계산된 PWM 출력
	*/
	 if(current_command.mode == CONTROL_DISABLED){
		 PID_Reset(&speed_pid);
		 pwm_percent = 0.0f;
	 }
	 else if(current_command.mode == CONTROL_OPEN_LOOP){
		 pwm_percent = current_command.pwm_percent;
	 }
	 else if (current_command.mode == CONTROL_SPEED_PID){
		 if(current_command.target_speed_mps == 0.0f){
			 pwm_percent = 0.0f;
			 PID_Reset(&speed_pid);
		 }
		 else{
			 pwm_percent = PID_Compute(&speed_pid,
					 	 current_command.target_speed_mps,
			             current_state.measured_speed_mps,
						 CONTROL_DT_SEC);
		 }
	 }
	 else
	 {
	     pwm_percent = 0.0f;
	     PID_Reset(&speed_pid);
	 }


	 Motor_SetPWM(pwm_percent); //모터 출력
	 current_state.applied_pwm_percent = pwm_percent; //pwm 상태 저장
}











