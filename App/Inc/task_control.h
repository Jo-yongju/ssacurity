/*
 * [제어 모듈 함수 사용 흐름]
 *
 * ControlTask
 *  - 시작 시 Control_Init() 호출
 *  - 10ms마다 Control_Run10ms() 호출
 *
 * CommTask / 테스트 코드
 *  - 새로운 주행 명령을 받으면 Control_SetCommand() 호출
 *  - 전달된 명령은 제어 모듈 내부에 저장됨
 *
 * TelemetryTask / 디버깅 코드
 *  - 현재 제어 상태가 필요할 때 Control_GetState() 호출
 *  - 엔코더 변화량, 측정 속도, 적용 PWM 등을 가져옴
 *
 * SafetyTask
 *  - 위험 상황 발생 시 Control_SetCommand()로 정지 명령 전달 가능
 *
 * 전체 흐름:
 * 명령 수신
 *   -> Control_SetCommand()
 *   -> Control_Run10ms()
 *   -> 모터 제어 및 상태 갱신
 *   -> Control_GetState()
 *   -> 상태 전송 또는 디버깅
 */


#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

#include "main.h"


// 제어 모드 enum
/*제어 모드

세 가지 상태
비활성화
PWM 직접 제어
속도 PID 제어
명령 구조체에 들어갈 값

*/

typedef enum{
	CONTROL_DISABLED = 0,
	CONTROL_OPEN_LOOP = 1,
	CONTROL_SPEED_PID = 2
} ControlMode_t;


// 명령 구조체
/*
명령 구조체에 들어갈 값
현재 제어 모드
직접 PWM 명령(%)
목표 속도(m/s)
목표 조향각(°)
*/

typedef struct {
	ControlMode_t mode;
	float pwm_percent;
	float target_speed_mps;
	float target_steering_deg;
} ControlCommand_t;


// 상태 구조체
/*
상태 구조체에 들어갈 값
최근 엔코더 변화량
측정 속도(m/s)
실제로 적용한 PWM(%)
현재 목표 조향각(°)
*/

typedef struct{
	int32_t encoder_diff;
	float measured_speed_mps;
	float   raw_speed_mps;
	float applied_pwm_percent;
	float target_steering_deg;
	int64_t total_encoder_ticks;
} ControlState_t;


//외부에서 사용할 함수 선언
/*
 초기화
10ms 주기 제어 실행
명령 저장
현재 상태 조회
 */
void Control_Init(void);
void Control_Run10ms(void);	// ControlTask에서 10ms마다 제어 실행
void Control_SetCommand(const ControlCommand_t *command); // CommTask 등이 새로운 주행 명령을 제어 모듈에 전달
void Control_GetState(ControlState_t *state); // TelemetryTask 등이 현재 제어 상태를 가져옴

#endif /* TASK_CONTROL_H */
