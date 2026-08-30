# Jetson–STM32 UART 연동 구현 기준 V3.0

이 문서는 Jetson 팀이 전달한 `Jetson–STM32 UART 연동 명세 V3.0`과
`Jetson_STM32_통신명세_V2.3_변경사항.md`를 STM32 프로젝트에 적용한
결과를 기록한다. Wire protocol version은 `0x02`다.

## 1. 물리 연결

| Jetson/USB-UART | 방향 | STM32F429I-DISC1 |
|---|:---:|---|
| TXD | → | PD2 / UART5_RX / P1-40 |
| RXD | ← | PC12 / UART5_TX / P1-43 |
| GND | ↔ | GND / P1-63 또는 P1-64 |

- 115200 bps, 8-N-1, flow control 없음, 3.3 V TTL
- TX/RX는 교차하고 GND는 공통으로 연결한다.
- USB-UART의 VCC는 연결하지 않는다.
- Jetson J12 직결과 USB-UART를 동시에 연결하지 않는다.

## 2. Frame

```text
[AA][55][02][MSG_ID][SEQ][LEN][PAYLOAD 0..64][CRC_L][CRC_H]
```

- CRC-16/CCITT-FALSE: poly `0x1021`, init `0xFFFF`, xorout `0`
- CRC 범위: `VERSION`부터 `PAYLOAD` 끝까지
- 다중 바이트 정수와 CRC는 little-endian이다.
- 송신자는 메시지 종류와 무관한 단일 `uint8_t` TX SEQ를 사용한다.
- 수신자는 duplicate와 old frame을 재실행하지 않는다.
- 유효 frame을 350 ms 이상 받지 못한 뒤의 첫 frame은 새 세션으로
  수락한다.

Parser는 256-byte 지속 버퍼에서 SOF를 재탐색한다. version, length, CRC
오류 시 후보 SOF의 첫 바이트만 버리며, 불완전 frame은 100 ms 후 같은
방식으로 복구한다.

UART5 수신은 256-byte DMA 버퍼에서 512-byte software RX ring으로 옮겨
stream parser가 처리한다. 송신 ring도 512 byte다.

## 3. 메시지

| ID | 이름 | 방향 | Payload | 주기 |
|---:|---|---|---:|---|
| `0x10` | `CMD_DRIVE` | Jetson → STM32 | 5 | 20 Hz |
| `0x11` | `CMD_STOP` | Jetson → STM32 | 1 | 이벤트 |
| `0x12` | `CMD_RESET_FAULT` | Jetson → STM32 | 4 | 이벤트 |
| `0x80` | `TELEMETRY_DRIVE` | STM32 → Jetson | 26 | 20 Hz |
| `0x81` | `FAULT_EVENT` | STM32 → Jetson | 14 | 상태 변화 |
| `0x82` | `COMMAND_RESULT` | STM32 → Jetson | 4 | 단발 명령 |
| `0x83` | `TELEMETRY_IMU` | STM32 → Jetson | 38 | 20 Hz |
| `0x84` | `TELEMETRY_RANGE` | STM32 → Jetson | 13 | 10 Hz |
| `0x85` | `TELEMETRY_ODOMETRY` | STM32 → Jetson | 36 | 20 Hz |
| `0xF0` | `DIAG_ECHO_REQ` | Jetson → STM32 | 0..31 | 개발 |
| `0xF1` | `DIAG_ECHO_RESP` | STM32 → Jetson | 1..32 | 개발 |
| `0xF2` | `DIAG_MOTOR_TEST_REQ` | PC → STM32 | 4 | 벤치 |
| `0xF3` | `DIAG_MOTOR_TEST_RESP` | STM32 → PC | 6 | 벤치 |
| `0xF6` | `DIAG_SERVO_REQUEST` | PC → STM32 | 2 | 벤치 |
| `0xF7` | `DIAG_SERVO_RESPONSE` | STM32 → PC | 7 | 벤치 |

기존 V1의 `0x82 TELEMETRY_ODOMETRY`와 `0xF4..0xF5` PID 진단은 V3 활성 UART
프로토콜에서 사용하지 않는다. `0xF2..0xF3` 모터 진단은 실차 배선 확인을 위해
V2 frame 형식으로 복구했으며 ±20~95%, 100~10000ms 범위에서만 동작한다.

### Echo

Echo response header의 SEQ는 STM32 자체 TX 카운터다. Payload는
`[요청 SEQ][요청 data...]`다. 요청 payload 내부의 `AA 55`도 그대로
보존한다.

### TELEMETRY_DRIVE

26-byte payload 순서는 다음과 같다.

```text
<IhhhhhihBBI
mcu_time_ms
target_speed_mm_s
measured_speed_mm_s
motor_duty_permille
steering_cmd_cdeg
steering_feedback_cdeg
encoder_count
yaw_cdeg
drive_state
last_drive_seq
active_fault_bits
```

`yaw_cdeg`는 pose update가 유효할 때의 odometry yaw다. 현재 기본
`IMU_ONLY`에서는 정렬된 quaternion yaw를 사용하고, IMU heading이 유효하지
않거나 stale이면 엔코더+조향 LUT의 model heading을 사용한다.
`last_drive_seq`는 마지막으로 실제 수락한 `CMD_DRIVE`의 SEQ이며, 거부된
명령에서는 갱신하지 않는다.

`active_fault_bits`와 14-byte `FAULT_EVENT`의 active/latched fault bit는
다음과 같다.

| bit | 이름 |
| ---: | --- |
| 0 | `COMM_TIMEOUT` |
| 1 | `CRC_ERROR` |
| 2 | `BAD_COMMAND` |
| 3 | `ENCODER_INVALID` |
| 4 | `MOTOR_STALL` |
| 5 | `DIRECTION` |
| 6 | `CONTROL_OVERRUN` |
| 7 | `IMU_LOST` |
| 8 | `RANGE_LOST` |
| 9 | `STEERING_INVALID` |
| 10 | `ESTOP_ACTIVE` |
| 11 | `INTERNAL` |
| 12 | `COMMAND_LIMIT` |
| 13 | `UART_RX_OVERFLOW` |
| 14 | `UART_TX_ERROR` |
| 15 | `SENSOR_STALE` |
| 16 | `OBSTACLE_NEAR` |

### TELEMETRY_IMU

38-byte payload 순서는 다음과 같다.

```text
<IhhhhiiihhhiBBBB
mcu_time_ms
quaternion_i_q14, quaternion_j_q14, quaternion_k_q14, quaternion_real_q14
gyro_x_mdeg_s, gyro_y_mdeg_s, gyro_z_mdeg_s
linear_accel_x_mm_s2, linear_accel_y_mm_s2, linear_accel_z_mm_s2
yaw_mdeg
gyro_accuracy, linear_accel_accuracy, quaternion_accuracy
status_flags
```

status bit 0~6은 각각 `CONNECTED`, `GYRO_VALID`, `LINEAR_ACCEL_VALID`,
`QUATERNION_VALID`, `STALE`, `SPI_ERROR`, `PROTOCOL_ERROR`다. BNO085 accuracy는
0=unreliable, 1=low, 2=medium, 3=high다.

### TELEMETRY_RANGE

13-byte payload 순서는 다음과 같다.

```text
<IHHHHB
mcu_time_ms
front_left_mm
front_right_mm
rear_left_mm
rear_right_mm
valid_mask
```

현재 단일 전방 HC-SR04는 V3의 primary front 슬롯인
`front_left_mm`/`valid_mask bit 0`으로 임시 매핑한다. 물리적인 좌/우 위치가
확정됐다는 의미는 아니며, `ULTRA_STATUS_OK`이고 200 ms 이내인 측정만
valid다. 나머지 세 채널은 `0xFFFF`/invalid로 유지한다.

현재 설정은 `VEHICLE_ENFORCE_ULTRASONIC_SAFETY=1U`이며 STM32가 로컬 안전
정지를 수행한다. CAUTION은 0.60 m에서 진입하고 0.65 m에서 해제된다. 거리가
0.40 m 미만이면 STOP하며, 0.50 m 이상인 새로운 샘플을 3회 연속 확인한 뒤
해제한다. 새로운 `NO_ECHO` 샘플은 최대 유효 거리 방향으로 평가하지만,
`INIT`/`TIMEOUT`/`OUT_OF_RANGE`/`STALE` 또는 200 ms 이상 갱신되지 않은
`NO_ECHO`는 STOP한다. 근접 정지 시 `OBSTACLE_NEAR` fault가 설정된다.

### TELEMETRY_ODOMETRY

36-byte payload 순서는 다음과 같다.

```text
<IiiiihihiHBB
mcu_time_ms
x_mm
y_mm
yaw_mdeg
distance_mm
linear_speed_mm_s
yaw_rate_mdeg_s
steering_cdeg
curvature_micro_per_m
status_flags
steering_source
last_drive_seq
```

`steering_source=2`는 물리 조향센서가 아니라 7점 LUT의 등가 중심 조향
명령각을 사용했다는 뜻이다. 이 경우 `status_flags`의 bit 3
`STEERING_ESTIMATED`가 반드시 함께 설정된다.

| status bit | 의미 |
| ---: | --- |
| 0 | pose update valid |
| 1 | encoder calibrated |
| 2 | wheelbase/track geometry calibrated |
| 3 | steering command estimate used |
| 4 | IMU heading used |
| 5 | input invalid |
| 6 | IMU heading fallback used |

현재 기본 heading mode는 `IMU_ONLY`다. 연결·quaternion 유효·100 ms 이내
수신·유한 yaw 조건을 만족하면 bit 4가 1이다. 현재
`VEHICLE_IMU_ENFORCE_ACCURACY_GATE=0U`이므로 BNO085 accuracy 값으로 heading
사용을 차단하지 않는다. IMU heading이
유효하지 않거나 stale이면 bit 4가 0, bit 6이 1로 바뀌고 엔코더+조향 LUT의
model heading으로 자동 복귀한다. `MODEL_ONLY`, `COMPLEMENTARY`, `IMU_ONLY`
세 모드가 있으며 complementary correction weight는 0.75다. 공중 바퀴
시험은 계산 부호와 UART 전송만 검증하며 실제 바닥 위치 정확도를 보증하지
않는다.

## 4. 안전 동작

- 신규·유효·수락 가능한 `CMD_DRIVE`만 300 ms watchdog을 갱신한다.
- 300 ms 만료 시 출력 중립, `COMM_TIMEOUT`, `SAFE_STOP`, 재무장 요구
- 재무장은 neutral `CMD_DRIVE(0,0,0)` 수락 후에만 해제
- `CMD_STOP`과 `CMD_RESET_FAULT`는 `COMMAND_RESULT`를 반환
- 현재 속도 명령 범위는 실측 지속속도 기준 `-1565~+1565 mm/s`이다.
- 현재 모터 출력 상한은 무부하 포화 측정점인 절댓값 95% PWM이다.
- 초음파 안전은 활성화되어 있다. 근접 장애물과 invalid/stale 센서 상태에서
  STM32가 로컬 STOP하고 PID를 reset하며 모터 출력을 차단한다.
- 조향 범위는 비대칭 `-28.69°~+19.55°`이며 7점 LUT 측정 범위와 같다.
- 위 한계보다 큰 `drive_enable=1` 명령은 거부
- UART `CMD_STOP`은 물리 E-stop을 대체하지 않는다.

물리 E-stop 입력은 현재 프로젝트에 할당된 핀이 없으므로
`COMM_STATE_ESTOP`과 fault bit 정의만 적용돼 있다. 실제 HW 경로와 GPIO가
확정되기 전에는 물리 E-stop 구현 완료로 간주하지 않는다.

## 5. UART5 기본 설정과 PC-as-Jetson 운영 명령 시험

현재 작업 브랜치는 최종 Jetson 연동을 위해
`VEHICLE_COMM_USE_STLINK_VCP=0U`로 설정되어 PC12/PD2의 UART5를 활성 통신
포트로 사용한다. USART1/ST-LINK USB VCP는 초기화되지만 프로토콜 송수신에는
사용되지 않는다.

이 시험은 진단용 PWM 명령이 아니라 Jetson과 동일한 `CMD_DRIVE` 20 Hz
heartbeat, 속도 PID, 서보 명령, `TELEMETRY_DRIVE`, `CMD_STOP` 경로를
사용한다.

전제:

- 차량을 뒤집거나 구동 바퀴를 모두 지면에서 띄운다.
- HC-SR04 정면 65 cm 이내를 비워 CLEAR 상태를 확보한다.
- 12 V 모터 전원, 서보용 5 V 강압 전원, STM32 GND가 공통인지 확인한다.
- 업데이트된 펌웨어를 플래시한 뒤 보드를 한 번 리셋한다.

단일 명령 시험:

```powershell
py tools\uart_protocol_test.py drive-test `
  --port COM11 `
  --speed-mm-s 80 `
  --steering-deg 0 `
  --seconds 1.5 `
  --verify-watchdog `
  --wheels-off-ground
```

`steering-deg`는 중앙 등가 조향각이며 양수는 좌회전, 음수는 우회전이다.
도구는 neutral 재무장, 20 Hz 주행 명령, 텔레메트리 수락 확인, 엔코더
방향, 300 ms link-loss 정지, 재무장, `CMD_STOP`까지 검사한다.

전체 시나리오:

```powershell
py tools\uart_protocol_test.py drive-scenario `
  --port COM11 `
  --wheels-off-ground
```

순서는 전진 직진 → 전진 좌조향 → 전진 우조향 → 후진 직진 → heartbeat
중단이다. 각 단계에서 `PASS`가 출력되고 마지막에
`PC-as-Jetson full scenario: PASS`가 나와야 한다.

현재 조향센서는 미장착이다. 따라서 `steering_feedback_cdeg`는
0(유효하지 않음)이다. 오도메트리 이동거리는 엔코더로 계산하고, 기본
heading은 정렬된 BNO085 Game Rotation Vector quaternion yaw를 사용한다.
IMU heading이 유효하지 않거나 stale이면 실측 7점 등가 중심각 LUT와
휠베이스 0.135 m의 model heading으로 자동 복귀한다.
`TELEMETRY_ODOMETRY`의 `steering_source=2`와
`STEERING_ESTIMATED`가 설정된 경우에만 이 추정 pose를 사용한다.
기구 유격·서보 미도달·타이어 미끄러짐은 감지할 수 없으므로 실제 바닥
정확도는 별도로 검증해야 한다.

## 6. Jetson/UART5 물리 Echo 시험

Jetson/UART5 물리 시험 전 `App/Inc/vehicle_config.h`가
`VEHICLE_COMM_USE_STLINK_VCP=0U`인지 확인한다. 최신 펌웨어를 빌드·플래시한
뒤 같은 wire protocol과 같은 명령 도구를 Jetson의 UART5 배선에서 사용한다.
PC12/PD2의 실제 배선은 소프트웨어 자체 시험으로 대신할 수 없으므로 UART5
Echo 1회는 생략하지 않는다.

```bash
python3 -m pip install pyserial
python3 tools/jetson_uart_echo_test.py --self-test
python3 tools/jetson_uart_echo_test.py \
  --port /dev/ttyUSB0 \
  --text JETSON
```

정상 결과:

```text
Protocol self-test: PASS
Golden frames GF-01..GF-07: PASS
Jetson <-> STM32 UART5 echo: PASS
```

## 7. 확정값과 미확정 항목

확정하여 코드에 반영한 값:

- 휠베이스: 0.135 m
- 앞바퀴 조향축 중심 간 거리: 0.085 m
- 엔코더: 823 count/바퀴 1회전
- 타이어 명목 지름: 64 mm, 명목 둘레: 201.06 mm
- 속도 명령 범위: -1565~+1565 mm/s
- 모터 출력 상한: 95% PWM
- 조향 명령 범위: -28.69~+19.55 deg
- 서보 7점 LUT: 766, 921, 1076, 1231, 1386, 1541, 1696 us
- 통신 watchdog: 300 ms
- 오도메트리 송신: `0x85`, 36 byte, 20 Hz
- 기본 heading mode: `IMU_ONLY`, 유효하지 않을 때 model fallback
- 초음파 로컬 안전: 활성화, STOP 0.40 m 미만, 해제 후보 0.50 m 이상 3회

실차에서 확인한 결과:

- 1 m 직선 거리: 1015, 1021, 1018 mm, 평균 1018 mm, 평균 절대 오차 1.8%
- heading MAE: `MODEL_ONLY` 8.19도, `IMU_ONLY` 1.32도,
  `COMPLEMENTARY(0.75)` 1.65도
- `IMU_ONLY` 좌회전 MAE 1.23도, 우회전 MAE 1.41도

실제 차량에서 아직 검증하거나 확정해야 하는 항목:

- 다양한 하중·노면에서의 타이어 실구름 둘레 재현성
- 전체 2D 경로의 위치 오차와 반복 주행 재현성
- 기구 유격, 서보 미도달 및 타이어 미끄러짐
- HC-SR04의 V3 front-left/front-right 채널 배치
- 0.40 m 강제정지값의 실제 바닥 정지거리 검증
- 물리 E-stop 입력과 독립 모터 에너지 차단 경로
- 실제 Jetson UART 장치 경로와 UART5 물리 Echo

엔코더 값은 정방향 3회전 +2473, 역방향 3회전 -2464를 평균해
823 count/바퀴 1회전으로 결정했다.
