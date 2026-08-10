# BNO085 SPI5 및 오도메트리 융합

## 1. 확정 배선

GY-BNO08X 모듈은 **3.3 V로만** 공급하고 STM32와 GND를 공통으로 연결한다.
이 모듈의 `SDA`는 SPI에서 센서 출력(MISO), `ADO`는 센서 입력(MOSI)이다.

| BNO085 모듈 | STM32F429I-DISC1 | 역할 |
| --- | --- | --- |
| VCC | 3.3 V | 센서 전원 |
| GND | GND | 공통 접지 |
| SCL | PF7 / P2-6 | SPI5_SCK |
| SDA | PF8 / P2-5 | SPI5_MISO, 센서 → STM32 |
| ADO | PF9 / P2-8 | SPI5_MOSI, STM32 → 센서 |
| CS | PF6 / P2-3 | BNO085 chip select, active-low |
| INT | PG3 / P2-61 | data-ready EXTI3, active-low |
| RST | PG2 / P2-62 | hardware reset, active-low |
| PS1 | 3.3 V | reset 시 SPI 선택 |
| PS0 | 3.3 V | reset 시 SPI 선택, 이후 WAKE high 유지 |

전원을 끈 상태에서 배선하고, PS1/PS0가 3.3 V에 연결된 뒤 전원을 넣는다.
온보드 L3GD20도 PF7/PF8/PF9에 연결돼 있으므로 펌웨어는 그 센서의 PC1 CS를
부팅 직후 HIGH로 유지한다. 두 센서의 CS를 동시에 LOW로 만들면 안 된다.

## 2. SPI와 태스크 설정

- SPI5 master, 8-bit, MSB first
- mode 3: CPOL=1, CPHA=1
- APB2 90 MHz / 32 = 2.8125 MHz (BNO085 최대 3 MHz 이하)
- 소프트웨어 NSS, BNO085 CS는 PF6 GPIO로 직접 제어
- PG3 EXTI3 우선순위 6
- ISR은 SPI를 실행하지 않고 `ImuTask` thread flag만 설정
- BNO085 report interval 10 ms
- 요청 report: calibrated gyro, linear acceleration, game rotation vector

`ImuTask`는 product-ID 응답까지 확인한 뒤 report를 켠다. 통신 오류 또는
500 ms 무응답이면 하드웨어 reset부터 다시 시도한다.

## 3. 장착 방향과 위치

센서는 차체의 회전 중심과 무게중심에 가까울수록 좋지만, 수 cm 오차 때문에
코드가 동작하지 않는 것은 아니다. 다음 조건이 더 중요하다.

- 차체 프레임에 단단히 고정하고 흔들리는 브래킷이나 완충 스펀지 위에 두지 않는다.
- 기본 좌표는 센서 X축=차량 전방, Y축=차량 왼쪽, Z축=위쪽이다.
- 모터, BTS7960, 굵은 배터리선, 스피커·자석에서 떨어뜨린다.
- 센서 축을 뒤집어 장착했다면 `VEHICLE_IMU_YAW_SIGN`을 실물 시험 후 바꾼다.

현재 자세에는 자기장 영향을 줄이기 위해 magnetometer 기반 Rotation Vector가
아닌 Game Rotation Vector를 사용한다. 따라서 절대 북쪽 방위가 아니라 부팅 후
상대 자세를 제공한다.

## 4. 오도메트리 융합

기존 모델은 엔코더 이동거리와 조향각으로 회전량을 계산한다.

```text
model_delta_yaw = encoder_delta_distance × steering_curvature
imu_delta_yaw   = gyro_z × 0.020 s
fused_delta_yaw = 0.25 × model_delta_yaw + 0.75 × imu_delta_yaw
```

다음 조건을 모두 만족할 때만 IMU를 섞는다.

- calibrated gyro report가 100 ms 이내에 수신됨
- BNO085 accuracy는 텔레메트리에 보고하지만 현재 긴급 시운전 설정에서는 융합을 차단하지 않음
- 차량 측정 속도의 절댓값이 0.02 m/s 이상
- yaw rate가 유한값이며 절댓값 10 rad/s 이하

조건이 깨지면 pose 적분을 중단하지 않고 기존 엔코더+조향 아커만 방식으로
자동 복귀한다. 이때 `COMM_FAULT_IMU_LOST`는 report-only이고 차량을 강제
정지시키지 않으며, odometry의 `IMU_FUSED` bit만 0이 된다.

0.75 가중치는 첫 시운전용 값이다. 바닥 원주행과 직진 시험으로 자이로 부호,
바이어스, 조향 모델 오차를 확인한 뒤 조정해야 한다.

## 5. UART IMU 텔레메트리

`0x83 TELEMETRY_IMU`는 38 bytes, 20 Hz이며 little-endian 형식은 다음과 같다.

```text
<IhhhhiiihhhiBBBB
mcu_time_ms
quaternion_i_q14
quaternion_j_q14
quaternion_k_q14
quaternion_real_q14
gyro_x_mdeg_s
gyro_y_mdeg_s
gyro_z_mdeg_s
linear_accel_x_mm_s2
linear_accel_y_mm_s2
linear_accel_z_mm_s2
yaw_mdeg
gyro_accuracy
linear_accel_accuracy
quaternion_accuracy
status_flags
```

status bit는 `CONNECTED`, `GYRO_VALID`, `LINEAR_ACCEL_VALID`,
`QUATERNION_VALID`, `STALE`, `SPI_ERROR`, `PROTOCOL_ERROR` 순서로 bit 0~6이다.

## 6. 실물 시험 순서

1. 모터 12 V 전원은 끄고 STM32와 BNO085만 켠다.
2. 멀티미터로 VCC-GND가 3.3 V인지 확인한다.
3. PC 통신 모드 펌웨어를 플래시하고 다음 수신 전용 시험을 실행한다.

   ```powershell
   py tools\uart_protocol_test.py imu-monitor `
     --port COM11 `
     --seconds 30 `
     --settle-seconds 3 `
     --rate-hz 2 `
     --csv imu_test.csv
   ```

4. 처음 3초간 정지해 REFERENCE yaw를 잡은 뒤 차체를 수평으로 유지한다.
5. 왼쪽 약 90도 → 기준 방향 → 오른쪽 약 90도 순서로 천천히 회전한다.
6. `CONNECTED`, `GYRO_VALID`, `LINEAR_ACCEL_VALID`, `QUATERNION_VALID`가 있고
   stale/error bit가 0인지 확인한다. Accuracy는 기록하되 현재 융합 차단
   조건은 아니다.
7. 왼쪽 회전에서 `gyro_z`와 `delta_yaw`가 양수, 오른쪽에서 음수인지
   확인한다. 방향이 반대면 배선을 바꾸지 말고 `VEHICLE_IMU_YAW_SIGN`을
   `-1.0f`로 바꾼다.
8. 차체를 정지시킨 상태에서 gyro XYZ가 0 근처로 안정되는지 확인한다.
9. 저속 직진·좌회전·우회전에서 odometry `IMU_FUSED`가 1인지 확인한다.
10. 주행 중 INT 또는 CS 선을 분리하는 시험은 하지 않는다. 정지 상태에서 센서
   전원을 껐다 켜 `IMU_LOST`와 자동 재연결을 확인한다.

정상 기준은 20 Hz IMU frame 수신, stale/error bit 0, 정지 시 gyro 안정,
왼쪽 약 +90도·오른쪽 약 -90도 변화, 주행 중 `IMU_FUSED=1`이다. 최종 장착
상태의 반복 각도 오차와 융합 가중치는 실제 바닥 주행에서 추가 검증한다.
