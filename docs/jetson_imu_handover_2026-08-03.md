# Jetson 전달용 BNO085·융합 오도메트리 요약

작성일: 2026-08-03  
대상: `ssacurity-stm32-drive` STM32F429I-DISC1 ↔ Jetson UART 연동  
Wire protocol: `0x02`

## 1. Jetson에 IMU 정보를 전달해야 하는가

전달해야 한다. 단, 센서 융합 계산은 STM32가 이미 수행하므로 Jetson이 BNO085
값을 다시 융합해서는 안 된다.

| 구분 | STM32 책임 | Jetson 책임 |
| --- | --- | --- |
| BNO085 | SPI5 통신, 초기화, report 수신 | 직접 접근하지 않음 |
| 축·부호 | 차량 좌표계로 맞추고 yaw 부호 보정 | 수신 좌표계를 그대로 사용 |
| 융합 | encoder+steering 모델과 gyro Z 융합 | `0x85` 융합 결과 사용 |
| 장애 대응 | IMU stale 시 모델 odometry로 자동 복귀 | degraded 상태 표시·로그 |
| raw IMU | `0x83`으로 상태와 측정값 송신 | 모니터링 또는 ROS IMU publish |

Jetson의 위치·방향 기준 메시지는 `0x85 TELEMETRY_ODOMETRY`다. `0x83
TELEMETRY_IMU`는 센서 상태 확인과 디버깅용으로 사용한다.

## 2. UART 연결과 프레임

```text
Jetson TX  -> STM32 PD2  / UART5_RX
Jetson RX  <- STM32 PC12 / UART5_TX
Jetson GND -> STM32 GND
Jetson VCC -> 연결하지 않음
```

```text
115200 bps, 8-N-1, no flow control, 3.3 V TTL
[AA][55][02][MSG_ID][SEQ][LEN][PAYLOAD...][CRC_L][CRC_H]
CRC-16/CCITT-FALSE, poly=0x1021, init=0xFFFF, xorout=0
CRC 입력: VERSION부터 payload 마지막 byte까지
모든 다중 byte 값: little-endian
```

Jetson의 실제 `/dev/tty...` 경로는 연결 방식과 보드 설정에 따라 달라지므로
현장에서 확인한다. 하나의 프로세스만 serial device를 열고, 같은 프로세스가
RX parser와 20 Hz TX scheduler를 함께 소유해야 한다.

## 3. TELEMETRY_IMU `0x83`

- 방향: STM32 → Jetson
- 주기: 20 Hz
- payload: 38 bytes

```python
(
    mcu_time_ms,
    quaternion_i_q14,
    quaternion_j_q14,
    quaternion_k_q14,
    quaternion_real_q14,
    gyro_x_mdeg_s,
    gyro_y_mdeg_s,
    gyro_z_mdeg_s,
    linear_accel_x_mm_s2,
    linear_accel_y_mm_s2,
    linear_accel_z_mm_s2,
    yaw_mdeg,
    gyro_accuracy,
    linear_accel_accuracy,
    quaternion_accuracy,
    status_flags,
) = struct.unpack("<IhhhhiiihhhiBBBB", payload)
```

상태 비트:

| Bit | 이름 | 의미 |
| ---: | --- | --- |
| 0 | `CONNECTED` | BNO085 초기화·연결 확인 |
| 1 | `GYRO_VALID` | gyro report 유효 |
| 2 | `LINEAR_ACCEL_VALID` | 선형가속도 report 유효 |
| 3 | `QUATERNION_VALID` | game rotation vector 유효 |
| 4 | `STALE` | 최근 데이터가 오래됨 |
| 5 | `SPI_ERROR` | STM32-BNO085 SPI 오류 |
| 6 | `PROTOCOL_ERROR` | BNO085 SHTP 처리 오류 |

accuracy는 `0=unreliable`, `1=low`, `2=medium`, `3=high`다. 좌표계는 차량
기준 `X=전방`, `Y=왼쪽`, `Z=위쪽`, yaw 양수는 위에서 봤을 때 반시계 방향,
즉 좌회전이다.

현재 quaternion과 `yaw_mdeg`는 game rotation vector 기반 상대 자세다. 절대
북쪽 방위각이나 지도 좌표의 heading으로 간주하지 않는다. Jetson에서 raw yaw를
`0x85` yaw에 다시 더하거나 별도 가중치로 재융합하지 않는다.

## 4. TELEMETRY_ODOMETRY `0x85`

- 방향: STM32 → Jetson
- 주기: 20 Hz
- payload: 36 bytes

```python
(
    mcu_time_ms,
    x_mm,
    y_mm,
    yaw_mdeg,
    distance_mm,
    linear_speed_mm_s,
    yaw_rate_mdeg_s,
    steering_cdeg,
    curvature_micro_per_m,
    status_flags,
    steering_source,
    last_drive_seq,
) = struct.unpack("<IiiiihihiHBB", payload)
```

상태 비트:

| Bit | 이름 | Jetson 처리 |
| ---: | --- | --- |
| 0 | `VALID` | pose update 사용 가능 |
| 1 | `ENCODER_CALIBRATED` | 엔코더 보정 상태 표시 |
| 2 | `GEOMETRY_CALIBRATED` | 차량 형상값 보정 상태 표시 |
| 3 | `STEERING_ESTIMATED` | 실제 조향센서가 아닌 명령 LUT 사용 |
| 4 | `IMU_FUSED` | 해당 odometry에 gyro Z가 융합됨 |
| 5 | `INPUT_INVALID` | pose update를 정상값으로 사용하지 않음 |

Jetson은 `VALID=1`이고 `INPUT_INVALID=0`인 `0x85`를 odometry 입력으로 사용한다.
`IMU_FUSED=1`이면 융합 상태, 0이면 encoder+steering 모델로 동작하는 degraded
상태로 표시한다. `IMU_FUSED=0`만으로 주행을 중단하지 않는다.

`steering_source=2`는 실제 조향센서가 아니라 측정된 7점 servo LUT로 조향각을
추정했다는 뜻이다. 이때 `STEERING_ESTIMATED`도 함께 설정된다. STM32 재부팅 후
pose 원점은 다시 시작되므로 Jetson의 장기 지도 좌표와 직접 동일시하지 않는다.

## 5. Fault와 fallback

`active_fault_bits` 및 `FAULT_EVENT`의 bit 7은 `IMU_LOST`다.

```text
IMU 정상:
  0x83 valid -> 0x85 IMU_FUSED=1 -> gyro 융합 odometry

IMU stale/오류:
  IMU_LOST 보고 -> 0x85 IMU_FUSED=0 -> encoder+steering 모델 자동 복귀
```

현재 `IMU_LOST`는 report-only다. 이것만으로 STM32가 모터를 강제 정지하지
않으며 Jetson도 즉시 E-stop으로 승격하지 않는다. 경고를 표시하고 로그를
남기되, `0x85 VALID`와 실제 주행 안전 fault를 별도로 판단한다.

반면 Jetson의 유효한 새 `CMD_DRIVE`가 300 ms 동안 없으면 STM32가
`COMM_TIMEOUT`으로 안전정지한다. IMU fallback과 UART watchdog을 혼동하지 않는다.

## 6. Jetson 구현 최소 범위

1. 기존 UART parser에 `0x83` 38-byte decoder를 추가한다.
2. `0x85` 36-byte decoder를 odometry의 유일한 입력으로 사용한다.
3. `0x83` status, accuracy, 수신 시각을 health 상태로 저장한다.
4. `0x85 VALID`, `INPUT_INVALID`, `IMU_FUSED`, `STEERING_ESTIMATED`를 노출한다.
5. Fault bit 7 `IMU_LOST`를 경고·로그로 처리한다.
6. ROS/ROS2를 사용한다면 `0x85`를 odometry/TF 원본으로 사용하고, `0x83`은
   선택적으로 IMU topic에 publish한다.
7. serial owner는 하나만 두고 기존 `CMD_DRIVE` 20 Hz heartbeat를 유지한다.

## 7. 연결 시험

```bash
python3 -m pip install pyserial
python3 tools/jetson_uart_echo_test.py --self-test
python3 tools/jetson_uart_echo_test.py --port <UART_PORT> --text JETSON
python3 tools/uart_protocol_test.py telemetry-monitor \
  --port <UART_PORT> --seconds 30
```

합격 기준:

- UART Echo `PASS`
- 30초 동안 `0x83`, `0x85`가 각각 약 20 Hz로 수신됨
- IMU `CONNECTED|GYRO_VALID|QUATERNION_VALID`
- 정지 상태에서 gyro Z가 0 근처로 안정됨
- 좌회전 시 `gyro_z_mdeg_s`와 yaw가 양수
- 주행 시 `0x85 IMU_FUSED=1`
- IMU가 unavailable일 때 `IMU_LOST`, `IMU_FUSED=0`, odometry fallback 확인

## 8. 아직 실차에서 확정할 항목

- BNO085 실제 장착 후 yaw 부호와 정지 bias
- 바닥 주행에서 융합 odometry 거리·회전 오차
- 모터 동작 중 진동과 SPI 노이즈
- Jetson 실제 UART device path

축 부호 보정은 STM32에서 수행한다. 실차 시험 후 부호가 바뀌더라도 Jetson에
제공되는 계약은 계속 `전진 X`, `왼쪽 Y`, `위쪽 Z`, `좌회전 yaw 양수`로 유지한다.

## 9. 함께 전달하는 참조 파일

| 파일 | 용도 |
| --- | --- |
| `jetson_stm32_uart_interface_v3.md` | 전체 UART 계약 |
| `jetson_handover_2026-07-31.md` | 전체 주행 제어 인수인계 |
| `bno085_spi5_integration.md` | STM32 BNO085 구현·시험 근거 |
| `uart_protocol_test.py` | Python 인코더·디코더 참조 구현 |
| `jetson_uart_echo_test.py` | Jetson UART Echo 시험 |
| `comm_protocol.h` | STM32 메시지·상태·fault 원본 정의 |

