# Jetson AI 실행 프롬프트 — 초음파 장애물 HOLD 안전 복귀 수정

아래 내용을 현재 Jetson 작업 저장소의 AI 에이전트에게 그대로 전달한다.

---

## 작업 목표

STM32가 초음파 장애물을 감지해 차량을 `SAFE_STOP`으로 정지시킨 뒤, 장애물이
제거되어 `active_fault_bits`의 bit16이 0으로 내려갔는데도 Jetson 미션이 재개되지
않는 문제를 수정한다.

대상은 현재 작업본의 다음 영역이다.

- `hardware/jetson/ros2_ws/src/robot_navigation`
- `hardware/jetson/ros2_ws/src/stm32_bridge`
- 관련 launch, fake STM32, 단위/통합 테스트

STM32 펌웨어는 수정 대상이 아니다. 기존 사용자 변경을 보존하고, 관련 없는 파일은
수정하지 않는다. 커밋이나 push는 별도 요청이 없으면 수행하지 않는다.

## 먼저 지켜야 할 원칙

1. 현재 코드를 직접 확인한 뒤 수정한다. 아래 분석의 파일·라인 번호가 최신 작업본과
   다르면 실제 코드 구조를 기준으로 다시 추적한다.
2. 실제 로그가 있으면 먼저 다음 전이가 있었는지 확인한다.

   ```text
   HOLD -> FAULT — STM32 가 READY/DRIVING 이 아니다
   ```

3. 로그가 없거나 위 전이가 재현되지 않아도 작업을 중단하지 않는다. 현재 상태머신에는
   `SAFE_STOP + bit16=0`의 짧은 복구 구간을 즉시 영구 FAULT로 만드는 구조적 취약점이
   있으므로 이를 회귀시험과 함께 보완한다.
4. 장애물 해제를 모터 자동 재출발 신호로 사용하지 않는다. Jetson이 STM32의
   `READY(2)`를 확인한 뒤 명시적으로 새 `CMD_DRIVE`를 보내야 한다.
5. 장애물 복구 중 통신 단절이나 다른 하드웨어 fault를 숨기지 않는다.

## STM32 실제 동작 계약

### 메시지와 상태

- `0x80 TELEMETRY_DRIVE.active_fault_bits`
  - bit16 (`1 << 16`) = `OBSTACLE_NEAR`
  - bit16=1: 장애물 감지
  - bit16=0: 장애물 해제
- bit16 해제는 UART 통신 단절을 의미하지 않는다.
- STM32 상태값:
  - `READY = 2`
  - `DRIVING = 3`
  - `SAFE_STOP = 4`
  - `FAULT = 5`
  - `ESTOP = 6`
- 통신 타임아웃은 bit0이며, 유효한 `CMD_DRIVE`가 300ms 동안 없으면 발생한다.
- `0x81 FAULT_EVENT`는 상태 변화 이벤트다. 지속 상태 판단은 주기적으로 수신되는
  `0x80 TELEMETRY_DRIVE`를 기준으로 한다.

### neutral과 rearm

- neutral은 다음 프레임이다.

  ```text
  CMD_DRIVE(speed=0, steering=0, drive_enable=0)
  ```

- neutral은 `SAFE_STOP` 및 bit16=1 상태에서도 수락된다.
- 수락된 neutral은 `rearm_required`를 해제하고 통신 watchdog을 갱신한다.
- 장애물 STOP 자체는 `rearm_required`를 새로 설정하지 않는다.
- 따라서 HOLD 중 neutral이 20Hz로 정상 수신되면 rearm은 이미 해제될 수 있다.
- `CMD_STOP`은 `rearm_required`를 다시 설정하므로 장애물 HOLD 및 정상 복구 경로에서는
  사용하지 않는다.
- 모든 `CMD_DRIVE`는 매번 증가하는 새 uint8 SEQ를 사용해야 한다.

### SAFE_STOP 복귀

STM32는 별도 일회성 복귀 함수를 실행하는 것이 아니라 상태를 주기적으로 다시 계산한다.
다음 조건이 모두 충족되면 `SAFE_STOP`에서 빠져나온다.

1. bit16을 포함한 SAFE_STOP fault가 모두 해제됨
2. 초음파 safety의 `stop_request`가 false
3. `rearm_required`가 false

Jetson이 neutral을 보내 제어 명령이 disabled 상태라면 결과 상태는 `READY(2)`다. 이후
새 `CMD_DRIVE(..., drive_enable=1)`가 수락되면 `DRIVING(3)`으로 전환된다.

중요: STM32의 `0x80` telemetry는 `state`와 `active_fault_bits`를 같은 상태 계산에서
만든다. 정상 경로에서는 첫 bit16=0 프레임이 이미 `READY(2)`일 수 있다. 따라서
`bit16=0, state=SAFE_STOP`이 반드시 약 100ms 발생한다고 가정하거나 100% 재현된다고
단정하지 않는다. 다만 rearm, 스케줄링, 기존 `CMD_STOP` 또는 다른 fault 때문에 이 조합이
실제로 나타날 수 있으므로 Jetson은 이를 안전하게 처리해야 한다.

### 주행 명령 수락 조건

- STM32가 `SAFE_STOP(4)`이거나 `rearm_required=true`이면
  `drive_enable=1` 명령은 거부된다.
- STM32는 장애물 해제 후 과거 주행 명령을 자동 재생하지 않는다. Jetson이 같은 미션
  단계를 유지한 채 목표를 다시 계산하고 새 SEQ로 20Hz 전송해야 한다.

## 현재 Jetson 코드에서 확인된 문제

최신 코드를 다시 확인하되, 기존 분석에서는 다음 흐름이 확인됐다.

1. `route_runner.py`는 `0x80`의 bit16을 매 tick 새로 계산하므로 bit16 자체는 정상적으로
   true에서 false로 바뀐다.
2. `route_logic.py`는 HOLD 처리보다 `not inp.stm32_ready` 검사를 먼저 수행한다.
3. `stm32_ready`는 `SAFE_STOP`을 bit16=1일 때만 예외적으로 허용한다.
4. 따라서 실제로 `bit16=0, state=SAFE_STOP` 프레임이 들어오면 장애물 clear debounce에
   도달하기 전에 Jetson 상태가 영구 `FAULT`로 래치될 수 있다.
5. FAULT 전이 hook은 `CMD_STOP(STOP_INTERNAL)`을 보내 STM32 rearm을 다시 요구하게 만든다.
6. FAULT가 래치되면 나중에 STM32가 READY가 되어도 미션은 자동 복귀하지 않는다.

이 구조적 문제를 수정하되, 실제 미재개의 직접 원인이라고 보고하기 전에는 반드시
실측 로그 또는 추가한 회귀시험으로 전이 조건을 증명한다.

## 필수 구현 요구사항

### 1. 장애물 HOLD 복구 단계를 명시적으로 표현

장애물 HOLD 중 bit16이 0이 된 이후를 일반 READY 실패가 아니라 제한된 복구 구간으로
처리한다. 이름은 현재 코드 스타일에 맞추되 다음 의미가 명확해야 한다.

```text
OBSTACLE_HOLD
  -> OBSTACLE_CLEAR_DEBOUNCE / STM32_REARM_WAIT
  -> READY 확인
  -> 기존 미션 단계 RUNNING
```

별도 enum 상태를 추가하거나 HOLD 내부의 명시적 sub-state/필드를 사용해도 된다. 단순히
`stm32_ready=True`로 덮어써서 다른 fault를 숨기는 방식은 금지한다.

### 2. bit16 해제 후 READY 대기

- bit16=1 동안:
  - 장애물 HOLD 유지
  - clear 타이머 초기화
  - neutral을 20Hz로 계속 송신
- bit16=0이 된 뒤:
  - neutral을 20Hz로 계속 송신
  - 0.5초 연속 clear debounce 시작
  - STM32 `READY(2)`를 기다림
- 다음 두 조건을 모두 충족한 경우에만 HOLD 해제:
  - bit16=0이 0.5초 이상 연속 유지
  - 최신 `0x80`의 state가 `READY(2)`
- READY가 먼저 와도 0.5초가 끝나기 전에는 재개하지 않는다.
- 0.5초가 먼저 끝나도 READY가 오기 전에는 `drive_enable=1`을 보내지 않는다.
- bit16이 다시 1이 되면 clear 및 READY 대기 타이머를 초기화한다.

### 3. 제한 시간과 다른 fault 처리

- bit16 해제 후 STM32 READY 대기 timeout을 파라미터로 추가한다. 기본값은 2.0초로 한다.
- 다음 조건을 만족할 때만 `SAFE_STOP + bit16=0`을 임시 복구 상태로 허용한다.
  - 현재 미션 상태가 실제 장애물 HOLD에서 시작됨
  - telemetry 연결이 유효함
  - bit16을 제외한 다른 blocking fault가 없음
- 기존 blocking mask를 사용할 경우 다음처럼 의도를 명확히 분리한다.

  ```python
  other_blocking_faults = active_fault_bits & (
      FAULT_ARM_BLOCKING_MASK & ~FAULT_OBSTACLE_NEAR
  )
  ```

  실제 상수 구성에 맞춰 구현하되 bit0 통신 타임아웃, 엔코더/조향 오류, 센서 stale,
  ESTOP, 내부 fault 등이 복구 대기로 가려지지 않게 한다.
- 다른 blocking fault가 생기거나 telemetry가 timeout되면 기존 안전 정책에 따라 즉시
  FAULT 처리한다.
- bit16=0 이후 2.0초 안에 READY가 오지 않으면 원인과 전체 fault bits를 남기고 FAULT로
  전환한다.

### 4. 장애물 정상 복구 경로에서는 CMD_STOP 금지

- 장애물 HOLD 진입, clear debounce, rearm/READY 대기, 정상 미션 재개 과정에서는
  `CMD_STOP`을 보내지 않는다.
- neutral `CMD_DRIVE(0,0,0)`만 계속 보낸다.
- 실제 치명 fault, 운영자 abort, 미션 완료, 노드 종료 등 기존에 STOP이 필요한 경로의
  의미는 유지한다.
- 상태 전이 hook이 모든 FAULT에 일괄 STOP을 보내는 구조라면, 장애물 복구 대기가
  FAULT로 잘못 분류되지 않도록 먼저 상태머신을 바로잡는다. 실제 FAULT의 STOP 정책까지
  무조건 제거하지 않는다.

### 5. 미션 재개

- HOLD 해제 시 `_hold_from`에 저장한 원래 상태 또는 현재 프로젝트의 동일 기능을 사용해
  같은 미션 단계로 돌아간다.
- 과거 UART 프레임을 그대로 재생하지 않는다. 현재 odometry와 유지된 단계 정보를 바탕으로
  `_tick_drive`/`_tick_turn`에서 명령을 다시 계산한다.
- 재개 명령은 새 SEQ로 20Hz 계속 송신한다.
- `last_drive_seq`가 Jetson이 보낸 새 SEQ를 따라 갱신되고, STM32 state가 `DRIVING(3)`으로
  전환되는지 관측할 수 있게 한다.

### 6. HOLD 시간을 단계 timeout에서 제외

장애물 HOLD가 길어도 재개 직후 turn/drive 단계의 timeout 또는 yaw jump guard가 오발하지
않도록 다음을 처리한다.

- HOLD 진입 시각과 해제 시각을 기록한다.
- 단계 timeout 기준 시각에 HOLD 지속시간을 반영하거나, 별도 active elapsed time을 사용한다.
- HOLD 중 odometry/yaw 기준값은 최신 상태로 동기화하되, 정지 중 변화를 미션 진행량에
  어떻게 반영할지 명시한다.
- 재개 첫 tick에서 HOLD 전체 기간의 yaw 변화가 단일 tick 변화로 계산되지 않게 한다.
- 이 변경은 기존 단계 인덱스나 완료 진행량을 임의로 초기화하지 않아야 한다.

### 7. 진단 로그

디버그 옵션이 활성화된 경우 다음 값을 한 줄로 확인할 수 있게 한다. 기본 운영 로그를
20Hz로 오염시키지 않도록 파라미터 또는 throttle을 사용하되, 재현 모드에서는 50~100ms
전이를 놓치지 않는 빈도로 출력할 수 있어야 한다.

```text
timestamp,
active_fault_bits,
bit16,
other_blocking_fault_bits,
obstacle_hold,
recovery_substate,
clear_elapsed_ms,
ready_wait_elapsed_ms,
stm32_state,
tx_speed,
tx_steering,
tx_enable,
tx_seq,
last_drive_seq
```

FAULT 전환 시에는 최소한 다음을 한 줄로 남긴다.

```text
reason, stm32_state, active_fault_bits, bit16,
other_blocking_fault_bits, telemetry_age_ms, last_drive_seq
```

### 8. 0x84 거리 telemetry 처리 범위

- 장애물 HOLD의 권위 있는 판정값은 `0x80 active_fault_bits` bit16이다.
- 이번 수정에서 `0x84` 원시 거리를 별도의 장애물 판정으로 중복 사용하지 않는다.
- `0x84`를 진단용으로 추가한다면 `valid_mask`를 반드시 확인하고, `0xFFFF`를 -1mm 또는
  0mm 장애물로 해석하지 않는다.
- 0x84 진단 추가가 핵심 상태머신 수정 범위를 크게 키우면 별도 후속 작업으로 남긴다.

## 필수 테스트

### fake STM32 확장

현재 fake에 없다면 다음 상태 전이를 재현할 수 있게 한다.

```text
DRIVING
-> SAFE_STOP + bit16=1
-> neutral 수신 유지
-> bit16=0
-> 필요 시 짧은 SAFE_STOP + faults=0 복구 구간
-> READY
-> 새 CMD_DRIVE 수신
-> DRIVING
```

fake에서도 neutral과 주행 CMD_DRIVE의 SEQ 증가, watchdog, READY 전 주행 명령 거부를
실제 STM32 계약과 일치시킨다.

### 단위/회귀 테스트

최소한 다음을 자동화한다.

1. bit16=1이면 장애물 HOLD에 들어가고 neutral을 반환한다.
2. bit16=0 직후 state=SAFE_STOP이어도 다른 fault가 없으면 즉시 FAULT로 가지 않는다.
3. clear 0.5초 미만에는 state=READY여도 `enable=1`을 내보내지 않는다.
4. clear 0.5초가 지나도 state=READY가 아니면 `enable=1`을 내보내지 않는다.
5. clear 0.5초 이상이고 READY일 때만 같은 미션 단계로 복귀한다.
6. clear 도중 bit16이 재설정되면 타이머가 초기화된다.
7. READY 대기 2.0초를 넘으면 명확한 이유로 FAULT가 된다.
8. recovery 중 bit0 또는 다른 blocking fault가 생기면 즉시 FAULT가 된다.
9. 장애물 정상 HOLD/복구 구간에는 `CMD_STOP`이 송신되지 않는다.
10. neutral 및 재개 CMD_DRIVE의 SEQ가 증가한다.
11. HOLD 지속시간이 turn/drive timeout에 포함되지 않는다.
12. HOLD 해제 첫 tick의 yaw 기준 동기화가 yaw jump 오발을 만들지 않는다.
13. telemetry가 0.3초 이상 끊기면 장애물 복구 대기로 숨기지 않고 통신 fault가 된다.

가능하면 다음 두 telemetry 순서를 모두 시험한다.

```text
A: state=SAFE_STOP, bit16=1 -> state=READY, bit16=0
B: state=SAFE_STOP, bit16=1 -> state=SAFE_STOP, bit16=0 -> state=READY, bit16=0
```

A는 현재 STM32 정상 경로에서 충분히 가능하며, B는 지연/rearm 상황에 대한 견고성 시험이다.

## 실장비 확인 절차

1. 주행 중 장애물을 STOP 임계 안으로 넣는다.
2. `bit16=1`, `state=SAFE_STOP`, neutral 20Hz, 증가하는 SEQ를 확인한다.
3. 장애물을 해제 임계 밖으로 제거한다.
4. bit16이 0으로 유지되는지 확인한다.
5. `SAFE_STOP + bit16=0`이 실제로 관측되는지와 지속시간을 기록한다.
6. 다른 blocking fault 없이 STM32가 READY로 전환되는지 확인한다.
7. clear 0.5초와 READY 두 조건 전에는 `enable=1`이 송신되지 않는지 확인한다.
8. 두 조건 충족 후 같은 미션 단계 명령이 새 SEQ로 20Hz 송신되는지 확인한다.
9. STM32 `last_drive_seq` 갱신과 `DRIVING(3)` 전환을 확인한다.
10. 장애물 재감지, READY timeout, 통신 단절도 각각 시험한다.

테스트 중 차량이 갑자기 움직일 수 있으므로 첫 검증은 구동 바퀴를 지면에서 띄우거나
안전하게 고정한 상태에서 수행한다.

## 완료 조건

- 장애물 제거 후 Jetson이 영구 FAULT에 빠지지 않는다.
- bit16 clear debounce 0.5초와 STM32 READY가 모두 확인되기 전에는 재출발하지 않는다.
- 정상 장애물 복구 중 CMD_STOP을 보내지 않는다.
- 다른 fault 및 통신 단절은 복구 대기에 가려지지 않는다.
- 같은 미션 단계가 새 CMD_DRIVE/SEQ로 재개된다.
- fake STM32와 자동 회귀시험이 추가되고 모두 통과한다.
- 기존 정상 주행, 운영자 정지, 미션 완료, 치명 fault 처리 테스트가 계속 통과한다.

## 최종 보고 형식

작업 후 다음 순서로 보고한다.

1. 실제 확인한 직접 원인과 증거 로그
2. 수정한 파일 목록
3. 상태 전이 변경 내용
4. `CMD_STOP`, neutral, SEQ 처리 결과
5. 추가한 fake 시나리오와 테스트 목록
6. 실행한 명령 및 테스트 결과
7. 실장비에서 아직 확인해야 할 항목
8. 남아 있는 위험 또는 후속 개선 사항

