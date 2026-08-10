# Jetson 파트 전달용 짧은 구현 프롬프트

아래 내용을 첨부 ZIP과 함께 Jetson 프로젝트를 작업하는 AI에게 전달한다.

## 요청 시작

첨부 문서를 기준으로 Jetson과 STM32 UART V3 연동에 BNO085 상태와 융합
오도메트리 수신을 추가해줘.

코드를 수정하기 전에 Jetson 프로젝트의 ROS/ROS2 버전, 기존 serial owner,
명령 송신 노드, odometry/TF/IMU topic과 테스트 구조를 먼저 분석하고 충돌을
보고해줘. 그다음 기존 구조를 최대한 유지해서 구현해줘.

필수 조건:

- UART `115200 8-N-1`, wire protocol `0x02`, CRC-16/CCITT-FALSE를 유지한다.
- 하나의 프로세스만 UART를 열고 RX parser와 20 Hz `CMD_DRIVE` TX를 함께 맡는다.
- `0x83 TELEMETRY_IMU`, 38 bytes,
  `struct.unpack("<IhhhhiiihhhiBBBB", payload)`를 파싱한다.
- `0x85 TELEMETRY_ODOMETRY`, 36 bytes,
  `struct.unpack("<IiiiihihiHBB", payload)`를 파싱한다.
- 위치와 yaw는 `0x85`만 사용한다. `0x83` raw yaw를 다시 융합하지 않는다.
- `0x85 VALID=1`, `INPUT_INVALID=0`일 때만 정상 odometry로 사용한다.
- `IMU_FUSED`를 융합/degraded 상태로 노출한다.
- Fault bit 7 `IMU_LOST`는 경고·로그로 처리하며 이것만으로 주행을 정지하지 않는다.
- 구형 `0x82 TELEMETRY_ODOMETRY`는 사용하지 않는다.
- 기존 300 ms STM32 command watchdog과 neutral 재무장 규칙을 유지한다.
- 파서 단위시험, CRC/frame 분할·결합 시험, `0x83/0x85` decoder 시험을 추가한다.

ROS/ROS2를 사용한다면 `0x85`를 odometry/TF 원본으로 publish하고 `0x83`은
선택적으로 IMU/health topic으로 publish해줘. frame 이름과 covariance는 기존
프로젝트 규칙을 확인한 뒤 결정하고 임의로 만들지 마라.

구현 후 수정 파일, 실행 방법, 테스트 결과, 남은 하드웨어 미검증 항목을
보고해줘.

## 요청 끝

