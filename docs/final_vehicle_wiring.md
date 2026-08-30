# OrinCar 최종 차량 전체 배선 구조도

이 문서는 현재 STM32 펌웨어와 실차에서 확인된 배선을 기준으로 한다.
OrinCar 조립 메뉴얼의 PCA9685/TB6612 Motor HAT 회로는 원래 조립 예시이며,
현재 차량의 최종 제어 회로에는 사용하지 않는다.

## 1. 최종 구성

```text
Jetson -- 3.3 V TTL UART --> STM32F429I-DISC1
                              |-- BTS7960 --> 12 V 구동모터
                              |-- TIM4 <---- 모터 엔코더 A/B
                               |-- TIM3 ----> MG996R 조향서보
                               |-- TIM2 <--> HC-SR04 전방 초음파
                               `-- SPI5 <--> BNO085 IMU
```

현재 조향센서는 장착하지 않았다. PC3/ADC1_IN13은 향후 센서용 예약 핀이며
외부에 연결하지 않는다. 오도메트리 이동거리는 엔코더를 사용하고, 현재 기본
heading은 BNO085 Game Rotation Vector quaternion yaw를 사용한다. IMU heading이
유효하지 않거나 stale이면 실측 7점 조향 LUT와 bicycle model로 자동 복귀한다.

## 2. 전체 전원 구조

```text
                                  +------------------------------+
                                  | Jetson 전용 정격 전원        |
                                  | (캐리어보드 규격 사용)       |
                                  +---------------+--------------+
                                                  |
                                               [Jetson]
                                                  |
                                TX / RX / GND만 연결
                                                  |
                                               [STM32]
                                                  |
                                                  | GND
                                                  v
12 V 배터리 (-) --------------------------- [공통 GND 분배점]
      |                                           | | | | |
      |                                           | | | | +-- Jetson UART GND
      |                                           | | | +---- 엔코더 GND
      |                                           | | +------ HC-SR04 GND
      |                                           | +-------- MG996R GND
      |                                           +---------- BTS7960 GND/B-
      |
12 V 배터리 (+)
      |
   [메인 스위치/퓨즈]
      |
      +-----------------------> BTS7960 B+
      |                              |
      |                         BTS7960 M+/M-
      |                              |
      |                         12 V 구동모터
      |
      `--> [12 V -> 고정 5 V 강압모듈]
                         |
                         +5 V BUS
                          |-- MG996R 빨간선
                          |-- HC-SR04 VCC
                          |-- BTS7960 논리 VCC
                          `-- 엔코더 VCC*
```

`*` 엔코더 VCC는 현재 정상 동작한 기존 전압을 유지한다. 엔코더의 정확한
모델명이나 라벨 전압이 확인되기 전에는 5 V로 임의 변경하지 않는다.

STM32는 ST-LINK USB 또는 보드 규격에 맞는 정격 5 V 전원으로 공급한다.
12 V 배터리를 STM32, Jetson 또는 서보에 직접 연결하지 않는다. Jetson은
캐리어보드가 요구하는 별도 정격 전원을 사용하며, 현재 5 V 강압모듈로
Jetson까지 공급한다고 가정하지 않는다.

## 3. STM32 신호 배선표

| 기능 | STM32F429I-DISC1 | Peripheral | 연결 대상 |
| --- | --- | --- | --- |
| Jetson 수신 | PC12 / P1-43 | UART5_TX / AF8 | Jetson RX |
| Jetson 송신 | PD2 / P1-40 | UART5_RX / AF8 | Jetson TX |
| UART 기준 접지 | P1-63 또는 P1-64 | GND | Jetson GND |
| 모터 정방향 PWM | PA0 / P2-18 | TIM5_CH1 / 20 kHz | BTS7960 RPWM |
| 모터 역방향 PWM | PA3 / P2-19 | TIM5_CH4 / 20 kHz | BTS7960 LPWM |
| 모터 정방향 Enable | PE2 / P1-15 | GPIO output | BTS7960 R_EN |
| 모터 역방향 Enable | PE3 / P1-16 | GPIO output | BTS7960 L_EN |
| 엔코더 A상 | PB6 / P1-23 | TIM4_CH1 | Encoder A |
| 엔코더 B상 | PB7 / P1-24 | TIM4_CH2 | Encoder B |
| 서보 PWM | PB4 / P1-25 | TIM3_CH1 / 50 Hz | MG996R 주황/노랑 |
| 초음파 TRIG | PA5 / P2-21 | GPIO output | HC-SR04 TRIG |
| 초음파 ECHO | PB3 / P1-28 | TIM2_CH2 / AF1 | HC-SR04 ECHO |
| 향후 조향센서 | PC3 / P2-15 | ADC1_IN13 | 현재 미연결 |
| IMU clock | PF7 / P2-6 | SPI5_SCK | BNO085 SCL |
| IMU data out | PF8 / P2-5 | SPI5_MISO | BNO085 SDA |
| IMU data in | PF9 / P2-8 | SPI5_MOSI | BNO085 ADO |
| IMU chip select | PF6 / P2-3 | GPIO output | BNO085 CS |
| IMU data ready | PG3 / P2-61 | EXTI3 input | BNO085 INT |
| IMU reset | PG2 / P2-62 | GPIO output | BNO085 RST |

P1/P2 번호는 STM32F429I-DISC1 공식 UM1670과 MB1075 회로도 기준이다.

## 4. 장치별 커넥터 배선

### Jetson - STM32 UART5

```text
Jetson TX  ------------------> STM32 PD2 / P1-40 / UART5_RX
Jetson RX  <------------------ STM32 PC12 / P1-43 / UART5_TX
Jetson GND ------------------- STM32 GND / P1-63 또는 P1-64
Jetson VCC ------------------- 연결하지 않음
```

- 115200 bps, 8-N-1, flow control 없음
- 3.3 V TTL UART만 사용한다. RS-232 또는 5 V UART를 연결하지 않는다.
- Jetson 직결 UART와 USB-UART TX를 동시에 PD2에 연결하지 않는다.
- Jetson 헤더의 실제 TX/RX 핀 번호와 `/dev/tty...` 이름은 캐리어보드
  모델을 확인한 뒤 확정한다.

### BTS7960

```text
STM32 PA0 / P2-18 ---- RPWM
STM32 PA3 / P2-19 ---- LPWM
STM32 PE2 / P1-15 ---- R_EN
STM32 PE3 / P1-16 ---- L_EN
STM32 GND ------------- GND
5 V logic bus --------- VCC

12 V battery (+) ------ B+
12 V battery (-) ------ B-
Motor terminal 1 ------ M+
Motor terminal 2 ------ M-
```

R_IS와 L_IS는 현재 펌웨어에서 읽지 않으므로 연결하지 않는다. 모터 방향이
차량 기준과 반대라면 신호 핀을 임의로 바꾸지 말고 모터 M+/M- 극성과
엔코더 부호를 함께 재검증한다.

### 엔코더

```text
Encoder A ------------- STM32 PB6 / P1-23 / TIM4_CH1
Encoder B ------------- STM32 PB7 / P1-24 / TIM4_CH2
Encoder GND ----------- 공통 GND
Encoder VCC ----------- 현재 검증된 센서 전압 유지
```

현재 보정값은 823 count/wheel-revolution이며 정방향 count가 양수다.

### MG996R

```text
MG996R 빨간선 --------- 강압모듈 +5 V
MG996R 갈색/검정 ------ 공통 GND
MG996R 주황/노랑 ------ STM32 PB4 / P1-25 / TIM3_CH1
```

서보 전류가 STM32 보드의 5 V/GND 배선을 통과하지 않게 전원선은 강압모듈과
GND 분배점으로 직접 연결한다. 현재 LUT는 766, 921, 1076, 1231, 1386,
1541, 1696 us이며 직진 보정값은 1231 us다.

| 등가 중심 조향각 | Servo pulse |
| ---: | ---: |
| +19.55도 | 766 us |
| +14.18도 | 921 us |
| +10.27도 | 1076 us |
| 0도 | 1231 us |
| -7.09도 | 1386 us |
| -17.56도 | 1541 us |
| -28.69도 | 1696 us |

### HC-SR04

```text
HC-SR04 VCC ----------- +5 V
HC-SR04 GND ----------- 공통 GND
HC-SR04 TRIG ---------- STM32 PA5 / P2-21
HC-SR04 ECHO ---------- STM32 PB3 / P1-28 / TIM2_CH2
```

PB3는 STM32F429ZI의 5 V tolerant 디지털 I/O다. STM32와 센서가 함께
전원 공급되고 공통 GND가 연결된 현재 디지털 입력 조건에서는 ECHO를 직접
연결한다. 이 판단은 다른 핀이나 아날로그 모드에 일반화하지 않는다.

### BNO085

```text
BNO085 VCC ----------- STM32 3.3 V
BNO085 GND ----------- STM32 GND
BNO085 SCL ----------- STM32 PF7 / P2-6 / SPI5_SCK
BNO085 SDA ----------- STM32 PF8 / P2-5 / SPI5_MISO
BNO085 ADO ----------- STM32 PF9 / P2-8 / SPI5_MOSI
BNO085 CS ------------ STM32 PF6 / P2-3
BNO085 INT ----------- STM32 PG3 / P2-61
BNO085 RST ----------- STM32 PG2 / P2-62
BNO085 PS1 ----------- 3.3 V
BNO085 PS0 ----------- 3.3 V
```

온보드 L3GD20과 SPI5 SCK/MISO/MOSI를 공유하므로 L3GD20 CS인 PC1은 항상
HIGH로 유지한다. BNO085는 차체에 단단히 고정하고 기본 축은 X 전방, Y 왼쪽,
Z 위쪽으로 맞춘다. 상세 시험 절차는 `docs/bno085_spi5_integration.md`를 따른다.

## 5. 배선 정리 순서

1. 12 V 배터리와 STM32/Jetson 전원을 모두 끈다.
2. 굵은 전력선(배터리-BTS7960-모터)을 차체 한쪽으로 묶는다.
3. UART, 엔코더, PWM, 초음파 신호선은 모터 전력선에서 떨어뜨린다.
4. 공통 GND 분배점을 먼저 만들고 각 장치 GND를 개별 분기한다.
5. TX/RX 교차와 P1/P2 번호를 한 선씩 대조한다.
6. 모터 전원을 넣기 전에 STM32와 Jetson UART Echo를 먼저 시험한다.
7. 차체를 뒤집어 바퀴를 공중에 둔 뒤 neutral, 저속, 조향 순으로 시험한다.

## 6. 아직 실물 확인이 필요한 항목

- Jetson 정확한 모델과 캐리어보드의 UART 헤더 번호
- Jetson에서 사용할 `/dev/tty...` 장치명과 콘솔 점유 해제 여부
- 엔코더 정확한 모델명과 정격 VCC
- 5 V 강압모듈의 연속/순간 허용 전류
- 12 V 모터의 정지전류에 맞는 퓨즈 정격과 케이블 굵기
- 물리 E-stop 스위치와 메인 전원 차단 위치

## 7. 근거

- `ssacurity-stm32-drive.ioc`
- `Core/Src/main.c`
- `Core/Src/stm32f4xx_hal_msp.c`
- `Core/Src/stm32f4xx_it.c`
- STM32F429I-DISC1 UM1670, extension connector Table 7
- STM32F429I-DISC1 MB1075 schematic pack
- `OrinCar_Manual (2).pdf`의 기구 조립 구조
