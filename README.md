# ssacurity-stm32-drive

싸큐리티 자율주행 로봇의 STM32F429 주행 제어 펌웨어 프로젝트입니다.

현재 구현 범위:

- STM32F429I-DISC1 / STM32F429ZIT6
- USART1 115200 8N1
- RX DMA Circular + UART IDLE
- TX DMA Normal + 송신 Ring Buffer
- UART Protocol V1 Frame / CRC16 / Stream Parser
- `CMD_DRIVE`, `CMD_STOP`, `CMD_RESET_FAULT`
- `TELEMETRY_DRIVE`, `FAULT_EVENT`
- 300ms 통신 Timeout 및 재출발 방지
- PC Protocol Echo 검증 도구

통신 규격은 [`docs/uart_protocol_v1.md`](docs/uart_protocol_v1.md)를 기준으로
합니다.

## PC Echo 시험

```powershell
py -m pip install pyserial
py tools\uart_protocol_test.py self-test
py tools\uart_protocol_test.py list
py tools\uart_protocol_test.py echo --port COM5 --text STM32
```

현재 USART1은 보드 내장 ST-LINK VCP를 통한 PC 검증에 사용합니다. 최종
Jetson 연결에서는 CubeMX에 UART5(PC12 TX / PD2 RX)와 DMA를 구성한 뒤
`CommService_Init(&huart1)`을 `CommService_Init(&huart5)`로 변경합니다.

실측 속도·조향 한계를 `CommService_SetDriveLimits()`로 설정하기 전에는
비영점 주행 명령을 거부합니다.
