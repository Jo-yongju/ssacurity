#ifndef COMM_SERVICE_H
#define COMM_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "comm_protocol.h"
#include "uart_transport.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint32_t accepted_commands;
  uint32_t duplicate_or_old_sequences;
  uint32_t invalid_payloads;
  uint32_t unknown_message_ids;
  uint32_t unsafe_rearm_rejections;
  uint32_t unconfigured_limit_rejections;
  uint32_t command_timeouts;
  uint32_t echo_requests;
  uint32_t response_queue_failures;
} CommServiceStats;

bool CommService_Init(UART_HandleTypeDef *uart);
void CommService_Process(void);

/*
 * Non-zero drive commands remain blocked until measured vehicle limits are
 * provided. Passing zero for either limit returns the service to the safe,
 * neutral-command-only state.
 */
void CommService_SetDriveLimits(int16_t max_abs_speed_mm_s,
                                int16_t max_abs_steering_cdeg);

bool CommService_TakeDriveCommand(CommDriveCommand *command);
bool CommService_TakeStopCommand(CommStopCommand *command);
bool CommService_TakeResetFaultCommand(CommResetFaultCommand *command);

bool CommService_IsCommandTimedOut(void);
bool CommService_IsRearmRequired(void);

bool CommService_SendDriveTelemetry(const CommDriveTelemetry *telemetry);
bool CommService_SendFaultEvent(const CommFaultEvent *fault_event);

void CommService_OnUartRxEvent(UART_HandleTypeDef *uart, uint16_t size);
void CommService_OnUartTxComplete(UART_HandleTypeDef *uart);
void CommService_OnUartError(UART_HandleTypeDef *uart);

void CommService_GetStats(CommServiceStats *service_stats,
                          CommParserStats *parser_stats,
                          UartTransportStats *transport_stats);

#ifdef __cplusplus
}
#endif

#endif
