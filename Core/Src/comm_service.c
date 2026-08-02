#include "comm_service.h"

#include <limits.h>
#include <string.h>

#define CMD_DRIVE_PAYLOAD_LENGTH       5U
#define CMD_STOP_PAYLOAD_LENGTH        1U
#define CMD_RESET_PAYLOAD_LENGTH       4U
#define DRIVE_TELEMETRY_PAYLOAD_LENGTH 21U
#define FAULT_EVENT_PAYLOAD_LENGTH     12U

typedef struct
{
  UartTransport transport;
  CommParser parser;

  bool initialized;
  uint8_t tx_sequence;
  bool rx_sequence_valid;
  uint8_t last_rx_sequence;

  bool drive_limits_configured;
  int16_t max_abs_speed_mm_s;
  int16_t max_abs_steering_cdeg;

  CommDriveCommand drive_command;
  CommStopCommand stop_command;
  CommResetFaultCommand reset_fault_command;
  bool drive_command_pending;
  bool stop_command_pending;
  bool reset_fault_command_pending;

  bool command_timed_out;
  bool rearm_required;
  uint32_t last_valid_drive_tick;

  CommServiceStats stats;
} CommServiceContext;

static CommServiceContext service;

static uint16_t ReadU16LittleEndian(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t ReadI16LittleEndian(const uint8_t *data)
{
  return (int16_t)ReadU16LittleEndian(data);
}

static uint32_t ReadU32LittleEndian(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static void WriteU16LittleEndian(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8);
}

static void WriteU32LittleEndian(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
  data[2] = (uint8_t)((value >> 16) & 0xFFU);
  data[3] = (uint8_t)(value >> 24);
}

static bool ValueWithinAbsoluteLimit(int16_t value, int16_t limit)
{
  int32_t wide_value = value;
  int32_t wide_limit = limit;

  if (wide_value < 0)
  {
    wide_value = -wide_value;
  }

  return wide_value <= wide_limit;
}

static void SetNeutralDriveCommand(void)
{
  service.drive_command.target_speed_mm_s = 0;
  service.drive_command.target_steering_cdeg = 0;
  service.drive_command.drive_enable = false;
  service.drive_command_pending = true;
}

static bool SequenceIsNew(uint8_t sequence)
{
  uint8_t difference;

  if (!service.rx_sequence_valid)
  {
    service.last_rx_sequence = sequence;
    service.rx_sequence_valid = true;
    return true;
  }

  difference = (uint8_t)(sequence - service.last_rx_sequence);
  if ((difference == 0U) || (difference >= 128U))
  {
    service.stats.duplicate_or_old_sequences++;
    return false;
  }

  service.last_rx_sequence = sequence;
  return true;
}

static bool QueueFrameWithSequence(uint8_t message_id,
                                   uint8_t sequence,
                                   const uint8_t *payload,
                                   uint8_t payload_length)
{
  uint8_t frame[COMM_PROTOCOL_MAX_FRAME_SIZE];
  size_t frame_length;

  frame_length = CommProtocol_EncodeFrame(message_id,
                                          sequence,
                                          payload,
                                          payload_length,
                                          frame,
                                          sizeof(frame));
  if ((frame_length == 0U) ||
      !UartTransport_Write(&service.transport, frame, frame_length))
  {
    service.stats.response_queue_failures++;
    return false;
  }

  return true;
}

static bool QueueFrame(uint8_t message_id,
                       const uint8_t *payload,
                       uint8_t payload_length)
{
  uint8_t sequence = service.tx_sequence;

  if (!QueueFrameWithSequence(message_id,
                              sequence,
                              payload,
                              payload_length))
  {
    return false;
  }

  service.tx_sequence++;
  return true;
}

static void HandleDriveCommand(const CommFrame *frame)
{
  CommDriveCommand command;
  bool neutral_command;

  if (frame->payload_length != CMD_DRIVE_PAYLOAD_LENGTH)
  {
    service.stats.invalid_payloads++;
    return;
  }

  command.target_speed_mm_s = ReadI16LittleEndian(&frame->payload[0]);
  command.target_steering_cdeg = ReadI16LittleEndian(&frame->payload[2]);

  if (frame->payload[4] > 1U)
  {
    service.stats.invalid_payloads++;
    return;
  }
  command.drive_enable = frame->payload[4] != 0U;

  neutral_command = !command.drive_enable &&
                    (command.target_speed_mm_s == 0) &&
                    (command.target_steering_cdeg == 0);

  if (service.rearm_required && !neutral_command)
  {
    service.stats.unsafe_rearm_rejections++;
    return;
  }

  if (!neutral_command)
  {
    if (!service.drive_limits_configured)
    {
      service.stats.unconfigured_limit_rejections++;
      return;
    }

    if (!command.drive_enable ||
        !ValueWithinAbsoluteLimit(command.target_speed_mm_s,
                                  service.max_abs_speed_mm_s) ||
        !ValueWithinAbsoluteLimit(command.target_steering_cdeg,
                                  service.max_abs_steering_cdeg))
    {
      service.stats.invalid_payloads++;
      return;
    }
  }

  if (!SequenceIsNew(frame->sequence))
  {
    return;
  }

  service.drive_command = command;
  service.drive_command_pending = true;
  service.last_valid_drive_tick = HAL_GetTick();
  service.command_timed_out = false;
  service.rearm_required = false;
  service.stats.accepted_commands++;
}

static void HandleStopCommand(const CommFrame *frame)
{
  if ((frame->payload_length != CMD_STOP_PAYLOAD_LENGTH) ||
      (frame->payload[0] > COMM_STOP_REASON_INTERNAL))
  {
    service.stats.invalid_payloads++;
    return;
  }

  if (!SequenceIsNew(frame->sequence))
  {
    return;
  }

  service.stop_command.reason = (CommStopReason)frame->payload[0];
  service.stop_command_pending = true;
  service.rearm_required = true;
  SetNeutralDriveCommand();
  service.stats.accepted_commands++;
}

static void HandleResetFaultCommand(const CommFrame *frame)
{
  if (frame->payload_length != CMD_RESET_PAYLOAD_LENGTH)
  {
    service.stats.invalid_payloads++;
    return;
  }

  if (!SequenceIsNew(frame->sequence))
  {
    return;
  }

  service.reset_fault_command.acknowledged_fault_bits =
      ReadU32LittleEndian(frame->payload);
  service.reset_fault_command_pending = true;
  service.stats.accepted_commands++;
}

static void HandleEchoRequest(const CommFrame *frame)
{
  if (frame->payload_length > COMM_DIAG_ECHO_MAX_PAYLOAD)
  {
    service.stats.invalid_payloads++;
    return;
  }

  if (!SequenceIsNew(frame->sequence))
  {
    return;
  }

  if (QueueFrameWithSequence(COMM_MSG_DIAG_ECHO_RESPONSE,
                             frame->sequence,
                             frame->payload,
                             frame->payload_length))
  {
    service.stats.echo_requests++;
  }
}

static void HandleFrame(const CommFrame *frame)
{
  switch (frame->message_id)
  {
    case COMM_MSG_CMD_DRIVE:
      HandleDriveCommand(frame);
      break;

    case COMM_MSG_CMD_STOP:
      HandleStopCommand(frame);
      break;

    case COMM_MSG_CMD_RESET_FAULT:
      HandleResetFaultCommand(frame);
      break;

    case COMM_MSG_DIAG_ECHO_REQUEST:
      HandleEchoRequest(frame);
      break;

    default:
      service.stats.unknown_message_ids++;
      break;
  }
}

bool CommService_Init(UART_HandleTypeDef *uart)
{
  memset(&service, 0, sizeof(service));
  CommProtocol_ParserInit(&service.parser);

  service.command_timed_out = true;
  service.rearm_required = true;

  if (!UartTransport_Init(&service.transport, uart))
  {
    return false;
  }

  service.initialized = true;
  return true;
}

void CommService_Process(void)
{
  uint8_t received[64];
  size_t count;
  uint32_t now;

  if (!service.initialized)
  {
    return;
  }

  UartTransport_Process(&service.transport);

  do
  {
    count = UartTransport_Read(&service.transport,
                               received,
                               sizeof(received));
    for (size_t index = 0U; index < count; index++)
    {
      CommFrame frame;
      CommParseResult result =
          CommProtocol_ParserPush(&service.parser, received[index], &frame);

      if (result == COMM_PARSE_FRAME_READY)
      {
        HandleFrame(&frame);
      }
    }
  } while (count == sizeof(received));

  if (!service.command_timed_out)
  {
    now = HAL_GetTick();
    if ((uint32_t)(now - service.last_valid_drive_tick) >
        COMM_COMMAND_TIMEOUT_MS)
    {
      service.command_timed_out = true;
      service.rearm_required = true;
      service.rx_sequence_valid = false;
      SetNeutralDriveCommand();
      service.stats.command_timeouts++;
    }
  }

  UartTransport_Process(&service.transport);
}

void CommService_SetDriveLimits(int16_t max_abs_speed_mm_s,
                                int16_t max_abs_steering_cdeg)
{
  if ((max_abs_speed_mm_s <= 0) || (max_abs_steering_cdeg <= 0))
  {
    service.drive_limits_configured = false;
    service.max_abs_speed_mm_s = 0;
    service.max_abs_steering_cdeg = 0;
    return;
  }

  service.max_abs_speed_mm_s = max_abs_speed_mm_s;
  service.max_abs_steering_cdeg = max_abs_steering_cdeg;
  service.drive_limits_configured = true;
}

bool CommService_TakeDriveCommand(CommDriveCommand *command)
{
  if ((command == NULL) || !service.drive_command_pending)
  {
    return false;
  }

  *command = service.drive_command;
  service.drive_command_pending = false;
  return true;
}

bool CommService_TakeStopCommand(CommStopCommand *command)
{
  if ((command == NULL) || !service.stop_command_pending)
  {
    return false;
  }

  *command = service.stop_command;
  service.stop_command_pending = false;
  return true;
}

bool CommService_TakeResetFaultCommand(CommResetFaultCommand *command)
{
  if ((command == NULL) || !service.reset_fault_command_pending)
  {
    return false;
  }

  *command = service.reset_fault_command;
  service.reset_fault_command_pending = false;
  return true;
}

bool CommService_IsCommandTimedOut(void)
{
  return service.command_timed_out;
}

bool CommService_IsRearmRequired(void)
{
  return service.rearm_required;
}

bool CommService_SendDriveTelemetry(const CommDriveTelemetry *telemetry)
{
  uint8_t payload[DRIVE_TELEMETRY_PAYLOAD_LENGTH];

  if ((telemetry == NULL) || !service.initialized)
  {
    return false;
  }

  WriteU16LittleEndian(&payload[0], (uint16_t)telemetry->target_speed_mm_s);
  WriteU16LittleEndian(&payload[2], (uint16_t)telemetry->measured_speed_mm_s);
  WriteU32LittleEndian(&payload[4], (uint32_t)telemetry->encoder_count);
  WriteU16LittleEndian(&payload[8], (uint16_t)telemetry->motor_duty_permille);
  WriteU16LittleEndian(&payload[10], (uint16_t)telemetry->steering_cdeg);
  payload[12] = (uint8_t)telemetry->state;
  WriteU32LittleEndian(&payload[13], telemetry->active_fault_bits);
  WriteU32LittleEndian(&payload[17], telemetry->uptime_ms);

  return QueueFrame(COMM_MSG_TELEMETRY_DRIVE,
                    payload,
                    sizeof(payload));
}

bool CommService_SendFaultEvent(const CommFaultEvent *fault_event)
{
  uint8_t payload[FAULT_EVENT_PAYLOAD_LENGTH];

  if ((fault_event == NULL) || !service.initialized)
  {
    return false;
  }

  WriteU32LittleEndian(&payload[0], fault_event->active_fault_bits);
  WriteU32LittleEndian(&payload[4], fault_event->latched_fault_bits);
  WriteU32LittleEndian(&payload[8], fault_event->occurred_at_ms);

  return QueueFrame(COMM_MSG_FAULT_EVENT,
                    payload,
                    sizeof(payload));
}

void CommService_OnUartRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
  UartTransport_OnRxEvent(&service.transport, uart, size);
}

void CommService_OnUartTxComplete(UART_HandleTypeDef *uart)
{
  UartTransport_OnTxComplete(&service.transport, uart);
}

void CommService_OnUartError(UART_HandleTypeDef *uart)
{
  UartTransport_OnError(&service.transport, uart);
}

void CommService_GetStats(CommServiceStats *service_stats,
                          CommParserStats *parser_stats,
                          UartTransportStats *transport_stats)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (service_stats != NULL)
  {
    *service_stats = service.stats;
  }
  if (parser_stats != NULL)
  {
    *parser_stats = service.parser.stats;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (transport_stats != NULL)
  {
    UartTransport_GetStats(&service.transport, transport_stats);
  }
}
