#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define COMM_PROTOCOL_SOF1              0xAAU
#define COMM_PROTOCOL_SOF2              0x55U
#define COMM_PROTOCOL_VERSION           0x01U
#define COMM_PROTOCOL_MAX_PAYLOAD       64U
#define COMM_PROTOCOL_FRAME_OVERHEAD    8U
#define COMM_PROTOCOL_MAX_FRAME_SIZE    \
  (COMM_PROTOCOL_FRAME_OVERHEAD + COMM_PROTOCOL_MAX_PAYLOAD)

#define COMM_COMMAND_TIMEOUT_MS         300U
#define COMM_DIAG_ECHO_MAX_PAYLOAD      32U

typedef enum
{
  COMM_MSG_CMD_DRIVE = 0x10,
  COMM_MSG_CMD_STOP = 0x11,
  COMM_MSG_CMD_RESET_FAULT = 0x12,

  COMM_MSG_TELEMETRY_DRIVE = 0x80,
  COMM_MSG_FAULT_EVENT = 0x81,
  COMM_MSG_TELEMETRY_SENSOR_RESERVED = 0x82,

  COMM_MSG_DIAG_ECHO_REQUEST = 0xF0,
  COMM_MSG_DIAG_ECHO_RESPONSE = 0xF1
} CommMessageId;

typedef enum
{
  COMM_STOP_REASON_OPERATOR = 0,
  COMM_STOP_REASON_MISSION_COMPLETE = 1,
  COMM_STOP_REASON_OBSTACLE = 2,
  COMM_STOP_REASON_REMOTE_REQUEST = 3,
  COMM_STOP_REASON_INTERNAL = 4
} CommStopReason;

typedef enum
{
  COMM_STATE_BOOT = 0,
  COMM_STATE_SELF_TEST = 1,
  COMM_STATE_READY = 2,
  COMM_STATE_DRIVING = 3,
  COMM_STATE_SAFE_STOP = 4,
  COMM_STATE_FAULT = 5
} CommSystemState;

typedef enum
{
  COMM_FAULT_COMM_TIMEOUT = (1UL << 0),
  COMM_FAULT_CRC_ERROR = (1UL << 1),
  COMM_FAULT_BAD_COMMAND = (1UL << 2),
  COMM_FAULT_ENCODER_INVALID = (1UL << 3),
  COMM_FAULT_MOTOR_STALL = (1UL << 4),
  COMM_FAULT_DIRECTION = (1UL << 5),
  COMM_FAULT_CONTROL_OVERRUN = (1UL << 6),
  COMM_FAULT_IMU_LOST = (1UL << 7),
  COMM_FAULT_RANGE_LOST = (1UL << 8),
  COMM_FAULT_STEERING_INVALID = (1UL << 9),
  COMM_FAULT_ESTOP_ACTIVE = (1UL << 10),
  COMM_FAULT_INTERNAL = (1UL << 11)
} CommFaultBit;

typedef struct
{
  int16_t target_speed_mm_s;
  int16_t target_steering_cdeg;
  bool drive_enable;
} CommDriveCommand;

typedef struct
{
  CommStopReason reason;
} CommStopCommand;

typedef struct
{
  uint32_t acknowledged_fault_bits;
} CommResetFaultCommand;

typedef struct
{
  int16_t target_speed_mm_s;
  int16_t measured_speed_mm_s;
  int32_t encoder_count;
  int16_t motor_duty_permille;
  int16_t steering_cdeg;
  CommSystemState state;
  uint32_t active_fault_bits;
  uint32_t uptime_ms;
} CommDriveTelemetry;

typedef struct
{
  uint32_t active_fault_bits;
  uint32_t latched_fault_bits;
  uint32_t occurred_at_ms;
} CommFaultEvent;

typedef struct
{
  uint8_t version;
  uint8_t message_id;
  uint8_t sequence;
  uint8_t payload_length;
  uint8_t payload[COMM_PROTOCOL_MAX_PAYLOAD];
} CommFrame;

typedef enum
{
  COMM_PARSE_NONE = 0,
  COMM_PARSE_FRAME_READY,
  COMM_PARSE_CRC_ERROR,
  COMM_PARSE_LENGTH_ERROR,
  COMM_PARSE_VERSION_ERROR
} CommParseResult;

typedef struct
{
  uint32_t valid_frames;
  uint32_t crc_errors;
  uint32_t length_errors;
  uint32_t version_errors;
  uint32_t discarded_bytes;
} CommParserStats;

typedef struct
{
  uint8_t state;
  uint8_t payload_index;
  uint16_t calculated_crc;
  uint16_t received_crc;
  CommFrame frame;
  CommParserStats stats;
} CommParser;

void CommProtocol_ParserInit(CommParser *parser);
CommParseResult CommProtocol_ParserPush(CommParser *parser,
                                        uint8_t byte,
                                        CommFrame *completed_frame);

uint16_t CommProtocol_Crc16CcittFalse(const uint8_t *data, size_t length);
size_t CommProtocol_EncodeFrame(uint8_t message_id,
                                uint8_t sequence,
                                const uint8_t *payload,
                                uint8_t payload_length,
                                uint8_t *output,
                                size_t output_capacity);

#ifdef __cplusplus
}
#endif

#endif
