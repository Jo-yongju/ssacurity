#include "comm_protocol.h"

#include <string.h>

enum
{
  PARSER_WAIT_SOF1 = 0,
  PARSER_WAIT_SOF2,
  PARSER_READ_VERSION,
  PARSER_READ_MESSAGE_ID,
  PARSER_READ_SEQUENCE,
  PARSER_READ_LENGTH,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC_LOW,
  PARSER_READ_CRC_HIGH
};

static uint16_t CrcUpdate(uint16_t crc, uint8_t byte)
{
  crc ^= (uint16_t)byte << 8;
  for (uint8_t bit = 0U; bit < 8U; bit++)
  {
    if ((crc & 0x8000U) != 0U)
    {
      crc = (uint16_t)((crc << 1) ^ 0x1021U);
    }
    else
    {
      crc <<= 1;
    }
  }
  return crc;
}

static void ResetForNextFrame(CommParser *parser)
{
  parser->state = PARSER_WAIT_SOF1;
  parser->payload_index = 0U;
  parser->calculated_crc = 0xFFFFU;
  parser->received_crc = 0U;
}

void CommProtocol_ParserInit(CommParser *parser)
{
  if (parser == NULL)
  {
    return;
  }

  memset(parser, 0, sizeof(*parser));
  ResetForNextFrame(parser);
}

CommParseResult CommProtocol_ParserPush(CommParser *parser,
                                        uint8_t byte,
                                        CommFrame *completed_frame)
{
  CommParseResult result = COMM_PARSE_NONE;

  if (parser == NULL)
  {
    return COMM_PARSE_NONE;
  }

  switch (parser->state)
  {
    case PARSER_WAIT_SOF1:
      if (byte == COMM_PROTOCOL_SOF1)
      {
        parser->state = PARSER_WAIT_SOF2;
      }
      else
      {
        parser->stats.discarded_bytes++;
      }
      break;

    case PARSER_WAIT_SOF2:
      if (byte == COMM_PROTOCOL_SOF2)
      {
        parser->state = PARSER_READ_VERSION;
        parser->calculated_crc = 0xFFFFU;
      }
      else if (byte != COMM_PROTOCOL_SOF1)
      {
        parser->stats.discarded_bytes++;
        parser->state = PARSER_WAIT_SOF1;
      }
      break;

    case PARSER_READ_VERSION:
      parser->frame.version = byte;
      parser->calculated_crc = CrcUpdate(parser->calculated_crc, byte);
      parser->state = PARSER_READ_MESSAGE_ID;
      break;

    case PARSER_READ_MESSAGE_ID:
      parser->frame.message_id = byte;
      parser->calculated_crc = CrcUpdate(parser->calculated_crc, byte);
      parser->state = PARSER_READ_SEQUENCE;
      break;

    case PARSER_READ_SEQUENCE:
      parser->frame.sequence = byte;
      parser->calculated_crc = CrcUpdate(parser->calculated_crc, byte);
      parser->state = PARSER_READ_LENGTH;
      break;

    case PARSER_READ_LENGTH:
      parser->frame.payload_length = byte;
      parser->calculated_crc = CrcUpdate(parser->calculated_crc, byte);
      parser->payload_index = 0U;

      if (byte > COMM_PROTOCOL_MAX_PAYLOAD)
      {
        parser->stats.length_errors++;
        ResetForNextFrame(parser);
        result = COMM_PARSE_LENGTH_ERROR;
      }
      else if (byte == 0U)
      {
        parser->state = PARSER_READ_CRC_LOW;
      }
      else
      {
        parser->state = PARSER_READ_PAYLOAD;
      }
      break;

    case PARSER_READ_PAYLOAD:
      parser->frame.payload[parser->payload_index] = byte;
      parser->payload_index++;
      parser->calculated_crc = CrcUpdate(parser->calculated_crc, byte);

      if (parser->payload_index >= parser->frame.payload_length)
      {
        parser->state = PARSER_READ_CRC_LOW;
      }
      break;

    case PARSER_READ_CRC_LOW:
      parser->received_crc = byte;
      parser->state = PARSER_READ_CRC_HIGH;
      break;

    case PARSER_READ_CRC_HIGH:
      parser->received_crc |= (uint16_t)byte << 8;

      if (parser->received_crc != parser->calculated_crc)
      {
        parser->stats.crc_errors++;
        result = COMM_PARSE_CRC_ERROR;
      }
      else if (parser->frame.version != COMM_PROTOCOL_VERSION)
      {
        parser->stats.version_errors++;
        result = COMM_PARSE_VERSION_ERROR;
      }
      else
      {
        parser->stats.valid_frames++;
        if (completed_frame != NULL)
        {
          *completed_frame = parser->frame;
        }
        result = COMM_PARSE_FRAME_READY;
      }

      ResetForNextFrame(parser);
      break;

    default:
      ResetForNextFrame(parser);
      break;
  }

  return result;
}

uint16_t CommProtocol_Crc16CcittFalse(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;

  if ((data == NULL) && (length > 0U))
  {
    return crc;
  }

  for (size_t index = 0U; index < length; index++)
  {
    crc = CrcUpdate(crc, data[index]);
  }

  return crc;
}

size_t CommProtocol_EncodeFrame(uint8_t message_id,
                                uint8_t sequence,
                                const uint8_t *payload,
                                uint8_t payload_length,
                                uint8_t *output,
                                size_t output_capacity)
{
  size_t frame_length;
  uint16_t crc;

  if ((output == NULL) ||
      ((payload == NULL) && (payload_length > 0U)) ||
      (payload_length > COMM_PROTOCOL_MAX_PAYLOAD))
  {
    return 0U;
  }

  frame_length = COMM_PROTOCOL_FRAME_OVERHEAD + payload_length;
  if (output_capacity < frame_length)
  {
    return 0U;
  }

  output[0] = COMM_PROTOCOL_SOF1;
  output[1] = COMM_PROTOCOL_SOF2;
  output[2] = COMM_PROTOCOL_VERSION;
  output[3] = message_id;
  output[4] = sequence;
  output[5] = payload_length;

  if (payload_length > 0U)
  {
    memcpy(&output[6], payload, payload_length);
  }

  crc = CommProtocol_Crc16CcittFalse(&output[2],
                                     (size_t)payload_length + 4U);
  output[6U + payload_length] = (uint8_t)(crc & 0xFFU);
  output[7U + payload_length] = (uint8_t)(crc >> 8);

  return frame_length;
}
