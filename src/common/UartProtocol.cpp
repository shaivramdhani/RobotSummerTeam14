#include "common/UartProtocol.h"

namespace robot {

namespace {

void putU16(std::uint8_t* output, const std::uint16_t value) {
  output[0] = static_cast<std::uint8_t>(value & 0xFFU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

std::uint8_t messageTypeToByte(const UartMessageType message_type) {
  return static_cast<std::uint8_t>(message_type);
}

}  // namespace

std::uint16_t calculateCrc16Ccitt(const std::uint8_t* data,
                                  const std::size_t size) {
  std::uint16_t crc = 0xFFFFU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
    for (std::uint8_t bit = 0; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U);
      } else {
        crc = static_cast<std::uint16_t>(crc << 1U);
      }
    }
  }
  return crc;
}

std::uint16_t calculatePacketIntegrity(const UartPacket& packet) {
  std::uint8_t bytes[kUartFrameOverheadSize - 4U + kUartMaxPayloadSize]{};
  std::size_t index = 0;
  bytes[index++] = packet.header.version;
  bytes[index++] = messageTypeToByte(packet.header.message_type);
  putU16(&bytes[index], packet.header.sequence);
  index += 2U;
  putU16(&bytes[index], packet.header.payload_size);
  index += 2U;

  for (std::uint16_t payload_index = 0;
       payload_index < packet.header.payload_size; ++payload_index) {
    bytes[index++] = packet.payload[payload_index];
  }

  return calculateCrc16Ccitt(bytes, index);
}

bool packetHeaderLooksValid(const UartPacketHeader& header) {
  return header.version == kUartProtocolVersion &&
         header.payload_size <= kUartMaxPayloadSize;
}

bool packetLooksValid(const UartPacket& packet) {
  return packetHeaderLooksValid(packet.header) &&
         packet.header.integrity_crc16 == calculatePacketIntegrity(packet);
}

bool encodeUartFrame(const UartPacket& packet, std::uint8_t* frame,
                     const std::size_t frame_capacity,
                     std::size_t& frame_size) {
  if (!packetLooksValid(packet)) {
    frame_size = 0U;
    return false;
  }

  const std::size_t required_size =
      kUartFrameOverheadSize + packet.header.payload_size;
  if (frame_capacity < required_size) {
    frame_size = 0U;
    return false;
  }

  frame[0] = kUartFrameMagic0;
  frame[1] = kUartFrameMagic1;
  frame[2] = packet.header.version;
  frame[3] = messageTypeToByte(packet.header.message_type);
  putU16(&frame[4], packet.header.sequence);
  putU16(&frame[6], packet.header.payload_size);
  putU16(&frame[8], packet.header.integrity_crc16);
  for (std::uint16_t index = 0; index < packet.header.payload_size; ++index) {
    frame[kUartFrameOverheadSize + index] = packet.payload[index];
  }

  frame_size = required_size;
  return true;
}

void UartFrameParser::reset() {
  state_ = State::WaitingForMagic0;
  header_bytes_read_ = 0U;
  payload_bytes_read_ = 0U;
  pending_packet_ = {};
}

UartFrameParserStatus UartFrameParser::push(const std::uint8_t byte,
                                            UartPacket& packet) {
  switch (state_) {
    case State::WaitingForMagic0:
      if (byte == kUartFrameMagic0) {
        state_ = State::WaitingForMagic1;
      }
      return UartFrameParserStatus::NeedMoreData;

    case State::WaitingForMagic1:
      if (byte == kUartFrameMagic1) {
        state_ = State::ReadingHeader;
        header_bytes_read_ = 0U;
        pending_packet_ = {};
      } else {
        reset();
      }
      return UartFrameParserStatus::NeedMoreData;

    case State::ReadingHeader: {
      switch (header_bytes_read_) {
        case 0:
          pending_packet_.header.version = byte;
          break;
        case 1:
          pending_packet_.header.message_type =
              static_cast<UartMessageType>(byte);
          break;
        case 2:
          pending_packet_.header.sequence = byte;
          break;
        case 3:
          pending_packet_.header.sequence |=
              static_cast<std::uint16_t>(byte) << 8U;
          break;
        case 4:
          pending_packet_.header.payload_size = byte;
          break;
        case 5:
          pending_packet_.header.payload_size |=
              static_cast<std::uint16_t>(byte) << 8U;
          break;
        case 6:
          pending_packet_.header.integrity_crc16 = byte;
          break;
        case 7:
          pending_packet_.header.integrity_crc16 |=
              static_cast<std::uint16_t>(byte) << 8U;
          break;
        default:
          break;
      }

      ++header_bytes_read_;
      if (header_bytes_read_ < 8U) {
        return UartFrameParserStatus::NeedMoreData;
      }

      if (!packetHeaderLooksValid(pending_packet_.header)) {
        reset();
        return UartFrameParserStatus::InvalidFrame;
      }

      if (pending_packet_.header.payload_size == 0U) {
        packet = pending_packet_;
        const bool valid = packetLooksValid(packet);
        reset();
        return valid ? UartFrameParserStatus::PacketReady
                     : UartFrameParserStatus::InvalidFrame;
      }

      state_ = State::ReadingPayload;
      payload_bytes_read_ = 0U;
      return UartFrameParserStatus::NeedMoreData;
    }

    case State::ReadingPayload:
      pending_packet_.payload[payload_bytes_read_] = byte;
      ++payload_bytes_read_;
      if (payload_bytes_read_ < pending_packet_.header.payload_size) {
        return UartFrameParserStatus::NeedMoreData;
      }

      packet = pending_packet_;
      {
        const bool valid = packetLooksValid(packet);
        reset();
        return valid ? UartFrameParserStatus::PacketReady
                     : UartFrameParserStatus::InvalidFrame;
      }
  }

  reset();
  return UartFrameParserStatus::InvalidFrame;
}

}  // namespace robot
