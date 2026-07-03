#include "common/RearDriveCommand.h"

namespace robot {

namespace {

void putI16(std::uint8_t* output, const std::int16_t value) {
  const std::uint16_t raw = static_cast<std::uint16_t>(value);
  output[0] = static_cast<std::uint8_t>(raw & 0xFFU);
  output[1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
}

std::int16_t getI16(const std::uint8_t* input) {
  const std::uint16_t raw =
      static_cast<std::uint16_t>(input[0]) |
      (static_cast<std::uint16_t>(input[1]) << 8U);
  return static_cast<std::int16_t>(raw);
}

void putU32(std::uint8_t* output, const std::uint32_t value) {
  output[0] = static_cast<std::uint8_t>(value & 0xFFU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  output[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  output[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

std::uint32_t getU32(const std::uint8_t* input) {
  return static_cast<std::uint32_t>(input[0]) |
         (static_cast<std::uint32_t>(input[1]) << 8U) |
         (static_cast<std::uint32_t>(input[2]) << 16U) |
         (static_cast<std::uint32_t>(input[3]) << 24U);
}

Milliseconds elapsedSince(const Milliseconds now_ms,
                          const Milliseconds then_ms) {
  return now_ms >= then_ms ? now_ms - then_ms : 0U;
}

}  // namespace

UartPacket makeRearDriveCommandPacket(const RearDriveCommand& command,
                                      const std::uint16_t sequence) {
  UartPacket packet{};
  packet.header.version = kUartProtocolVersion;
  packet.header.message_type = UartMessageType::RearWheelCommand;
  packet.header.sequence = sequence;
  packet.header.payload_size = kRearDrivePayloadSize;
  packet.payload[0] = command.enabled ? kRearDriveEnabledFlag : 0U;
  putI16(&packet.payload[1], clampCommandMilli(command.back_left_command_milli));
  putI16(&packet.payload[3],
         clampCommandMilli(command.back_right_command_milli));
  putU32(&packet.payload[5], command.sender_timestamp_ms);
  putU32(&packet.payload[9], command.timeout_ms);
  packet.header.integrity_crc16 = calculatePacketIntegrity(packet);
  return packet;
}

bool decodeRearDriveCommandPacket(const UartPacket& packet,
                                  RearDriveCommand& command) {
  if (!packetLooksValid(packet) ||
      packet.header.message_type != UartMessageType::RearWheelCommand ||
      packet.header.payload_size != kRearDrivePayloadSize) {
    command = {};
    return false;
  }

  command.enabled = (packet.payload[0] & kRearDriveEnabledFlag) != 0U;
  command.back_left_command_milli =
      clampCommandMilli(getI16(&packet.payload[1]));
  command.back_right_command_milli =
      clampCommandMilli(getI16(&packet.payload[3]));
  command.sender_timestamp_ms = getU32(&packet.payload[5]);
  command.timeout_ms = getU32(&packet.payload[9]);
  if (command.timeout_ms == 0U) {
    command = {};
    return false;
  }
  return true;
}

void RearDriveCommandReceiver::reset() {
  last_command_ = {};
  last_valid_received_at_ms_ = 0U;
  last_sequence_ = 0U;
  has_valid_command_ = false;
  invalid_packets_since_valid_ = 0U;
}

bool RearDriveCommandReceiver::acceptPacket(const UartPacket& packet,
                                            const Milliseconds received_at_ms) {
  RearDriveCommand decoded{};
  if (!decodeRearDriveCommandPacket(packet, decoded)) {
    ++invalid_packets_since_valid_;
    return false;
  }

  if (has_valid_command_ && packet.header.sequence == last_sequence_) {
    ++invalid_packets_since_valid_;
    return false;
  }

  last_command_ = decoded;
  last_valid_received_at_ms_ = received_at_ms;
  last_sequence_ = packet.header.sequence;
  has_valid_command_ = true;
  invalid_packets_since_valid_ = 0U;
  return true;
}

bool RearDriveCommandReceiver::commandIsFresh(
    const Milliseconds now_ms) const {
  return has_valid_command_ &&
         invalid_packets_since_valid_ < kMaxInvalidRearPacketsBeforeStop &&
         elapsedSince(now_ms, last_valid_received_at_ms_) <=
             last_command_.timeout_ms;
}

MotorCommand RearDriveCommandReceiver::motorCommand(
    const std::int16_t duty_command_milli, const Milliseconds now_ms) const {
  if (!commandIsFresh(now_ms) || !last_command_.enabled) {
    return disabledMotorCommand();
  }

  MotorCommand command{};
  command.enabled = duty_command_milli != 0;
  command.duty_command_milli = clampCommandMilli(duty_command_milli);
  command.expires_at_ms =
      last_valid_received_at_ms_ + last_command_.timeout_ms;
  return command;
}

MotorCommand RearDriveCommandReceiver::backLeftCommand(
    const Milliseconds now_ms) const {
  return motorCommand(last_command_.back_left_command_milli, now_ms);
}

MotorCommand RearDriveCommandReceiver::backRightCommand(
    const Milliseconds now_ms) const {
  return motorCommand(last_command_.back_right_command_milli, now_ms);
}

RearDriveStatus RearDriveCommandReceiver::status(
    const Milliseconds now_ms) const {
  RearDriveStatus status{};
  status.link_healthy = commandIsFresh(now_ms);
  status.has_valid_command = has_valid_command_;
  status.command_age_ms =
      has_valid_command_ ? elapsedSince(now_ms, last_valid_received_at_ms_) : 0U;
  status.last_sequence = last_sequence_;
  status.invalid_packets_since_valid = invalid_packets_since_valid_;
  return status;
}

}  // namespace robot
