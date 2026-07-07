#include "common/Esp1Status.h"

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

std::uint8_t modeToByte(const RobotTestMode mode) {
  return static_cast<std::uint8_t>(mode);
}

RobotTestMode byteToMode(const std::uint8_t value) {
  switch (static_cast<RobotTestMode>(value)) {
    case RobotTestMode::Disabled:
    case RobotTestMode::SensorMonitor:
    case RobotTestMode::SingleMotorTest:
    case RobotTestMode::ManualDriveTest:
    case RobotTestMode::DistributedDriveTest:
    case RobotTestMode::LineSensorTest:
    case RobotTestMode::LineFollowTest:
    case RobotTestMode::MechanismTest:
    case RobotTestMode::AutonomousDryRun:
      return static_cast<RobotTestMode>(value);
  }
  return RobotTestMode::Disabled;
}

FaultCode byteToFaultCode(const std::uint8_t value) {
  switch (static_cast<FaultCode>(value)) {
    case FaultCode::None:
    case FaultCode::CommunicationStale:
    case FaultCode::InvalidCommand:
    case FaultCode::LimitSwitchConflict:
    case FaultCode::HardwareNotConfigured:
      return static_cast<FaultCode>(value);
  }
  return FaultCode::InvalidCommand;
}

}  // namespace

UartPacket makeEsp1StatusPacket(const Esp1StatusReport& report,
                                const std::uint16_t sequence) {
  UartPacket packet{};
  packet.header.version = kUartProtocolVersion;
  packet.header.message_type = UartMessageType::HealthReport;
  packet.header.sequence = sequence;
  packet.header.payload_size = kEsp1StatusPayloadSize;
  putU32(&packet.payload[0], report.uptime_ms);
  packet.payload[4] = modeToByte(report.mode);
  packet.payload[5] = static_cast<std::uint8_t>(report.fault_code);
  putI16(&packet.payload[6],
         clampCommandMilli(report.back_left_applied_command_milli));
  putI16(&packet.payload[8],
         clampCommandMilli(report.back_right_applied_command_milli));
  std::uint8_t flags = 0U;
  flags |= report.fault_active ? kEsp1StatusFaultActiveFlag : 0U;
  flags |= report.back_left_inverted ? kEsp1StatusBackLeftInvertedFlag : 0U;
  flags |= report.back_right_inverted ? kEsp1StatusBackRightInvertedFlag : 0U;
  packet.payload[10] = flags;
  packet.header.integrity_crc16 = calculatePacketIntegrity(packet);
  return packet;
}

bool decodeEsp1StatusPacket(const UartPacket& packet,
                            Esp1StatusReport& report) {
  if (!packetLooksValid(packet) ||
      packet.header.message_type != UartMessageType::HealthReport ||
      packet.header.payload_size != kEsp1StatusPayloadSize) {
    report = {};
    return false;
  }

  report.uptime_ms = getU32(&packet.payload[0]);
  report.mode = byteToMode(packet.payload[4]);
  report.fault_code = byteToFaultCode(packet.payload[5]);
  report.back_left_applied_command_milli = getI16(&packet.payload[6]);
  report.back_right_applied_command_milli = getI16(&packet.payload[8]);
  const std::uint8_t flags = packet.payload[10];
  report.fault_active = (flags & kEsp1StatusFaultActiveFlag) != 0U;
  report.back_left_inverted =
      (flags & kEsp1StatusBackLeftInvertedFlag) != 0U;
  report.back_right_inverted =
      (flags & kEsp1StatusBackRightInvertedFlag) != 0U;
  return true;
}

}  // namespace robot
