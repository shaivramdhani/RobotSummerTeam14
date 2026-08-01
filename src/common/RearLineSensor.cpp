#include "common/RearLineSensor.h"

namespace robot {

namespace {

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

}  // namespace

UartPacket makeRearLineSensorPacket(
    const RearLineSensorSnapshot& snapshot, const std::uint16_t sequence) {
  UartPacket packet{};
  packet.header.version = kUartProtocolVersion;
  packet.header.message_type = UartMessageType::SensorSnapshot;
  packet.header.sequence = sequence;
  packet.header.payload_size = kRearLineSensorPayloadSize;
  putU32(&packet.payload[0], snapshot.captured_at_ms);
  std::uint8_t flags = 0U;
  flags |= snapshot.configured ? kRearLineSensorConfiguredFlag : 0U;
  flags |= snapshot.left_electrical_high ? kRearLineSensorLeftHighFlag : 0U;
  flags |= snapshot.right_electrical_high ? kRearLineSensorRightHighFlag : 0U;
  flags |= snapshot.side_configured ? kSideLineSensorConfiguredFlag : 0U;
  flags |= snapshot.side_electrical_high ? kSideLineSensorHighFlag : 0U;
  flags |= snapshot.side_2_configured ? kSideLineSensor2ConfiguredFlag : 0U;
  flags |= snapshot.side_2_electrical_high ? kSideLineSensor2HighFlag : 0U;
  packet.payload[4] = flags;
  std::uint8_t side_3_flags = 0U;
  side_3_flags |= snapshot.side_3_configured
                      ? kSideLineSensor3ConfiguredFlag
                      : 0U;
  side_3_flags |= snapshot.side_3_electrical_high
                      ? kSideLineSensor3HighFlag
                      : 0U;
  packet.payload[5] = side_3_flags;
  packet.header.integrity_crc16 = calculatePacketIntegrity(packet);
  return packet;
}

bool decodeRearLineSensorPacket(const UartPacket& packet,
                                RearLineSensorSnapshot& snapshot) {
  if (!packetLooksValid(packet) ||
      packet.header.message_type != UartMessageType::SensorSnapshot ||
      (packet.header.payload_size != kRearLineSensorPayloadSize &&
       packet.header.payload_size != kLegacyRearLineSensorPayloadSize)) {
    snapshot = {};
    return false;
  }

  snapshot.captured_at_ms = getU32(&packet.payload[0]);
  const std::uint8_t flags = packet.payload[4];
  snapshot.configured = (flags & kRearLineSensorConfiguredFlag) != 0U;
  snapshot.left_electrical_high =
      (flags & kRearLineSensorLeftHighFlag) != 0U;
  snapshot.right_electrical_high =
      (flags & kRearLineSensorRightHighFlag) != 0U;
  snapshot.side_configured =
      (flags & kSideLineSensorConfiguredFlag) != 0U;
  snapshot.side_electrical_high =
      (flags & kSideLineSensorHighFlag) != 0U;
  snapshot.side_2_configured =
      (flags & kSideLineSensor2ConfiguredFlag) != 0U;
  snapshot.side_2_electrical_high =
      (flags & kSideLineSensor2HighFlag) != 0U;
  const std::uint8_t side_3_flags =
      packet.header.payload_size == kRearLineSensorPayloadSize
          ? packet.payload[5]
          : 0U;
  snapshot.side_3_configured =
      (side_3_flags & kSideLineSensor3ConfiguredFlag) != 0U;
  snapshot.side_3_electrical_high =
      (side_3_flags & kSideLineSensor3HighFlag) != 0U;
  return true;
}

}  // namespace robot
