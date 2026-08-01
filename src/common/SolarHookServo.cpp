#include "common/SolarHookServo.h"

namespace robot {

UartPacket makeSolarHookCommandPacket(const SolarHookCommand& command,
                                      const std::uint16_t sequence) {
  UartPacket packet{};
  packet.header.version = kUartProtocolVersion;
  packet.header.message_type = UartMessageType::MechanismCommand;
  packet.header.sequence = sequence;
  packet.header.payload_size = kSolarHookCommandPayloadSize;
  packet.payload[0] = kMechanismPayloadTargetSolarHook;
  packet.payload[1] = command.enabled ? kSolarHookEnabledFlag : 0U;
  packet.payload[2] = command.angle_deg;
  packet.header.integrity_crc16 = calculatePacketIntegrity(packet);
  return packet;
}

bool decodeSolarHookCommandPacket(const UartPacket& packet,
                                  SolarHookCommand& command) {
  if (!packetLooksValid(packet) ||
      packet.header.message_type != UartMessageType::MechanismCommand ||
      packet.header.payload_size != kSolarHookCommandPayloadSize ||
      packet.payload[0] != kMechanismPayloadTargetSolarHook ||
      (packet.payload[1] & ~kSolarHookEnabledFlag) != 0U ||
      packet.payload[2] > kMaximumServoAngleDeg) {
    command = {};
    return false;
  }

  command.enabled = (packet.payload[1] & kSolarHookEnabledFlag) != 0U;
  command.angle_deg = packet.payload[2];
  return true;
}

}  // namespace robot
