#pragma once

#include <cstdint>

#include "common/UartProtocol.h"

namespace robot {

constexpr std::uint8_t kMechanismPayloadTargetSolarHook = 2U;
constexpr std::uint8_t kSolarHookEnabledFlag = 0x01U;
constexpr std::uint16_t kSolarHookCommandPayloadSize = 3U;
constexpr std::uint8_t kMaximumServoAngleDeg = 180U;

struct SolarHookCommand {
  bool enabled{false};
  std::uint8_t angle_deg{0U};
};

UartPacket makeSolarHookCommandPacket(const SolarHookCommand& command,
                                      std::uint16_t sequence);
bool decodeSolarHookCommandPacket(const UartPacket& packet,
                                  SolarHookCommand& command);

}  // namespace robot
