#pragma once

#include <cstdint>

#include "common/UartProtocol.h"
#include "common/Units.h"

namespace robot {

constexpr std::uint16_t kRearLineSensorPayloadSize = 5U;
constexpr std::uint8_t kRearLineSensorConfiguredFlag = 0x01U;
constexpr std::uint8_t kRearLineSensorLeftHighFlag = 0x02U;
constexpr std::uint8_t kRearLineSensorRightHighFlag = 0x04U;
constexpr std::uint8_t kSideLineSensorConfiguredFlag = 0x08U;
constexpr std::uint8_t kSideLineSensorHighFlag = 0x10U;

struct RearLineSensorSnapshot {
  Milliseconds captured_at_ms{0};
  bool configured{false};
  bool left_electrical_high{false};
  bool right_electrical_high{false};
  bool side_configured{false};
  bool side_electrical_high{false};
};

UartPacket makeRearLineSensorPacket(const RearLineSensorSnapshot& snapshot,
                                    std::uint16_t sequence);
bool decodeRearLineSensorPacket(const UartPacket& packet,
                                RearLineSensorSnapshot& snapshot);

}  // namespace robot
