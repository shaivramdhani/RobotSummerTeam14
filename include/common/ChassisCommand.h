#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class ChassisMode : std::uint8_t {
  Disabled = 0,
  HoldPosition = 1,
  OpenLoopTwist = 2,
  LineFollow = 3,
};

struct ChassisCommand {
  ChassisMode mode{ChassisMode::Disabled};
  std::int16_t forward_command_milli{0};
  std::int16_t lateral_command_milli{0};
  std::int16_t yaw_command_milli{0};
  Milliseconds issued_at_ms{0};
  Milliseconds timeout_ms{kDefaultCommunicationTimeoutMs};
};

constexpr ChassisCommand disabledChassisCommand() {
  return {};
}

}  // namespace robot
