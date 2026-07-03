#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class FaultCode : std::uint8_t {
  None = 0,
  CommunicationStale = 1,
  InvalidCommand = 2,
  LimitSwitchConflict = 3,
  HardwareNotConfigured = 4,
};

struct HealthReport {
  FaultCode active_fault{FaultCode::None};
  Milliseconds last_valid_communication_ms{0};
  bool local_actuators_enabled{false};
};

constexpr bool hasFault(const HealthReport& report) {
  return report.active_fault != FaultCode::None;
}

}  // namespace robot
