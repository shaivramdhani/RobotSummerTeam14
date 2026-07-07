#pragma once

#include <cstdint>

#include "common/FaultHealth.h"
#include "common/RobotTestMode.h"
#include "common/UartProtocol.h"
#include "common/Units.h"

namespace robot {

constexpr std::uint16_t kEsp1StatusPayloadSize = 11U;
constexpr std::uint8_t kEsp1StatusFaultActiveFlag = 0x01U;
constexpr std::uint8_t kEsp1StatusBackLeftInvertedFlag = 0x02U;
constexpr std::uint8_t kEsp1StatusBackRightInvertedFlag = 0x04U;

struct Esp1StatusReport {
  Milliseconds uptime_ms{0};
  RobotTestMode mode{RobotTestMode::Disabled};
  bool fault_active{false};
  FaultCode fault_code{FaultCode::None};
  std::int16_t back_left_applied_command_milli{0};
  std::int16_t back_right_applied_command_milli{0};
  bool back_left_inverted{false};
  bool back_right_inverted{false};
};

UartPacket makeEsp1StatusPacket(const Esp1StatusReport& report,
                                std::uint16_t sequence);
bool decodeEsp1StatusPacket(const UartPacket& packet,
                            Esp1StatusReport& report);

}  // namespace robot
