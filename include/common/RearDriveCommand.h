#pragma once

#include <cstdint>

#include "common/MotorOutput.h"
#include "common/UartProtocol.h"
#include "common/Units.h"

namespace robot {

constexpr std::uint16_t kRearDrivePayloadSize = 13U;
constexpr std::uint8_t kRearDriveEnabledFlag = 0x01U;
constexpr std::uint32_t kMaxInvalidRearPacketsBeforeStop = 3U;

struct RearDriveCommand {
  bool enabled{false};
  std::int16_t back_left_command_milli{0};
  std::int16_t back_right_command_milli{0};
  Milliseconds sender_timestamp_ms{0};
  Milliseconds timeout_ms{kDefaultCommunicationTimeoutMs};
};

struct RearDriveStatus {
  bool link_healthy{false};
  bool has_valid_command{false};
  Milliseconds command_age_ms{0};
  std::uint16_t last_sequence{0};
  std::uint32_t invalid_packets_since_valid{0};
};

UartPacket makeRearDriveCommandPacket(const RearDriveCommand& command,
                                      std::uint16_t sequence);
bool decodeRearDriveCommandPacket(const UartPacket& packet,
                                  RearDriveCommand& command);

class RearDriveCommandReceiver {
 public:
  void reset();
  bool acceptPacket(const UartPacket& packet, Milliseconds received_at_ms);
  MotorCommand backLeftCommand(Milliseconds now_ms) const;
  MotorCommand backRightCommand(Milliseconds now_ms) const;
  RearDriveStatus status(Milliseconds now_ms) const;

 private:
  bool commandIsFresh(Milliseconds now_ms) const;
  MotorCommand motorCommand(std::int16_t duty_command_milli,
                            Milliseconds now_ms) const;

  RearDriveCommand last_command_{};
  Milliseconds last_valid_received_at_ms_{0};
  std::uint16_t last_sequence_{0};
  bool has_valid_command_{false};
  std::uint32_t invalid_packets_since_valid_{0};
};

}  // namespace robot
