#pragma once

#include <cstdint>

#include "common/MotorOutput.h"
#include "common/UartProtocol.h"
#include "common/Units.h"

namespace robot {

constexpr std::uint16_t kFunnelCommandPayloadSize = 12U;
constexpr std::uint8_t kMechanismPayloadTargetFunnel = 1U;
constexpr std::uint8_t kFunnelCommandEnabledFlag = 0x01U;
constexpr std::uint32_t kMaxInvalidFunnelPacketsBeforeStop = 3U;

struct FunnelCommand {
  bool enabled{false};
  std::int16_t command_milli{0};
  Milliseconds sender_timestamp_ms{0};
  Milliseconds timeout_ms{kDefaultCommunicationTimeoutMs};
};

struct FunnelStatus {
  bool link_healthy{false};
  bool has_valid_command{false};
  bool command_enabled{false};
  Milliseconds command_age_ms{0};
  std::uint16_t last_sequence{0};
  std::uint32_t invalid_packets_since_valid{0};
};

UartPacket makeFunnelCommandPacket(const FunnelCommand& command,
                                   std::uint16_t sequence);
bool decodeFunnelCommandPacket(const UartPacket& packet,
                               FunnelCommand& command);
bool enabledFunnelCommandIsStale(const FunnelStatus& status,
                                 Milliseconds fault_timeout_ms);

class FunnelCommandReceiver {
 public:
  void reset();
  bool acceptPacket(const UartPacket& packet, Milliseconds received_at_ms);
  MotorCommand motorCommand(Milliseconds now_ms) const;
  FunnelStatus status(Milliseconds now_ms) const;

 private:
  bool commandIsFresh(Milliseconds now_ms) const;

  FunnelCommand last_command_{};
  Milliseconds last_valid_received_at_ms_{0};
  std::uint16_t last_sequence_{0};
  bool has_valid_command_{false};
  std::uint32_t invalid_packets_since_valid_{0};
};

}  // namespace robot
