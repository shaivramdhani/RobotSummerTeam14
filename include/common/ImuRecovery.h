#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class ImuRecoveryDecision : std::uint8_t {
  Continue = 0,
  Paused = 1,
  Recovered = 2,
  TimedOut = 3,
};

struct ImuRecoveryConfig {
  Milliseconds maximum_pause_ms{0U};
  std::uint8_t consecutive_fresh_samples_required{0U};
};

struct ImuRecoveryState {
  bool active{false};
  float saved_heading_deg{0.0F};
  Milliseconds pause_started_at_ms{0U};
  Milliseconds last_serviced_at_ms{0U};
  Milliseconds last_pause_duration_ms{0U};
  Milliseconds total_paused_ms{0U};
  std::uint32_t last_successful_read_count{0U};
  std::uint32_t pause_count{0U};
  std::uint8_t consecutive_fresh_samples{0U};
};

struct ImuRecoveryUpdate {
  ImuRecoveryDecision decision{ImuRecoveryDecision::Continue};
  Milliseconds pause_elapsed_ms{0U};
  Milliseconds timer_adjustment_ms{0U};
  bool pause_started{false};
};

bool imuRecoveryConfigValid(const ImuRecoveryConfig& config);
void resetImuRecovery(ImuRecoveryState& state);
void cancelImuRecovery(ImuRecoveryState& state);
ImuRecoveryUpdate updateImuRecovery(
    ImuRecoveryState& state, const ImuRecoveryConfig& config,
    bool imu_ready, std::uint32_t successful_read_count,
    float current_heading_deg, Milliseconds now_ms);

const char* imuRecoveryDecisionName(ImuRecoveryDecision decision);

}  // namespace robot
