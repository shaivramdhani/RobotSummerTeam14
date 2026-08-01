#include "common/ImuRecovery.h"

#include <cmath>
#include <limits>

namespace robot {

namespace {

Milliseconds elapsed(const Milliseconds now_ms,
                     const Milliseconds then_ms) {
  const Milliseconds difference =
      static_cast<Milliseconds>(now_ms - then_ms);
  const Milliseconds maximum_unambiguous_difference =
      std::numeric_limits<Milliseconds>::max() / 2U;
  return difference <= maximum_unambiguous_difference ? difference : 0U;
}

Milliseconds saturatingAdd(const Milliseconds left,
                           const Milliseconds right) {
  const Milliseconds maximum =
      std::numeric_limits<Milliseconds>::max();
  return right > maximum - left ? maximum : left + right;
}

}  // namespace

bool imuRecoveryConfigValid(const ImuRecoveryConfig& config) {
  return config.maximum_pause_ms > 0U &&
         config.consecutive_fresh_samples_required > 0U;
}

void resetImuRecovery(ImuRecoveryState& state) {
  state = {};
}

void cancelImuRecovery(ImuRecoveryState& state) {
  state.active = false;
  state.pause_started_at_ms = 0U;
  state.last_serviced_at_ms = 0U;
  state.last_successful_read_count = 0U;
  state.consecutive_fresh_samples = 0U;
}

ImuRecoveryUpdate updateImuRecovery(
    ImuRecoveryState& state, const ImuRecoveryConfig& config,
    const bool imu_ready,
    const std::uint32_t successful_read_count,
    const float current_heading_deg, const Milliseconds now_ms) {
  ImuRecoveryUpdate update{};
  if (!imuRecoveryConfigValid(config)) {
    update.decision = ImuRecoveryDecision::TimedOut;
    return update;
  }

  if (!state.active) {
    if (imu_ready) {
      return update;
    }
    state.active = true;
    state.saved_heading_deg =
        std::isfinite(current_heading_deg) ? current_heading_deg : 0.0F;
    state.pause_started_at_ms = now_ms;
    state.last_serviced_at_ms = now_ms;
    state.last_successful_read_count = successful_read_count;
    state.consecutive_fresh_samples = 0U;
    ++state.pause_count;
    update.decision = ImuRecoveryDecision::Paused;
    update.pause_started = true;
    return update;
  }

  update.timer_adjustment_ms =
      elapsed(now_ms, state.last_serviced_at_ms);
  state.last_serviced_at_ms = now_ms;
  state.total_paused_ms =
      saturatingAdd(state.total_paused_ms,
                    update.timer_adjustment_ms);
  update.pause_elapsed_ms =
      elapsed(now_ms, state.pause_started_at_ms);
  state.last_pause_duration_ms = update.pause_elapsed_ms;

  if (update.pause_elapsed_ms >= config.maximum_pause_ms) {
    state.active = false;
    state.consecutive_fresh_samples = 0U;
    update.decision = ImuRecoveryDecision::TimedOut;
    return update;
  }

  const bool new_successful_sample =
      successful_read_count != state.last_successful_read_count;
  state.last_successful_read_count = successful_read_count;
  if (!imu_ready) {
    state.consecutive_fresh_samples = 0U;
    update.decision = ImuRecoveryDecision::Paused;
    return update;
  }
  if (new_successful_sample &&
      state.consecutive_fresh_samples <
          std::numeric_limits<std::uint8_t>::max()) {
    ++state.consecutive_fresh_samples;
  }
  if (state.consecutive_fresh_samples <
      config.consecutive_fresh_samples_required) {
    update.decision = ImuRecoveryDecision::Paused;
    return update;
  }

  state.active = false;
  update.decision = ImuRecoveryDecision::Recovered;
  return update;
}

const char* imuRecoveryDecisionName(
    const ImuRecoveryDecision decision) {
  switch (decision) {
    case ImuRecoveryDecision::Continue:
      return "CONTINUE";
    case ImuRecoveryDecision::Paused:
      return "PAUSED";
    case ImuRecoveryDecision::Recovered:
      return "RECOVERED";
    case ImuRecoveryDecision::TimedOut:
      return "TIMED_OUT";
  }
  return "TIMED_OUT";
}

}  // namespace robot
