#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

constexpr std::uint8_t kTransitionTransmissionCount = 2U;

struct CommandTransmissionState {
  Milliseconds last_transmitted_at_ms{0U};
  std::uint8_t transition_repeats_remaining{0U};
};

constexpr Milliseconds commandRefreshPeriodMs(
    const Milliseconds timeout_ms) {
  return timeout_ms > 1U ? timeout_ms / 2U : timeout_ms;
}

inline bool commandTransmissionDue(
    const CommandTransmissionState& state, const bool semantic_change,
    const bool continuous_heartbeat, const Milliseconds now_ms,
    const Milliseconds refresh_period_ms) {
  return semantic_change || continuous_heartbeat ||
         state.transition_repeats_remaining > 0U ||
         (refresh_period_ms > 0U &&
          static_cast<Milliseconds>(now_ms - state.last_transmitted_at_ms) >=
              refresh_period_ms);
}

inline void noteCommandTransmitted(CommandTransmissionState& state,
                                   const bool semantic_change,
                                   const Milliseconds now_ms) {
  state.last_transmitted_at_ms = now_ms;
  if (semantic_change) {
    state.transition_repeats_remaining =
        kTransitionTransmissionCount > 0U
            ? kTransitionTransmissionCount - 1U
            : 0U;
  } else if (state.transition_repeats_remaining > 0U) {
    --state.transition_repeats_remaining;
  }
}

}  // namespace robot
