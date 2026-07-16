#include "common/SolarPanelAutonomy.h"

#include <cmath>

namespace robot {

namespace {

float clampFloat(const float value, const float minimum, const float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

Milliseconds elapsedSince(const Milliseconds now_ms,
                          const Milliseconds then_ms) {
  return now_ms >= then_ms ? now_ms - then_ms : 0U;
}

float normalizedFilterAlpha(const float alpha) {
  return std::isfinite(alpha) ? clampFloat(alpha, 0.0F, 0.99F) : 0.0F;
}

void clearConfirmation(SolarBeaconDetectorState& state) {
  state.confirmation_active = false;
  state.confirmation_started_at_ms = 0U;
  state.confirmation_progress_ms = 0U;
}

}  // namespace

const char* solarPanelAutonomyStateName(
    const SolarPanelAutonomyState state) {
  switch (state) {
    case SolarPanelAutonomyState::WaitForStart:
      return "WAIT_FOR_START";
    case SolarPanelAutonomyState::LineFollowToSolar:
      return "LINE_FOLLOW_TO_SOLAR";
    case SolarPanelAutonomyState::SolarBeaconAligned:
      return "SOLAR_BEACON_ALIGNED";
    case SolarPanelAutonomyState::SolarSearchFault:
      return "SOLAR_SEARCH_FAULT";
  }
  return "WAIT_FOR_START";
}

const char* solarPanelFaultReasonName(
    const SolarPanelFaultReason reason) {
  switch (reason) {
    case SolarPanelFaultReason::None:
      return "NONE";
    case SolarPanelFaultReason::SearchTimeout:
      return "SEARCH_TIMEOUT";
    case SolarPanelFaultReason::HardwareNotReady:
      return "HARDWARE_NOT_READY";
    case SolarPanelFaultReason::LineLost:
      return "LINE_LOST";
    case SolarPanelFaultReason::RearLinkStale:
      return "REAR_LINK_STALE";
  }
  return "NONE";
}

bool solarPanelAutonomyConfigValid(
    const SolarPanelAutonomyConfig& config) {
  return config.release_threshold <= config.detection_threshold &&
         std::isfinite(config.filter_alpha) && config.filter_alpha >= 0.0F &&
         config.filter_alpha < 1.0F && config.search_timeout_ms > 0U;
}

void resetSolarBeaconDetectorState(SolarBeaconDetectorState& state) {
  state = {};
}

SolarBeaconDetectorUpdate updateSolarBeaconDetector(
    SolarBeaconDetectorState& state, const std::uint16_t raw_amplitude,
    const SolarPanelAutonomyConfig& config, const Milliseconds now_ms,
    const bool detection_permitted) {
  const float alpha = normalizedFilterAlpha(config.filter_alpha);
  if (!state.filter_initialized) {
    state.filtered_amplitude = static_cast<float>(raw_amplitude);
    state.filter_initialized = true;
  } else {
    state.filtered_amplitude =
        (alpha * state.filtered_amplitude) +
        ((1.0F - alpha) * static_cast<float>(raw_amplitude));
  }

  if (!detection_permitted) {
    clearConfirmation(state);
    state.beacon_detected = false;
  } else {
    if (state.filtered_amplitude >=
        static_cast<float>(config.detection_threshold)) {
      if (!state.confirmation_active) {
        state.confirmation_active = true;
        state.confirmation_started_at_ms = now_ms;
        state.confirmation_progress_ms = 0U;
      }
    } else if (state.filtered_amplitude <=
               static_cast<float>(config.release_threshold)) {
      clearConfirmation(state);
      state.beacon_detected = false;
    }

    if (state.confirmation_active) {
      state.confirmation_progress_ms =
          elapsedSince(now_ms, state.confirmation_started_at_ms);
      if (state.confirmation_progress_ms >= config.confirmation_time_ms) {
        state.confirmation_progress_ms = config.confirmation_time_ms;
        state.beacon_detected = true;
      }
    }
  }

  return {raw_amplitude, state.filtered_amplitude,
          state.confirmation_active, state.confirmation_progress_ms,
          state.beacon_detected};
}

}  // namespace robot
