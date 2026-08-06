#pragma once

#include <cstdint>

namespace robot {

enum class FinalCompetitionStartSwitchAction : std::uint8_t {
  None = 0,
  Start = 1,
  EmergencyStop = 2,
};

struct FinalCompetitionStartSwitchState {
  bool initialized{false};
  bool start_armed{false};
  bool previous_high{false};
};

inline FinalCompetitionStartSwitchAction updateFinalCompetitionStartSwitch(
    FinalCompetitionStartSwitchState& state, const bool signal_high,
    const bool final_competition_mode_active) {
  if (!state.initialized) {
    state.initialized = true;
    state.start_armed = !signal_high;
    state.previous_high = signal_high;
    return final_competition_mode_active && !signal_high
               ? FinalCompetitionStartSwitchAction::EmergencyStop
               : FinalCompetitionStartSwitchAction::None;
  }

  if (!signal_high) {
    state.start_armed = true;
    state.previous_high = false;
    return final_competition_mode_active
               ? FinalCompetitionStartSwitchAction::EmergencyStop
               : FinalCompetitionStartSwitchAction::None;
  }

  const bool rising_edge = !state.previous_high;
  state.previous_high = true;
  if (rising_edge && state.start_armed) {
    state.start_armed = false;
    return FinalCompetitionStartSwitchAction::Start;
  }
  return FinalCompetitionStartSwitchAction::None;
}

}  // namespace robot
