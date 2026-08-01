#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

constexpr float kDefaultHabitatPiecesLineFollowDuty = 0.12F;

enum class HabitatPiecesState : std::uint8_t {
  WaitForStart = 0,
  LineFollowing = 1,
  Reversing = 2,
  Complete = 3,
  Fault = 4,
};

enum class HabitatPiecesStopReason : std::uint8_t {
  None = 0,
  ConfigurationIncomplete = 1,
  Lss2Unavailable = 2,
  Lss2DataStale = 3,
  Lss2BlackDetected = 4,
  RunTimeout = 5,
  FrontLineLost = 6,
  RearCommandFailed = 7,
};

struct HabitatPiecesConfig {
  float line_follow_duty{kDefaultHabitatPiecesLineFollowDuty};
  Milliseconds lss2_detection_delay_ms{0U};
  Milliseconds run_timeout_ms{0U};
  float reverse_duty{0.0F};
  Milliseconds reverse_duration_ms{0U};
};

struct HabitatPiecesAutonomy {
  HabitatPiecesState state{HabitatPiecesState::WaitForStart};
  HabitatPiecesStopReason stop_reason{
      HabitatPiecesStopReason::ConfigurationIncomplete};
  Milliseconds state_entered_at_ms{0U};
  Milliseconds run_started_at_ms{0U};
  Milliseconds run_elapsed_ms{0U};
  Milliseconds reverse_elapsed_ms{0U};
  bool lss2_detection_armed{false};
  bool timed_out{false};
};

struct HabitatPiecesUpdate {
  HabitatPiecesState state{HabitatPiecesState::WaitForStart};
  HabitatPiecesStopReason stop_reason{
      HabitatPiecesStopReason::ConfigurationIncomplete};
  bool should_stop{true};
  bool should_line_follow{false};
  bool should_reverse{false};
  bool lss2_detection_armed{false};
  bool lss2_black{false};
  bool target_reached{false};
  bool transitioned{false};
};

const char* habitatPiecesStateName(HabitatPiecesState state);
const char* habitatPiecesStopReasonName(HabitatPiecesStopReason reason);
bool habitatPiecesConfigValid(const HabitatPiecesConfig& config,
                              float maximum_duty);
void resetHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                                Milliseconds now_ms);
void startHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                                Milliseconds now_ms);
void failHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                               HabitatPiecesStopReason reason,
                               Milliseconds now_ms);
HabitatPiecesUpdate updateHabitatPiecesAutonomy(
    HabitatPiecesAutonomy& autonomy, const HabitatPiecesConfig& config,
    bool lss2_black, Milliseconds now_ms);

}  // namespace robot
