#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class TimeTrialState : std::uint8_t {
  WaitForStart = 0,
  AutonomousSolar = 1,
  PostSolarDelay = 2,
  SolarToTowerStrafeRight = 3,
  TowerPieces = 4,
  PostTowerDelay = 5,
  PegFinder = 6,
  Complete = 7,
  Fault = 8,
};

struct TimeTrialConfig {
  // TODO(team): tune these transitions on the real robot.
  Milliseconds post_solar_delay_ms{0U};
  Milliseconds solar_to_tower_strafe_right_duration_ms{0U};
  Milliseconds post_tower_delay_ms{0U};
};

struct TimeTrialInputs {
  bool solar_complete{false};
  bool solar_fault{false};
  bool tower_pieces_complete{false};
  bool tower_pieces_fault{false};
  bool peg_finder_complete{false};
  bool peg_finder_fault{false};
};

struct TimeTrialAutonomy {
  TimeTrialState state{TimeTrialState::WaitForStart};
  Milliseconds state_entered_at_ms{0U};
};

struct TimeTrialUpdate {
  TimeTrialState state{TimeTrialState::WaitForStart};
  bool should_start_solar{false};
  bool should_strafe_right{false};
  bool should_start_tower_pieces{false};
  bool should_start_peg_finder{false};
};

const char* timeTrialStateName(TimeTrialState state);
bool timeTrialConfigValid(const TimeTrialConfig& config,
                          float maximum_allowed_duty);
void resetTimeTrialAutonomy(TimeTrialAutonomy& autonomy,
                            Milliseconds now_ms);
TimeTrialUpdate startTimeTrialAutonomy(TimeTrialAutonomy& autonomy,
                                       Milliseconds now_ms);
void failTimeTrialAutonomy(TimeTrialAutonomy& autonomy,
                           Milliseconds now_ms);
TimeTrialUpdate updateTimeTrialAutonomy(
    TimeTrialAutonomy& autonomy, const TimeTrialInputs& inputs,
    const TimeTrialConfig& config, Milliseconds now_ms);

}  // namespace robot
