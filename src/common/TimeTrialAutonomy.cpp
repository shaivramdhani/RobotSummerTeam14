#include "common/TimeTrialAutonomy.h"

#include <cmath>

namespace robot {

namespace {

TimeTrialUpdate makeUpdate(const TimeTrialState state) {
  TimeTrialUpdate update{};
  update.state = state;
  update.should_strafe_right =
      state == TimeTrialState::SolarToTowerStrafeRight;
  return update;
}

void enterState(TimeTrialAutonomy& autonomy, const TimeTrialState state,
                const Milliseconds now_ms) {
  autonomy.state = state;
  autonomy.state_entered_at_ms = now_ms;
}

}  // namespace

const char* timeTrialStateName(const TimeTrialState state) {
  switch (state) {
    case TimeTrialState::WaitForStart:
      return "WAIT_FOR_START";
    case TimeTrialState::AutonomousSolar:
      return "AUTONOMOUS_SOLAR";
    case TimeTrialState::PostSolarDelay:
      return "POST_SOLAR_DELAY";
    case TimeTrialState::SolarToTowerStrafeRight:
      return "SOLAR_TO_TOWER_STRAFE_RIGHT";
    case TimeTrialState::TowerPieces:
      return "TOWER_PIECES";
    case TimeTrialState::PostTowerDelay:
      return "POST_TOWER_DELAY";
    case TimeTrialState::PegFinder:
      return "PEG_FINDER";
    case TimeTrialState::Complete:
      return "COMPLETE";
    case TimeTrialState::Fault:
      return "FAULT";
  }
  return "WAIT_FOR_START";
}

bool timeTrialConfigValid(const TimeTrialConfig& config,
                          const float maximum_allowed_duty) {
  return std::isfinite(config.solar_to_tower_strafe_right_duty) &&
         std::isfinite(maximum_allowed_duty) &&
         config.solar_to_tower_strafe_right_duty >= 0.0F &&
         config.solar_to_tower_strafe_right_duty <= maximum_allowed_duty &&
         (config.solar_to_tower_strafe_right_duration_ms == 0U ||
          config.solar_to_tower_strafe_right_duty > 0.0F);
}

void resetTimeTrialAutonomy(TimeTrialAutonomy& autonomy,
                            const Milliseconds now_ms) {
  autonomy = {};
  autonomy.state_entered_at_ms = now_ms;
}

TimeTrialUpdate startTimeTrialAutonomy(TimeTrialAutonomy& autonomy,
                                       const Milliseconds now_ms) {
  enterState(autonomy, TimeTrialState::AutonomousSolar, now_ms);
  TimeTrialUpdate update = makeUpdate(autonomy.state);
  update.should_start_solar = true;
  return update;
}

void failTimeTrialAutonomy(TimeTrialAutonomy& autonomy,
                           const Milliseconds now_ms) {
  enterState(autonomy, TimeTrialState::Fault, now_ms);
}

TimeTrialUpdate updateTimeTrialAutonomy(
    TimeTrialAutonomy& autonomy, const TimeTrialInputs& inputs,
    const TimeTrialConfig& config, const Milliseconds now_ms) {
  const Milliseconds elapsed_ms = now_ms - autonomy.state_entered_at_ms;
  TimeTrialUpdate update = makeUpdate(autonomy.state);

  switch (autonomy.state) {
    case TimeTrialState::AutonomousSolar:
      if (inputs.solar_fault) {
        failTimeTrialAutonomy(autonomy, now_ms);
      } else if (inputs.solar_complete) {
        enterState(autonomy, TimeTrialState::PostSolarDelay, now_ms);
      }
      break;

    case TimeTrialState::PostSolarDelay:
      if (elapsed_ms >= config.post_solar_delay_ms) {
        if (config.solar_to_tower_strafe_right_duration_ms == 0U) {
          enterState(autonomy, TimeTrialState::TowerPieces, now_ms);
          update.should_start_tower_pieces = true;
        } else {
          enterState(autonomy, TimeTrialState::SolarToTowerStrafeRight,
                     now_ms);
        }
      }
      break;

    case TimeTrialState::SolarToTowerStrafeRight:
      if (elapsed_ms >=
          config.solar_to_tower_strafe_right_duration_ms) {
        enterState(autonomy, TimeTrialState::TowerPieces, now_ms);
        update.should_start_tower_pieces = true;
      }
      break;

    case TimeTrialState::TowerPieces:
      if (inputs.tower_pieces_fault) {
        failTimeTrialAutonomy(autonomy, now_ms);
      } else if (inputs.tower_pieces_complete) {
        enterState(autonomy, TimeTrialState::PostTowerDelay, now_ms);
      }
      break;

    case TimeTrialState::PostTowerDelay:
      if (elapsed_ms >= config.post_tower_delay_ms) {
        enterState(autonomy, TimeTrialState::PegFinder, now_ms);
        update.should_start_peg_finder = true;
      }
      break;

    case TimeTrialState::PegFinder:
      if (inputs.peg_finder_fault) {
        failTimeTrialAutonomy(autonomy, now_ms);
      } else if (inputs.peg_finder_complete) {
        enterState(autonomy, TimeTrialState::Complete, now_ms);
      }
      break;

    case TimeTrialState::WaitForStart:
    case TimeTrialState::Complete:
    case TimeTrialState::Fault:
      break;
  }

  update.state = autonomy.state;
  update.should_strafe_right =
      autonomy.state == TimeTrialState::SolarToTowerStrafeRight;
  return update;
}

}  // namespace robot
