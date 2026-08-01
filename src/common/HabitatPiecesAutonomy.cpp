#include "common/HabitatPiecesAutonomy.h"

#include <cmath>

namespace robot {

const char* habitatPiecesStateName(const HabitatPiecesState state) {
  switch (state) {
    case HabitatPiecesState::WaitForStart:
      return "WAIT_FOR_START";
    case HabitatPiecesState::LineFollowing:
      return "LINE_FOLLOWING";
    case HabitatPiecesState::Reversing:
      return "REVERSING";
    case HabitatPiecesState::Complete:
      return "COMPLETE";
    case HabitatPiecesState::Fault:
      return "FAULT";
  }
  return "FAULT";
}

const char* habitatPiecesStopReasonName(
    const HabitatPiecesStopReason reason) {
  switch (reason) {
    case HabitatPiecesStopReason::None:
      return "NONE";
    case HabitatPiecesStopReason::ConfigurationIncomplete:
      return "CONFIGURATION_INCOMPLETE";
    case HabitatPiecesStopReason::Lss2Unavailable:
      return "LSS2_UNAVAILABLE";
    case HabitatPiecesStopReason::Lss2DataStale:
      return "LSS2_DATA_STALE";
    case HabitatPiecesStopReason::Lss2BlackDetected:
      return "LSS2_BLACK_DETECTED";
    case HabitatPiecesStopReason::RunTimeout:
      return "RUN_TIMEOUT";
    case HabitatPiecesStopReason::FrontLineLost:
      return "FRONT_LINE_LOST";
    case HabitatPiecesStopReason::RearCommandFailed:
      return "REAR_COMMAND_FAILED";
  }
  return "CONFIGURATION_INCOMPLETE";
}

bool habitatPiecesConfigValid(const HabitatPiecesConfig& config,
                              const float maximum_duty) {
  return std::isfinite(config.line_follow_duty) &&
         std::isfinite(maximum_duty) && config.line_follow_duty > 0.0F &&
         maximum_duty > 0.0F && config.line_follow_duty <= maximum_duty &&
         config.lss2_detection_delay_ms > 0U &&
         config.run_timeout_ms > config.lss2_detection_delay_ms &&
         std::isfinite(config.reverse_duty) && config.reverse_duty > 0.0F &&
         config.reverse_duty <= maximum_duty &&
         config.reverse_duration_ms > 0U;
}

void resetHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                                const Milliseconds now_ms) {
  autonomy = {};
  autonomy.state_entered_at_ms = now_ms;
  autonomy.stop_reason = HabitatPiecesStopReason::ConfigurationIncomplete;
}

void startHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                                const Milliseconds now_ms) {
  resetHabitatPiecesAutonomy(autonomy, now_ms);
  autonomy.state = HabitatPiecesState::LineFollowing;
  autonomy.stop_reason = HabitatPiecesStopReason::None;
  autonomy.run_started_at_ms = now_ms;
}

void failHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                               const HabitatPiecesStopReason reason,
                               const Milliseconds now_ms) {
  autonomy.state = HabitatPiecesState::Fault;
  autonomy.stop_reason = reason;
  autonomy.state_entered_at_ms = now_ms;
}

HabitatPiecesUpdate updateHabitatPiecesAutonomy(
    HabitatPiecesAutonomy& autonomy, const HabitatPiecesConfig& config,
    const bool lss2_black, const Milliseconds now_ms) {
  HabitatPiecesUpdate update{};
  update.state = autonomy.state;
  update.stop_reason = autonomy.stop_reason;
  update.lss2_detection_armed = autonomy.lss2_detection_armed;
  update.lss2_black = lss2_black;
  update.target_reached =
      autonomy.state == HabitatPiecesState::Reversing ||
      autonomy.state == HabitatPiecesState::Complete;

  if (autonomy.state == HabitatPiecesState::Reversing) {
    autonomy.reverse_elapsed_ms = now_ms - autonomy.state_entered_at_ms;
    if (autonomy.reverse_elapsed_ms >= config.reverse_duration_ms) {
      autonomy.state = HabitatPiecesState::Complete;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_reverse = true;
    return update;
  }

  if (autonomy.state != HabitatPiecesState::LineFollowing) {
    return update;
  }

  autonomy.run_elapsed_ms = now_ms - autonomy.run_started_at_ms;
  autonomy.lss2_detection_armed =
      autonomy.run_elapsed_ms >= config.lss2_detection_delay_ms;
  update.lss2_detection_armed = autonomy.lss2_detection_armed;

  if (autonomy.lss2_detection_armed && lss2_black) {
    autonomy.state = HabitatPiecesState::Reversing;
    autonomy.stop_reason = HabitatPiecesStopReason::Lss2BlackDetected;
    autonomy.state_entered_at_ms = now_ms;
    autonomy.reverse_elapsed_ms = 0U;
    update.state = autonomy.state;
    update.stop_reason = autonomy.stop_reason;
    update.target_reached = true;
    update.should_stop = false;
    update.should_reverse = true;
    update.transitioned = true;
    return update;
  }

  if (config.run_timeout_ms == 0U ||
      autonomy.run_elapsed_ms >= config.run_timeout_ms) {
    autonomy.state = HabitatPiecesState::Fault;
    autonomy.stop_reason = HabitatPiecesStopReason::RunTimeout;
    autonomy.state_entered_at_ms = now_ms;
    autonomy.timed_out = true;
    update.state = autonomy.state;
    update.stop_reason = autonomy.stop_reason;
    update.transitioned = true;
    return update;
  }

  update.should_stop = false;
  update.should_line_follow = true;
  return update;
}

}  // namespace robot
