#include "common/TowerPiecesAutonomy.h"

#include <cmath>

namespace robot {

const char* towerPiecesStateName(const TowerPiecesState state) {
  switch (state) {
    case TowerPiecesState::WaitForStart:
      return "WAIT_FOR_START";
    case TowerPiecesState::ReverseLineFollow:
      return "REVERSE_LINE_FOLLOW";
    case TowerPiecesState::PostLineDelay:
      return "POST_LINE_DELAY";
    case TowerPiecesState::StrafeRight:
      return "STRAFE_RIGHT";
    case TowerPiecesState::PostStrafePause:
      return "POST_STRAFE_PAUSE";
    case TowerPiecesState::RotateClockwise:
      return "ROTATE_CLOCKWISE";
    case TowerPiecesState::PostRotationPause:
      return "POST_ROTATION_PAUSE";
    case TowerPiecesState::ReverseTimed:
      return "REVERSE_TIMED";
    case TowerPiecesState::ShimmyLeft:
      return "SHIMMY_LEFT";
    case TowerPiecesState::ShimmyRight:
      return "SHIMMY_RIGHT";
    case TowerPiecesState::Complete:
      return "COMPLETE";
    case TowerPiecesState::Fault:
      return "FAULT";
  }
  return "WAIT_FOR_START";
}

const char* towerPiecesFaultReasonName(
    const TowerPiecesFaultReason reason) {
  switch (reason) {
    case TowerPiecesFaultReason::None:
      return "NONE";
    case TowerPiecesFaultReason::HardwareNotReady:
      return "HARDWARE_NOT_READY";
    case TowerPiecesFaultReason::RearLinkStale:
      return "REAR_LINK_STALE";
    case TowerPiecesFaultReason::LineLost:
      return "LINE_LOST";
    case TowerPiecesFaultReason::SideLineTimeout:
      return "SIDE_LINE_TIMEOUT";
    case TowerPiecesFaultReason::RearCommandFailed:
      return "REAR_COMMAND_FAILED";
    case TowerPiecesFaultReason::ShimmyTimeout:
      return "SHIMMY_TIMEOUT";
  }
  return "NONE";
}

bool towerPiecesConfigValid(const TowerPiecesConfig& config,
                            const float maximum_allowed_duty) {
  return std::isfinite(config.reverse_line_duty) &&
         std::isfinite(config.strafe_right_duty) &&
         std::isfinite(config.clockwise_rotation_duty) &&
         std::isfinite(config.reverse_duty) &&
         std::isfinite(config.shimmy_duty) &&
         std::isfinite(maximum_allowed_duty) &&
         config.reverse_line_duty > 0.0F &&
         config.reverse_line_duty <= maximum_allowed_duty &&
         config.strafe_right_duty > 0.0F &&
         config.strafe_right_duty <= maximum_allowed_duty &&
         config.clockwise_rotation_duty > 0.0F &&
         config.clockwise_rotation_duty <= maximum_allowed_duty &&
         config.reverse_duty > 0.0F &&
         config.reverse_duty <= maximum_allowed_duty &&
         config.shimmy_duty > 0.0F &&
         config.shimmy_duty <= maximum_allowed_duty &&
         config.side_line_timeout_ms > 0U &&
         config.post_line_delay_ms > 0U &&
         config.strafe_right_duration_ms > 0U &&
         config.post_strafe_pause_ms > 0U &&
         config.clockwise_rotation_duration_ms > 0U &&
         config.post_rotation_pause_ms > 0U &&
         config.reverse_duration_ms > 0U &&
         config.shimmy_right_duration_ms > 0U &&
         config.shimmy_left_duration_ms > 0U &&
         config.shimmy_timeout_ms > 0U;
}

void resetTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                              const Milliseconds now_ms) {
  autonomy = {};
  autonomy.state_entered_at_ms = now_ms;
}

void startTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                              const bool side_line_high,
                              const Milliseconds now_ms) {
  autonomy = {};
  autonomy.state = TowerPiecesState::ReverseLineFollow;
  autonomy.state_entered_at_ms = now_ms;
  autonomy.started_at_ms = now_ms;
  // A line already under the sensor at start is not a new crossing. The
  // sensor must return LOW before a later HIGH can increment the count.
  autonomy.previous_side_line_high = side_line_high;
}

TowerPiecesUpdate updateTowerPiecesAutonomy(
    TowerPiecesAutonomy& autonomy, const bool side_line_high,
    const bool back_left_line_high, const bool back_right_line_high,
    const TowerPiecesConfig& config, const Milliseconds now_ms) {
  TowerPiecesUpdate update{};

  switch (autonomy.state) {
    case TowerPiecesState::ReverseLineFollow:
      update.side_line_rising_edge =
          !autonomy.previous_side_line_high && side_line_high;
      autonomy.previous_side_line_high = side_line_high;
      if (update.side_line_rising_edge &&
          autonomy.side_line_count < kTowerPiecesTargetSideLineCount) {
        ++autonomy.side_line_count;
      }

      if (autonomy.side_line_count >= kTowerPiecesTargetSideLineCount) {
        autonomy.state = TowerPiecesState::PostLineDelay;
        autonomy.fault_reason = TowerPiecesFaultReason::None;
        autonomy.state_entered_at_ms = now_ms;
      } else if (config.side_line_timeout_ms == 0U ||
                 now_ms - autonomy.started_at_ms >=
                     config.side_line_timeout_ms) {
        failTowerPiecesAutonomy(autonomy,
                                TowerPiecesFaultReason::SideLineTimeout,
                                now_ms);
      }
      break;

    case TowerPiecesState::PostLineDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_line_delay_ms) {
        autonomy.state = TowerPiecesState::StrafeRight;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::StrafeRight:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.strafe_right_duration_ms) {
        autonomy.state = TowerPiecesState::PostStrafePause;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::PostStrafePause:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_strafe_pause_ms) {
        autonomy.state = TowerPiecesState::RotateClockwise;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::RotateClockwise:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.clockwise_rotation_duration_ms) {
        autonomy.state = TowerPiecesState::PostRotationPause;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::PostRotationPause:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_rotation_pause_ms) {
        autonomy.state = TowerPiecesState::ReverseTimed;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::ReverseTimed:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.reverse_duration_ms) {
        autonomy.state = TowerPiecesState::ShimmyRight;
        autonomy.state_entered_at_ms = now_ms;
        autonomy.shimmy_started_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::ShimmyLeft:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.shimmy_left_duration_ms) {
        autonomy.state = TowerPiecesState::ShimmyRight;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::ShimmyRight:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.shimmy_right_duration_ms) {
        autonomy.state = TowerPiecesState::ShimmyLeft;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::WaitForStart:
    case TowerPiecesState::Complete:
    case TowerPiecesState::Fault:
      break;
  }

  if (autonomy.state == TowerPiecesState::ShimmyLeft ||
      autonomy.state == TowerPiecesState::ShimmyRight) {
    update.back_line_detected =
        back_left_line_high || back_right_line_high;
    if (update.back_line_detected) {
      autonomy.state = TowerPiecesState::Complete;
      autonomy.fault_reason = TowerPiecesFaultReason::None;
      autonomy.state_entered_at_ms = now_ms;
    } else if (config.shimmy_timeout_ms == 0U ||
               now_ms - autonomy.shimmy_started_at_ms >=
                   config.shimmy_timeout_ms) {
      failTowerPiecesAutonomy(autonomy,
                              TowerPiecesFaultReason::ShimmyTimeout,
                              now_ms);
    }
  }

  update.state = autonomy.state;
  update.fault_reason = autonomy.fault_reason;
  update.side_line_count = autonomy.side_line_count;
  update.should_line_follow =
      autonomy.state == TowerPiecesState::ReverseLineFollow;
  update.should_initial_strafe_right =
      autonomy.state == TowerPiecesState::StrafeRight;
  update.should_rotate_clockwise =
      autonomy.state == TowerPiecesState::RotateClockwise;
  update.should_drive_backward =
      autonomy.state == TowerPiecesState::ReverseTimed;
  update.should_shimmy_left =
      autonomy.state == TowerPiecesState::ShimmyLeft;
  update.should_shimmy_right =
      autonomy.state == TowerPiecesState::ShimmyRight;
  return update;
}

void failTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                             const TowerPiecesFaultReason reason,
                             const Milliseconds now_ms) {
  autonomy.state = TowerPiecesState::Fault;
  autonomy.fault_reason = reason;
  autonomy.state_entered_at_ms = now_ms;
}

}  // namespace robot
