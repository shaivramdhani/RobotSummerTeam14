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
    case TowerPiecesState::FinalReverse:
      return "FINAL_REVERSE";
    case TowerPiecesState::PostFinalReverseDelay:
      return "POST_FINAL_REVERSE_DELAY";
    case TowerPiecesState::WinchOpen:
      return "WINCH_OPEN";
    case TowerPiecesState::PostWinchOpenDelay:
      return "POST_WINCH_OPEN_DELAY";
    case TowerPiecesState::ClawsOpen:
      return "CLAWS_OPEN";
    case TowerPiecesState::PostClawsOpenDelay:
      return "POST_CLAWS_OPEN_DELAY";
    case TowerPiecesState::MoveStepperBottom:
      return "MOVE_STEPPER_BOTTOM";
    case TowerPiecesState::PostStepperBottomDelay:
      return "POST_STEPPER_BOTTOM_DELAY";
    case TowerPiecesState::ClawsClosed:
      return "CLAWS_CLOSED";
    case TowerPiecesState::PostClawsClosedDelay:
      return "POST_CLAWS_CLOSED_DELAY";
    case TowerPiecesState::MoveStepperTop:
      return "MOVE_STEPPER_TOP";
    case TowerPiecesState::WinchClosed:
      return "WINCH_CLOSED";
    case TowerPiecesState::PreStepperBottomDelay:
      return "PRE_STEPPER_BOTTOM_DELAY";
    case TowerPiecesState::PreShimmyDelay:
      return "PRE_SHIMMY_DELAY";
    case TowerPiecesState::PostShimmyDelay:
      return "POST_SHIMMY_DELAY";
    case TowerPiecesState::Complete:
      return "COMPLETE";
    case TowerPiecesState::Fault:
      return "FAULT";
  }
  return "WAIT_FOR_START";
}

const char* towerPiecesShimmyInitialDirectionName(
    const TowerPiecesShimmyInitialDirection direction) {
  switch (direction) {
    case TowerPiecesShimmyInitialDirection::Left:
      return "LEFT";
    case TowerPiecesShimmyInitialDirection::Right:
      return "RIGHT";
  }
  return "INVALID";
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
    case TowerPiecesFaultReason::ServoCommandFailed:
      return "SERVO_COMMAND_FAILED";
    case TowerPiecesFaultReason::StepperCommandFailed:
      return "STEPPER_COMMAND_FAILED";
    case TowerPiecesFaultReason::StepperLimitSearchFailed:
      return "STEPPER_LIMIT_SEARCH_FAILED";
    case TowerPiecesFaultReason::ConflictingLimitSwitches:
      return "CONFLICTING_LIMIT_SWITCHES";
    case TowerPiecesFaultReason::ImuUnavailable:
      return "IMU_UNAVAILABLE";
    case TowerPiecesFaultReason::ImuStrafeFailed:
      return "IMU_STRAFE_FAILED";
    case TowerPiecesFaultReason::ImuTurnFailed:
      return "IMU_TURN_FAILED";
    case TowerPiecesFaultReason::ImuTurnTimeout:
      return "IMU_TURN_TIMEOUT";
  }
  return "NONE";
}

bool towerPiecesConfigValid(const TowerPiecesConfig& config,
                            const float maximum_allowed_duty,
                            const std::uint32_t
                                maximum_stepper_speed_steps_per_second) {
  const bool final_reverse_valid =
      std::isfinite(config.final_reverse_duty) &&
      config.final_reverse_duty >= 0.0F &&
      config.final_reverse_duty <= maximum_allowed_duty &&
      (config.final_reverse_duration_ms == 0U ||
       config.final_reverse_duty > 0.0F);
  const bool shimmy_direction_valid =
      config.shimmy_initial_direction ==
          TowerPiecesShimmyInitialDirection::Left ||
      config.shimmy_initial_direction ==
          TowerPiecesShimmyInitialDirection::Right;
  const Milliseconds initial_shimmy_duration_ms =
      config.shimmy_initial_direction ==
              TowerPiecesShimmyInitialDirection::Left
          ? config.shimmy_left_duration_ms
          : config.shimmy_right_duration_ms;

  return std::isfinite(config.reverse_line_duty) &&
         std::isfinite(config.clockwise_rotation_angle_deg) &&
         std::isfinite(config.reverse_duty) &&
         std::isfinite(maximum_allowed_duty) &&
         config.reverse_line_duty > 0.0F &&
         config.reverse_line_duty <= maximum_allowed_duty &&
         config.clockwise_rotation_angle_deg > 0.0F &&
         config.reverse_duty > 0.0F &&
         config.reverse_duty <= maximum_allowed_duty &&
         config.side_line_timeout_ms >
             config.side_line_ignore_after_start_ms &&
         config.post_line_delay_ms > 0U &&
         std::isfinite(config.strafe_right_duty) &&
         config.strafe_right_duty > 0.0F &&
         config.strafe_right_duty <= maximum_allowed_duty &&
         config.strafe_right_duration_ms > 0U &&
         config.post_strafe_pause_ms > 0U &&
         config.post_rotation_pause_ms > 0U &&
         config.reverse_duration_ms > 0U &&
         shimmy_direction_valid &&
         config.shimmy_right_duration_ms > 0U &&
         config.shimmy_left_duration_ms > 0U &&
         initial_shimmy_duration_ms >= 2U &&
         config.shimmy_timeout_ms > 0U &&
         std::isfinite(config.shimmy_duty) &&
         config.shimmy_duty > 0.0F &&
         config.shimmy_duty <= maximum_allowed_duty &&
         final_reverse_valid &&
         config.post_final_reverse_delay_ms > 0U &&
         config.post_winch_open_delay_ms > 0U &&
         config.post_claws_open_delay_ms > 0U &&
         config.initial_stepper_lift_steps > 0U &&
         config.stepper_down_speed_steps_per_second > 0U &&
         config.stepper_down_speed_steps_per_second <=
             maximum_stepper_speed_steps_per_second &&
         config.post_stepper_bottom_delay_ms > 0U &&
         config.post_claws_closed_delay_ms > 0U &&
         config.stepper_up_speed_steps_per_second > 0U &&
         config.stepper_up_speed_steps_per_second <=
             maximum_stepper_speed_steps_per_second;
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
  autonomy.side_line_armed = !side_line_high;
  autonomy.side_line_off_timing = !side_line_high;
  autonomy.side_line_off_started_at_ms = now_ms;
}

TowerPiecesUpdate updateTowerPiecesAutonomy(
    TowerPiecesAutonomy& autonomy, const TowerPiecesInputs& inputs,
    const TowerPiecesConfig& config, const Milliseconds now_ms) {
  TowerPiecesUpdate update{};

  const bool is_active = autonomy.state != TowerPiecesState::WaitForStart &&
                         autonomy.state != TowerPiecesState::Complete &&
                         autonomy.state != TowerPiecesState::Fault;
  if (is_active && inputs.bottom_limit_active && inputs.top_limit_active) {
    failTowerPiecesAutonomy(autonomy,
                            TowerPiecesFaultReason::ConflictingLimitSwitches,
                            now_ms);
  }

  switch (autonomy.state) {
    case TowerPiecesState::ReverseLineFollow:
      autonomy.side_line_ignore_active =
          now_ms - autonomy.started_at_ms <
          config.side_line_ignore_after_start_ms;
      if (autonomy.side_line_ignore_active) {
        autonomy.previous_side_line_high = inputs.side_line_high;
        autonomy.side_line_armed = false;
        autonomy.side_line_off_timing = false;
        break;
      }
      update.side_line_rising_edge =
          !autonomy.previous_side_line_high && inputs.side_line_high;
      if (!inputs.side_line_high) {
        if (!autonomy.side_line_off_timing) {
          autonomy.side_line_off_timing = true;
          autonomy.side_line_off_started_at_ms = now_ms;
        }
        const bool cooldown_complete =
            autonomy.side_line_count == 0U ||
            now_ms - autonomy.side_line_last_accepted_at_ms >=
                config.side_line_cooldown_ms;
        if (!autonomy.side_line_armed && cooldown_complete &&
            now_ms - autonomy.side_line_off_started_at_ms >=
                config.side_line_rearm_ms) {
          autonomy.side_line_armed = true;
        }
      } else {
        const bool cooldown_complete =
            autonomy.side_line_count == 0U ||
            now_ms - autonomy.side_line_last_accepted_at_ms >=
                config.side_line_cooldown_ms;
        if (!autonomy.side_line_armed &&
            autonomy.side_line_off_timing && cooldown_complete &&
            now_ms - autonomy.side_line_off_started_at_ms >=
                config.side_line_rearm_ms) {
          autonomy.side_line_armed = true;
        }
        autonomy.side_line_off_timing = false;
      }
      autonomy.previous_side_line_high = inputs.side_line_high;
      if (update.side_line_rising_edge && autonomy.side_line_armed &&
          autonomy.side_line_count < kTowerPiecesTargetSideLineCount) {
        ++autonomy.side_line_count;
        autonomy.side_line_armed = false;
        autonomy.side_line_last_accepted_at_ms = now_ms;
        update.side_line_detection_accepted = true;
        autonomy.last_side_line_detection_accepted = true;
        autonomy.last_side_line_detection_rejected = false;
      } else if (update.side_line_rising_edge) {
        update.side_line_detection_rejected = true;
        autonomy.last_side_line_detection_accepted = false;
        autonomy.last_side_line_detection_rejected = true;
        if (autonomy.side_line_rejected_count < UINT16_MAX) {
          ++autonomy.side_line_rejected_count;
        }
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
      if (inputs.clockwise_turn_complete) {
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
        autonomy.state = TowerPiecesState::PreShimmyDelay;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::PreShimmyDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.pre_shimmy_delay_ms) {
        autonomy.state =
            config.shimmy_initial_direction ==
                    TowerPiecesShimmyInitialDirection::Left
                ? TowerPiecesState::ShimmyLeft
                : TowerPiecesState::ShimmyRight;
        autonomy.state_entered_at_ms = now_ms;
        autonomy.shimmy_started_at_ms = now_ms;
        autonomy.first_shimmy_pulse = true;
      }
      break;

    case TowerPiecesState::ShimmyLeft:
      if (now_ms - autonomy.state_entered_at_ms >=
          (autonomy.first_shimmy_pulse
               ? config.shimmy_left_duration_ms / 2U
               : config.shimmy_left_duration_ms)) {
        autonomy.state = TowerPiecesState::ShimmyRight;
        autonomy.state_entered_at_ms = now_ms;
        autonomy.first_shimmy_pulse = false;
      }
      break;

    case TowerPiecesState::ShimmyRight:
      if (now_ms - autonomy.state_entered_at_ms >=
          (autonomy.first_shimmy_pulse
               ? config.shimmy_right_duration_ms / 2U
               : config.shimmy_right_duration_ms)) {
        autonomy.state = TowerPiecesState::ShimmyLeft;
        autonomy.state_entered_at_ms = now_ms;
        autonomy.first_shimmy_pulse = false;
      }
      break;

    case TowerPiecesState::PostShimmyDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_shimmy_delay_ms) {
        autonomy.state = config.final_reverse_duration_ms == 0U
                             ? TowerPiecesState::PostFinalReverseDelay
                             : TowerPiecesState::FinalReverse;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::FinalReverse:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.final_reverse_duration_ms) {
        autonomy.state = TowerPiecesState::PostFinalReverseDelay;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::PostFinalReverseDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_final_reverse_delay_ms) {
        autonomy.state = TowerPiecesState::WinchOpen;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::WinchOpen:
      autonomy.state = TowerPiecesState::PostWinchOpenDelay;
      autonomy.state_entered_at_ms = now_ms;
      break;

    case TowerPiecesState::PostWinchOpenDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_winch_open_delay_ms) {
        autonomy.state = TowerPiecesState::ClawsOpen;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::ClawsOpen:
      autonomy.state = TowerPiecesState::PostClawsOpenDelay;
      autonomy.state_entered_at_ms = now_ms;
      break;

    case TowerPiecesState::PostClawsOpenDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_claws_open_delay_ms) {
        autonomy.state = TowerPiecesState::PreStepperBottomDelay;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::PreStepperBottomDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
              config.pre_stepper_bottom_delay_ms &&
          inputs.initial_stepper_lift_complete) {
        autonomy.state = TowerPiecesState::MoveStepperBottom;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::MoveStepperBottom:
      if (inputs.bottom_limit_active) {
        autonomy.state = TowerPiecesState::PostStepperBottomDelay;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::PostStepperBottomDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_stepper_bottom_delay_ms) {
        autonomy.state = TowerPiecesState::ClawsClosed;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::ClawsClosed:
      autonomy.state = TowerPiecesState::PostClawsClosedDelay;
      autonomy.state_entered_at_ms = now_ms;
      break;

    case TowerPiecesState::PostClawsClosedDelay:
      if (now_ms - autonomy.state_entered_at_ms >=
          config.post_claws_closed_delay_ms) {
        autonomy.state = TowerPiecesState::MoveStepperTop;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::MoveStepperTop:
      if (inputs.top_limit_active) {
        autonomy.state = TowerPiecesState::WinchClosed;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;

    case TowerPiecesState::WinchClosed:
      autonomy.state = TowerPiecesState::Complete;
      autonomy.fault_reason = TowerPiecesFaultReason::None;
      autonomy.state_entered_at_ms = now_ms;
      break;

    case TowerPiecesState::WaitForStart:
    case TowerPiecesState::Complete:
    case TowerPiecesState::Fault:
      break;
  }

  if (autonomy.state == TowerPiecesState::ShimmyLeft ||
      autonomy.state == TowerPiecesState::ShimmyRight) {
    autonomy.back_line_detected =
        inputs.back_left_line_high || inputs.back_right_line_high;
    if (autonomy.back_line_detected) {
      autonomy.state = TowerPiecesState::PostShimmyDelay;
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
  update.back_line_detected = autonomy.back_line_detected;

  update.state = autonomy.state;
  update.fault_reason = autonomy.fault_reason;
  update.side_line_count = autonomy.side_line_count;
  update.side_line_rejected_count =
      autonomy.side_line_rejected_count;
  update.side_line_armed = autonomy.side_line_armed;
  update.side_line_ignore_active = autonomy.side_line_ignore_active;
  update.side_line_detection_accepted =
      autonomy.last_side_line_detection_accepted;
  update.side_line_detection_rejected =
      autonomy.last_side_line_detection_rejected;
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
  update.should_drive_final_reverse =
      autonomy.state == TowerPiecesState::FinalReverse;
  update.waiting_for_initial_stepper_lift =
      autonomy.state == TowerPiecesState::PreStepperBottomDelay &&
      now_ms - autonomy.state_entered_at_ms >=
          config.pre_stepper_bottom_delay_ms &&
      !inputs.initial_stepper_lift_complete;
  update.should_move_stepper_bottom =
      autonomy.state == TowerPiecesState::MoveStepperBottom;
  update.should_move_stepper_top =
      autonomy.state == TowerPiecesState::MoveStepperTop;
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
