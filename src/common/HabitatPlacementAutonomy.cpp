#include "common/HabitatPlacementAutonomy.h"

#include <cmath>

namespace robot {

namespace {

Milliseconds elapsedSince(const Milliseconds now_ms,
                          const Milliseconds then_ms) {
  return static_cast<Milliseconds>(now_ms - then_ms);
}

void enterState(HabitatPlacementAutonomy& autonomy,
                const HabitatPlacementState state,
                const Milliseconds now_ms) {
  autonomy.state = state;
  autonomy.state_entered_at_ms = now_ms;
}

bool dutyValid(const float duty, const float maximum_allowed_duty) {
  return std::isfinite(duty) && duty > 0.0F &&
         duty <= maximum_allowed_duty;
}

HabitatPlacementUpdate makeUpdate(
    const HabitatPlacementAutonomy& autonomy,
    const Milliseconds now_ms) {
  HabitatPlacementUpdate update{};
  update.state = autonomy.state;
  update.fault_reason = autonomy.fault_reason;
  update.time_in_state_ms =
      elapsedSince(now_ms, autonomy.state_entered_at_ms);
  update.should_reverse_line_follow =
      autonomy.state == HabitatPlacementState::ReverseLineFollow;
  update.should_turn_to_initial_heading =
      autonomy.state == HabitatPlacementState::TurnToInitialHeading;
  update.should_strafe_right_before_counter_clockwise =
      autonomy.state ==
      HabitatPlacementState::StrafeRightBeforeCounterClockwise;
  update.should_turn_counter_clockwise =
      autonomy.state == HabitatPlacementState::TurnCounterClockwise;
  update.should_drive_forward_to_slide =
      autonomy.state == HabitatPlacementState::ForwardToSlide;
  update.should_lower_slide =
      autonomy.state == HabitatPlacementState::ForwardToSlide ||
      autonomy.state == HabitatPlacementState::LowerSlide;
  update.should_open_pusher =
      autonomy.state == HabitatPlacementState::OpenPusher;
  update.should_drive_forward_push =
      autonomy.state == HabitatPlacementState::ForwardPush;
  update.should_drive_reverse_retreat =
      autonomy.state == HabitatPlacementState::ReverseRetreat;
  update.should_turn_clockwise =
      autonomy.state == HabitatPlacementState::TurnClockwise;
  update.should_drive_reverse_after_clockwise =
      autonomy.state == HabitatPlacementState::ReverseAfterClockwise;
  update.should_strafe_left_after_clockwise =
      autonomy.state == HabitatPlacementState::StrafeLeftAfterClockwise;
  update.should_strafe_right_after_clockwise =
      autonomy.state == HabitatPlacementState::StrafeRightAfterClockwise;
  update.should_drive_forward_exit =
      autonomy.state == HabitatPlacementState::ForwardExit;
  update.should_strafe_right =
      autonomy.state == HabitatPlacementState::StrafeRightToReturnLine;
  update.should_close_pusher =
      autonomy.state == HabitatPlacementState::ClosePusher;
  update.should_stop_drive =
      !(update.should_reverse_line_follow ||
        update.should_turn_to_initial_heading ||
        update.should_strafe_right_before_counter_clockwise ||
        update.should_turn_counter_clockwise ||
        update.should_drive_forward_to_slide ||
        update.should_drive_forward_push ||
        update.should_drive_reverse_retreat || update.should_turn_clockwise ||
        update.should_drive_reverse_after_clockwise ||
        update.should_strafe_left_after_clockwise ||
        update.should_strafe_right_after_clockwise ||
        update.should_drive_forward_exit || update.should_strafe_right);
  update.complete = autonomy.state == HabitatPlacementState::Complete;
  update.faulted = autonomy.state == HabitatPlacementState::Fault;
  return update;
}

}  // namespace

const char* habitatPlacementStateName(const HabitatPlacementState state) {
  switch (state) {
    case HabitatPlacementState::WaitForStart:
      return "WAIT_FOR_START";
    case HabitatPlacementState::ReverseLineFollow:
      return "REVERSE_LINE_FOLLOW";
    case HabitatPlacementState::PostLss1Delay:
      return "POST_LSS1_DELAY";
    case HabitatPlacementState::TurnToInitialHeading:
      return "TURN_TO_INITIAL_HEADING";
    case HabitatPlacementState::StrafeRightBeforeCounterClockwise:
      return "STRAFE_RIGHT_BEFORE_COUNTER_CLOCKWISE";
    case HabitatPlacementState::TurnCounterClockwise:
      return "TURN_COUNTER_CLOCKWISE";
    case HabitatPlacementState::ForwardToSlide:
      return "FORWARD_TO_SLIDE";
    case HabitatPlacementState::LowerSlide:
      return "LOWER_SLIDE";
    case HabitatPlacementState::OpenPusher:
      return "OPEN_PUSHER";
    case HabitatPlacementState::PostPusherOpenDelay:
      return "POST_PUSHER_OPEN_DELAY";
    case HabitatPlacementState::ForwardPush:
      return "FORWARD_PUSH";
    case HabitatPlacementState::ReverseRetreat:
      return "REVERSE_RETREAT";
    case HabitatPlacementState::TurnClockwise:
      return "TURN_CLOCKWISE";
    case HabitatPlacementState::ReverseAfterClockwise:
      return "REVERSE_AFTER_CLOCKWISE";
    case HabitatPlacementState::StrafeLeftAfterClockwise:
      return "STRAFE_LEFT_AFTER_CLOCKWISE";
    case HabitatPlacementState::StrafeRightAfterClockwise:
      return "STRAFE_RIGHT_AFTER_CLOCKWISE";
    case HabitatPlacementState::PostClockwiseDelay:
      return "POST_CLOCKWISE_DELAY";
    case HabitatPlacementState::ForwardExit:
      return "FORWARD_EXIT";
    case HabitatPlacementState::PostForwardDelay:
      return "POST_FORWARD_DELAY";
    case HabitatPlacementState::StrafeRightToReturnLine:
      return "STRAFE_RIGHT_TO_RETURN_LINE";
    case HabitatPlacementState::ClosePusher:
      return "CLOSE_PUSHER";
    case HabitatPlacementState::Complete:
      return "COMPLETE";
    case HabitatPlacementState::Fault:
      return "FAULT";
  }
  return "WAIT_FOR_START";
}

bool habitatPlacementStateRequiresImuTurn(
    const HabitatPlacementState state) {
  return state == HabitatPlacementState::TurnToInitialHeading ||
         state == HabitatPlacementState::TurnCounterClockwise ||
         state == HabitatPlacementState::TurnClockwise;
}

bool habitatPlacementStateRequiresImuStrafe(
    const HabitatPlacementState state) {
  return state ==
             HabitatPlacementState::StrafeRightBeforeCounterClockwise ||
         state == HabitatPlacementState::StrafeLeftAfterClockwise ||
         state == HabitatPlacementState::StrafeRightAfterClockwise ||
         state == HabitatPlacementState::StrafeRightToReturnLine;
}

const char* habitatPlacementReturnLineSourceName(
    const HabitatPlacementReturnLineSource source) {
  switch (source) {
    case HabitatPlacementReturnLineSource::Front:
      return "FRONT";
    case HabitatPlacementReturnLineSource::Rear:
      return "REAR";
  }
  return "FRONT";
}

const char* habitatPlacementFaultReasonName(
    const HabitatPlacementFaultReason reason) {
  switch (reason) {
    case HabitatPlacementFaultReason::None:
      return "NONE";
    case HabitatPlacementFaultReason::HardwareNotReady:
      return "HARDWARE_NOT_READY";
    case HabitatPlacementFaultReason::RearLinkStale:
      return "REAR_LINK_STALE";
    case HabitatPlacementFaultReason::RearLineDataStale:
      return "REAR_LINE_DATA_STALE";
    case HabitatPlacementFaultReason::LineLost:
      return "LINE_LOST";
    case HabitatPlacementFaultReason::Lss1Timeout:
      return "LSS1_TIMEOUT";
    case HabitatPlacementFaultReason::DriveCommandFailed:
      return "DRIVE_COMMAND_FAILED";
    case HabitatPlacementFaultReason::ImuUnavailable:
      return "IMU_UNAVAILABLE";
    case HabitatPlacementFaultReason::ImuTurnFailed:
      return "IMU_TURN_FAILED";
    case HabitatPlacementFaultReason::ImuTurnTimeout:
      return "IMU_TURN_TIMEOUT";
    case HabitatPlacementFaultReason::StepperCommandFailed:
      return "STEPPER_COMMAND_FAILED";
    case HabitatPlacementFaultReason::StepperTimeout:
      return "STEPPER_TIMEOUT";
    case HabitatPlacementFaultReason::ConflictingLimitSwitches:
      return "CONFLICTING_LIMIT_SWITCHES";
    case HabitatPlacementFaultReason::PusherCommandFailed:
      return "PUSHER_COMMAND_FAILED";
    case HabitatPlacementFaultReason::ReturnLineTimeout:
      return "RETURN_LINE_TIMEOUT";
    case HabitatPlacementFaultReason::ImuStrafeFailed:
      return "IMU_STRAFE_FAILED";
  }
  return "NONE";
}

bool habitatPlacementConfigValid(
    const HabitatPlacementConfig& config,
    const float maximum_allowed_duty,
    const std::uint32_t maximum_stepper_speed_steps_per_second) {
  return std::isfinite(maximum_allowed_duty) &&
         maximum_allowed_duty > 0.0F &&
         dutyValid(config.reverse_line_follow_duty,
                   maximum_allowed_duty) &&
         config.lss1_timeout_ms > 0U &&
         config.post_lss1_delay_ms > 0U &&
         config.initial_heading_turn_timeout_ms > 0U &&
         dutyValid(config.pre_counter_clockwise_strafe_right_duty,
                   maximum_allowed_duty) &&
         config.pre_counter_clockwise_strafe_right_duration_ms > 0U &&
         std::isfinite(config.counter_clockwise_angle_deg) &&
         config.counter_clockwise_angle_deg > 0.0F &&
         config.counter_clockwise_timeout_ms > 0U &&
         dutyValid(config.forward_to_slide_duty, maximum_allowed_duty) &&
         config.forward_to_slide_duration_ms > 0U &&
         config.stepper_down_speed_steps_per_second > 0U &&
         config.stepper_down_speed_steps_per_second <=
             maximum_stepper_speed_steps_per_second &&
         config.stepper_down_timeout_ms > 0U &&
         config.pusher_open_settle_ms > 0U &&
         dutyValid(config.push_forward_duty, maximum_allowed_duty) &&
         config.push_forward_duration_ms > 0U &&
         dutyValid(config.reverse_retreat_duty, maximum_allowed_duty) &&
         config.reverse_retreat_duration_ms > 0U &&
         std::isfinite(config.clockwise_angle_deg) &&
         config.clockwise_angle_deg > 0.0F &&
         config.clockwise_timeout_ms > 0U &&
         dutyValid(config.post_clockwise_reverse_duty,
                   maximum_allowed_duty) &&
         config.post_clockwise_reverse_duration_ms > 0U &&
         dutyValid(config.post_clockwise_strafe_left_duty,
                   maximum_allowed_duty) &&
         config.post_clockwise_strafe_left_duration_ms > 0U &&
         config.post_clockwise_strafe_right_duration_ms > 0U &&
         config.post_clockwise_delay_ms > 0U &&
         dutyValid(config.exit_forward_duty, maximum_allowed_duty) &&
         config.exit_forward_duration_ms > 0U &&
         config.post_forward_delay_ms > 0U &&
         dutyValid(config.strafe_right_duty, maximum_allowed_duty) &&
         config.strafe_right_timeout_ms > 0U &&
         (config.return_line_source ==
              HabitatPlacementReturnLineSource::Front ||
          config.return_line_source ==
              HabitatPlacementReturnLineSource::Rear);
}

void resetHabitatPlacementAutonomy(HabitatPlacementAutonomy& autonomy,
                                  const Milliseconds now_ms) {
  autonomy = {};
  autonomy.state_entered_at_ms = now_ms;
}

bool startHabitatPlacementAutonomy(
    HabitatPlacementAutonomy& autonomy,
    const float initial_heading_deg,
    const float counter_clockwise_relative_angle_deg,
    const Milliseconds now_ms) {
  resetHabitatPlacementAutonomy(autonomy, now_ms);
  if (!std::isfinite(initial_heading_deg) ||
      !std::isfinite(counter_clockwise_relative_angle_deg) ||
      std::fabs(counter_clockwise_relative_angle_deg) <= 0.0001F) {
    return false;
  }
  const float counter_clockwise_target_heading_deg =
      initial_heading_deg + counter_clockwise_relative_angle_deg;
  if (!std::isfinite(counter_clockwise_target_heading_deg)) {
    return false;
  }
  autonomy.state = HabitatPlacementState::ReverseLineFollow;
  autonomy.started_at_ms = now_ms;
  autonomy.initial_heading_captured = true;
  autonomy.initial_heading_deg = initial_heading_deg;
  autonomy.counter_clockwise_target_heading_deg =
      counter_clockwise_target_heading_deg;
  return true;
}

void failHabitatPlacementAutonomy(
    HabitatPlacementAutonomy& autonomy,
    const HabitatPlacementFaultReason reason,
    const Milliseconds now_ms) {
  autonomy.fault_reason = reason;
  enterState(autonomy, HabitatPlacementState::Fault, now_ms);
}

HabitatPlacementUpdate updateHabitatPlacementAutonomy(
    HabitatPlacementAutonomy& autonomy,
    const HabitatPlacementInputs& inputs,
    const HabitatPlacementConfig& config,
    const Milliseconds now_ms) {
  if (autonomy.state != HabitatPlacementState::WaitForStart &&
      autonomy.state != HabitatPlacementState::Complete &&
      autonomy.state != HabitatPlacementState::Fault &&
      inputs.bottom_limit_active && inputs.top_limit_active) {
    failHabitatPlacementAutonomy(
        autonomy, HabitatPlacementFaultReason::ConflictingLimitSwitches,
        now_ms);
    return makeUpdate(autonomy, now_ms);
  }

  const Milliseconds elapsed_ms =
      elapsedSince(now_ms, autonomy.state_entered_at_ms);
  switch (autonomy.state) {
    case HabitatPlacementState::WaitForStart:
    case HabitatPlacementState::Complete:
    case HabitatPlacementState::Fault:
      break;
    case HabitatPlacementState::ReverseLineFollow:
      if (inputs.lss1_black) {
        enterState(autonomy, HabitatPlacementState::PostLss1Delay,
                   now_ms);
      } else if (elapsed_ms >= config.lss1_timeout_ms) {
        failHabitatPlacementAutonomy(
            autonomy, HabitatPlacementFaultReason::Lss1Timeout, now_ms);
      }
      break;
    case HabitatPlacementState::PostLss1Delay:
      if (elapsed_ms >= config.post_lss1_delay_ms) {
        enterState(autonomy,
                   HabitatPlacementState::TurnToInitialHeading, now_ms);
      }
      break;
    case HabitatPlacementState::TurnToInitialHeading:
      if (inputs.initial_heading_turn_complete) {
        enterState(
            autonomy,
            HabitatPlacementState::StrafeRightBeforeCounterClockwise,
            now_ms);
      } else if (elapsed_ms >=
                 config.initial_heading_turn_timeout_ms) {
        failHabitatPlacementAutonomy(
            autonomy, HabitatPlacementFaultReason::ImuTurnTimeout,
            now_ms);
      }
      break;
    case HabitatPlacementState::StrafeRightBeforeCounterClockwise:
      if (elapsed_ms >=
          config.pre_counter_clockwise_strafe_right_duration_ms) {
        enterState(autonomy,
                   HabitatPlacementState::TurnCounterClockwise, now_ms);
      }
      break;
    case HabitatPlacementState::TurnCounterClockwise:
      if (inputs.counter_clockwise_turn_complete) {
        autonomy.slide_down_started_at_ms = now_ms;
        enterState(autonomy, HabitatPlacementState::ForwardToSlide,
                   now_ms);
      } else if (elapsed_ms >= config.counter_clockwise_timeout_ms) {
        failHabitatPlacementAutonomy(
            autonomy, HabitatPlacementFaultReason::ImuTurnTimeout, now_ms);
      }
      break;
    case HabitatPlacementState::ForwardToSlide:
      if (!inputs.bottom_limit_active &&
          elapsedSince(now_ms, autonomy.slide_down_started_at_ms) >=
              config.stepper_down_timeout_ms) {
        failHabitatPlacementAutonomy(
            autonomy, HabitatPlacementFaultReason::StepperTimeout, now_ms);
        break;
      }
      if (elapsed_ms >= config.forward_to_slide_duration_ms) {
        enterState(autonomy, HabitatPlacementState::LowerSlide, now_ms);
      }
      break;
    case HabitatPlacementState::LowerSlide:
      if (inputs.bottom_limit_active) {
        enterState(autonomy, HabitatPlacementState::OpenPusher, now_ms);
      } else if (elapsedSince(
                     now_ms, autonomy.slide_down_started_at_ms) >=
                 config.stepper_down_timeout_ms) {
        failHabitatPlacementAutonomy(
            autonomy, HabitatPlacementFaultReason::StepperTimeout, now_ms);
      }
      break;
    case HabitatPlacementState::OpenPusher:
      if (inputs.pusher_open_commanded) {
        enterState(autonomy,
                   HabitatPlacementState::PostPusherOpenDelay, now_ms);
      }
      break;
    case HabitatPlacementState::PostPusherOpenDelay:
      if (elapsed_ms >= config.pusher_open_settle_ms) {
        enterState(autonomy, HabitatPlacementState::ForwardPush, now_ms);
      }
      break;
    case HabitatPlacementState::ForwardPush:
      if (elapsed_ms >= config.push_forward_duration_ms) {
        enterState(autonomy, HabitatPlacementState::ReverseRetreat,
                   now_ms);
      }
      break;
    case HabitatPlacementState::ReverseRetreat:
      if (elapsed_ms >= config.reverse_retreat_duration_ms) {
        enterState(autonomy, HabitatPlacementState::TurnClockwise,
                   now_ms);
      }
      break;
    case HabitatPlacementState::TurnClockwise:
      if (inputs.clockwise_turn_complete) {
        enterState(autonomy,
                   HabitatPlacementState::ReverseAfterClockwise,
                   now_ms);
      } else if (elapsed_ms >= config.clockwise_timeout_ms) {
        failHabitatPlacementAutonomy(
            autonomy, HabitatPlacementFaultReason::ImuTurnTimeout, now_ms);
      }
      break;
    case HabitatPlacementState::ReverseAfterClockwise:
      if (elapsed_ms >=
          config.post_clockwise_reverse_duration_ms) {
        enterState(autonomy,
                   HabitatPlacementState::StrafeLeftAfterClockwise,
                   now_ms);
      }
      break;
    case HabitatPlacementState::StrafeLeftAfterClockwise:
      if (elapsed_ms >=
          config.post_clockwise_strafe_left_duration_ms) {
        enterState(
            autonomy,
            HabitatPlacementState::StrafeRightAfterClockwise, now_ms);
      }
      break;
    case HabitatPlacementState::StrafeRightAfterClockwise:
      if (elapsed_ms >=
          config.post_clockwise_strafe_right_duration_ms) {
        enterState(autonomy, HabitatPlacementState::PostClockwiseDelay,
                   now_ms);
      }
      break;
    case HabitatPlacementState::PostClockwiseDelay:
      if (elapsed_ms >= config.post_clockwise_delay_ms) {
        enterState(autonomy, HabitatPlacementState::ForwardExit, now_ms);
      }
      break;
    case HabitatPlacementState::ForwardExit:
      if (elapsed_ms >= config.exit_forward_duration_ms) {
        enterState(autonomy, HabitatPlacementState::PostForwardDelay,
                   now_ms);
      }
      break;
    case HabitatPlacementState::PostForwardDelay:
      if (elapsed_ms >= config.post_forward_delay_ms) {
        enterState(autonomy,
                   HabitatPlacementState::StrafeRightToReturnLine,
                   now_ms);
      }
      break;
    case HabitatPlacementState::StrafeRightToReturnLine:
      if (inputs.return_line_black) {
        enterState(autonomy, HabitatPlacementState::ClosePusher, now_ms);
      } else if (elapsed_ms >= config.strafe_right_timeout_ms) {
        failHabitatPlacementAutonomy(
            autonomy, HabitatPlacementFaultReason::ReturnLineTimeout,
            now_ms);
      }
      break;
    case HabitatPlacementState::ClosePusher:
      if (inputs.pusher_closed_commanded) {
        enterState(autonomy, HabitatPlacementState::Complete, now_ms);
      }
      break;
  }

  return makeUpdate(autonomy, now_ms);
}

}  // namespace robot
