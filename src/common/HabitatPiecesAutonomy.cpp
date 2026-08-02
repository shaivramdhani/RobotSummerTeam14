#include "common/HabitatPiecesAutonomy.h"

#include <cmath>

#include "common/ChassisMixer.h"

namespace robot {

const char* habitatPiecesStateName(const HabitatPiecesState state) {
  switch (state) {
    case HabitatPiecesState::WaitForStart:
      return "WAIT_FOR_START";
    case HabitatPiecesState::LineFollowing:
      return "LINE_FOLLOWING";
    case HabitatPiecesState::SideLineAligning:
      return "SIDE_LINE_ALIGNING";
    case HabitatPiecesState::DistanceStrafing:
      return "DISTANCE_STRAFING";
    case HabitatPiecesState::CompensationStrafing:
      return "COMPENSATION_STRAFING";
    case HabitatPiecesState::LoweringSlide:
      return "LOWERING_SLIDE";
    case HabitatPiecesState::ForwardToDistance:
      return "FORWARD_TO_DISTANCE";
    case HabitatPiecesState::PostPickupReversing:
      return "POST_PICKUP_REVERSING";
    case HabitatPiecesState::ReturnLineStrafing:
      return "RETURN_LINE_STRAFING";
    case HabitatPiecesState::WaitForSlideLift:
      return "WAIT_FOR_SLIDE_LIFT";
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
    case HabitatPiecesStopReason::SideSensorsUnavailable:
      return "SIDE_SENSORS_UNAVAILABLE";
    case HabitatPiecesStopReason::SideSensorDataStale:
      return "SIDE_SENSOR_DATA_STALE";
    case HabitatPiecesStopReason::BothSideLinesDetected:
      return "BOTH_SIDE_LINES_DETECTED";
    case HabitatPiecesStopReason::RunTimeout:
      return "RUN_TIMEOUT";
    case HabitatPiecesStopReason::FrontLineLost:
      return "FRONT_LINE_LOST";
    case HabitatPiecesStopReason::RearCommandFailed:
      return "REAR_COMMAND_FAILED";
    case HabitatPiecesStopReason::DistanceZoneCountReached:
      return "DISTANCE_ZONE_COUNT_REACHED";
    case HabitatPiecesStopReason::DistanceStrafeTimeout:
      return "DISTANCE_STRAFE_TIMEOUT";
    case HabitatPiecesStopReason::SlideDownTimeout:
      return "SLIDE_DOWN_TIMEOUT";
    case HabitatPiecesStopReason::SlideCommandFailed:
      return "SLIDE_COMMAND_FAILED";
    case HabitatPiecesStopReason::ForwardDistanceTimeout:
      return "FORWARD_DISTANCE_TIMEOUT";
    case HabitatPiecesStopReason::SlideLiftTimeout:
      return "SLIDE_LIFT_TIMEOUT";
    case HabitatPiecesStopReason::RearLineDataStale:
      return "REAR_LINE_DATA_STALE";
    case HabitatPiecesStopReason::RearLineSensorsUnavailable:
      return "REAR_LINE_SENSORS_UNAVAILABLE";
    case HabitatPiecesStopReason::ReturnLineTimeout:
      return "RETURN_LINE_TIMEOUT";
    case HabitatPiecesStopReason::ReturnLineDetected:
      return "RETURN_LINE_DETECTED";
    case HabitatPiecesStopReason::ConflictingSlideLimits:
      return "CONFLICTING_SLIDE_LIMITS";
  }
  return "CONFIGURATION_INCOMPLETE";
}

HabitatPiecesStrafeDirection oppositeHabitatPiecesStrafeDirection(
    const HabitatPiecesStrafeDirection direction) {
  switch (direction) {
    case HabitatPiecesStrafeDirection::Left:
      return HabitatPiecesStrafeDirection::Right;
    case HabitatPiecesStrafeDirection::Right:
      return HabitatPiecesStrafeDirection::Left;
    case HabitatPiecesStrafeDirection::None:
      return HabitatPiecesStrafeDirection::None;
  }
  return HabitatPiecesStrafeDirection::None;
}

const char* habitatPiecesStrafeDirectionName(
    const HabitatPiecesStrafeDirection direction) {
  switch (direction) {
    case HabitatPiecesStrafeDirection::None:
      return "NONE";
    case HabitatPiecesStrafeDirection::Left:
      return "LEFT";
    case HabitatPiecesStrafeDirection::Right:
      return "RIGHT";
  }
  return "NONE";
}

bool habitatPiecesConfigValid(const HabitatPiecesConfig& config,
                              const float maximum_duty,
                              const Milliseconds maximum_duration_ms,
                              const std::uint32_t
                                  maximum_stepper_speed_steps_per_second,
                              const std::uint32_t
                                  maximum_slide_position_steps) {
  return std::isfinite(config.line_follow_duty) &&
         std::isfinite(maximum_duty) && config.line_follow_duty > 0.0F &&
         maximum_duty > 0.0F && config.line_follow_duty <= maximum_duty &&
         config.lss2_detection_delay_ms > 0U &&
         config.run_timeout_ms > config.lss2_detection_delay_ms &&
         std::isfinite(config.reverse_duty) && config.reverse_duty > 0.0F &&
         config.reverse_duty <= maximum_duty &&
         config.reverse_duration_ms > 0U &&
         (config.distance_strafe_direction ==
              HabitatPiecesStrafeDirection::Left ||
          config.distance_strafe_direction ==
              HabitatPiecesStrafeDirection::Right) &&
         config.distance_threshold_mm > 0U &&
         config.distance_zone_target_count > 0U &&
         std::isfinite(config.distance_strafe_duty) &&
         config.distance_strafe_duty > 0.0F &&
         config.distance_strafe_duty <= maximum_duty &&
         maximum_duration_ms > 0U &&
         config.distance_strafe_timeout_ms > 0U &&
         config.distance_strafe_timeout_ms <=
             maximum_duration_ms &&
         std::isfinite(config.compensation_strafe_duty) &&
         config.compensation_strafe_duty > 0.0F &&
         config.compensation_strafe_duty <= maximum_duty &&
         config.compensation_strafe_duration_ms > 0U &&
         config.compensation_strafe_duration_ms <= maximum_duration_ms &&
         maximum_stepper_speed_steps_per_second > 0U &&
         config.slide_down_speed_steps_per_second > 0U &&
         config.slide_down_speed_steps_per_second <=
             maximum_stepper_speed_steps_per_second &&
         config.slide_down_timeout_ms > 0U &&
         config.slide_down_timeout_ms <= maximum_duration_ms &&
         std::isfinite(config.forward_to_distance_duty) &&
         config.forward_to_distance_duty > 0.0F &&
         config.forward_to_distance_duty <= maximum_duty &&
         config.forward_stop_distance_mm > 0U &&
         config.forward_to_distance_timeout_ms > 0U &&
         config.forward_to_distance_timeout_ms <= maximum_duration_ms &&
         maximum_slide_position_steps > 0U &&
         config.slide_lift_steps > 0U &&
         config.slide_lift_steps <= maximum_slide_position_steps &&
         config.slide_lift_speed_steps_per_second > 0U &&
         config.slide_lift_speed_steps_per_second <=
             maximum_stepper_speed_steps_per_second &&
         config.slide_lift_timeout_ms > 0U &&
         config.slide_lift_timeout_ms <= maximum_duration_ms &&
         std::isfinite(config.post_pickup_reverse_duty) &&
         config.post_pickup_reverse_duty > 0.0F &&
         config.post_pickup_reverse_duty <= maximum_duty &&
         config.post_pickup_reverse_duration_ms > 0U &&
         config.post_pickup_reverse_duration_ms <= maximum_duration_ms &&
         std::isfinite(config.return_strafe_duty) &&
         config.return_strafe_duty > 0.0F &&
         config.return_strafe_duty <= maximum_duty &&
         config.return_line_timeout_ms > 0U &&
         config.return_line_timeout_ms <= maximum_duration_ms;
}

FourWheelCommand makeHabitatPiecesSideAlignmentCommand(
    const float forward_duty, const bool lss2_left_latched,
    const bool lss3_right_latched, const Milliseconds now_ms,
    const Milliseconds command_timeout_ms) {
  if (!std::isfinite(forward_duty) || forward_duty <= 0.0F ||
      forward_duty > 1.0F || command_timeout_ms == 0U) {
    return disabledFourWheelCommand();
  }

  MotorCommand drive{};
  drive.enabled = true;
  drive.duty_command_milli = clampCommandMilli(
      static_cast<std::int16_t>(forward_duty * 1000.0F));
  drive.expires_at_ms = now_ms + command_timeout_ms;

  FourWheelCommand command{};
  command.front_left =
      lss2_left_latched ? disabledMotorCommand() : drive;
  command.back_left = command.front_left;
  command.front_right =
      lss3_right_latched ? disabledMotorCommand() : drive;
  command.back_right = command.front_right;
  return command;
}

FourWheelCommand makeHabitatPiecesDistanceStrafeCommand(
    const HabitatPiecesStrafeDirection direction, const float duty,
    const Milliseconds now_ms,
    const Milliseconds command_timeout_ms) {
  if ((direction != HabitatPiecesStrafeDirection::Left &&
       direction != HabitatPiecesStrafeDirection::Right) ||
      !std::isfinite(duty) || duty <= 0.0F || duty > 1.0F ||
      command_timeout_ms == 0U) {
    return disabledFourWheelCommand();
  }
  const float lateral =
      direction == HabitatPiecesStrafeDirection::Left ? -1.0F : 1.0F;
  return mixOpenLoopMecanum(lateral, 0.0F, 0.0F, duty, now_ms,
                            command_timeout_ms);
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
    const HabitatPiecesInputs& inputs, const Milliseconds now_ms) {
  HabitatPiecesUpdate update{};
  update.state = autonomy.state;
  update.stop_reason = autonomy.stop_reason;
  update.lss2_detection_armed = autonomy.lss2_detection_armed;
  update.lss2_black = inputs.lss2_black;
  update.lss3_black = inputs.lss3_black;
  update.lss2_latched = autonomy.lss2_latched;
  update.lss3_latched = autonomy.lss3_latched;
  update.distance_measurement_available =
      autonomy.distance_measurement_available;
  update.distance_zone_active = autonomy.distance_zone_active;
  update.distance_mm = autonomy.latest_distance_mm;
  update.distance_zone_count = autonomy.distance_zone_count;
  update.slide_bottom_ready = inputs.slide_bottom_ready;
  update.slide_lift_started = autonomy.slide_lift_started;
  update.slide_lift_complete = autonomy.slide_lift_complete;
  update.rear_left_black = inputs.rear_left_black;
  update.rear_right_black = inputs.rear_right_black;
  update.rear_line_detected = autonomy.rear_line_detected;
  update.target_reached = autonomy.state == HabitatPiecesState::Complete;

  if (autonomy.state == HabitatPiecesState::Reversing) {
    autonomy.reverse_elapsed_ms = now_ms - autonomy.state_entered_at_ms;
    if (autonomy.reverse_elapsed_ms >= config.reverse_duration_ms) {
      autonomy.state = HabitatPiecesState::DistanceStrafing;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.distance_strafe_elapsed_ms = 0U;
      autonomy.distance_zone_count = 0U;
      autonomy.distance_zone_active = false;
      autonomy.distance_measurement_available = false;
      autonomy.distance_sequence_initialized =
          inputs.distance_sample.available;
      autonomy.last_distance_measurement_sequence =
          inputs.distance_sample.measurement_sequence;
      update.state = autonomy.state;
      update.should_stop = false;
      update.should_distance_strafe = true;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_reverse = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::DistanceStrafing) {
    autonomy.distance_strafe_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    autonomy.distance_measurement_available =
        inputs.distance_sample.available;
    update.distance_measurement_available =
        inputs.distance_sample.available;

    if (inputs.distance_sample.available) {
      autonomy.latest_distance_mm = inputs.distance_sample.distance_mm;
      update.distance_mm = inputs.distance_sample.distance_mm;
      const bool new_measurement =
          !autonomy.distance_sequence_initialized ||
          inputs.distance_sample.measurement_sequence !=
              autonomy.last_distance_measurement_sequence;
      if (new_measurement) {
        autonomy.distance_sequence_initialized = true;
        autonomy.last_distance_measurement_sequence =
            inputs.distance_sample.measurement_sequence;
        update.distance_sample_new = true;
        const bool in_zone =
            inputs.distance_sample.distance_mm >
            config.distance_threshold_mm;
        if (in_zone && !autonomy.distance_zone_active) {
          if (autonomy.distance_zone_count < UINT16_MAX) {
            ++autonomy.distance_zone_count;
          }
          update.distance_zone_entered = true;
        }
        autonomy.distance_zone_active = in_zone;
      }
    }
    update.distance_zone_active = autonomy.distance_zone_active;
    update.distance_zone_count = autonomy.distance_zone_count;

    if (autonomy.distance_zone_count >=
        config.distance_zone_target_count) {
      autonomy.state = HabitatPiecesState::CompensationStrafing;
      autonomy.stop_reason =
          HabitatPiecesStopReason::DistanceZoneCountReached;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.compensation_strafe_elapsed_ms = 0U;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.target_reached = false;
      update.should_stop = false;
      update.should_compensation_strafe = true;
      update.transitioned = true;
      return update;
    }

    if (config.distance_strafe_timeout_ms == 0U ||
        autonomy.distance_strafe_elapsed_ms >=
            config.distance_strafe_timeout_ms) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason =
          HabitatPiecesStopReason::DistanceStrafeTimeout;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.timed_out = true;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }

    update.should_stop = false;
    update.should_distance_strafe = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::CompensationStrafing) {
    autonomy.compensation_strafe_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    if (autonomy.compensation_strafe_elapsed_ms >=
        config.compensation_strafe_duration_ms) {
      autonomy.state = HabitatPiecesState::LoweringSlide;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.slide_down_elapsed_ms = 0U;
      update.state = autonomy.state;
      update.should_start_slide_down = true;
      update.should_lower_slide = true;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_compensation_strafe = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::LoweringSlide) {
    autonomy.slide_down_elapsed_ms = now_ms - autonomy.state_entered_at_ms;
    if (inputs.slide_bottom_limit_active &&
        inputs.slide_top_limit_active) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason =
          HabitatPiecesStopReason::ConflictingSlideLimits;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
    if (inputs.slide_down_failed) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason = HabitatPiecesStopReason::SlideCommandFailed;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
    if (inputs.slide_bottom_ready) {
      autonomy.state = HabitatPiecesState::ForwardToDistance;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.forward_to_distance_elapsed_ms = 0U;
      autonomy.distance_measurement_available = false;
      autonomy.distance_sequence_initialized =
          inputs.distance_sample.available;
      autonomy.last_distance_measurement_sequence =
          inputs.distance_sample.measurement_sequence;
      update.state = autonomy.state;
      update.transitioned = true;
      return update;
    }
    if (config.slide_down_timeout_ms == 0U ||
        autonomy.slide_down_elapsed_ms >= config.slide_down_timeout_ms) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason = HabitatPiecesStopReason::SlideDownTimeout;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.timed_out = true;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
    update.should_lower_slide = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::ForwardToDistance) {
    autonomy.forward_to_distance_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    const bool new_measurement =
        inputs.distance_sample.available &&
        (!autonomy.distance_sequence_initialized ||
         inputs.distance_sample.measurement_sequence !=
             autonomy.last_distance_measurement_sequence);
    autonomy.distance_measurement_available = new_measurement;
    update.distance_measurement_available = new_measurement;
    update.distance_sample_new = new_measurement;
    if (new_measurement) {
      autonomy.distance_sequence_initialized = true;
      autonomy.last_distance_measurement_sequence =
          inputs.distance_sample.measurement_sequence;
      autonomy.latest_distance_mm = inputs.distance_sample.distance_mm;
      update.distance_mm = inputs.distance_sample.distance_mm;
    }
    if (new_measurement &&
        inputs.distance_sample.distance_mm <=
            config.forward_stop_distance_mm) {
      autonomy.state = HabitatPiecesState::PostPickupReversing;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.post_pickup_reverse_elapsed_ms = 0U;
      autonomy.slide_lift_started = true;
      autonomy.slide_lift_complete = false;
      autonomy.slide_lift_started_at_ms = now_ms;
      autonomy.slide_lift_elapsed_ms = 0U;
      update.state = autonomy.state;
      update.forward_distance_reached = true;
      update.slide_lift_started = true;
      update.should_start_slide_lift = true;
      update.should_stop = false;
      update.should_post_pickup_reverse = true;
      update.transitioned = true;
      return update;
    }
    if (config.forward_to_distance_timeout_ms == 0U ||
        autonomy.forward_to_distance_elapsed_ms >=
            config.forward_to_distance_timeout_ms) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason =
          HabitatPiecesStopReason::ForwardDistanceTimeout;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.timed_out = true;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_drive_forward_to_distance = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::PostPickupReversing ||
      autonomy.state == HabitatPiecesState::ReturnLineStrafing ||
      autonomy.state == HabitatPiecesState::WaitForSlideLift) {
    autonomy.slide_lift_elapsed_ms =
        now_ms - autonomy.slide_lift_started_at_ms;
    autonomy.slide_lift_complete =
        autonomy.slide_lift_complete || inputs.slide_lift_complete;
    update.slide_lift_started = autonomy.slide_lift_started;
    update.slide_lift_complete = autonomy.slide_lift_complete;
    if (inputs.slide_lift_failed) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason = HabitatPiecesStopReason::SlideCommandFailed;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
    if (!autonomy.slide_lift_complete &&
        (config.slide_lift_timeout_ms == 0U ||
         autonomy.slide_lift_elapsed_ms >= config.slide_lift_timeout_ms)) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason = HabitatPiecesStopReason::SlideLiftTimeout;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.timed_out = true;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
  }

  if (autonomy.state == HabitatPiecesState::PostPickupReversing) {
    autonomy.post_pickup_reverse_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    if (autonomy.post_pickup_reverse_elapsed_ms >=
        config.post_pickup_reverse_duration_ms) {
      autonomy.state = HabitatPiecesState::ReturnLineStrafing;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.return_strafe_elapsed_ms = 0U;
      update.state = autonomy.state;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_post_pickup_reverse = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::ReturnLineStrafing) {
    autonomy.return_strafe_elapsed_ms = now_ms - autonomy.state_entered_at_ms;
    autonomy.rear_line_detected =
        inputs.rear_left_black || inputs.rear_right_black;
    update.rear_line_detected = autonomy.rear_line_detected;
    if (autonomy.rear_line_detected) {
      autonomy.state = autonomy.slide_lift_complete
                           ? HabitatPiecesState::Complete
                           : HabitatPiecesState::WaitForSlideLift;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.target_reached =
          autonomy.state == HabitatPiecesState::Complete;
      update.should_wait_for_slide_lift =
          autonomy.state == HabitatPiecesState::WaitForSlideLift;
      if (autonomy.state == HabitatPiecesState::Complete) {
        autonomy.stop_reason = HabitatPiecesStopReason::ReturnLineDetected;
        update.stop_reason = autonomy.stop_reason;
      }
      update.transitioned = true;
      return update;
    }
    if (config.return_line_timeout_ms == 0U ||
        autonomy.return_strafe_elapsed_ms >=
            config.return_line_timeout_ms) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason = HabitatPiecesStopReason::ReturnLineTimeout;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.timed_out = true;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_return_line_strafe = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::WaitForSlideLift) {
    if (autonomy.slide_lift_complete) {
      autonomy.state = HabitatPiecesState::Complete;
      autonomy.stop_reason = HabitatPiecesStopReason::ReturnLineDetected;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.target_reached = true;
      update.transitioned = true;
      return update;
    }
    update.should_wait_for_slide_lift = true;
    return update;
  }

  if (autonomy.state != HabitatPiecesState::LineFollowing &&
      autonomy.state != HabitatPiecesState::SideLineAligning) {
    return update;
  }

  autonomy.run_elapsed_ms = now_ms - autonomy.run_started_at_ms;
  if (autonomy.state == HabitatPiecesState::LineFollowing) {
    autonomy.lss2_detection_armed =
        autonomy.run_elapsed_ms >= config.lss2_detection_delay_ms;
  }
  update.lss2_detection_armed = autonomy.lss2_detection_armed;

  if (autonomy.lss2_detection_armed) {
    update.lss2_newly_latched =
        inputs.lss2_black && !autonomy.lss2_latched;
    update.lss3_newly_latched =
        inputs.lss3_black && !autonomy.lss3_latched;
    autonomy.lss2_latched =
        autonomy.lss2_latched || inputs.lss2_black;
    autonomy.lss3_latched =
        autonomy.lss3_latched || inputs.lss3_black;
    update.lss2_latched = autonomy.lss2_latched;
    update.lss3_latched = autonomy.lss3_latched;
  }

  if (autonomy.lss2_latched && autonomy.lss3_latched) {
    autonomy.state = HabitatPiecesState::Reversing;
    autonomy.stop_reason =
        HabitatPiecesStopReason::BothSideLinesDetected;
    autonomy.state_entered_at_ms = now_ms;
    autonomy.reverse_elapsed_ms = 0U;
    update.state = autonomy.state;
    update.stop_reason = autonomy.stop_reason;
    update.target_reached = false;
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

  if (autonomy.lss2_latched || autonomy.lss3_latched) {
    const bool state_changed =
        autonomy.state != HabitatPiecesState::SideLineAligning;
    autonomy.state = HabitatPiecesState::SideLineAligning;
    if (state_changed) {
      autonomy.state_entered_at_ms = now_ms;
      update.transitioned = true;
    }
    update.state = autonomy.state;
    update.should_stop = false;
    update.should_align_side_lines = true;
    update.should_drive_left_side = !autonomy.lss2_latched;
    update.should_drive_right_side = !autonomy.lss3_latched;
    return update;
  }

  update.should_stop = false;
  update.should_line_follow = true;
  return update;
}

HabitatPiecesUpdate updateHabitatPiecesAutonomy(
    HabitatPiecesAutonomy& autonomy, const HabitatPiecesConfig& config,
    const bool lss2_black, const bool lss3_black,
    const Milliseconds now_ms,
    const HabitatPiecesDistanceSample& distance_sample) {
  HabitatPiecesInputs inputs{};
  inputs.lss2_black = lss2_black;
  inputs.lss3_black = lss3_black;
  inputs.distance_sample = distance_sample;
  return updateHabitatPiecesAutonomy(autonomy, config, inputs, now_ms);
}

}  // namespace robot
