#include "common/HabitatPiecesAutonomy.h"

#include <cmath>

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
    case HabitatPiecesState::PostCountStopDelay:
      return "POST_COUNT_STOP_DELAY";
    case HabitatPiecesState::ExitStrafePulse:
      return "EXIT_STRAFE_PULSE";
    case HabitatPiecesState::ExitDistanceCheck:
      return "EXIT_DISTANCE_CHECK";
    case HabitatPiecesState::LowerSlide:
      return "LOWER_SLIDE";
    case HabitatPiecesState::ApproachPiece:
      return "APPROACH_PIECE";
    case HabitatPiecesState::ReverseAfterPickup:
      return "REVERSE_AFTER_PICKUP";
    case HabitatPiecesState::RearLineReacquire:
      return "REAR_LINE_REACQUIRE";
    case HabitatPiecesState::WaitForLiftCompletion:
      return "WAIT_FOR_LIFT_COMPLETION";
    case HabitatPiecesState::LiftStartDelay:
      return "LIFT_START_DELAY";
    case HabitatPiecesState::PreLiftReverse:
      return "PRE_LIFT_REVERSE";
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
    case HabitatPiecesStopReason::Lss2Detected:
      return "LSS2_DETECTED";
    case HabitatPiecesStopReason::ImuUnavailable:
      return "IMU_UNAVAILABLE";
    case HabitatPiecesStopReason::ImuStrafeFailed:
      return "IMU_STRAFE_FAILED";
    case HabitatPiecesStopReason::DistanceExitReached:
      return "DISTANCE_EXIT_REACHED";
    case HabitatPiecesStopReason::SlideDownTimeout:
      return "SLIDE_DOWN_TIMEOUT";
    case HabitatPiecesStopReason::ApproachLimitTimeout:
      return "APPROACH_LIMIT_TIMEOUT";
    case HabitatPiecesStopReason::LiftTimeout:
      return "LIFT_TIMEOUT";
    case HabitatPiecesStopReason::RearLineTimeout:
      return "REAR_LINE_TIMEOUT";
    case HabitatPiecesStopReason::StepperCommandFailed:
      return "STEPPER_COMMAND_FAILED";
    case HabitatPiecesStopReason::RearLineReached:
      return "REAR_LINE_REACHED";
    case HabitatPiecesStopReason::Lss3Detected:
      return "LSS3_DETECTED";
  }
  return "CONFIGURATION_INCOMPLETE";
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
                              const Milliseconds
                                  maximum_distance_strafe_timeout_ms) {
  return std::isfinite(config.line_follow_duty) &&
         std::isfinite(maximum_duty) && config.line_follow_duty > 0.0F &&
         maximum_duty > 0.0F && config.line_follow_duty <= maximum_duty &&
         config.side_line_ignore_after_start_ms > 0U &&
         config.run_timeout_ms > config.side_line_ignore_after_start_ms &&
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
         maximum_distance_strafe_timeout_ms > 0U &&
         config.distance_strafe_timeout_ms > 0U &&
         config.distance_strafe_timeout_ms <=
             maximum_distance_strafe_timeout_ms &&
         config.distance_count_ignore_ms <
             config.distance_strafe_timeout_ms &&
         config.post_count_stop_delay_ms > 0U &&
         config.post_count_stop_delay_ms <=
             config.distance_strafe_timeout_ms &&
         std::isfinite(config.exit_strafe_duty) &&
         config.exit_strafe_duty > 0.0F &&
         config.exit_strafe_duty <= maximum_duty &&
         config.exit_strafe_pulse_ms > 0U &&
         config.exit_strafe_pulse_ms <=
             config.distance_strafe_timeout_ms &&
         static_cast<std::uint64_t>(config.distance_count_ignore_ms) +
                 static_cast<std::uint64_t>(
                     config.post_count_stop_delay_ms) +
                 static_cast<std::uint64_t>(config.exit_strafe_pulse_ms) <
             static_cast<std::uint64_t>(
                 config.distance_strafe_timeout_ms) &&
         config.slide_down_speed_steps_per_second > 0U &&
         config.slide_down_timeout_ms > 0U &&
         config.slide_down_timeout_ms <=
             maximum_distance_strafe_timeout_ms &&
         std::isfinite(config.approach_forward_duty) &&
         config.approach_forward_duty > 0.0F &&
         config.approach_forward_duty <= maximum_duty &&
         config.approach_timeout_ms > 0U &&
         config.approach_timeout_ms <=
             maximum_distance_strafe_timeout_ms &&
         config.pre_lift_reverse_duration_ms > 0U &&
         config.pre_lift_reverse_duration_ms <=
             maximum_distance_strafe_timeout_ms &&
         config.lift_steps > 0U &&
         config.lift_speed_steps_per_second > 0U &&
         config.lift_timeout_ms > 0U &&
         config.lift_timeout_ms <= maximum_distance_strafe_timeout_ms &&
         config.lift_start_delay_ms > 0U &&
         config.lift_start_delay_ms <=
             maximum_distance_strafe_timeout_ms &&
         std::isfinite(config.post_pickup_reverse_duty) &&
         config.post_pickup_reverse_duty > 0.0F &&
         config.post_pickup_reverse_duty <= maximum_duty &&
         config.post_pickup_reverse_duration_ms > 0U &&
         config.post_pickup_reverse_duration_ms <=
             maximum_distance_strafe_timeout_ms &&
         std::isfinite(config.rear_line_reacquire_duty) &&
         config.rear_line_reacquire_duty > 0.0F &&
         config.rear_line_reacquire_duty <= maximum_duty &&
         config.rear_line_reacquire_timeout_ms > 0U &&
         config.rear_line_reacquire_timeout_ms <=
             maximum_distance_strafe_timeout_ms;
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
    const bool lss2_black, const bool lss3_black,
    const Milliseconds now_ms,
    const HabitatPiecesDistanceSample& distance_sample,
    const HabitatPiecesMechanismInputs& mechanism_inputs) {
  HabitatPiecesUpdate update{};
  update.state = autonomy.state;
  update.stop_reason = autonomy.stop_reason;
  update.lss2_detection_armed = autonomy.lss2_detection_armed;
  update.lss2_black = lss2_black;
  update.lss3_black = lss3_black;
  update.lss2_latched = autonomy.lss2_latched;
  update.lss3_latched = autonomy.lss3_latched;
  update.distance_measurement_available =
      autonomy.distance_measurement_available;
  update.distance_zone_active = autonomy.distance_zone_active;
  update.distance_mm = autonomy.latest_distance_mm;
  update.distance_zone_count = autonomy.distance_zone_count;
  update.distance_exit_pulse_count =
      autonomy.distance_exit_pulse_count;
  update.distance_substituted_no_target =
      autonomy.distance_substituted_no_target;
  update.distance_exit_above_threshold =
      autonomy.distance_exit_above_threshold;
  update.approach_limit_reached = autonomy.approach_limit_reached;
  update.lift_complete = autonomy.lift_complete;
  update.rear_line_detected = autonomy.rear_line_latched;
  update.target_reached = autonomy.state == HabitatPiecesState::Complete;

  if (autonomy.state == HabitatPiecesState::Reversing) {
    autonomy.reverse_elapsed_ms = now_ms - autonomy.state_entered_at_ms;
    if (autonomy.reverse_elapsed_ms >= config.reverse_duration_ms) {
      autonomy.state = HabitatPiecesState::DistanceStrafing;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.distance_strafe_started_at_ms = now_ms;
      autonomy.distance_strafe_elapsed_ms = 0U;
      autonomy.post_count_stop_elapsed_ms = 0U;
      autonomy.exit_strafe_pulse_elapsed_ms = 0U;
      autonomy.distance_zone_count = 0U;
      autonomy.distance_exit_pulse_count = 0U;
      autonomy.distance_zone_active = false;
      autonomy.distance_exit_above_threshold = false;
      autonomy.distance_measurement_available = false;
      autonomy.distance_sequence_initialized = distance_sample.available;
      autonomy.last_distance_measurement_sequence =
          distance_sample.measurement_sequence;
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
        now_ms - autonomy.distance_strafe_started_at_ms;
    update.distance_count_ignore_active =
        autonomy.distance_strafe_elapsed_ms <
        config.distance_count_ignore_ms;
    autonomy.distance_measurement_available = distance_sample.available;
    update.distance_measurement_available = distance_sample.available;

    if (distance_sample.available) {
      autonomy.latest_distance_mm = distance_sample.distance_mm;
      update.distance_mm = distance_sample.distance_mm;
      const bool new_measurement =
          !autonomy.distance_sequence_initialized ||
          distance_sample.measurement_sequence !=
              autonomy.last_distance_measurement_sequence;
      if (new_measurement) {
        autonomy.distance_sequence_initialized = true;
        autonomy.last_distance_measurement_sequence =
            distance_sample.measurement_sequence;
        autonomy.distance_substituted_no_target =
            distance_sample.substituted_no_target;
        update.distance_substituted_no_target =
            distance_sample.substituted_no_target;
        update.distance_sample_new = true;
        const bool in_zone =
            distance_sample.distance_mm <= config.distance_threshold_mm;
        if (!update.distance_count_ignore_active && in_zone &&
            !autonomy.distance_zone_active) {
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
      autonomy.state = HabitatPiecesState::PostCountStopDelay;
      autonomy.stop_reason =
          HabitatPiecesStopReason::DistanceZoneCountReached;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.post_count_stop_elapsed_ms = 0U;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.should_wait_after_distance_count = true;
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

  if (autonomy.state == HabitatPiecesState::PostCountStopDelay ||
      autonomy.state == HabitatPiecesState::ExitStrafePulse ||
      autonomy.state == HabitatPiecesState::ExitDistanceCheck) {
    autonomy.distance_strafe_elapsed_ms =
        now_ms - autonomy.distance_strafe_started_at_ms;
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
  }

  if (autonomy.state == HabitatPiecesState::PostCountStopDelay) {
    autonomy.post_count_stop_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    if (autonomy.post_count_stop_elapsed_ms >=
        config.post_count_stop_delay_ms) {
      autonomy.state = HabitatPiecesState::ExitStrafePulse;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.exit_strafe_pulse_elapsed_ms = 0U;
      if (autonomy.distance_exit_pulse_count < UINT16_MAX) {
        ++autonomy.distance_exit_pulse_count;
      }
      update.state = autonomy.state;
      update.distance_exit_pulse_count =
          autonomy.distance_exit_pulse_count;
      update.should_stop = false;
      update.should_exit_strafe_pulse = true;
      update.transitioned = true;
      return update;
    }
    update.should_wait_after_distance_count = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::ExitStrafePulse) {
    autonomy.exit_strafe_pulse_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    if (autonomy.exit_strafe_pulse_elapsed_ms >=
        config.exit_strafe_pulse_ms) {
      autonomy.state = HabitatPiecesState::ExitDistanceCheck;
      autonomy.state_entered_at_ms = now_ms;
      // Require a measurement acquired after the pulse has stopped. This
      // prevents the exit decision from reusing a reading taken while moving.
      autonomy.distance_sequence_initialized = distance_sample.available;
      autonomy.last_distance_measurement_sequence =
          distance_sample.measurement_sequence;
      update.state = autonomy.state;
      update.should_check_exit_distance = true;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_exit_strafe_pulse = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::ExitDistanceCheck) {
    autonomy.distance_measurement_available = distance_sample.available;
    update.distance_measurement_available = distance_sample.available;
    if (distance_sample.available) {
      autonomy.latest_distance_mm = distance_sample.distance_mm;
      update.distance_mm = distance_sample.distance_mm;
      const bool new_measurement =
          !autonomy.distance_sequence_initialized ||
          distance_sample.measurement_sequence !=
              autonomy.last_distance_measurement_sequence;
      if (new_measurement) {
        autonomy.distance_sequence_initialized = true;
        autonomy.last_distance_measurement_sequence =
            distance_sample.measurement_sequence;
        autonomy.distance_substituted_no_target =
            distance_sample.substituted_no_target;
        update.distance_substituted_no_target =
            distance_sample.substituted_no_target;
        update.distance_sample_new = true;
        autonomy.distance_zone_active =
            distance_sample.distance_mm <= config.distance_threshold_mm;
        update.distance_zone_active = autonomy.distance_zone_active;
        autonomy.distance_exit_above_threshold =
            !autonomy.distance_zone_active;
        update.distance_exit_above_threshold =
            autonomy.distance_exit_above_threshold;
        if (autonomy.distance_exit_above_threshold) {
          autonomy.state = HabitatPiecesState::LowerSlide;
          autonomy.stop_reason =
              HabitatPiecesStopReason::DistanceExitReached;
          autonomy.state_entered_at_ms = now_ms;
          autonomy.slide_down_elapsed_ms = 0U;
          update.state = autonomy.state;
          update.stop_reason = autonomy.stop_reason;
          update.transitioned = true;
          return update;
        }

        autonomy.state = HabitatPiecesState::ExitStrafePulse;
        autonomy.state_entered_at_ms = now_ms;
        autonomy.exit_strafe_pulse_elapsed_ms = 0U;
        if (autonomy.distance_exit_pulse_count < UINT16_MAX) {
          ++autonomy.distance_exit_pulse_count;
        }
        update.state = autonomy.state;
        update.distance_exit_pulse_count =
            autonomy.distance_exit_pulse_count;
        update.should_stop = false;
        update.should_exit_strafe_pulse = true;
        update.transitioned = true;
        return update;
      }
    }
    update.should_check_exit_distance = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::LowerSlide) {
    autonomy.slide_down_elapsed_ms = now_ms - autonomy.state_entered_at_ms;
    if (mechanism_inputs.bottom_limit_active) {
      autonomy.state = HabitatPiecesState::ApproachPiece;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.approach_elapsed_ms = 0U;
      autonomy.approach_limit_reached = false;
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

  if (autonomy.state == HabitatPiecesState::ApproachPiece) {
    autonomy.approach_elapsed_ms = now_ms - autonomy.state_entered_at_ms;
    if (mechanism_inputs.approach_limit_active) {
      autonomy.approach_limit_reached = true;
      update.approach_limit_reached = true;
      autonomy.state = HabitatPiecesState::PreLiftReverse;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.pre_lift_reverse_elapsed_ms = 0U;
      update.state = autonomy.state;
      update.transitioned = true;
      return update;
    }
    if (config.approach_timeout_ms == 0U ||
        autonomy.approach_elapsed_ms >= config.approach_timeout_ms) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason =
          HabitatPiecesStopReason::ApproachLimitTimeout;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.timed_out = true;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_drive_forward_to_piece = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::PreLiftReverse) {
    autonomy.pre_lift_reverse_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    if (autonomy.pre_lift_reverse_elapsed_ms >=
        config.pre_lift_reverse_duration_ms) {
      autonomy.state = HabitatPiecesState::LiftStartDelay;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.lift_started_at_ms = now_ms;
      autonomy.lift_elapsed_ms = 0U;
      autonomy.lift_start_delay_elapsed_ms = 0U;
      autonomy.post_pickup_reverse_elapsed_ms = 0U;
      autonomy.lift_complete = false;
      update.state = autonomy.state;
      update.should_start_lift = true;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_drive_back_before_lift = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::LiftStartDelay ||
      autonomy.state == HabitatPiecesState::ReverseAfterPickup ||
      autonomy.state == HabitatPiecesState::RearLineReacquire ||
      autonomy.state == HabitatPiecesState::WaitForLiftCompletion) {
    autonomy.lift_elapsed_ms = now_ms - autonomy.lift_started_at_ms;
    autonomy.lift_complete =
        autonomy.lift_complete || mechanism_inputs.lift_complete;
    update.lift_complete = autonomy.lift_complete;
    if (!autonomy.lift_complete &&
        (config.lift_timeout_ms == 0U ||
         autonomy.lift_elapsed_ms >= config.lift_timeout_ms)) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason = HabitatPiecesStopReason::LiftTimeout;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.timed_out = true;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
  }

  if (autonomy.state == HabitatPiecesState::LiftStartDelay) {
    autonomy.lift_start_delay_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    if (autonomy.lift_start_delay_elapsed_ms >=
        config.lift_start_delay_ms) {
      autonomy.state = HabitatPiecesState::ReverseAfterPickup;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.post_pickup_reverse_elapsed_ms = 0U;
      update.state = autonomy.state;
      update.transitioned = true;
      return update;
    }
    update.should_wait_after_lift_start = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::ReverseAfterPickup) {
    autonomy.post_pickup_reverse_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    if (autonomy.post_pickup_reverse_elapsed_ms >=
        config.post_pickup_reverse_duration_ms) {
      autonomy.state = HabitatPiecesState::RearLineReacquire;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.rear_line_reacquire_elapsed_ms = 0U;
      autonomy.rear_line_latched = false;
      update.state = autonomy.state;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_drive_back_after_pickup = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::RearLineReacquire) {
    autonomy.rear_line_reacquire_elapsed_ms =
        now_ms - autonomy.state_entered_at_ms;
    if (mechanism_inputs.rear_line_available &&
        (mechanism_inputs.rear_left_black ||
         mechanism_inputs.rear_right_black)) {
      autonomy.rear_line_latched = true;
    }
    update.rear_line_detected = autonomy.rear_line_latched;
    if (autonomy.rear_line_latched) {
      if (autonomy.lift_complete) {
        autonomy.state = HabitatPiecesState::Complete;
        autonomy.stop_reason = HabitatPiecesStopReason::RearLineReached;
        autonomy.state_entered_at_ms = now_ms;
        update.state = autonomy.state;
        update.stop_reason = autonomy.stop_reason;
        update.target_reached = true;
        update.should_start_habitat_placement = true;
        update.transitioned = true;
        return update;
      }
      autonomy.state = HabitatPiecesState::WaitForLiftCompletion;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.should_wait_for_lift = true;
      update.transitioned = true;
      return update;
    }
    if (config.rear_line_reacquire_timeout_ms == 0U ||
        autonomy.rear_line_reacquire_elapsed_ms >=
            config.rear_line_reacquire_timeout_ms) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason = HabitatPiecesStopReason::RearLineTimeout;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.timed_out = true;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.transitioned = true;
      return update;
    }
    update.should_stop = false;
    update.should_reacquire_rear_line = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::WaitForLiftCompletion) {
    update.rear_line_detected = true;
    if (autonomy.lift_complete) {
      autonomy.state = HabitatPiecesState::Complete;
      autonomy.stop_reason = HabitatPiecesStopReason::RearLineReached;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.target_reached = true;
      update.should_start_habitat_placement = true;
      update.transitioned = true;
      return update;
    }
    update.should_wait_for_lift = true;
    return update;
  }

  if (autonomy.state == HabitatPiecesState::SideLineAligning) {
    autonomy.run_elapsed_ms = now_ms - autonomy.run_started_at_ms;
    const bool rotating_clockwise =
        autonomy.stop_reason == HabitatPiecesStopReason::Lss2Detected;
    const bool rotating_counter_clockwise =
        autonomy.stop_reason == HabitatPiecesStopReason::Lss3Detected;

    update.lss2_newly_latched =
        lss2_black && !autonomy.lss2_latched;
    update.lss3_newly_latched =
        lss3_black && !autonomy.lss3_latched;
    autonomy.lss2_latched = autonomy.lss2_latched || lss2_black;
    autonomy.lss3_latched = autonomy.lss3_latched || lss3_black;
    update.lss2_latched = autonomy.lss2_latched;
    update.lss3_latched = autonomy.lss3_latched;

    const bool opposite_sensor_found =
        (rotating_clockwise && autonomy.lss3_latched) ||
        (rotating_counter_clockwise && autonomy.lss2_latched);
    if (opposite_sensor_found) {
      autonomy.state = HabitatPiecesState::Reversing;
      autonomy.stop_reason =
          HabitatPiecesStopReason::BothSideLinesDetected;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.reverse_elapsed_ms = 0U;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      // Stop for one complete update between sensor alignment and reverse.
      update.should_stop = true;
      update.transitioned = true;
      return update;
    }

    if (!rotating_clockwise && !rotating_counter_clockwise) {
      autonomy.state = HabitatPiecesState::Fault;
      autonomy.stop_reason =
          HabitatPiecesStopReason::ConfigurationIncomplete;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
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
    update.should_align_side_lines = true;
    update.should_rotate_clockwise = rotating_clockwise;
    update.should_rotate_counter_clockwise =
        rotating_counter_clockwise;
    return update;
  }

  if (autonomy.state != HabitatPiecesState::LineFollowing) {
    return update;
  }

  autonomy.run_elapsed_ms = now_ms - autonomy.run_started_at_ms;
  autonomy.lss2_detection_armed =
      autonomy.run_elapsed_ms >= config.side_line_ignore_after_start_ms;
  update.lss2_detection_armed = autonomy.lss2_detection_armed;

  if (autonomy.lss2_detection_armed) {
    update.lss2_newly_latched =
        lss2_black && !autonomy.lss2_latched;
    autonomy.lss2_latched = autonomy.lss2_latched || lss2_black;
    update.lss2_latched = autonomy.lss2_latched;
    update.lss3_newly_latched =
        lss3_black && !autonomy.lss3_latched;
    autonomy.lss3_latched = autonomy.lss3_latched || lss3_black;
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
    // Both sensors already see the line, so alignment is complete. Stop all
    // four wheels for this update and begin reversing on the next one.
    update.should_stop = true;
    update.should_reverse = false;
    update.transitioned = true;
    return update;
  }

  if (autonomy.lss2_latched || autonomy.lss3_latched) {
    autonomy.state = HabitatPiecesState::SideLineAligning;
    autonomy.stop_reason = autonomy.lss2_latched
                               ? HabitatPiecesStopReason::Lss2Detected
                               : HabitatPiecesStopReason::Lss3Detected;
    autonomy.state_entered_at_ms = now_ms;
    update.state = autonomy.state;
    update.stop_reason = autonomy.stop_reason;
    // Stop all four wheels for this control update. Sensor-directed rotation
    // starts on the next update, so no line-follow command survives detection.
    update.should_stop = true;
    update.should_align_side_lines = false;
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
