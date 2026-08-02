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
         maximum_distance_strafe_timeout_ms > 0U &&
         config.distance_strafe_timeout_ms > 0U &&
         config.distance_strafe_timeout_ms <=
             maximum_distance_strafe_timeout_ms;
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
    const bool lss2_black, const bool lss3_black,
    const Milliseconds now_ms,
    const HabitatPiecesDistanceSample& distance_sample) {
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
        now_ms - autonomy.state_entered_at_ms;
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
        update.distance_sample_new = true;
        const bool in_zone =
            distance_sample.distance_mm > config.distance_threshold_mm;
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
      autonomy.state = HabitatPiecesState::Complete;
      autonomy.stop_reason =
          HabitatPiecesStopReason::DistanceZoneCountReached;
      autonomy.state_entered_at_ms = now_ms;
      update.state = autonomy.state;
      update.stop_reason = autonomy.stop_reason;
      update.target_reached = true;
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
        lss2_black && !autonomy.lss2_latched;
    update.lss3_newly_latched =
        lss3_black && !autonomy.lss3_latched;
    autonomy.lss2_latched = autonomy.lss2_latched || lss2_black;
    autonomy.lss3_latched = autonomy.lss3_latched || lss3_black;
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

}  // namespace robot
