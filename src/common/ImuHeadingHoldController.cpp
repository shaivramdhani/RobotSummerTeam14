#include "common/ImuHeadingHoldController.h"

#include <cmath>
#include <limits>

namespace robot {

namespace {

float clampFloat(const float value, const float minimum,
                 const float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

Milliseconds elapsed(const Milliseconds now_ms,
                     const Milliseconds then_ms) {
  const Milliseconds difference =
      static_cast<Milliseconds>(now_ms - then_ms);
  const Milliseconds maximum_unambiguous_difference =
      std::numeric_limits<Milliseconds>::max() / 2U;
  return difference <= maximum_unambiguous_difference ? difference : 0U;
}

ImuHeadingHoldUpdate makeUpdate(
    const ImuHeadingHoldControllerState& state,
    const float current_heading_deg, const float yaw_rate_dps,
    const Milliseconds now_ms) {
  ImuHeadingHoldUpdate update{};
  update.state = state.state;
  update.fault_reason = state.fault_reason;
  update.current_heading_deg = current_heading_deg;
  update.target_heading_deg = state.target_heading_deg;
  update.angle_error_deg =
      state.target_heading_deg - current_heading_deg;
  update.yaw_rate_dps = yaw_rate_dps;
  update.lateral_direction = state.lateral_direction;
  if (imuHeadingHoldActive(state)) {
    update.elapsed_ms = elapsed(now_ms, state.started_at_ms);
  }
  update.should_strafe = imuHeadingHoldActive(state);
  update.faulted = state.state == ImuHeadingHoldState::Fault;
  return update;
}

}  // namespace

bool imuHeadingHoldConfigValid(
    const ImuHeadingHoldConfig& config,
    const float maximum_allowed_duty) {
  return std::isfinite(maximum_allowed_duty) &&
         maximum_allowed_duty > 0.0F &&
         std::isfinite(config.maximum_strafe_duty) &&
         config.maximum_strafe_duty > 0.0F &&
         std::isfinite(config.kp) && config.kp > 0.0F &&
         std::isfinite(config.kd) && config.kd >= 0.0F &&
         std::isfinite(config.maximum_yaw_correction_duty) &&
         config.maximum_yaw_correction_duty > 0.0F &&
         config.maximum_strafe_duty +
                 config.maximum_yaw_correction_duty <=
             maximum_allowed_duty &&
         (config.yaw_command_polarity == -1 ||
          config.yaw_command_polarity == 1);
}

bool imuHeadingHoldActive(
    const ImuHeadingHoldControllerState& state) {
  return state.state == ImuHeadingHoldState::Active;
}

void resetImuHeadingHoldController(
    ImuHeadingHoldControllerState& state) {
  state = {};
}

bool startImuHeadingHold(
    ImuHeadingHoldControllerState& state,
    const float current_heading_deg, const int lateral_direction,
    const ImuHeadingHoldConfig& config,
    const float maximum_allowed_duty,
    const Milliseconds now_ms) {
  if (!imuHeadingHoldConfigValid(config, maximum_allowed_duty) ||
      !std::isfinite(current_heading_deg) ||
      (lateral_direction != -1 && lateral_direction != 1)) {
    state = {};
    state.state = ImuHeadingHoldState::Fault;
    state.fault_reason =
        ImuHeadingHoldFaultReason::InvalidConfiguration;
    return false;
  }

  state = {};
  state.state = ImuHeadingHoldState::Active;
  state.start_heading_deg = current_heading_deg;
  state.target_heading_deg = current_heading_deg;
  state.lateral_direction = lateral_direction;
  state.started_at_ms = now_ms;
  return true;
}

void stopImuHeadingHold(ImuHeadingHoldControllerState& state) {
  state.state = ImuHeadingHoldState::Stopped;
  state.fault_reason = ImuHeadingHoldFaultReason::None;
}

void deferImuHeadingHoldTimer(
    ImuHeadingHoldControllerState& state,
    const Milliseconds duration_ms) {
  if (!imuHeadingHoldActive(state) || duration_ms == 0U) {
    return;
  }
  state.started_at_ms += duration_ms;
}

void faultImuHeadingHold(
    ImuHeadingHoldControllerState& state,
    const ImuHeadingHoldFaultReason reason) {
  state.state = ImuHeadingHoldState::Fault;
  state.fault_reason =
      reason == ImuHeadingHoldFaultReason::None
          ? ImuHeadingHoldFaultReason::InvalidMeasurement
          : reason;
}

ImuHeadingHoldUpdate updateImuHeadingHold(
    ImuHeadingHoldControllerState& state,
    const float current_heading_deg, const float yaw_rate_dps,
    const ImuHeadingHoldConfig& config,
    const float maximum_allowed_duty,
    const Milliseconds now_ms) {
  ImuHeadingHoldUpdate update =
      makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
  if (!imuHeadingHoldActive(state)) {
    return update;
  }

  if (!imuHeadingHoldConfigValid(config, maximum_allowed_duty)) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuHeadingHold(
        state, ImuHeadingHoldFaultReason::InvalidConfiguration);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }
  if (!std::isfinite(current_heading_deg) ||
      !std::isfinite(yaw_rate_dps)) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuHeadingHold(
        state, ImuHeadingHoldFaultReason::InvalidMeasurement);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }

  update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
  if (!std::isfinite(update.angle_error_deg)) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuHeadingHold(
        state, ImuHeadingHoldFaultReason::InvalidMeasurement);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }

  update.proportional_term = config.kp * update.angle_error_deg;
  update.damping_term = -config.kd * yaw_rate_dps;
  const float unconstrained_correction =
      update.proportional_term + update.damping_term;
  if (!std::isfinite(update.proportional_term) ||
      !std::isfinite(update.damping_term) ||
      !std::isfinite(unconstrained_correction)) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuHeadingHold(
        state, ImuHeadingHoldFaultReason::InvalidConfiguration);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }

  update.yaw_correction_duty =
      clampFloat(unconstrained_correction,
                 -config.maximum_yaw_correction_duty,
                 config.maximum_yaw_correction_duty);
  return update;
}

const char* imuHeadingHoldStateName(
    const ImuHeadingHoldState state) {
  switch (state) {
    case ImuHeadingHoldState::Idle:
      return "IDLE";
    case ImuHeadingHoldState::Active:
      return "ACTIVE";
    case ImuHeadingHoldState::Stopped:
      return "STOPPED";
    case ImuHeadingHoldState::Fault:
      return "FAULT";
  }
  return "FAULT";
}

const char* imuHeadingHoldFaultReasonName(
    const ImuHeadingHoldFaultReason reason) {
  switch (reason) {
    case ImuHeadingHoldFaultReason::None:
      return "NONE";
    case ImuHeadingHoldFaultReason::InvalidConfiguration:
      return "INVALID_CONFIGURATION";
    case ImuHeadingHoldFaultReason::InvalidMeasurement:
      return "INVALID_MEASUREMENT";
    case ImuHeadingHoldFaultReason::ImuUnavailable:
      return "IMU_UNAVAILABLE";
    case ImuHeadingHoldFaultReason::RearLinkUnavailable:
      return "REAR_LINK_UNAVAILABLE";
    case ImuHeadingHoldFaultReason::CommandFailed:
      return "COMMAND_FAILED";
  }
  return "INVALID_MEASUREMENT";
}

}  // namespace robot
