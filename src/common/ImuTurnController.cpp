#include "common/ImuTurnController.h"

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
  // Unsigned subtraction correctly handles millis() rollover, but a web
  // handler can start a turn a few milliseconds after the motion task
  // captured that cycle's now_ms. In that case the large half-range result
  // means "not reached yet", not an elapsed interval.
  const Milliseconds maximum_unambiguous_difference =
      std::numeric_limits<Milliseconds>::max() / 2U;
  return difference <= maximum_unambiguous_difference ? difference : 0U;
}

ImuTurnUpdate makeUpdate(const ImuTurnControllerState& state,
                         const float current_heading_deg,
                         const float yaw_rate_dps,
                         const Milliseconds now_ms) {
  ImuTurnUpdate update{};
  update.state = state.state;
  update.fault_reason = state.fault_reason;
  update.current_heading_deg = current_heading_deg;
  update.target_heading_deg = state.target_heading_deg;
  update.angle_error_deg = state.target_heading_deg - current_heading_deg;
  update.yaw_rate_dps = yaw_rate_dps;
  if (imuTurnActive(state)) {
    update.elapsed_ms = elapsed(now_ms, state.started_at_ms);
  }
  if (state.state == ImuTurnState::Settling) {
    update.settling_elapsed_ms =
        elapsed(now_ms, state.settling_started_at_ms);
  }
  update.completed = state.state == ImuTurnState::Complete;
  update.faulted = state.state == ImuTurnState::Fault;
  return update;
}

}  // namespace

bool imuTurnConfigValid(const ImuTurnConfig& config,
                        const float maximum_allowed_duty) {
  return std::isfinite(maximum_allowed_duty) &&
         maximum_allowed_duty > 0.0F &&
         std::isfinite(config.maximum_rotation_duty) &&
         config.maximum_rotation_duty > 0.0F &&
         config.maximum_rotation_duty <= maximum_allowed_duty &&
         std::isfinite(config.kp) && config.kp > 0.0F &&
         std::isfinite(config.kd) && config.kd >= 0.0F &&
         std::isfinite(config.angle_tolerance_deg) &&
         config.angle_tolerance_deg > 0.0F &&
         std::isfinite(config.maximum_finishing_yaw_rate_dps) &&
         config.maximum_finishing_yaw_rate_dps > 0.0F &&
         config.settling_time_ms > 0U &&
         config.timeout_ms > config.settling_time_ms &&
         (config.yaw_command_polarity == -1 ||
          config.yaw_command_polarity == 1);
}

bool imuTurnActive(const ImuTurnControllerState& state) {
  return state.state == ImuTurnState::Turning ||
         state.state == ImuTurnState::Settling;
}

float clockwiseTurnRelativeAngleDeg(
    const float clockwise_angle_deg,
    const int yaw_command_polarity) {
  return clockwise_angle_deg *
         static_cast<float>(yaw_command_polarity);
}

void resetImuTurnController(ImuTurnControllerState& state) {
  state = {};
}

bool startImuTurn(ImuTurnControllerState& state,
                  const float current_heading_deg,
                  const float relative_angle_deg,
                  const ImuTurnConfig& config,
                  const float maximum_allowed_duty,
                  const Milliseconds now_ms) {
  if (!imuTurnConfigValid(config, maximum_allowed_duty) ||
      !std::isfinite(current_heading_deg) ||
      !std::isfinite(relative_angle_deg) ||
      std::fabs(relative_angle_deg) <= 0.0001F) {
    state = {};
    state.state = ImuTurnState::Fault;
    state.fault_reason = ImuTurnFaultReason::InvalidConfiguration;
    return false;
  }

  state = {};
  const float target_heading_deg =
      current_heading_deg + relative_angle_deg;
  if (!std::isfinite(target_heading_deg)) {
    state.state = ImuTurnState::Fault;
    state.fault_reason = ImuTurnFaultReason::InvalidMeasurement;
    return false;
  }
  state.state = ImuTurnState::Turning;
  state.start_heading_deg = current_heading_deg;
  state.target_heading_deg = target_heading_deg;
  state.relative_angle_deg = relative_angle_deg;
  state.started_at_ms = now_ms;
  return true;
}

void stopImuTurn(ImuTurnControllerState& state) {
  state.state = ImuTurnState::Stopped;
  state.fault_reason = ImuTurnFaultReason::None;
  state.settling_started_at_ms = 0U;
}

void deferImuTurnTimers(ImuTurnControllerState& state,
                        const Milliseconds duration_ms) {
  if (!imuTurnActive(state) || duration_ms == 0U) {
    return;
  }
  state.started_at_ms += duration_ms;
  if (state.state == ImuTurnState::Settling) {
    state.settling_started_at_ms += duration_ms;
  }
}

void faultImuTurn(ImuTurnControllerState& state,
                  const ImuTurnFaultReason reason) {
  state.state = ImuTurnState::Fault;
  state.fault_reason =
      reason == ImuTurnFaultReason::None
          ? ImuTurnFaultReason::InvalidMeasurement
          : reason;
  state.settling_started_at_ms = 0U;
}

ImuTurnUpdate updateImuTurn(ImuTurnControllerState& state,
                            const float current_heading_deg,
                            const float yaw_rate_dps,
                            const ImuTurnConfig& config,
                            const float maximum_allowed_duty,
                            const Milliseconds now_ms) {
  ImuTurnUpdate update =
      makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
  if (!imuTurnActive(state)) {
    return update;
  }

  if (!imuTurnConfigValid(config, maximum_allowed_duty)) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuTurn(state, ImuTurnFaultReason::InvalidConfiguration);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }
  if (!std::isfinite(current_heading_deg) ||
      !std::isfinite(yaw_rate_dps)) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuTurn(state, ImuTurnFaultReason::InvalidMeasurement);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }

  update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
  if (!std::isfinite(update.angle_error_deg)) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuTurn(state, ImuTurnFaultReason::InvalidMeasurement);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }
  if (update.elapsed_ms >= config.timeout_ms) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuTurn(state, ImuTurnFaultReason::Timeout);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }

  const bool angle_settled =
      std::fabs(update.angle_error_deg) <= config.angle_tolerance_deg;
  const bool rate_settled =
      std::fabs(yaw_rate_dps) <=
      config.maximum_finishing_yaw_rate_dps;
  if (angle_settled && rate_settled) {
    if (state.state != ImuTurnState::Settling) {
      state.state = ImuTurnState::Settling;
      state.settling_started_at_ms = now_ms;
    }
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    if (update.settling_elapsed_ms >= config.settling_time_ms) {
      const Milliseconds elapsed_ms = update.elapsed_ms;
      const Milliseconds settling_elapsed_ms =
          update.settling_elapsed_ms;
      state.state = ImuTurnState::Complete;
      update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
      update.elapsed_ms = elapsed_ms;
      update.settling_elapsed_ms = settling_elapsed_ms;
    }
    return update;
  }

  state.state = ImuTurnState::Turning;
  state.settling_started_at_ms = 0U;
  update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
  update.proportional_term = config.kp * update.angle_error_deg;
  update.damping_term = -config.kd * yaw_rate_dps;
  const float unconstrained_command =
      update.proportional_term + update.damping_term;
  if (!std::isfinite(update.proportional_term) ||
      !std::isfinite(update.damping_term) ||
      !std::isfinite(unconstrained_command)) {
    const Milliseconds elapsed_ms = update.elapsed_ms;
    faultImuTurn(state, ImuTurnFaultReason::InvalidConfiguration);
    update = makeUpdate(state, current_heading_deg, yaw_rate_dps, now_ms);
    update.elapsed_ms = elapsed_ms;
    return update;
  }
  update.rotation_command =
      clampFloat(unconstrained_command,
                 -config.maximum_rotation_duty,
                 config.maximum_rotation_duty);
  update.should_rotate = std::fabs(update.rotation_command) > 0.0001F;
  return update;
}

const char* imuTurnStateName(const ImuTurnState state) {
  switch (state) {
    case ImuTurnState::Idle:
      return "IDLE";
    case ImuTurnState::Turning:
      return "TURNING";
    case ImuTurnState::Settling:
      return "SETTLING";
    case ImuTurnState::Complete:
      return "COMPLETE";
    case ImuTurnState::Stopped:
      return "STOPPED";
    case ImuTurnState::Fault:
      return "FAULT";
  }
  return "FAULT";
}

const char* imuTurnFaultReasonName(const ImuTurnFaultReason reason) {
  switch (reason) {
    case ImuTurnFaultReason::None:
      return "NONE";
    case ImuTurnFaultReason::InvalidConfiguration:
      return "INVALID_CONFIGURATION";
    case ImuTurnFaultReason::InvalidMeasurement:
      return "INVALID_MEASUREMENT";
    case ImuTurnFaultReason::ImuUnavailable:
      return "IMU_UNAVAILABLE";
    case ImuTurnFaultReason::RearLinkUnavailable:
      return "REAR_LINK_UNAVAILABLE";
    case ImuTurnFaultReason::CommandFailed:
      return "COMMAND_FAILED";
    case ImuTurnFaultReason::Timeout:
      return "TIMEOUT";
  }
  return "INVALID_MEASUREMENT";
}

}  // namespace robot
