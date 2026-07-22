#include "common/LineFollower.h"

#include <cmath>

#include "common/ChassisMixer.h"

namespace robot {

namespace {

float clampFloat(const float value, const float minimum, const float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float nonnegative(const float value) {
  return value < 0.0F ? 0.0F : value;
}

float normalizedFilterCoefficient(const float coefficient) {
  return clampFloat(coefficient, 0.0F, 0.95F);
}

}  // namespace

void resetLineFollowerState(LineFollowerState& state) {
  state = {};
}

void startLineFollower(LineFollowerState& state, const Milliseconds now_ms) {
  resetLineFollowerState(state);
  state.enabled = true;
  state.started_at_ms = now_ms;
  state.previous_update_ms = now_ms;
}

void stopLineFollower(LineFollowerState& state) {
  resetLineFollowerState(state);
}

PidTerms calculatePidTerms(LineFollowerState& state,
                           const LineObservation& observation,
                           const LineFollowerConfig& config,
                           const Milliseconds now_ms) {
  PidTerms terms{};

  if (!observation.safe_to_drive) {
    state.integral = 0.0F;
    state.has_previous_error = false;
    state.filtered_derivative = 0.0F;
    return terms;
  }

  const float error = static_cast<float>(observation.error);
  float elapsed_seconds = 0.0F;
  if (state.previous_update_ms != 0U && now_ms > state.previous_update_ms) {
    elapsed_seconds =
        static_cast<float>(now_ms - state.previous_update_ms) / 1000.0F;
  }

  if (observation.line_visible && std::fabs(error) <= 1.0F &&
      elapsed_seconds > 0.0F) {
    state.integral += error * elapsed_seconds;
    const float integral_limit = nonnegative(config.integralLimit);
    state.integral =
        clampFloat(state.integral, -integral_limit, integral_limit);
  } else if (!observation.line_visible) {
    state.integral = 0.0F;
  }

  float derivative = 0.0F;
  if (state.has_previous_error && elapsed_seconds > 0.0F) {
    derivative = (error - state.previous_error) / elapsed_seconds;
    const float derivative_limit = nonnegative(config.derivativeLimit);
    derivative =
        clampFloat(derivative, -derivative_limit, derivative_limit);
    const float alpha =
        normalizedFilterCoefficient(config.derivativeFilterAlpha);
    derivative = (alpha * state.filtered_derivative) +
                 ((1.0F - alpha) * derivative);
  }

  state.filtered_derivative = derivative;
  state.previous_error = error;
  state.previous_update_ms = now_ms;
  state.has_previous_error = true;

  terms.proportional_term = config.kp * error;
  terms.integral_term = config.ki * state.integral;
  terms.derivative_term = config.kd * derivative;
  terms.correction = terms.proportional_term + terms.integral_term +
                     terms.derivative_term;
  const float maximum_correction = nonnegative(config.maxCorrection);
  terms.correction =
      clampFloat(terms.correction, -maximum_correction, maximum_correction);
  return terms;
}

LineFollowerUpdate updateLineFollower(LineFollowerState& state,
                                      const bool left_black,
                                      const bool right_black,
                                      const LineFollowerConfig& config,
                                      const Milliseconds now_ms) {
  LineFollowerUpdate update{};
  update.observation = observeDigitalLineSensors(
      left_black, right_black, state.last_known_side, now_ms);
  state.last_known_side = update.observation.last_known_side;

  if (!state.enabled) {
    update.wheel_command = disabledFourWheelCommand();
    return update;
  }

  if (!update.observation.safe_to_drive) {
    stopLineFollower(state);
    update.wheel_command = disabledFourWheelCommand();
    return update;
  }

  update.pid_terms =
      calculatePidTerms(state, update.observation, config, now_ms);
  update.should_drive = update.observation.safe_to_drive;
  update.wheel_command =
      update.should_drive
          ? mixDifferentialLineFollow(update.pid_terms.correction, config,
                                      now_ms)
          : disabledFourWheelCommand();
  return update;
}

LineFollowerConfig makeReverseTravelLineFollowerConfig(
    const LineFollowerConfig& configured_values) {
  LineFollowerConfig reverse = configured_values;
  reverse.baseDuty = -std::fabs(configured_values.baseDuty);
  return reverse;
}

}  // namespace robot
