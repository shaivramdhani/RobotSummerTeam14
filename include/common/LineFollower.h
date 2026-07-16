#pragma once

#include <cstdint>

#include "common/MotorOutput.h"
#include "common/LineObservation.h"

namespace robot {

struct LineFollowerConfig {
  float kp{0.20F};
  float ki{0.0F};
  float kd{0.0F};
  float baseDuty{0.0F};
  float maxDuty{0.0F};
  float maxCorrection{0.25F};
  float integralLimit{2.0F};
  float derivativeLimit{20.0F};
  float derivativeFilterAlpha{0.0F};
  int steeringPolarity{1};
  Milliseconds controlPeriodMs{10U};
  Milliseconds remoteCommandTimeoutMs{kDefaultCommunicationTimeoutMs};
  bool telemetryEnabled{true};
};

struct PidTerms {
  float proportional_term{0.0F};
  float integral_term{0.0F};
  float derivative_term{0.0F};
  float correction{0.0F};
};

struct LineFollowerState {
  bool enabled{false};
  Milliseconds started_at_ms{0};
  Milliseconds previous_update_ms{0};
  bool has_previous_error{false};
  float previous_error{0.0F};
  float integral{0.0F};
  float filtered_derivative{0.0F};
  std::int8_t last_known_side{0};
};

struct LineFollowerUpdate {
  LineObservation observation{};
  PidTerms pid_terms{};
  FourWheelCommand wheel_command{};
  bool should_drive{false};
};

void resetLineFollowerState(LineFollowerState& state);
void startLineFollower(LineFollowerState& state, Milliseconds now_ms);
void stopLineFollower(LineFollowerState& state);

PidTerms calculatePidTerms(LineFollowerState& state,
                           const LineObservation& observation,
                           const LineFollowerConfig& config,
                           Milliseconds now_ms);
LineFollowerUpdate updateLineFollower(LineFollowerState& state,
                                      bool left_black, bool right_black,
                                      const LineFollowerConfig& config,
                                      Milliseconds now_ms);

}  // namespace robot
