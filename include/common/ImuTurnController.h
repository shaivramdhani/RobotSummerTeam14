#pragma once

#include "common/Units.h"

namespace robot {

enum class ImuTurnState {
  Idle,
  Turning,
  Settling,
  Complete,
  Stopped,
  Fault,
};

enum class ImuTurnFaultReason {
  None,
  InvalidConfiguration,
  InvalidMeasurement,
  ImuUnavailable,
  RearLinkUnavailable,
  CommandFailed,
  Timeout,
};

struct ImuTurnConfig {
  float maximum_rotation_duty{0.0F};
  float kp{0.0F};
  float kd{0.0F};
  float angle_tolerance_deg{0.0F};
  float maximum_finishing_yaw_rate_dps{0.0F};
  Milliseconds settling_time_ms{0U};
  Milliseconds timeout_ms{0U};

  // Converts a positive heading-controller output into the chassis mixer's
  // positive-yaw convention. It must be measured on the assembled robot.
  int yaw_command_polarity{0};
};

struct ImuTurnControllerState {
  ImuTurnState state{ImuTurnState::Idle};
  ImuTurnFaultReason fault_reason{ImuTurnFaultReason::None};
  float start_heading_deg{0.0F};
  float target_heading_deg{0.0F};
  float relative_angle_deg{0.0F};
  Milliseconds started_at_ms{0U};
  Milliseconds settling_started_at_ms{0U};
};

struct ImuTurnUpdate {
  ImuTurnState state{ImuTurnState::Idle};
  ImuTurnFaultReason fault_reason{ImuTurnFaultReason::None};
  float current_heading_deg{0.0F};
  float target_heading_deg{0.0F};
  float angle_error_deg{0.0F};
  float yaw_rate_dps{0.0F};
  float proportional_term{0.0F};
  float damping_term{0.0F};
  float rotation_command{0.0F};
  Milliseconds elapsed_ms{0U};
  Milliseconds settling_elapsed_ms{0U};
  bool should_rotate{false};
  bool completed{false};
  bool faulted{false};
};

bool imuTurnConfigValid(const ImuTurnConfig& config,
                        float maximum_allowed_duty);
bool imuTurnActive(const ImuTurnControllerState& state);

// Converts a positive physical clockwise request into the IMU heading sign.
// With the same polarity applied to the controller output, the chassis yaw
// command is positive (clockwise) for either valid sensor mounting polarity.
float clockwiseTurnRelativeAngleDeg(float clockwise_angle_deg,
                                    int yaw_command_polarity);

void resetImuTurnController(ImuTurnControllerState& state);
bool startImuTurn(ImuTurnControllerState& state, float current_heading_deg,
                  float relative_angle_deg, const ImuTurnConfig& config,
                  float maximum_allowed_duty, Milliseconds now_ms);
void stopImuTurn(ImuTurnControllerState& state);
void deferImuTurnTimers(ImuTurnControllerState& state,
                        Milliseconds duration_ms);
void faultImuTurn(ImuTurnControllerState& state,
                  ImuTurnFaultReason reason);
ImuTurnUpdate updateImuTurn(ImuTurnControllerState& state,
                            float current_heading_deg, float yaw_rate_dps,
                            const ImuTurnConfig& config,
                            float maximum_allowed_duty, Milliseconds now_ms);

const char* imuTurnStateName(ImuTurnState state);
const char* imuTurnFaultReasonName(ImuTurnFaultReason reason);

}  // namespace robot
