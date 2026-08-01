#pragma once

#include "common/Units.h"

namespace robot {

enum class ImuHeadingHoldState {
  Idle,
  Active,
  Stopped,
  Fault,
};

enum class ImuHeadingHoldFaultReason {
  None,
  InvalidConfiguration,
  InvalidMeasurement,
  ImuUnavailable,
  RearLinkUnavailable,
  CommandFailed,
};

struct ImuHeadingHoldConfig {
  float maximum_strafe_duty{0.0F};
  float kp{0.0F};
  float kd{0.0F};
  float maximum_yaw_correction_duty{0.0F};

  // Converts a positive heading-controller output into the chassis mixer's
  // positive-yaw convention. It must be measured on the assembled robot.
  int yaw_command_polarity{0};
};

struct ImuHeadingHoldControllerState {
  ImuHeadingHoldState state{ImuHeadingHoldState::Idle};
  ImuHeadingHoldFaultReason fault_reason{
      ImuHeadingHoldFaultReason::None};
  float start_heading_deg{0.0F};
  float target_heading_deg{0.0F};
  int lateral_direction{0};
  Milliseconds started_at_ms{0U};
};

struct ImuHeadingHoldUpdate {
  ImuHeadingHoldState state{ImuHeadingHoldState::Idle};
  ImuHeadingHoldFaultReason fault_reason{
      ImuHeadingHoldFaultReason::None};
  float current_heading_deg{0.0F};
  float target_heading_deg{0.0F};
  float angle_error_deg{0.0F};
  float yaw_rate_dps{0.0F};
  float proportional_term{0.0F};
  float damping_term{0.0F};
  float yaw_correction_duty{0.0F};
  int lateral_direction{0};
  Milliseconds elapsed_ms{0U};
  bool should_strafe{false};
  bool faulted{false};
};

bool imuHeadingHoldConfigValid(const ImuHeadingHoldConfig& config,
                               float maximum_allowed_duty);
bool imuHeadingHoldActive(
    const ImuHeadingHoldControllerState& state);

void resetImuHeadingHoldController(
    ImuHeadingHoldControllerState& state);
bool startImuHeadingHold(
    ImuHeadingHoldControllerState& state, float current_heading_deg,
    int lateral_direction, const ImuHeadingHoldConfig& config,
    float maximum_allowed_duty, Milliseconds now_ms);
void stopImuHeadingHold(ImuHeadingHoldControllerState& state);
void deferImuHeadingHoldTimer(
    ImuHeadingHoldControllerState& state, Milliseconds duration_ms);
void faultImuHeadingHold(ImuHeadingHoldControllerState& state,
                         ImuHeadingHoldFaultReason reason);
ImuHeadingHoldUpdate updateImuHeadingHold(
    ImuHeadingHoldControllerState& state, float current_heading_deg,
    float yaw_rate_dps, const ImuHeadingHoldConfig& config,
    float maximum_allowed_duty, Milliseconds now_ms);

const char* imuHeadingHoldStateName(ImuHeadingHoldState state);
const char* imuHeadingHoldFaultReasonName(
    ImuHeadingHoldFaultReason reason);

}  // namespace robot
