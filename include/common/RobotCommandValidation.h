#pragma once

#include <cstdint>

#include "common/LineFollower.h"
#include "common/RobotTestMode.h"
#include "common/Units.h"

namespace robot {

enum class WheelId : std::uint8_t {
  FrontLeft = 0,
  FrontRight = 1,
  BackLeft = 2,
  BackRight = 3,
};

struct CommandValidationLimits {
  float maximum_duty{0.0F};
  float maximum_single_motor_duty{0.0F};
  Milliseconds maximum_duration_ms{0};
};

struct CommandValidationResult {
  bool accepted{false};
  const char* reason{"invalid command"};
};

const char* wheelIdName(WheelId wheel);
bool parseWheelId(const char* text, WheelId& wheel);

CommandValidationResult validateModeAllowsDrive(RobotTestMode mode);
CommandValidationResult validateModeAllowsSingleMotor(RobotTestMode mode);
CommandValidationResult validateModeAllowsMechanism(RobotTestMode mode);
CommandValidationResult validateNormalizedDuty(float duty, float maximum_abs);
CommandValidationResult validateTimedDuration(Milliseconds duration_ms,
                                              Milliseconds maximum_ms);
CommandValidationResult validateSingleMotorCommand(
    RobotTestMode mode, float duty, Milliseconds duration_ms,
    const CommandValidationLimits& limits);
CommandValidationResult validateDriveCommand(
    RobotTestMode mode, float vx, float vy, float wz, float duty,
    const CommandValidationLimits& limits);
CommandValidationResult validateLineFollowerConfig(
    const LineFollowerConfig& config, float hardware_duty_cap);

}  // namespace robot
