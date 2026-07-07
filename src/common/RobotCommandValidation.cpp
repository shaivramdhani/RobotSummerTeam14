#include "common/RobotCommandValidation.h"

#include <cmath>
#include <cstring>

namespace robot {

namespace {

CommandValidationResult accepted() {
  return {true, "ok"};
}

CommandValidationResult rejected(const char* reason) {
  return {false, reason};
}

bool sameToken(const char* lhs, const char* rhs) {
  if (lhs == nullptr || rhs == nullptr) {
    return false;
  }
  while (*lhs != '\0' && *rhs != '\0') {
    char left = *lhs;
    char right = *rhs;
    if (left >= 'A' && left <= 'Z') {
      left = static_cast<char>(left - 'A' + 'a');
    }
    if (right >= 'A' && right <= 'Z') {
      right = static_cast<char>(right - 'A' + 'a');
    }
    if (left != right) {
      return false;
    }
    ++lhs;
    ++rhs;
  }
  return *lhs == '\0' && *rhs == '\0';
}

bool finiteInRange(const float value, const float minimum,
                   const float maximum) {
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

}  // namespace

const char* wheelIdName(const WheelId wheel) {
  switch (wheel) {
    case WheelId::FrontLeft:
      return "FL";
    case WheelId::FrontRight:
      return "FR";
    case WheelId::BackLeft:
      return "BL";
    case WheelId::BackRight:
      return "BR";
  }
  return "FL";
}

bool parseWheelId(const char* const text, WheelId& wheel) {
  if (sameToken(text, "fl") || sameToken(text, "front-left") ||
      sameToken(text, "0")) {
    wheel = WheelId::FrontLeft;
    return true;
  }
  if (sameToken(text, "fr") || sameToken(text, "front-right") ||
      sameToken(text, "1")) {
    wheel = WheelId::FrontRight;
    return true;
  }
  if (sameToken(text, "bl") || sameToken(text, "rl") ||
      sameToken(text, "back-left") || sameToken(text, "rear-left") ||
      sameToken(text, "2")) {
    wheel = WheelId::BackLeft;
    return true;
  }
  if (sameToken(text, "br") || sameToken(text, "rr") ||
      sameToken(text, "back-right") || sameToken(text, "rear-right") ||
      sameToken(text, "3")) {
    wheel = WheelId::BackRight;
    return true;
  }
  return false;
}

CommandValidationResult validateModeAllowsDrive(const RobotTestMode mode) {
  if (mode == RobotTestMode::ManualDriveTest ||
      mode == RobotTestMode::DistributedDriveTest) {
    return accepted();
  }
  return rejected("drive commands require MANUAL_DRIVE_TEST or DISTRIBUTED_DRIVE_TEST");
}

CommandValidationResult validateModeAllowsSingleMotor(
    const RobotTestMode mode) {
  if (mode == RobotTestMode::SingleMotorTest) {
    return accepted();
  }
  return rejected("motor commands require SINGLE_MOTOR_TEST");
}

CommandValidationResult validateNormalizedDuty(const float duty,
                                               const float maximum_abs) {
  if (!std::isfinite(duty)) {
    return rejected("duty must be finite");
  }
  if (maximum_abs < 0.0F || maximum_abs > 1.0F) {
    return rejected("configured duty limit is invalid");
  }
  if (std::fabs(duty) > maximum_abs) {
    return rejected("duty exceeds configured limit");
  }
  return accepted();
}

CommandValidationResult validateTimedDuration(
    const Milliseconds duration_ms, const Milliseconds maximum_ms) {
  if (duration_ms == 0U) {
    return rejected("duration must be greater than zero");
  }
  if (maximum_ms == 0U || duration_ms > maximum_ms) {
    return rejected("duration exceeds safe cap");
  }
  return accepted();
}

CommandValidationResult validateSingleMotorCommand(
    const RobotTestMode mode, const float duty,
    const Milliseconds duration_ms, const CommandValidationLimits& limits) {
  CommandValidationResult result = validateModeAllowsSingleMotor(mode);
  if (!result.accepted) {
    return result;
  }

  result =
      validateNormalizedDuty(duty, limits.maximum_single_motor_duty);
  if (!result.accepted) {
    return result;
  }

  return validateTimedDuration(duration_ms, limits.maximum_duration_ms);
}

CommandValidationResult validateDriveCommand(
    const RobotTestMode mode, const float vx, const float vy, const float wz,
    const float duty, const CommandValidationLimits& limits) {
  CommandValidationResult result = validateModeAllowsDrive(mode);
  if (!result.accepted) {
    return result;
  }
  if (!finiteInRange(vx, -1.0F, 1.0F) ||
      !finiteInRange(vy, -1.0F, 1.0F) ||
      !finiteInRange(wz, -1.0F, 1.0F)) {
    return rejected("vx, vy, and wz must be in [-1, 1]");
  }
  return validateNormalizedDuty(duty, limits.maximum_duty);
}

CommandValidationResult validateLineFollowerConfig(
    const LineFollowerConfig& config, const float hardware_duty_cap) {
  if (!finiteInRange(config.kp, 0.0F, 20.0F) ||
      !finiteInRange(config.ki, 0.0F, 20.0F) ||
      !finiteInRange(config.kd, 0.0F, 20.0F)) {
    return rejected("PID gains must be in [0, 20]");
  }
  if (!finiteInRange(config.maximumDuty, 0.0F, hardware_duty_cap)) {
    return rejected("maximumDuty exceeds hardware cap");
  }
  if (!finiteInRange(config.baseDuty, -config.maximumDuty,
                     config.maximumDuty)) {
    return rejected("baseDuty exceeds maximumDuty");
  }
  if (!finiteInRange(config.maximumCorrection, 0.0F,
                     config.maximumDuty)) {
    return rejected("maximumCorrection must be in [0, maximumDuty]");
  }
  if (config.steeringPolarity != 1 && config.steeringPolarity != -1) {
    return rejected("steeringPolarity must be 1 or -1");
  }
  if (config.controlPeriodMs < 5U || config.controlPeriodMs > 100U) {
    return rejected("controlPeriodMs must be in [5, 100]");
  }
  return accepted();
}

}  // namespace robot
