#include "common/ChassisMixer.h"

#include <cmath>

namespace robot {

namespace {

float clampFloat(const float value, const float minimum, const float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

std::int16_t dutyToMilli(const float duty) {
  const float clamped = clampFloat(duty, -1.0F, 1.0F);
  return clampCommandMilli(static_cast<std::int16_t>(clamped * 1000.0F));
}

MotorCommand makeMotorCommand(const float duty, const Milliseconds now_ms,
                              const Milliseconds timeout_ms) {
  MotorCommand command{};
  command.enabled = std::fabs(duty) > 0.0001F;
  command.duty_command_milli = dutyToMilli(duty);
  command.expires_at_ms = now_ms + timeout_ms;
  return command;
}

}  // namespace

FourWheelCommand calculateDisabledFourWheelCommand(
    const ChassisCommand& command) {
  (void)command;
  return disabledFourWheelCommand();
}

FourWheelCommand mixOpenLoopMecanum(const float lateral_command,
                                    const float forward_command,
                                    const float yaw_command,
                                    const float duty,
                                    const Milliseconds now_ms,
                                    const Milliseconds timeout_ms) {
  const float lateral = clampFloat(lateral_command, -1.0F, 1.0F);
  const float forward = clampFloat(forward_command, -1.0F, 1.0F);
  const float yaw = clampFloat(yaw_command, -1.0F, 1.0F);
  const float duty_scale = clampFloat(duty, 0.0F, 1.0F);

  float front_left = forward + lateral + yaw;
  float front_right = forward - lateral - yaw;
  float back_left = forward - lateral + yaw;
  float back_right = forward + lateral - yaw;
  const float peak =
      std::fmax(std::fmax(std::fabs(front_left), std::fabs(front_right)),
                std::fmax(std::fabs(back_left), std::fabs(back_right)));
  if (peak > 1.0F) {
    front_left /= peak;
    front_right /= peak;
    back_left /= peak;
    back_right /= peak;
  }

  FourWheelCommand command{};
  command.front_left =
      makeMotorCommand(front_left * duty_scale, now_ms, timeout_ms);
  command.front_right =
      makeMotorCommand(front_right * duty_scale, now_ms, timeout_ms);
  command.back_left =
      makeMotorCommand(back_left * duty_scale, now_ms, timeout_ms);
  command.back_right =
      makeMotorCommand(back_right * duty_scale, now_ms, timeout_ms);
  return command;
}

FourWheelCommand mixDifferentialLineFollow(const float correction,
                                           const LineFollowerConfig& config,
                                           const Milliseconds now_ms) {
  const float maximum_duty = clampFloat(config.maxDuty, 0.0F, 1.0F);
  if (maximum_duty <= 0.0F) {
    return disabledFourWheelCommand();
  }

  const float base_duty =
      clampFloat(config.baseDuty, -maximum_duty, maximum_duty);
  const float maximum_correction =
      clampFloat(config.maxCorrection, 0.0F, maximum_duty);
  const float limited_correction =
      clampFloat(correction, -maximum_correction, maximum_correction);
  const float polarity =
      config.steeringPolarity < 0 ? -1.0F : 1.0F;
  const float signed_correction = polarity * limited_correction;

  float left_duty = base_duty - signed_correction;
  float right_duty = base_duty + signed_correction;
  const float peak = std::fmax(std::fabs(left_duty), std::fabs(right_duty));
  if (peak > maximum_duty && peak > 0.0F) {
    const float scale = maximum_duty / peak;
    left_duty *= scale;
    right_duty *= scale;
  }

  left_duty = clampFloat(left_duty, -maximum_duty, maximum_duty);
  right_duty = clampFloat(right_duty, -maximum_duty, maximum_duty);

  FourWheelCommand command{};
  command.front_left =
      makeMotorCommand(left_duty, now_ms, config.remoteCommandTimeoutMs);
  command.back_left =
      makeMotorCommand(left_duty, now_ms, config.remoteCommandTimeoutMs);
  command.front_right =
      makeMotorCommand(right_duty, now_ms, config.remoteCommandTimeoutMs);
  command.back_right =
      makeMotorCommand(right_duty, now_ms, config.remoteCommandTimeoutMs);
  return command;
}

}  // namespace robot
