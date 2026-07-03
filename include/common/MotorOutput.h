#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

struct MotorCommand {
  bool enabled{false};
  std::int16_t duty_command_milli{0};
  Milliseconds expires_at_ms{0};
};

struct FourWheelCommand {
  MotorCommand front_left{};
  MotorCommand front_right{};
  MotorCommand back_left{};
  MotorCommand back_right{};
};

constexpr MotorCommand disabledMotorCommand() {
  return {};
}

constexpr FourWheelCommand disabledFourWheelCommand() {
  return {};
}

class IMotorOutput {
 public:
  virtual ~IMotorOutput() = default;
  virtual void initializeDisabled() = 0;
  virtual void apply(const MotorCommand& command) = 0;
  virtual void disable() = 0;
};

class NullMotorOutput final : public IMotorOutput {
 public:
  void initializeDisabled() override {}
  void apply(const MotorCommand& command) override { last_command_ = command; }
  void disable() override { last_command_ = disabledMotorCommand(); }

  const MotorCommand& lastCommand() const { return last_command_; }

 private:
  MotorCommand last_command_{};
};

}  // namespace robot
