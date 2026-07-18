#pragma once

#include <Arduino.h>

#include <cstdint>

namespace robot::esp2 {

enum class StepperDirection : std::int8_t { Down = -1, Up = 1 };
enum class StepperMotionState : std::uint8_t {
  Stopped,
  Waking,
  Continuous,
  Jogging,
  MovingTo,
  HomingRelease,
  HomingSeek,
  HomingFailed,
};

struct StepperAxisConfig {
  int sleep_gpio;
  int direction_gpio;
  int step_gpio;
  int lower_limit_gpio;
  std::uint32_t speed_steps_per_s{800U};
  std::uint32_t acceleration_steps_per_s2{1200U};
  std::uint32_t homing_speed_steps_per_s{200U};
  std::uint32_t step_high_us{3U};
  std::uint32_t direction_setup_us{3U};
  std::uint32_t wake_delay_us{2000U};
  std::uint32_t hold_timeout_ms{500U};
  std::uint32_t limit_debounce_ms{15U};
  std::int64_t maximum_position_steps{0};  // 0 = not configured; upward commands rejected.
  std::uint64_t homing_max_steps{0};       // 0 = not configured; Home rejected.
  std::uint32_t homing_timeout_ms{0};      // 0 = not configured; Home rejected.
};

class StepperAxis {
 public:
  explicit StepperAxis(const StepperAxisConfig& config);
  void begin();
  bool wake();
  void sleep();
  bool home();
  bool moveContinuous(StepperDirection direction);
  bool moveToSteps(std::int64_t target);
  bool jogSteps(std::int64_t delta);
  void stop();
  void update();
  bool setSpeed(std::uint32_t steps_per_s);
  bool setAcceleration(std::uint32_t steps_per_s2);
  bool setMaximumPositionSteps(std::int64_t steps);
  void refreshHoldCommand();
  void setZeroDebug();

  bool isBusy() const;
  bool isHomed() const { return homed_; }
  std::int64_t positionSteps() const;
  bool lowerLimitActive() const { return limit_active_; }
  bool sleeping() const { return sleeping_; }
  std::uint32_t speedStepsPerSecond() const;
  std::uint32_t configuredSpeedStepsPerSecond() const { return speed_steps_per_s_; }
  std::uint32_t accelerationStepsPerSecond2() const { return acceleration_steps_per_s2_; }
  std::int64_t maximumPositionSteps() const { return maximum_position_steps_; }
  StepperMotionState motionState() const { return state_; }
  const char* motionStateName() const;

 private:
  static void IRAM_ATTR timerThunk();
  void IRAM_ATTR onTimer();
  bool startMotion(StepperDirection direction, StepperMotionState state,
                   std::int64_t target, bool has_target, std::uint32_t speed);
  void updateLimit(std::uint32_t now_ms);
  void finishHoming(bool succeeded);
  void applySpeed(std::uint32_t speed);

  StepperAxisConfig config_;
  hw_timer_t* timer_{nullptr};
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile std::int64_t position_steps_{0};
  volatile std::int64_t target_steps_{0};
  volatile std::uint64_t total_step_count_{0U};
  volatile std::int8_t direction_sign_{1};
  volatile std::uint32_t step_interval_us_{1250U};
  volatile bool timer_running_{false};
  volatile bool step_high_{false};
  volatile bool has_target_{false};
  volatile bool raw_limit_active_{false};
  StepperMotionState state_{StepperMotionState::Stopped};
  std::uint32_t speed_steps_per_s_{800U};
  std::uint32_t acceleration_steps_per_s2_{1200U};
  std::uint32_t current_speed_steps_per_s_{0U};
  std::int64_t maximum_position_steps_{0};
  std::uint64_t homing_start_step_count_{0U};
  std::uint32_t homing_started_ms_{0U};
  std::uint32_t last_update_ms_{0U};
  std::uint32_t last_hold_refresh_ms_{0U};
  std::uint32_t limit_changed_ms_{0U};
  bool limit_candidate_{false};
  bool limit_active_{false};
  bool sleeping_{true};
  bool homed_{false};
  static StepperAxis* instance_;
};

}  // namespace robot::esp2
