#include "esp2/StepperAxis.h"

#include <algorithm>
#include <cstdlib>

namespace robot::esp2 {

StepperAxis* StepperAxis::instance_ = nullptr;

StepperAxis::StepperAxis(const StepperAxisConfig& config)
    : config_(config), speed_steps_per_s_(config.speed_steps_per_s),
      acceleration_steps_per_s2_(config.acceleration_steps_per_s2),
      maximum_position_steps_(config.maximum_position_steps) {}

void StepperAxis::begin() {
  pinMode(config_.sleep_gpio, OUTPUT);
  pinMode(config_.direction_gpio, OUTPUT);
  pinMode(config_.step_gpio, OUTPUT);
  pinMode(config_.lower_limit_gpio, INPUT);  // External circuit defines LOW/HIGH.
  digitalWrite(config_.sleep_gpio, LOW);
  digitalWrite(config_.direction_gpio, LOW);
  digitalWrite(config_.step_gpio, LOW);
  sleeping_ = true;
  instance_ = this;
  timer_ = timerBegin(0, 80, true);  // 80 MHz / 80 = one timer tick per us.
  timerAttachInterrupt(timer_, &StepperAxis::timerThunk, true);
}

bool StepperAxis::wake() {
  if (!sleeping_) return true;
  digitalWrite(config_.sleep_gpio, HIGH);
  sleeping_ = false;
  state_ = StepperMotionState::Waking;
  last_update_ms_ = millis();
  return true;
}

void StepperAxis::sleep() {
  stop();
  digitalWrite(config_.sleep_gpio, LOW);
  sleeping_ = true;
}

bool StepperAxis::startMotion(StepperDirection direction,
                              StepperMotionState state, std::int64_t target,
                              bool has_target, std::uint32_t speed) {
  if (speed == 0U || (direction == StepperDirection::Down &&
                     (limit_active_ || digitalRead(config_.lower_limit_gpio) == HIGH))) return false;
  if (direction == StepperDirection::Up &&
      (maximum_position_steps_ <= 0 || positionSteps() >= maximum_position_steps_)) return false;
  const bool was_sleeping = sleeping_;
  wake();
  const std::int8_t sign = static_cast<std::int8_t>(direction);
  portENTER_CRITICAL(&mux_);
  direction_sign_ = sign;
  target_steps_ = target;
  has_target_ = has_target;
  current_speed_steps_per_s_ = std::min<std::uint32_t>(speed, 1U);
  step_interval_us_ = 1000000U / current_speed_steps_per_s_;
  step_high_ = false;
  timer_running_ = true;
  portEXIT_CRITICAL(&mux_);
  digitalWrite(config_.direction_gpio, sign > 0 ? HIGH : LOW);
  digitalWrite(config_.step_gpio, LOW);
  state_ = state;
  timerAlarmWrite(timer_, was_sleeping ? config_.wake_delay_us
                                      : config_.direction_setup_us, false);
  timerAlarmEnable(timer_);
  return true;
}

bool StepperAxis::moveContinuous(StepperDirection direction) {
  last_hold_refresh_ms_ = millis();
  return startMotion(direction, StepperMotionState::Continuous, 0, false,
                     speed_steps_per_s_);
}

void StepperAxis::refreshHoldCommand() { last_hold_refresh_ms_ = millis(); }

bool StepperAxis::moveToSteps(std::int64_t target) {
  if (!homed_ || target < 0 || maximum_position_steps_ <= 0 ||
      target > maximum_position_steps_) return false;
  const std::int64_t current = positionSteps();
  if (target == current) { stop(); return true; }
  return startMotion(target > current ? StepperDirection::Up : StepperDirection::Down,
                     StepperMotionState::MovingTo, target, true, speed_steps_per_s_);
}

bool StepperAxis::jogSteps(std::int64_t delta) {
  if (delta == 0) return true;
  const std::int64_t current = positionSteps();
  std::int64_t target = current + delta;
  if (maximum_position_steps_ <= 0) return false;
  target = std::max<std::int64_t>(0, std::min(target, maximum_position_steps_));
  if (target == current) return false;
  return startMotion(target > current ? StepperDirection::Up : StepperDirection::Down,
                     StepperMotionState::Jogging, target, true, speed_steps_per_s_);
}

bool StepperAxis::home() {
  if (maximum_position_steps_ <= 0) return false;
  homed_ = false;
  homing_started_ms_ = millis();
  portENTER_CRITICAL(&mux_);
  homing_start_step_count_ = total_step_count_;
  portEXIT_CRITICAL(&mux_);
  if (limit_active_ || digitalRead(config_.lower_limit_gpio) == HIGH)
    return startMotion(StepperDirection::Up, StepperMotionState::HomingRelease,
                       0, false, config_.homing_speed_steps_per_s);
  return startMotion(StepperDirection::Down, StepperMotionState::HomingSeek,
                     0, false, config_.homing_speed_steps_per_s);
}

void StepperAxis::stop() {
  if (timer_) timerAlarmDisable(timer_);
  portENTER_CRITICAL(&mux_);
  timer_running_ = false;
  step_high_ = false;
  has_target_ = false;
  current_speed_steps_per_s_ = 0U;
  portEXIT_CRITICAL(&mux_);
  digitalWrite(config_.step_gpio, LOW);
  if (state_ != StepperMotionState::HomingFailed) state_ = StepperMotionState::Stopped;
}

void StepperAxis::updateLimit(std::uint32_t now_ms) {
  const bool raw = digitalRead(config_.lower_limit_gpio) == HIGH;
  raw_limit_active_ = raw;  // ISR interlock is intentionally not debounced.
  if (raw != limit_candidate_) { limit_candidate_ = raw; limit_changed_ms_ = now_ms; }
  if (raw == limit_candidate_ && now_ms - limit_changed_ms_ >= config_.limit_debounce_ms)
    limit_active_ = limit_candidate_;
}

void StepperAxis::finishHoming(bool succeeded) {
  stop();
  homed_ = succeeded;
  if (succeeded) {
    portENTER_CRITICAL(&mux_); position_steps_ = 0; portEXIT_CRITICAL(&mux_);
  } else state_ = StepperMotionState::HomingFailed;
}

void StepperAxis::update() {
  const std::uint32_t now = millis();
  updateLimit(now);
  if (state_ == StepperMotionState::Waking &&
      now - last_update_ms_ >= (config_.wake_delay_us + 999U) / 1000U) state_ = StepperMotionState::Stopped;
  if (state_ == StepperMotionState::Continuous &&
      now - last_hold_refresh_ms_ > config_.hold_timeout_ms) stop();
  if (!timer_running_ && (state_ == StepperMotionState::Continuous ||
                          state_ == StepperMotionState::Jogging ||
                          state_ == StepperMotionState::MovingTo)) stop();
  if (state_ == StepperMotionState::HomingRelease && !limit_active_ &&
      digitalRead(config_.lower_limit_gpio) == LOW) {
    stop();
    startMotion(StepperDirection::Down, StepperMotionState::HomingSeek, 0,
                false, config_.homing_speed_steps_per_s);
  }
  if (state_ == StepperMotionState::HomingSeek && limit_active_) finishHoming(true);
  if (state_ == StepperMotionState::HomingRelease || state_ == StepperMotionState::HomingSeek) {
    portENTER_CRITICAL(&mux_);
    const std::uint64_t traveled = total_step_count_ - homing_start_step_count_;
    portEXIT_CRITICAL(&mux_);
    const std::uint64_t maximum_homing_steps =
        config_.homing_max_steps == 0U
            ? static_cast<std::uint64_t>(maximum_position_steps_) * 2U
            : config_.homing_max_steps;
    const std::uint64_t derived_timeout_ms =
        (maximum_homing_steps * 1000U) / config_.homing_speed_steps_per_s +
        config_.hold_timeout_ms;
    const std::uint64_t timeout_ms = config_.homing_timeout_ms == 0U
                                         ? derived_timeout_ms
                                         : config_.homing_timeout_ms;
    if (now - homing_started_ms_ > timeout_ms || traveled > maximum_homing_steps)
      finishHoming(false);
  }
  if (timer_running_) {
    const std::uint32_t target_speed =
        (state_ == StepperMotionState::HomingRelease ||
         state_ == StepperMotionState::HomingSeek)
            ? config_.homing_speed_steps_per_s
            : speed_steps_per_s_;
    const std::uint32_t elapsed_ms = now - last_update_ms_;
    const std::uint32_t increment = std::max<std::uint32_t>(
        1U, static_cast<std::uint32_t>((static_cast<std::uint64_t>(
             acceleration_steps_per_s2_) * elapsed_ms) / 1000U));
    current_speed_steps_per_s_ =
        std::min(target_speed, current_speed_steps_per_s_ + increment);
    portENTER_CRITICAL(&mux_);
    step_interval_us_ = std::max(config_.step_high_us + 2U,
                                 1000000U / current_speed_steps_per_s_);
    portEXIT_CRITICAL(&mux_);
  }
  last_update_ms_ = now;
}

bool StepperAxis::setSpeed(std::uint32_t value) { if (!value || value > 1000000U / (config_.step_high_us + 2U)) return false; speed_steps_per_s_ = value; return true; }
bool StepperAxis::setAcceleration(std::uint32_t value) { if (!value) return false; acceleration_steps_per_s2_ = value; return true; }
bool StepperAxis::setMaximumPositionSteps(std::int64_t value) { if (value <= 0) return false; maximum_position_steps_ = value; if (positionSteps() > value) stop(); return true; }
void StepperAxis::setZeroDebug() { stop(); portENTER_CRITICAL(&mux_); position_steps_ = 0; portEXIT_CRITICAL(&mux_); }
bool StepperAxis::isBusy() const { return state_ != StepperMotionState::Stopped && state_ != StepperMotionState::HomingFailed; }
std::int64_t StepperAxis::positionSteps() const { portENTER_CRITICAL(&mux_); const auto p = position_steps_; portEXIT_CRITICAL(&mux_); return p; }
std::uint32_t StepperAxis::speedStepsPerSecond() const { return current_speed_steps_per_s_; }

const char* StepperAxis::motionStateName() const {
  switch (state_) { case StepperMotionState::Stopped:return "STOPPED"; case StepperMotionState::Waking:return "WAKING"; case StepperMotionState::Continuous:return "CONTINUOUS"; case StepperMotionState::Jogging:return "JOGGING"; case StepperMotionState::MovingTo:return "MOVING_TO"; case StepperMotionState::HomingRelease:return "HOMING_RELEASE"; case StepperMotionState::HomingSeek:return "HOMING_SEEK"; case StepperMotionState::HomingFailed:return "HOMING_FAILED"; } return "UNKNOWN";
}

void IRAM_ATTR StepperAxis::timerThunk() { if (instance_) instance_->onTimer(); }
void IRAM_ATTR StepperAxis::onTimer() {
  portENTER_CRITICAL_ISR(&mux_);
  if (!timer_running_) { portEXIT_CRITICAL_ISR(&mux_); return; }
  if (step_high_) {
    GPIO.out_w1tc = (1UL << config_.step_gpio);
    step_high_ = false;
    position_steps_ += direction_sign_;
    ++total_step_count_;
    if (has_target_ && position_steps_ == target_steps_) timer_running_ = false;
    if (!timer_running_) { timerAlarmDisable(timer_); portEXIT_CRITICAL_ISR(&mux_); return; }
    timerAlarmWrite(timer_, step_interval_us_ - config_.step_high_us, false);
  } else {
    const bool lower_limit_now = (GPIO.in & (1UL << config_.lower_limit_gpio)) != 0U;
    if ((direction_sign_ < 0 && lower_limit_now) ||
        (direction_sign_ > 0 && maximum_position_steps_ > 0 && position_steps_ >= maximum_position_steps_)) {
      timer_running_ = false; timerAlarmDisable(timer_); portEXIT_CRITICAL_ISR(&mux_); return;
    }
    GPIO.out_w1ts = (1UL << config_.step_gpio);
    step_high_ = true;
    timerAlarmWrite(timer_, config_.step_high_us, false);
  }
  timerAlarmEnable(timer_);
  portEXIT_CRITICAL_ISR(&mux_);
}

}  // namespace robot::esp2
