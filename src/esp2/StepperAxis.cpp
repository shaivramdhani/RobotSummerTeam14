#include "esp2/StepperAxis.h"

#include <driver/timer.h>

#include <algorithm>

namespace robot::esp2 {

namespace {

// timerBegin(0, ...) maps to timer group 0, timer 0 in the ESP32 Arduino
// core used by this project. ISR rescheduling must use the matching ESP-IDF
// ISR-safe APIs and an absolute future counter value.
constexpr timer_group_t kStepperTimerGroup = TIMER_GROUP_0;
constexpr timer_idx_t kStepperTimerIndex = TIMER_0;

void IRAM_ATTR scheduleStepperAlarmFromIsr(const std::uint32_t delay_us) {
  const std::uint64_t now = timer_group_get_counter_value_in_isr(
      kStepperTimerGroup, kStepperTimerIndex);
  timer_group_set_alarm_value_in_isr(
      kStepperTimerGroup, kStepperTimerIndex, now + delay_us);
  timer_group_enable_alarm_in_isr(kStepperTimerGroup, kStepperTimerIndex);
}

}  // namespace

StepperAxis* StepperAxis::instance_ = nullptr;

StepperAxis::StepperAxis(const StepperAxisConfig& config)
    : config_(config), speed_steps_per_s_(config.speed_steps_per_s),
      limit_search_speed_steps_per_s_(config.homing_speed_steps_per_s),
      acceleration_steps_per_s2_(config.acceleration_steps_per_s2),
      maximum_position_steps_(config.maximum_position_steps) {}

void StepperAxis::begin() {
  pinMode(config_.sleep_gpio, OUTPUT);
  pinMode(config_.direction_gpio, OUTPUT);
  pinMode(config_.step_gpio, OUTPUT);
  pinMode(config_.lower_limit_gpio, INPUT);  // External circuit defines LOW/HIGH.
  pinMode(config_.upper_limit_gpio, INPUT);  // External circuit defines LOW/HIGH.
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
  if (speed == 0U ||
      (direction == StepperDirection::Down &&
       (limit_active_ || digitalRead(config_.lower_limit_gpio) == HIGH)) ||
      (direction == StepperDirection::Up &&
       (upper_limit_active_ ||
        digitalRead(config_.upper_limit_gpio) == HIGH))) return false;
  if (direction == StepperDirection::Up &&
      state != StepperMotionState::SeekingUpperLimit &&
      (maximum_position_steps_ <= 0 || positionSteps() >= maximum_position_steps_)) return false;
  const bool was_sleeping = sleeping_;
  wake();
  const std::int8_t sign = static_cast<std::int8_t>(direction);
  portENTER_CRITICAL(&mux_);
  direction_sign_ = sign;
  target_steps_ = target;
  has_target_ = has_target;
  ignore_maximum_position_ = state == StepperMotionState::SeekingUpperLimit;
  current_speed_steps_per_s_ = 1U;
  step_interval_us_ = 1000000U / current_speed_steps_per_s_;
  step_high_ = false;
  timer_running_ = true;
  portEXIT_CRITICAL(&mux_);
  // This mechanism's wiring maps DIR LOW to physical Up and DIR HIGH to Down.
  digitalWrite(config_.direction_gpio, sign > 0 ? LOW : HIGH);
  digitalWrite(config_.step_gpio, LOW);
  state_ = state;
  timerAlarmDisable(timer_);
  timerWrite(timer_, 0U);
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
  return moveToLowerLimit();
}

bool StepperAxis::startLimitSearch(const StepperDirection direction,
                                   const StepperMotionState state) {
  stop();
  homing_started_ms_ = millis();
  portENTER_CRITICAL(&mux_);
  homing_start_step_count_ = total_step_count_;
  portEXIT_CRITICAL(&mux_);
  const int limit_gpio = direction == StepperDirection::Down
                             ? config_.lower_limit_gpio
                             : config_.upper_limit_gpio;
  if (digitalRead(limit_gpio) == HIGH) {
    wake();
    state_ = state;
    return true;
  }
  return startMotion(direction, state, 0, false,
                     limit_search_speed_steps_per_s_);
}

bool StepperAxis::moveToLowerLimit() {
  homed_ = false;
  return startLimitSearch(StepperDirection::Down,
                          StepperMotionState::SeekingLowerLimit);
}

bool StepperAxis::moveToUpperLimit() {
  return startLimitSearch(StepperDirection::Up,
                          StepperMotionState::SeekingUpperLimit);
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
  if (state_ != StepperMotionState::LimitSearchFailed) state_ = StepperMotionState::Stopped;
}

void StepperAxis::updateLimits(std::uint32_t now_ms) {
  const bool raw_lower = digitalRead(config_.lower_limit_gpio) == HIGH;
  const bool raw_upper = digitalRead(config_.upper_limit_gpio) == HIGH;
  raw_lower_limit_active_ = raw_lower;
  raw_upper_limit_active_ = raw_upper;
  if (raw_lower != limit_candidate_) {
    limit_candidate_ = raw_lower;
    limit_changed_ms_ = now_ms;
  }
  if (raw_lower == limit_candidate_ &&
      now_ms - limit_changed_ms_ >= config_.limit_debounce_ms)
    limit_active_ = limit_candidate_;
  if (raw_upper != upper_limit_candidate_) {
    upper_limit_candidate_ = raw_upper;
    upper_limit_changed_ms_ = now_ms;
  }
  if (raw_upper == upper_limit_candidate_ &&
      now_ms - upper_limit_changed_ms_ >= config_.limit_debounce_ms)
    upper_limit_active_ = upper_limit_candidate_;
}

void StepperAxis::finishLimitSearch(const bool succeeded,
                                    const bool at_lower_limit) {
  stop();
  if (succeeded) {
    portENTER_CRITICAL(&mux_);
    position_steps_ = at_lower_limit ? 0 : maximum_position_steps_;
    portEXIT_CRITICAL(&mux_);
    if (at_lower_limit) homed_ = true;
  } else {
    homed_ = false;
    state_ = StepperMotionState::LimitSearchFailed;
  }
}

void StepperAxis::update() {
  const std::uint32_t now = millis();
  updateLimits(now);
  if (state_ == StepperMotionState::Waking &&
      now - last_update_ms_ >= (config_.wake_delay_us + 999U) / 1000U) state_ = StepperMotionState::Stopped;
  if (state_ == StepperMotionState::Continuous &&
      now - last_hold_refresh_ms_ > config_.hold_timeout_ms) stop();
  if (!timer_running_ && (state_ == StepperMotionState::Continuous ||
                          state_ == StepperMotionState::Jogging ||
                          state_ == StepperMotionState::MovingTo)) stop();
  const bool seeking_lower = state_ == StepperMotionState::SeekingLowerLimit;
  const bool seeking_upper = state_ == StepperMotionState::SeekingUpperLimit;
  if ((seeking_lower && limit_active_) ||
      (seeking_upper && upper_limit_active_)) {
    finishLimitSearch(true, seeking_lower);
  } else if ((seeking_lower || seeking_upper) && !timer_running_) {
    const StepperDirection direction = seeking_lower
                                           ? StepperDirection::Down
                                           : StepperDirection::Up;
    const int limit_gpio = seeking_lower ? config_.lower_limit_gpio
                                         : config_.upper_limit_gpio;
    if (digitalRead(limit_gpio) == LOW) {
      startMotion(direction, state_, 0, false,
                  limit_search_speed_steps_per_s_);
    }
  }
  if (state_ == StepperMotionState::SeekingLowerLimit ||
      state_ == StepperMotionState::SeekingUpperLimit) {
    portENTER_CRITICAL(&mux_);
    const std::uint64_t traveled = total_step_count_ - homing_start_step_count_;
    portEXIT_CRITICAL(&mux_);
    const std::uint64_t maximum_homing_steps =
        config_.homing_max_steps == 0U
            ? static_cast<std::uint64_t>(maximum_position_steps_) * 2U
            : config_.homing_max_steps;
    const std::uint64_t derived_timeout_ms =
        (maximum_homing_steps * 1000U) / limit_search_speed_steps_per_s_ +
        config_.hold_timeout_ms;
    const std::uint64_t timeout_ms = config_.homing_timeout_ms == 0U
                                         ? derived_timeout_ms
                                         : config_.homing_timeout_ms;
    if (now - homing_started_ms_ > timeout_ms || traveled > maximum_homing_steps)
      finishLimitSearch(false, false);
  }
  if (timer_running_) {
    const std::uint32_t target_speed =
        (state_ == StepperMotionState::SeekingLowerLimit ||
         state_ == StepperMotionState::SeekingUpperLimit)
            ? limit_search_speed_steps_per_s_
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
bool StepperAxis::setLimitSearchSpeed(std::uint32_t value) { if (!value || value > 1000000U / (config_.step_high_us + 2U)) return false; limit_search_speed_steps_per_s_ = value; return true; }
bool StepperAxis::setMotionSpeeds(const std::uint32_t manual_steps_per_s,
                                  const std::uint32_t limit_steps_per_s) {
  const std::uint32_t maximum_speed =
      1000000U / (config_.step_high_us + 2U);
  if (manual_steps_per_s == 0U || limit_steps_per_s == 0U ||
      manual_steps_per_s > maximum_speed ||
      limit_steps_per_s > maximum_speed) {
    return false;
  }
  speed_steps_per_s_ = manual_steps_per_s;
  limit_search_speed_steps_per_s_ = limit_steps_per_s;
  return true;
}
bool StepperAxis::setAcceleration(std::uint32_t value) { if (!value) return false; acceleration_steps_per_s2_ = value; return true; }
bool StepperAxis::setMaximumPositionSteps(std::int64_t value) { if (value <= 0) return false; maximum_position_steps_ = value; if (positionSteps() > value) stop(); return true; }
void StepperAxis::setZeroDebug() { stop(); portENTER_CRITICAL(&mux_); position_steps_ = 0; portEXIT_CRITICAL(&mux_); }
bool StepperAxis::isBusy() const { return state_ != StepperMotionState::Stopped && state_ != StepperMotionState::LimitSearchFailed; }
std::int64_t StepperAxis::positionSteps() const { portENTER_CRITICAL(&mux_); const auto p = position_steps_; portEXIT_CRITICAL(&mux_); return p; }
std::uint32_t StepperAxis::speedStepsPerSecond() const { return current_speed_steps_per_s_; }

const char* StepperAxis::motionStateName() const {
  switch (state_) { case StepperMotionState::Stopped:return "STOPPED"; case StepperMotionState::Waking:return "WAKING"; case StepperMotionState::Continuous:return "CONTINUOUS"; case StepperMotionState::Jogging:return "JOGGING"; case StepperMotionState::MovingTo:return "MOVING_TO"; case StepperMotionState::SeekingLowerLimit:return "GOING_TO_BOTTOM"; case StepperMotionState::SeekingUpperLimit:return "GOING_TO_TOP"; case StepperMotionState::LimitSearchFailed:return "LIMIT_SEARCH_FAILED"; } return "UNKNOWN";
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
    if (!timer_running_) { portEXIT_CRITICAL_ISR(&mux_); return; }
    scheduleStepperAlarmFromIsr(step_interval_us_ - config_.step_high_us);
  } else {
    const bool lower_limit_now = (GPIO.in & (1UL << config_.lower_limit_gpio)) != 0U;
    const bool upper_limit_now = (GPIO.in & (1UL << config_.upper_limit_gpio)) != 0U;
    if ((direction_sign_ < 0 && lower_limit_now) ||
        (direction_sign_ > 0 && upper_limit_now) ||
        (direction_sign_ > 0 && !ignore_maximum_position_ &&
         maximum_position_steps_ > 0 &&
         position_steps_ >= maximum_position_steps_)) {
      timer_running_ = false; portEXIT_CRITICAL_ISR(&mux_); return;
    }
    GPIO.out_w1ts = (1UL << config_.step_gpio);
    step_high_ = true;
    scheduleStepperAlarmFromIsr(config_.step_high_us);
  }
  portEXIT_CRITICAL_ISR(&mux_);
}

}  // namespace robot::esp2
