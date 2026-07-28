#include "esp2/ImuAcquisitionService.h"

#include <Arduino.h>
#include <Wire.h>

#include <cmath>

namespace robot::esp2 {

bool imuSnapshotFresh(const ImuAcquisitionSnapshot& snapshot,
                      const std::uint32_t now_us,
                      const std::uint32_t timeout_us) {
  return snapshot.acquisition_running &&
         snapshot.state.initialized &&
         snapshot.state.calibrated &&
         snapshot.state.last_successful_read_us != 0U &&
         now_us - snapshot.state.last_successful_read_us <= timeout_us;
}

bool ImuAcquisitionService::initialize(
    TwoWire& wire, const int sda_gpio, const int scl_gpio,
    const std::uint8_t address,
    const std::uint16_t calibration_sample_count) {
  if (snapshot_queue_ != nullptr || command_queue_ != nullptr ||
      soak_reset_queue_ != nullptr || task_handle_ != nullptr) {
    return false;
  }

  snapshot_queue_ = xQueueCreateStatic(
      1U, sizeof(ImuAcquisitionSnapshot), snapshot_queue_storage_,
      &snapshot_queue_buffer_);
  command_queue_ = xQueueCreateStatic(
      1U, sizeof(Command), command_queue_storage_,
      &command_queue_buffer_);
  soak_reset_queue_ = xQueueCreateStatic(
      1U, sizeof(std::uint8_t), soak_reset_queue_storage_,
      &soak_reset_queue_buffer_);
  if (snapshot_queue_ == nullptr || command_queue_ == nullptr ||
      soak_reset_queue_ == nullptr) {
    return false;
  }

  const bool initialized = imu_.begin(wire, sda_gpio, scl_gpio, address);
  const bool calibrated =
      initialized && imu_.calibrateGyroZ(calibration_sample_count);
  initialized_and_calibrated_ = initialized && calibrated;
  publish(0U, false);
  return initialized_and_calibrated_;
}

bool ImuAcquisitionService::start(const std::uint32_t period_ms,
                                  const UBaseType_t priority,
                                  const BaseType_t core_id) {
  if (!initialized_and_calibrated_ || task_handle_ != nullptr ||
      period_ms == 0U || pdMS_TO_TICKS(period_ms) == 0U) {
    return false;
  }

  period_ms_ = period_ms;
  publish(0U, true);
  task_handle_ = xTaskCreateStaticPinnedToCore(
      taskEntry, "sensor_acquisition", kTaskStackBytes, this, priority,
      task_stack_, &task_buffer_, core_id);
  if (task_handle_ == nullptr) {
    publish(0U, false);
    return false;
  }
  return true;
}

bool ImuAcquisitionService::latest(
    ImuAcquisitionSnapshot& output) const {
  if (snapshot_queue_ == nullptr) {
    output = {};
    return false;
  }
  return xQueuePeek(snapshot_queue_, &output, 0U) == pdPASS;
}

bool ImuAcquisitionService::requestHeadingReset(
    const float heading_deg, std::uint32_t& request_sequence) {
  if (command_queue_ == nullptr || task_handle_ == nullptr ||
      !std::isfinite(heading_deg)) {
    return false;
  }

  ++next_heading_reset_sequence_;
  if (next_heading_reset_sequence_ == 0U) {
    ++next_heading_reset_sequence_;
  }
  const Command command{heading_deg, next_heading_reset_sequence_};
  if (xQueueOverwrite(command_queue_, &command) != pdPASS) {
    return false;
  }
  request_sequence = command.sequence;
  return true;
}

bool ImuAcquisitionService::requestSoakCountersReset() {
  if (soak_reset_queue_ == nullptr || task_handle_ == nullptr) {
    return false;
  }
  constexpr std::uint8_t reset_requested = 1U;
  return xQueueOverwrite(soak_reset_queue_, &reset_requested) == pdPASS;
}

void ImuAcquisitionService::taskEntry(void* const parameter) {
  static_cast<ImuAcquisitionService*>(parameter)->run();
}

void ImuAcquisitionService::run() {
  TickType_t last_wake_tick = xTaskGetTickCount();
  const TickType_t period_ticks = pdMS_TO_TICKS(period_ms_);
  for (;;) {
    processPendingCommands();
    const std::uint32_t acquisition_started_us = micros();
    ++total_acquisition_attempts_;
    const bool acquisition_succeeded = imu_.update();
    const std::uint32_t acquisition_duration_us =
        static_cast<std::uint32_t>(micros() - acquisition_started_us);
    if (acquisition_succeeded) {
      ++successful_acquisitions_;
      consecutive_acquisition_failures_ = 0U;
    } else {
      ++failed_acquisitions_;
      ++consecutive_acquisition_failures_;
    }
    if (acquisition_duration_us >
        maximum_completed_acquisition_duration_us_) {
      maximum_completed_acquisition_duration_us_ =
          acquisition_duration_us;
    }
    publish(acquisition_duration_us, true);
    const TickType_t now_tick = xTaskGetTickCount();
    if (now_tick - last_wake_tick > period_ticks) {
      // A stalled I2C transaction must not cause a burst of catch-up reads
      // that competes with WiFi and other core-0 system work.
      last_wake_tick = now_tick;
    }
    vTaskDelayUntil(&last_wake_tick, period_ticks);
  }
}

void ImuAcquisitionService::publish(
    const std::uint32_t acquisition_duration_us,
    const bool acquisition_running) {
  if (snapshot_queue_ == nullptr) {
    return;
  }
  ImuAcquisitionSnapshot snapshot{};
  snapshot.state = imu_.state();
  snapshot.published_at_us = micros();
  snapshot.acquisition_duration_us = acquisition_duration_us;
  snapshot.total_acquisition_attempts = total_acquisition_attempts_;
  snapshot.successful_acquisitions = successful_acquisitions_;
  snapshot.failed_acquisitions = failed_acquisitions_;
  snapshot.consecutive_acquisition_failures =
      consecutive_acquisition_failures_;
  snapshot.maximum_completed_acquisition_duration_us =
      maximum_completed_acquisition_duration_us_;
  snapshot.last_heading_reset_sequence = last_heading_reset_sequence_;
  snapshot.acquisition_running = acquisition_running;
  (void)xQueueOverwrite(snapshot_queue_, &snapshot);
}

void ImuAcquisitionService::processPendingCommands() {
  if (command_queue_ == nullptr) {
    return;
  }
  Command command{};
  while (xQueueReceive(command_queue_, &command, 0U) == pdPASS) {
    imu_.resetHeading(command.heading_deg);
    last_heading_reset_sequence_ = command.sequence;
    publish(0U, true);
  }
  std::uint8_t reset_requested = 0U;
  if (soak_reset_queue_ != nullptr &&
      xQueueReceive(soak_reset_queue_, &reset_requested, 0U) == pdPASS) {
    resetSoakCounters();
    publish(0U, true);
  }
}

void ImuAcquisitionService::resetSoakCounters() {
  total_acquisition_attempts_ = 0U;
  successful_acquisitions_ = 0U;
  failed_acquisitions_ = 0U;
  consecutive_acquisition_failures_ = 0U;
  maximum_completed_acquisition_duration_us_ = 0U;
}

}  // namespace robot::esp2
