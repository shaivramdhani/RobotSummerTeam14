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

const char* imuDisconnectReason(
    const ImuAcquisitionSnapshot& snapshot,
    const std::uint32_t now_us,
    const std::uint32_t timeout_us) {
  if (!snapshot.state.configured) {
    return "PINS_UNASSIGNED";
  }
  if (!snapshot.state.initialized) {
    return imuInitializationErrorName(
        snapshot.state.initialization_error);
  }
  if (!snapshot.state.calibrated) {
    return snapshot.state.last_read_failure_reason !=
                   ImuReadFailureReason::None
               ? imuReadFailureReasonName(
                     snapshot.state.last_read_failure_reason)
               : "CALIBRATION_FAILED";
  }
  if (!snapshot.acquisition_running) {
    return "ACQUISITION_NOT_RUNNING";
  }
  if (snapshot.state.last_successful_read_us == 0U) {
    return "NO_SUCCESSFUL_READ";
  }
  if (!snapshot.state.healthy) {
    return snapshot.state.last_read_failure_reason !=
                   ImuReadFailureReason::None
               ? imuReadFailureReasonName(
                     snapshot.state.last_read_failure_reason)
               : "IMU_UNHEALTHY";
  }
  if (now_us - snapshot.state.last_successful_read_us > timeout_us) {
    return snapshot.consecutive_acquisition_failures > 0U &&
                   snapshot.state.last_read_failure_reason !=
                       ImuReadFailureReason::None
               ? imuReadFailureReasonName(
                     snapshot.state.last_read_failure_reason)
               : "STALE_DATA";
  }
  return "NONE";
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
  publish(0U, false, false);
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
  publish(0U, true, false);
  task_handle_ = xTaskCreateStaticPinnedToCore(
      taskEntry, "sensor_acquisition", kTaskStackBytes, this, priority,
      task_stack_, &task_buffer_, core_id);
  if (task_handle_ == nullptr) {
    publish(0U, false, false);
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
    const std::uint32_t iteration_started_us = micros();
    acquisition_loop_interval_us_ =
        previous_iteration_started_us_ == 0U
            ? 0U
            : iteration_started_us -
                  previous_iteration_started_us_;
    previous_iteration_started_us_ = iteration_started_us;
    if (acquisition_loop_interval_us_ >
        maximum_acquisition_loop_interval_us_) {
      maximum_acquisition_loop_interval_us_ =
          acquisition_loop_interval_us_;
    }
    if (acquisition_loop_interval_us_ >
        period_ms_ * 1000U + 1000U) {
      ++delayed_iteration_count_;
    }

    const std::uint32_t synchronization_started_us = micros();
    processPendingCommands();
    synchronization_duration_us_ =
        static_cast<std::uint32_t>(
            micros() - synchronization_started_us);
    if (synchronization_duration_us_ >
        maximum_synchronization_duration_us_) {
      maximum_synchronization_duration_us_ =
          synchronization_duration_us_;
    }

    const std::uint32_t acquisition_started_us = micros();
    ++total_acquisition_attempts_;
    const bool acquisition_succeeded = imu_.update();
    const std::uint32_t acquisition_duration_us =
        static_cast<std::uint32_t>(micros() - acquisition_started_us);
    if (acquisition_succeeded) {
      ++successful_acquisitions_;
      ++lifetime_successful_acquisitions_;
      consecutive_acquisition_failures_ = 0U;
    } else {
      ++failed_acquisitions_;
      ++lifetime_failed_acquisitions_;
      ++consecutive_acquisition_failures_;
    }
    if (acquisition_duration_us >
        maximum_completed_acquisition_duration_us_) {
      maximum_completed_acquisition_duration_us_ =
          acquisition_duration_us;
    }
    publish(acquisition_duration_us, true,
            acquisition_succeeded);
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
    const bool acquisition_running,
    const bool successful_acquisition) {
  if (snapshot_queue_ == nullptr) {
    return;
  }
  const std::uint32_t published_at_us = micros();
  ++publication_sequence_;
  if (successful_acquisition) {
    successful_sample_publication_gap_us_ =
        previous_successful_sample_published_at_us_ == 0U
            ? 0U
            : published_at_us -
                  previous_successful_sample_published_at_us_;
    previous_successful_sample_published_at_us_ =
        published_at_us;
    if (successful_sample_publication_gap_us_ >
        maximum_successful_sample_publication_gap_us_) {
      maximum_successful_sample_publication_gap_us_ =
          successful_sample_publication_gap_us_;
    }
    ++successful_sample_sequence_;
  }
  ImuAcquisitionSnapshot snapshot{};
  snapshot.state = imu_.state();
  snapshot.published_at_us = published_at_us;
  snapshot.acquisition_duration_us = acquisition_duration_us;
  snapshot.total_acquisition_attempts = total_acquisition_attempts_;
  snapshot.successful_acquisitions = successful_acquisitions_;
  snapshot.failed_acquisitions = failed_acquisitions_;
  snapshot.lifetime_successful_acquisitions =
      lifetime_successful_acquisitions_;
  snapshot.lifetime_failed_acquisitions =
      lifetime_failed_acquisitions_;
  snapshot.consecutive_acquisition_failures =
      consecutive_acquisition_failures_;
  snapshot.maximum_completed_acquisition_duration_us =
      maximum_completed_acquisition_duration_us_;
  snapshot.acquisition_loop_interval_us =
      acquisition_loop_interval_us_;
  snapshot.maximum_acquisition_loop_interval_us =
      maximum_acquisition_loop_interval_us_;
  snapshot.synchronization_duration_us =
      synchronization_duration_us_;
  snapshot.maximum_synchronization_duration_us =
      maximum_synchronization_duration_us_;
  snapshot.successful_read_to_publication_us =
      successful_read_to_publication_us_;
  snapshot.maximum_successful_read_to_publication_us =
      maximum_successful_read_to_publication_us_;
  snapshot.publication_queue_duration_us =
      publication_queue_duration_us_;
  snapshot.maximum_publication_queue_duration_us =
      maximum_publication_queue_duration_us_;
  snapshot.successful_sample_publication_gap_us =
      successful_sample_publication_gap_us_;
  snapshot.maximum_successful_sample_publication_gap_us =
      maximum_successful_sample_publication_gap_us_;
  snapshot.last_successful_sample_published_at_us =
      previous_successful_sample_published_at_us_;
  snapshot.publication_sequence = publication_sequence_;
  snapshot.successful_sample_sequence =
      successful_sample_sequence_;
  snapshot.delayed_iteration_count = delayed_iteration_count_;
  snapshot.last_heading_reset_sequence = last_heading_reset_sequence_;
  snapshot.acquisition_running = acquisition_running;
  const std::uint32_t publication_started_us = micros();
  (void)xQueueOverwrite(snapshot_queue_, &snapshot);
  publication_queue_duration_us_ =
      static_cast<std::uint32_t>(
          micros() - publication_started_us);
  if (publication_queue_duration_us_ >
      maximum_publication_queue_duration_us_) {
    maximum_publication_queue_duration_us_ =
        publication_queue_duration_us_;
  }
  if (successful_acquisition) {
    successful_read_to_publication_us_ =
        static_cast<std::uint32_t>(
            micros() -
            imu_.state().last_successful_read_us);
    if (successful_read_to_publication_us_ >
        maximum_successful_read_to_publication_us_) {
      maximum_successful_read_to_publication_us_ =
          successful_read_to_publication_us_;
    }
  }
}

void ImuAcquisitionService::processPendingCommands() {
  if (command_queue_ == nullptr) {
    return;
  }
  Command command{};
  while (xQueueReceive(command_queue_, &command, 0U) == pdPASS) {
    imu_.resetHeading(command.heading_deg);
    last_heading_reset_sequence_ = command.sequence;
    publish(0U, true, false);
  }
  std::uint8_t reset_requested = 0U;
  if (soak_reset_queue_ != nullptr &&
      xQueueReceive(soak_reset_queue_, &reset_requested, 0U) == pdPASS) {
    resetSoakCounters();
    publish(0U, true, false);
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
