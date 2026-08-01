#pragma once

#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "esp2/Mpu6050Imu.h"

class TwoWire;

namespace robot::esp2 {

struct ImuAcquisitionSnapshot {
  ImuState state{};
  std::uint32_t published_at_us{0U};
  std::uint32_t acquisition_duration_us{0U};
  std::uint32_t total_acquisition_attempts{0U};
  std::uint32_t successful_acquisitions{0U};
  std::uint32_t failed_acquisitions{0U};
  std::uint32_t lifetime_successful_acquisitions{0U};
  std::uint32_t lifetime_failed_acquisitions{0U};
  std::uint32_t consecutive_acquisition_failures{0U};
  std::uint32_t maximum_completed_acquisition_duration_us{0U};
  std::uint32_t acquisition_loop_interval_us{0U};
  std::uint32_t maximum_acquisition_loop_interval_us{0U};
  std::uint32_t synchronization_duration_us{0U};
  std::uint32_t maximum_synchronization_duration_us{0U};
  std::uint32_t successful_read_to_publication_us{0U};
  std::uint32_t maximum_successful_read_to_publication_us{0U};
  std::uint32_t publication_queue_duration_us{0U};
  std::uint32_t maximum_publication_queue_duration_us{0U};
  std::uint32_t successful_sample_publication_gap_us{0U};
  std::uint32_t maximum_successful_sample_publication_gap_us{0U};
  std::uint32_t last_successful_sample_published_at_us{0U};
  std::uint32_t publication_sequence{0U};
  std::uint32_t successful_sample_sequence{0U};
  std::uint32_t delayed_iteration_count{0U};
  std::uint32_t last_heading_reset_sequence{0U};
  bool acquisition_running{false};
};

bool imuSnapshotFresh(const ImuAcquisitionSnapshot& snapshot,
                      std::uint32_t now_us, std::uint32_t timeout_us);
const char* imuDisconnectReason(
    const ImuAcquisitionSnapshot& snapshot, std::uint32_t now_us,
    std::uint32_t timeout_us);

class ImuAcquisitionService {
 public:
  bool initialize(TwoWire& wire, int sda_gpio, int scl_gpio,
                  std::uint8_t address,
                  std::uint16_t calibration_sample_count);
  bool start(std::uint32_t period_ms, UBaseType_t priority,
             BaseType_t core_id);

  bool latest(ImuAcquisitionSnapshot& output) const;
  bool requestHeadingReset(float heading_deg,
                           std::uint32_t& request_sequence);
  bool requestSoakCountersReset();

 private:
  struct Command {
    float heading_deg{0.0F};
    std::uint32_t sequence{0U};
  };

  static constexpr std::uint32_t kTaskStackBytes = 4096U;

  static void taskEntry(void* parameter);
  void run();
  void publish(std::uint32_t acquisition_duration_us,
               bool acquisition_running,
               bool successful_acquisition);
  void processPendingCommands();
  void resetSoakCounters();

  Mpu6050Imu imu_{};
  QueueHandle_t snapshot_queue_{nullptr};
  StaticQueue_t snapshot_queue_buffer_{};
  std::uint8_t snapshot_queue_storage_[sizeof(ImuAcquisitionSnapshot)]{};
  QueueHandle_t command_queue_{nullptr};
  StaticQueue_t command_queue_buffer_{};
  std::uint8_t command_queue_storage_[sizeof(Command)]{};
  QueueHandle_t soak_reset_queue_{nullptr};
  StaticQueue_t soak_reset_queue_buffer_{};
  std::uint8_t soak_reset_queue_storage_[sizeof(std::uint8_t)]{};
  TaskHandle_t task_handle_{nullptr};
  StaticTask_t task_buffer_{};
  StackType_t task_stack_[kTaskStackBytes]{};
  std::uint32_t period_ms_{0U};
  std::uint32_t next_heading_reset_sequence_{0U};
  std::uint32_t last_heading_reset_sequence_{0U};
  std::uint32_t total_acquisition_attempts_{0U};
  std::uint32_t successful_acquisitions_{0U};
  std::uint32_t failed_acquisitions_{0U};
  std::uint32_t lifetime_successful_acquisitions_{0U};
  std::uint32_t lifetime_failed_acquisitions_{0U};
  std::uint32_t consecutive_acquisition_failures_{0U};
  std::uint32_t maximum_completed_acquisition_duration_us_{0U};
  std::uint32_t previous_iteration_started_us_{0U};
  std::uint32_t acquisition_loop_interval_us_{0U};
  std::uint32_t maximum_acquisition_loop_interval_us_{0U};
  std::uint32_t synchronization_duration_us_{0U};
  std::uint32_t maximum_synchronization_duration_us_{0U};
  std::uint32_t successful_read_to_publication_us_{0U};
  std::uint32_t maximum_successful_read_to_publication_us_{0U};
  std::uint32_t publication_queue_duration_us_{0U};
  std::uint32_t maximum_publication_queue_duration_us_{0U};
  std::uint32_t previous_successful_sample_published_at_us_{0U};
  std::uint32_t successful_sample_publication_gap_us_{0U};
  std::uint32_t maximum_successful_sample_publication_gap_us_{0U};
  std::uint32_t publication_sequence_{0U};
  std::uint32_t successful_sample_sequence_{0U};
  std::uint32_t delayed_iteration_count_{0U};
  bool initialized_and_calibrated_{false};
};

}  // namespace robot::esp2
