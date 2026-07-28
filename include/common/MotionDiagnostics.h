#pragma once

#include <cstddef>
#include <cstdint>

#include "common/ImuTurnController.h"
#include "common/RobotTestMode.h"
#include "common/Units.h"

namespace robot {

constexpr std::size_t kMotionDiagnosticSampleCapacity = 20U;

enum class MotionDiagnosticEvent : std::uint8_t {
  Periodic = 0,
  WebDriveStart = 1,
  WebDriveHeartbeatAfterStop = 2,
  WebStop = 3,
  OutputsDisabled = 4,
  CommandDeadmanExpired = 5,
  ImuTurnStarted = 6,
  ImuTurnStopped = 7,
  ImuTurnCompleted = 8,
  ImuTurnTimedOut = 9,
  ImuTurnFaulted = 10,
};

enum class MotionDiagnosticTrigger : std::uint8_t {
  None = 0,
  ManualFreeze = 1,
  CommandDeadmanExpired = 2,
  DriveHeartbeatAfterStop = 3,
  ImuTurnCompleted = 4,
  ImuTurnTimedOut = 5,
  ImuTurnFaulted = 6,
};

struct MotionDiagnosticSample {
  Milliseconds timestamp_ms{0U};
  MotionDiagnosticEvent event{MotionDiagnosticEvent::Periodic};
  RobotTestMode mode{RobotTestMode::Disabled};

  std::uint32_t loop_interval_us{0U};
  std::uint32_t loop_work_us{0U};
  std::uint32_t imu_update_us{0U};
  std::uint32_t web_handle_us{0U};

  // Wheel order is FL, FR, BL, BR.
  std::int16_t requested_command_milli[4]{};
  std::int16_t applied_command_milli[4]{};
  // The front driver retains its last desired value after disable; keeping it
  // here makes that distinction explicit in the diagnostic report.
  std::int16_t front_driver_desired_command_milli[2]{};
  std::uint32_t front_command_expires_at_ms[2]{};
  // PWM order is FL0, FL1, FR0, FR1.
  std::uint32_t front_pwm_readback[4]{};

  std::uint16_t rear_sequence{0U};
  Milliseconds rear_command_age_ms{0U};
  Milliseconds esp1_status_age_ms{0U};
  bool esp1_status_fresh{false};
  std::uint32_t esp1_packet_error_count{0U};

  bool command_deadman_armed{false};
  Milliseconds command_deadline_ms{0U};
  Milliseconds last_command_age_ms{0U};

  ImuTurnState imu_turn_state{ImuTurnState::Idle};
  float heading_deg{0.0F};
  float target_heading_deg{0.0F};
  float angle_error_deg{0.0F};
  float yaw_rate_dps{0.0F};
  float rotation_command{0.0F};
};

class MotionDiagnostics {
 public:
  void reset(Milliseconds now_ms);
  void observeLoop(std::uint32_t interval_us, std::uint32_t work_us,
                   std::uint32_t imu_update_us,
                   std::uint32_t web_handle_us,
                   Milliseconds expected_period_ms);
  void record(const MotionDiagnosticSample& sample);
  void freeze(MotionDiagnosticTrigger trigger, Milliseconds now_ms);

  void noteWebDrive(Milliseconds now_ms, bool starts_new_hold,
                    bool arrived_after_stop);
  void noteWebStop(Milliseconds now_ms);

  bool frozen() const { return frozen_; }
  MotionDiagnosticTrigger trigger() const { return trigger_; }
  Milliseconds resetAtMs() const { return reset_at_ms_; }
  Milliseconds frozenAtMs() const { return frozen_at_ms_; }
  std::uint32_t captureId() const { return capture_id_; }
  std::size_t sampleCount() const { return sample_count_; }
  const MotionDiagnosticSample& sampleFromOldest(std::size_t index) const;

  std::uint32_t lastLoopIntervalUs() const {
    return last_loop_interval_us_;
  }
  std::uint32_t maximumLoopIntervalUs() const {
    return maximum_loop_interval_us_;
  }
  std::uint32_t maximumLoopWorkUs() const { return maximum_loop_work_us_; }
  std::uint32_t maximumImuUpdateUs() const { return maximum_imu_update_us_; }
  std::uint32_t maximumWebHandleUs() const { return maximum_web_handle_us_; }
  std::uint32_t missedDeadlineCount() const {
    return missed_deadline_count_;
  }
  std::uint32_t webDriveRequestCount() const {
    return web_drive_request_count_;
  }
  std::uint32_t webStopRequestCount() const {
    return web_stop_request_count_;
  }
  std::uint32_t driveAfterStopCount() const {
    return drive_after_stop_count_;
  }
  Milliseconds lastWebDriveAtMs() const { return last_web_drive_at_ms_; }
  Milliseconds lastWebStopAtMs() const { return last_web_stop_at_ms_; }

 private:
  MotionDiagnosticSample samples_[kMotionDiagnosticSampleCapacity]{};
  std::size_t next_sample_index_{0U};
  std::size_t sample_count_{0U};
  std::uint32_t capture_id_{0U};
  Milliseconds reset_at_ms_{0U};
  Milliseconds frozen_at_ms_{0U};
  MotionDiagnosticTrigger trigger_{MotionDiagnosticTrigger::None};
  bool frozen_{false};

  std::uint32_t last_loop_interval_us_{0U};
  std::uint32_t maximum_loop_interval_us_{0U};
  std::uint32_t maximum_loop_work_us_{0U};
  std::uint32_t maximum_imu_update_us_{0U};
  std::uint32_t maximum_web_handle_us_{0U};
  std::uint32_t missed_deadline_count_{0U};

  std::uint32_t web_drive_request_count_{0U};
  std::uint32_t web_stop_request_count_{0U};
  std::uint32_t drive_after_stop_count_{0U};
  Milliseconds last_web_drive_at_ms_{0U};
  Milliseconds last_web_stop_at_ms_{0U};
};

const char* motionDiagnosticEventName(MotionDiagnosticEvent event);
const char* motionDiagnosticTriggerName(MotionDiagnosticTrigger trigger);
bool writeMotionDiagnosticsJson(const MotionDiagnostics& diagnostics,
                                char* output, std::size_t capacity);

}  // namespace robot
