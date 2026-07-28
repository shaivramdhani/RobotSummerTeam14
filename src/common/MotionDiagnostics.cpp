#include "common/MotionDiagnostics.h"

#include <cstdarg>
#include <cstdio>

namespace robot {

namespace {

class JsonWriter {
 public:
  JsonWriter(char* output, const std::size_t capacity)
      : output_(output), capacity_(capacity) {
    if (capacity_ > 0U) {
      output_[0] = '\0';
    }
  }

  bool append(const char* format, ...) {
    if (!ok_ || output_ == nullptr || capacity_ == 0U || used_ >= capacity_) {
      ok_ = false;
      return false;
    }

    va_list args;
    va_start(args, format);
    const int written =
        std::vsnprintf(output_ + used_, capacity_ - used_, format, args);
    va_end(args);
    if (written < 0 ||
        static_cast<std::size_t>(written) >= capacity_ - used_) {
      ok_ = false;
      output_[capacity_ - 1U] = '\0';
      return false;
    }
    used_ += static_cast<std::size_t>(written);
    return true;
  }

  bool ok() const { return ok_; }

 private:
  char* output_{nullptr};
  std::size_t capacity_{0U};
  std::size_t used_{0U};
  bool ok_{true};
};

const char* jsonBool(const bool value) {
  return value ? "true" : "false";
}

std::uint32_t maximum(const std::uint32_t lhs, const std::uint32_t rhs) {
  return lhs > rhs ? lhs : rhs;
}

}  // namespace

void MotionDiagnostics::reset(const Milliseconds now_ms) {
  next_sample_index_ = 0U;
  sample_count_ = 0U;
  ++capture_id_;
  reset_at_ms_ = now_ms;
  frozen_at_ms_ = 0U;
  trigger_ = MotionDiagnosticTrigger::None;
  frozen_ = false;

  last_loop_interval_us_ = 0U;
  maximum_loop_interval_us_ = 0U;
  maximum_loop_work_us_ = 0U;
  maximum_imu_update_us_ = 0U;
  maximum_web_handle_us_ = 0U;
  missed_deadline_count_ = 0U;

  web_drive_request_count_ = 0U;
  web_stop_request_count_ = 0U;
  drive_after_stop_count_ = 0U;
  last_web_drive_at_ms_ = 0U;
  last_web_stop_at_ms_ = 0U;
}

void MotionDiagnostics::observeLoop(
    const std::uint32_t interval_us, const std::uint32_t work_us,
    const std::uint32_t imu_update_us, const std::uint32_t web_handle_us,
    const Milliseconds expected_period_ms) {
  if (frozen_) {
    return;
  }

  last_loop_interval_us_ = interval_us;
  maximum_loop_interval_us_ =
      maximum(maximum_loop_interval_us_, interval_us);
  maximum_loop_work_us_ = maximum(maximum_loop_work_us_, work_us);
  maximum_imu_update_us_ = maximum(maximum_imu_update_us_, imu_update_us);
  maximum_web_handle_us_ = maximum(maximum_web_handle_us_, web_handle_us);

  const std::uint32_t expected_period_us =
      expected_period_ms > (UINT32_MAX / 1000U)
          ? UINT32_MAX
          : expected_period_ms * 1000U;
  if (expected_period_us > 0U && interval_us > expected_period_us) {
    const std::uint32_t elapsed_periods = interval_us / expected_period_us;
    if (elapsed_periods > 1U) {
      missed_deadline_count_ += elapsed_periods - 1U;
    }
  }
}

void MotionDiagnostics::record(const MotionDiagnosticSample& sample) {
  if (frozen_) {
    return;
  }
  samples_[next_sample_index_] = sample;
  next_sample_index_ =
      (next_sample_index_ + 1U) % kMotionDiagnosticSampleCapacity;
  if (sample_count_ < kMotionDiagnosticSampleCapacity) {
    ++sample_count_;
  }
}

void MotionDiagnostics::freeze(const MotionDiagnosticTrigger trigger,
                               const Milliseconds now_ms) {
  if (frozen_) {
    return;
  }
  frozen_ = true;
  trigger_ = trigger;
  frozen_at_ms_ = now_ms;
}

void MotionDiagnostics::noteWebDrive(const Milliseconds now_ms,
                                     const bool starts_new_hold,
                                     const bool arrived_after_stop) {
  (void)starts_new_hold;
  if (frozen_) {
    return;
  }
  ++web_drive_request_count_;
  last_web_drive_at_ms_ = now_ms;
  if (arrived_after_stop) {
    ++drive_after_stop_count_;
  }
}

void MotionDiagnostics::noteWebStop(const Milliseconds now_ms) {
  if (frozen_) {
    return;
  }
  ++web_stop_request_count_;
  last_web_stop_at_ms_ = now_ms;
}

const MotionDiagnosticSample& MotionDiagnostics::sampleFromOldest(
    const std::size_t index) const {
  static const MotionDiagnosticSample empty{};
  if (index >= sample_count_) {
    return empty;
  }
  const std::size_t oldest_index =
      sample_count_ < kMotionDiagnosticSampleCapacity ? 0U
                                                     : next_sample_index_;
  return samples_[(oldest_index + index) %
                  kMotionDiagnosticSampleCapacity];
}

const char* motionDiagnosticEventName(const MotionDiagnosticEvent event) {
  switch (event) {
    case MotionDiagnosticEvent::Periodic:
      return "PERIODIC";
    case MotionDiagnosticEvent::WebDriveStart:
      return "WEB_DRIVE_START";
    case MotionDiagnosticEvent::WebDriveHeartbeatAfterStop:
      return "WEB_DRIVE_HEARTBEAT_AFTER_STOP";
    case MotionDiagnosticEvent::WebStop:
      return "WEB_STOP";
    case MotionDiagnosticEvent::OutputsDisabled:
      return "OUTPUTS_DISABLED";
    case MotionDiagnosticEvent::CommandDeadmanExpired:
      return "COMMAND_DEADMAN_EXPIRED";
    case MotionDiagnosticEvent::ImuTurnStarted:
      return "IMU_TURN_STARTED";
    case MotionDiagnosticEvent::ImuTurnStopped:
      return "IMU_TURN_STOPPED";
    case MotionDiagnosticEvent::ImuTurnCompleted:
      return "IMU_TURN_COMPLETED";
    case MotionDiagnosticEvent::ImuTurnTimedOut:
      return "IMU_TURN_TIMED_OUT";
    case MotionDiagnosticEvent::ImuTurnFaulted:
      return "IMU_TURN_FAULTED";
  }
  return "UNKNOWN";
}

const char* motionDiagnosticTriggerName(
    const MotionDiagnosticTrigger trigger) {
  switch (trigger) {
    case MotionDiagnosticTrigger::None:
      return "NONE";
    case MotionDiagnosticTrigger::ManualFreeze:
      return "MANUAL_FREEZE";
    case MotionDiagnosticTrigger::CommandDeadmanExpired:
      return "COMMAND_DEADMAN_EXPIRED";
    case MotionDiagnosticTrigger::DriveHeartbeatAfterStop:
      return "DRIVE_HEARTBEAT_AFTER_STOP";
    case MotionDiagnosticTrigger::ImuTurnCompleted:
      return "IMU_TURN_COMPLETED";
    case MotionDiagnosticTrigger::ImuTurnTimedOut:
      return "IMU_TURN_TIMED_OUT";
    case MotionDiagnosticTrigger::ImuTurnFaulted:
      return "IMU_TURN_FAULTED";
  }
  return "UNKNOWN";
}

bool writeMotionDiagnosticsJson(const MotionDiagnostics& diagnostics,
                                char* output, const std::size_t capacity) {
  JsonWriter writer{output, capacity};
  writer.append(
      "{\"capture_id\":%u,\"reset_at_ms\":%u,\"frozen\":%s,"
      "\"trigger\":\"%s\",\"frozen_at_ms\":%u,\"sample_count\":%u,"
      "\"timing\":{\"last_loop_interval_us\":%u,"
      "\"maximum_loop_interval_us\":%u,\"maximum_loop_work_us\":%u,"
      "\"maximum_imu_update_us\":%u,\"maximum_web_handle_us\":%u,"
      "\"missed_deadline_count\":%u},"
      "\"web\":{\"drive_request_count\":%u,\"stop_request_count\":%u,"
      "\"drive_after_stop_count\":%u,\"last_drive_at_ms\":%u,"
      "\"last_stop_at_ms\":%u},"
      "\"command_order\":[\"FL\",\"FR\",\"BL\",\"BR\"],"
      "\"pwm_order\":[\"FL0\",\"FL1\",\"FR0\",\"FR1\"],\"samples\":[",
      static_cast<unsigned>(diagnostics.captureId()),
      static_cast<unsigned>(diagnostics.resetAtMs()),
      jsonBool(diagnostics.frozen()),
      motionDiagnosticTriggerName(diagnostics.trigger()),
      static_cast<unsigned>(diagnostics.frozenAtMs()),
      static_cast<unsigned>(diagnostics.sampleCount()),
      static_cast<unsigned>(diagnostics.lastLoopIntervalUs()),
      static_cast<unsigned>(diagnostics.maximumLoopIntervalUs()),
      static_cast<unsigned>(diagnostics.maximumLoopWorkUs()),
      static_cast<unsigned>(diagnostics.maximumImuUpdateUs()),
      static_cast<unsigned>(diagnostics.maximumWebHandleUs()),
      static_cast<unsigned>(diagnostics.missedDeadlineCount()),
      static_cast<unsigned>(diagnostics.webDriveRequestCount()),
      static_cast<unsigned>(diagnostics.webStopRequestCount()),
      static_cast<unsigned>(diagnostics.driveAfterStopCount()),
      static_cast<unsigned>(diagnostics.lastWebDriveAtMs()),
      static_cast<unsigned>(diagnostics.lastWebStopAtMs()));

  for (std::size_t index = 0U; index < diagnostics.sampleCount(); ++index) {
    const MotionDiagnosticSample& sample =
        diagnostics.sampleFromOldest(index);
    writer.append(
        "%s{\"t_ms\":%u,\"event\":\"%s\",\"mode\":\"%s\","
        "\"loop\":{\"interval_us\":%u,\"work_us\":%u,\"imu_us\":%u,"
        "\"web_us\":%u},"
        "\"cmd\":{\"requested\":[%d,%d,%d,%d],"
        "\"front_driver_desired\":[%d,%d],"
        "\"applied\":[%d,%d,%d,%d],"
        "\"front_expires_at_ms\":[%u,%u],"
        "\"pwm_readback\":[%u,%u,%u,%u]},"
        "\"rear\":{\"sequence\":%u,\"command_age_ms\":%u,"
        "\"status_age_ms\":%u,\"status_fresh\":%s,"
        "\"packet_error_count\":%u},"
        "\"control\":{\"deadman_armed\":%s,\"deadline_ms\":%u,"
        "\"last_command_age_ms\":%u},"
        "\"imu\":{\"turn_state\":\"%s\",\"heading_deg\":%.3f,"
        "\"target_deg\":%.3f,\"error_deg\":%.3f,"
        "\"yaw_rate_dps\":%.3f,\"rotation_command\":%.4f}}",
        index == 0U ? "" : ",",
        static_cast<unsigned>(sample.timestamp_ms),
        motionDiagnosticEventName(sample.event),
        robotTestModeName(sample.mode),
        static_cast<unsigned>(sample.loop_interval_us),
        static_cast<unsigned>(sample.loop_work_us),
        static_cast<unsigned>(sample.imu_update_us),
        static_cast<unsigned>(sample.web_handle_us),
        sample.requested_command_milli[0],
        sample.requested_command_milli[1],
        sample.requested_command_milli[2],
        sample.requested_command_milli[3],
        sample.front_driver_desired_command_milli[0],
        sample.front_driver_desired_command_milli[1],
        sample.applied_command_milli[0],
        sample.applied_command_milli[1],
        sample.applied_command_milli[2],
        sample.applied_command_milli[3],
        static_cast<unsigned>(sample.front_command_expires_at_ms[0]),
        static_cast<unsigned>(sample.front_command_expires_at_ms[1]),
        static_cast<unsigned>(sample.front_pwm_readback[0]),
        static_cast<unsigned>(sample.front_pwm_readback[1]),
        static_cast<unsigned>(sample.front_pwm_readback[2]),
        static_cast<unsigned>(sample.front_pwm_readback[3]),
        static_cast<unsigned>(sample.rear_sequence),
        static_cast<unsigned>(sample.rear_command_age_ms),
        static_cast<unsigned>(sample.esp1_status_age_ms),
        jsonBool(sample.esp1_status_fresh),
        static_cast<unsigned>(sample.esp1_packet_error_count),
        jsonBool(sample.command_deadman_armed),
        static_cast<unsigned>(sample.command_deadline_ms),
        static_cast<unsigned>(sample.last_command_age_ms),
        imuTurnStateName(sample.imu_turn_state),
        sample.heading_deg, sample.target_heading_deg,
        sample.angle_error_deg, sample.yaw_rate_dps,
        sample.rotation_command);
  }

  writer.append("]}");
  return writer.ok();
}

}  // namespace robot
