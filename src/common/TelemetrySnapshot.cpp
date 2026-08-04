#include "common/TelemetrySnapshot.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

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
    const int written = std::vsnprintf(output_ + used_, capacity_ - used_,
                                       format, args);
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

  bool appendEscaped(const char* text) {
    if (!append("\"")) {
      return false;
    }
    const char* cursor = text == nullptr ? "" : text;
    while (*cursor != '\0') {
      const char ch = *cursor++;
      if (ch == '"' || ch == '\\') {
        if (!append("\\%c", ch)) {
          return false;
        }
      } else if (static_cast<unsigned char>(ch) < 0x20U) {
        if (!append("_")) {
          return false;
        }
      } else if (!append("%c", ch)) {
        return false;
      }
    }
    return append("\"");
  }

  bool ok() const { return ok_; }

 private:
  char* output_{nullptr};
  std::size_t capacity_{0};
  std::size_t used_{0};
  bool ok_{true};
};

const char* jsonBool(const bool value) {
  return value ? "true" : "false";
}

const char* digitalLevelName(const int level) {
  if (level == 1) {
    return "HIGH";
  }
  if (level == 0) {
    return "LOW";
  }
  return "UNKNOWN";
}

void appendMotor(JsonWriter& writer, const char* name,
                 const MotorTelemetry& motor, const bool trailing_comma) {
  writer.append("\"%s\":{\"desired_command_milli\":%d,"
                "\"applied_command_milli\":%d,\"enabled\":%s,"
                "\"inverted\":%s,\"configured\":%s}%s",
                name, motor.desired_command_milli,
                motor.applied_command_milli, jsonBool(motor.enabled),
                jsonBool(motor.inverted), jsonBool(motor.configured),
                trailing_comma ? "," : "");
}

void appendClaw(JsonWriter& writer, const char* name,
                const ServoClawTelemetry& claw,
                const bool trailing_comma) {
  const char* pwm_backend =
      claw.mcpwm_unit >= 0
          ? "MCPWM"
          : (claw.ledc_channel >= 0 ? "LEDC" : "UNCONFIGURED");
  writer.append("\"%s\":{\"hardwareConfigured\":%s,"
                "\"pwmBackend\":\"%s\",\"gpio\":%d,"
                "\"ledcChannel\":%d,\"mcpwmUnit\":%d,"
                "\"mcpwmTimer\":%d,\"mcpwmGenerator\":%d,"
                "\"pwmFrequencyHz\":%u,"
                "\"mcpwmTimerResolutionHz\":%u,"
                "\"openConfigured\":%s,\"closedConfigured\":%s,"
                "\"outputEnabled\":%s,\"openAngleDeg\":%d,"
                "\"closedAngleDeg\":%d,\"commandedAngleDeg\":%d,"
                "\"commandedOpen\":%s}%s",
                name, jsonBool(claw.hardware_configured), pwm_backend,
                claw.gpio, claw.ledc_channel, claw.mcpwm_unit,
                claw.mcpwm_timer, claw.mcpwm_generator,
                static_cast<unsigned>(claw.pwm_frequency_hz),
                static_cast<unsigned>(claw.mcpwm_timer_resolution_hz),
                jsonBool(claw.open_configured),
                jsonBool(claw.closed_configured),
                jsonBool(claw.output_enabled), claw.open_angle_deg,
                claw.closed_angle_deg,
                claw.commanded_angle_deg, jsonBool(claw.commanded_open),
                trailing_comma ? "," : "");
}

void appendEvent(JsonWriter& writer, const EventRecord& event,
                 const bool trailing_comma) {
  writer.append("{\"timestamp_ms\":%u,\"severity\":\"%s\","
                "\"source\":\"%s\",\"message\":",
                static_cast<unsigned>(event.timestamp_ms),
                eventSeverityName(event.severity),
                eventSourceName(event.source));
  writer.appendEscaped(event.message);
  writer.append("}%s", trailing_comma ? "," : "");
}

}  // namespace

const char* faultCodeName(const FaultCode fault_code) {
  switch (fault_code) {
    case FaultCode::None:
      return "NONE";
    case FaultCode::CommunicationStale:
      return "COMMUNICATION_STALE";
    case FaultCode::InvalidCommand:
      return "INVALID_COMMAND";
    case FaultCode::LimitSwitchConflict:
      return "LIMIT_SWITCH_CONFLICT";
    case FaultCode::HardwareNotConfigured:
      return "HARDWARE_NOT_CONFIGURED";
    case FaultCode::SearchTimeout:
      return "SEARCH_TIMEOUT";
  }
  return "NONE";
}

bool writeTelemetryJson(const TelemetrySnapshot& snapshot, char* output,
                        const std::size_t output_size, const bool compact) {
  (void)compact;
  JsonWriter writer{output, output_size};
  writer.append("{");
  writer.append("\"uptime_ms\":%u,",
                static_cast<unsigned>(snapshot.uptime_ms));
  writer.append("\"current_mode\":\"%s\",",
                robotTestModeName(snapshot.current_mode));
  writer.append("\"previous_mode\":\"%s\",",
                robotTestModeName(snapshot.previous_mode));
  writer.append("\"enabled\":%s,", jsonBool(snapshot.enabled));
  writer.append("\"fault_active\":%s,", jsonBool(snapshot.fault_active));
  writer.append("\"fault_code\":\"%s\",", faultCodeName(snapshot.fault_code));
  writer.append("\"fault_message\":");
  writer.appendEscaped(snapshot.fault_message);
  writer.append(",");
  writer.append("\"last_command_age_ms\":%u,",
                static_cast<unsigned>(snapshot.last_command_age_ms));
  writer.append("\"deadman_remaining_ms\":%u,",
                static_cast<unsigned>(snapshot.deadman_remaining_ms));
  writer.append("\"wifi_clients\":%u,",
                static_cast<unsigned>(snapshot.wifi_clients));
  writer.append("\"ip_address\":");
  writer.appendEscaped(snapshot.ip_address);
  writer.append(",\"free_heap_bytes\":%u,",
                static_cast<unsigned>(snapshot.free_heap_bytes));
  writer.append("\"reset_reason\":");
  writer.appendEscaped(snapshot.reset_reason);

  writer.append(
      ",\"imu\":{\"configured\":%s,\"initialized\":%s,"
      "\"calibrated\":%s,\"healthy\":%s,\"data_fresh\":%s,"
      "\"acquisition_running\":%s,"
      "\"device_acknowledged\":%s,"
      "\"runtime_configuration_valid\":%s,"
      "\"register_reads_use_repeated_start\":%s,"
      "\"i2c_address\":%u,"
      "\"who_am_i\":%u,\"sda_gpio\":%d,\"scl_gpio\":%d,"
      "\"last_wire_status\":%d,\"initialization_error\":\"%s\","
      "\"last_read_failure_reason\":\"%s\","
      "\"disconnect_reason\":\"%s\","
      "\"last_disconnect_reason\":\"%s\","
      "\"raw_gyro_z\":%d,"
      "\"gyro_z_bias_dps\":%.5f,\"yaw_rate_dps\":%.5f,"
      "\"heading_deg\":%.5f,\"sample_age_ms\":%u,"
      "\"snapshot_age_ms\":%u,\"acquisition_duration_us\":%u,"
      "\"maximum_completed_acquisition_duration_us\":%u,"
      "\"total_acquisition_attempts\":%u,"
      "\"last_successful_read_us\":%u,"
      "\"last_sample_interval_us\":%u,\"last_read_failure_us\":%u,"
      "\"successful_read_count\":%u,"
      "\"failed_read_count\":%u,\"consecutive_failed_reads\":%u,"
      "\"disconnect_count\":%u,\"last_disconnect_at_ms\":%u",
      jsonBool(snapshot.imu.configured),
      jsonBool(snapshot.imu.initialized),
      jsonBool(snapshot.imu.calibrated),
      jsonBool(snapshot.imu.healthy),
      jsonBool(snapshot.imu.data_fresh),
      jsonBool(snapshot.imu.acquisition_running),
      jsonBool(snapshot.imu.device_acknowledged),
      jsonBool(snapshot.imu.runtime_configuration_valid),
      jsonBool(snapshot.imu.register_reads_use_repeated_start),
      static_cast<unsigned>(snapshot.imu.i2c_address),
      static_cast<unsigned>(snapshot.imu.who_am_i),
      snapshot.imu.sda_gpio, snapshot.imu.scl_gpio,
      snapshot.imu.last_wire_status,
      snapshot.imu.initialization_error,
      snapshot.imu.last_read_failure_reason,
      snapshot.imu.disconnect_reason,
      snapshot.imu.last_disconnect_reason,
      static_cast<int>(snapshot.imu.raw_gyro_z),
      snapshot.imu.gyro_z_bias_dps, snapshot.imu.yaw_rate_dps,
      snapshot.imu.heading_deg,
      static_cast<unsigned>(snapshot.imu.sample_age_ms),
      static_cast<unsigned>(snapshot.imu.snapshot_age_ms),
      static_cast<unsigned>(snapshot.imu.acquisition_duration_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_completed_acquisition_duration_us),
      static_cast<unsigned>(snapshot.imu.total_acquisition_attempts),
      static_cast<unsigned>(snapshot.imu.last_successful_read_us),
      static_cast<unsigned>(snapshot.imu.last_sample_interval_us),
      static_cast<unsigned>(snapshot.imu.last_read_failure_us),
      static_cast<unsigned>(snapshot.imu.successful_read_count),
      static_cast<unsigned>(snapshot.imu.failed_read_count),
      static_cast<unsigned>(snapshot.imu.consecutive_failed_reads),
      static_cast<unsigned>(snapshot.imu.disconnect_count),
      static_cast<unsigned>(snapshot.imu.last_disconnect_at_ms));

  writer.append(
      ",\"acquisition_timing\":{"
      "\"loop_interval_us\":%u,\"maximum_loop_interval_us\":%u,"
      "\"synchronization_duration_us\":%u,"
      "\"maximum_synchronization_duration_us\":%u,"
      "\"wire_lock_acquire_duration_us\":%u,"
      "\"maximum_wire_lock_acquire_duration_us\":%u,"
      "\"measurement_read_duration_us\":%u,"
      "\"maximum_measurement_read_duration_us\":%u,"
      "\"successful_read_to_publication_us\":%u,"
      "\"maximum_successful_read_to_publication_us\":%u,"
      "\"publication_queue_duration_us\":%u,"
      "\"maximum_publication_queue_duration_us\":%u,"
      "\"successful_sample_publication_gap_us\":%u,"
      "\"maximum_successful_sample_publication_gap_us\":%u,"
      "\"current_observed_publication_gap_us\":%u,"
      "\"maximum_observed_publication_gap_us\":%u,"
      "\"publication_sequence\":%u,"
      "\"successful_sample_sequence\":%u,"
      "\"delayed_iteration_count\":%u}}",
      static_cast<unsigned>(
          snapshot.imu.acquisition_loop_interval_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_acquisition_loop_interval_us),
      static_cast<unsigned>(
          snapshot.imu.synchronization_duration_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_synchronization_duration_us),
      static_cast<unsigned>(
          snapshot.imu.wire_lock_acquire_duration_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_wire_lock_acquire_duration_us),
      static_cast<unsigned>(
          snapshot.imu.measurement_read_duration_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_measurement_read_duration_us),
      static_cast<unsigned>(
          snapshot.imu.successful_read_to_publication_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_successful_read_to_publication_us),
      static_cast<unsigned>(
          snapshot.imu.publication_queue_duration_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_publication_queue_duration_us),
      static_cast<unsigned>(
          snapshot.imu.successful_sample_publication_gap_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_successful_sample_publication_gap_us),
      static_cast<unsigned>(
          snapshot.imu.current_observed_publication_gap_us),
      static_cast<unsigned>(
          snapshot.imu.maximum_observed_publication_gap_us),
      static_cast<unsigned>(snapshot.imu.publication_sequence),
      static_cast<unsigned>(
          snapshot.imu.successful_sample_sequence),
      static_cast<unsigned>(snapshot.imu.delayed_iteration_count));

  writer.append(
      ",\"imu_turn\":{\"configuration_valid\":%s,\"active\":%s,"
      "\"state\":\"%s\",\"fault_reason\":\"%s\","
      "\"maximum_rotation_duty\":%.5f,\"kp\":%.5f,\"kd\":%.5f,"
      "\"angle_tolerance_deg\":%.5f,"
      "\"maximum_finishing_yaw_rate_dps\":%.5f,"
      "\"settling_time_ms\":%u,\"timeout_ms\":%u,"
      "\"yaw_command_polarity\":%d,"
      "\"start_heading_deg\":%.5f,\"current_heading_deg\":%.5f,"
      "\"target_heading_deg\":%.5f,\"relative_angle_deg\":%.5f,"
      "\"angle_error_deg\":%.5f,\"yaw_rate_dps\":%.5f,"
      "\"proportional_term\":%.5f,\"damping_term\":%.5f,"
      "\"rotation_command\":%.5f,\"elapsed_ms\":%u,"
      "\"settling_elapsed_ms\":%u",
      jsonBool(snapshot.imu_turn.configuration_valid),
      jsonBool(snapshot.imu_turn.active),
      imuTurnStateName(snapshot.imu_turn.state),
      imuTurnFaultReasonName(snapshot.imu_turn.fault_reason),
      snapshot.imu_turn.maximum_rotation_duty,
      snapshot.imu_turn.kp, snapshot.imu_turn.kd,
      snapshot.imu_turn.angle_tolerance_deg,
      snapshot.imu_turn.maximum_finishing_yaw_rate_dps,
      static_cast<unsigned>(snapshot.imu_turn.settling_time_ms),
      static_cast<unsigned>(snapshot.imu_turn.timeout_ms),
      snapshot.imu_turn.yaw_command_polarity,
      snapshot.imu_turn.start_heading_deg,
      snapshot.imu_turn.current_heading_deg,
      snapshot.imu_turn.target_heading_deg,
      snapshot.imu_turn.relative_angle_deg,
      snapshot.imu_turn.angle_error_deg,
      snapshot.imu_turn.yaw_rate_dps,
      snapshot.imu_turn.proportional_term,
      snapshot.imu_turn.damping_term,
      snapshot.imu_turn.rotation_command,
      static_cast<unsigned>(snapshot.imu_turn.elapsed_ms),
      static_cast<unsigned>(snapshot.imu_turn.settling_elapsed_ms));

  writer.append(
      ",\"availability_fault\":{"
      "\"capture_valid\":%s,\"latched\":%s,"
      "\"imu_currently_available\":%s,"
      "\"origin\":\"%s\",\"reason\":\"%s\","
      "\"configured\":%s,\"initialized\":%s,\"calibrated\":%s,"
      "\"healthy\":%s,\"sample_valid\":%s,\"data_fresh\":%s,"
      "\"acquisition_running\":%s,"
      "\"shared_snapshot_available\":%s,"
      "\"newest_snapshot_available\":%s,"
      "\"cached_snapshot_matches_newest\":%s,"
      "\"front_left_configured\":%s,\"front_right_configured\":%s,"
      "\"rear_link_configured\":%s,\"rear_status_available\":%s,"
      "\"rear_status_fresh\":%s,"
      "\"evaluated_at_us\":%u,\"evaluated_at_ms\":%u,"
      "\"published_at_us\":%u,\"last_successful_read_us\":%u,"
      "\"sample_age_us\":%u,\"snapshot_age_us\":%u,"
      "\"freshness_timeout_us\":%u,"
      "\"cached_snapshot_sequence\":%u,"
      "\"newest_snapshot_sequence\":%u,"
      "\"cached_successful_sample_sequence\":%u,"
      "\"newest_successful_sample_sequence\":%u,"
      "\"cached_snapshot_fetched_at_us\":%u,"
      "\"cached_snapshot_fetch_to_gate_us\":%u,"
      "\"successful_sample_publication_gap_us\":%u,"
      "\"maximum_successful_sample_publication_gap_us\":%u,"
      "\"current_observed_publication_gap_us\":%u,"
      "\"maximum_observed_publication_gap_us\":%u,"
      "\"rear_last_status_received_at_ms\":%u,"
      "\"rear_status_age_ms\":%u}}",
      jsonBool(snapshot.imu_turn.availability_fault_capture_valid),
      jsonBool(snapshot.imu_turn.availability_fault_latched),
      jsonBool(snapshot.imu_turn.imu_currently_available),
      snapshot.imu_turn.availability_fault_origin,
      snapshot.imu_turn.captured_availability_reason,
      jsonBool(snapshot.imu_turn.captured_configured),
      jsonBool(snapshot.imu_turn.captured_initialized),
      jsonBool(snapshot.imu_turn.captured_calibrated),
      jsonBool(snapshot.imu_turn.captured_healthy),
      jsonBool(snapshot.imu_turn.captured_sample_valid),
      jsonBool(snapshot.imu_turn.captured_data_fresh),
      jsonBool(snapshot.imu_turn.captured_acquisition_running),
      jsonBool(
          snapshot.imu_turn.captured_shared_snapshot_available),
      jsonBool(
          snapshot.imu_turn.captured_newest_snapshot_available),
      jsonBool(
          snapshot.imu_turn.captured_cached_snapshot_matches_newest),
      jsonBool(snapshot.imu_turn.captured_front_left_configured),
      jsonBool(snapshot.imu_turn.captured_front_right_configured),
      jsonBool(snapshot.imu_turn.captured_rear_link_configured),
      jsonBool(snapshot.imu_turn.captured_rear_status_available),
      jsonBool(snapshot.imu_turn.captured_rear_status_fresh),
      static_cast<unsigned>(
          snapshot.imu_turn.availability_evaluated_at_us),
      static_cast<unsigned>(
          snapshot.imu_turn.availability_evaluated_at_ms),
      static_cast<unsigned>(snapshot.imu_turn.captured_published_at_us),
      static_cast<unsigned>(
          snapshot.imu_turn.captured_last_successful_read_us),
      static_cast<unsigned>(snapshot.imu_turn.captured_sample_age_us),
      static_cast<unsigned>(snapshot.imu_turn.captured_snapshot_age_us),
      static_cast<unsigned>(
          snapshot.imu_turn.captured_freshness_timeout_us),
      static_cast<unsigned>(
          snapshot.imu_turn.captured_cached_snapshot_sequence),
      static_cast<unsigned>(
          snapshot.imu_turn.captured_newest_snapshot_sequence),
      static_cast<unsigned>(
          snapshot.imu_turn
              .captured_cached_successful_sample_sequence),
      static_cast<unsigned>(
          snapshot.imu_turn
              .captured_newest_successful_sample_sequence),
      static_cast<unsigned>(
          snapshot.imu_turn.captured_cached_snapshot_fetched_at_us),
      static_cast<unsigned>(
          snapshot.imu_turn
              .captured_cached_snapshot_fetch_to_gate_us),
      static_cast<unsigned>(
          snapshot.imu_turn
              .captured_successful_sample_publication_gap_us),
      static_cast<unsigned>(
          snapshot.imu_turn
              .captured_maximum_successful_sample_publication_gap_us),
      static_cast<unsigned>(
          snapshot.imu_turn
              .captured_current_observed_publication_gap_us),
      static_cast<unsigned>(
          snapshot.imu_turn
              .captured_maximum_observed_publication_gap_us),
      static_cast<unsigned>(
          snapshot.imu_turn.captured_rear_last_status_received_at_ms),
      static_cast<unsigned>(
          snapshot.imu_turn.captured_rear_status_age_ms));

  writer.append(
      ",\"imu_heading_hold\":{\"configuration_valid\":%s,"
      "\"active\":%s,\"state\":\"%s\",\"fault_reason\":\"%s\","
      "\"maximum_strafe_duty\":%.5f,\"kp\":%.5f,\"kd\":%.5f,"
      "\"maximum_yaw_correction_duty\":%.5f,"
      "\"yaw_command_polarity\":%d,"
      "\"start_heading_deg\":%.5f,\"current_heading_deg\":%.5f,"
      "\"target_heading_deg\":%.5f,\"angle_error_deg\":%.5f,"
      "\"yaw_rate_dps\":%.5f,\"proportional_term\":%.5f,"
      "\"damping_term\":%.5f,\"yaw_correction_duty\":%.5f,"
      "\"lateral_direction\":%d,\"elapsed_ms\":%u}",
      jsonBool(snapshot.imu_heading_hold.configuration_valid),
      jsonBool(snapshot.imu_heading_hold.active),
      imuHeadingHoldStateName(snapshot.imu_heading_hold.state),
      imuHeadingHoldFaultReasonName(
          snapshot.imu_heading_hold.fault_reason),
      snapshot.imu_heading_hold.maximum_strafe_duty,
      snapshot.imu_heading_hold.kp,
      snapshot.imu_heading_hold.kd,
      snapshot.imu_heading_hold.maximum_yaw_correction_duty,
      snapshot.imu_heading_hold.yaw_command_polarity,
      snapshot.imu_heading_hold.start_heading_deg,
      snapshot.imu_heading_hold.current_heading_deg,
      snapshot.imu_heading_hold.target_heading_deg,
      snapshot.imu_heading_hold.angle_error_deg,
      snapshot.imu_heading_hold.yaw_rate_dps,
      snapshot.imu_heading_hold.proportional_term,
      snapshot.imu_heading_hold.damping_term,
      snapshot.imu_heading_hold.yaw_correction_duty,
      snapshot.imu_heading_hold.lateral_direction,
      static_cast<unsigned>(snapshot.imu_heading_hold.elapsed_ms));

  writer.append(
      ",\"imu_recovery\":{\"turn_paused\":%s,"
      "\"strafe_paused\":%s,"
      "\"turn_saved_heading_deg\":%.5f,"
      "\"strafe_saved_heading_deg\":%.5f,"
      "\"turn_pause_elapsed_ms\":%u,"
      "\"strafe_pause_elapsed_ms\":%u,"
      "\"maximum_pause_ms\":%u,"
      "\"consecutive_fresh_samples_required\":%u,"
      "\"turn_consecutive_fresh_samples\":%u,"
      "\"strafe_consecutive_fresh_samples\":%u,"
      "\"turn_pause_count\":%u,\"strafe_pause_count\":%u,"
      "\"total_paused_ms\":%u}",
      jsonBool(snapshot.imu_recovery.turn_paused),
      jsonBool(snapshot.imu_recovery.strafe_paused),
      snapshot.imu_recovery.turn_saved_heading_deg,
      snapshot.imu_recovery.strafe_saved_heading_deg,
      static_cast<unsigned>(
          snapshot.imu_recovery.turn_pause_elapsed_ms),
      static_cast<unsigned>(
          snapshot.imu_recovery.strafe_pause_elapsed_ms),
      static_cast<unsigned>(
          snapshot.imu_recovery.maximum_pause_ms),
      static_cast<unsigned>(
          snapshot.imu_recovery
              .consecutive_fresh_samples_required),
      static_cast<unsigned>(
          snapshot.imu_recovery
              .turn_consecutive_fresh_samples),
      static_cast<unsigned>(
          snapshot.imu_recovery
              .strafe_consecutive_fresh_samples),
      static_cast<unsigned>(
          snapshot.imu_recovery.turn_pause_count),
      static_cast<unsigned>(
          snapshot.imu_recovery.strafe_pause_count),
      static_cast<unsigned>(
          snapshot.imu_recovery.total_paused_ms));

  writer.append(",\"line\":{\"lsfl_raw_level\":%d,\"lsfr_raw_level\":%d,"
                "\"lss_raw_level\":%d,\"lss2_raw_level\":%d,"
                "\"lss3_raw_level\":%d,"
                "\"lsfl_level\":\"%s\",\"lsfr_level\":\"%s\","
                "\"lss_level\":\"%s\",\"lss2_level\":\"%s\","
                "\"lss3_level\":\"%s\","
                "\"lsfl_black\":%s,\"lsfr_black\":%s,"
                "\"lss_black\":%s,\"lss_configured\":%s,"
                "\"lss2_black\":%s,\"lss2_configured\":%s,"
                "\"lss3_black\":%s,\"lss3_configured\":%s,"
                "\"line_error\":%d,"
                "\"line_visible\":%s,\"has_history\":%s,"
                "\"hasHistory\":%s,"
                "\"last_known_line_side\":%d,"
                "\"line_follower_enabled\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                snapshot.lss_raw_level, snapshot.lss2_raw_level,
                snapshot.lss3_raw_level,
                digitalLevelName(snapshot.lsfl_raw_level),
                digitalLevelName(snapshot.lsfr_raw_level),
                digitalLevelName(snapshot.lss_raw_level),
                digitalLevelName(snapshot.lss2_raw_level),
                digitalLevelName(snapshot.lss3_raw_level),
                jsonBool(snapshot.lsfl_black), jsonBool(snapshot.lsfr_black),
                jsonBool(snapshot.lss_black),
                jsonBool(snapshot.lss_configured),
                jsonBool(snapshot.lss2_black),
                jsonBool(snapshot.lss2_configured),
                jsonBool(snapshot.lss3_black),
                jsonBool(snapshot.lss3_configured),
                static_cast<int>(snapshot.line_error),
                jsonBool(snapshot.line_visible),
                jsonBool(snapshot.line_has_history),
                jsonBool(snapshot.line_has_history),
                static_cast<int>(snapshot.last_known_line_side),
                jsonBool(snapshot.line_follower_enabled));

  writer.append(
      ",\"rear_line\":{\"lsbl_raw_level\":%d,\"lsbr_raw_level\":%d,"
      "\"lsbl_level\":\"%s\",\"lsbr_level\":\"%s\","
      "\"lsbl_black\":%s,\"lsbr_black\":%s,\"configured\":%s,"
      "\"data_fresh\":%s,\"sequence\":%u,\"sample_age_ms\":%u,"
      "\"captured_at_ms\":%u,\"line_error\":%d,"
      "\"line_visible\":%s,\"has_history\":%s,\"hasHistory\":%s,"
      "\"last_known_line_side\":%d,\"line_follower_enabled\":%s,"
      "\"logical_left_source\":\"LSBR\","
      "\"logical_right_source\":\"LSBL\","
      "\"logical_left_black\":%s,\"logical_right_black\":%s}",
      snapshot.lsbl_raw_level, snapshot.lsbr_raw_level,
      digitalLevelName(snapshot.lsbl_raw_level),
      digitalLevelName(snapshot.lsbr_raw_level),
      jsonBool(snapshot.lsbl_black), jsonBool(snapshot.lsbr_black),
      jsonBool(snapshot.rear_line_configured),
      jsonBool(snapshot.rear_line_data_fresh),
      static_cast<unsigned>(snapshot.rear_line_sequence),
      static_cast<unsigned>(snapshot.rear_line_sample_age_ms),
      static_cast<unsigned>(snapshot.rear_line_captured_at_ms),
      static_cast<int>(snapshot.rear_line_error),
      jsonBool(snapshot.rear_line_visible),
      jsonBool(snapshot.rear_line_has_history),
      jsonBool(snapshot.rear_line_has_history),
      static_cast<int>(snapshot.rear_last_known_line_side),
      jsonBool(snapshot.rear_line_follower_enabled),
      jsonBool(snapshot.rear_logical_left_black),
      jsonBool(snapshot.rear_logical_right_black));

  writer.append(",\"pid\":{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f,"
                "\"baseDuty\":%.5f,\"maxDuty\":%.5f,"
                "\"maximumDuty\":%.5f,\"maxCorrection\":%.5f,"
                "\"maximumCorrection\":%.5f,\"integralLimit\":%.5f,"
                "\"derivativeLimit\":%.5f,"
                "\"derivativeFilterAlpha\":%.5f,"
                "\"steeringPolarity\":%d,\"controlPeriodMs\":%u,"
                "\"remoteCommandTimeoutMs\":%u,\"telemetryEnabled\":%s,"
                "\"p_term\":%.5f,\"i_term\":%.5f,\"d_term\":%.5f,"
                "\"correction\":%.5f}",
                snapshot.kp, snapshot.ki, snapshot.kd, snapshot.base_duty,
                snapshot.maximum_duty, snapshot.maximum_duty,
                snapshot.maximum_correction, snapshot.maximum_correction,
                snapshot.integral_limit, snapshot.derivative_limit,
                snapshot.derivative_filter_alpha, snapshot.steering_polarity,
                static_cast<unsigned>(snapshot.control_period_ms),
                static_cast<unsigned>(snapshot.remote_command_timeout_ms),
                jsonBool(snapshot.line_telemetry_enabled), snapshot.pid_p_term,
                snapshot.pid_i_term, snapshot.pid_d_term,
                snapshot.pid_correction);

  writer.append(
      ",\"rear_pid\":{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f,"
      "\"baseDuty\":%.5f,\"effectiveBaseDuty\":%.5f,"
      "\"maxDuty\":%.5f,\"maximumDuty\":%.5f,"
      "\"maxCorrection\":%.5f,\"maximumCorrection\":%.5f,"
      "\"integralLimit\":%.5f,\"derivativeLimit\":%.5f,"
      "\"derivativeFilterAlpha\":%.5f,\"steeringPolarity\":%d,"
      "\"controlPeriodMs\":%u,\"remoteCommandTimeoutMs\":%u,"
      "\"telemetryEnabled\":%s,\"p_term\":%.5f,\"i_term\":%.5f,"
      "\"d_term\":%.5f,\"correction\":%.5f}",
      snapshot.rear_kp, snapshot.rear_ki, snapshot.rear_kd,
      snapshot.rear_base_duty, snapshot.rear_effective_base_duty,
      snapshot.rear_maximum_duty, snapshot.rear_maximum_duty,
      snapshot.rear_maximum_correction,
      snapshot.rear_maximum_correction, snapshot.rear_integral_limit,
      snapshot.rear_derivative_limit,
      snapshot.rear_derivative_filter_alpha,
      snapshot.rear_steering_polarity,
      static_cast<unsigned>(snapshot.rear_control_period_ms),
      static_cast<unsigned>(snapshot.rear_remote_command_timeout_ms),
      jsonBool(snapshot.rear_line_telemetry_enabled),
      snapshot.rear_pid_p_term, snapshot.rear_pid_i_term,
      snapshot.rear_pid_d_term, snapshot.rear_pid_correction);

  writer.append(",\"autonomous_state\":\"%s\",",
                solarPanelAutonomyStateName(snapshot.autonomous_state));
  writer.append("\"autonomous\":{\"state\":\"%s\","
                "\"time_in_state_ms\":%u,\"fault_reason\":\"%s\","
                "\"ir_raw_amplitude\":%u,"
                "\"ir_filtered_amplitude\":%.2f,"
                "\"ir_detection_threshold\":%u,"
                "\"ir_release_threshold\":%u,"
                "\"ir_detection_threshold_1khz\":%u,"
                "\"ir_release_threshold_1khz\":%u,"
                "\"ir_detection_threshold_10khz\":%u,"
                "\"ir_release_threshold_10khz\":%u,"
                "\"confirmation_progress_ms\":%u,"
                "\"confirmation_time_ms\":%u,"
                "\"filter_alpha\":%.5f,"
                "\"confirmation_active\":%s,"
                "\"beacon_detected\":%s,"
                "\"ignore_after_start_ms\":%u,"
                "\"search_timeout_ms\":%u,"
                "\"start_base_duty\":%.5f,"
                "\"slow_after_ms\":%u,"
                "\"slow_base_duty\":%.5f,"
                "\"slow_mode_active\":%s,"
                "\"contact_timeout_ms\":%u,"
                "\"strafe_duty\":%.5f,"
                "\"strafe_start_delay_ms\":%u,"
                "\"retry_strafe_left_duration_ms\":%u,"
                "\"retry_forward_duration_ms\":%u,"
                "\"retry_forward_duty\":%.5f,"
                "\"retry_strafe_timeout_ms\":%u,"
                "\"post_contact_forward_duration_ms\":%u,"
                "\"line_reacquire_strafe_duty\":%.5f,"
                "\"post_contact_forward_start_delay_ms\":%u,"
                "\"line_reacquire_strafe_start_delay_ms\":%u,"
                "\"post_contact_forward_duty\":%.5f,"
                "\"limit_switches\":{\"configured\":%s,"
                "\"back_right_high\":%s,\"front_right_high\":%s,"
                "\"back_right_hit\":%s,\"front_right_hit\":%s,"
                "\"all_hit\":%s}}",
                solarPanelAutonomyStateName(snapshot.autonomous_state),
                static_cast<unsigned>(
                    snapshot.autonomous_time_in_state_ms),
                solarPanelFaultReasonName(
                    snapshot.autonomous_fault_reason),
                static_cast<unsigned>(
                    snapshot.solar_ir_raw_amplitude),
                snapshot.solar_ir_filtered_amplitude,
                static_cast<unsigned>(
                    snapshot.solar_ir_detection_threshold),
                static_cast<unsigned>(
                    snapshot.solar_ir_release_threshold),
                static_cast<unsigned>(
                    snapshot.solar_ir_detection_threshold_1khz),
                static_cast<unsigned>(
                    snapshot.solar_ir_release_threshold_1khz),
                static_cast<unsigned>(
                    snapshot.solar_ir_detection_threshold_10khz),
                static_cast<unsigned>(
                    snapshot.solar_ir_release_threshold_10khz),
                static_cast<unsigned>(
                    snapshot.solar_ir_confirmation_progress_ms),
                static_cast<unsigned>(
                    snapshot.solar_ir_confirmation_time_ms),
                snapshot.solar_ir_filter_alpha,
                jsonBool(snapshot.solar_ir_confirmation_active),
                jsonBool(snapshot.solar_beacon_confirmed),
                static_cast<unsigned>(
                    snapshot.solar_ir_ignore_after_start_ms),
                static_cast<unsigned>(
                    snapshot.solar_search_timeout_ms),
                snapshot.solar_start_base_duty,
                static_cast<unsigned>(
                    snapshot.solar_slow_after_ms),
                snapshot.solar_slow_base_duty,
                jsonBool(snapshot.solar_slow_mode_active),
                static_cast<unsigned>(
                    snapshot.solar_contact_timeout_ms),
                snapshot.solar_contact_strafe_duty,
                static_cast<unsigned>(
                    snapshot.solar_strafe_start_delay_ms),
                static_cast<unsigned>(
                    snapshot.solar_retry_strafe_left_duration_ms),
                static_cast<unsigned>(
                    snapshot.solar_retry_forward_duration_ms),
                snapshot.solar_retry_forward_duty,
                static_cast<unsigned>(
                    snapshot.solar_retry_strafe_timeout_ms),
                static_cast<unsigned>(
                    snapshot.solar_post_contact_forward_duration_ms),
                snapshot.solar_line_reacquire_strafe_duty,
                static_cast<unsigned>(
                    snapshot.solar_post_contact_forward_start_delay_ms),
                static_cast<unsigned>(
                    snapshot.solar_line_reacquire_strafe_start_delay_ms),
                snapshot.solar_post_contact_forward_duty,
                jsonBool(
                    snapshot.solar_panel_limit_switches_configured),
                jsonBool(snapshot.solar_limit_back_right_high),
                jsonBool(snapshot.solar_limit_front_right_high),
                jsonBool(snapshot.solar_limit_back_right_hit),
                jsonBool(snapshot.solar_limit_front_right_hit),
                jsonBool(snapshot.solar_limit_all_hit));

  writer.append(",\"solarLimitSwitches\":{\"configured\":%s,"
                "\"backRightHigh\":%s,\"frontRightHigh\":%s,"
                "\"backRightHit\":%s,\"frontRightHit\":%s,"
                "\"allHit\":%s}",
                jsonBool(
                    snapshot.solar_panel_limit_switches_configured),
                jsonBool(snapshot.solar_limit_back_right_high),
                jsonBool(snapshot.solar_limit_front_right_high),
                jsonBool(snapshot.solar_limit_back_right_hit),
                jsonBool(snapshot.solar_limit_front_right_hit),
                jsonBool(snapshot.solar_limit_all_hit));

  writer.append(
      ",\"solar_strafe_speeds\":{\"initial_right_duty\":%.5f,"
      "\"retry_left_duty\":%.5f,\"retry_right_duty\":%.5f,"
      "\"rear_line_strafe_duty\":%.5f,"
      "\"backward_pid_duration_ms\":%u}",
      snapshot.solar_contact_strafe_duty,
      snapshot.solar_retry_left_strafe_duty,
      snapshot.solar_retry_right_strafe_duty,
      snapshot.solar_line_reacquire_strafe_duty,
      static_cast<unsigned>(
          snapshot.solar_rear_line_follow_duration_ms));

  writer.append(
      ",\"habitat_pieces\":{\"state\":\"%s\","
      "\"stop_reason\":\"%s\",\"time_in_state_ms\":%u,"
      "\"line_follow_duty\":%.5f,\"lss2_detection_delay_ms\":%u,"
      "\"lss2_detection_remaining_ms\":%u,"
      "\"run_timeout_ms\":%u,\"run_elapsed_ms\":%u,"
      "\"timeout_remaining_ms\":%u,"
      "\"reverse_duty\":%.5f,\"reverse_duration_ms\":%u,"
      "\"reverse_elapsed_ms\":%u,\"reverse_remaining_ms\":%u,"
      "\"distance_strafe_direction\":\"%s\","
      "\"distance_threshold_mm\":%u,"
      "\"distance_zone_target_count\":%u,"
      "\"distance_strafe_duty\":%.5f,"
      "\"distance_strafe_timeout_ms\":%u,"
      "\"distance_strafe_elapsed_ms\":%u,"
      "\"distance_strafe_remaining_ms\":%u,"
      "\"post_count_stop_delay_ms\":%u,"
      "\"post_count_stop_elapsed_ms\":%u,"
      "\"post_count_stop_remaining_ms\":%u,"
      "\"exit_strafe_pulse_ms\":%u,"
      "\"exit_strafe_pulse_elapsed_ms\":%u,"
      "\"exit_strafe_pulse_remaining_ms\":%u,"
      "\"distance_mm\":%u,\"distance_zone_count\":%u,"
      "\"distance_exit_pulse_count\":%u,"
      "\"configuration_valid\":%s,\"start_ready\":%s,"
      "\"lss2_configured\":%s,\"lss2_data_fresh\":%s,"
      "\"lss3_configured\":%s,\"lss3_data_fresh\":%s,"
      "\"lss2_detection_armed\":%s,\"lss2_black\":%s,"
      "\"lss3_black\":%s,\"lss2_latched\":%s,"
      "\"lss3_latched\":%s,"
      "\"approach_limit_configured\":%s,"
      "\"approach_limit_raw_level\":%d,"
      "\"approach_limit_active\":%s,"
      "\"should_stop\":%s,"
      "\"target_reached\":%s,\"line_following\":%s,"
      "\"side_line_aligning\":%s,"
      "\"left_side_driving\":%s,\"right_side_driving\":%s,"
      "\"reversing\":%s,\"distance_strafing\":%s,"
      "\"post_count_waiting\":%s,"
      "\"exit_strafe_pulsing\":%s,"
      "\"exit_distance_checking\":%s,"
      "\"distance_measurement_available\":%s,"
      "\"distance_substituted_no_target\":%s,"
      "\"distance_sample_new\":%s,"
      "\"distance_zone_active\":%s,"
      "\"distance_zone_entered\":%s,"
      "\"distance_exit_above_threshold\":%s,"
      "\"timed_out\":%s,"
      "\"slide_down_speed_steps_per_second\":%u,"
      "\"slide_down_timeout_ms\":%u,"
      "\"slide_down_elapsed_ms\":%u,"
      "\"approach_forward_duty\":%.5f,"
      "\"approach_timeout_ms\":%u,"
      "\"approach_elapsed_ms\":%u,"
      "\"pre_lift_reverse_duration_ms\":%u,"
      "\"pre_lift_reverse_elapsed_ms\":%u,"
      "\"lift_steps\":%u,"
      "\"lift_speed_steps_per_second\":%u,"
      "\"lift_timeout_ms\":%u,"
      "\"lift_elapsed_ms\":%u,"
      "\"lift_start_delay_ms\":%u,"
      "\"lift_start_delay_elapsed_ms\":%u,"
      "\"post_pickup_reverse_duty\":%.5f,"
      "\"post_pickup_reverse_duration_ms\":%u,"
      "\"post_pickup_reverse_elapsed_ms\":%u,"
      "\"rear_line_reacquire_duty\":%.5f,"
      "\"rear_line_reacquire_timeout_ms\":%u,"
      "\"rear_line_reacquire_elapsed_ms\":%u,"
      "\"lowering_slide\":%s,\"approaching_piece\":%s,"
      "\"pre_lift_reversing\":%s,"
      "\"lifting_slide\":%s,"
      "\"lift_start_waiting\":%s,"
      "\"post_pickup_reversing\":%s,"
      "\"reacquiring_rear_line\":%s,"
      "\"waiting_for_lift\":%s,"
      "\"approach_limit_reached\":%s,"
      "\"lift_complete\":%s,"
      "\"rear_line_detected\":%s}",
      habitatPiecesStateName(snapshot.habitat_pieces_state),
      habitatPiecesStopReasonName(snapshot.habitat_pieces_stop_reason),
      static_cast<unsigned>(snapshot.habitat_pieces_time_in_state_ms),
      snapshot.habitat_pieces_line_follow_duty,
      static_cast<unsigned>(
          snapshot.habitat_pieces_lss2_detection_delay_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_lss2_detection_remaining_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_run_timeout_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_run_elapsed_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_timeout_remaining_ms),
      snapshot.habitat_pieces_reverse_duty,
      static_cast<unsigned>(snapshot.habitat_pieces_reverse_duration_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_reverse_elapsed_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_reverse_remaining_ms),
      habitatPiecesStrafeDirectionName(
          snapshot.habitat_pieces_distance_strafe_direction),
      static_cast<unsigned>(
          snapshot.habitat_pieces_distance_threshold_mm),
      static_cast<unsigned>(
          snapshot.habitat_pieces_distance_zone_target_count),
      snapshot.habitat_pieces_distance_strafe_duty,
      static_cast<unsigned>(
          snapshot.habitat_pieces_distance_strafe_timeout_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_distance_strafe_elapsed_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_distance_strafe_remaining_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_post_count_stop_delay_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_post_count_stop_elapsed_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_post_count_stop_remaining_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_exit_strafe_pulse_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_exit_strafe_pulse_elapsed_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_exit_strafe_pulse_remaining_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_distance_mm),
      static_cast<unsigned>(
          snapshot.habitat_pieces_distance_zone_count),
      static_cast<unsigned>(
          snapshot.habitat_pieces_distance_exit_pulse_count),
      jsonBool(snapshot.habitat_pieces_configuration_valid),
      jsonBool(snapshot.habitat_pieces_start_ready),
      jsonBool(snapshot.habitat_pieces_lss2_configured),
      jsonBool(snapshot.habitat_pieces_lss2_data_fresh),
      jsonBool(snapshot.habitat_pieces_lss3_configured),
      jsonBool(snapshot.habitat_pieces_lss3_data_fresh),
      jsonBool(snapshot.habitat_pieces_lss2_detection_armed),
      jsonBool(snapshot.habitat_pieces_lss2_black),
      jsonBool(snapshot.habitat_pieces_lss3_black),
      jsonBool(snapshot.habitat_pieces_lss2_latched),
      jsonBool(snapshot.habitat_pieces_lss3_latched),
      jsonBool(snapshot.habitat_pieces_approach_limit_configured),
      snapshot.habitat_pieces_approach_limit_raw_level,
      jsonBool(snapshot.habitat_pieces_approach_limit_active),
      jsonBool(snapshot.habitat_pieces_should_stop),
      jsonBool(snapshot.habitat_pieces_target_reached),
      jsonBool(snapshot.habitat_pieces_line_following),
      jsonBool(snapshot.habitat_pieces_side_line_aligning),
      jsonBool(snapshot.habitat_pieces_left_side_driving),
      jsonBool(snapshot.habitat_pieces_right_side_driving),
      jsonBool(snapshot.habitat_pieces_reversing),
      jsonBool(snapshot.habitat_pieces_distance_strafing),
      jsonBool(snapshot.habitat_pieces_post_count_waiting),
      jsonBool(snapshot.habitat_pieces_exit_strafe_pulsing),
      jsonBool(snapshot.habitat_pieces_exit_distance_checking),
      jsonBool(
          snapshot.habitat_pieces_distance_measurement_available),
      jsonBool(
          snapshot.habitat_pieces_distance_substituted_no_target),
      jsonBool(snapshot.habitat_pieces_distance_sample_new),
      jsonBool(snapshot.habitat_pieces_distance_zone_active),
      jsonBool(snapshot.habitat_pieces_distance_zone_entered),
      jsonBool(
          snapshot.habitat_pieces_distance_exit_above_threshold),
      jsonBool(snapshot.habitat_pieces_timed_out),
      static_cast<unsigned>(
          snapshot.habitat_pieces_slide_down_speed_steps_per_second),
      static_cast<unsigned>(snapshot.habitat_pieces_slide_down_timeout_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_slide_down_elapsed_ms),
      snapshot.habitat_pieces_approach_forward_duty,
      static_cast<unsigned>(snapshot.habitat_pieces_approach_timeout_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_approach_elapsed_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_pre_lift_reverse_duration_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_pre_lift_reverse_elapsed_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_lift_steps),
      static_cast<unsigned>(
          snapshot.habitat_pieces_lift_speed_steps_per_second),
      static_cast<unsigned>(snapshot.habitat_pieces_lift_timeout_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_lift_elapsed_ms),
      static_cast<unsigned>(snapshot.habitat_pieces_lift_start_delay_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_lift_start_delay_elapsed_ms),
      snapshot.habitat_pieces_post_pickup_reverse_duty,
      static_cast<unsigned>(
          snapshot.habitat_pieces_post_pickup_reverse_duration_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_post_pickup_reverse_elapsed_ms),
      snapshot.habitat_pieces_rear_line_reacquire_duty,
      static_cast<unsigned>(
          snapshot.habitat_pieces_rear_line_reacquire_timeout_ms),
      static_cast<unsigned>(
          snapshot.habitat_pieces_rear_line_reacquire_elapsed_ms),
      jsonBool(snapshot.habitat_pieces_lowering_slide),
      jsonBool(snapshot.habitat_pieces_approaching_piece),
      jsonBool(snapshot.habitat_pieces_pre_lift_reversing),
      jsonBool(snapshot.habitat_pieces_lifting_slide),
      jsonBool(snapshot.habitat_pieces_lift_start_waiting),
      jsonBool(snapshot.habitat_pieces_post_pickup_reversing),
      jsonBool(snapshot.habitat_pieces_reacquiring_rear_line),
      jsonBool(snapshot.habitat_pieces_waiting_for_lift),
      jsonBool(snapshot.habitat_pieces_approach_limit_reached),
      jsonBool(snapshot.habitat_pieces_lift_complete),
      jsonBool(snapshot.habitat_pieces_rear_line_detected));

  const HabitatPlacementConfig& habitat_placement =
      snapshot.habitat_placement_config;
  writer.append(
      ",\"habitat_placement\":{\"state\":\"%s\","
      "\"fault_reason\":\"%s\",\"time_in_state_ms\":%u,"
      "\"configuration_valid\":%s,\"start_ready\":%s,"
      "\"initial_heading_captured\":%s,"
      "\"initial_heading_deg\":%.5f,"
      "\"counter_clockwise_target_heading_deg\":%.5f,"
      "\"reverse_line_follow_duty\":%.5f,\"lss1_timeout_ms\":%u,"
      "\"post_lss1_delay_ms\":%u,"
      "\"initial_heading_turn_timeout_ms\":%u,"
      "\"pre_counter_clockwise_strafe_right_duty\":%.5f,"
      "\"pre_counter_clockwise_strafe_right_duration_ms\":%u,"
      "\"counter_clockwise_angle_deg\":%.5f,"
      "\"counter_clockwise_timeout_ms\":%u,"
      "\"forward_to_slide_duty\":%.5f,"
      "\"forward_to_slide_duration_ms\":%u,"
      "\"stepper_down_speed_steps_per_second\":%u,"
      "\"stepper_down_timeout_ms\":%u,"
      "\"pusher_open_settle_ms\":%u,"
      "\"push_forward_duty\":%.5f,"
      "\"push_forward_duration_ms\":%u,"
      "\"reverse_retreat_duty\":%.5f,"
      "\"reverse_retreat_duration_ms\":%u,"
      "\"clockwise_angle_deg\":%.5f,"
      "\"clockwise_timeout_ms\":%u,"
      "\"post_clockwise_reverse_duty\":%.5f,"
      "\"post_clockwise_reverse_duration_ms\":%u,"
      "\"post_clockwise_strafe_left_duty\":%.5f,"
      "\"post_clockwise_strafe_left_duration_ms\":%u,"
      "\"post_clockwise_strafe_right_duration_ms\":%u,"
      "\"post_clockwise_delay_ms\":%u,"
      "\"exit_forward_duty\":%.5f,"
      "\"exit_forward_duration_ms\":%u,"
      "\"post_forward_delay_ms\":%u,"
      "\"strafe_right_duty\":%.5f,"
      "\"strafe_right_timeout_ms\":%u}",
      habitatPlacementStateName(snapshot.habitat_placement_state),
      habitatPlacementFaultReasonName(
          snapshot.habitat_placement_fault_reason),
      static_cast<unsigned>(
          snapshot.habitat_placement_time_in_state_ms),
      jsonBool(snapshot.habitat_placement_configuration_valid),
      jsonBool(snapshot.habitat_placement_start_ready),
      jsonBool(
          snapshot.habitat_placement_initial_heading_captured),
      snapshot.habitat_placement_initial_heading_deg,
      snapshot.habitat_placement_counter_clockwise_target_heading_deg,
      habitat_placement.reverse_line_follow_duty,
      static_cast<unsigned>(habitat_placement.lss1_timeout_ms),
      static_cast<unsigned>(habitat_placement.post_lss1_delay_ms),
      static_cast<unsigned>(
          habitat_placement.initial_heading_turn_timeout_ms),
      habitat_placement.pre_counter_clockwise_strafe_right_duty,
      static_cast<unsigned>(
          habitat_placement
              .pre_counter_clockwise_strafe_right_duration_ms),
      habitat_placement.counter_clockwise_angle_deg,
      static_cast<unsigned>(
          habitat_placement.counter_clockwise_timeout_ms),
      habitat_placement.forward_to_slide_duty,
      static_cast<unsigned>(
          habitat_placement.forward_to_slide_duration_ms),
      static_cast<unsigned>(
          habitat_placement.stepper_down_speed_steps_per_second),
      static_cast<unsigned>(habitat_placement.stepper_down_timeout_ms),
      static_cast<unsigned>(habitat_placement.pusher_open_settle_ms),
      habitat_placement.push_forward_duty,
      static_cast<unsigned>(habitat_placement.push_forward_duration_ms),
      habitat_placement.reverse_retreat_duty,
      static_cast<unsigned>(
          habitat_placement.reverse_retreat_duration_ms),
      habitat_placement.clockwise_angle_deg,
      static_cast<unsigned>(habitat_placement.clockwise_timeout_ms),
      habitat_placement.post_clockwise_reverse_duty,
      static_cast<unsigned>(
          habitat_placement.post_clockwise_reverse_duration_ms),
      habitat_placement.post_clockwise_strafe_left_duty,
      static_cast<unsigned>(
          habitat_placement.post_clockwise_strafe_left_duration_ms),
      static_cast<unsigned>(
          habitat_placement.post_clockwise_strafe_right_duration_ms),
      static_cast<unsigned>(habitat_placement.post_clockwise_delay_ms),
      habitat_placement.exit_forward_duty,
      static_cast<unsigned>(habitat_placement.exit_forward_duration_ms),
      static_cast<unsigned>(habitat_placement.post_forward_delay_ms),
      habitat_placement.strafe_right_duty,
      static_cast<unsigned>(habitat_placement.strafe_right_timeout_ms));

  writer.append(
      ",\"tower_pieces\":{\"state\":\"%s\","
      "\"fault_reason\":\"%s\",\"time_in_state_ms\":%u,"
      "\"reverse_line_duty\":%.5f,\"side_line_timeout_ms\":%u,"
      "\"post_line_delay_ms\":%u,\"strafe_right_duty\":%.5f,"
      "\"strafe_right_duration_ms\":%u,"
      "\"post_strafe_pause_ms\":%u,"
      "\"clockwise_rotation_duty\":%.5f,"
      "\"clockwise_rotation_angle_deg\":%.5f,"
      "\"post_rotation_pause_ms\":%u,"
      "\"reverse_duty\":%.5f,\"reverse_duration_ms\":%u,"
      "\"shimmy_duty\":%.5f,"
      "\"shimmy_right_duration_ms\":%u,"
      "\"shimmy_left_duration_ms\":%u,"
      "\"shimmy_timeout_ms\":%u,"
      "\"final_reverse_duty\":%.5f,"
      "\"final_reverse_duration_ms\":%u,"
      "\"post_final_reverse_delay_ms\":%u,"
      "\"post_winch_open_delay_ms\":%u,"
      "\"post_claws_open_delay_ms\":%u,"
      "\"stepper_down_speed_steps_per_second\":%u,"
      "\"post_stepper_bottom_delay_ms\":%u,"
      "\"post_claws_closed_delay_ms\":%u,"
      "\"stepper_up_speed_steps_per_second\":%u,"
      "\"side_line_count\":%u,\"target_side_line_count\":%u,"
      "\"side_line_sensor_configured\":%s,"
      "\"side_line_sensor_high\":%s,\"line_following\":%s,"
      "\"strafing_right\":%s,\"rotating_clockwise\":%s,"
      "\"driving_backward\":%s,\"shimmying_left\":%s,"
      "\"shimmying_right\":%s,\"back_line_detected\":%s,"
      "\"final_reverse_active\":%s,"
      "\"stepper_moving_down\":%s,\"stepper_moving_up\":%s}",
      towerPiecesStateName(snapshot.tower_pieces_state),
      towerPiecesFaultReasonName(snapshot.tower_pieces_fault_reason),
      static_cast<unsigned>(snapshot.tower_pieces_time_in_state_ms),
      snapshot.tower_pieces_reverse_line_duty,
      static_cast<unsigned>(snapshot.tower_pieces_side_line_timeout_ms),
      static_cast<unsigned>(snapshot.tower_pieces_post_line_delay_ms),
      snapshot.tower_pieces_strafe_right_duty,
      static_cast<unsigned>(snapshot.tower_pieces_strafe_right_duration_ms),
      static_cast<unsigned>(snapshot.tower_pieces_post_strafe_pause_ms),
      snapshot.tower_pieces_clockwise_rotation_duty,
      snapshot.tower_pieces_clockwise_rotation_angle_deg,
      static_cast<unsigned>(snapshot.tower_pieces_post_rotation_pause_ms),
      snapshot.tower_pieces_reverse_duty,
      static_cast<unsigned>(snapshot.tower_pieces_reverse_duration_ms),
      snapshot.tower_pieces_shimmy_duty,
      static_cast<unsigned>(
          snapshot.tower_pieces_shimmy_right_duration_ms),
      static_cast<unsigned>(
          snapshot.tower_pieces_shimmy_left_duration_ms),
      static_cast<unsigned>(snapshot.tower_pieces_shimmy_timeout_ms),
      snapshot.tower_pieces_final_reverse_duty,
      static_cast<unsigned>(snapshot.tower_pieces_final_reverse_duration_ms),
      static_cast<unsigned>(
          snapshot.tower_pieces_post_final_reverse_delay_ms),
      static_cast<unsigned>(snapshot.tower_pieces_post_winch_open_delay_ms),
      static_cast<unsigned>(snapshot.tower_pieces_post_claws_open_delay_ms),
      static_cast<unsigned>(
          snapshot.tower_pieces_stepper_down_speed_steps_per_second),
      static_cast<unsigned>(
          snapshot.tower_pieces_post_stepper_bottom_delay_ms),
      static_cast<unsigned>(snapshot.tower_pieces_post_claws_closed_delay_ms),
      static_cast<unsigned>(
          snapshot.tower_pieces_stepper_up_speed_steps_per_second),
      static_cast<unsigned>(snapshot.tower_pieces_side_line_count),
      static_cast<unsigned>(snapshot.tower_pieces_target_side_line_count),
      jsonBool(snapshot.tower_pieces_side_line_sensor_configured),
      jsonBool(snapshot.tower_pieces_side_line_sensor_high),
      jsonBool(snapshot.tower_pieces_line_following),
      jsonBool(snapshot.tower_pieces_strafing_right),
      jsonBool(snapshot.tower_pieces_rotating_clockwise),
      jsonBool(snapshot.tower_pieces_driving_backward),
      jsonBool(snapshot.tower_pieces_shimmying_left),
      jsonBool(snapshot.tower_pieces_shimmying_right),
      jsonBool(snapshot.tower_pieces_back_line_detected),
      jsonBool(snapshot.tower_pieces_final_reverse_active),
      jsonBool(snapshot.tower_pieces_stepper_moving_down),
      jsonBool(snapshot.tower_pieces_stepper_moving_up));

  writer.append(
      ",\"tower_line_control\":{\"initial_strafe_duty\":%.5f,"
      "\"shimmy_duty\":%.5f,\"side_line_cooldown_ms\":%u,"
      "\"side_line_rearm_ms\":%u,\"side_line_armed\":%s,"
      "\"crossing_count\":%u,\"rejected_detection_count\":%u,"
      "\"detection_accepted\":%s,"
      "\"detection_rejected\":%s,"
      "\"pre_stepper_bottom_delay_ms\":%u}",
      snapshot.tower_pieces_strafe_right_duty,
      snapshot.tower_pieces_shimmy_duty,
      static_cast<unsigned>(
          snapshot.tower_pieces_side_line_cooldown_ms),
      static_cast<unsigned>(snapshot.tower_pieces_side_line_rearm_ms),
      jsonBool(snapshot.tower_pieces_side_line_armed),
      static_cast<unsigned>(snapshot.tower_pieces_side_line_count),
      static_cast<unsigned>(
          snapshot.tower_pieces_side_line_rejected_count),
      jsonBool(snapshot.tower_pieces_side_line_detection_accepted),
      jsonBool(snapshot.tower_pieces_side_line_detection_rejected),
      static_cast<unsigned>(
          snapshot.tower_pieces_pre_stepper_bottom_delay_ms));

  writer.append(
      ",\"peg_finder\":{\"state\":\"%s\","
      "\"fault_reason\":\"%s\",\"time_in_state_ms\":%u,"
      "\"clockwise_duty\":%.5f,\"clockwise_angle_deg\":%.5f,"
      "\"post_rotation_pause_ms\":%u,"
      "\"reverse_duty\":%.5f,\"reverse_duration_ms\":%u,"
      "\"post_reverse_pause_ms\":%u,"
      "\"forward_duty\":%.5f,\"forward_duration_ms\":%u,"
      "\"funnel_forward_duty\":%.5f,"
      "\"funnel_forward_timeout_ms\":%u,"
      "\"funnel_forward_duration_ms\":%u,"
      "\"post_funnel_limit_delay_ms\":%u,"
      "\"claw_open_interval_ms\":%u,"
      "\"claw_open_order\":[%u,%u,%u],"
      "\"post_claws_open_delay_ms\":%u,"
      "\"funnel_reverse_duty\":%.5f,"
      "\"funnel_reverse_duration_ms\":%u,"
      "\"funnel_limit_configured\":%s,\"funnel_limit_high\":%s,"
      "\"rotating_clockwise\":%s,\"driving_backward\":%s,"
      "\"driving_forward\":%s,\"funnel_forward\":%s,"
      "\"funnel_reverse\":%s,"
      "\"opening_claw_1\":%s,\"opening_claw_2\":%s,"
      "\"opening_claw_3\":%s}",
      pegFinderStateName(snapshot.peg_finder_state),
      pegFinderFaultReasonName(snapshot.peg_finder_fault_reason),
      static_cast<unsigned>(snapshot.peg_finder_time_in_state_ms),
      snapshot.peg_finder_clockwise_duty,
      snapshot.peg_finder_clockwise_angle_deg,
      static_cast<unsigned>(snapshot.peg_finder_post_rotation_pause_ms),
      snapshot.peg_finder_reverse_duty,
      static_cast<unsigned>(snapshot.peg_finder_reverse_duration_ms),
      static_cast<unsigned>(snapshot.peg_finder_post_reverse_pause_ms),
      snapshot.peg_finder_forward_duty,
      static_cast<unsigned>(snapshot.peg_finder_forward_duration_ms),
      snapshot.peg_finder_funnel_forward_duty,
      static_cast<unsigned>(snapshot.peg_finder_funnel_forward_timeout_ms),
      static_cast<unsigned>(snapshot.peg_finder_funnel_forward_timeout_ms),
      static_cast<unsigned>(
          snapshot.peg_finder_post_funnel_limit_delay_ms),
      static_cast<unsigned>(snapshot.peg_finder_claw_open_interval_ms),
      static_cast<unsigned>(snapshot.peg_finder_claw_open_order_1),
      static_cast<unsigned>(snapshot.peg_finder_claw_open_order_2),
      static_cast<unsigned>(snapshot.peg_finder_claw_open_order_3),
      static_cast<unsigned>(snapshot.peg_finder_post_claws_open_delay_ms),
      snapshot.peg_finder_funnel_reverse_duty,
      static_cast<unsigned>(snapshot.peg_finder_funnel_reverse_duration_ms),
      jsonBool(snapshot.peg_finder_funnel_limit_configured),
      jsonBool(snapshot.peg_finder_funnel_limit_high),
      jsonBool(snapshot.peg_finder_rotating_clockwise),
      jsonBool(snapshot.peg_finder_driving_backward),
      jsonBool(snapshot.peg_finder_driving_forward),
      jsonBool(snapshot.peg_finder_funnel_forward),
      jsonBool(snapshot.peg_finder_funnel_reverse),
      jsonBool(snapshot.peg_finder_opening_claw_1),
      jsonBool(snapshot.peg_finder_opening_claw_2),
      jsonBool(snapshot.peg_finder_opening_claw_3));

  writer.append(
      ",\"time_trial\":{\"state\":\"%s\",\"time_in_state_ms\":%u,"
      "\"post_solar_delay_ms\":%u,\"strafe_right_duty\":%.5f,"
      "\"strafe_right_duration_ms\":%u,\"post_tower_delay_ms\":%u,"
      "\"strafing_right\":%s}",
      timeTrialStateName(snapshot.time_trial_state),
      static_cast<unsigned>(snapshot.time_trial_time_in_state_ms),
      static_cast<unsigned>(snapshot.time_trial_post_solar_delay_ms),
      snapshot.time_trial_strafe_right_duty,
      static_cast<unsigned>(snapshot.time_trial_strafe_right_duration_ms),
      static_cast<unsigned>(snapshot.time_trial_post_tower_delay_ms),
      jsonBool(snapshot.time_trial_strafing_right));

  writer.append(",\"motors\":{");
  appendMotor(writer, "front_left", snapshot.front_left, true);
  appendMotor(writer, "front_right", snapshot.front_right, true);
  appendMotor(writer, "funnel", snapshot.funnel, false);
  writer.append("}");

  writer.append(",\"rear\":{\"back_left_desired_command_milli\":%d,"
                "\"back_right_desired_command_milli\":%d,"
                "\"rear_command_sequence_number\":%u,"
                "\"rear_command_age_ms\":%u,\"esp1_link_healthy\":%s,"
                "\"esp1_link_configured\":%s,"
                "\"esp1_last_packet_age_ms\":%u,"
                "\"esp1_packet_error_count\":%u}",
                snapshot.rear.back_left_desired_command_milli,
                snapshot.rear.back_right_desired_command_milli,
                static_cast<unsigned>(snapshot.rear.sequence),
                static_cast<unsigned>(snapshot.rear.command_age_ms),
                jsonBool(snapshot.rear.esp1_link_healthy),
                jsonBool(snapshot.rear.esp1_link_configured),
                static_cast<unsigned>(snapshot.rear.esp1_last_packet_age_ms),
                static_cast<unsigned>(snapshot.rear.esp1_packet_error_count));

  writer.append(",\"esp1\":{\"available\":%s,\"uptime_ms\":%u,"
                "\"mode\":\"%s\",\"fault_active\":%s,\"fault_code\":\"%s\","
                "\"back_left_applied_command_milli\":%d,"
                "\"back_right_applied_command_milli\":%d,"
                "\"funnel_applied_command_milli\":%d,"
                "\"back_left_inverted\":%s,\"back_right_inverted\":%s,"
                "\"funnel_configured\":%s,"
                "\"solar_panel_limit_switches_configured\":%s,"
                "\"solar_limit_back_right_high\":%s,"
                "\"solar_limit_front_right_high\":%s,"
                "\"side_line_sensor_configured\":%s,"
                "\"side_line_sensor_high\":%s,"
                "\"ultrasonic_1_configured\":%s,"
                "\"ultrasonic_1_echo_valid\":%s,"
                "\"ultrasonic_1_distance_mm\":%u,"
                "\"ultrasonic_1_echo_duration_us\":%u,"
                "\"solar_hook_configured\":%s,"
                "\"solar_hook_output_enabled\":%s,"
                "\"solar_hook_commanded_angle_deg\":%d}",
                jsonBool(snapshot.esp1.available),
                static_cast<unsigned>(snapshot.esp1.uptime_ms),
                robotTestModeName(snapshot.esp1.mode),
                jsonBool(snapshot.esp1.fault_active),
                faultCodeName(snapshot.esp1.fault_code),
                snapshot.esp1.back_left_applied_command_milli,
                snapshot.esp1.back_right_applied_command_milli,
                snapshot.esp1.funnel_applied_command_milli,
                jsonBool(snapshot.esp1.back_left_inverted),
                jsonBool(snapshot.esp1.back_right_inverted),
                jsonBool(snapshot.esp1.funnel_configured),
                jsonBool(snapshot.esp1
                             .solar_panel_limit_switches_configured),
                jsonBool(snapshot.esp1.solar_limit_back_right_high),
                jsonBool(snapshot.esp1.solar_limit_front_right_high),
                jsonBool(snapshot.esp1.side_line_sensor_configured),
                jsonBool(snapshot.esp1.side_line_sensor_high),
                jsonBool(snapshot.esp1.ultrasonic_1_configured),
                jsonBool(snapshot.esp1.ultrasonic_1_echo_valid),
                static_cast<unsigned>(
                    snapshot.esp1.ultrasonic_1_distance_mm),
                static_cast<unsigned>(
                    snapshot.esp1.ultrasonic_1_echo_duration_us),
                jsonBool(snapshot.esp1.solar_hook_configured),
                jsonBool(snapshot.esp1.solar_hook_output_enabled),
                snapshot.esp1.solar_hook_commanded_angle_deg);

  writer.append(
      ",\"ultrasonic_1\":{\"configured\":%s,\"data_fresh\":%s,"
      "\"echo_valid\":%s,\"distance_mm\":%u,\"echo_duration_us\":%u,"
      "\"sample_age_ms\":%u}",
      jsonBool(snapshot.ultrasonic_1.configured),
      jsonBool(snapshot.ultrasonic_1.data_fresh),
      jsonBool(snapshot.ultrasonic_1.echo_valid),
      static_cast<unsigned>(snapshot.ultrasonic_1.distance_mm),
      static_cast<unsigned>(snapshot.ultrasonic_1.echo_duration_us),
      static_cast<unsigned>(snapshot.ultrasonic_1.sample_age_ms));

  writer.append(
      ",\"laser_distance\":{\"available\":%s,\"configured\":%s,"
      "\"initialized\":%s,\"ranging\":%s,\"data_fresh\":%s,"
      "\"data_valid\":%s,\"profile\":\"%s\",\"distance_mm\":%u,"
      "\"measurement_sequence\":%u,\"packet_sequence\":%u,"
      "\"sensor_range_status\":%u,\"driver_status\":%d,"
      "\"sda_gpio\":%d,\"scl_gpio\":%d,\"i2c_address\":%u,"
      "\"captured_at_ms\":%u,\"sample_age_ms\":%u,"
      "\"snapshot_age_ms\":%u,\"intermeasurement_period_ms\":%u,"
      "\"successful_measurement_count\":%u,"
      "\"failed_measurement_count\":%u,"
      "\"consecutive_failed_measurements\":%u,"
      "\"acquisition_duration_us\":%u,"
      "\"maximum_acquisition_duration_us\":%u}",
      jsonBool(snapshot.laser_distance.available),
      jsonBool(snapshot.laser_distance.configured),
      jsonBool(snapshot.laser_distance.initialized),
      jsonBool(snapshot.laser_distance.ranging),
      jsonBool(snapshot.laser_distance.data_fresh),
      jsonBool(snapshot.laser_distance.data_valid),
      laserDistanceProfileName(snapshot.laser_distance.profile),
      static_cast<unsigned>(snapshot.laser_distance.distance_mm),
      static_cast<unsigned>(
          snapshot.laser_distance.measurement_sequence),
      static_cast<unsigned>(snapshot.laser_distance.packet_sequence),
      static_cast<unsigned>(
          snapshot.laser_distance.sensor_range_status),
      static_cast<int>(snapshot.laser_distance.driver_status),
      static_cast<int>(snapshot.laser_distance.sda_gpio),
      static_cast<int>(snapshot.laser_distance.scl_gpio),
      static_cast<unsigned>(snapshot.laser_distance.i2c_address),
      static_cast<unsigned>(snapshot.laser_distance.captured_at_ms),
      static_cast<unsigned>(snapshot.laser_distance.sample_age_ms),
      static_cast<unsigned>(snapshot.laser_distance.snapshot_age_ms),
      static_cast<unsigned>(
          snapshot.laser_distance.intermeasurement_period_ms),
      static_cast<unsigned>(
          snapshot.laser_distance.successful_measurement_count),
      static_cast<unsigned>(
          snapshot.laser_distance.failed_measurement_count),
      static_cast<unsigned>(
          snapshot.laser_distance.consecutive_failed_measurements),
      static_cast<unsigned>(
          snapshot.laser_distance.acquisition_duration_us),
      static_cast<unsigned>(
          snapshot.laser_distance.maximum_acquisition_duration_us));

  writer.append(",\"claws\":{");
  appendClaw(writer, "claw_1", snapshot.claws.claw_1, true);
  appendClaw(writer, "claw_2", snapshot.claws.claw_2, true);
  appendClaw(writer, "claw_3", snapshot.claws.claw_3, true);
  appendClaw(writer, "habitat_pusher", snapshot.claws.habitat_pusher,
             true);
  appendClaw(writer, "winch", snapshot.claws.winch, false);
  writer.append("}");

  writer.append(
      ",\"solar_hook\":{\"hardwareConfigured\":%s,"
      "\"openConfigured\":%s,\"closedConfigured\":%s,"
      "\"outputEnabled\":%s,\"openAngleDeg\":%d,"
      "\"closedAngleDeg\":%d,\"commandedAngleDeg\":%d,"
      "\"commandedOpen\":%s}",
      jsonBool(snapshot.solar_hook.hardware_configured),
      jsonBool(snapshot.solar_hook.open_configured),
      jsonBool(snapshot.solar_hook.closed_configured),
      jsonBool(snapshot.solar_hook.output_enabled),
      snapshot.solar_hook.open_angle_deg,
      snapshot.solar_hook.closed_angle_deg,
      snapshot.solar_hook.commanded_angle_deg,
      jsonBool(snapshot.solar_hook.commanded_open));

  writer.append(",\"selectedBeaconFrequencyHz\":%u,"
                "\"switchRawState\":%s,"
                "\"switchDebouncedState\":%s,"
                "\"ir_adc_average\":%u,\"ir_adc_min\":%u,"
                "\"ir_adc_max\":%u,\"ir_amplitude_pp\":%u,"
                "\"latest_raw_adc_sample\":%u,"
                "\"adc_sample_mean\":%u,"
                "\"peak_to_peak_amplitude\":%u,"
                "\"ir_1khz_goertzel_amplitude\":%u,"
                "\"ir_10khz_goertzel_amplitude\":%u,"
                "\"ir_selected_frequency_amplitude\":%u,"
                "\"ir_active_threshold\":%u,"
                "\"ir_beacon_detected\":%s,"
                "\"ir_consecutive_detection_count\":%u,"
                "\"ir_adc_sample_rate_hz\":%u,"
                "\"motor_command_magnitude_milli\":%u",
                static_cast<unsigned>(
                    snapshot.selected_beacon_frequency_hz),
                jsonBool(snapshot.ir_switch_raw_state),
                jsonBool(snapshot.ir_switch_debounced_state),
                static_cast<unsigned>(snapshot.ir_adc_average),
                static_cast<unsigned>(snapshot.ir_adc_min),
                static_cast<unsigned>(snapshot.ir_adc_max),
                static_cast<unsigned>(snapshot.ir_amplitude_pp),
                static_cast<unsigned>(snapshot.ir_adc_latest_sample),
                static_cast<unsigned>(snapshot.ir_adc_sample_mean),
                static_cast<unsigned>(snapshot.ir_amplitude_pp),
                static_cast<unsigned>(
                    snapshot.ir_1khz_goertzel_amplitude),
                static_cast<unsigned>(
                    snapshot.ir_10khz_goertzel_amplitude),
                static_cast<unsigned>(
                    snapshot.ir_selected_frequency_amplitude),
                static_cast<unsigned>(snapshot.ir_active_threshold),
                jsonBool(snapshot.ir_beacon_detected),
                static_cast<unsigned>(
                    snapshot.ir_consecutive_detection_count),
                static_cast<unsigned>(snapshot.ir_adc_sample_rate_hz),
                static_cast<unsigned>(
                    snapshot.motor_command_magnitude_milli));

  writer.append("}");
  return writer.ok();
}

bool writeEventLogJson(const EventLog& log, char* output,
                       const std::size_t output_size) {
  JsonWriter writer{output, output_size};
  writer.append("{\"events\":[");
  for (std::size_t index = 0U; index < log.size(); ++index) {
    EventRecord record{};
    if (log.newest(index, record)) {
      appendEvent(writer, record, index + 1U < log.size());
    }
  }
  writer.append("]}");
  return writer.ok();
}

}  // namespace robot
