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
  writer.append("\"%s\":{\"hardwareConfigured\":%s,"
                "\"startConfigured\":%s,\"outputEnabled\":%s,"
                "\"startAngleDeg\":%d,\"openAngleDeg\":%d,"
                "\"openDirection\":%d,\"commandedAngleDeg\":%d,"
                "\"commandedOpen\":%s}%s",
                name, jsonBool(claw.hardware_configured),
                jsonBool(claw.start_configured),
                jsonBool(claw.output_enabled), claw.start_angle_deg,
                claw.open_angle_deg, claw.open_direction,
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

  writer.append(",\"line\":{\"lsfl_raw_level\":%d,\"lsfr_raw_level\":%d,"
                "\"lsfl_level\":\"%s\",\"lsfr_level\":\"%s\","
                "\"lsfl_black\":%s,\"lsfr_black\":%s,\"line_error\":%d,"
                "\"line_visible\":%s,\"has_history\":%s,"
                "\"hasHistory\":%s,"
                "\"last_known_line_side\":%d,"
                "\"line_follower_enabled\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                digitalLevelName(snapshot.lsfl_raw_level),
                digitalLevelName(snapshot.lsfr_raw_level),
                jsonBool(snapshot.lsfl_black), jsonBool(snapshot.lsfr_black),
                static_cast<int>(snapshot.line_error),
                jsonBool(snapshot.line_visible),
                jsonBool(snapshot.line_has_history),
                jsonBool(snapshot.line_has_history),
                static_cast<int>(snapshot.last_known_line_side),
                jsonBool(snapshot.line_follower_enabled));

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
                "\"slow_mode_active\":%s}",
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
                jsonBool(snapshot.solar_slow_mode_active));

  writer.append(",\"motors\":{");
  appendMotor(writer, "front_left", snapshot.front_left, true);
  appendMotor(writer, "front_right", snapshot.front_right, false);
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
                "\"back_left_inverted\":%s,\"back_right_inverted\":%s}",
                jsonBool(snapshot.esp1.available),
                static_cast<unsigned>(snapshot.esp1.uptime_ms),
                robotTestModeName(snapshot.esp1.mode),
                jsonBool(snapshot.esp1.fault_active),
                faultCodeName(snapshot.esp1.fault_code),
                snapshot.esp1.back_left_applied_command_milli,
                snapshot.esp1.back_right_applied_command_milli,
                jsonBool(snapshot.esp1.back_left_inverted),
                jsonBool(snapshot.esp1.back_right_inverted));

  writer.append(",\"claws\":{\"rotation_deg\":%d,", snapshot.claws.rotation_deg);
  appendClaw(writer, "claw_1", snapshot.claws.claw_1, true);
  appendClaw(writer, "claw_2", snapshot.claws.claw_2, true);
  appendClaw(writer, "claw_3", snapshot.claws.claw_3, false);
  writer.append("}");

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

  if (!compact) {
    writer.append(",\"future\":{\"ir_left_strength\":%d,"
                  "\"ir_right_strength\":%d,"
                  "\"ultrasonic_1_distance_mm\":%d,"
                  "\"ultrasonic_2_distance_mm\":%d,"
                  "\"stepper_position\":%d,"
                  "\"servo_claw_1_position\":%d,"
                  "\"servo_claw_2_position\":%d,"
                  "\"servo_claw_3_position\":%d,"
                  "\"servo_pusher_position\":%d,"
                  "\"servo_winch_position\":%d,"
                  "\"limit_switch_stepper_bottom\":%s,"
                  "\"limit_switch_stepper_middle\":%s,"
                  "\"limit_switch_stepper_top\":%s,"
                  "\"limit_switch_funnel_left\":%s,"
                  "\"limit_switch_funnel_right\":%s}",
                  snapshot.ir_left_strength, snapshot.ir_right_strength,
                  snapshot.ultrasonic_1_distance_mm,
                  snapshot.ultrasonic_2_distance_mm,
                  snapshot.stepper_position, snapshot.servo_claw_1_position,
                  snapshot.servo_claw_2_position,
                  snapshot.servo_claw_3_position,
                  snapshot.servo_pusher_position,
                  snapshot.servo_winch_position,
                  jsonBool(snapshot.limit_switch_stepper_bottom),
                  jsonBool(snapshot.limit_switch_stepper_middle),
                  jsonBool(snapshot.limit_switch_stepper_top),
                  jsonBool(snapshot.limit_switch_funnel_left),
                  jsonBool(snapshot.limit_switch_funnel_right));
  }

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
