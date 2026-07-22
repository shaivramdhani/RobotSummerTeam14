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
                "\"openConfigured\":%s,\"closedConfigured\":%s,"
                "\"outputEnabled\":%s,\"openAngleDeg\":%d,"
                "\"closedAngleDeg\":%d,\"commandedAngleDeg\":%d,"
                "\"commandedOpen\":%s}%s",
                name, jsonBool(claw.hardware_configured),
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
                "\"lss_raw_level\":%d,"
                "\"lsfl_level\":\"%s\",\"lsfr_level\":\"%s\","
                "\"lss_level\":\"%s\","
                "\"lsfl_black\":%s,\"lsfr_black\":%s,"
                "\"lss_black\":%s,\"lss_configured\":%s,"
                "\"line_error\":%d,"
                "\"line_visible\":%s,\"has_history\":%s,"
                "\"hasHistory\":%s,"
                "\"last_known_line_side\":%d,"
                "\"line_follower_enabled\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                snapshot.lss_raw_level,
                digitalLevelName(snapshot.lsfl_raw_level),
                digitalLevelName(snapshot.lsfr_raw_level),
                digitalLevelName(snapshot.lss_raw_level),
                jsonBool(snapshot.lsfl_black), jsonBool(snapshot.lsfr_black),
                jsonBool(snapshot.lss_black),
                jsonBool(snapshot.lss_configured),
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
      ",\"tower_pieces\":{\"state\":\"%s\","
      "\"fault_reason\":\"%s\",\"time_in_state_ms\":%u,"
      "\"reverse_line_duty\":%.5f,\"side_line_timeout_ms\":%u,"
      "\"post_line_delay_ms\":%u,\"strafe_right_duty\":%.5f,"
      "\"strafe_right_duration_ms\":%u,"
      "\"post_strafe_pause_ms\":%u,"
      "\"clockwise_rotation_duty\":%.5f,"
      "\"clockwise_rotation_duration_ms\":%u,"
      "\"post_rotation_pause_ms\":%u,"
      "\"reverse_duty\":%.5f,\"reverse_duration_ms\":%u,"
      "\"shimmy_duty\":%.5f,"
      "\"shimmy_right_duration_ms\":%u,"
      "\"shimmy_left_duration_ms\":%u,"
      "\"shimmy_timeout_ms\":%u,"
      "\"side_line_count\":%u,\"target_side_line_count\":%u,"
      "\"side_line_sensor_configured\":%s,"
      "\"side_line_sensor_high\":%s,\"line_following\":%s,"
      "\"strafing_right\":%s,\"rotating_clockwise\":%s,"
      "\"driving_backward\":%s,\"shimmying_left\":%s,"
      "\"shimmying_right\":%s,\"back_line_detected\":%s}",
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
      static_cast<unsigned>(
          snapshot.tower_pieces_clockwise_rotation_duration_ms),
      static_cast<unsigned>(snapshot.tower_pieces_post_rotation_pause_ms),
      snapshot.tower_pieces_reverse_duty,
      static_cast<unsigned>(snapshot.tower_pieces_reverse_duration_ms),
      snapshot.tower_pieces_shimmy_duty,
      static_cast<unsigned>(
          snapshot.tower_pieces_shimmy_right_duration_ms),
      static_cast<unsigned>(
          snapshot.tower_pieces_shimmy_left_duration_ms),
      static_cast<unsigned>(snapshot.tower_pieces_shimmy_timeout_ms),
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
      jsonBool(snapshot.tower_pieces_back_line_detected));

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
                "\"side_line_sensor_high\":%s}",
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
                jsonBool(snapshot.esp1.side_line_sensor_high));

  writer.append(",\"claws\":{");
  appendClaw(writer, "claw_1", snapshot.claws.claw_1, true);
  appendClaw(writer, "claw_2", snapshot.claws.claw_2, true);
  appendClaw(writer, "claw_3", snapshot.claws.claw_3, true);
  appendClaw(writer, "winch", snapshot.claws.winch, false);
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
