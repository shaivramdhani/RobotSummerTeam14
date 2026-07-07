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
                "\"lsfl_black\":%s,\"lsfr_black\":%s,\"line_error\":%d,"
                "\"line_visible\":%s,\"last_known_line_side\":%d,"
                "\"line_follower_enabled\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                jsonBool(snapshot.lsfl_black), jsonBool(snapshot.lsfr_black),
                static_cast<int>(snapshot.line_error),
                jsonBool(snapshot.line_visible),
                static_cast<int>(snapshot.last_known_line_side),
                jsonBool(snapshot.line_follower_enabled));

  writer.append(",\"pid\":{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f,"
                "\"baseDuty\":%.5f,\"maximumDuty\":%.5f,"
                "\"maximumCorrection\":%.5f,\"steeringPolarity\":%d,"
                "\"p_term\":%.5f,\"i_term\":%.5f,\"d_term\":%.5f,"
                "\"correction\":%.5f}",
                snapshot.kp, snapshot.ki, snapshot.kd, snapshot.base_duty,
                snapshot.maximum_duty, snapshot.maximum_correction,
                snapshot.steering_polarity, snapshot.pid_p_term,
                snapshot.pid_i_term, snapshot.pid_d_term,
                snapshot.pid_correction);

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
