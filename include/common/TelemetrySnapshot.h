#pragma once

#include <cstddef>
#include <cstdint>

#include "common/EventLog.h"
#include "common/FaultHealth.h"
#include "common/LineFollower.h"
#include "common/RobotTestMode.h"
#include "common/Units.h"

namespace robot {

constexpr std::size_t kTelemetryIpAddressSize = 24U;
constexpr std::size_t kTelemetryResetReasonSize = 32U;
constexpr std::size_t kTelemetryFaultMessageSize = 64U;

struct MotorTelemetry {
  std::int16_t desired_command_milli{0};
  std::int16_t applied_command_milli{0};
  bool enabled{false};
  bool inverted{false};
  bool configured{false};
};

struct RearCommandTelemetry {
  std::int16_t back_left_desired_command_milli{0};
  std::int16_t back_right_desired_command_milli{0};
  std::uint16_t sequence{0};
  Milliseconds command_age_ms{0};
  bool esp1_link_healthy{false};
  bool esp1_link_configured{false};
  Milliseconds esp1_last_packet_age_ms{0};
  std::uint32_t esp1_packet_error_count{0};
};

struct Esp1RemoteStatusTelemetry {
  bool available{false};
  Milliseconds uptime_ms{0};
  RobotTestMode mode{RobotTestMode::Disabled};
  bool fault_active{false};
  FaultCode fault_code{FaultCode::None};
  std::int16_t back_left_applied_command_milli{0};
  std::int16_t back_right_applied_command_milli{0};
  bool back_left_inverted{false};
  bool back_right_inverted{false};
};

struct TelemetrySnapshot {
  Milliseconds uptime_ms{0};
  RobotTestMode current_mode{RobotTestMode::Disabled};
  RobotTestMode previous_mode{RobotTestMode::Disabled};
  bool enabled{false};
  bool fault_active{false};
  FaultCode fault_code{FaultCode::None};
  char fault_message[kTelemetryFaultMessageSize]{};
  Milliseconds last_command_age_ms{0};
  Milliseconds deadman_remaining_ms{0};
  std::uint8_t wifi_clients{0};
  char ip_address[kTelemetryIpAddressSize]{};
  std::uint32_t free_heap_bytes{0};
  char reset_reason[kTelemetryResetReasonSize]{};

  int lsfl_raw_level{-1};
  int lsfr_raw_level{-1};
  bool lsfl_black{false};
  bool lsfr_black{false};
  std::int8_t line_error{0};
  bool line_visible{false};
  std::int8_t last_known_line_side{0};
  bool line_follower_enabled{false};

  float kp{0.0F};
  float ki{0.0F};
  float kd{0.0F};
  float base_duty{0.0F};
  float maximum_duty{0.0F};
  float maximum_correction{0.0F};
  int steering_polarity{1};
  float pid_p_term{0.0F};
  float pid_i_term{0.0F};
  float pid_d_term{0.0F};
  float pid_correction{0.0F};

  MotorTelemetry front_left{};
  MotorTelemetry front_right{};
  RearCommandTelemetry rear{};
  Esp1RemoteStatusTelemetry esp1{};

  int ir_left_strength{-1};
  int ir_right_strength{-1};
  int ultrasonic_1_distance_mm{-1};
  int ultrasonic_2_distance_mm{-1};
  int stepper_position{-1};
  int servo_claw_1_position{-1};
  int servo_claw_2_position{-1};
  int servo_claw_3_position{-1};
  int servo_pusher_position{-1};
  int servo_winch_position{-1};
  bool limit_switch_stepper_bottom{false};
  bool limit_switch_stepper_middle{false};
  bool limit_switch_stepper_top{false};
  bool limit_switch_funnel_left{false};
  bool limit_switch_funnel_right{false};
};

const char* faultCodeName(FaultCode fault_code);
bool writeTelemetryJson(const TelemetrySnapshot& snapshot, char* output,
                        std::size_t output_size, bool compact);
bool writeEventLogJson(const EventLog& log, char* output,
                       std::size_t output_size);

}  // namespace robot
