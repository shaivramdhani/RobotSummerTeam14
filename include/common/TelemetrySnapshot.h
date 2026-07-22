#pragma once

#include <cstddef>
#include <cstdint>

#include "common/EventLog.h"
#include "common/FaultHealth.h"
#include "common/LineFollower.h"
#include "common/RobotTestMode.h"
#include "common/SolarPanelAutonomy.h"
#include "common/TowerPiecesAutonomy.h"
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
  std::int16_t funnel_applied_command_milli{0};
  bool back_left_inverted{false};
  bool back_right_inverted{false};
  bool funnel_configured{false};
  bool solar_panel_limit_switches_configured{false};
  bool solar_limit_back_right_high{false};
  bool solar_limit_front_right_high{false};
  bool side_line_sensor_configured{false};
  bool side_line_sensor_high{false};
};

struct ServoClawTelemetry {
  bool hardware_configured{false};
  bool open_configured{false};
  bool closed_configured{false};
  bool output_enabled{false};
  int open_angle_deg{-1};
  int closed_angle_deg{-1};
  int commanded_angle_deg{-1};
  bool commanded_open{false};
};

struct ServoClawBankTelemetry {
  ServoClawTelemetry claw_1{};
  ServoClawTelemetry claw_2{};
  ServoClawTelemetry claw_3{};
  ServoClawTelemetry winch{};
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
  int lss_raw_level{-1};
  bool lsfl_black{false};
  bool lsfr_black{false};
  bool lss_black{false};
  bool lss_configured{false};
  std::int8_t line_error{0};
  bool line_visible{false};
  bool line_has_history{false};
  std::int8_t last_known_line_side{0};
  bool line_follower_enabled{false};

  int lsbl_raw_level{-1};
  int lsbr_raw_level{-1};
  bool lsbl_black{false};
  bool lsbr_black{false};
  bool rear_line_configured{false};
  bool rear_line_data_fresh{false};
  std::uint16_t rear_line_sequence{0};
  Milliseconds rear_line_sample_age_ms{0};
  Milliseconds rear_line_captured_at_ms{0};
  std::int8_t rear_line_error{0};
  bool rear_line_visible{false};
  bool rear_line_has_history{false};
  std::int8_t rear_last_known_line_side{0};
  bool rear_line_follower_enabled{false};
  bool rear_logical_left_black{false};
  bool rear_logical_right_black{false};

  float rear_kp{0.0F};
  float rear_ki{0.0F};
  float rear_kd{0.0F};
  float rear_base_duty{0.0F};
  float rear_effective_base_duty{0.0F};
  float rear_maximum_duty{0.0F};
  float rear_maximum_correction{0.0F};
  float rear_integral_limit{0.0F};
  float rear_derivative_limit{0.0F};
  float rear_derivative_filter_alpha{0.0F};
  int rear_steering_polarity{1};
  Milliseconds rear_control_period_ms{0};
  Milliseconds rear_remote_command_timeout_ms{0};
  bool rear_line_telemetry_enabled{false};
  float rear_pid_p_term{0.0F};
  float rear_pid_i_term{0.0F};
  float rear_pid_d_term{0.0F};
  float rear_pid_correction{0.0F};

  float kp{0.0F};
  float ki{0.0F};
  float kd{0.0F};
  float base_duty{0.0F};
  float maximum_duty{0.0F};
  float maximum_correction{0.0F};
  float integral_limit{0.0F};
  float derivative_limit{0.0F};
  float derivative_filter_alpha{0.0F};
  int steering_polarity{1};
  Milliseconds control_period_ms{0};
  Milliseconds remote_command_timeout_ms{0};
  bool line_telemetry_enabled{false};
  float pid_p_term{0.0F};
  float pid_i_term{0.0F};
  float pid_d_term{0.0F};
  float pid_correction{0.0F};

  SolarPanelAutonomyState autonomous_state{
      SolarPanelAutonomyState::WaitForStart};
  SolarPanelFaultReason autonomous_fault_reason{
      SolarPanelFaultReason::None};
  Milliseconds autonomous_time_in_state_ms{0};
  std::uint16_t solar_ir_raw_amplitude{0};
  float solar_ir_filtered_amplitude{0.0F};
  std::uint16_t solar_ir_detection_threshold{0};
  std::uint16_t solar_ir_release_threshold{0};
  std::uint16_t solar_ir_detection_threshold_1khz{0};
  std::uint16_t solar_ir_release_threshold_1khz{0};
  std::uint16_t solar_ir_detection_threshold_10khz{0};
  std::uint16_t solar_ir_release_threshold_10khz{0};
  Milliseconds solar_ir_confirmation_progress_ms{0};
  Milliseconds solar_ir_confirmation_time_ms{0};
  float solar_ir_filter_alpha{0.0F};
  Milliseconds solar_ir_ignore_after_start_ms{0};
  Milliseconds solar_search_timeout_ms{0};
  float solar_start_base_duty{0.0F};
  Milliseconds solar_slow_after_ms{0};
  float solar_slow_base_duty{0.0F};
  bool solar_slow_mode_active{false};
  bool solar_ir_confirmation_active{false};
  bool solar_beacon_confirmed{false};
  Milliseconds solar_contact_timeout_ms{0};
  float solar_contact_strafe_duty{0.0F};
  Milliseconds solar_strafe_start_delay_ms{0};
  Milliseconds solar_retry_strafe_left_duration_ms{0};
  Milliseconds solar_retry_forward_duration_ms{0};
  Milliseconds solar_retry_strafe_timeout_ms{0};
  Milliseconds solar_post_contact_forward_duration_ms{0};
  float solar_line_reacquire_strafe_duty{0.0F};
  Milliseconds solar_post_contact_forward_start_delay_ms{0};
  Milliseconds solar_line_reacquire_strafe_start_delay_ms{0};
  float solar_post_contact_forward_duty{0.0F};
  bool solar_panel_limit_switches_configured{false};
  bool solar_limit_back_right_high{false};
  bool solar_limit_front_right_high{false};
  bool solar_limit_back_right_hit{false};
  bool solar_limit_front_right_hit{false};
  bool solar_limit_all_hit{false};

  TowerPiecesState tower_pieces_state{TowerPiecesState::WaitForStart};
  TowerPiecesFaultReason tower_pieces_fault_reason{
      TowerPiecesFaultReason::None};
  Milliseconds tower_pieces_time_in_state_ms{0};
  float tower_pieces_reverse_line_duty{0.0F};
  Milliseconds tower_pieces_side_line_timeout_ms{0};
  Milliseconds tower_pieces_post_line_delay_ms{0};
  float tower_pieces_strafe_right_duty{0.0F};
  Milliseconds tower_pieces_strafe_right_duration_ms{0};
  Milliseconds tower_pieces_post_strafe_pause_ms{0};
  float tower_pieces_clockwise_rotation_duty{0.0F};
  Milliseconds tower_pieces_clockwise_rotation_duration_ms{0};
  Milliseconds tower_pieces_post_rotation_pause_ms{0};
  float tower_pieces_reverse_duty{0.0F};
  Milliseconds tower_pieces_reverse_duration_ms{0};
  float tower_pieces_shimmy_duty{0.0F};
  Milliseconds tower_pieces_shimmy_right_duration_ms{0};
  Milliseconds tower_pieces_shimmy_left_duration_ms{0};
  Milliseconds tower_pieces_shimmy_timeout_ms{0};
  std::uint8_t tower_pieces_side_line_count{0};
  std::uint8_t tower_pieces_target_side_line_count{
      kTowerPiecesTargetSideLineCount};
  bool tower_pieces_side_line_sensor_configured{false};
  bool tower_pieces_side_line_sensor_high{false};
  bool tower_pieces_line_following{false};
  bool tower_pieces_strafing_right{false};
  bool tower_pieces_rotating_clockwise{false};
  bool tower_pieces_driving_backward{false};
  bool tower_pieces_shimmying_left{false};
  bool tower_pieces_shimmying_right{false};
  bool tower_pieces_back_line_detected{false};

  MotorTelemetry front_left{};
  MotorTelemetry front_right{};
  MotorTelemetry funnel{};
  RearCommandTelemetry rear{};
  Esp1RemoteStatusTelemetry esp1{};
  ServoClawBankTelemetry claws{};

  std::uint16_t ir_adc_average{0};
  std::uint16_t ir_adc_min{0};
  std::uint16_t ir_adc_max{0};
  std::uint16_t ir_amplitude_pp{0};
  bool ir_beacon_detected{false};
  bool ir_switch_raw_state{false};
  bool ir_switch_debounced_state{false};
  std::uint32_t selected_beacon_frequency_hz{0};
  std::uint16_t ir_adc_latest_sample{0};
  std::uint16_t ir_adc_sample_mean{0};
  std::uint16_t ir_1khz_goertzel_amplitude{0};
  std::uint16_t ir_10khz_goertzel_amplitude{0};
  std::uint16_t ir_selected_frequency_amplitude{0};
  std::uint16_t ir_active_threshold{0};
  std::uint8_t ir_consecutive_detection_count{0};
  std::uint32_t ir_adc_sample_rate_hz{0};
  std::uint16_t motor_command_magnitude_milli{0};

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
