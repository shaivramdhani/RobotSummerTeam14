#pragma once

#include <cstddef>
#include <cstdint>

#include "common/EventLog.h"
#include "common/FaultHealth.h"
#include "common/HabitatPiecesAutonomy.h"
#include "common/HabitatPlacementAutonomy.h"
#include "common/ImuHeadingHoldController.h"
#include "common/ImuTurnController.h"
#include "common/LaserDistance.h"
#include "common/LineFollower.h"
#include "common/PegFinderAutonomy.h"
#include "common/RobotTestMode.h"
#include "common/SolarPanelAutonomy.h"
#include "common/TimeTrialAutonomy.h"
#include "common/TowerPiecesAutonomy.h"
#include "common/Units.h"

namespace robot {

constexpr std::size_t kTelemetryIpAddressSize = 24U;
constexpr std::size_t kTelemetryResetReasonSize = 32U;
constexpr std::size_t kTelemetryFaultMessageSize = 64U;
constexpr std::size_t kTelemetryImuInitializationErrorSize = 32U;
constexpr std::size_t kTelemetryImuDiagnosticReasonSize = 40U;

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
  bool ultrasonic_1_configured{false};
  bool ultrasonic_1_echo_valid{false};
  std::uint16_t ultrasonic_1_distance_mm{0};
  std::uint32_t ultrasonic_1_echo_duration_us{0};
  bool solar_hook_configured{false};
  bool solar_hook_output_enabled{false};
  int solar_hook_commanded_angle_deg{-1};
};

struct UltrasonicTelemetry {
  bool configured{false};
  bool data_fresh{false};
  bool echo_valid{false};
  std::uint16_t distance_mm{0};
  std::uint32_t echo_duration_us{0};
  Milliseconds sample_age_ms{0};
};

struct LaserDistanceTelemetry {
  bool available{false};
  bool configured{false};
  bool initialized{false};
  bool ranging{false};
  bool data_fresh{false};
  bool data_valid{false};
  LaserDistanceProfile profile{LaserDistanceProfile::Default};
  std::uint16_t distance_mm{0U};
  std::uint16_t measurement_sequence{0U};
  std::uint16_t packet_sequence{0U};
  std::uint8_t sensor_range_status{0xFFU};
  std::int8_t driver_status{0};
  std::int8_t sda_gpio{-1};
  std::int8_t scl_gpio{-1};
  std::uint8_t i2c_address{kVl53l0xDefaultI2cAddress};
  Milliseconds captured_at_ms{0U};
  Milliseconds sample_age_ms{0U};
  Milliseconds snapshot_age_ms{0U};
  std::uint16_t intermeasurement_period_ms{0U};
  std::uint32_t successful_measurement_count{0U};
  std::uint32_t failed_measurement_count{0U};
  std::uint16_t consecutive_failed_measurements{0U};
  std::uint32_t acquisition_duration_us{0U};
  std::uint32_t maximum_acquisition_duration_us{0U};
};

struct ImuTelemetry {
  bool configured{false};
  bool initialized{false};
  bool calibrated{false};
  bool healthy{false};
  bool data_fresh{false};
  bool acquisition_running{false};
  bool device_acknowledged{false};
  bool runtime_configuration_valid{false};
  bool register_reads_use_repeated_start{true};

  std::uint8_t i2c_address{0x68U};
  std::uint8_t who_am_i{0U};
  int sda_gpio{-1};
  int scl_gpio{-1};
  int last_wire_status{-1};
  char initialization_error[kTelemetryImuInitializationErrorSize]{};
  char last_read_failure_reason[kTelemetryImuDiagnosticReasonSize]{};
  char disconnect_reason[kTelemetryImuDiagnosticReasonSize]{};
  char last_disconnect_reason[kTelemetryImuDiagnosticReasonSize]{};

  std::int16_t raw_gyro_z{0};
  float gyro_z_bias_dps{0.0F};
  float yaw_rate_dps{0.0F};
  float heading_deg{0.0F};

  Milliseconds sample_age_ms{0U};
  Milliseconds snapshot_age_ms{0U};
  std::uint32_t acquisition_duration_us{0U};
  std::uint32_t maximum_completed_acquisition_duration_us{0U};
  std::uint32_t total_acquisition_attempts{0U};
  std::uint32_t last_successful_read_us{0U};
  std::uint32_t last_sample_interval_us{0U};
  std::uint32_t last_read_failure_us{0U};
  std::uint32_t successful_read_count{0U};
  std::uint32_t failed_read_count{0U};
  std::uint32_t consecutive_failed_reads{0U};
  std::uint32_t disconnect_count{0U};
  Milliseconds last_disconnect_at_ms{0U};
  std::uint32_t acquisition_loop_interval_us{0U};
  std::uint32_t maximum_acquisition_loop_interval_us{0U};
  std::uint32_t synchronization_duration_us{0U};
  std::uint32_t maximum_synchronization_duration_us{0U};
  std::uint32_t wire_lock_acquire_duration_us{0U};
  std::uint32_t maximum_wire_lock_acquire_duration_us{0U};
  std::uint32_t measurement_read_duration_us{0U};
  std::uint32_t maximum_measurement_read_duration_us{0U};
  std::uint32_t successful_read_to_publication_us{0U};
  std::uint32_t maximum_successful_read_to_publication_us{0U};
  std::uint32_t publication_queue_duration_us{0U};
  std::uint32_t maximum_publication_queue_duration_us{0U};
  std::uint32_t successful_sample_publication_gap_us{0U};
  std::uint32_t maximum_successful_sample_publication_gap_us{0U};
  std::uint32_t current_observed_publication_gap_us{0U};
  std::uint32_t maximum_observed_publication_gap_us{0U};
  std::uint32_t publication_sequence{0U};
  std::uint32_t successful_sample_sequence{0U};
  std::uint32_t delayed_iteration_count{0U};
};

struct ImuTurnTelemetry {
  bool configuration_valid{false};
  bool active{false};
  ImuTurnState state{ImuTurnState::Idle};
  ImuTurnFaultReason fault_reason{ImuTurnFaultReason::None};
  bool availability_fault_capture_valid{false};
  bool availability_fault_latched{false};
  bool imu_currently_available{false};
  bool captured_configured{false};
  bool captured_initialized{false};
  bool captured_calibrated{false};
  bool captured_healthy{false};
  bool captured_sample_valid{false};
  bool captured_data_fresh{false};
  bool captured_acquisition_running{false};
  bool captured_shared_snapshot_available{false};
  bool captured_front_left_configured{false};
  bool captured_front_right_configured{false};
  bool captured_rear_link_configured{false};
  bool captured_rear_status_available{false};
  bool captured_rear_status_fresh{false};
  bool captured_newest_snapshot_available{false};
  bool captured_cached_snapshot_matches_newest{false};
  char availability_fault_origin[kTelemetryImuDiagnosticReasonSize]{};
  char captured_availability_reason[kTelemetryImuDiagnosticReasonSize]{};
  std::uint32_t availability_evaluated_at_us{0U};
  Milliseconds availability_evaluated_at_ms{0U};
  std::uint32_t captured_published_at_us{0U};
  std::uint32_t captured_last_successful_read_us{0U};
  std::uint32_t captured_sample_age_us{0U};
  std::uint32_t captured_snapshot_age_us{0U};
  std::uint32_t captured_freshness_timeout_us{0U};
  std::uint32_t captured_cached_snapshot_sequence{0U};
  std::uint32_t captured_newest_snapshot_sequence{0U};
  std::uint32_t captured_cached_successful_sample_sequence{0U};
  std::uint32_t captured_newest_successful_sample_sequence{0U};
  std::uint32_t captured_cached_snapshot_fetched_at_us{0U};
  std::uint32_t captured_cached_snapshot_fetch_to_gate_us{0U};
  std::uint32_t captured_successful_sample_publication_gap_us{0U};
  std::uint32_t captured_maximum_successful_sample_publication_gap_us{0U};
  std::uint32_t captured_current_observed_publication_gap_us{0U};
  std::uint32_t captured_maximum_observed_publication_gap_us{0U};
  Milliseconds captured_rear_last_status_received_at_ms{0U};
  Milliseconds captured_rear_status_age_ms{0U};

  float maximum_rotation_duty{0.0F};
  float kp{0.0F};
  float kd{0.0F};
  float angle_tolerance_deg{0.0F};
  float maximum_finishing_yaw_rate_dps{0.0F};
  Milliseconds settling_time_ms{0U};
  Milliseconds timeout_ms{0U};
  int yaw_command_polarity{0};

  float start_heading_deg{0.0F};
  float current_heading_deg{0.0F};
  float target_heading_deg{0.0F};
  float relative_angle_deg{0.0F};
  float angle_error_deg{0.0F};
  float yaw_rate_dps{0.0F};
  float proportional_term{0.0F};
  float damping_term{0.0F};
  float rotation_command{0.0F};
  Milliseconds elapsed_ms{0U};
  Milliseconds settling_elapsed_ms{0U};
};

struct ImuHeadingHoldTelemetry {
  bool configuration_valid{false};
  bool active{false};
  ImuHeadingHoldState state{ImuHeadingHoldState::Idle};
  ImuHeadingHoldFaultReason fault_reason{
      ImuHeadingHoldFaultReason::None};

  float maximum_strafe_duty{0.0F};
  float kp{0.0F};
  float kd{0.0F};
  float maximum_yaw_correction_duty{0.0F};
  int yaw_command_polarity{0};

  float start_heading_deg{0.0F};
  float current_heading_deg{0.0F};
  float target_heading_deg{0.0F};
  float angle_error_deg{0.0F};
  float yaw_rate_dps{0.0F};
  float proportional_term{0.0F};
  float damping_term{0.0F};
  float yaw_correction_duty{0.0F};
  int lateral_direction{0};
  Milliseconds elapsed_ms{0U};
};

struct ImuRecoveryTelemetry {
  bool turn_paused{false};
  bool strafe_paused{false};
  float turn_saved_heading_deg{0.0F};
  float strafe_saved_heading_deg{0.0F};
  Milliseconds turn_pause_elapsed_ms{0U};
  Milliseconds strafe_pause_elapsed_ms{0U};
  Milliseconds maximum_pause_ms{0U};
  std::uint8_t consecutive_fresh_samples_required{0U};
  std::uint8_t turn_consecutive_fresh_samples{0U};
  std::uint8_t strafe_consecutive_fresh_samples{0U};
  std::uint32_t turn_pause_count{0U};
  std::uint32_t strafe_pause_count{0U};
  Milliseconds total_paused_ms{0U};
};

struct ServoClawTelemetry {
  bool hardware_configured{false};
  int gpio{-1};
  int ledc_channel{-1};
  int mcpwm_unit{-1};
  int mcpwm_timer{-1};
  int mcpwm_generator{-1};
  std::uint32_t pwm_frequency_hz{0U};
  std::uint32_t mcpwm_timer_resolution_hz{0U};
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
  ServoClawTelemetry habitat_pusher{};
  ServoClawTelemetry winch{};
};

struct SolarHookServoTelemetry {
  bool hardware_configured{false};
  bool open_configured{false};
  bool closed_configured{false};
  bool output_enabled{false};
  int open_angle_deg{-1};
  int closed_angle_deg{-1};
  int commanded_angle_deg{-1};
  bool commanded_open{false};
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

  ImuTelemetry imu{};
  ImuTurnTelemetry imu_turn{};
  ImuHeadingHoldTelemetry imu_heading_hold{};
  ImuRecoveryTelemetry imu_recovery{};

  int lsfl_raw_level{-1};
  int lsfr_raw_level{-1};
  int lss_raw_level{-1};
  int lss2_raw_level{-1};
  int lss3_raw_level{-1};
  bool lsfl_black{false};
  bool lsfr_black{false};
  bool lss_black{false};
  bool lss_configured{false};
  bool lss2_black{false};
  bool lss2_configured{false};
  bool lss3_black{false};
  bool lss3_configured{false};
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
  float solar_retry_left_strafe_duty{0.0F};
  float solar_retry_right_strafe_duty{0.0F};
  Milliseconds solar_strafe_start_delay_ms{0};
  Milliseconds solar_retry_strafe_left_duration_ms{0};
  Milliseconds solar_retry_forward_duration_ms{0};
  float solar_retry_forward_duty{0.0F};
  Milliseconds solar_retry_strafe_timeout_ms{0};
  Milliseconds solar_post_contact_forward_duration_ms{0};
  float solar_line_reacquire_strafe_duty{0.0F};
  Milliseconds solar_post_contact_forward_start_delay_ms{0};
  Milliseconds solar_line_reacquire_strafe_start_delay_ms{0};
  float solar_post_contact_forward_duty{0.0F};
  Milliseconds solar_rear_line_follow_duration_ms{0U};
  bool solar_panel_limit_switches_configured{false};
  bool solar_limit_back_right_high{false};
  bool solar_limit_front_right_high{false};
  bool solar_limit_back_right_hit{false};
  bool solar_limit_front_right_hit{false};
  bool solar_limit_all_hit{false};

  HabitatPiecesState habitat_pieces_state{
      HabitatPiecesState::WaitForStart};
  HabitatPiecesStopReason habitat_pieces_stop_reason{
      HabitatPiecesStopReason::ConfigurationIncomplete};
  Milliseconds habitat_pieces_time_in_state_ms{0U};
  float habitat_pieces_line_follow_duty{
      kDefaultHabitatPiecesLineFollowDuty};
  Milliseconds habitat_pieces_lss2_detection_delay_ms{0U};
  Milliseconds habitat_pieces_lss2_detection_remaining_ms{0U};
  Milliseconds habitat_pieces_run_timeout_ms{0U};
  Milliseconds habitat_pieces_run_elapsed_ms{0U};
  Milliseconds habitat_pieces_timeout_remaining_ms{0U};
  float habitat_pieces_reverse_duty{0.0F};
  Milliseconds habitat_pieces_reverse_duration_ms{0U};
  Milliseconds habitat_pieces_reverse_elapsed_ms{0U};
  Milliseconds habitat_pieces_reverse_remaining_ms{0U};
  HabitatPiecesStrafeDirection
      habitat_pieces_distance_strafe_direction{
          HabitatPiecesStrafeDirection::None};
  std::uint16_t habitat_pieces_distance_threshold_mm{0U};
  std::uint16_t habitat_pieces_distance_zone_target_count{0U};
  float habitat_pieces_distance_strafe_duty{0.0F};
  Milliseconds habitat_pieces_distance_strafe_timeout_ms{0U};
  Milliseconds habitat_pieces_distance_strafe_elapsed_ms{0U};
  Milliseconds habitat_pieces_distance_strafe_remaining_ms{0U};
  Milliseconds habitat_pieces_post_count_stop_delay_ms{0U};
  Milliseconds habitat_pieces_post_count_stop_elapsed_ms{0U};
  Milliseconds habitat_pieces_post_count_stop_remaining_ms{0U};
  Milliseconds habitat_pieces_exit_strafe_pulse_ms{0U};
  Milliseconds habitat_pieces_exit_strafe_pulse_elapsed_ms{0U};
  Milliseconds habitat_pieces_exit_strafe_pulse_remaining_ms{0U};
  std::uint32_t habitat_pieces_slide_down_speed_steps_per_second{0U};
  Milliseconds habitat_pieces_slide_down_timeout_ms{0U};
  Milliseconds habitat_pieces_slide_down_elapsed_ms{0U};
  float habitat_pieces_approach_forward_duty{0.0F};
  Milliseconds habitat_pieces_approach_timeout_ms{0U};
  Milliseconds habitat_pieces_approach_elapsed_ms{0U};
  Milliseconds habitat_pieces_pre_lift_reverse_duration_ms{0U};
  Milliseconds habitat_pieces_pre_lift_reverse_elapsed_ms{0U};
  std::uint32_t habitat_pieces_lift_steps{0U};
  std::uint32_t habitat_pieces_lift_speed_steps_per_second{0U};
  Milliseconds habitat_pieces_lift_timeout_ms{0U};
  Milliseconds habitat_pieces_lift_elapsed_ms{0U};
  Milliseconds habitat_pieces_lift_start_delay_ms{0U};
  Milliseconds habitat_pieces_lift_start_delay_elapsed_ms{0U};
  float habitat_pieces_post_pickup_reverse_duty{0.0F};
  Milliseconds habitat_pieces_post_pickup_reverse_duration_ms{0U};
  Milliseconds habitat_pieces_post_pickup_reverse_elapsed_ms{0U};
  float habitat_pieces_rear_line_reacquire_duty{0.0F};
  Milliseconds habitat_pieces_rear_line_reacquire_timeout_ms{0U};
  Milliseconds habitat_pieces_rear_line_reacquire_elapsed_ms{0U};
  std::uint32_t habitat_pieces_distance_mm{0U};
  std::uint16_t habitat_pieces_distance_zone_count{0U};
  std::uint16_t habitat_pieces_distance_exit_pulse_count{0U};
  bool habitat_pieces_configuration_valid{false};
  bool habitat_pieces_start_ready{false};
  bool habitat_pieces_lss2_configured{false};
  bool habitat_pieces_lss2_data_fresh{false};
  bool habitat_pieces_lss3_configured{false};
  bool habitat_pieces_lss3_data_fresh{false};
  bool habitat_pieces_lss2_detection_armed{false};
  bool habitat_pieces_lss2_black{false};
  bool habitat_pieces_lss3_black{false};
  bool habitat_pieces_lss2_latched{false};
  bool habitat_pieces_lss3_latched{false};
  bool habitat_pieces_approach_limit_configured{false};
  int habitat_pieces_approach_limit_raw_level{-1};
  bool habitat_pieces_approach_limit_active{false};
  bool habitat_pieces_should_stop{true};
  bool habitat_pieces_target_reached{false};
  bool habitat_pieces_line_following{false};
  bool habitat_pieces_side_line_aligning{false};
  bool habitat_pieces_left_side_driving{false};
  bool habitat_pieces_right_side_driving{false};
  bool habitat_pieces_reversing{false};
  bool habitat_pieces_distance_strafing{false};
  bool habitat_pieces_post_count_waiting{false};
  bool habitat_pieces_exit_strafe_pulsing{false};
  bool habitat_pieces_exit_distance_checking{false};
  bool habitat_pieces_lowering_slide{false};
  bool habitat_pieces_approaching_piece{false};
  bool habitat_pieces_pre_lift_reversing{false};
  bool habitat_pieces_lifting_slide{false};
  bool habitat_pieces_lift_start_waiting{false};
  bool habitat_pieces_post_pickup_reversing{false};
  bool habitat_pieces_reacquiring_rear_line{false};
  bool habitat_pieces_waiting_for_lift{false};
  bool habitat_pieces_approach_limit_reached{false};
  bool habitat_pieces_lift_complete{false};
  bool habitat_pieces_rear_line_detected{false};
  bool habitat_pieces_distance_measurement_available{false};
  bool habitat_pieces_distance_substituted_no_target{false};
  bool habitat_pieces_distance_sample_new{false};
  bool habitat_pieces_distance_zone_active{false};
  bool habitat_pieces_distance_zone_entered{false};
  bool habitat_pieces_distance_exit_above_threshold{false};
  bool habitat_pieces_timed_out{false};

  HabitatPlacementState habitat_placement_state{
      HabitatPlacementState::WaitForStart};
  HabitatPlacementFaultReason habitat_placement_fault_reason{
      HabitatPlacementFaultReason::None};
  HabitatPlacementConfig habitat_placement_config{};
  Milliseconds habitat_placement_time_in_state_ms{0U};
  bool habitat_placement_configuration_valid{false};
  bool habitat_placement_start_ready{false};
  bool habitat_placement_initial_heading_captured{false};
  float habitat_placement_initial_heading_deg{0.0F};
  float habitat_placement_counter_clockwise_target_heading_deg{0.0F};

  TowerPiecesState tower_pieces_state{TowerPiecesState::WaitForStart};
  TowerPiecesFaultReason tower_pieces_fault_reason{
      TowerPiecesFaultReason::None};
  Milliseconds tower_pieces_time_in_state_ms{0};
  float tower_pieces_reverse_line_duty{0.0F};
  Milliseconds tower_pieces_side_line_timeout_ms{0};
  Milliseconds tower_pieces_side_line_cooldown_ms{0};
  Milliseconds tower_pieces_side_line_rearm_ms{0};
  Milliseconds tower_pieces_post_line_delay_ms{0};
  float tower_pieces_strafe_right_duty{0.0F};
  Milliseconds tower_pieces_strafe_right_duration_ms{0};
  Milliseconds tower_pieces_post_strafe_pause_ms{0};
  float tower_pieces_clockwise_rotation_duty{0.0F};
  float tower_pieces_clockwise_rotation_angle_deg{0.0F};
  Milliseconds tower_pieces_post_rotation_pause_ms{0};
  float tower_pieces_reverse_duty{0.0F};
  Milliseconds tower_pieces_reverse_duration_ms{0};
  float tower_pieces_shimmy_duty{0.0F};
  Milliseconds tower_pieces_shimmy_right_duration_ms{0};
  Milliseconds tower_pieces_shimmy_left_duration_ms{0};
  Milliseconds tower_pieces_shimmy_timeout_ms{0};
  float tower_pieces_final_reverse_duty{0.0F};
  Milliseconds tower_pieces_final_reverse_duration_ms{0};
  Milliseconds tower_pieces_post_final_reverse_delay_ms{0};
  Milliseconds tower_pieces_post_winch_open_delay_ms{0};
  Milliseconds tower_pieces_post_claws_open_delay_ms{0};
  Milliseconds tower_pieces_pre_stepper_bottom_delay_ms{0};
  std::uint32_t tower_pieces_stepper_down_speed_steps_per_second{0};
  Milliseconds tower_pieces_post_stepper_bottom_delay_ms{0};
  Milliseconds tower_pieces_post_claws_closed_delay_ms{0};
  std::uint32_t tower_pieces_stepper_up_speed_steps_per_second{0};
  std::uint8_t tower_pieces_side_line_count{0};
  std::uint16_t tower_pieces_side_line_rejected_count{0};
  std::uint8_t tower_pieces_target_side_line_count{
      kTowerPiecesTargetSideLineCount};
  bool tower_pieces_side_line_sensor_configured{false};
  bool tower_pieces_side_line_sensor_high{false};
  bool tower_pieces_side_line_armed{false};
  bool tower_pieces_side_line_detection_accepted{false};
  bool tower_pieces_side_line_detection_rejected{false};
  bool tower_pieces_line_following{false};
  bool tower_pieces_strafing_right{false};
  bool tower_pieces_rotating_clockwise{false};
  bool tower_pieces_driving_backward{false};
  bool tower_pieces_shimmying_left{false};
  bool tower_pieces_shimmying_right{false};
  bool tower_pieces_back_line_detected{false};
  bool tower_pieces_final_reverse_active{false};
  bool tower_pieces_stepper_moving_down{false};
  bool tower_pieces_stepper_moving_up{false};

  PegFinderState peg_finder_state{PegFinderState::WaitForStart};
  PegFinderFaultReason peg_finder_fault_reason{
      PegFinderFaultReason::None};
  Milliseconds peg_finder_time_in_state_ms{0};
  float peg_finder_clockwise_duty{0.0F};
  float peg_finder_clockwise_angle_deg{0.0F};
  Milliseconds peg_finder_post_rotation_pause_ms{0};
  float peg_finder_reverse_duty{0.0F};
  Milliseconds peg_finder_reverse_duration_ms{0};
  Milliseconds peg_finder_post_reverse_pause_ms{0};
  float peg_finder_forward_duty{0.0F};
  Milliseconds peg_finder_forward_duration_ms{0};
  float peg_finder_funnel_forward_duty{0.0F};
  Milliseconds peg_finder_funnel_forward_timeout_ms{0};
  Milliseconds peg_finder_post_funnel_limit_delay_ms{0};
  Milliseconds peg_finder_claw_open_interval_ms{0};
  std::uint8_t peg_finder_claw_open_order_1{1U};
  std::uint8_t peg_finder_claw_open_order_2{2U};
  std::uint8_t peg_finder_claw_open_order_3{3U};
  Milliseconds peg_finder_post_claws_open_delay_ms{0};
  float peg_finder_funnel_reverse_duty{0.0F};
  Milliseconds peg_finder_funnel_reverse_duration_ms{0};
  bool peg_finder_funnel_limit_configured{false};
  bool peg_finder_funnel_limit_high{false};
  bool peg_finder_rotating_clockwise{false};
  bool peg_finder_driving_backward{false};
  bool peg_finder_driving_forward{false};
  bool peg_finder_funnel_forward{false};
  bool peg_finder_funnel_reverse{false};
  bool peg_finder_opening_claw_1{false};
  bool peg_finder_opening_claw_2{false};
  bool peg_finder_opening_claw_3{false};

  TimeTrialState time_trial_state{TimeTrialState::WaitForStart};
  Milliseconds time_trial_time_in_state_ms{0};
  Milliseconds time_trial_post_solar_delay_ms{0};
  float time_trial_strafe_right_duty{0.0F};
  Milliseconds time_trial_strafe_right_duration_ms{0};
  Milliseconds time_trial_post_tower_delay_ms{0};
  bool time_trial_strafing_right{false};

  MotorTelemetry front_left{};
  MotorTelemetry front_right{};
  MotorTelemetry funnel{};
  RearCommandTelemetry rear{};
  Esp1RemoteStatusTelemetry esp1{};
  UltrasonicTelemetry ultrasonic_1{};
  LaserDistanceTelemetry laser_distance{};
  ServoClawBankTelemetry claws{};
  SolarHookServoTelemetry solar_hook{};

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

  // Legacy flat values remain available to the smaller dedicated API
  // handlers, but are no longer duplicated in the full telemetry document.
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
