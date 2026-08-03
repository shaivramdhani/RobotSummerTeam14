#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

constexpr float kDefaultHabitatPiecesLineFollowDuty = 0.12F;
// One millimetre above every configurable uint16_t threshold. A fresh
// no-target VL53L0X result is represented by this value so it remains outside
// the distance-counting "at or below threshold" zone without masquerading as
// a nearby target.
constexpr std::uint32_t kHabitatPiecesNoTargetDistanceMm =
    static_cast<std::uint32_t>(UINT16_MAX) + 1U;

enum class HabitatPiecesState : std::uint8_t {
  WaitForStart = 0,
  LineFollowing = 1,
  Reversing = 2,
  Complete = 3,
  Fault = 4,
  SideLineAligning = 5,
  DistanceStrafing = 6,
  PostCountStopDelay = 7,
  ExitStrafePulse = 8,
  ExitDistanceCheck = 9,
  LowerSlide = 10,
  ApproachPiece = 11,
  ReverseAfterPickup = 12,
  RearLineReacquire = 13,
  WaitForLiftCompletion = 14,
  LiftStartDelay = 15,
};

enum class HabitatPiecesStopReason : std::uint8_t {
  None = 0,
  ConfigurationIncomplete = 1,
  SideSensorsUnavailable = 2,
  SideSensorDataStale = 3,
  BothSideLinesDetected = 4,
  RunTimeout = 5,
  FrontLineLost = 6,
  RearCommandFailed = 7,
  DistanceZoneCountReached = 8,
  DistanceStrafeTimeout = 9,
  Lss2Detected = 10,
  ImuUnavailable = 11,
  ImuStrafeFailed = 12,
  DistanceExitReached = 13,
  SlideDownTimeout = 14,
  ApproachDistanceTimeout = 15,
  LiftTimeout = 16,
  RearLineTimeout = 17,
  StepperCommandFailed = 18,
  RearLineReached = 19,
};

enum class HabitatPiecesStrafeDirection : std::int8_t {
  None = 0,
  Left = -1,
  Right = 1,
};

struct HabitatPiecesConfig {
  float line_follow_duty{kDefaultHabitatPiecesLineFollowDuty};
  Milliseconds lss2_detection_delay_ms{0U};
  Milliseconds run_timeout_ms{0U};
  float reverse_duty{0.0F};
  Milliseconds reverse_duration_ms{0U};
  HabitatPiecesStrafeDirection distance_strafe_direction{
      HabitatPiecesStrafeDirection::None};
  std::uint16_t distance_threshold_mm{0U};
  std::uint16_t distance_zone_target_count{0U};
  float distance_strafe_duty{0.0F};
  Milliseconds distance_strafe_timeout_ms{0U};
  Milliseconds post_count_stop_delay_ms{0U};
  Milliseconds exit_strafe_pulse_ms{0U};
  std::uint32_t slide_down_speed_steps_per_second{0U};
  Milliseconds slide_down_timeout_ms{0U};
  std::uint16_t approach_distance_threshold_mm{0U};
  float approach_forward_duty{0.0F};
  Milliseconds approach_timeout_ms{0U};
  std::uint32_t lift_steps{0U};
  std::uint32_t lift_speed_steps_per_second{0U};
  Milliseconds lift_timeout_ms{0U};
  Milliseconds lift_start_delay_ms{0U};
  float post_pickup_reverse_duty{0.0F};
  Milliseconds post_pickup_reverse_duration_ms{0U};
  Milliseconds rear_line_reacquire_timeout_ms{0U};
};

struct HabitatPiecesDistanceSample {
  bool available{false};
  std::uint32_t distance_mm{0U};
  std::uint16_t measurement_sequence{0U};
  bool substituted_no_target{false};
};

struct HabitatPiecesMechanismInputs {
  bool bottom_limit_active{false};
  bool lift_complete{false};
  bool rear_line_available{false};
  bool rear_left_black{false};
  bool rear_right_black{false};
};

struct HabitatPiecesAutonomy {
  HabitatPiecesState state{HabitatPiecesState::WaitForStart};
  HabitatPiecesStopReason stop_reason{
      HabitatPiecesStopReason::ConfigurationIncomplete};
  Milliseconds state_entered_at_ms{0U};
  Milliseconds run_started_at_ms{0U};
  Milliseconds run_elapsed_ms{0U};
  Milliseconds reverse_elapsed_ms{0U};
  Milliseconds distance_strafe_started_at_ms{0U};
  Milliseconds distance_strafe_elapsed_ms{0U};
  Milliseconds post_count_stop_elapsed_ms{0U};
  Milliseconds exit_strafe_pulse_elapsed_ms{0U};
  Milliseconds slide_down_elapsed_ms{0U};
  Milliseconds approach_elapsed_ms{0U};
  Milliseconds lift_started_at_ms{0U};
  Milliseconds lift_elapsed_ms{0U};
  Milliseconds lift_start_delay_elapsed_ms{0U};
  Milliseconds post_pickup_reverse_elapsed_ms{0U};
  Milliseconds rear_line_reacquire_elapsed_ms{0U};
  // The existing delay arms the LSS2 stop gate. LSS3 remains telemetry-only.
  bool lss2_detection_armed{false};
  bool lss2_latched{false};
  bool lss3_latched{false};
  std::uint16_t distance_zone_count{0U};
  std::uint16_t distance_exit_pulse_count{0U};
  std::uint16_t last_distance_measurement_sequence{0U};
  std::uint32_t latest_distance_mm{0U};
  bool distance_sequence_initialized{false};
  bool distance_measurement_available{false};
  bool distance_substituted_no_target{false};
  bool distance_zone_active{false};
  bool distance_exit_above_threshold{false};
  bool approach_distance_reached{false};
  bool lift_complete{false};
  bool rear_line_latched{false};
  bool timed_out{false};
};

struct HabitatPiecesUpdate {
  HabitatPiecesState state{HabitatPiecesState::WaitForStart};
  HabitatPiecesStopReason stop_reason{
      HabitatPiecesStopReason::ConfigurationIncomplete};
  bool should_stop{true};
  bool should_line_follow{false};
  bool should_align_side_lines{false};
  bool should_drive_left_side{false};
  bool should_drive_right_side{false};
  bool should_reverse{false};
  bool should_distance_strafe{false};
  bool should_wait_after_distance_count{false};
  bool should_exit_strafe_pulse{false};
  bool should_check_exit_distance{false};
  bool should_lower_slide{false};
  bool should_drive_forward_to_piece{false};
  bool should_start_lift{false};
  bool should_wait_after_lift_start{false};
  bool should_drive_back_after_pickup{false};
  bool should_reacquire_rear_line{false};
  bool should_wait_for_lift{false};
  bool should_start_habitat_placement{false};
  bool lss2_detection_armed{false};
  bool lss2_black{false};
  bool lss3_black{false};
  bool lss2_latched{false};
  bool lss3_latched{false};
  bool lss2_newly_latched{false};
  bool lss3_newly_latched{false};
  bool distance_measurement_available{false};
  bool distance_sample_new{false};
  bool distance_zone_active{false};
  bool distance_zone_entered{false};
  std::uint32_t distance_mm{0U};
  std::uint16_t distance_zone_count{0U};
  std::uint16_t distance_exit_pulse_count{0U};
  bool distance_substituted_no_target{false};
  bool distance_exit_above_threshold{false};
  bool approach_distance_reached{false};
  bool lift_complete{false};
  bool rear_line_detected{false};
  bool target_reached{false};
  bool transitioned{false};
};

const char* habitatPiecesStateName(HabitatPiecesState state);
const char* habitatPiecesStopReasonName(HabitatPiecesStopReason reason);
const char* habitatPiecesStrafeDirectionName(
    HabitatPiecesStrafeDirection direction);
bool habitatPiecesConfigValid(const HabitatPiecesConfig& config,
                              float maximum_duty,
                              Milliseconds maximum_distance_strafe_timeout_ms);
void resetHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                                Milliseconds now_ms);
void startHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                                Milliseconds now_ms);
void failHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                               HabitatPiecesStopReason reason,
                               Milliseconds now_ms);
HabitatPiecesUpdate updateHabitatPiecesAutonomy(
    HabitatPiecesAutonomy& autonomy, const HabitatPiecesConfig& config,
    bool lss2_black, bool lss3_black, Milliseconds now_ms,
    const HabitatPiecesDistanceSample& distance_sample = {},
    const HabitatPiecesMechanismInputs& mechanism_inputs = {});
}  // namespace robot
