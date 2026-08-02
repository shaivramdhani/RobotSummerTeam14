#pragma once

#include <cstdint>

#include "common/MotorOutput.h"
#include "common/Units.h"

namespace robot {

constexpr float kDefaultHabitatPiecesLineFollowDuty = 0.12F;

enum class HabitatPiecesState : std::uint8_t {
  WaitForStart = 0,
  LineFollowing = 1,
  Reversing = 2,
  Complete = 3,
  Fault = 4,
  SideLineAligning = 5,
  DistanceStrafing = 6,
  CompensationStrafing = 7,
  LoweringSlide = 8,
  ForwardToDistance = 9,
  PostPickupReversing = 10,
  ReturnLineStrafing = 11,
  WaitForSlideLift = 12,
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
  SlideDownTimeout = 10,
  SlideCommandFailed = 11,
  ForwardDistanceTimeout = 12,
  SlideLiftTimeout = 13,
  RearLineDataStale = 14,
  RearLineSensorsUnavailable = 15,
  ReturnLineTimeout = 16,
  ReturnLineDetected = 17,
  ConflictingSlideLimits = 18,
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
  float compensation_strafe_duty{0.0F};
  Milliseconds compensation_strafe_duration_ms{0U};
  std::uint32_t slide_down_speed_steps_per_second{0U};
  Milliseconds slide_down_timeout_ms{0U};
  float forward_to_distance_duty{0.0F};
  std::uint16_t forward_stop_distance_mm{0U};
  Milliseconds forward_to_distance_timeout_ms{0U};
  std::uint32_t slide_lift_steps{0U};
  std::uint32_t slide_lift_speed_steps_per_second{0U};
  Milliseconds slide_lift_timeout_ms{0U};
  float post_pickup_reverse_duty{0.0F};
  Milliseconds post_pickup_reverse_duration_ms{0U};
  float return_strafe_duty{0.0F};
  Milliseconds return_line_timeout_ms{0U};
};

struct HabitatPiecesDistanceSample {
  bool available{false};
  std::uint16_t distance_mm{0U};
  std::uint16_t measurement_sequence{0U};
};

struct HabitatPiecesInputs {
  bool lss2_black{false};
  bool lss3_black{false};
  HabitatPiecesDistanceSample distance_sample{};
  bool slide_bottom_limit_active{false};
  bool slide_bottom_ready{false};
  bool slide_top_limit_active{false};
  bool slide_down_failed{false};
  bool slide_lift_complete{false};
  bool slide_lift_failed{false};
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
  Milliseconds distance_strafe_elapsed_ms{0U};
  Milliseconds compensation_strafe_elapsed_ms{0U};
  Milliseconds slide_down_elapsed_ms{0U};
  Milliseconds forward_to_distance_elapsed_ms{0U};
  Milliseconds slide_lift_started_at_ms{0U};
  Milliseconds slide_lift_elapsed_ms{0U};
  Milliseconds post_pickup_reverse_elapsed_ms{0U};
  Milliseconds return_strafe_elapsed_ms{0U};
  // The existing delay now arms both LSS2 and LSS3 together.
  bool lss2_detection_armed{false};
  bool lss2_latched{false};
  bool lss3_latched{false};
  std::uint16_t distance_zone_count{0U};
  std::uint16_t last_distance_measurement_sequence{0U};
  std::uint16_t latest_distance_mm{0U};
  bool distance_sequence_initialized{false};
  bool distance_measurement_available{false};
  bool distance_zone_active{false};
  bool slide_lift_started{false};
  bool slide_lift_complete{false};
  bool rear_line_detected{false};
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
  bool should_compensation_strafe{false};
  bool should_start_slide_down{false};
  bool should_lower_slide{false};
  bool should_drive_forward_to_distance{false};
  bool should_start_slide_lift{false};
  bool should_post_pickup_reverse{false};
  bool should_return_line_strafe{false};
  bool should_wait_for_slide_lift{false};
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
  std::uint16_t distance_mm{0U};
  std::uint16_t distance_zone_count{0U};
  bool forward_distance_reached{false};
  bool slide_bottom_ready{false};
  bool slide_lift_started{false};
  bool slide_lift_complete{false};
  bool rear_left_black{false};
  bool rear_right_black{false};
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
                              Milliseconds maximum_duration_ms,
                              std::uint32_t maximum_stepper_speed_steps_per_second,
                              std::uint32_t maximum_slide_position_steps);
void resetHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                                Milliseconds now_ms);
void startHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                                Milliseconds now_ms);
void failHabitatPiecesAutonomy(HabitatPiecesAutonomy& autonomy,
                               HabitatPiecesStopReason reason,
                               Milliseconds now_ms);
HabitatPiecesUpdate updateHabitatPiecesAutonomy(
    HabitatPiecesAutonomy& autonomy, const HabitatPiecesConfig& config,
    const HabitatPiecesInputs& inputs, Milliseconds now_ms);
HabitatPiecesUpdate updateHabitatPiecesAutonomy(
    HabitatPiecesAutonomy& autonomy, const HabitatPiecesConfig& config,
    bool lss2_black, bool lss3_black, Milliseconds now_ms,
    const HabitatPiecesDistanceSample& distance_sample = {});
FourWheelCommand makeHabitatPiecesSideAlignmentCommand(
    float forward_duty, bool lss2_left_latched,
    bool lss3_right_latched, Milliseconds now_ms,
    Milliseconds command_timeout_ms);
FourWheelCommand makeHabitatPiecesDistanceStrafeCommand(
    HabitatPiecesStrafeDirection direction, float duty,
    Milliseconds now_ms, Milliseconds command_timeout_ms);
HabitatPiecesStrafeDirection oppositeHabitatPiecesStrafeDirection(
    HabitatPiecesStrafeDirection direction);

}  // namespace robot
