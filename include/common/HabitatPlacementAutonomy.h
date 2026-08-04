#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class HabitatPlacementState : std::uint8_t {
  WaitForStart = 0,
  ReverseLineFollow = 1,
  PostLss1Delay = 2,
  TurnCounterClockwise = 3,
  ForwardToSlide = 4,
  LowerSlide = 5,
  OpenPusher = 6,
  PostPusherOpenDelay = 7,
  ForwardPush = 8,
  ReverseRetreat = 9,
  TurnClockwise = 10,
  PostClockwiseDelay = 11,
  ForwardExit = 12,
  PostForwardDelay = 13,
  StrafeRightToFrontLine = 14,
  ClosePusher = 15,
  Complete = 16,
  Fault = 17,
  ReverseAfterClockwise = 18,
  StrafeLeftAfterClockwise = 19,
  TurnToInitialHeading = 20,
  StrafeRightBeforeCounterClockwise = 21,
  StrafeRightAfterClockwise = 22,
};

enum class HabitatPlacementFaultReason : std::uint8_t {
  None = 0,
  HardwareNotReady = 1,
  RearLinkStale = 2,
  RearLineDataStale = 3,
  LineLost = 4,
  Lss1Timeout = 5,
  DriveCommandFailed = 6,
  ImuUnavailable = 7,
  ImuTurnFailed = 8,
  ImuTurnTimeout = 9,
  StepperCommandFailed = 10,
  StepperTimeout = 11,
  ConflictingLimitSwitches = 12,
  PusherCommandFailed = 13,
  FrontLineTimeout = 14,
};

struct HabitatPlacementConfig {
  // TODO(team): configure every field from telemetry after hardware tests.
  float reverse_line_follow_duty{0.0F};
  Milliseconds lss1_timeout_ms{0U};
  Milliseconds post_lss1_delay_ms{0U};
  Milliseconds initial_heading_turn_timeout_ms{0U};
  float pre_counter_clockwise_strafe_right_duty{0.0F};
  Milliseconds pre_counter_clockwise_strafe_right_duration_ms{0U};
  float counter_clockwise_angle_deg{0.0F};
  Milliseconds counter_clockwise_timeout_ms{0U};
  float forward_to_slide_duty{0.0F};
  Milliseconds forward_to_slide_duration_ms{0U};
  std::uint32_t stepper_down_speed_steps_per_second{0U};
  Milliseconds stepper_down_timeout_ms{0U};
  Milliseconds pusher_open_settle_ms{0U};
  float push_forward_duty{0.0F};
  Milliseconds push_forward_duration_ms{0U};
  float reverse_retreat_duty{0.0F};
  Milliseconds reverse_retreat_duration_ms{0U};
  float clockwise_angle_deg{0.0F};
  Milliseconds clockwise_timeout_ms{0U};
  float post_clockwise_reverse_duty{0.0F};
  Milliseconds post_clockwise_reverse_duration_ms{0U};
  float post_clockwise_strafe_left_duty{0.0F};
  Milliseconds post_clockwise_strafe_left_duration_ms{0U};
  Milliseconds post_clockwise_strafe_right_duration_ms{0U};
  Milliseconds post_clockwise_delay_ms{0U};
  float exit_forward_duty{0.0F};
  Milliseconds exit_forward_duration_ms{0U};
  Milliseconds post_forward_delay_ms{0U};
  float strafe_right_duty{0.0F};
  Milliseconds strafe_right_timeout_ms{0U};
};

struct HabitatPlacementInputs {
  bool lss1_black{false};
  bool initial_heading_turn_complete{false};
  bool counter_clockwise_turn_complete{false};
  bool bottom_limit_active{false};
  bool top_limit_active{false};
  bool pusher_open_commanded{false};
  bool clockwise_turn_complete{false};
  bool front_line_black{false};
  bool pusher_closed_commanded{false};
};

struct HabitatPlacementAutonomy {
  HabitatPlacementState state{HabitatPlacementState::WaitForStart};
  HabitatPlacementFaultReason fault_reason{
      HabitatPlacementFaultReason::None};
  Milliseconds state_entered_at_ms{0U};
  Milliseconds started_at_ms{0U};
  // Captured once, before the initial rear-line motion is enabled.
  bool initial_heading_captured{false};
  float initial_heading_deg{0.0F};
  float counter_clockwise_target_heading_deg{0.0F};
};

struct HabitatPlacementUpdate {
  HabitatPlacementState state{HabitatPlacementState::WaitForStart};
  HabitatPlacementFaultReason fault_reason{
      HabitatPlacementFaultReason::None};
  Milliseconds time_in_state_ms{0U};
  bool should_reverse_line_follow{false};
  bool should_turn_to_initial_heading{false};
  bool should_strafe_right_before_counter_clockwise{false};
  bool should_turn_counter_clockwise{false};
  bool should_drive_forward_to_slide{false};
  bool should_lower_slide{false};
  bool should_open_pusher{false};
  bool should_drive_forward_push{false};
  bool should_drive_reverse_retreat{false};
  bool should_turn_clockwise{false};
  bool should_drive_reverse_after_clockwise{false};
  bool should_strafe_left_after_clockwise{false};
  bool should_strafe_right_after_clockwise{false};
  bool should_drive_forward_exit{false};
  bool should_strafe_right{false};
  bool should_close_pusher{false};
  bool should_stop_drive{true};
  bool complete{false};
  bool faulted{false};
};

const char* habitatPlacementStateName(HabitatPlacementState state);
const char* habitatPlacementFaultReasonName(
    HabitatPlacementFaultReason reason);
bool habitatPlacementConfigValid(
    const HabitatPlacementConfig& config, float maximum_allowed_duty,
    std::uint32_t maximum_stepper_speed_steps_per_second);
void resetHabitatPlacementAutonomy(HabitatPlacementAutonomy& autonomy,
                                  Milliseconds now_ms);
bool startHabitatPlacementAutonomy(
    HabitatPlacementAutonomy& autonomy, float initial_heading_deg,
    float counter_clockwise_relative_angle_deg, Milliseconds now_ms);
void failHabitatPlacementAutonomy(HabitatPlacementAutonomy& autonomy,
                                 HabitatPlacementFaultReason reason,
                                 Milliseconds now_ms);
HabitatPlacementUpdate updateHabitatPlacementAutonomy(
    HabitatPlacementAutonomy& autonomy,
    const HabitatPlacementInputs& inputs,
    const HabitatPlacementConfig& config, Milliseconds now_ms);

}  // namespace robot
