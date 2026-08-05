#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

constexpr std::uint8_t kTowerPiecesTargetSideLineCount = 2U;

enum class TowerPiecesState : std::uint8_t {
  WaitForStart = 0,
  ReverseLineFollow = 1,
  PostLineDelay = 2,
  StrafeRight = 3,
  PostStrafePause = 4,
  RotateClockwise = 5,
  PostRotationPause = 6,
  ReverseTimed = 7,
  ShimmyLeft = 8,
  ShimmyRight = 9,
  Complete = 10,
  Fault = 11,
  FinalReverse = 12,
  PostFinalReverseDelay = 13,
  WinchOpen = 14,
  PostWinchOpenDelay = 15,
  ClawsOpen = 16,
  PostClawsOpenDelay = 17,
  MoveStepperBottom = 18,
  PostStepperBottomDelay = 19,
  ClawsClosed = 20,
  PostClawsClosedDelay = 21,
  MoveStepperTop = 22,
  WinchClosed = 23,
  PreStepperBottomDelay = 24,
  PreShimmyDelay = 25,
  PostShimmyDelay = 26,
};

enum class TowerPiecesShimmyInitialDirection : std::int8_t {
  Left = -1,
  Right = 1,
};

enum class TowerPiecesFaultReason : std::uint8_t {
  None = 0,
  HardwareNotReady = 1,
  RearLinkStale = 2,
  LineLost = 3,
  SideLineTimeout = 4,
  RearCommandFailed = 5,
  ShimmyTimeout = 6,
  ServoCommandFailed = 7,
  StepperCommandFailed = 8,
  StepperLimitSearchFailed = 9,
  ConflictingLimitSwitches = 10,
  ImuUnavailable = 11,
  ImuStrafeFailed = 12,
  ImuTurnFailed = 13,
  ImuTurnTimeout = 14,
};

struct TowerPiecesConfig {
  // TODO(team): set all values from the telemetry panel after hardware tests.
  float reverse_line_duty{0.0F};
  Milliseconds side_line_ignore_after_start_ms{0U};
  Milliseconds side_line_timeout_ms{0U};
  Milliseconds side_line_cooldown_ms{0U};
  Milliseconds side_line_rearm_ms{0U};
  Milliseconds post_line_delay_ms{1000U};
  float strafe_right_duty{0.0F};
  Milliseconds strafe_right_duration_ms{0U};
  Milliseconds post_strafe_pause_ms{1000U};
  float clockwise_rotation_angle_deg{0.0F};
  Milliseconds post_rotation_pause_ms{1000U};
  float reverse_duty{0.0F};
  Milliseconds reverse_duration_ms{0U};
  TowerPiecesShimmyInitialDirection shimmy_initial_direction{
      TowerPiecesShimmyInitialDirection::Right};
  Milliseconds pre_shimmy_delay_ms{0U};
  Milliseconds shimmy_right_duration_ms{0U};
  Milliseconds shimmy_left_duration_ms{0U};
  Milliseconds shimmy_timeout_ms{0U};
  float shimmy_duty{0.0F};
  Milliseconds post_shimmy_delay_ms{0U};
  float final_reverse_duty{0.0F};
  Milliseconds final_reverse_duration_ms{0U};
  Milliseconds post_final_reverse_delay_ms{1000U};
  Milliseconds post_winch_open_delay_ms{1000U};
  Milliseconds post_claws_open_delay_ms{1000U};
  // Relative upward jog that begins concurrently with the route.
  // Zero keeps Tower Pieces locked until the travel is calibrated.
  std::uint32_t initial_stepper_lift_steps{0U};
  Milliseconds pre_stepper_bottom_delay_ms{0U};
  std::uint32_t stepper_down_speed_steps_per_second{2000U};
  Milliseconds post_stepper_bottom_delay_ms{1000U};
  Milliseconds post_claws_closed_delay_ms{1000U};
  std::uint32_t stepper_up_speed_steps_per_second{2000U};
};

struct TowerPiecesInputs {
  bool side_line_high{false};
  bool back_left_line_high{false};
  bool back_right_line_high{false};
  bool bottom_limit_active{false};
  bool top_limit_active{false};
  bool clockwise_turn_complete{false};
  bool initial_stepper_lift_complete{false};
};

struct TowerPiecesAutonomy {
  TowerPiecesState state{TowerPiecesState::WaitForStart};
  TowerPiecesFaultReason fault_reason{TowerPiecesFaultReason::None};
  Milliseconds state_entered_at_ms{0U};
  Milliseconds started_at_ms{0U};
  Milliseconds shimmy_started_at_ms{0U};
  std::uint8_t side_line_count{0U};
  std::uint16_t side_line_rejected_count{0U};
  bool previous_side_line_high{false};
  bool side_line_armed{false};
  bool side_line_ignore_active{false};
  bool side_line_off_timing{false};
  bool last_side_line_detection_accepted{false};
  bool last_side_line_detection_rejected{false};
  Milliseconds side_line_off_started_at_ms{0U};
  Milliseconds side_line_last_accepted_at_ms{0U};
  bool first_shimmy_pulse{false};
  bool back_line_detected{false};
};

struct TowerPiecesUpdate {
  TowerPiecesState state{TowerPiecesState::WaitForStart};
  TowerPiecesFaultReason fault_reason{TowerPiecesFaultReason::None};
  std::uint8_t side_line_count{0U};
  std::uint16_t side_line_rejected_count{0U};
  bool side_line_rising_edge{false};
  bool side_line_detection_accepted{false};
  bool side_line_detection_rejected{false};
  bool side_line_armed{false};
  bool side_line_ignore_active{false};
  bool should_line_follow{false};
  bool should_initial_strafe_right{false};
  bool should_rotate_clockwise{false};
  bool should_drive_backward{false};
  bool should_shimmy_left{false};
  bool should_shimmy_right{false};
  bool back_line_detected{false};
  bool should_drive_final_reverse{false};
  bool waiting_for_initial_stepper_lift{false};
  bool should_move_stepper_bottom{false};
  bool should_move_stepper_top{false};
};

const char* towerPiecesStateName(TowerPiecesState state);
const char* towerPiecesShimmyInitialDirectionName(
    TowerPiecesShimmyInitialDirection direction);
const char* towerPiecesFaultReasonName(TowerPiecesFaultReason reason);
bool towerPiecesConfigValid(const TowerPiecesConfig& config,
                            float maximum_allowed_duty,
                            std::uint32_t maximum_stepper_speed_steps_per_second);
void resetTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                              Milliseconds now_ms);
void startTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                              bool side_line_high, Milliseconds now_ms);
TowerPiecesUpdate updateTowerPiecesAutonomy(
    TowerPiecesAutonomy& autonomy, const TowerPiecesInputs& inputs,
    const TowerPiecesConfig& config, Milliseconds now_ms);
void failTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                             TowerPiecesFaultReason reason,
                             Milliseconds now_ms);

}  // namespace robot
