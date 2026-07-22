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
};

enum class TowerPiecesFaultReason : std::uint8_t {
  None = 0,
  HardwareNotReady = 1,
  RearLinkStale = 2,
  LineLost = 3,
  SideLineTimeout = 4,
  RearCommandFailed = 5,
  ShimmyTimeout = 6,
};

struct TowerPiecesConfig {
  // TODO(team): set all values from the telemetry panel after hardware tests.
  float reverse_line_duty{0.0F};
  Milliseconds side_line_timeout_ms{0U};
  Milliseconds post_line_delay_ms{0U};
  float strafe_right_duty{0.0F};
  Milliseconds strafe_right_duration_ms{0U};
  Milliseconds post_strafe_pause_ms{0U};
  float clockwise_rotation_duty{0.0F};
  Milliseconds clockwise_rotation_duration_ms{0U};
  Milliseconds post_rotation_pause_ms{0U};
  float reverse_duty{0.0F};
  Milliseconds reverse_duration_ms{0U};
  float shimmy_duty{0.0F};
  Milliseconds shimmy_right_duration_ms{0U};
  Milliseconds shimmy_left_duration_ms{0U};
  Milliseconds shimmy_timeout_ms{0U};
};

struct TowerPiecesAutonomy {
  TowerPiecesState state{TowerPiecesState::WaitForStart};
  TowerPiecesFaultReason fault_reason{TowerPiecesFaultReason::None};
  Milliseconds state_entered_at_ms{0U};
  Milliseconds started_at_ms{0U};
  Milliseconds shimmy_started_at_ms{0U};
  std::uint8_t side_line_count{0U};
  bool previous_side_line_high{false};
};

struct TowerPiecesUpdate {
  TowerPiecesState state{TowerPiecesState::WaitForStart};
  TowerPiecesFaultReason fault_reason{TowerPiecesFaultReason::None};
  std::uint8_t side_line_count{0U};
  bool side_line_rising_edge{false};
  bool should_line_follow{false};
  bool should_initial_strafe_right{false};
  bool should_rotate_clockwise{false};
  bool should_drive_backward{false};
  bool should_shimmy_left{false};
  bool should_shimmy_right{false};
  bool back_line_detected{false};
};

const char* towerPiecesStateName(TowerPiecesState state);
const char* towerPiecesFaultReasonName(TowerPiecesFaultReason reason);
bool towerPiecesConfigValid(const TowerPiecesConfig& config,
                            float maximum_allowed_duty);
void resetTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                              Milliseconds now_ms);
void startTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                              bool side_line_high, Milliseconds now_ms);
TowerPiecesUpdate updateTowerPiecesAutonomy(
    TowerPiecesAutonomy& autonomy, bool side_line_high,
    bool back_left_line_high, bool back_right_line_high,
    const TowerPiecesConfig& config, Milliseconds now_ms);
void failTowerPiecesAutonomy(TowerPiecesAutonomy& autonomy,
                             TowerPiecesFaultReason reason,
                             Milliseconds now_ms);

}  // namespace robot
