#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class PegFinderState : std::uint8_t {
  WaitForStart = 0,
  RotateClockwise = 1,
  PostRotationPause = 2,
  Reverse = 3,
  PostReversePause = 4,
  Forward = 5,
  FunnelForward = 6,
  Complete = 7,
  Fault = 8,
  PostFunnelLimitDelay = 9,
  OpenClaw1 = 10,
  PostClaw1OpenDelay = 11,
  OpenClaw2 = 12,
  PostClaw2OpenDelay = 13,
  OpenClaw3 = 14,
  PostClawsOpenDelay = 15,
  FunnelReverse = 16,
  ShakeLeftAfterClaw1 = 17,
  ShakeRightAfterClaw1 = 18,
  ShakeLeftAfterClaw2 = 19,
  ShakeRightAfterClaw2 = 20,
  PostShakeAfterClaw1Delay = 21,
  PostShakeAfterClaw2Delay = 22,
  ShakeLeftAfterClaw3 = 23,
  ShakeRightAfterClaw3 = 24,
  PostShakeAfterClaw3Delay = 25,
};

enum class PegFinderFaultReason : std::uint8_t {
  None = 0,
  HardwareNotReady = 1,
  RearLinkStale = 2,
  RearCommandFailed = 3,
  FunnelCommandFailed = 4,
  FunnelLimitTimeout = 5,
  ServoCommandFailed = 6,
  ImuUnavailable = 7,
  ImuTurnFailed = 8,
  ImuTurnTimeout = 9,
};

struct PegFinderConfig {
  float clockwise_angle_deg{0.0F};
  Milliseconds post_rotation_pause_ms{0U};
  float reverse_duty{0.0F};
  Milliseconds reverse_duration_ms{0U};
  Milliseconds post_reverse_pause_ms{0U};
  float forward_duty{0.0F};
  Milliseconds forward_duration_ms{0U};
  float funnel_forward_duty{0.0F};
  Milliseconds funnel_forward_timeout_ms{0U};
  Milliseconds post_funnel_limit_delay_ms{0U};
  Milliseconds claw_open_interval_ms{0U};
  // Zero for all three fields keeps shaking disabled until the team tunes it.
  float shake_duty{0.0F};
  Milliseconds shake_left_duration_ms{0U};
  Milliseconds shake_right_duration_ms{0U};
  Milliseconds post_shake_delay_ms{0U};
  std::uint8_t claw_open_order_1{1U};
  std::uint8_t claw_open_order_2{2U};
  std::uint8_t claw_open_order_3{3U};
  Milliseconds post_claws_open_delay_ms{0U};
  float funnel_reverse_duty{0.0F};
  Milliseconds funnel_reverse_duration_ms{0U};
};

struct PegFinderInputs {
  bool funnel_limit_active{false};
  bool clockwise_turn_complete{false};
};

struct PegFinderAutonomy {
  PegFinderState state{PegFinderState::WaitForStart};
  PegFinderFaultReason fault_reason{PegFinderFaultReason::None};
  Milliseconds state_entered_at_ms{0U};
};

struct PegFinderUpdate {
  PegFinderState state{PegFinderState::WaitForStart};
  PegFinderFaultReason fault_reason{PegFinderFaultReason::None};
  bool should_rotate_clockwise{false};
  bool should_drive_backward{false};
  bool should_drive_forward{false};
  bool should_run_funnel_forward{false};
  bool should_run_funnel_reverse{false};
  bool should_shake_left{false};
  bool should_shake_right{false};
  bool funnel_limit_detected{false};
  bool should_open_claw_1{false};
  bool should_open_claw_2{false};
  bool should_open_claw_3{false};
  bool should_close_claw_1{false};
  bool should_close_claw_2{false};
  bool should_close_claw_3{false};
};

const char* pegFinderStateName(PegFinderState state);
const char* pegFinderFaultReasonName(PegFinderFaultReason reason);
bool pegFinderConfigValid(const PegFinderConfig& config,
                          float maximum_drive_duty,
                          float maximum_funnel_duty);
void resetPegFinderAutonomy(PegFinderAutonomy& autonomy,
                            Milliseconds now_ms);
void startPegFinderAutonomy(PegFinderAutonomy& autonomy,
                            Milliseconds now_ms);
void failPegFinderAutonomy(PegFinderAutonomy& autonomy,
                           PegFinderFaultReason reason,
                           Milliseconds now_ms);
PegFinderUpdate updatePegFinderAutonomy(PegFinderAutonomy& autonomy,
                                        const PegFinderInputs& inputs,
                                        const PegFinderConfig& config,
                                        Milliseconds now_ms);

}  // namespace robot
