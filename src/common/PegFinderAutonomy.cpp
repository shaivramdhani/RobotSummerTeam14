#include "common/PegFinderAutonomy.h"

#include <cmath>

namespace robot {

const char* pegFinderStateName(const PegFinderState state) {
  switch (state) {
    case PegFinderState::WaitForStart:
      return "WAIT_FOR_START";
    case PegFinderState::RotateClockwise:
      return "ROTATE_CLOCKWISE";
    case PegFinderState::PostRotationPause:
      return "POST_ROTATION_PAUSE";
    case PegFinderState::Reverse:
      return "REVERSE";
    case PegFinderState::PostReversePause:
      return "POST_REVERSE_PAUSE";
    case PegFinderState::Forward:
      return "FORWARD";
    case PegFinderState::FunnelForward:
      return "FUNNEL_FORWARD";
    case PegFinderState::PostFunnelLimitDelay:
      return "POST_FUNNEL_LIMIT_DELAY";
    case PegFinderState::OpenClaw1:
      return "OPEN_FIRST_CLAW";
    case PegFinderState::PostClaw1OpenDelay:
      return "POST_FIRST_CLAW_OPEN_DELAY";
    case PegFinderState::ShakeLeftAfterClaw1:
      return "SHAKE_LEFT_AFTER_FIRST_CLAW";
    case PegFinderState::ShakeRightAfterClaw1:
      return "SHAKE_RIGHT_AFTER_FIRST_CLAW";
    case PegFinderState::PostShakeAfterClaw1Delay:
      return "POST_SHAKE_DELAY_AFTER_FIRST_CLAW";
    case PegFinderState::OpenClaw2:
      return "OPEN_SECOND_CLAW";
    case PegFinderState::PostClaw2OpenDelay:
      return "POST_SECOND_CLAW_OPEN_DELAY";
    case PegFinderState::ShakeLeftAfterClaw2:
      return "SHAKE_LEFT_AFTER_SECOND_CLAW";
    case PegFinderState::ShakeRightAfterClaw2:
      return "SHAKE_RIGHT_AFTER_SECOND_CLAW";
    case PegFinderState::PostShakeAfterClaw2Delay:
      return "POST_SHAKE_DELAY_AFTER_SECOND_CLAW";
    case PegFinderState::OpenClaw3:
      return "OPEN_THIRD_CLAW";
    case PegFinderState::PostClawsOpenDelay:
      return "POST_CLAWS_OPEN_DELAY";
    case PegFinderState::FunnelReverse:
      return "FUNNEL_REVERSE";
    case PegFinderState::Complete:
      return "COMPLETE";
    case PegFinderState::Fault:
      return "FAULT";
  }
  return "WAIT_FOR_START";
}

const char* pegFinderFaultReasonName(const PegFinderFaultReason reason) {
  switch (reason) {
    case PegFinderFaultReason::None:
      return "NONE";
    case PegFinderFaultReason::HardwareNotReady:
      return "HARDWARE_NOT_READY";
    case PegFinderFaultReason::RearLinkStale:
      return "REAR_LINK_STALE";
    case PegFinderFaultReason::RearCommandFailed:
      return "REAR_COMMAND_FAILED";
    case PegFinderFaultReason::FunnelCommandFailed:
      return "FUNNEL_COMMAND_FAILED";
    case PegFinderFaultReason::FunnelLimitTimeout:
      return "FUNNEL_LIMIT_TIMEOUT";
    case PegFinderFaultReason::ServoCommandFailed:
      return "SERVO_COMMAND_FAILED";
    case PegFinderFaultReason::ImuUnavailable:
      return "IMU_UNAVAILABLE";
    case PegFinderFaultReason::ImuTurnFailed:
      return "IMU_TURN_FAILED";
    case PegFinderFaultReason::ImuTurnTimeout:
      return "IMU_TURN_TIMEOUT";
  }
  return "NONE";
}

bool pegFinderConfigValid(const PegFinderConfig& config,
                          const float maximum_drive_duty,
                          const float maximum_funnel_duty) {
  const bool claw_order_valid =
      config.claw_open_order_1 >= 1U &&
      config.claw_open_order_1 <= 3U &&
      config.claw_open_order_2 >= 1U &&
      config.claw_open_order_2 <= 3U &&
      config.claw_open_order_3 >= 1U &&
      config.claw_open_order_3 <= 3U &&
      config.claw_open_order_1 != config.claw_open_order_2 &&
      config.claw_open_order_1 != config.claw_open_order_3 &&
      config.claw_open_order_2 != config.claw_open_order_3;
  const bool shake_disabled =
      config.shake_duty == 0.0F &&
      config.shake_left_duration_ms == 0U &&
      config.shake_right_duration_ms == 0U;
  const bool shake_enabled_and_valid =
      std::isfinite(config.shake_duty) && config.shake_duty > 0.0F &&
      config.shake_duty <= maximum_drive_duty &&
      config.shake_left_duration_ms > 0U &&
      config.shake_right_duration_ms > 0U;
  return std::isfinite(config.reverse_duty) &&
         std::isfinite(config.forward_duty) &&
         std::isfinite(config.funnel_forward_duty) &&
         std::isfinite(config.funnel_reverse_duty) &&
         std::isfinite(config.clockwise_angle_deg) &&
         std::isfinite(maximum_drive_duty) &&
         std::isfinite(maximum_funnel_duty) &&
         config.reverse_duty > 0.0F &&
         config.reverse_duty <= maximum_drive_duty &&
         config.forward_duty > 0.0F &&
         config.forward_duty <= maximum_drive_duty &&
         config.funnel_forward_duty > 0.0F &&
         config.funnel_forward_duty <= maximum_funnel_duty &&
         config.funnel_reverse_duty > 0.0F &&
         config.funnel_reverse_duty <= maximum_funnel_duty &&
         config.clockwise_angle_deg > 0.0F &&
         config.post_rotation_pause_ms > 0U &&
         config.reverse_duration_ms > 0U &&
         config.post_reverse_pause_ms > 0U &&
         config.forward_duration_ms > 0U &&
         config.funnel_forward_timeout_ms > 0U &&
         config.post_funnel_limit_delay_ms > 0U &&
         config.claw_open_interval_ms > 0U &&
         (shake_disabled || shake_enabled_and_valid) &&
         claw_order_valid &&
         config.post_claws_open_delay_ms > 0U &&
         config.funnel_reverse_duration_ms > 0U;
}

namespace {

std::uint8_t clawForOpenStage(const PegFinderConfig& config,
                              const PegFinderState state) {
  switch (state) {
    case PegFinderState::OpenClaw1:
      return config.claw_open_order_1;
    case PegFinderState::OpenClaw2:
      return config.claw_open_order_2;
    case PegFinderState::OpenClaw3:
      return config.claw_open_order_3;
    default:
      return 0U;
  }
}

}  // namespace

void resetPegFinderAutonomy(PegFinderAutonomy& autonomy,
                            const Milliseconds now_ms) {
  autonomy = {};
  autonomy.state_entered_at_ms = now_ms;
}

void startPegFinderAutonomy(PegFinderAutonomy& autonomy,
                            const Milliseconds now_ms) {
  autonomy.state = PegFinderState::RotateClockwise;
  autonomy.fault_reason = PegFinderFaultReason::None;
  autonomy.state_entered_at_ms = now_ms;
}

void failPegFinderAutonomy(PegFinderAutonomy& autonomy,
                           const PegFinderFaultReason reason,
                           const Milliseconds now_ms) {
  autonomy.state = PegFinderState::Fault;
  autonomy.fault_reason = reason;
  autonomy.state_entered_at_ms = now_ms;
}

PegFinderUpdate updatePegFinderAutonomy(PegFinderAutonomy& autonomy,
                                        const PegFinderInputs& inputs,
                                        const PegFinderConfig& config,
                                        const Milliseconds now_ms) {
  const Milliseconds elapsed_ms = now_ms - autonomy.state_entered_at_ms;
  switch (autonomy.state) {
    case PegFinderState::RotateClockwise:
      if (inputs.clockwise_turn_complete) {
        autonomy.state = PegFinderState::PostRotationPause;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::PostRotationPause:
      if (elapsed_ms >= config.post_rotation_pause_ms) {
        autonomy.state = PegFinderState::Reverse;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::Reverse:
      if (elapsed_ms >= config.reverse_duration_ms) {
        autonomy.state = PegFinderState::PostReversePause;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::PostReversePause:
      if (elapsed_ms >= config.post_reverse_pause_ms) {
        autonomy.state = PegFinderState::Forward;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::Forward:
      if (elapsed_ms >= config.forward_duration_ms) {
        autonomy.state =
            inputs.funnel_limit_active
                ? PegFinderState::PostFunnelLimitDelay
                : PegFinderState::FunnelForward;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::FunnelForward:
      if (inputs.funnel_limit_active) {
        autonomy.state = PegFinderState::PostFunnelLimitDelay;
        autonomy.state_entered_at_ms = now_ms;
        autonomy.fault_reason = PegFinderFaultReason::None;
      } else if (config.funnel_forward_timeout_ms == 0U ||
                 elapsed_ms >= config.funnel_forward_timeout_ms) {
        failPegFinderAutonomy(autonomy,
                              PegFinderFaultReason::FunnelLimitTimeout,
                              now_ms);
      }
      break;
    case PegFinderState::PostFunnelLimitDelay:
      if (elapsed_ms >= config.post_funnel_limit_delay_ms) {
        autonomy.state = PegFinderState::OpenClaw1;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::OpenClaw1:
      autonomy.state = PegFinderState::PostClaw1OpenDelay;
      autonomy.state_entered_at_ms = now_ms;
      break;
    case PegFinderState::PostClaw1OpenDelay:
      if (elapsed_ms >= config.claw_open_interval_ms) {
        autonomy.state = config.shake_duty > 0.0F
                             ? PegFinderState::ShakeLeftAfterClaw1
                             : PegFinderState::OpenClaw2;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::ShakeLeftAfterClaw1:
      if (elapsed_ms >= config.shake_left_duration_ms) {
        autonomy.state = PegFinderState::ShakeRightAfterClaw1;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::ShakeRightAfterClaw1:
      if (elapsed_ms >= config.shake_right_duration_ms) {
        autonomy.state = PegFinderState::PostShakeAfterClaw1Delay;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::PostShakeAfterClaw1Delay:
      if (elapsed_ms >= config.post_shake_delay_ms) {
        autonomy.state = PegFinderState::OpenClaw2;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::OpenClaw2:
      autonomy.state = PegFinderState::PostClaw2OpenDelay;
      autonomy.state_entered_at_ms = now_ms;
      break;
    case PegFinderState::PostClaw2OpenDelay:
      if (elapsed_ms >= config.claw_open_interval_ms) {
        autonomy.state = config.shake_duty > 0.0F
                             ? PegFinderState::ShakeLeftAfterClaw2
                             : PegFinderState::OpenClaw3;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::ShakeLeftAfterClaw2:
      if (elapsed_ms >= config.shake_left_duration_ms) {
        autonomy.state = PegFinderState::ShakeRightAfterClaw2;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::ShakeRightAfterClaw2:
      if (elapsed_ms >= config.shake_right_duration_ms) {
        autonomy.state = PegFinderState::PostShakeAfterClaw2Delay;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::PostShakeAfterClaw2Delay:
      if (elapsed_ms >= config.post_shake_delay_ms) {
        autonomy.state = PegFinderState::OpenClaw3;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::OpenClaw3:
      autonomy.state = PegFinderState::PostClawsOpenDelay;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.fault_reason = PegFinderFaultReason::None;
      break;
    case PegFinderState::PostClawsOpenDelay:
      if (elapsed_ms >= config.post_claws_open_delay_ms) {
        autonomy.state = PegFinderState::FunnelReverse;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::FunnelReverse:
      if (elapsed_ms >= config.funnel_reverse_duration_ms) {
        autonomy.state = PegFinderState::Complete;
        autonomy.state_entered_at_ms = now_ms;
        autonomy.fault_reason = PegFinderFaultReason::None;
      }
      break;
    case PegFinderState::WaitForStart:
    case PegFinderState::Complete:
    case PegFinderState::Fault:
      break;
  }

  PegFinderUpdate update{};
  update.state = autonomy.state;
  update.fault_reason = autonomy.fault_reason;
  update.should_rotate_clockwise =
      autonomy.state == PegFinderState::RotateClockwise;
  update.should_drive_backward = autonomy.state == PegFinderState::Reverse;
  update.should_drive_forward = autonomy.state == PegFinderState::Forward;
  update.should_run_funnel_forward =
      autonomy.state == PegFinderState::FunnelForward;
  update.should_run_funnel_reverse =
      autonomy.state == PegFinderState::FunnelReverse;
  update.should_shake_left =
      autonomy.state == PegFinderState::ShakeLeftAfterClaw1 ||
      autonomy.state == PegFinderState::ShakeLeftAfterClaw2;
  update.should_shake_right =
      autonomy.state == PegFinderState::ShakeRightAfterClaw1 ||
      autonomy.state == PegFinderState::ShakeRightAfterClaw2;
  update.funnel_limit_detected = inputs.funnel_limit_active;
  const std::uint8_t claw_to_open =
      clawForOpenStage(config, autonomy.state);
  update.should_open_claw_1 = claw_to_open == 1U;
  update.should_open_claw_2 = claw_to_open == 2U;
  update.should_open_claw_3 = claw_to_open == 3U;
  return update;
}

}  // namespace robot
