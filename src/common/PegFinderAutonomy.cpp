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
      return "OPEN_CLAW_1";
    case PegFinderState::PostClaw1OpenDelay:
      return "POST_CLAW_1_OPEN_DELAY";
    case PegFinderState::OpenClaw2:
      return "OPEN_CLAW_2";
    case PegFinderState::PostClaw2OpenDelay:
      return "POST_CLAW_2_OPEN_DELAY";
    case PegFinderState::OpenClaw3:
      return "OPEN_CLAW_3";
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
  }
  return "NONE";
}

bool pegFinderConfigValid(const PegFinderConfig& config,
                          const float maximum_drive_duty,
                          const float maximum_funnel_duty) {
  return std::isfinite(config.clockwise_duty) &&
         std::isfinite(config.reverse_duty) &&
         std::isfinite(config.forward_duty) &&
         std::isfinite(config.funnel_forward_duty) &&
         std::isfinite(maximum_drive_duty) &&
         std::isfinite(maximum_funnel_duty) &&
         config.clockwise_duty > 0.0F &&
         config.clockwise_duty <= maximum_drive_duty &&
         config.reverse_duty > 0.0F &&
         config.reverse_duty <= maximum_drive_duty &&
         config.forward_duty > 0.0F &&
         config.forward_duty <= maximum_drive_duty &&
         config.funnel_forward_duty > 0.0F &&
         config.funnel_forward_duty <= maximum_funnel_duty &&
         config.clockwise_duration_ms > 0U &&
         config.post_rotation_pause_ms > 0U &&
         config.reverse_duration_ms > 0U &&
         config.post_reverse_pause_ms > 0U &&
         config.forward_duration_ms > 0U &&
         config.funnel_forward_timeout_ms > 0U &&
         config.post_funnel_limit_delay_ms > 0U &&
         config.claw_open_interval_ms > 0U;
}

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
      if (elapsed_ms >= config.clockwise_duration_ms) {
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
        autonomy.state = PegFinderState::OpenClaw3;
        autonomy.state_entered_at_ms = now_ms;
      }
      break;
    case PegFinderState::OpenClaw3:
      autonomy.state = PegFinderState::Complete;
      autonomy.state_entered_at_ms = now_ms;
      autonomy.fault_reason = PegFinderFaultReason::None;
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
  update.funnel_limit_detected = inputs.funnel_limit_active;
  update.should_open_claw_1 =
      autonomy.state == PegFinderState::OpenClaw1;
  update.should_open_claw_2 =
      autonomy.state == PegFinderState::OpenClaw2;
  update.should_open_claw_3 =
      autonomy.state == PegFinderState::OpenClaw3;
  return update;
}

}  // namespace robot
