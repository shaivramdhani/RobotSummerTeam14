#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class SolarPanelAutonomyState : std::uint8_t {
  WaitForStart = 0,
  LineFollowToSolar = 1,
  SolarBeaconAligned = 2,
  SolarSearchFault = 3,
  StrafeRightToSolarPanel = 4,
  SolarPanelContacted = 5,
  StrafeLeftForSolarRetry = 6,
  MoveForwardForSolarRetry = 7,
  RetryStrafeRightToSolarPanel = 8,
  MoveForwardAfterSolarContact = 9,
  StrafeLeftToRearLine = 10,
  RearLineReacquired = 11,
  WaitBeforeStrafeLeftToRearLine = 12,
};

enum class SolarPanelFaultReason : std::uint8_t {
  None = 0,
  SearchTimeout = 1,
  HardwareNotReady = 2,
  LineLost = 3,
  RearLinkStale = 4,
  LimitSwitchTimeout = 5,
};

struct SolarPanelAutonomyConfig {
  std::uint16_t detection_threshold{0};
  std::uint16_t release_threshold{0};
  Milliseconds confirmation_time_ms{0};
  float filter_alpha{0.0F};
  Milliseconds ignore_after_start_ms{0};
  Milliseconds search_timeout_ms{0};
};

struct SolarPanelContactConfig {
  Milliseconds timeout_ms{0};
  float strafe_duty{0.0F};
  Milliseconds strafe_start_delay_ms{0};
  Milliseconds retry_strafe_left_duration_ms{0};
  Milliseconds retry_forward_duration_ms{0};
  Milliseconds retry_strafe_timeout_ms{0};
  Milliseconds post_contact_forward_duration_ms{0};
  float line_reacquire_strafe_duty{0.0F};
  Milliseconds post_contact_forward_start_delay_ms{0};
  Milliseconds line_reacquire_strafe_start_delay_ms{0};
  float post_contact_forward_duty{0.0F};
};

struct SolarPanelContactSequenceUpdate {
  SolarPanelAutonomyState next_state{
      SolarPanelAutonomyState::WaitForStart};
  bool transitioned{false};
};

struct SolarBeaconDetectorState {
  bool filter_initialized{false};
  float filtered_amplitude{0.0F};
  bool confirmation_active{false};
  Milliseconds confirmation_started_at_ms{0};
  Milliseconds confirmation_progress_ms{0};
  bool beacon_detected{false};
};

struct SolarBeaconDetectorUpdate {
  std::uint16_t raw_amplitude{0};
  float filtered_amplitude{0.0F};
  bool confirmation_active{false};
  Milliseconds confirmation_progress_ms{0};
  bool beacon_detected{false};
};

const char* solarPanelAutonomyStateName(SolarPanelAutonomyState state);
const char* solarPanelFaultReasonName(SolarPanelFaultReason reason);

bool solarPanelAutonomyConfigValid(const SolarPanelAutonomyConfig& config);
bool solarPanelContactConfigValid(const SolarPanelContactConfig& config);
SolarPanelContactSequenceUpdate updateSolarPanelContactSequence(
    SolarPanelAutonomyState current_state, bool front_hit, bool back_hit,
    Milliseconds time_in_state_ms, const SolarPanelContactConfig& config);
SolarPanelContactSequenceUpdate updateSolarPanelContactSequence(
    SolarPanelAutonomyState current_state, bool front_hit, bool back_hit,
    bool reacquisition_line_detected, Milliseconds time_in_state_ms,
    const SolarPanelContactConfig& config);
void resetSolarBeaconDetectorState(SolarBeaconDetectorState& state);
SolarBeaconDetectorUpdate updateSolarBeaconDetector(
    SolarBeaconDetectorState& state, std::uint16_t raw_amplitude,
    const SolarPanelAutonomyConfig& config, Milliseconds now_ms,
    bool detection_permitted);

}  // namespace robot
