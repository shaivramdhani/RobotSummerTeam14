#pragma once

#include <cstdint>

#include "common/LaserDistance.h"
#include "common/Units.h"

namespace robot {

struct HabitatDistanceStopConfig {
  std::uint16_t stop_distance_mm{0U};
  Milliseconds maximum_sample_age_ms{0U};
};

enum class HabitatDistanceStopReason : std::uint8_t {
  None = 0,
  ConfigurationIncomplete = 1,
  SnapshotUnavailable = 2,
  SensorUnavailable = 3,
  MeasurementInvalid = 4,
  MeasurementStale = 5,
  TargetDistanceReached = 6,
};

struct HabitatDistanceStopState {
  bool target_reached_latched{false};
};

struct HabitatDistanceStopUpdate {
  HabitatDistanceStopReason reason{
      HabitatDistanceStopReason::ConfigurationIncomplete};
  Milliseconds sample_age_ms{0U};
  bool configuration_valid{false};
  bool measurement_available{false};
  bool should_stop{true};
  bool target_reached{false};
};

bool habitatDistanceStopConfigValid(
    const HabitatDistanceStopConfig& config);
void resetHabitatDistanceStop(HabitatDistanceStopState& state);
HabitatDistanceStopUpdate updateHabitatDistanceStop(
    HabitatDistanceStopState& state,
    const HabitatDistanceStopConfig& config,
    const LaserDistanceSnapshot* snapshot,
    Milliseconds measurement_received_at_ms, Milliseconds now_ms);
const char* habitatDistanceStopReasonName(
    HabitatDistanceStopReason reason);

}  // namespace robot
