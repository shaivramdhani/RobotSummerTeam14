#include "common/HabitatDistanceStop.h"

namespace robot {

namespace {

Milliseconds elapsedSince(const Milliseconds now_ms,
                          const Milliseconds then_ms) {
  return now_ms - then_ms;
}

}  // namespace

bool habitatDistanceStopConfigValid(
    const HabitatDistanceStopConfig& config) {
  return config.stop_distance_mm > 0U &&
         config.maximum_sample_age_ms > 0U;
}

void resetHabitatDistanceStop(HabitatDistanceStopState& state) {
  state = {};
}

HabitatDistanceStopUpdate updateHabitatDistanceStop(
    HabitatDistanceStopState& state,
    const HabitatDistanceStopConfig& config,
    const LaserDistanceSnapshot* const snapshot,
    const Milliseconds measurement_received_at_ms,
    const Milliseconds now_ms) {
  HabitatDistanceStopUpdate update{};
  update.configuration_valid =
      habitatDistanceStopConfigValid(config);
  update.target_reached = state.target_reached_latched;
  if (!update.configuration_valid) {
    update.reason =
        HabitatDistanceStopReason::ConfigurationIncomplete;
    return update;
  }
  if (state.target_reached_latched) {
    update.reason =
        HabitatDistanceStopReason::TargetDistanceReached;
    return update;
  }
  if (snapshot == nullptr) {
    update.reason = HabitatDistanceStopReason::SnapshotUnavailable;
    return update;
  }

  update.sample_age_ms =
      elapsedSince(now_ms, measurement_received_at_ms);
  if (!snapshot->configured || !snapshot->initialized ||
      !snapshot->ranging) {
    update.reason = HabitatDistanceStopReason::SensorUnavailable;
    return update;
  }
  if (update.sample_age_ms > config.maximum_sample_age_ms) {
    update.reason = HabitatDistanceStopReason::MeasurementStale;
    return update;
  }
  if (!snapshot->data_valid ||
      snapshot->sensor_range_status !=
          kVl53l0xValidRangeStatus) {
    update.reason = HabitatDistanceStopReason::MeasurementInvalid;
    return update;
  }

  update.measurement_available = true;
  if (snapshot->distance_mm <= config.stop_distance_mm) {
    state.target_reached_latched = true;
    update.reason =
        HabitatDistanceStopReason::TargetDistanceReached;
    update.should_stop = true;
    update.target_reached = true;
    return update;
  }

  update.reason = HabitatDistanceStopReason::None;
  update.should_stop = false;
  return update;
}

const char* habitatDistanceStopReasonName(
    const HabitatDistanceStopReason reason) {
  switch (reason) {
    case HabitatDistanceStopReason::None:
      return "NONE";
    case HabitatDistanceStopReason::ConfigurationIncomplete:
      return "CONFIGURATION_INCOMPLETE";
    case HabitatDistanceStopReason::SnapshotUnavailable:
      return "SNAPSHOT_UNAVAILABLE";
    case HabitatDistanceStopReason::SensorUnavailable:
      return "SENSOR_UNAVAILABLE";
    case HabitatDistanceStopReason::MeasurementInvalid:
      return "MEASUREMENT_INVALID";
    case HabitatDistanceStopReason::MeasurementStale:
      return "MEASUREMENT_STALE";
    case HabitatDistanceStopReason::TargetDistanceReached:
      return "TARGET_DISTANCE_REACHED";
  }
  return "UNKNOWN";
}

}  // namespace robot
