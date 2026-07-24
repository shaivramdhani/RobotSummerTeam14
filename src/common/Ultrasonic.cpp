#include "common/Ultrasonic.h"

#include <limits>

namespace robot {

std::uint16_t hcSr04DistanceMmFromEchoUs(
    const std::uint32_t echo_duration_us) {
  const std::uint64_t round_trip_distance_micrometers =
      static_cast<std::uint64_t>(echo_duration_us) *
      kHcSr04SoundVelocityMmPerSecond;
  const std::uint64_t distance_mm =
      (round_trip_distance_micrometers + 1000000ULL) / 2000000ULL;
  if (distance_mm > std::numeric_limits<std::uint16_t>::max()) {
    return std::numeric_limits<std::uint16_t>::max();
  }
  return static_cast<std::uint16_t>(distance_mm);
}

bool hcSr04DistanceMmIsValid(const std::uint16_t distance_mm) {
  return distance_mm >= kHcSr04MinimumDistanceMm &&
         distance_mm <= kHcSr04MaximumDistanceMm;
}

}  // namespace robot
