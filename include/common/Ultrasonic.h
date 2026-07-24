#pragma once

#include <cstdint>

namespace robot {

// HC-SR04 data-sheet values. The echo timeout is derived from the maximum
// specified distance and the same 340 m/s sound velocity used for conversion.
constexpr std::uint32_t kHcSr04TriggerPulseUs = 10U;
constexpr std::uint32_t kHcSr04SoundVelocityMmPerSecond = 340000U;
constexpr std::uint16_t kHcSr04MinimumDistanceMm = 20U;
constexpr std::uint16_t kHcSr04MaximumDistanceMm = 4000U;
constexpr std::uint32_t kHcSr04EchoTimeoutUs =
    (static_cast<std::uint32_t>(kHcSr04MaximumDistanceMm) * 2000000UL +
     kHcSr04SoundVelocityMmPerSecond - 1U) /
    kHcSr04SoundVelocityMmPerSecond;

std::uint16_t hcSr04DistanceMmFromEchoUs(std::uint32_t echo_duration_us);
bool hcSr04DistanceMmIsValid(std::uint16_t distance_mm);

}  // namespace robot
