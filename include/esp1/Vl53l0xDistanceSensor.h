#pragma once

#include <cstdint>

#include <Adafruit_VL53L0X.h>

#include "common/LaserDistance.h"
#include "common/Units.h"
#include "esp1/PinConfig.h"

class TwoWire;

namespace robot::esp1 {

class Vl53l0xDistanceSensor {
 public:
  bool initialize(TwoWire& wire,
                  const I2cDistanceSensorConfig& config);
  bool setProfile(const I2cDistanceSensorConfig& config,
                  LaserDistanceProfile profile);
  bool service(Milliseconds now_ms);

  const LaserDistanceSnapshot& snapshot() const;

 private:
  void recordFailure(Milliseconds now_ms, std::int8_t driver_status);
  void completeAttempt(Milliseconds now_ms,
                       std::uint32_t acquisition_started_us,
                       bool valid);

  Adafruit_VL53L0X sensor_{};
  LaserDistanceSnapshot snapshot_{};
  bool initialized_{false};
  bool ranging_{false};
  LaserDistanceProfile profile_{LaserDistanceProfile::HighAccuracy};
  std::uint32_t default_timing_budget_us_{0U};
  std::uint32_t default_signal_rate_limit_{0U};
  std::uint32_t default_sigma_limit_{0U};
  std::uint8_t default_pre_range_vcsel_period_{0U};
  std::uint8_t default_final_range_vcsel_period_{0U};
  bool default_profile_captured_{false};
};

}  // namespace robot::esp1
