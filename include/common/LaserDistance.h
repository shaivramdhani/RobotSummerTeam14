#pragma once

#include <cstdint>

#include "common/UartProtocol.h"
#include "common/Units.h"

namespace robot {

constexpr std::uint8_t kVl53l0xDefaultI2cAddress = 0x29U;
constexpr std::uint8_t kVl53l0xValidRangeStatus = 0U;
constexpr std::uint16_t kLaserDistancePayloadSize = 34U;
constexpr std::uint8_t kLaserConfiguredFlag = 0x01U;
constexpr std::uint8_t kLaserInitializedFlag = 0x02U;
constexpr std::uint8_t kLaserRangingFlag = 0x04U;
constexpr std::uint8_t kLaserDataValidFlag = 0x08U;
constexpr std::uint8_t kLaserHighAccuracyProfileFlag = 0x10U;

enum class LaserDistanceProfile : std::uint8_t {
  Default = 0,
  HighAccuracy = 1,
};

constexpr LaserDistanceProfile kOperationalLaserDistanceProfile =
    LaserDistanceProfile::HighAccuracy;

struct LaserDistanceSnapshot {
  Milliseconds captured_at_ms{0U};
  std::uint16_t distance_mm{0U};
  std::uint16_t measurement_sequence{0U};
  bool configured{false};
  bool initialized{false};
  bool ranging{false};
  bool data_valid{false};
  LaserDistanceProfile profile{LaserDistanceProfile::Default};
  std::uint8_t sensor_range_status{0xFFU};
  std::int8_t driver_status{0};
  std::uint32_t successful_measurement_count{0U};
  std::uint32_t failed_measurement_count{0U};
  std::uint16_t consecutive_failed_measurements{0U};
  std::uint32_t acquisition_duration_us{0U};
  std::uint32_t maximum_acquisition_duration_us{0U};
  std::int8_t sda_gpio{-1};
  std::int8_t scl_gpio{-1};
  std::uint8_t i2c_address{kVl53l0xDefaultI2cAddress};
  std::uint16_t intermeasurement_period_ms{0U};
};

UartPacket makeLaserDistancePacket(const LaserDistanceSnapshot& snapshot,
                                   std::uint16_t sequence);
bool decodeLaserDistancePacket(const UartPacket& packet,
                               LaserDistanceSnapshot& snapshot);
const char* laserDistanceProfileName(LaserDistanceProfile profile);

}  // namespace robot
