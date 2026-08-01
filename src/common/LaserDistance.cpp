#include "common/LaserDistance.h"

namespace robot {

namespace {

void putU16(std::uint8_t* const output, const std::uint16_t value) {
  output[0] = static_cast<std::uint8_t>(value & 0xFFU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void putU32(std::uint8_t* const output, const std::uint32_t value) {
  output[0] = static_cast<std::uint8_t>(value & 0xFFU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  output[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  output[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

std::uint16_t getU16(const std::uint8_t* const input) {
  return static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(input[0]) |
      (static_cast<std::uint16_t>(input[1]) << 8U));
}

std::uint32_t getU32(const std::uint8_t* const input) {
  return static_cast<std::uint32_t>(input[0]) |
         (static_cast<std::uint32_t>(input[1]) << 8U) |
         (static_cast<std::uint32_t>(input[2]) << 16U) |
         (static_cast<std::uint32_t>(input[3]) << 24U);
}

}  // namespace

UartPacket makeLaserDistancePacket(
    const LaserDistanceSnapshot& snapshot,
    const std::uint16_t sequence) {
  UartPacket packet{};
  packet.header.version = kUartProtocolVersion;
  packet.header.message_type = UartMessageType::LaserDistanceSnapshot;
  packet.header.sequence = sequence;
  packet.header.payload_size = kLaserDistancePayloadSize;
  putU32(&packet.payload[0], snapshot.captured_at_ms);
  putU16(&packet.payload[4], snapshot.distance_mm);
  putU16(&packet.payload[6], snapshot.measurement_sequence);
  std::uint8_t flags = 0U;
  flags |= snapshot.configured ? kLaserConfiguredFlag : 0U;
  flags |= snapshot.initialized ? kLaserInitializedFlag : 0U;
  flags |= snapshot.ranging ? kLaserRangingFlag : 0U;
  flags |= snapshot.data_valid ? kLaserDataValidFlag : 0U;
  flags |= snapshot.profile == LaserDistanceProfile::HighAccuracy
               ? kLaserHighAccuracyProfileFlag
               : 0U;
  packet.payload[8] = flags;
  packet.payload[9] = snapshot.sensor_range_status;
  packet.payload[10] =
      static_cast<std::uint8_t>(snapshot.driver_status);
  putU32(&packet.payload[11], snapshot.successful_measurement_count);
  putU32(&packet.payload[15], snapshot.failed_measurement_count);
  putU16(&packet.payload[19],
         snapshot.consecutive_failed_measurements);
  putU32(&packet.payload[21], snapshot.acquisition_duration_us);
  putU32(&packet.payload[25],
         snapshot.maximum_acquisition_duration_us);
  packet.payload[29] = static_cast<std::uint8_t>(snapshot.sda_gpio);
  packet.payload[30] = static_cast<std::uint8_t>(snapshot.scl_gpio);
  packet.payload[31] = snapshot.i2c_address;
  putU16(&packet.payload[32],
         snapshot.intermeasurement_period_ms);
  packet.header.integrity_crc16 = calculatePacketIntegrity(packet);
  return packet;
}

bool decodeLaserDistancePacket(
    const UartPacket& packet, LaserDistanceSnapshot& snapshot) {
  if (!packetLooksValid(packet) ||
      packet.header.message_type !=
          UartMessageType::LaserDistanceSnapshot ||
      packet.header.payload_size != kLaserDistancePayloadSize) {
    snapshot = {};
    return false;
  }

  snapshot.captured_at_ms = getU32(&packet.payload[0]);
  snapshot.distance_mm = getU16(&packet.payload[4]);
  snapshot.measurement_sequence = getU16(&packet.payload[6]);
  const std::uint8_t flags = packet.payload[8];
  snapshot.configured = (flags & kLaserConfiguredFlag) != 0U;
  snapshot.initialized = (flags & kLaserInitializedFlag) != 0U;
  snapshot.ranging = (flags & kLaserRangingFlag) != 0U;
  snapshot.data_valid = (flags & kLaserDataValidFlag) != 0U;
  snapshot.profile =
      (flags & kLaserHighAccuracyProfileFlag) != 0U
          ? LaserDistanceProfile::HighAccuracy
          : LaserDistanceProfile::Default;
  snapshot.sensor_range_status = packet.payload[9];
  snapshot.driver_status =
      static_cast<std::int8_t>(packet.payload[10]);
  snapshot.successful_measurement_count = getU32(&packet.payload[11]);
  snapshot.failed_measurement_count = getU32(&packet.payload[15]);
  snapshot.consecutive_failed_measurements =
      getU16(&packet.payload[19]);
  snapshot.acquisition_duration_us = getU32(&packet.payload[21]);
  snapshot.maximum_acquisition_duration_us =
      getU32(&packet.payload[25]);
  snapshot.sda_gpio = static_cast<std::int8_t>(packet.payload[29]);
  snapshot.scl_gpio = static_cast<std::int8_t>(packet.payload[30]);
  snapshot.i2c_address = packet.payload[31];
  snapshot.intermeasurement_period_ms = getU16(&packet.payload[32]);
  return true;
}

const char* laserDistanceProfileName(const LaserDistanceProfile profile) {
  switch (profile) {
    case LaserDistanceProfile::Default:
      return "DEFAULT";
    case LaserDistanceProfile::HighAccuracy:
      return "HIGH_ACCURACY";
  }
  return "DEFAULT";
}

}  // namespace robot
