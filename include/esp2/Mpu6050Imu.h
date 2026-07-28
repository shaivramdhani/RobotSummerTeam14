#pragma once

#include <cstdint>

class TwoWire;

namespace robot::esp2 {

enum class ImuInitializationError : std::uint8_t {
  None = 0,
  PinsUnassigned,
  BusBeginFailed,
  DeviceDidNotAcknowledge,
  WhoAmIReadFailed,
  UnsupportedWhoAmI,
  ResetWriteFailed,
  WakeWriteFailed,
  DlpfConfigWriteFailed,
  GyroConfigWriteFailed,
  AccelConfigWriteFailed,
  MeasurementReadFailed,
};

const char* imuInitializationErrorName(ImuInitializationError error);

struct ImuState {
  bool configured{false};
  bool initialized{false};
  bool calibrated{false};
  bool healthy{false};
  bool device_acknowledged{false};
  bool register_reads_use_repeated_start{true};

  std::uint8_t address{0x68U};
  std::uint8_t who_am_i{0U};
  int sda_gpio{-1};
  int scl_gpio{-1};
  int last_wire_status{-1};
  ImuInitializationError initialization_error{
      ImuInitializationError::PinsUnassigned};

  std::int16_t raw_gyro_z{0};
  float gyro_z_bias_dps{0.0F};
  float yaw_rate_dps{0.0F};
  float heading_deg{0.0F};

  std::uint32_t successful_read_count{0U};
  std::uint32_t failed_read_count{0U};
  std::uint32_t consecutive_failed_reads{0U};

  std::uint32_t last_update_us{0U};
  std::uint32_t last_successful_read_us{0U};
  std::uint32_t last_sample_interval_us{0U};
};

class Mpu6050Imu {
 public:
  bool begin(TwoWire& wire, int sda_gpio, int scl_gpio,
             std::uint8_t address = 0x68U);
  bool calibrateGyroZ(std::uint16_t sample_count);
  bool update();

  void resetHeading(float heading_deg = 0.0F);

  const ImuState& state() const;
  bool fresh(std::uint32_t now_us, std::uint32_t timeout_us) const;

 private:
  bool deviceAcknowledges();
  bool writeRegister(std::uint8_t register_address, std::uint8_t value);
  bool readRegisters(std::uint8_t start_register, std::uint8_t* output,
                     std::uint8_t length);
  bool readRawGyroZ(std::int16_t& raw_gyro_z);
  void recordReadFailure();

  TwoWire* wire_{nullptr};
  ImuState state_{};
};

}  // namespace robot::esp2
