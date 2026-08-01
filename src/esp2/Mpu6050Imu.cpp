#include "esp2/Mpu6050Imu.h"

#include <Arduino.h>
#include <Wire.h>

#include <cmath>
#include <cstddef>

namespace robot::esp2 {

namespace {

constexpr std::uint8_t kConfigRegister = 0x1AU;
constexpr std::uint8_t kGyroConfigRegister = 0x1BU;
constexpr std::uint8_t kAccelConfigRegister = 0x1CU;
constexpr std::uint8_t kAccelXoutHighRegister = 0x3BU;
constexpr std::uint8_t kPowerManagement1Register = 0x6BU;
constexpr std::uint8_t kWhoAmIRegister = 0x75U;

constexpr std::uint8_t kExpectedWhoAmI = 0x68U;
constexpr std::uint8_t kCompatibleWhoAmI = 0x74U;
constexpr std::uint8_t kDeviceReset = 0x80U;
constexpr std::uint8_t kGyroClockSource = 0x01U;
constexpr std::uint8_t kModerateDigitalLowPassFilter = 0x03U;
constexpr std::uint8_t kGyroRange500Dps = 0x08U;
constexpr std::uint8_t kAccelRange4G = 0x08U;
constexpr std::uint8_t kMeasurementByteCount = 14U;
constexpr std::uint8_t kGyroZHighByteOffset = 12U;
constexpr float kGyroLsbPerDps = 65.5F;
constexpr float kYawRateDeadbandDps = 0.0F;
constexpr std::uint32_t kMaximumIntegrationIntervalUs = 100000U;
constexpr std::uint32_t kUnhealthyConsecutiveReadCount = 3U;
constexpr std::uint16_t kI2cFrequencyKhz = 50U;
constexpr std::uint16_t kI2cTransactionTimeoutMs = 5U;
constexpr std::uint16_t kResetDelayMs = 100U;
constexpr std::uint16_t kWakeDelayMs = 10U;
constexpr std::uint16_t kCalibrationSampleDelayMs = 5U;
constexpr std::uint16_t kMinimumCalibrationSuccessPercent = 80U;

bool gpioAssigned(const int gpio) {
  return gpio >= 0;
}

bool supportedIdentity(const std::uint8_t identity) {
  return identity == kExpectedWhoAmI || identity == kCompatibleWhoAmI;
}

std::int16_t signedBigEndian(const std::uint8_t high,
                            const std::uint8_t low) {
  const std::uint16_t combined =
      (static_cast<std::uint16_t>(high) << 8U) |
      static_cast<std::uint16_t>(low);
  return static_cast<std::int16_t>(combined);
}

}  // namespace

const char* imuInitializationErrorName(
    const ImuInitializationError error) {
  switch (error) {
    case ImuInitializationError::None:
      return "NONE";
    case ImuInitializationError::PinsUnassigned:
      return "PINS_UNASSIGNED";
    case ImuInitializationError::BusBeginFailed:
      return "BUS_BEGIN_FAILED";
    case ImuInitializationError::DeviceDidNotAcknowledge:
      return "NO_DEVICE_ACK";
    case ImuInitializationError::WhoAmIReadFailed:
      return "WHO_AM_I_READ_FAILED";
    case ImuInitializationError::UnsupportedWhoAmI:
      return "UNSUPPORTED_WHO_AM_I";
    case ImuInitializationError::ResetWriteFailed:
      return "RESET_WRITE_FAILED";
    case ImuInitializationError::WakeWriteFailed:
      return "WAKE_WRITE_FAILED";
    case ImuInitializationError::DlpfConfigWriteFailed:
      return "DLPF_CONFIG_WRITE_FAILED";
    case ImuInitializationError::GyroConfigWriteFailed:
      return "GYRO_CONFIG_WRITE_FAILED";
    case ImuInitializationError::AccelConfigWriteFailed:
      return "ACCEL_CONFIG_WRITE_FAILED";
    case ImuInitializationError::MeasurementReadFailed:
      return "MEASUREMENT_READ_FAILED";
  }
  return "UNKNOWN";
}

const char* imuReadFailureReasonName(
    const ImuReadFailureReason reason) {
  switch (reason) {
    case ImuReadFailureReason::None:
      return "NONE";
    case ImuReadFailureReason::NotInitialized:
      return "NOT_INITIALIZED";
    case ImuReadFailureReason::NotCalibrated:
      return "NOT_CALIBRATED";
    case ImuReadFailureReason::I2cBufferOverflow:
      return "I2C_BUFFER_OVERFLOW";
    case ImuReadFailureReason::I2cAddressNack:
      return "I2C_ADDRESS_NACK";
    case ImuReadFailureReason::I2cDataNack:
      return "I2C_DATA_NACK";
    case ImuReadFailureReason::I2cBusError:
      return "I2C_BUS_ERROR";
    case ImuReadFailureReason::I2cTimeout:
      return "I2C_TIMEOUT";
    case ImuReadFailureReason::I2cWriteFailed:
      return "I2C_WRITE_FAILED";
    case ImuReadFailureReason::IncompleteRead:
      return "INCOMPLETE_READ";
    case ImuReadFailureReason::RuntimeConfigurationMismatch:
      return "RUNTIME_CONFIGURATION_MISMATCH";
    case ImuReadFailureReason::InvalidYawRate:
      return "INVALID_YAW";
    case ImuReadFailureReason::InvalidHeading:
      return "INVALID_HEADING";
    case ImuReadFailureReason::Unknown:
      return "UNKNOWN_READ_FAILURE";
  }
  return "UNKNOWN_READ_FAILURE";
}

bool Mpu6050Imu::begin(TwoWire& wire, const int sda_gpio,
                       const int scl_gpio, const std::uint8_t address) {
  wire_ = nullptr;
  state_ = {};
  state_.address = address;
  state_.sda_gpio = sda_gpio;
  state_.scl_gpio = scl_gpio;
  state_.configured = gpioAssigned(sda_gpio) && gpioAssigned(scl_gpio) &&
                      sda_gpio != scl_gpio;
  if (!state_.configured) {
    state_.initialization_error =
        ImuInitializationError::PinsUnassigned;
    return false;
  }

  wire_ = &wire;
  if (!wire_->begin(sda_gpio, scl_gpio, kI2cFrequencyKhz * 1000U)) {
    state_.initialization_error =
        ImuInitializationError::BusBeginFailed;
    wire_ = nullptr;
    return false;
  }
  wire_->setTimeOut(kI2cTransactionTimeoutMs);

  if (!deviceAcknowledges()) {
    state_.initialization_error =
        ImuInitializationError::DeviceDidNotAcknowledge;
    return false;
  }
  if (!readRegisters(kWhoAmIRegister, &state_.who_am_i, 1U)) {
    // Standard MPU-6050 transactions use a repeated start. Some compatible
    // modules only complete register reads when the register-address write
    // ends with STOP, so detect that once during safe startup and retain the
    // working mode. Steady-state reads never perform a fallback retry.
    state_.register_reads_use_repeated_start = false;
    if (!readRegisters(kWhoAmIRegister, &state_.who_am_i, 1U)) {
      state_.initialization_error =
          ImuInitializationError::WhoAmIReadFailed;
      return false;
    }
  }
  if (!supportedIdentity(state_.who_am_i)) {
    state_.initialization_error =
        ImuInitializationError::UnsupportedWhoAmI;
    return false;
  }

  if (!writeRegister(kPowerManagement1Register, kDeviceReset)) {
    state_.initialization_error =
        ImuInitializationError::ResetWriteFailed;
    return false;
  }
  delay(kResetDelayMs);

  if (!writeRegister(kPowerManagement1Register, kGyroClockSource)) {
    state_.initialization_error =
        ImuInitializationError::WakeWriteFailed;
    return false;
  }
  delay(kWakeDelayMs);

  if (!writeRegister(kConfigRegister,
                     kModerateDigitalLowPassFilter)) {
    state_.initialization_error =
        ImuInitializationError::DlpfConfigWriteFailed;
    return false;
  }
  if (!writeRegister(kGyroConfigRegister, kGyroRange500Dps)) {
    state_.initialization_error =
        ImuInitializationError::GyroConfigWriteFailed;
    return false;
  }
  if (!writeRegister(kAccelConfigRegister, kAccelRange4G)) {
    state_.initialization_error =
        ImuInitializationError::AccelConfigWriteFailed;
    return false;
  }

  std::uint8_t measurements[kMeasurementByteCount]{};
  if (!readRegisters(kAccelXoutHighRegister, measurements,
                     kMeasurementByteCount)) {
    state_.initialization_error =
        ImuInitializationError::MeasurementReadFailed;
    return false;
  }

  state_.initialized = true;
  state_.runtime_configuration_valid = true;
  state_.initialization_error = ImuInitializationError::None;
  return true;
}

bool Mpu6050Imu::calibrateGyroZ(const std::uint16_t sample_count) {
  state_.calibrated = false;
  state_.healthy = false;
  state_.gyro_z_bias_dps = 0.0F;
  state_.yaw_rate_dps = 0.0F;

  if (!state_.initialized || sample_count == 0U) {
    return false;
  }

  double sum_dps = 0.0;
  std::uint16_t successful_samples = 0U;
  for (std::uint16_t sample = 0U; sample < sample_count; ++sample) {
    std::int16_t raw_gyro_z = 0;
    if (readRawGyroZ(raw_gyro_z)) {
      sum_dps += static_cast<double>(raw_gyro_z) /
                 static_cast<double>(kGyroLsbPerDps);
      ++successful_samples;
      ++state_.successful_read_count;
      state_.consecutive_failed_reads = 0U;
      state_.last_successful_read_us = micros();
    } else {
      recordReadFailure(
          pending_read_failure_reason_ == ImuReadFailureReason::None
              ? ImuReadFailureReason::Unknown
              : pending_read_failure_reason_);
    }
    delay(kCalibrationSampleDelayMs);
  }

  const std::uint32_t success_percent =
      (static_cast<std::uint32_t>(successful_samples) * 100U) /
      sample_count;
  if (success_percent < kMinimumCalibrationSuccessPercent ||
      successful_samples == 0U) {
    return false;
  }

  state_.gyro_z_bias_dps =
      static_cast<float>(sum_dps / static_cast<double>(successful_samples));
  if (!std::isfinite(state_.gyro_z_bias_dps)) {
    state_.gyro_z_bias_dps = 0.0F;
    return false;
  }

  state_.calibrated = true;
  state_.healthy = true;
  state_.consecutive_failed_reads = 0U;
  state_.last_update_us = 0U;
  state_.last_sample_interval_us = 0U;
  resetHeading();
  return true;
}

bool Mpu6050Imu::update() {
  if (!state_.initialized) {
    state_.last_read_failure_reason =
        ImuReadFailureReason::NotInitialized;
    state_.last_read_failure_us = micros();
    state_.healthy = false;
    return false;
  }
  if (!state_.calibrated) {
    state_.last_read_failure_reason =
        ImuReadFailureReason::NotCalibrated;
    state_.last_read_failure_us = micros();
    state_.healthy = false;
    return false;
  }

  const bool recovering_from_read_failure =
      state_.consecutive_failed_reads > 0U;
  if (recovering_from_read_failure &&
      !runtimeConfigurationMatches()) {
    recordReadFailure(
        pending_read_failure_reason_ == ImuReadFailureReason::None
            ? ImuReadFailureReason::RuntimeConfigurationMismatch
            : pending_read_failure_reason_);
    return false;
  }

  std::int16_t raw_gyro_z = 0;
  if (!readRawGyroZ(raw_gyro_z)) {
    recordReadFailure(
        pending_read_failure_reason_ == ImuReadFailureReason::None
            ? ImuReadFailureReason::Unknown
            : pending_read_failure_reason_);
    return false;
  }

  const std::uint32_t now_us = micros();
  const std::uint32_t previous_update_us = state_.last_update_us;
  const std::uint32_t interval_us =
      previous_update_us == 0U || recovering_from_read_failure
          ? 0U
          : now_us - previous_update_us;

  state_.raw_gyro_z = raw_gyro_z;
  state_.last_update_us = now_us;
  state_.last_successful_read_us = now_us;
  state_.last_sample_interval_us = interval_us;
  ++state_.successful_read_count;
  state_.consecutive_failed_reads = 0U;
  state_.runtime_configuration_valid = true;

  float corrected_rate =
      static_cast<float>(raw_gyro_z) / kGyroLsbPerDps -
      state_.gyro_z_bias_dps;
  if (std::fabs(corrected_rate) < kYawRateDeadbandDps) {
    corrected_rate = 0.0F;
  }
  if (!std::isfinite(corrected_rate)) {
    state_.yaw_rate_dps = 0.0F;
    state_.last_read_failure_reason =
        ImuReadFailureReason::InvalidYawRate;
    state_.last_read_failure_us = micros();
    state_.healthy = false;
    return false;
  }
  state_.yaw_rate_dps = corrected_rate;

  if (interval_us > 0U && interval_us <= kMaximumIntegrationIntervalUs) {
    state_.heading_deg +=
        corrected_rate * (static_cast<float>(interval_us) / 1000000.0F);
  }
  if (!std::isfinite(state_.heading_deg)) {
    state_.heading_deg = 0.0F;
    state_.last_read_failure_reason =
        ImuReadFailureReason::InvalidHeading;
    state_.last_read_failure_us = micros();
    state_.healthy = false;
    return false;
  }

  state_.healthy = true;
  return true;
}

void Mpu6050Imu::resetHeading(const float heading_deg) {
  state_.heading_deg = std::isfinite(heading_deg) ? heading_deg : 0.0F;
}

const ImuState& Mpu6050Imu::state() const {
  return state_;
}

bool Mpu6050Imu::fresh(const std::uint32_t now_us,
                       const std::uint32_t timeout_us) const {
  return state_.initialized && state_.calibrated &&
         state_.last_successful_read_us != 0U &&
         now_us - state_.last_successful_read_us <= timeout_us;
}

bool Mpu6050Imu::deviceAcknowledges() {
  if (wire_ == nullptr) {
    return false;
  }
  wire_->beginTransmission(state_.address);
  state_.last_wire_status =
      static_cast<int>(wire_->endTransmission(true));
  state_.device_acknowledged = state_.last_wire_status == 0;
  return state_.device_acknowledged;
}

bool Mpu6050Imu::writeRegister(const std::uint8_t register_address,
                               const std::uint8_t value) {
  if (wire_ == nullptr) {
    return false;
  }
  wire_->beginTransmission(state_.address);
  if (wire_->write(register_address) != 1U ||
      wire_->write(value) != 1U) {
    state_.last_wire_status =
        static_cast<int>(wire_->endTransmission(true));
    return false;
  }
  state_.last_wire_status =
      static_cast<int>(wire_->endTransmission(true));
  return state_.last_wire_status == 0;
}

bool Mpu6050Imu::readRegisters(const std::uint8_t start_register,
                               std::uint8_t* const output,
                               const std::uint8_t length) {
  pending_read_failure_reason_ = ImuReadFailureReason::None;
  if (wire_ == nullptr || output == nullptr || length == 0U) {
    pending_read_failure_reason_ = ImuReadFailureReason::Unknown;
    return false;
  }

  const std::uint32_t lock_acquire_started_us = micros();
  wire_->beginTransmission(state_.address);
  state_.last_wire_lock_acquire_duration_us =
      static_cast<std::uint32_t>(
          micros() - lock_acquire_started_us);
  if (state_.last_wire_lock_acquire_duration_us >
      state_.maximum_wire_lock_acquire_duration_us) {
    state_.maximum_wire_lock_acquire_duration_us =
        state_.last_wire_lock_acquire_duration_us;
  }
  if (wire_->write(start_register) != 1U) {
    state_.last_wire_status =
        static_cast<int>(wire_->endTransmission(true));
    pending_read_failure_reason_ = ImuReadFailureReason::I2cWriteFailed;
    return false;
  }
  state_.last_wire_status =
      static_cast<int>(wire_->endTransmission(
          !state_.register_reads_use_repeated_start));
  if (state_.last_wire_status != 0) {
    switch (state_.last_wire_status) {
      case 1:
        pending_read_failure_reason_ =
            ImuReadFailureReason::I2cBufferOverflow;
        break;
      case 2:
        pending_read_failure_reason_ =
            ImuReadFailureReason::I2cAddressNack;
        break;
      case 3:
        pending_read_failure_reason_ =
            ImuReadFailureReason::I2cDataNack;
        break;
      case 5:
        pending_read_failure_reason_ =
            ImuReadFailureReason::I2cTimeout;
        break;
      default:
        pending_read_failure_reason_ =
            ImuReadFailureReason::I2cBusError;
        break;
    }
    return false;
  }

  const std::uint32_t read_started_us = micros();
  const std::size_t received =
      wire_->requestFrom(state_.address,
                         static_cast<std::size_t>(length), true);
  const std::uint32_t read_duration_us =
      static_cast<std::uint32_t>(micros() - read_started_us);
  if (received != length) {
    pending_read_failure_reason_ =
        read_duration_us >=
                static_cast<std::uint32_t>(
                    kI2cTransactionTimeoutMs) *
                    1000U
            ? ImuReadFailureReason::I2cTimeout
            : ImuReadFailureReason::IncompleteRead;
    while (wire_->available() > 0) {
      (void)wire_->read();
    }
    return false;
  }

  for (std::uint8_t index = 0U; index < length; ++index) {
    if (wire_->available() <= 0) {
      pending_read_failure_reason_ =
          ImuReadFailureReason::IncompleteRead;
      return false;
    }
    output[index] = static_cast<std::uint8_t>(wire_->read());
  }
  return true;
}

bool Mpu6050Imu::readRawGyroZ(std::int16_t& raw_gyro_z) {
  const std::uint32_t read_started_us = micros();
  std::uint8_t measurements[kMeasurementByteCount]{};
  const bool read_succeeded =
      readRegisters(kAccelXoutHighRegister, measurements,
                    kMeasurementByteCount);
  state_.last_measurement_read_duration_us =
      static_cast<std::uint32_t>(micros() - read_started_us);
  if (state_.last_measurement_read_duration_us >
      state_.maximum_measurement_read_duration_us) {
    state_.maximum_measurement_read_duration_us =
        state_.last_measurement_read_duration_us;
  }
  if (!read_succeeded) {
    return false;
  }
  raw_gyro_z =
      signedBigEndian(measurements[kGyroZHighByteOffset],
                      measurements[kGyroZHighByteOffset + 1U]);
  return true;
}

bool Mpu6050Imu::runtimeConfigurationMatches() {
  std::uint8_t identity = 0U;
  std::uint8_t filter_and_ranges[3]{};
  std::uint8_t power_management = 0U;
  const bool reads_succeeded =
      readRegisters(kWhoAmIRegister, &identity, 1U) &&
      readRegisters(kConfigRegister, filter_and_ranges,
                    sizeof(filter_and_ranges)) &&
      readRegisters(kPowerManagement1Register, &power_management, 1U);
  state_.runtime_configuration_valid =
      reads_succeeded && supportedIdentity(identity) &&
      filter_and_ranges[0] == kModerateDigitalLowPassFilter &&
      filter_and_ranges[1] == kGyroRange500Dps &&
      filter_and_ranges[2] == kAccelRange4G &&
      power_management == kGyroClockSource;
  if (reads_succeeded && !state_.runtime_configuration_valid) {
    pending_read_failure_reason_ =
        ImuReadFailureReason::RuntimeConfigurationMismatch;
  }
  return state_.runtime_configuration_valid;
}

void Mpu6050Imu::recordReadFailure(
    const ImuReadFailureReason reason) {
  ++state_.failed_read_count;
  ++state_.consecutive_failed_reads;
  state_.last_read_failure_reason =
      reason == ImuReadFailureReason::None
          ? ImuReadFailureReason::Unknown
          : reason;
  state_.last_read_failure_us = micros();
  if (state_.consecutive_failed_reads >= kUnhealthyConsecutiveReadCount) {
    state_.healthy = false;
  }
}

}  // namespace robot::esp2
