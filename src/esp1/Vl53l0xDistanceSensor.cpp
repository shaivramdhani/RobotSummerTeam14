#include "esp1/Vl53l0xDistanceSensor.h"

#include <Arduino.h>
#include <Wire.h>

#include <limits>

namespace robot::esp1 {

namespace {

bool configValid(const I2cDistanceSensorConfig& config) {
  return config.sda_gpio >= 0 && config.scl_gpio >= 0 &&
         config.sda_gpio != config.scl_gpio &&
         config.address == kVl53l0xDefaultI2cAddress &&
         config.frequency_hz > 0U &&
         config.transaction_timeout_ms > 0U &&
         config.intermeasurement_period_ms > 0U;
}

std::int8_t driverStatus(const VL53L0X_Error status) {
  return static_cast<std::int8_t>(status);
}

Adafruit_VL53L0X::VL53L0X_Sense_config_t adafruitProfile(
    const LaserDistanceProfile profile) {
  return profile == LaserDistanceProfile::HighAccuracy
             ? Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY
             : Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT;
}

}  // namespace

bool Vl53l0xDistanceSensor::initialize(
    TwoWire& wire, const I2cDistanceSensorConfig& config) {
  snapshot_ = {};
  snapshot_.configured = configValid(config);
  snapshot_.sda_gpio = static_cast<std::int8_t>(config.sda_gpio);
  snapshot_.scl_gpio = static_cast<std::int8_t>(config.scl_gpio);
  snapshot_.i2c_address = config.address;
  snapshot_.intermeasurement_period_ms =
      config.intermeasurement_period_ms;
  if (!snapshot_.configured) {
    return false;
  }

  if (!wire.begin(config.sda_gpio, config.scl_gpio,
                  config.frequency_hz)) {
    snapshot_.driver_status =
        driverStatus(VL53L0X_ERROR_CONTROL_INTERFACE);
    return false;
  }
  wire.setTimeOut(config.transaction_timeout_ms);

  initialized_ = sensor_.begin(
      config.address, false, &wire,
      adafruitProfile(LaserDistanceProfile::Default));
  snapshot_.driver_status = driverStatus(sensor_.Status);
  snapshot_.initialized = initialized_;
  if (!initialized_) {
    return false;
  }

  default_signal_rate_limit_ = sensor_.getLimitCheckValue(
      VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);
  if (sensor_.Status != VL53L0X_ERROR_NONE) {
    snapshot_.driver_status = driverStatus(sensor_.Status);
    snapshot_.initialized = false;
    initialized_ = false;
    return false;
  }
  default_sigma_limit_ = sensor_.getLimitCheckValue(
      VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE);
  if (sensor_.Status != VL53L0X_ERROR_NONE) {
    snapshot_.driver_status = driverStatus(sensor_.Status);
    snapshot_.initialized = false;
    initialized_ = false;
    return false;
  }
  default_timing_budget_us_ =
      sensor_.getMeasurementTimingBudgetMicroSeconds();
  if (sensor_.Status != VL53L0X_ERROR_NONE) {
    snapshot_.driver_status = driverStatus(sensor_.Status);
    snapshot_.initialized = false;
    initialized_ = false;
    return false;
  }
  default_pre_range_vcsel_period_ = sensor_.getVcselPulsePeriod(
      VL53L0X_VCSEL_PERIOD_PRE_RANGE);
  if (sensor_.Status != VL53L0X_ERROR_NONE) {
    snapshot_.driver_status = driverStatus(sensor_.Status);
    snapshot_.initialized = false;
    initialized_ = false;
    return false;
  }
  default_final_range_vcsel_period_ = sensor_.getVcselPulsePeriod(
      VL53L0X_VCSEL_PERIOD_FINAL_RANGE);
  if (sensor_.Status != VL53L0X_ERROR_NONE) {
    snapshot_.driver_status = driverStatus(sensor_.Status);
    snapshot_.initialized = false;
    initialized_ = false;
    return false;
  }
  default_profile_captured_ = true;

  if (!sensor_.configSensor(
          adafruitProfile(LaserDistanceProfile::HighAccuracy))) {
    snapshot_.driver_status = driverStatus(sensor_.Status);
    snapshot_.initialized = false;
    initialized_ = false;
    return false;
  }

  ranging_ =
      sensor_.startRangeContinuous(config.intermeasurement_period_ms);
  snapshot_.driver_status = driverStatus(sensor_.Status);
  snapshot_.ranging = ranging_;
  snapshot_.profile = LaserDistanceProfile::HighAccuracy;
  profile_ = LaserDistanceProfile::HighAccuracy;
  return ranging_;
}

bool Vl53l0xDistanceSensor::setProfile(
    const I2cDistanceSensorConfig& config,
    const LaserDistanceProfile profile) {
  if (!snapshot_.configured || !configValid(config) || !initialized_ ||
      !default_profile_captured_) {
    return false;
  }
  if (initialized_ && ranging_ && profile_ == profile) {
    return true;
  }

  if (ranging_) {
    sensor_.stopRangeContinuous();
    snapshot_.driver_status = driverStatus(sensor_.Status);
    if (sensor_.Status != VL53L0X_ERROR_NONE) {
      ranging_ = false;
      snapshot_.ranging = false;
      snapshot_.data_valid = false;
      snapshot_.sensor_range_status = 0xFFU;
      return false;
    }
  }

  ranging_ = false;
  snapshot_.ranging = false;
  snapshot_.data_valid = false;
  snapshot_.distance_mm = 0U;
  snapshot_.sensor_range_status = 0xFFU;

  bool profile_configured = false;
  if (profile == LaserDistanceProfile::HighAccuracy) {
    profile_configured = sensor_.configSensor(
        adafruitProfile(LaserDistanceProfile::HighAccuracy));
  } else {
    profile_configured = sensor_.configSensor(
        adafruitProfile(LaserDistanceProfile::Default));
    profile_configured =
        profile_configured &&
        sensor_.setLimitCheckValue(
            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            default_signal_rate_limit_);
    profile_configured =
        profile_configured &&
        sensor_.setLimitCheckValue(
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
            default_sigma_limit_);
    profile_configured =
        profile_configured &&
        sensor_.setMeasurementTimingBudgetMicroSeconds(
            default_timing_budget_us_);
    profile_configured =
        profile_configured &&
        sensor_.setVcselPulsePeriod(
            VL53L0X_VCSEL_PERIOD_PRE_RANGE,
            default_pre_range_vcsel_period_);
    profile_configured =
        profile_configured &&
        sensor_.setVcselPulsePeriod(
            VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
            default_final_range_vcsel_period_);
  }
  snapshot_.driver_status = driverStatus(sensor_.Status);
  if (!profile_configured) {
    return false;
  }

  ranging_ = sensor_.startRangeContinuous(
      config.intermeasurement_period_ms);
  snapshot_.driver_status = driverStatus(sensor_.Status);
  snapshot_.ranging = ranging_;
  if (!ranging_) {
    return false;
  }

  profile_ = profile;
  snapshot_.profile = profile;
  return true;
}

bool Vl53l0xDistanceSensor::service(const Milliseconds now_ms) {
  if (!initialized_ || !ranging_) {
    return false;
  }

  const std::uint32_t acquisition_started_us = micros();
  const bool complete = sensor_.isRangeComplete();
  if (sensor_.Status != VL53L0X_ERROR_NONE) {
    recordFailure(now_ms, driverStatus(sensor_.Status));
    completeAttempt(now_ms, acquisition_started_us, false);
    return true;
  }
  if (!complete) {
    return false;
  }

  const std::uint16_t distance_mm = sensor_.readRangeResult();
  const std::uint8_t range_status = sensor_.readRangeStatus();
  const std::int8_t status = driverStatus(sensor_.Status);
  const bool valid =
      sensor_.Status == VL53L0X_ERROR_NONE &&
      range_status == kVl53l0xValidRangeStatus &&
      distance_mm != std::numeric_limits<std::uint16_t>::max();

  snapshot_.distance_mm = valid ? distance_mm : 0U;
  snapshot_.sensor_range_status = range_status;
  snapshot_.driver_status = status;
  if (!valid) {
    ++snapshot_.failed_measurement_count;
    ++snapshot_.consecutive_failed_measurements;
  } else {
    ++snapshot_.successful_measurement_count;
    snapshot_.consecutive_failed_measurements = 0U;
  }
  completeAttempt(now_ms, acquisition_started_us, valid);
  return true;
}

const LaserDistanceSnapshot& Vl53l0xDistanceSensor::snapshot() const {
  return snapshot_;
}

void Vl53l0xDistanceSensor::recordFailure(
    const Milliseconds now_ms, const std::int8_t status) {
  snapshot_.captured_at_ms = now_ms;
  snapshot_.distance_mm = 0U;
  snapshot_.data_valid = false;
  snapshot_.sensor_range_status = 0xFFU;
  snapshot_.driver_status = status;
  ++snapshot_.failed_measurement_count;
  ++snapshot_.consecutive_failed_measurements;
}

void Vl53l0xDistanceSensor::completeAttempt(
    const Milliseconds now_ms,
    const std::uint32_t acquisition_started_us,
    const bool valid) {
  snapshot_.captured_at_ms = now_ms;
  ++snapshot_.measurement_sequence;
  snapshot_.data_valid = valid;
  snapshot_.acquisition_duration_us =
      static_cast<std::uint32_t>(micros() - acquisition_started_us);
  if (snapshot_.acquisition_duration_us >
      snapshot_.maximum_acquisition_duration_us) {
    snapshot_.maximum_acquisition_duration_us =
        snapshot_.acquisition_duration_us;
  }
}

}  // namespace robot::esp1
