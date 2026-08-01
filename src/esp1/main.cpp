#include <Arduino.h>
#include <Wire.h>

#include <freertos/queue.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "common/Esp1Status.h"
#include "common/FaultHealth.h"
#include "common/FunnelCommand.h"
#include "common/LaserDistance.h"
#include "common/MotorOutput.h"
#include "common/RearDriveCommand.h"
#include "common/RearLineSensor.h"
#include "common/SolarHookServo.h"
#include "common/Ultrasonic.h"
#include "common/UartProtocol.h"
#include "esp1/MissionStateMachine.h"
#include "esp1/PinConfig.h"
#include "esp1/Vl53l0xDistanceSensor.h"

namespace {

constexpr robot::Milliseconds kRearDriveTaskPeriodMs = 10U;
constexpr robot::Milliseconds kRearStatusPrintPeriodMs = 250U;
constexpr std::uint32_t kTaskStackBytes = 6144U;
constexpr UBaseType_t kTaskPriority = 1U;
constexpr BaseType_t kTaskCore = 1;
constexpr std::uint8_t kMaxUartPacketsPerCycle = 4U;

// IR beacon ADC monitor defaults. GPIO stays in PinConfig; tune these values
// here during bench testing. The analog input must remain within 0-3.3 V.
constexpr int kIrAdcGpio =
    robot::esp1::kHardwareConfig.pins.right_ir_filtered;
constexpr int kIrFrequencySelectGpio =
    robot::esp1::kHardwareConfig.pins.freq;
constexpr int kSolarLimitBackRightSideGpio =
    robot::esp1::kHardwareConfig.pins.limit_switch_back_right_side;
constexpr int kSolarLimitFrontRightSideGpio =
    robot::esp1::kHardwareConfig.pins.limit_switch_front_right_side;
constexpr int kSideLineSensorGpio =
    robot::esp1::kHardwareConfig.pins.line_sensor_side;
constexpr int kSideLineSensor2Gpio =
    robot::esp1::kHardwareConfig.pins.line_sensor_side_2;
constexpr int kSideLineSensor3Gpio =
    robot::esp1::kHardwareConfig.pins.line_sensor_side_3;
constexpr int kRearLineSensorLeftGpio =
    robot::esp1::kHardwareConfig.pins.line_sensor_back_left;
constexpr int kRearLineSensorRightGpio =
    robot::esp1::kHardwareConfig.pins.line_sensor_back_right;
constexpr int kUltrasonic1TriggerGpio =
    robot::esp1::kHardwareConfig.pins.ultrasonic_trigger_1;
constexpr int kUltrasonic1EchoGpio =
    robot::esp1::kHardwareConfig.pins.ultrasonic_echo_1;
constexpr std::uint8_t kIrAdcResolutionBits = 12U;
constexpr std::uint16_t kIrAdcMaximumSample =
    (static_cast<std::uint16_t>(1U) << kIrAdcResolutionBits) - 1U;
constexpr std::uint32_t kIrAdcSampleRateHz = 50000U;
constexpr robot::Milliseconds kIrAdcWindowMs = 20U;
constexpr std::uint16_t kIrAdcSamplesPerWindow = 1000U;
constexpr std::uint32_t kIrAdcSampleIntervalUs =
    1000000UL / kIrAdcSampleRateHz;
constexpr std::uint16_t kIrAdcSamplesPerYield = 50U;
constexpr bool kIrDebugTelemetryMode = false;
constexpr robot::Milliseconds kEsp1StatusPeriodMs =
    kIrDebugTelemetryMode ? 50U : 100U;
constexpr robot::Milliseconds kSensorServicePeriodMs = 10U;
constexpr robot::Milliseconds kIrSerialDebugPeriodMs = 100U;
constexpr std::uint32_t kSensorTaskStackBytes = 8192U;
constexpr UBaseType_t kSensorTaskPriority = 0U;
constexpr BaseType_t kSensorTaskCore = 0;

constexpr std::uint32_t kIrBeaconFrequency1Khz = 1000U;
constexpr std::uint32_t kIrBeaconFrequency10Khz = 10000U;
constexpr int kIrSwitchRawStateFor1Khz = HIGH;
constexpr int kIrSwitchRawStateFor10Khz = LOW;
constexpr robot::Milliseconds kIrSwitchDebounceMs = 30U;

// TODO: calibrate these with switch-selected beacon off/on and motors running.
constexpr std::uint16_t IR_THRESHOLD_1KHZ = 120U;
constexpr std::uint16_t IR_THRESHOLD_1KHZ_OFF = 80U;
constexpr std::uint16_t IR_THRESHOLD_10KHZ = 120U;
constexpr std::uint16_t IR_THRESHOLD_10KHZ_OFF = 80U;
constexpr std::uint8_t kIrConsecutiveDetectWindowsRequired = 3U;
constexpr std::uint8_t kIrConsecutiveClearWindowsRequired = 3U;
constexpr float kIrPi = 3.14159265358979323846F;

// Validation notes:
// 1) Switch HIGH, expose the sensor to a 1 kHz beacon, and confirm the 1 kHz
//    amplitude rises and detection turns on after the confirmation windows.
// 2) In 1 kHz mode, confirm a 10 kHz beacon does not cross the active threshold.
// 3) Switch LOW and repeat for a 10 kHz beacon.
// 4) Block the beacon and confirm detection clears after the clear windows.
// 5) While driving, confirm rear_status continues and motor commands stay live.

static_assert(kIrAdcSampleRateHz > 0U, "IR ADC sample rate must be nonzero");
static_assert(kIrAdcSamplesPerWindow > 0U,
              "IR ADC samples per window must be nonzero");
static_assert(kIrAdcSampleRateHz * kIrAdcWindowMs / 1000U ==
                  kIrAdcSamplesPerWindow,
              "IR ADC sample count must match sample rate and window");
static_assert(IR_THRESHOLD_1KHZ_OFF <= IR_THRESHOLD_1KHZ,
              "1 kHz IR off threshold must be <= on threshold");
static_assert(IR_THRESHOLD_10KHZ_OFF <= IR_THRESHOLD_10KHZ,
              "10 kHz IR off threshold must be <= on threshold");
static_assert(robot::kHcSr04EchoTimeoutUs > 0U,
              "HC-SR04 echo timeout must be nonzero");
static_assert(robot::kHcSr04EchoTimeoutUs < 100000U,
              "HC-SR04 echo timeout must remain bounded");

struct IrAdcWindowResult {
  std::uint16_t average{0};
  std::uint16_t minimum{0};
  std::uint16_t maximum{0};
  std::uint16_t amplitude_pp{0};
  std::uint16_t latest_sample{0};
  std::uint16_t amplitude_1khz{0};
  std::uint16_t amplitude_10khz{0};
  std::uint16_t selected_amplitude{0};
  std::uint16_t active_threshold{0};
  std::uint32_t selected_frequency_hz{kIrBeaconFrequency1Khz};
  std::uint32_t achieved_sample_rate_hz{0};
  robot::Milliseconds sampled_at_ms{0};
  std::uint8_t consecutive_detection_count{0};
  bool switch_raw_high{true};
  bool switch_debounced_high{true};
  bool beacon_detected{false};
  bool configured{false};
};

struct DebouncedSwitchState {
  bool raw_high{true};
  bool debounced_high{true};
  bool candidate_high{true};
  robot::Milliseconds candidate_since_ms{0};
};

struct SolarPanelLimitSwitchReading {
  bool configured{false};
  bool back_right_high{false};
  bool front_right_high{false};
};

struct UltrasonicReading {
  std::uint16_t distance_mm{0};
  std::uint32_t echo_duration_us{0};
  bool configured{false};
  bool echo_valid{false};
};

portMUX_TYPE g_ir_adc_result_mux = portMUX_INITIALIZER_UNLOCKED;
IrAdcWindowResult g_latest_ir_adc_result{};
std::uint16_t g_ir_adc_samples[kIrAdcSamplesPerWindow]{};
portMUX_TYPE g_ultrasonic_result_mux = portMUX_INITIALIZER_UNLOCKED;
UltrasonicReading g_latest_ultrasonic_1_reading{};
StaticQueue_t g_laser_snapshot_queue_buffer{};
std::uint8_t g_laser_snapshot_queue_storage[
    sizeof(robot::LaserDistanceSnapshot)]{};
QueueHandle_t g_laser_snapshot_queue{nullptr};
StaticQueue_t g_laser_profile_request_queue_buffer{};
std::uint8_t g_laser_profile_request_queue_storage[
    sizeof(robot::LaserDistanceProfile)]{};
QueueHandle_t g_laser_profile_request_queue{nullptr};

bool gpioAssigned(const int gpio) {
  return gpio >= 0;
}

bool motorConfigComplete(
    const robot::esp1::DualPwmMotorOutputConfig& config) {
  return gpioAssigned(config.pwm0_gpio) && gpioAssigned(config.pwm1_gpio) &&
         config.pwm0_channel >= 0 && config.pwm1_channel >= 0 &&
         config.pwm_frequency_hz > 0U && config.pwm_resolution_bits > 0U &&
         config.pwm_resolution_bits < 31U &&
         (config.forward_sign == 1 || config.forward_sign == -1) &&
         config.h_bridge_mode !=
             robot::esp1::DualPwmHBridgeMode::Unconfigured;
}

bool servoConfigComplete(
    const robot::esp1::ServoOutputConfig& config) {
  if (!gpioAssigned(config.gpio) || config.pwm_channel < 0 ||
      config.pwm_frequency_hz == 0U ||
      config.pwm_resolution_bits == 0U ||
      config.pwm_resolution_bits >= 31U ||
      config.minimum_pulse_us == 0U ||
      config.maximum_pulse_us <= config.minimum_pulse_us) {
    return false;
  }
  const std::uint32_t period_us =
      1000000UL / config.pwm_frequency_hz;
  return period_us > config.maximum_pulse_us;
}

bool uartConfigComplete(const robot::esp1::UartConfig& config) {
  return gpioAssigned(config.tx_gpio) && gpioAssigned(config.rx_gpio) &&
         config.baud_rate > 0U;
}

std::uint32_t pwmMaxDuty(const std::uint8_t resolution_bits) {
  return (static_cast<std::uint32_t>(1U) << resolution_bits) - 1U;
}

bool irAdcConfigComplete() {
  return gpioAssigned(kIrAdcGpio);
}

bool irSwitchConfigComplete() {
  return gpioAssigned(kIrFrequencySelectGpio);
}

bool solarPanelLimitSwitchesConfigComplete() {
  return gpioAssigned(kSolarLimitBackRightSideGpio) &&
         gpioAssigned(kSolarLimitFrontRightSideGpio);
}

bool sideLineSensorConfigComplete() {
  return gpioAssigned(kSideLineSensorGpio);
}

bool sideLineSensor2ConfigComplete() {
  return gpioAssigned(kSideLineSensor2Gpio);
}

bool sideLineSensor3ConfigComplete() {
  return gpioAssigned(kSideLineSensor3Gpio);
}

bool rearLineSensorsConfigComplete() {
  return gpioAssigned(kRearLineSensorLeftGpio) &&
         gpioAssigned(kRearLineSensorRightGpio);
}

bool ultrasonic1ConfigComplete() {
  return gpioAssigned(kUltrasonic1TriggerGpio) &&
         gpioAssigned(kUltrasonic1EchoGpio) &&
         kUltrasonic1TriggerGpio != kUltrasonic1EchoGpio;
}

void initializeUltrasonic1() {
  if (!ultrasonic1ConfigComplete()) {
    return;
  }
  digitalWrite(kUltrasonic1TriggerGpio, LOW);
  pinMode(kUltrasonic1TriggerGpio, OUTPUT);
  pinMode(kUltrasonic1EchoGpio, INPUT);
}

void storeUltrasonic1Reading(const UltrasonicReading& reading) {
  portENTER_CRITICAL(&g_ultrasonic_result_mux);
  g_latest_ultrasonic_1_reading = reading;
  portEXIT_CRITICAL(&g_ultrasonic_result_mux);
}

UltrasonicReading latestUltrasonic1Reading() {
  portENTER_CRITICAL(&g_ultrasonic_result_mux);
  const UltrasonicReading reading = g_latest_ultrasonic_1_reading;
  portEXIT_CRITICAL(&g_ultrasonic_result_mux);
  return reading;
}

bool initializeLaserSnapshotMailbox() {
  if (g_laser_snapshot_queue != nullptr) {
    return false;
  }
  g_laser_snapshot_queue = xQueueCreateStatic(
      1U, sizeof(robot::LaserDistanceSnapshot),
      g_laser_snapshot_queue_storage, &g_laser_snapshot_queue_buffer);
  return g_laser_snapshot_queue != nullptr;
}

bool initializeLaserProfileRequestMailbox() {
  if (g_laser_profile_request_queue != nullptr) {
    return false;
  }
  g_laser_profile_request_queue = xQueueCreateStatic(
      1U, sizeof(robot::LaserDistanceProfile),
      g_laser_profile_request_queue_storage,
      &g_laser_profile_request_queue_buffer);
  return g_laser_profile_request_queue != nullptr;
}

void requestLaserDistanceProfile(
    const robot::LaserDistanceProfile profile) {
  if (g_laser_profile_request_queue != nullptr) {
    (void)xQueueOverwrite(g_laser_profile_request_queue, &profile);
  }
}

bool takeLaserDistanceProfileRequest(
    robot::LaserDistanceProfile& profile) {
  return g_laser_profile_request_queue != nullptr &&
         xQueueReceive(g_laser_profile_request_queue, &profile, 0U) ==
             pdPASS;
}

void storeLaserDistanceSnapshot(
    const robot::LaserDistanceSnapshot& snapshot) {
  if (g_laser_snapshot_queue != nullptr) {
    (void)xQueueOverwrite(g_laser_snapshot_queue, &snapshot);
  }
}

bool latestLaserDistanceSnapshot(
    robot::LaserDistanceSnapshot& snapshot) {
  if (g_laser_snapshot_queue == nullptr) {
    snapshot = {};
    return false;
  }
  return xQueuePeek(g_laser_snapshot_queue, &snapshot, 0U) == pdPASS;
}

UltrasonicReading readUltrasonic1() {
  UltrasonicReading reading{};
  reading.configured = ultrasonic1ConfigComplete();
  if (!reading.configured) {
    return reading;
  }

  // The trigger remains LOW between samples. HC-SR04 requires a minimum 10 us
  // HIGH pulse; pulseIn is confined to this low-priority sensor task and has a
  // timeout derived from the data-sheet 4 m maximum range.
  digitalWrite(kUltrasonic1TriggerGpio, HIGH);
  delayMicroseconds(robot::kHcSr04TriggerPulseUs);
  digitalWrite(kUltrasonic1TriggerGpio, LOW);

  reading.echo_duration_us = static_cast<std::uint32_t>(
      pulseIn(kUltrasonic1EchoGpio, HIGH, robot::kHcSr04EchoTimeoutUs));
  if (reading.echo_duration_us == 0U) {
    return reading;
  }

  reading.distance_mm =
      robot::hcSr04DistanceMmFromEchoUs(reading.echo_duration_us);
  reading.echo_valid =
      robot::hcSr04DistanceMmIsValid(reading.distance_mm);
  return reading;
}

void initializeSolarPanelLimitSwitches() {
  if (!solarPanelLimitSwitchesConfigComplete()) {
    return;
  }
  pinMode(kSolarLimitBackRightSideGpio, INPUT);
  pinMode(kSolarLimitFrontRightSideGpio, INPUT);
}

SolarPanelLimitSwitchReading readSolarPanelLimitSwitches() {
  SolarPanelLimitSwitchReading reading{};
  reading.configured = solarPanelLimitSwitchesConfigComplete();
  if (!reading.configured) {
    return reading;
  }
  reading.back_right_high =
      digitalRead(kSolarLimitBackRightSideGpio) == HIGH;
  reading.front_right_high =
      digitalRead(kSolarLimitFrontRightSideGpio) == HIGH;
  return reading;
}

void initializeSideLineSensor() {
  if (sideLineSensorConfigComplete()) {
    pinMode(kSideLineSensorGpio, INPUT);
  }
  if (sideLineSensor2ConfigComplete()) {
    pinMode(kSideLineSensor2Gpio, INPUT);
  }
  if (sideLineSensor3ConfigComplete()) {
    pinMode(kSideLineSensor3Gpio, INPUT);
  }
}

void initializeRearLineSensors() {
  if (!rearLineSensorsConfigComplete()) {
    return;
  }
  pinMode(kRearLineSensorLeftGpio, INPUT);
  pinMode(kRearLineSensorRightGpio, INPUT);
}

robot::RearLineSensorSnapshot readRearLineSensors(
    const robot::Milliseconds now_ms) {
  robot::RearLineSensorSnapshot snapshot{};
  snapshot.captured_at_ms = now_ms;
  snapshot.configured = rearLineSensorsConfigComplete();
  if (snapshot.configured) {
    snapshot.left_electrical_high =
        digitalRead(kRearLineSensorLeftGpio) == HIGH;
    snapshot.right_electrical_high =
        digitalRead(kRearLineSensorRightGpio) == HIGH;
  }
  snapshot.side_configured = sideLineSensorConfigComplete();
  if (snapshot.side_configured) {
    snapshot.side_electrical_high =
        digitalRead(kSideLineSensorGpio) == HIGH;
  }
  snapshot.side_2_configured = sideLineSensor2ConfigComplete();
  if (snapshot.side_2_configured) {
    snapshot.side_2_electrical_high =
        digitalRead(kSideLineSensor2Gpio) == HIGH;
  }
  snapshot.side_3_configured = sideLineSensor3ConfigComplete();
  if (snapshot.side_3_configured) {
    snapshot.side_3_electrical_high =
        digitalRead(kSideLineSensor3Gpio) == HIGH;
  }
  return snapshot;
}

std::uint16_t clampAdcSample(const int sample) {
  if (sample <= 0) {
    return 0U;
  }
  if (sample >= static_cast<int>(kIrAdcMaximumSample)) {
    return kIrAdcMaximumSample;
  }
  return static_cast<std::uint16_t>(sample);
}

std::uint32_t selectedFrequencyHzForSwitch(const bool debounced_high) {
  const int state = debounced_high ? HIGH : LOW;
  if (state == kIrSwitchRawStateFor1Khz) {
    return kIrBeaconFrequency1Khz;
  }
  if (state == kIrSwitchRawStateFor10Khz) {
    return kIrBeaconFrequency10Khz;
  }
  return kIrBeaconFrequency1Khz;
}

std::uint16_t thresholdOnForFrequency(const std::uint32_t frequency_hz) {
  return frequency_hz == kIrBeaconFrequency10Khz ? IR_THRESHOLD_10KHZ
                                                 : IR_THRESHOLD_1KHZ;
}

std::uint16_t thresholdOffForFrequency(const std::uint32_t frequency_hz) {
  return frequency_hz == kIrBeaconFrequency10Khz ? IR_THRESHOLD_10KHZ_OFF
                                                 : IR_THRESHOLD_1KHZ_OFF;
}

DebouncedSwitchState updateIrSwitch(DebouncedSwitchState state,
                                    const robot::Milliseconds now_ms) {
  if (!irSwitchConfigComplete()) {
    return state;
  }

  state.raw_high = digitalRead(kIrFrequencySelectGpio) == HIGH;
  if (state.raw_high != state.candidate_high) {
    state.candidate_high = state.raw_high;
    state.candidate_since_ms = now_ms;
  } else if (state.debounced_high != state.candidate_high &&
             now_ms - state.candidate_since_ms >= kIrSwitchDebounceMs) {
    state.debounced_high = state.candidate_high;
  }
  return state;
}

std::uint16_t goertzelAmplitude(const std::uint16_t* samples,
                                const std::uint16_t sample_count,
                                const float mean,
                                const std::uint32_t target_frequency_hz,
                                const std::uint32_t sample_rate_hz) {
  const float normalized_frequency =
      static_cast<float>(target_frequency_hz) /
      static_cast<float>(sample_rate_hz);
  const float coefficient = 2.0F * std::cos(2.0F * kIrPi *
                                           normalized_frequency);
  float s_prev = 0.0F;
  float s_prev2 = 0.0F;

  for (std::uint16_t index = 0U; index < sample_count; ++index) {
    const float centered =
        static_cast<float>(samples[index]) - mean;
    const float s = centered + coefficient * s_prev - s_prev2;
    s_prev2 = s_prev;
    s_prev = s;
  }

  const float power =
      s_prev2 * s_prev2 + s_prev * s_prev - coefficient * s_prev * s_prev2;
  const float amplitude =
      2.0F * std::sqrt(power < 0.0F ? 0.0F : power) /
      static_cast<float>(sample_count);
  if (amplitude >= static_cast<float>(kIrAdcMaximumSample)) {
    return kIrAdcMaximumSample;
  }
  if (amplitude <= 0.0F) {
    return 0U;
  }
  return static_cast<std::uint16_t>(amplitude + 0.5F);
}

void storeIrAdcResult(const IrAdcWindowResult& result) {
  portENTER_CRITICAL(&g_ir_adc_result_mux);
  g_latest_ir_adc_result = result;
  portEXIT_CRITICAL(&g_ir_adc_result_mux);
}

IrAdcWindowResult latestIrAdcResult() {
  portENTER_CRITICAL(&g_ir_adc_result_mux);
  const IrAdcWindowResult result = g_latest_ir_adc_result;
  portEXIT_CRITICAL(&g_ir_adc_result_mux);
  return result;
}

IrAdcWindowResult sampleIrAdcWindow(const DebouncedSwitchState& switch_state,
                                    bool& detected,
                                    std::uint8_t& detect_count,
                                    std::uint8_t& clear_count) {
  // The 20 ms window captures many cycles of both supported beacons. Peak-to-
  // peak remains a diagnostic; detection uses mean-subtracted Goertzel output.
  std::uint16_t minimum = kIrAdcMaximumSample;
  std::uint16_t maximum = 0U;
  std::uint32_t sum = 0U;
  std::uint32_t next_sample_us = micros();
  const std::uint32_t window_start_us = next_sample_us;

  for (std::uint16_t index = 0U; index < kIrAdcSamplesPerWindow; ++index) {
    while (static_cast<std::int32_t>(micros() - next_sample_us) < 0) {
    }

    const std::uint16_t sample = clampAdcSample(analogRead(kIrAdcGpio));
    g_ir_adc_samples[index] = sample;
    if (sample < minimum) {
      minimum = sample;
    }
    if (sample > maximum) {
      maximum = sample;
    }
    sum += sample;
    next_sample_us += kIrAdcSampleIntervalUs;

    if ((index + 1U) % kIrAdcSamplesPerYield == 0U) {
      taskYIELD();
    }
  }
  const std::uint32_t window_elapsed_us = micros() - window_start_us;
  const std::uint32_t achieved_sample_rate_hz =
      window_elapsed_us == 0U
          ? 0U
          : (static_cast<std::uint32_t>(kIrAdcSamplesPerWindow) *
             1000000UL) /
                window_elapsed_us;

  IrAdcWindowResult result{};
  result.average =
      static_cast<std::uint16_t>(sum / kIrAdcSamplesPerWindow);
  result.minimum = minimum;
  result.maximum = maximum;
  result.amplitude_pp = static_cast<std::uint16_t>(maximum - minimum);
  result.latest_sample = g_ir_adc_samples[kIrAdcSamplesPerWindow - 1U];
  result.switch_raw_high = switch_state.raw_high;
  result.switch_debounced_high = switch_state.debounced_high;
  result.selected_frequency_hz =
      selectedFrequencyHzForSwitch(switch_state.debounced_high);
  result.achieved_sample_rate_hz = achieved_sample_rate_hz;

  const float mean = static_cast<float>(sum) /
                     static_cast<float>(kIrAdcSamplesPerWindow);
  const std::uint32_t detector_sample_rate_hz =
      achieved_sample_rate_hz == 0U ? kIrAdcSampleRateHz
                                    : achieved_sample_rate_hz;
  result.amplitude_1khz =
      goertzelAmplitude(g_ir_adc_samples, kIrAdcSamplesPerWindow, mean,
                        kIrBeaconFrequency1Khz, detector_sample_rate_hz);
  result.amplitude_10khz =
      goertzelAmplitude(g_ir_adc_samples, kIrAdcSamplesPerWindow, mean,
                        kIrBeaconFrequency10Khz, detector_sample_rate_hz);
  result.selected_amplitude =
      result.selected_frequency_hz == kIrBeaconFrequency10Khz
          ? result.amplitude_10khz
          : result.amplitude_1khz;
  result.active_threshold = detected
                                ? thresholdOffForFrequency(
                                      result.selected_frequency_hz)
                                : thresholdOnForFrequency(
                                      result.selected_frequency_hz);

  const bool window_valid =
      detected ? result.selected_amplitude >= result.active_threshold
               : result.selected_amplitude > result.active_threshold;
  if (window_valid) {
    if (detect_count < 255U) {
      ++detect_count;
    }
    clear_count = 0U;
    if (!detected &&
        detect_count >= kIrConsecutiveDetectWindowsRequired) {
      detected = true;
      result.active_threshold =
          thresholdOffForFrequency(result.selected_frequency_hz);
    }
  } else {
    detect_count = 0U;
    if (clear_count < 255U) {
      ++clear_count;
    }
    if (detected && clear_count >= kIrConsecutiveClearWindowsRequired) {
      detected = false;
      result.active_threshold =
          thresholdOnForFrequency(result.selected_frequency_hz);
    }
  }

  result.consecutive_detection_count = detect_count;
  result.sampled_at_ms = static_cast<robot::Milliseconds>(millis());
  result.beacon_detected = detected;
  result.configured = true;
  return result;
}

class DualPwmMotorOutput final : public robot::IMotorOutput {
 public:
  explicit DualPwmMotorOutput(
      const robot::esp1::DualPwmMotorOutputConfig& config)
      : config_(config) {}

  void initializeDisabled() override {
    configured_ = motorConfigComplete(config_);
    if (!configured_) {
      return;
    }

    ledcSetup(config_.pwm0_channel, config_.pwm_frequency_hz,
              config_.pwm_resolution_bits);
    ledcSetup(config_.pwm1_channel, config_.pwm_frequency_hz,
              config_.pwm_resolution_bits);
    ledcAttachPin(config_.pwm0_gpio, config_.pwm0_channel);
    ledcAttachPin(config_.pwm1_gpio, config_.pwm1_channel);
    disable();
  }

  void apply(const robot::MotorCommand& command) override {
    if (!configured_ || !command.enabled ||
        command.duty_command_milli == 0) {
      disable();
      return;
    }

    const std::int16_t signed_milli =
        robot::clampCommandMilli(static_cast<std::int16_t>(
            command.duty_command_milli * config_.forward_sign));
    const std::uint32_t pwm_duty =
        (static_cast<std::uint32_t>(std::abs(signed_milli)) *
         pwmMaxDuty(config_.pwm_resolution_bits)) /
        1000U;
    const bool positive = signed_milli > 0;
    const bool pwm0_forward =
        config_.h_bridge_mode ==
        robot::esp1::DualPwmHBridgeMode::Pwm0ForwardPwm1Reverse;
    const bool use_pwm0 = positive ? pwm0_forward : !pwm0_forward;

    ledcWrite(config_.pwm0_channel, use_pwm0 ? pwm_duty : 0U);
    ledcWrite(config_.pwm1_channel, use_pwm0 ? 0U : pwm_duty);
    last_command_ = command;
    last_command_.duty_command_milli = signed_milli;
  }

  void disable() override {
    if (configured_) {
      ledcWrite(config_.pwm0_channel, 0U);
      ledcWrite(config_.pwm1_channel, 0U);
    }
    last_command_ = robot::disabledMotorCommand();
  }

  bool configured() const { return configured_; }
  const robot::MotorCommand& lastAppliedCommand() const {
    return last_command_;
  }

 private:
  const robot::esp1::DualPwmMotorOutputConfig& config_;
  bool configured_{false};
  robot::MotorCommand last_command_{};
};

class SolarHookServoOutput {
 public:
  explicit SolarHookServoOutput(
      const robot::esp1::ServoOutputConfig& config)
      : config_(config) {}

  void initializeDisabled() {
    configured_ = servoConfigComplete(config_);
    if (!configured_) {
      return;
    }
    ledcSetup(config_.pwm_channel, config_.pwm_frequency_hz,
              config_.pwm_resolution_bits);
    pinMode(config_.gpio, INPUT);
    disable();
  }

  bool apply(const robot::SolarHookCommand& command) {
    if (!configured_) {
      return false;
    }
    if (!command.enabled) {
      disable();
      return true;
    }

    ledcSetup(config_.pwm_channel, config_.pwm_frequency_hz,
              config_.pwm_resolution_bits);
    ledcAttachPin(config_.gpio, config_.pwm_channel);
    ledcWrite(config_.pwm_channel,
              dutyForPulseUs(pulseUsForAngle(command.angle_deg)));
    output_enabled_ = true;
    commanded_angle_deg_ =
        static_cast<std::int16_t>(command.angle_deg);
    return true;
  }

  void disable() {
    if (configured_) {
      ledcWrite(config_.pwm_channel, 0U);
      ledcDetachPin(config_.gpio);
      pinMode(config_.gpio, INPUT);
    }
    output_enabled_ = false;
    commanded_angle_deg_ = -1;
  }

  bool configured() const { return configured_; }
  bool outputEnabled() const { return output_enabled_; }
  std::int16_t commandedAngleDeg() const {
    return commanded_angle_deg_;
  }

 private:
  std::uint32_t pulseUsForAngle(
      const std::uint8_t angle_deg) const {
    const std::uint32_t pulse_range_us =
        static_cast<std::uint32_t>(
            config_.maximum_pulse_us - config_.minimum_pulse_us);
    return config_.minimum_pulse_us +
           ((static_cast<std::uint32_t>(angle_deg) *
                 pulse_range_us +
             90U) /
            180U);
  }

  std::uint32_t dutyForPulseUs(
      const std::uint32_t pulse_us) const {
    const std::uint32_t period_us =
        1000000UL / config_.pwm_frequency_hz;
    return ((pulse_us * pwmMaxDuty(config_.pwm_resolution_bits)) +
            (period_us / 2U)) /
           period_us;
  }

  const robot::esp1::ServoOutputConfig& config_;
  bool configured_{false};
  bool output_enabled_{false};
  std::int16_t commanded_angle_deg_{-1};
};

std::uint16_t motorMagnitudeMilli(const DualPwmMotorOutput& back_left,
                                  const DualPwmMotorOutput& back_right) {
  const std::uint16_t left =
      static_cast<std::uint16_t>(
          std::abs(back_left.lastAppliedCommand().duty_command_milli));
  const std::uint16_t right =
      static_cast<std::uint16_t>(
          std::abs(back_right.lastAppliedCommand().duty_command_milli));
  return left > right ? left : right;
}

class RearCommandLink {
 public:
  explicit RearCommandLink(const robot::esp1::UartConfig& config)
      : config_(config) {}

  void initialize() {
    configured_ = uartConfigComplete(config_);
    if (configured_) {
      Serial1.begin(config_.baud_rate, SERIAL_8N1, config_.rx_gpio,
                    config_.tx_gpio);
    }
  }

  bool receive(robot::UartPacket& packet) {
    if (!configured_) {
      return false;
    }

    while (Serial1.available() > 0) {
      const robot::UartFrameParserStatus status =
          parser_.push(static_cast<std::uint8_t>(Serial1.read()), packet);
      if (status == robot::UartFrameParserStatus::PacketReady) {
        return true;
      } else if (status == robot::UartFrameParserStatus::InvalidFrame) {
        invalid_frame_seen_ = true;
      }
    }
    return false;
  }

  bool consumeInvalidFrameSeen() {
    const bool seen = invalid_frame_seen_;
    invalid_frame_seen_ = false;
    return seen;
  }

  bool send(const robot::UartPacket& packet) {
    if (!configured_) {
      return false;
    }

    std::uint8_t frame[robot::kUartFrameOverheadSize +
                       robot::kUartMaxPayloadSize]{};
    std::size_t frame_size = 0U;
    if (!robot::encodeUartFrame(packet, frame, sizeof(frame), frame_size)) {
      return false;
    }
    return Serial1.write(frame, frame_size) == frame_size;
  }

  bool configured() const { return configured_; }

 private:
  const robot::esp1::UartConfig& config_;
  robot::UartFrameParser parser_{};
  bool configured_{false};
  bool invalid_frame_seen_{false};
};

void publishEsp1Status(RearCommandLink& link,
                       const DualPwmMotorOutput& back_left,
                       const DualPwmMotorOutput& back_right,
                       const DualPwmMotorOutput& funnel_motor,
                       const SolarHookServoOutput& solar_hook,
                       const robot::RearDriveStatus& rear_status,
                       const robot::FunnelStatus& funnel_status,
                       const robot::RearLineSensorSnapshot& line_sensors,
                       const robot::Milliseconds now_ms) {
  static robot::Milliseconds last_publish_ms = 0U;
  static std::uint16_t sequence = 0U;
  if (now_ms - last_publish_ms < kEsp1StatusPeriodMs) {
    return;
  }
  last_publish_ms = now_ms;

  const IrAdcWindowResult ir = latestIrAdcResult();
  const UltrasonicReading ultrasonic_1 = latestUltrasonic1Reading();
  robot::Esp1StatusReport report{};
  report.uptime_ms = now_ms;
  report.mode = robot::RobotTestMode::Disabled;
  const bool rear_stale = !rear_status.link_healthy &&
                          rear_status.has_valid_command &&
                          rear_status.command_age_ms >
                              robot::kDefaultCommunicationTimeoutMs;
  const bool funnel_stale = robot::enabledFunnelCommandIsStale(
      funnel_status, robot::kDefaultCommunicationTimeoutMs);
  report.fault_active = rear_stale || funnel_stale;
  report.fault_code = report.fault_active
                          ? robot::FaultCode::CommunicationStale
                          : robot::FaultCode::None;
  report.back_left_applied_command_milli =
      back_left.lastAppliedCommand().duty_command_milli;
  report.back_right_applied_command_milli =
      back_right.lastAppliedCommand().duty_command_milli;
  report.funnel_applied_command_milli =
      funnel_motor.lastAppliedCommand().duty_command_milli;
  report.funnel_configured = funnel_motor.configured();
  report.solar_hook_configured = solar_hook.configured();
  report.solar_hook_output_enabled = solar_hook.outputEnabled();
  report.solar_hook_commanded_angle_deg =
      solar_hook.commandedAngleDeg();
  const SolarPanelLimitSwitchReading solar_limits =
      readSolarPanelLimitSwitches();
  report.solar_panel_limit_switches_configured =
      solar_limits.configured;
  report.solar_limit_back_right_high =
      solar_limits.back_right_high;
  report.solar_limit_front_right_high =
      solar_limits.front_right_high;
  report.side_line_sensor_configured = line_sensors.side_configured;
  report.side_line_sensor_high = line_sensors.side_electrical_high;
  report.ultrasonic_1_configured = ultrasonic_1.configured;
  report.ultrasonic_1_echo_valid = ultrasonic_1.echo_valid;
  report.ultrasonic_1_distance_mm = ultrasonic_1.distance_mm;
  report.ultrasonic_1_echo_duration_us =
      ultrasonic_1.echo_duration_us;
  report.ir_adc_average = ir.average;
  report.ir_adc_min = ir.minimum;
  report.ir_adc_max = ir.maximum;
  report.ir_amplitude_pp = ir.amplitude_pp;
  report.ir_beacon_detected = ir.beacon_detected;
  report.ir_switch_raw_high = ir.switch_raw_high;
  report.ir_switch_debounced_high = ir.switch_debounced_high;
  report.ir_selected_frequency_hz = ir.selected_frequency_hz;
  report.ir_adc_latest_sample = ir.latest_sample;
  report.ir_1khz_amplitude = ir.amplitude_1khz;
  report.ir_10khz_amplitude = ir.amplitude_10khz;
  report.ir_selected_amplitude = ir.selected_amplitude;
  report.ir_active_threshold = ir.active_threshold;
  report.ir_consecutive_detection_count = ir.consecutive_detection_count;
  report.ir_adc_sample_rate_hz = ir.achieved_sample_rate_hz;
  link.send(robot::makeEsp1StatusPacket(report, sequence++));
}

void publishRearLineSensors(
    RearCommandLink& link,
    const robot::RearLineSensorSnapshot& snapshot) {
  static std::uint16_t sequence = 0U;
  link.send(robot::makeRearLineSensorPacket(snapshot, sequence++));
}

void publishLaserDistance(RearCommandLink& link,
                          const robot::Milliseconds now_ms) {
  static robot::Milliseconds last_publish_ms = 0U;
  static std::uint16_t packet_sequence = 0U;
  static std::uint16_t last_measurement_sequence = 0U;
  static robot::LaserDistanceProfile last_profile =
      robot::LaserDistanceProfile::HighAccuracy;
  static bool published_once = false;

  robot::LaserDistanceSnapshot snapshot{};
  if (!latestLaserDistanceSnapshot(snapshot)) {
    return;
  }
  const bool new_measurement =
      !published_once ||
      snapshot.measurement_sequence != last_measurement_sequence;
  const bool new_profile =
      !published_once || snapshot.profile != last_profile;
  const bool heartbeat_due =
      now_ms - last_publish_ms >= kEsp1StatusPeriodMs;
  if (!new_measurement && !new_profile && !heartbeat_due) {
    return;
  }

  if (link.send(
          robot::makeLaserDistancePacket(snapshot, packet_sequence++))) {
    published_once = true;
    last_measurement_sequence = snapshot.measurement_sequence;
    last_profile = snapshot.profile;
    last_publish_ms = now_ms;
  }
}

void printRearStatus(const robot::RearDriveStatus& status,
                     const DualPwmMotorOutput& back_left,
                     const DualPwmMotorOutput& back_right,
                     const RearCommandLink& link,
                     const robot::Milliseconds now_ms) {
  static robot::Milliseconds last_print_ms = 0U;
  if (now_ms - last_print_ms < kRearStatusPrintPeriodMs ||
      Serial.availableForWrite() < 100) {
    return;
  }
  last_print_ms = now_ms;

  Serial.print("rear_status,");
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(link.configured() ? 1 : 0);
  Serial.print(',');
  Serial.print(status.link_healthy ? 1 : 0);
  Serial.print(',');
  Serial.print(status.command_age_ms);
  Serial.print(',');
  Serial.print(status.last_sequence);
  Serial.print(',');
  Serial.print(status.invalid_packets_since_valid);
  Serial.print(',');
  Serial.print(back_left.configured() ? 1 : 0);
  Serial.print(',');
  Serial.println(back_right.configured() ? 1 : 0);
}

void printIrDebugLine(const DualPwmMotorOutput& back_left,
                      const DualPwmMotorOutput& back_right,
                      const robot::Milliseconds now_ms) {
  static robot::Milliseconds last_print_ms = 0U;
  if (now_ms - last_print_ms < kIrSerialDebugPeriodMs ||
      Serial.availableForWrite() < 96) {
    return;
  }
  last_print_ms = now_ms;

  const IrAdcWindowResult ir = latestIrAdcResult();
  Serial.print("IR avg=");
  Serial.print(ir.average);
  Serial.print(" min=");
  Serial.print(ir.minimum);
  Serial.print(" max=");
  Serial.print(ir.maximum);
  Serial.print(" amp=");
  Serial.print(ir.amplitude_pp);
  Serial.print(" f=");
  Serial.print(ir.selected_frequency_hz);
  Serial.print(" e1k=");
  Serial.print(ir.amplitude_1khz);
  Serial.print(" e10k=");
  Serial.print(ir.amplitude_10khz);
  Serial.print(" selected=");
  Serial.print(ir.selected_amplitude);
  Serial.print(" raw=");
  Serial.print(ir.switch_raw_high ? 1 : 0);
  Serial.print(" debounced=");
  Serial.print(ir.switch_debounced_high ? 1 : 0);
  Serial.print(" detected=");
  Serial.print(ir.beacon_detected ? 1 : 0);
  Serial.print(" motor=");
  Serial.println(motorMagnitudeMilli(back_left, back_right));
}

void sensorAcquisitionTask(void* parameters) {
  (void)parameters;

  robot::esp1::Vl53l0xDistanceSensor laser_distance_sensor{};
  bool detected = false;
  std::uint8_t detect_count = 0U;
  std::uint8_t clear_count = 0U;
  DebouncedSwitchState switch_state{};
  if (irSwitchConfigComplete()) {
    pinMode(kIrFrequencySelectGpio, INPUT_PULLUP);
    switch_state.raw_high = digitalRead(kIrFrequencySelectGpio) == HIGH;
    switch_state.debounced_high = switch_state.raw_high;
    switch_state.candidate_high = switch_state.raw_high;
    switch_state.candidate_since_ms =
        static_cast<robot::Milliseconds>(millis());
  }
  std::uint32_t last_selected_frequency_hz =
      selectedFrequencyHzForSwitch(switch_state.debounced_high);

  if (!irAdcConfigComplete()) {
    storeIrAdcResult({});
  } else {
    pinMode(kIrAdcGpio, INPUT);
    analogReadResolution(kIrAdcResolutionBits);
    analogSetPinAttenuation(kIrAdcGpio, ADC_11db);
  }
  initializeUltrasonic1();
  storeUltrasonic1Reading(
      UltrasonicReading{0U, 0U, ultrasonic1ConfigComplete(), false});
  (void)laser_distance_sensor.initialize(
      Wire, robot::esp1::kHardwareConfig.laser_distance_sensor);
  storeLaserDistanceSnapshot(laser_distance_sensor.snapshot());

  TickType_t last_wake_tick = xTaskGetTickCount();
  robot::Milliseconds last_slow_acquisition_ms =
      static_cast<robot::Milliseconds>(millis()) -
      kEsp1StatusPeriodMs;

  for (;;) {
    const robot::Milliseconds now_ms =
        static_cast<robot::Milliseconds>(millis());
    robot::LaserDistanceProfile requested_profile{};
    if (takeLaserDistanceProfileRequest(requested_profile)) {
      (void)laser_distance_sensor.setProfile(
          robot::esp1::kHardwareConfig.laser_distance_sensor,
          requested_profile);
      storeLaserDistanceSnapshot(laser_distance_sensor.snapshot());
    }
    if (laser_distance_sensor.service(now_ms)) {
      storeLaserDistanceSnapshot(laser_distance_sensor.snapshot());
    }

    if (now_ms - last_slow_acquisition_ms >= kEsp1StatusPeriodMs) {
      last_slow_acquisition_ms = now_ms;
      switch_state = updateIrSwitch(switch_state, now_ms);
      const std::uint32_t selected_frequency_hz =
          selectedFrequencyHzForSwitch(switch_state.debounced_high);
      if (selected_frequency_hz != last_selected_frequency_hz) {
        detected = false;
        detect_count = 0U;
        clear_count = 0U;
        last_selected_frequency_hz = selected_frequency_hz;
      }

      if (irAdcConfigComplete()) {
        const IrAdcWindowResult result =
            sampleIrAdcWindow(switch_state, detected, detect_count,
                              clear_count);
        storeIrAdcResult(result);
      }
      storeUltrasonic1Reading(readUltrasonic1());
    }

    const TickType_t period_ticks =
        pdMS_TO_TICKS(kSensorServicePeriodMs);
    const TickType_t now_tick = xTaskGetTickCount();
    if (now_tick - last_wake_tick > period_ticks) {
      last_wake_tick = now_tick;
    }
    vTaskDelayUntil(&last_wake_tick, period_ticks);
  }
}

void rearDriveTask(void* parameters) {
  (void)parameters;

  DualPwmMotorOutput back_left_motor{
      robot::esp1::kHardwareConfig.back_left_motor};
  DualPwmMotorOutput back_right_motor{
      robot::esp1::kHardwareConfig.back_right_motor};
  DualPwmMotorOutput funnel_motor{
      robot::esp1::kHardwareConfig.funnel_motor};
  SolarHookServoOutput solar_hook{
      robot::esp1::kHardwareConfig.solar_hook_servo};
  RearCommandLink link{robot::esp1::kHardwareConfig.uart_to_esp2};
  robot::RearDriveCommandReceiver receiver{};
  robot::FunnelCommandReceiver funnel_receiver{};

  back_left_motor.initializeDisabled();
  back_right_motor.initializeDisabled();
  funnel_motor.initializeDisabled();
  solar_hook.initializeDisabled();
  link.initialize();
  initializeSolarPanelLimitSwitches();
  initializeSideLineSensor();
  initializeRearLineSensors();

  TickType_t last_wake_tick = xTaskGetTickCount();
  robot::LaserDistanceProfile last_requested_laser_profile =
      robot::LaserDistanceProfile::HighAccuracy;
  robot::Milliseconds last_laser_profile_request_ms = 0U;

  for (;;) {
    const robot::Milliseconds now_ms =
        static_cast<robot::Milliseconds>(millis());

    for (std::uint8_t packet_index = 0U;
         packet_index < kMaxUartPacketsPerCycle; ++packet_index) {
      robot::UartPacket packet{};
      if (!link.receive(packet)) {
        break;
      }
      if (packet.header.message_type == robot::UartMessageType::RearWheelCommand) {
        receiver.acceptPacket(packet, now_ms);
      } else if (packet.header.message_type ==
                 robot::UartMessageType::MechanismCommand) {
        if (packet.header.payload_size > 0U &&
            packet.payload[0] ==
                robot::kMechanismPayloadTargetSolarHook) {
          robot::SolarHookCommand command{};
          if (robot::decodeSolarHookCommandPacket(packet, command)) {
            (void)solar_hook.apply(command);
          }
        } else {
          // Preserve the funnel receiver's existing invalid-packet counting
          // for malformed and unsupported mechanism commands.
          funnel_receiver.acceptPacket(packet, now_ms);
        }
      }
    }
    if (link.consumeInvalidFrameSeen()) {
      robot::UartPacket packet{};
      receiver.acceptPacket(packet, now_ms);
      funnel_receiver.acceptPacket(packet, now_ms);
    }

    back_left_motor.apply(receiver.backLeftCommand(now_ms));
    back_right_motor.apply(receiver.backRightCommand(now_ms));
    const robot::LaserDistanceProfile requested_laser_profile =
        receiver.laserProfile(now_ms);
    robot::LaserDistanceSnapshot laser_snapshot{};
    const bool laser_profile_ready =
        latestLaserDistanceSnapshot(laser_snapshot) &&
        laser_snapshot.initialized && laser_snapshot.ranging &&
        laser_snapshot.driver_status == 0 &&
        laser_snapshot.profile == requested_laser_profile;
    const bool profile_changed =
        requested_laser_profile != last_requested_laser_profile;
    const bool retry_due =
        !laser_profile_ready &&
        now_ms - last_laser_profile_request_ms >= kEsp1StatusPeriodMs;
    if (profile_changed || retry_due) {
      requestLaserDistanceProfile(requested_laser_profile);
      last_requested_laser_profile = requested_laser_profile;
      last_laser_profile_request_ms = now_ms;
    }
    funnel_motor.apply(funnel_receiver.motorCommand(now_ms));

    const robot::RearDriveStatus rear_status = receiver.status(now_ms);
    const robot::FunnelStatus funnel_status = funnel_receiver.status(now_ms);
    const robot::RearLineSensorSnapshot line_sensors =
        readRearLineSensors(now_ms);
    publishEsp1Status(link, back_left_motor, back_right_motor, funnel_motor,
                      solar_hook, rear_status, funnel_status, line_sensors,
                      now_ms);
    publishRearLineSensors(link, line_sensors);
    publishLaserDistance(link, now_ms);
    printRearStatus(rear_status, back_left_motor, back_right_motor, link,
                    now_ms);
    printIrDebugLine(back_left_motor, back_right_motor, now_ms);

    vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(kRearDriveTaskPeriodMs));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  (void)initializeLaserSnapshotMailbox();
  (void)initializeLaserProfileRequestMailbox();
  xTaskCreatePinnedToCore(sensorAcquisitionTask, "esp1_sensors",
                          kSensorTaskStackBytes, nullptr,
                          kSensorTaskPriority, nullptr, kSensorTaskCore);
  xTaskCreatePinnedToCore(rearDriveTask, "esp1_rear_drive", kTaskStackBytes,
                          nullptr, kTaskPriority, nullptr, kTaskCore);
}

void loop() {
  vTaskSuspend(nullptr);
}
