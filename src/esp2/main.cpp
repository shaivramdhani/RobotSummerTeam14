#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <driver/mcpwm.h>
#include <esp_system.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "common/ChassisMixer.h"
#include "common/Esp1Status.h"
#include "common/EventLog.h"
#include "common/FunnelCommand.h"
#include "common/HabitatPiecesAutonomy.h"
#include "common/HabitatPlacementAutonomy.h"
#include "common/ImuHeadingHoldController.h"
#include "common/ImuRecovery.h"
#include "common/ImuTurnController.h"
#include "common/LineFollower.h"
#include "common/LineObservation.h"
#include "common/LineSensor.h"
#include "common/LaserDistance.h"
#include "common/MotionDiagnostics.h"
#include "common/MotorOutput.h"
#include "common/PegFinderAutonomy.h"
#include "common/RearDriveCommand.h"
#include "common/RearLineSensor.h"
#include "common/RobotCommandValidation.h"
#include "common/RobotTestModeManager.h"
#include "common/SolarHookServo.h"
#include "common/SolarPanelAutonomy.h"
#include "common/TelemetrySnapshot.h"
#include "common/TimeTrialAutonomy.h"
#include "common/TowerPiecesAutonomy.h"
#include "common/UartProtocol.h"
#include "esp2/ImuAcquisitionService.h"
#include "esp2/MechanismControllers.h"
#include "esp2/PinConfig.h"
#include "esp2/StepperAxis.h"

namespace {

constexpr const char* kApSsid = "Team14Robot";
constexpr const char* kApPassword = "robotdebug";
constexpr robot::Milliseconds kDefaultMotionTaskPeriodMs = 10U;
constexpr robot::Milliseconds kTelemetryPeriodMs = 100U;
constexpr robot::Milliseconds kMotionDiagnosticSamplePeriodMs = 100U;
constexpr robot::Milliseconds kCommandTimeoutMs = 700U;
constexpr robot::Milliseconds kMaxTimedTestDurationMs = 30000U;
constexpr float kSingleMotorDutyCap = 1.0F;
constexpr std::uint32_t kTaskStackBytes = 12288U;
constexpr UBaseType_t kTaskPriority = 1U;
constexpr BaseType_t kTaskCore = 1;
constexpr UBaseType_t kSensorAcquisitionTaskPriority = 1U;
constexpr BaseType_t kSensorAcquisitionTaskCore = 0;
constexpr std::size_t kSerialCommandBufferSize = 128U;
constexpr std::size_t kJsonBufferSize = 20480U;
constexpr std::size_t kClawServoCount = 3U;
constexpr std::size_t kMechanismServoCount = 5U;
constexpr std::size_t kHabitatPusherServoIndex = 3U;
constexpr std::size_t kWinchServoIndex = 4U;
constexpr int kClawServoUnsetAngleDeg = -1;
constexpr int kLegacyClawServoRotationDeg = 90;
constexpr std::uint8_t kImuI2cAddress = 0x68U;
constexpr std::uint16_t kImuCalibrationSampleCount = 500U;
constexpr std::uint32_t kImuFreshnessTimeoutUs = 75000U;
// Reuse the established maximum timed-motion bound while all wheel outputs
// are disabled. Three confirmations match the driver's unhealthy threshold.
constexpr robot::ImuRecoveryConfig kAutonomousImuRecoveryConfig{
    kMaxTimedTestDurationMs, 3U};
constexpr robot::Milliseconds kPreCalibrationStopSettleMs = 20U;
static_assert(kAutonomousImuRecoveryConfig.maximum_pause_ms > 0U,
              "autonomous IMU recovery must remain bounded");
static_assert(
    kAutonomousImuRecoveryConfig
            .consecutive_fresh_samples_required > 0U,
    "autonomous IMU recovery requires fresh-sample confirmation");

// Solar-panel first-stage tuning. TEMPORARY DEFAULTS: tune with the real
// beacon, sensor, lighting, and motors running before driving at speed.
constexpr std::uint32_t kIrBeaconFrequency1Khz = 1000U;
constexpr std::uint32_t kIrBeaconFrequency10Khz = 10000U;
constexpr std::uint16_t IR_BEACON_DETECT_THRESHOLD_1KHZ = 15U;
constexpr std::uint16_t IR_BEACON_RELEASE_THRESHOLD_1KHZ = 10U;
constexpr std::uint16_t IR_BEACON_DETECT_THRESHOLD_10KHZ = 12U;
constexpr std::uint16_t IR_BEACON_RELEASE_THRESHOLD_10KHZ = 6U;
constexpr robot::Milliseconds IR_BEACON_CONFIRM_TIME_MS = 50U;
constexpr float IR_FILTER_ALPHA = 0.5F;
constexpr robot::Milliseconds IR_IGNORE_AFTER_START_MS = 7000U;
constexpr robot::Milliseconds SOLAR_SEARCH_TIMEOUT_MS = 13000U;
constexpr float SOLAR_START_BASE_DUTY = 0.3F;
constexpr robot::Milliseconds SOLAR_SLOW_AFTER_MS = 7500U;
constexpr float SOLAR_SLOW_BASE_DUTY = 0.12F;
constexpr robot::Milliseconds SOLAR_CONTACT_TIMEOUT_MS =
    SOLAR_SEARCH_TIMEOUT_MS;
constexpr float SOLAR_RETRY_FORWARD_DUTY = SOLAR_SLOW_BASE_DUTY;
constexpr robot::Milliseconds SOLAR_STRAFE_START_DELAY_MS = 300U;
// TODO(team): tune both adjustment durations on the real robot. Zero keeps the
// new motion phases disabled until values are applied through telemetry.
constexpr robot::Milliseconds SOLAR_RETRY_STRAFE_LEFT_DURATION_MS = 300U;
constexpr robot::Milliseconds SOLAR_RETRY_FORWARD_DURATION_MS = 150U;
constexpr robot::Milliseconds SOLAR_RETRY_STRAFE_TIMEOUT_MS =
    SOLAR_CONTACT_TIMEOUT_MS;
constexpr robot::Milliseconds SOLAR_POST_CONTACT_FORWARD_DURATION_MS = 1300U;
constexpr robot::Milliseconds SOLAR_POST_CONTACT_FORWARD_START_DELAY_MS = 500U;
constexpr robot::Milliseconds SOLAR_LINE_REACQUIRE_STRAFE_START_DELAY_MS = 500U;
constexpr float SOLAR_POST_CONTACT_FORWARD_DUTY = SOLAR_RETRY_FORWARD_DUTY;
// Team wiring report: raw HIGH means the solar side switch has been hit.
constexpr bool SOLAR_LIMIT_SWITCH_HIT_WHEN_HIGH = true;
constexpr robot::SolarPanelAutonomyConfig kSolarPanelAutonomyConfig{
    IR_BEACON_DETECT_THRESHOLD_1KHZ, IR_BEACON_RELEASE_THRESHOLD_1KHZ,
    IR_BEACON_CONFIRM_TIME_MS, IR_FILTER_ALPHA, IR_IGNORE_AFTER_START_MS,
    SOLAR_SEARCH_TIMEOUT_MS};
constexpr robot::SolarPanelContactConfig kSolarPanelContactConfig{
    SOLAR_CONTACT_TIMEOUT_MS,
    0.0F,  // TODO(team): tune each strafe duty in telemetry.
    0.0F,
    0.0F,
    SOLAR_RETRY_FORWARD_DUTY,
    SOLAR_STRAFE_START_DELAY_MS,
    SOLAR_RETRY_STRAFE_LEFT_DURATION_MS,
    SOLAR_RETRY_FORWARD_DURATION_MS,
    SOLAR_RETRY_STRAFE_TIMEOUT_MS,
    SOLAR_POST_CONTACT_FORWARD_DURATION_MS,
    SOLAR_POST_CONTACT_FORWARD_START_DELAY_MS,
    SOLAR_LINE_REACQUIRE_STRAFE_START_DELAY_MS,
    SOLAR_POST_CONTACT_FORWARD_DUTY,
    0.0F,  // TODO(team): tune the rear-line strafe duty in telemetry.
    0U};   // TODO(team): tune the backward PID duration in telemetry.

static_assert(IR_BEACON_RELEASE_THRESHOLD_1KHZ <=
                  IR_BEACON_DETECT_THRESHOLD_1KHZ,
              "1 kHz solar IR release threshold must be <= detect threshold");
static_assert(IR_BEACON_RELEASE_THRESHOLD_10KHZ <=
                  IR_BEACON_DETECT_THRESHOLD_10KHZ,
              "10 kHz solar IR release threshold must be <= detect threshold");
static_assert(IR_FILTER_ALPHA >= 0.0F && IR_FILTER_ALPHA < 1.0F,
              "solar IR filter alpha must be in [0, 1)");
static_assert(SOLAR_SEARCH_TIMEOUT_MS > 0U,
              "solar search timeout must be nonzero");
static_assert(SOLAR_START_BASE_DUTY >= 0.0F &&
                  SOLAR_START_BASE_DUTY <= 1.0F,
              "solar start base duty must be in [0, 1]");
static_assert(SOLAR_SLOW_BASE_DUTY >= 0.0F && SOLAR_SLOW_BASE_DUTY <= 1.0F,
              "solar slow base duty must be in [0, 1]");
static_assert(SOLAR_CONTACT_TIMEOUT_MS > 0U,
              "solar contact timeout must be nonzero");
static_assert(SOLAR_RETRY_FORWARD_DUTY >= 0.0F &&
                  SOLAR_RETRY_FORWARD_DUTY <= 1.0F,
              "solar retry forward duty must be in [0, 1]");
static_assert(SOLAR_STRAFE_START_DELAY_MS > 0U,
              "solar strafe start delay must be nonzero");
static_assert(SOLAR_RETRY_STRAFE_TIMEOUT_MS > 0U,
              "solar retry strafe timeout must be nonzero");
static_assert(SOLAR_POST_CONTACT_FORWARD_DUTY >= 0.0F &&
                  SOLAR_POST_CONTACT_FORWARD_DUTY <= 1.0F,
              "solar post-contact forward duty must be in [0, 1]");

WebServer g_server{80};
char g_json_buffer[kJsonBufferSize]{};

// Drive-test wiring profile:
// - ESP2 owns the physical front motors and the web dashboard.
// - ESP1 owns the physical back motors. RearDriveCommand back_left/back_right
//   payload fields carry logical BL/BR commands to ESP1.

bool gpioAssigned(const int gpio) {
  return gpio >= 0;
}

float clampFloat(const float value, const float minimum, const float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

robot::Milliseconds elapsedSince(const robot::Milliseconds now_ms,
                                 const robot::Milliseconds then_ms) {
  return now_ms >= then_ms ? now_ms - then_ms : 0U;
}

bool parseFloat(const char* text, float& value) {
  if (text == nullptr) {
    return false;
  }
  char* end = nullptr;
  const float parsed = strtof(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parseUnsigned(const char* text, robot::Milliseconds& value) {
  if (text == nullptr || text[0] < '0' || text[0] > '9') {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0' ||
      parsed > std::numeric_limits<robot::Milliseconds>::max()) {
    return false;
  }
  value = static_cast<robot::Milliseconds>(parsed);
  return true;
}

bool parseSignedInteger(const char* text, int& value) {
  if (text == nullptr) {
    return false;
  }
  char* end = nullptr;
  const long parsed = strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool parsePolarity(const char* text, int& value) {
  int parsed = 0;
  if (!parseSignedInteger(text, parsed) || (parsed != 1 && parsed != -1)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parseOnOff(const char* text, bool& value) {
  if (text == nullptr) {
    return false;
  }
  if (std::strcmp(text, "on") == 0 || std::strcmp(text, "true") == 0 ||
      std::strcmp(text, "1") == 0) {
    value = true;
    return true;
  }
  if (std::strcmp(text, "off") == 0 || std::strcmp(text, "false") == 0 ||
      std::strcmp(text, "0") == 0) {
    value = false;
    return true;
  }
  return false;
}

bool motorConfigComplete(
    const robot::esp2::DualPwmMotorOutputConfig& config) {
  return gpioAssigned(config.pwm0_gpio) && gpioAssigned(config.pwm1_gpio) &&
         config.pwm0_channel >= 0 && config.pwm1_channel >= 0 &&
         config.pwm_frequency_hz > 0U && config.pwm_resolution_bits > 0U &&
         config.pwm_resolution_bits < 31U &&
         (config.forward_sign == 1 || config.forward_sign == -1) &&
         config.h_bridge_mode !=
             robot::esp2::DualPwmHBridgeMode::Unconfigured;
}

bool uartConfigComplete(const robot::esp2::UartConfig& config) {
  return gpioAssigned(config.tx_gpio) && gpioAssigned(config.rx_gpio) &&
         config.baud_rate > 0U;
}

std::uint32_t pwmMaxDuty(const std::uint8_t resolution_bits) {
  return (static_cast<std::uint32_t>(1U) << resolution_bits) - 1U;
}

bool servoOutputConfigComplete(
    const robot::esp2::ServoOutputConfig& config) {
  if (!gpioAssigned(config.gpio) ||
      config.backend == robot::esp2::ServoPwmBackend::Unconfigured ||
      config.pwm_frequency_hz == 0U ||
      config.minimum_pulse_us == 0U ||
      config.maximum_pulse_us <= config.minimum_pulse_us) {
    return false;
  }

  const std::uint32_t period_us = 1000000UL / config.pwm_frequency_hz;
  if (period_us <= config.maximum_pulse_us) {
    return false;
  }

  if (config.backend == robot::esp2::ServoPwmBackend::Ledc) {
    return config.ledc_channel >= 0 &&
           config.ledc_resolution_bits > 0U &&
           config.ledc_resolution_bits < 31U;
  }
  if (config.backend == robot::esp2::ServoPwmBackend::Mcpwm) {
    return config.mcpwm_unit >= 0 &&
           config.mcpwm_unit < static_cast<int>(MCPWM_UNIT_MAX) &&
           config.mcpwm_timer >= 0 &&
           config.mcpwm_timer < static_cast<int>(MCPWM_TIMER_MAX) &&
           config.mcpwm_generator >= 0 &&
           config.mcpwm_generator < static_cast<int>(MCPWM_GEN_MAX) &&
           config.mcpwm_timer_resolution_hz > 0U;
  }
  return false;
}

mcpwm_unit_t mcpwmUnit(
    const robot::esp2::ServoOutputConfig& config) {
  return static_cast<mcpwm_unit_t>(config.mcpwm_unit);
}

mcpwm_timer_t mcpwmTimer(
    const robot::esp2::ServoOutputConfig& config) {
  return static_cast<mcpwm_timer_t>(config.mcpwm_timer);
}

mcpwm_generator_t mcpwmGenerator(
    const robot::esp2::ServoOutputConfig& config) {
  return static_cast<mcpwm_generator_t>(config.mcpwm_generator);
}

bool mcpwmSignal(const robot::esp2::ServoOutputConfig& config,
                 mcpwm_io_signals_t& signal) {
  if (config.mcpwm_timer == 0 && config.mcpwm_generator == 0) {
    signal = MCPWM0A;
    return true;
  }
  if (config.mcpwm_timer == 0 && config.mcpwm_generator == 1) {
    signal = MCPWM0B;
    return true;
  }
  if (config.mcpwm_timer == 1 && config.mcpwm_generator == 0) {
    signal = MCPWM1A;
    return true;
  }
  if (config.mcpwm_timer == 1 && config.mcpwm_generator == 1) {
    signal = MCPWM1B;
    return true;
  }
  if (config.mcpwm_timer == 2 && config.mcpwm_generator == 0) {
    signal = MCPWM2A;
    return true;
  }
  if (config.mcpwm_timer == 2 && config.mcpwm_generator == 1) {
    signal = MCPWM2B;
    return true;
  }
  return false;
}

const char* resetReasonName(const esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "POWERON";
    case ESP_RST_EXT:
      return "EXTERNAL";
    case ESP_RST_SW:
      return "SOFTWARE";
    case ESP_RST_PANIC:
      return "PANIC";
    case ESP_RST_INT_WDT:
      return "INT_WDT";
    case ESP_RST_TASK_WDT:
      return "TASK_WDT";
    case ESP_RST_WDT:
      return "WDT";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    case ESP_RST_SDIO:
      return "SDIO";
    case ESP_RST_UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

const char* digitalLevelName(const int level) {
  if (level == HIGH) {
    return "HIGH";
  }
  if (level == LOW) {
    return "LOW";
  }
  return "UNKNOWN";
}

class DigitalFrontLineSensorReader final : public robot::ILineSensorReader {
 public:
  explicit DigitalFrontLineSensorReader(const robot::esp2::Esp2Pins& pins)
      : pins_(pins) {}

  void initialize() {
    configured_ = gpioAssigned(pins_.line_sensor_front_left) &&
                  gpioAssigned(pins_.line_sensor_front_right);
    if (!configured_) {
      return;
    }
    pinMode(pins_.line_sensor_front_left, INPUT);
    pinMode(pins_.line_sensor_front_right, INPUT);
  }

  robot::FrontLineSensorSnapshot readSnapshot(
      const robot::Milliseconds now_ms) override {
    if (!configured_) {
      last_left_level_ = -1;
      last_right_level_ = -1;
      return {robot::LineSample::Unknown, robot::LineSample::Unknown, now_ms,
              false};
    }

    last_left_level_ = digitalRead(pins_.line_sensor_front_left);
    last_right_level_ = digitalRead(pins_.line_sensor_front_right);
    return {last_left_level_ == HIGH ? robot::LineSample::OnTape
                                     : robot::LineSample::OffTape,
            last_right_level_ == HIGH ? robot::LineSample::OnTape
                                      : robot::LineSample::OffTape,
            now_ms, true};
  }

  bool configured() const { return configured_; }
  int lastLeftLevel() const { return last_left_level_; }
  int lastRightLevel() const { return last_right_level_; }

 private:
  const robot::esp2::Esp2Pins& pins_;
  bool configured_{false};
  int last_left_level_{-1};
  int last_right_level_{-1};
};

class DigitalActiveHighLimitSwitch {
 public:
  explicit DigitalActiveHighLimitSwitch(const int gpio) : gpio_(gpio) {}

  void initialize() {
    configured_ = gpioAssigned(gpio_);
    if (!configured_) {
      return;
    }
    pinMode(gpio_, INPUT);
    update();
  }

  void update() {
    raw_level_ = configured_ ? digitalRead(gpio_) : -1;
    active_ = raw_level_ == HIGH;
  }

  bool configured() const { return configured_; }
  bool active() const { return active_; }
  int rawLevel() const { return raw_level_; }

 private:
  int gpio_{robot::esp2::kUnassignedGpio};
  bool configured_{false};
  bool active_{false};
  int raw_level_{-1};
};

class DualPwmMotorOutput final : public robot::IMotorOutput {
 public:
  explicit DualPwmMotorOutput(
      const robot::esp2::DualPwmMotorOutputConfig& config)
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
    last_desired_ = command;
    if (!configured_ || !command.enabled ||
        command.duty_command_milli == 0) {
      disable();
      return;
    }

    const int runtime_sign = runtime_inverted_ ? -1 : 1;
    const std::int16_t signed_milli =
        robot::clampCommandMilli(static_cast<std::int16_t>(
            command.duty_command_milli * config_.forward_sign *
            runtime_sign));
    const std::uint32_t pwm_duty =
        (static_cast<std::uint32_t>(std::abs(signed_milli)) *
         pwmMaxDuty(config_.pwm_resolution_bits)) /
        1000U;
    const bool positive = signed_milli > 0;
    const bool pwm0_forward =
        config_.h_bridge_mode ==
        robot::esp2::DualPwmHBridgeMode::Pwm0ForwardPwm1Reverse;
    const bool use_pwm0 = positive ? pwm0_forward : !pwm0_forward;

    ledcWrite(config_.pwm0_channel, use_pwm0 ? pwm_duty : 0U);
    ledcWrite(config_.pwm1_channel, use_pwm0 ? 0U : pwm_duty);
    last_applied_ = command;
    last_applied_.duty_command_milli = signed_milli;
    last_applied_.enabled = signed_milli != 0;
  }

  void disable() override {
    if (configured_) {
      ledcWrite(config_.pwm0_channel, 0U);
      ledcWrite(config_.pwm1_channel, 0U);
    }
    last_applied_ = robot::disabledMotorCommand();
  }

  bool configured() const { return configured_; }
  bool runtimeInverted() const { return runtime_inverted_; }
  void setRuntimeInverted(const bool inverted) {
    runtime_inverted_ = inverted;
  }
  const robot::MotorCommand& lastDesiredCommand() const {
    return last_desired_;
  }
  const robot::MotorCommand& lastAppliedCommand() const {
    return last_applied_;
  }
  std::uint32_t pwm0DutyReadback() const {
    return configured_
               ? ledcRead(static_cast<std::uint8_t>(config_.pwm0_channel))
               : 0U;
  }
  std::uint32_t pwm1DutyReadback() const {
    return configured_
               ? ledcRead(static_cast<std::uint8_t>(config_.pwm1_channel))
               : 0U;
  }

 private:
  const robot::esp2::DualPwmMotorOutputConfig& config_;
  bool configured_{false};
  bool runtime_inverted_{false};
  robot::MotorCommand last_desired_{};
  robot::MotorCommand last_applied_{};
};

class RearCommandLink {
 public:
  explicit RearCommandLink(const robot::esp2::UartConfig& config)
      : config_(config) {}

  void initialize() {
    configured_ = uartConfigComplete(config_);
    if (configured_) {
      Serial1.begin(config_.baud_rate, SERIAL_8N1, config_.rx_gpio,
                    config_.tx_gpio);
    }
  }

  bool send(const robot::RearDriveCommand& command) {
    if (!configured_) {
      healthy_ = false;
      return false;
    }

    const std::uint16_t sequence = next_sequence_++;
    const robot::UartPacket packet =
        robot::makeRearDriveCommandPacket(command, sequence);
    const bool sent = sendPacket(packet);
    last_rear_sequence_sent_ = sequence;
    last_rear_sent_at_ms_ = command.sender_timestamp_ms;
    return sent;
  }

  bool send(const robot::FunnelCommand& command) {
    if (!configured_) {
      healthy_ = false;
      return false;
    }

    const std::uint16_t sequence = next_sequence_++;
    const robot::UartPacket packet =
        robot::makeFunnelCommandPacket(command, sequence);
    return sendPacket(packet);
  }

  bool send(const robot::SolarHookCommand& command) {
    if (!configured_) {
      healthy_ = false;
      return false;
    }

    const std::uint16_t sequence = next_sequence_++;
    const robot::UartPacket packet =
        robot::makeSolarHookCommandPacket(command, sequence);
    return sendPacket(packet);
  }

 private:
  bool sendPacket(const robot::UartPacket& packet) {
    std::uint8_t frame[robot::kUartFrameOverheadSize +
                       robot::kUartMaxPayloadSize]{};
    std::size_t frame_size = 0U;
    if (!robot::encodeUartFrame(packet, frame, sizeof(frame), frame_size)) {
      healthy_ = false;
      ++packet_error_count_;
      return false;
    }

    const std::size_t written = Serial1.write(frame, frame_size);
    healthy_ = written == frame_size;
    if (!healthy_) {
      ++packet_error_count_;
    }
    return healthy_;
  }

 public:
  void pollReceive(const robot::Milliseconds now_ms) {
    if (!configured_) {
      return;
    }

    robot::UartPacket packet{};
    while (Serial1.available() > 0) {
      const robot::UartFrameParserStatus status =
          parser_.push(static_cast<std::uint8_t>(Serial1.read()), packet);
      if (status == robot::UartFrameParserStatus::PacketReady) {
        if (packet.header.message_type ==
            robot::UartMessageType::HealthReport) {
          robot::Esp1StatusReport report{};
          if (robot::decodeEsp1StatusPacket(packet, report)) {
            latest_status_ = report;
            last_status_received_at_ms_ = now_ms;
            status_available_ = true;
          } else {
            ++packet_error_count_;
          }
        } else if (packet.header.message_type ==
                   robot::UartMessageType::SensorSnapshot) {
          robot::RearLineSensorSnapshot snapshot{};
          if (robot::decodeRearLineSensorPacket(packet, snapshot)) {
            if (rear_line_snapshot_available_ &&
                packet.header.sequence == last_rear_line_sequence_) {
              ++packet_error_count_;
            } else {
              latest_rear_line_snapshot_ = snapshot;
              last_rear_line_sequence_ = packet.header.sequence;
              last_rear_line_received_at_ms_ = now_ms;
              rear_line_snapshot_available_ = true;
            }
          } else {
            ++packet_error_count_;
          }
        } else if (packet.header.message_type ==
                   robot::UartMessageType::LaserDistanceSnapshot) {
          robot::LaserDistanceSnapshot snapshot{};
          if (robot::decodeLaserDistancePacket(packet, snapshot)) {
            if (laser_snapshot_available_ &&
                packet.header.sequence == last_laser_packet_sequence_) {
              ++packet_error_count_;
            } else {
              const bool new_measurement =
                  !laser_snapshot_available_ ||
                  snapshot.measurement_sequence !=
                      latest_laser_snapshot_.measurement_sequence;
              latest_laser_snapshot_ = snapshot;
              last_laser_packet_sequence_ = packet.header.sequence;
              last_laser_snapshot_received_at_ms_ = now_ms;
              if (new_measurement) {
                last_laser_measurement_received_at_ms_ = now_ms;
              }
              laser_snapshot_available_ = true;
            }
          } else {
            ++packet_error_count_;
          }
        } else {
          ++packet_error_count_;
        }
      } else if (status == robot::UartFrameParserStatus::InvalidFrame) {
        ++packet_error_count_;
      }
    }
  }

  bool configured() const { return configured_; }
  bool healthy(const robot::Milliseconds now_ms,
               const robot::Milliseconds timeout_ms) const {
    return configured_ && healthy_ &&
           elapsedSince(now_ms, last_rear_sent_at_ms_) <= timeout_ms;
  }
  bool remoteStatusFresh(const robot::Milliseconds now_ms,
                         const robot::Milliseconds timeout_ms) const {
    return configured_ && status_available_ &&
           elapsedSince(now_ms, last_status_received_at_ms_) <= timeout_ms;
  }
  robot::Milliseconds lastSentAtMs() const { return last_rear_sent_at_ms_; }
  robot::Milliseconds lastStatusReceivedAtMs() const {
    return last_status_received_at_ms_;
  }
  std::uint16_t lastSequenceSent() const { return last_rear_sequence_sent_; }
  std::uint32_t packetErrorCount() const { return packet_error_count_; }
  bool statusAvailable() const { return status_available_; }
  const robot::Esp1StatusReport& latestStatus() const {
    return latest_status_;
  }
  bool rearLineSnapshotAvailable() const {
    return rear_line_snapshot_available_;
  }
  bool rearLineSnapshotFresh(const robot::Milliseconds now_ms,
                             const robot::Milliseconds timeout_ms) const {
    return configured_ && rear_line_snapshot_available_ &&
           elapsedSince(now_ms, last_rear_line_received_at_ms_) <= timeout_ms;
  }
  const robot::RearLineSensorSnapshot& latestRearLineSnapshot() const {
    return latest_rear_line_snapshot_;
  }
  robot::Milliseconds lastRearLineReceivedAtMs() const {
    return last_rear_line_received_at_ms_;
  }
  std::uint16_t lastRearLineSequence() const {
    return last_rear_line_sequence_;
  }
  bool laserSnapshotAvailable() const {
    return laser_snapshot_available_;
  }
  const robot::LaserDistanceSnapshot& latestLaserSnapshot() const {
    return latest_laser_snapshot_;
  }
  robot::Milliseconds lastLaserSnapshotReceivedAtMs() const {
    return last_laser_snapshot_received_at_ms_;
  }
  robot::Milliseconds lastLaserMeasurementReceivedAtMs() const {
    return last_laser_measurement_received_at_ms_;
  }
  std::uint16_t lastLaserPacketSequence() const {
    return last_laser_packet_sequence_;
  }

 private:
  const robot::esp2::UartConfig& config_;
  robot::UartFrameParser parser_{};
  std::uint16_t next_sequence_{0};
  std::uint16_t last_rear_sequence_sent_{0};
  bool configured_{false};
  bool healthy_{false};
  bool status_available_{false};
  bool rear_line_snapshot_available_{false};
  bool laser_snapshot_available_{false};
  robot::Milliseconds last_rear_sent_at_ms_{0};
  robot::Milliseconds last_status_received_at_ms_{0};
  robot::Milliseconds last_rear_line_received_at_ms_{0};
  robot::Milliseconds last_laser_snapshot_received_at_ms_{0};
  robot::Milliseconds last_laser_measurement_received_at_ms_{0};
  std::uint32_t packet_error_count_{0};
  std::uint16_t last_rear_line_sequence_{0};
  std::uint16_t last_laser_packet_sequence_{0};
  robot::Esp1StatusReport latest_status_{};
  robot::RearLineSensorSnapshot latest_rear_line_snapshot_{};
  robot::LaserDistanceSnapshot latest_laser_snapshot_{};
};

enum class ClawServoPositionRequest : std::uint8_t {
  Closed = 0,
  Open = 1,
};

enum class ClawServoCommandResult : std::uint8_t {
  Accepted = 0,
  InvalidClaw = 1,
  HardwareUnconfigured = 2,
  OpenAngleUnset = 3,
  ClosedAngleUnset = 4,
  OpenAngleOutOfRange = 5,
  ClosedAngleOutOfRange = 6,
  PwmWriteFailed = 7,
};

struct ClawServoSettings {
  std::array<int, kMechanismServoCount> open_angle_deg{
      {23, 40, 80, kClawServoUnsetAngleDeg, 0}};
  std::array<int, kMechanismServoCount> closed_angle_deg{
      {110, 100, 180, kClawServoUnsetAngleDeg, 180}};
};

struct SolarHookServoSettings {
  int open_angle_deg{kClawServoUnsetAngleDeg};
  int closed_angle_deg{kClawServoUnsetAngleDeg};
};

const char* clawServoResultReason(const ClawServoCommandResult result) {
  switch (result) {
    case ClawServoCommandResult::Accepted:
      return "servo command accepted";
    case ClawServoCommandResult::InvalidClaw:
      return "invalid servo id";
    case ClawServoCommandResult::HardwareUnconfigured:
      return "servo PWM hardware is not configured";
    case ClawServoCommandResult::OpenAngleUnset:
      return "servo open angle is not set";
    case ClawServoCommandResult::ClosedAngleUnset:
      return "servo closed angle is not set";
    case ClawServoCommandResult::OpenAngleOutOfRange:
      return "servo open angle must be 0..180 degrees";
    case ClawServoCommandResult::ClosedAngleOutOfRange:
      return "servo closed angle must be 0..180 degrees";
    case ClawServoCommandResult::PwmWriteFailed:
      return "servo PWM peripheral write failed";
  }
  return "servo command rejected";
}

robot::FaultCode clawFaultCode(ClawServoCommandResult result);

class ClawServoBank {
 public:
  ClawServoBank(const robot::esp2::ServoOutputConfig& claw_1,
                const robot::esp2::ServoOutputConfig& claw_2,
                const robot::esp2::ServoOutputConfig& claw_3,
                const robot::esp2::ServoOutputConfig& habitat_pusher,
                const robot::esp2::ServoOutputConfig& winch)
      : configs_{{&claw_1, &claw_2, &claw_3, &habitat_pusher, &winch}} {}

  void initializeDisabled() {
    for (std::size_t index = 0U; index < kMechanismServoCount; ++index) {
      hardware_configured_[index] = initializeOutput(index);
    }
    disable();
  }

  void disable() {
    for (std::size_t index = 0U; index < kMechanismServoCount; ++index) {
      disableOutput(index);
    }
  }

  ClawServoCommandResult applySettings(
      const ClawServoSettings& settings) {
    const ClawServoCommandResult result = validateSettings(settings);
    if (result != ClawServoCommandResult::Accepted) {
      return result;
    }
    settings_ = settings;
    for (std::size_t index = 0U; index < kMechanismServoCount; ++index) {
      if (!output_enabled_[index]) {
        continue;
      }
      const int target_angle_deg =
          commanded_open_[index] ? settings_.open_angle_deg[index]
                                 : settings_.closed_angle_deg[index];
      if (angleConfigured(target_angle_deg) &&
          !writeAngle(index, target_angle_deg)) {
        disableOutput(index);
        return ClawServoCommandResult::PwmWriteFailed;
      }
    }
    return ClawServoCommandResult::Accepted;
  }

  const ClawServoSettings& settings() const { return settings_; }

  bool allTargetsConfigured() const {
    for (std::size_t index = 0U; index < kClawServoCount; ++index) {
      int target_angle_deg = kClawServoUnsetAngleDeg;
      if (targetAngle(index, ClawServoPositionRequest::Open,
                      target_angle_deg) !=
              ClawServoCommandResult::Accepted ||
          targetAngle(index, ClawServoPositionRequest::Closed,
                      target_angle_deg) !=
              ClawServoCommandResult::Accepted) {
        return false;
      }
    }
    int target_angle_deg = kClawServoUnsetAngleDeg;
    return targetAngle(kWinchServoIndex, ClawServoPositionRequest::Open,
                       target_angle_deg) ==
               ClawServoCommandResult::Accepted &&
           targetAngle(kWinchServoIndex, ClawServoPositionRequest::Closed,
                       target_angle_deg) ==
               ClawServoCommandResult::Accepted;
  }

  bool habitatPusherTargetsConfigured() const {
    int target_angle_deg = kClawServoUnsetAngleDeg;
    return targetAngle(kHabitatPusherServoIndex,
                       ClawServoPositionRequest::Open,
                       target_angle_deg) ==
               ClawServoCommandResult::Accepted &&
           targetAngle(kHabitatPusherServoIndex,
                       ClawServoPositionRequest::Closed,
                       target_angle_deg) ==
               ClawServoCommandResult::Accepted;
  }

  bool habitatPusherCommandedOpen() const {
    return output_enabled_[kHabitatPusherServoIndex] &&
           commanded_open_[kHabitatPusherServoIndex];
  }

  bool habitatPusherCommandedClosed() const {
    return output_enabled_[kHabitatPusherServoIndex] &&
           !commanded_open_[kHabitatPusherServoIndex] &&
           commanded_angle_deg_[kHabitatPusherServoIndex] !=
               kClawServoUnsetAngleDeg;
  }

  bool allClawOpenTargetsConfigured() const {
    for (std::size_t index = 0U; index < kClawServoCount; ++index) {
      int target_angle_deg = kClawServoUnsetAngleDeg;
      if (targetAngle(index, ClawServoPositionRequest::Open,
                      target_angle_deg) !=
          ClawServoCommandResult::Accepted) {
        return false;
      }
    }
    return true;
  }

  ClawServoCommandResult command(
      const std::size_t index,
      const ClawServoPositionRequest request) {
    int target_angle_deg = kClawServoUnsetAngleDeg;
    const ClawServoCommandResult result =
        targetAngle(index, request, target_angle_deg);
    if (result != ClawServoCommandResult::Accepted) {
      return result;
    }
    if (!writeAngle(index, target_angle_deg)) {
      disableOutput(index);
      return ClawServoCommandResult::PwmWriteFailed;
    }
    commanded_open_[index] = request == ClawServoPositionRequest::Open;
    return ClawServoCommandResult::Accepted;
  }

  ClawServoCommandResult commandAll(
      const ClawServoPositionRequest request) {
    std::array<int, kClawServoCount> target_angles_deg{
        {kClawServoUnsetAngleDeg, kClawServoUnsetAngleDeg,
         kClawServoUnsetAngleDeg}};
    for (std::size_t index = 0U; index < kClawServoCount; ++index) {
      const ClawServoCommandResult result =
          targetAngle(index, request, target_angles_deg[index]);
      if (result != ClawServoCommandResult::Accepted) {
        return result;
      }
    }
    for (std::size_t index = 0U; index < kClawServoCount; ++index) {
      if (!writeAngle(index, target_angles_deg[index])) {
        disableOutput(index);
        return ClawServoCommandResult::PwmWriteFailed;
      }
      commanded_open_[index] = request == ClawServoPositionRequest::Open;
    }
    return ClawServoCommandResult::Accepted;
  }

  void fillTelemetry(robot::ServoClawBankTelemetry& output) const {
    output = {};
    fillClawTelemetry(output.claw_1, 0U);
    fillClawTelemetry(output.claw_2, 1U);
    fillClawTelemetry(output.claw_3, 2U);
    fillClawTelemetry(output.habitat_pusher,
                      kHabitatPusherServoIndex);
    fillClawTelemetry(output.winch, kWinchServoIndex);
  }

 private:
  bool initializeOutput(const std::size_t index) {
    if (index >= kMechanismServoCount ||
        !servoOutputConfigComplete(*configs_[index])) {
      return false;
    }
    const robot::esp2::ServoOutputConfig& config = *configs_[index];
    pinMode(config.gpio, INPUT);
    if (config.backend == robot::esp2::ServoPwmBackend::Ledc) {
      ledcSetup(config.ledc_channel, config.pwm_frequency_hz,
                config.ledc_resolution_bits);
      return true;
    }
    if (config.backend != robot::esp2::ServoPwmBackend::Mcpwm) {
      return false;
    }

    mcpwm_config_t mcpwm_config{};
    mcpwm_config.frequency = config.pwm_frequency_hz;
    mcpwm_config.cmpr_a = 0.0F;
    mcpwm_config.cmpr_b = 0.0F;
    mcpwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_config.counter_mode = MCPWM_UP_COUNTER;
    const mcpwm_unit_t unit = mcpwmUnit(config);
    const mcpwm_timer_t timer = mcpwmTimer(config);
    const mcpwm_generator_t generator = mcpwmGenerator(config);
    if (mcpwm_timer_set_resolution(
            unit, timer, config.mcpwm_timer_resolution_hz) != ESP_OK ||
        mcpwm_init(unit, timer, &mcpwm_config) != ESP_OK ||
        mcpwm_set_signal_low(unit, timer, generator) != ESP_OK) {
      pinMode(config.gpio, INPUT);
      return false;
    }
    return true;
  }

  static bool angleConfigured(const int angle_deg) {
    return angle_deg != kClawServoUnsetAngleDeg;
  }

  static bool angleInRange(const int angle_deg) {
    return angle_deg >= 0 && angle_deg <= 180;
  }

  static ClawServoCommandResult validateSettings(
      const ClawServoSettings& settings) {
    for (std::size_t index = 0U; index < kMechanismServoCount; ++index) {
      const int open_angle_deg = settings.open_angle_deg[index];
      const int closed_angle_deg = settings.closed_angle_deg[index];
      if (angleConfigured(open_angle_deg) && !angleInRange(open_angle_deg)) {
        return ClawServoCommandResult::OpenAngleOutOfRange;
      }
      if (angleConfigured(closed_angle_deg) &&
          !angleInRange(closed_angle_deg)) {
        return ClawServoCommandResult::ClosedAngleOutOfRange;
      }
    }
    return ClawServoCommandResult::Accepted;
  }

  ClawServoCommandResult targetAngle(
      const std::size_t index, const ClawServoPositionRequest request,
      int& target_angle_deg) const {
    if (index >= kMechanismServoCount) {
      return ClawServoCommandResult::InvalidClaw;
    }
    if (!hardware_configured_[index]) {
      return ClawServoCommandResult::HardwareUnconfigured;
    }
    target_angle_deg =
        request == ClawServoPositionRequest::Closed
            ? settings_.closed_angle_deg[index]
            : settings_.open_angle_deg[index];
    if (!angleConfigured(target_angle_deg)) {
      return request == ClawServoPositionRequest::Closed
                 ? ClawServoCommandResult::ClosedAngleUnset
                 : ClawServoCommandResult::OpenAngleUnset;
    }
    if (!angleInRange(target_angle_deg)) {
      return request == ClawServoPositionRequest::Closed
                 ? ClawServoCommandResult::ClosedAngleOutOfRange
                 : ClawServoCommandResult::OpenAngleOutOfRange;
    }
    return ClawServoCommandResult::Accepted;
  }

  std::uint32_t pulseUsForAngle(const std::size_t index,
                                const int angle_deg) const {
    const robot::esp2::ServoOutputConfig& config = *configs_[index];
    const std::uint32_t pulse_range_us =
        static_cast<std::uint32_t>(config.maximum_pulse_us -
                                   config.minimum_pulse_us);
    return config.minimum_pulse_us +
           ((static_cast<std::uint32_t>(angle_deg) * pulse_range_us +
             90U) /
            180U);
  }

  std::uint32_t dutyForPulseUs(const std::size_t index,
                               const std::uint32_t pulse_us) const {
    const robot::esp2::ServoOutputConfig& config = *configs_[index];
    const std::uint32_t period_us = 1000000UL / config.pwm_frequency_hz;
    return ((pulse_us * pwmMaxDuty(config.ledc_resolution_bits)) +
            (period_us / 2U)) /
           period_us;
  }

  bool writeAngle(const std::size_t index, const int angle_deg) {
    const robot::esp2::ServoOutputConfig& config = *configs_[index];
    const std::uint32_t pulse_us = pulseUsForAngle(index, angle_deg);
    if (config.backend == robot::esp2::ServoPwmBackend::Ledc) {
      ledcSetup(config.ledc_channel, config.pwm_frequency_hz,
                config.ledc_resolution_bits);
      ledcAttachPin(config.gpio, config.ledc_channel);
      ledcWrite(config.ledc_channel,
                dutyForPulseUs(index, pulse_us));
    } else if (config.backend == robot::esp2::ServoPwmBackend::Mcpwm) {
      mcpwm_io_signals_t signal = MCPWM0A;
      if (!mcpwmSignal(config, signal)) {
        return false;
      }
      const mcpwm_unit_t unit = mcpwmUnit(config);
      const mcpwm_timer_t timer = mcpwmTimer(config);
      const mcpwm_generator_t generator = mcpwmGenerator(config);
      if (mcpwm_set_signal_low(unit, timer, generator) != ESP_OK ||
          mcpwm_set_duty_in_us(unit, timer, generator, pulse_us) != ESP_OK ||
          mcpwm_gpio_init(unit, signal, config.gpio) != ESP_OK ||
          mcpwm_set_duty_type(
              unit, timer, generator, MCPWM_DUTY_MODE_0) != ESP_OK) {
        (void)mcpwm_set_signal_low(unit, timer, generator);
        pinMode(config.gpio, INPUT);
        return false;
      }
    } else {
      return false;
    }
    output_enabled_[index] = true;
    commanded_angle_deg_[index] = angle_deg;
    return true;
  }

  void disableOutput(const std::size_t index) {
    if (index >= kMechanismServoCount) {
      return;
    }
    if (hardware_configured_[index]) {
      const robot::esp2::ServoOutputConfig& config = *configs_[index];
      if (config.backend == robot::esp2::ServoPwmBackend::Ledc) {
        ledcWrite(config.ledc_channel, 0U);
        ledcDetachPin(config.gpio);
      } else if (config.backend == robot::esp2::ServoPwmBackend::Mcpwm) {
        (void)mcpwm_set_signal_low(
            mcpwmUnit(config), mcpwmTimer(config), mcpwmGenerator(config));
      }
      pinMode(config.gpio, INPUT);
    }
    output_enabled_[index] = false;
    commanded_angle_deg_[index] = kClawServoUnsetAngleDeg;
    commanded_open_[index] = false;
  }

  void fillClawTelemetry(robot::ServoClawTelemetry& output,
                         const std::size_t index) const {
    const int open_angle_deg = settings_.open_angle_deg[index];
    const int closed_angle_deg = settings_.closed_angle_deg[index];
    const robot::esp2::ServoOutputConfig& config = *configs_[index];
    output.hardware_configured = hardware_configured_[index];
    output.gpio = config.gpio;
    output.ledc_channel = config.ledc_channel;
    output.mcpwm_unit = config.mcpwm_unit;
    output.mcpwm_timer = config.mcpwm_timer;
    output.mcpwm_generator = config.mcpwm_generator;
    output.pwm_frequency_hz = config.pwm_frequency_hz;
    output.mcpwm_timer_resolution_hz =
        config.mcpwm_timer_resolution_hz;
    output.open_configured = angleConfigured(open_angle_deg);
    output.closed_configured = angleConfigured(closed_angle_deg);
    output.output_enabled = output_enabled_[index];
    output.open_angle_deg = open_angle_deg;
    output.closed_angle_deg = closed_angle_deg;
    output.commanded_angle_deg = commanded_angle_deg_[index];
    output.commanded_open = commanded_open_[index];
  }

  std::array<const robot::esp2::ServoOutputConfig*, kMechanismServoCount>
      configs_;
  ClawServoSettings settings_{};
  std::array<bool, kMechanismServoCount> hardware_configured_{
      {false, false, false, false, false}};
  std::array<bool, kMechanismServoCount> output_enabled_{
      {false, false, false, false, false}};
  std::array<int, kMechanismServoCount> commanded_angle_deg_{
      {kClawServoUnsetAngleDeg, kClawServoUnsetAngleDeg,
       kClawServoUnsetAngleDeg, kClawServoUnsetAngleDeg,
       kClawServoUnsetAngleDeg}};
  std::array<bool, kMechanismServoCount> commanded_open_{
      {false, false, false, false, false}};
};

struct SolarIrThresholds {
  std::uint16_t detect_1khz{IR_BEACON_DETECT_THRESHOLD_1KHZ};
  std::uint16_t release_1khz{IR_BEACON_RELEASE_THRESHOLD_1KHZ};
  std::uint16_t detect_10khz{IR_BEACON_DETECT_THRESHOLD_10KHZ};
  std::uint16_t release_10khz{IR_BEACON_RELEASE_THRESHOLD_10KHZ};
};

struct SolarLineFollowSpeedConfig {
  float start_base_duty{SOLAR_START_BASE_DUTY};
  robot::Milliseconds slow_after_ms{SOLAR_SLOW_AFTER_MS};
  float slow_base_duty{SOLAR_SLOW_BASE_DUTY};
};

struct ImuTurnAvailabilityFaultCapture {
  bool valid{false};
  bool configured{false};
  bool initialized{false};
  bool calibrated{false};
  bool healthy{false};
  bool sample_valid{false};
  bool data_fresh{false};
  bool acquisition_running{false};
  bool shared_snapshot_available{false};
  bool front_left_configured{false};
  bool front_right_configured{false};
  bool rear_link_configured{false};
  bool rear_status_available{false};
  bool rear_status_fresh{false};
  bool newest_snapshot_available{false};
  bool cached_snapshot_matches_newest{false};
  char origin[robot::kTelemetryImuDiagnosticReasonSize]{};
  char reason[robot::kTelemetryImuDiagnosticReasonSize]{};
  std::uint32_t evaluated_at_us{0U};
  robot::Milliseconds evaluated_at_ms{0U};
  std::uint32_t published_at_us{0U};
  std::uint32_t last_successful_read_us{0U};
  std::uint32_t sample_age_us{0U};
  std::uint32_t snapshot_age_us{0U};
  std::uint32_t freshness_timeout_us{0U};
  std::uint32_t cached_snapshot_sequence{0U};
  std::uint32_t newest_snapshot_sequence{0U};
  std::uint32_t cached_successful_sample_sequence{0U};
  std::uint32_t newest_successful_sample_sequence{0U};
  std::uint32_t cached_snapshot_fetched_at_us{0U};
  std::uint32_t cached_snapshot_fetch_to_gate_us{0U};
  std::uint32_t successful_sample_publication_gap_us{0U};
  std::uint32_t maximum_successful_sample_publication_gap_us{0U};
  std::uint32_t current_observed_publication_gap_us{0U};
  std::uint32_t maximum_observed_publication_gap_us{0U};
  robot::Milliseconds rear_last_status_received_at_ms{0U};
  robot::Milliseconds rear_status_age_ms{0U};
};

struct RuntimeContext {
  robot::LineFollowerConfig config{};
  robot::LineFollowerConfig rear_config{};
  robot::LineFollowerState follower_state{};
  robot::RobotTestModeManager modes{};
  robot::EventLog events{};
  robot::MotionDiagnostics motion_diagnostics{};
  robot::FourWheelCommand requested_command{};
  robot::FourWheelCommand last_commanded_wheels{};
  robot::MotorCommand requested_funnel_command{};
  robot::LineFollowerUpdate last_update{};
  robot::LineFollowerUpdate last_rear_update{};
  robot::LineObservation last_line_observation{};
  robot::LineObservation last_rear_line_observation{};
  robot::SolarPanelAutonomyConfig solar_config{kSolarPanelAutonomyConfig};
  SolarIrThresholds solar_thresholds{};
  SolarLineFollowSpeedConfig solar_speed_config{};
  SolarHookServoSettings solar_hook_servo_settings{};
  robot::SolarPanelContactConfig solar_contact_config{
      kSolarPanelContactConfig};
  robot::SolarBeaconDetectorState solar_detector{};
  robot::SolarBeaconDetectorUpdate last_solar_detector_update{};
  robot::SolarPanelAutonomyState autonomous_state{
      robot::SolarPanelAutonomyState::WaitForStart};
  robot::SolarPanelFaultReason autonomous_fault_reason{
      robot::SolarPanelFaultReason::None};
  robot::HabitatPiecesConfig habitat_pieces_config{};
  robot::HabitatPiecesAutonomy habitat_pieces{};
  robot::HabitatPiecesUpdate last_habitat_pieces_update{};
  robot::HabitatPlacementConfig habitat_placement_config{};
  robot::HabitatPlacementAutonomy habitat_placement{};
  robot::HabitatPlacementUpdate last_habitat_placement_update{};
  robot::TowerPiecesConfig tower_pieces_config{};
  robot::TowerPiecesAutonomy tower_pieces{};
  robot::TowerPiecesUpdate last_tower_pieces_update{};
  robot::PegFinderConfig peg_finder_config{};
  robot::PegFinderAutonomy peg_finder{};
  robot::TimeTrialConfig time_trial_config{};
  robot::TimeTrialAutonomy time_trial{};
  robot::ImuTurnConfig imu_turn_config{};
  robot::ImuTurnControllerState imu_turn_state{};
  robot::ImuTurnUpdate last_imu_turn_update{};
  robot::ImuHeadingHoldConfig imu_heading_hold_config{};
  robot::ImuHeadingHoldControllerState imu_heading_hold_state{};
  robot::ImuHeadingHoldUpdate last_imu_heading_hold_update{};
  robot::ImuRecoveryState imu_turn_recovery{};
  robot::ImuRecoveryState imu_strafe_recovery{};
  robot::esp2::ImuAcquisitionSnapshot latest_imu_snapshot{};
  ImuTurnAvailabilityFaultCapture imu_turn_availability_fault{};
  std::uint32_t imu_heading_reset_pending_sequence{0U};
  robot::Milliseconds autonomous_state_entered_at_ms{0};
  char command_buffer[kSerialCommandBufferSize]{};
  std::size_t command_length{0};
  robot::Milliseconds last_telemetry_at_ms{0};
  robot::Milliseconds mode_expires_at_ms{0};
  robot::Milliseconds last_command_ms{0};
  robot::Milliseconds last_motion_diagnostic_sample_ms{0};
  std::uint32_t diagnostic_loop_started_us{0U};
  std::uint32_t diagnostic_loop_interval_us{0U};
  std::uint32_t diagnostic_imu_update_us{0U};
  std::uint32_t diagnostic_web_handle_us{0U};
  std::int8_t line_sensor_last_known_side{0};
  std::int8_t rear_line_sensor_last_known_side{0};
  bool command_deadman_armed{false};
  bool solar_start_requested{false};
  bool habitat_pieces_start_requested{false};
  bool habitat_placement_start_requested{false};
  bool tower_pieces_start_requested{false};
  bool peg_finder_start_requested{false};
  bool time_trial_start_requested{false};
  bool solar_hook_commanded_open{false};
  bool fault_active{false};
  bool imu_disconnect_active{false};
  bool imu_disconnect_observed{false};
  bool imu_shared_snapshot_available{false};
  std::uint32_t imu_shared_snapshot_fetched_at_us{0U};
  std::uint32_t maximum_observed_imu_publication_gap_us{0U};
  std::uint32_t imu_disconnect_count{0U};
  robot::Milliseconds last_imu_disconnect_at_ms{0U};
  char last_imu_disconnect_reason[
      robot::kTelemetryImuDiagnosticReasonSize]{};
  bool diagnostic_motion_was_active{false};
  bool diagnostic_web_stop_seen{false};
  robot::MotionDiagnosticTrigger diagnostic_freeze_pending{
      robot::MotionDiagnosticTrigger::None};
  robot::FaultCode fault_code{robot::FaultCode::None};
  char fault_message[robot::kTelemetryFaultMessageSize]{};
};

struct RuntimeBindings {
  RuntimeContext* context{nullptr};
  DigitalFrontLineSensorReader* sensors{nullptr};
  DigitalActiveHighLimitSwitch* peg_finder_funnel_limit{nullptr};
  DualPwmMotorOutput* front_left{nullptr};
  DualPwmMotorOutput* front_right{nullptr};
  RearCommandLink* rear_link{nullptr};
  ClawServoBank* claws{nullptr};
  robot::esp2::StepperAxis* stepper{nullptr};
  robot::esp2::ImuAcquisitionService* imu_acquisition{nullptr};
  Preferences* preferences{nullptr};
};

RuntimeBindings g_runtime{};

float hardwareDutyCap() {
  return clampFloat(robot::esp2::kHardwareConfig.maximum_safe_test_duty, 0.0F,
                    1.0F);
}

bool imuTurnRuntimeConfigValid(const robot::ImuTurnConfig& config) {
  return robot::imuTurnConfigValid(config, hardwareDutyCap()) &&
         config.timeout_ms <= kMaxTimedTestDurationMs;
}

bool imuHeadingHoldRuntimeConfigValid(
    const robot::ImuHeadingHoldConfig& config) {
  return robot::imuHeadingHoldConfigValid(config, hardwareDutyCap());
}

bool imuReadyForAutonomousMotion(const RuntimeContext& context);

bool autonomousStrafeDutyValid(const RuntimeContext& context,
                               const float duty) {
  robot::ImuHeadingHoldConfig config =
      context.imu_heading_hold_config;
  config.maximum_strafe_duty = duty;
  return imuHeadingHoldRuntimeConfigValid(config);
}

float activeMotionDutyCap(const RuntimeContext& context) {
  return clampFloat(context.config.maxDuty, 0.0F, hardwareDutyCap());
}

float rearMotionDutyCap(const RuntimeContext& context) {
  return clampFloat(context.rear_config.maxDuty, 0.0F, hardwareDutyCap());
}

float funnelMotionDutyCap() {
  return clampFloat(kSingleMotorDutyCap, 0.0F, hardwareDutyCap());
}

robot::LineFollowerConfig habitatPiecesLineFollowerConfig(
    const RuntimeContext& context) {
  robot::LineFollowerConfig config = context.config;
  config.baseDuty = context.habitat_pieces_config.line_follow_duty;
  return config;
}

robot::LineFollowerConfig reverseRearLineFollowerConfig(
    const RuntimeContext& context) {
  return robot::makeReverseTravelLineFollowerConfig(context.rear_config);
}

robot::LineFollowerConfig towerPiecesLineFollowerConfig(
    const RuntimeContext& context, const float base_duty) {
  robot::LineFollowerConfig config = context.rear_config;
  config.baseDuty =
      clampFloat(base_duty, 0.0F, rearMotionDutyCap(context));
  return robot::makeReverseTravelLineFollowerConfig(config);
}

robot::LineFollowerConfig habitatPlacementLineFollowerConfig(
    const RuntimeContext& context) {
  return towerPiecesLineFollowerConfig(
      context, context.habitat_placement_config.reverse_line_follow_duty);
}

robot::Milliseconds remoteStatusTimeoutMs(
    const robot::LineFollowerConfig& config) {
  const robot::Milliseconds base =
      config.remoteCommandTimeoutMs == 0U
          ? robot::kDefaultCommunicationTimeoutMs
          : config.remoteCommandTimeoutMs;
  return base > (UINT32_MAX / 2U) ? UINT32_MAX : base * 2U;
}

bool solarThresholdsValid(const SolarIrThresholds& thresholds) {
  return thresholds.release_1khz <= thresholds.detect_1khz &&
         thresholds.release_10khz <= thresholds.detect_10khz;
}

bool solarSpeedConfigValid(const SolarLineFollowSpeedConfig& config) {
  return std::isfinite(config.start_base_duty) &&
         config.start_base_duty >= 0.0F &&
         config.start_base_duty <= 1.0F &&
         std::isfinite(config.slow_base_duty) &&
         config.slow_base_duty >= 0.0F && config.slow_base_duty <= 1.0F;
}

robot::SolarPanelAutonomyConfig activeSolarPanelConfig(
    const RuntimeContext& context, const std::uint32_t frequency_hz) {
  robot::SolarPanelAutonomyConfig config = context.solar_config;
  if (frequency_hz == kIrBeaconFrequency10Khz) {
    config.detection_threshold = context.solar_thresholds.detect_10khz;
    config.release_threshold = context.solar_thresholds.release_10khz;
  } else {
    config.detection_threshold = context.solar_thresholds.detect_1khz;
    config.release_threshold = context.solar_thresholds.release_1khz;
  }
  return config;
}

bool solarPanelLimitSwitchHit(const bool raw_high) {
  return raw_high == SOLAR_LIMIT_SWITCH_HIT_WHEN_HIGH;
}

bool solarPanelLimitSwitchesAllHit(
    const robot::Esp1StatusReport& report) {
  return report.solar_panel_limit_switches_configured &&
         solarPanelLimitSwitchHit(report.solar_limit_back_right_high) &&
         solarPanelLimitSwitchHit(report.solar_limit_front_right_high);
}

bool solarPanelLimitSwitchesReady(
    const RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  return rear_link.remoteStatusFresh(now_ms,
                                     remoteStatusTimeoutMs(context.config)) &&
         rear_link.latestStatus().solar_panel_limit_switches_configured;
}

bool solarSlowModeActive(const RuntimeContext& context,
                         const robot::Milliseconds time_in_state_ms) {
  return context.solar_speed_config.slow_after_ms > 0U &&
         time_in_state_ms >= context.solar_speed_config.slow_after_ms;
}

robot::LineFollowerConfig activeSolarLineFollowerConfig(
    const RuntimeContext& context,
    const robot::Milliseconds time_in_state_ms) {
  robot::LineFollowerConfig config = context.config;
  const float requested_base_duty =
      solarSlowModeActive(context, time_in_state_ms)
          ? context.solar_speed_config.slow_base_duty
          : context.solar_speed_config.start_base_duty;
  config.baseDuty =
      clampFloat(requested_base_duty, 0.0F, activeMotionDutyCap(context));
  return config;
}

robot::CommandValidationLimits validationLimits(
    const RuntimeContext& context) {
  const float motion_cap = activeMotionDutyCap(context);
  return {motion_cap, clampFloat(kSingleMotorDutyCap, 0.0F, motion_cap),
          kMaxTimedTestDurationMs};
}

void copyText(char* destination, const std::size_t capacity,
              const char* text) {
  if (destination == nullptr || capacity == 0U) {
    return;
  }
  destination[0] = '\0';
  if (text != nullptr) {
    std::strncpy(destination, text, capacity - 1U);
    destination[capacity - 1U] = '\0';
  }
}

void setFault(RuntimeContext& context, const robot::FaultCode code,
              const char* message) {
  context.fault_active = code != robot::FaultCode::None;
  context.fault_code = code;
  copyText(context.fault_message, sizeof(context.fault_message), message);
}

void clearFault(RuntimeContext& context) {
  setFault(context, robot::FaultCode::None, "");
}

void logEvent(RuntimeContext& context, const robot::Milliseconds now_ms,
              const robot::EventSeverity severity,
              const robot::EventSource source, const char* message) {
  context.events.add(now_ms, severity, source, message);
}

void resetSolarPanelAutonomy(RuntimeContext& context,
                             const robot::Milliseconds now_ms) {
  robot::resetSolarBeaconDetectorState(context.solar_detector);
  context.last_solar_detector_update = {};
  context.autonomous_state = robot::SolarPanelAutonomyState::WaitForStart;
  context.autonomous_fault_reason = robot::SolarPanelFaultReason::None;
  context.autonomous_state_entered_at_ms = now_ms;
  context.solar_start_requested = false;
  robot::resetImuHeadingHoldController(context.imu_heading_hold_state);
  context.last_imu_heading_hold_update = {};
  robot::cancelImuRecovery(context.imu_strafe_recovery);
}

void resetHabitatPieces(RuntimeContext& context,
                        const robot::Milliseconds now_ms) {
  if (g_runtime.stepper != nullptr) {
    g_runtime.stepper->stop();
  }
  robot::resetHabitatPiecesAutonomy(context.habitat_pieces, now_ms);
  context.last_habitat_pieces_update = {};
  context.habitat_pieces_start_requested = false;
}

void resetHabitatPlacement(RuntimeContext& context,
                           const robot::Milliseconds now_ms) {
  if (g_runtime.stepper != nullptr) {
    g_runtime.stepper->stop();
  }
  robot::resetHabitatPlacementAutonomy(context.habitat_placement, now_ms);
  context.last_habitat_placement_update = {};
  context.habitat_placement_start_requested = false;
  robot::resetImuTurnController(context.imu_turn_state);
  context.last_imu_turn_update = {};
  robot::cancelImuRecovery(context.imu_turn_recovery);
}

void resetTowerPieces(RuntimeContext& context,
                      const robot::Milliseconds now_ms) {
  if (g_runtime.stepper != nullptr) {
    g_runtime.stepper->stop();
  }
  robot::resetTowerPiecesAutonomy(context.tower_pieces, now_ms);
  context.tower_pieces_start_requested = false;
  robot::resetImuTurnController(context.imu_turn_state);
  context.last_imu_turn_update = {};
  robot::resetImuHeadingHoldController(context.imu_heading_hold_state);
  context.last_imu_heading_hold_update = {};
  robot::cancelImuRecovery(context.imu_turn_recovery);
  robot::cancelImuRecovery(context.imu_strafe_recovery);
}

void resetPegFinder(RuntimeContext& context,
                    const robot::Milliseconds now_ms) {
  robot::resetPegFinderAutonomy(context.peg_finder, now_ms);
  context.peg_finder_start_requested = false;
  robot::resetImuTurnController(context.imu_turn_state);
  context.last_imu_turn_update = {};
  robot::cancelImuRecovery(context.imu_turn_recovery);
}

void resetTimeTrial(RuntimeContext& context,
                    const robot::Milliseconds now_ms) {
  robot::resetTimeTrialAutonomy(context.time_trial, now_ms);
  context.time_trial_start_requested = false;
  robot::resetImuHeadingHoldController(context.imu_heading_hold_state);
  context.last_imu_heading_hold_update = {};
  robot::cancelImuRecovery(context.imu_strafe_recovery);
}

void resetImuTurn(RuntimeContext& context) {
  robot::resetImuTurnController(context.imu_turn_state);
  context.last_imu_turn_update = {};
  robot::cancelImuRecovery(context.imu_turn_recovery);
}

void resetImuHeadingHold(RuntimeContext& context) {
  robot::resetImuHeadingHoldController(context.imu_heading_hold_state);
  context.last_imu_heading_hold_update = {};
  robot::cancelImuRecovery(context.imu_strafe_recovery);
}

void enterSolarPanelAutonomyState(
    RuntimeContext& context,
    const robot::SolarPanelAutonomyState state,
    const robot::Milliseconds now_ms,
    const robot::SolarPanelFaultReason fault_reason =
        robot::SolarPanelFaultReason::None) {
  if (context.autonomous_state == state &&
      context.autonomous_fault_reason == fault_reason) {
    return;
  }
  context.autonomous_state = state;
  context.autonomous_fault_reason = fault_reason;
  context.autonomous_state_entered_at_ms = now_ms;
}

robot::MotorCommand makeTimedMotorCommand(
    const float duty, const robot::Milliseconds now_ms,
    const robot::Milliseconds duration_ms) {
  robot::MotorCommand command{};
  command.enabled = std::fabs(duty) > 0.0001F;
  command.duty_command_milli =
      robot::clampCommandMilli(static_cast<std::int16_t>(duty * 1000.0F));
  command.expires_at_ms = now_ms + duration_ms;
  return command;
}

robot::FourWheelCommand makeTowerPiecesBackwardCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.tower_pieces_config.reverse_duty, 0.0F,
                 rearMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(
      0.0F, -1.0F, 0.0F, duty, now_ms,
      context.rear_config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeHabitatPiecesBackwardCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.habitat_pieces_config.reverse_duty, 0.0F,
                 activeMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(
      0.0F, -1.0F, 0.0F, duty, now_ms,
      context.config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeHabitatPiecesTranslationCommand(
    const RuntimeContext& context, const float lateral,
    const float forward, const float requested_duty,
    const robot::Milliseconds now_ms) {
  const float duty = clampFloat(requested_duty, 0.0F,
                                activeMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(
      lateral, forward, 0.0F, duty, now_ms,
      context.config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeHabitatPlacementTranslationCommand(
    const RuntimeContext& context, const float vx, const float vy,
    const float duty, const robot::Milliseconds now_ms) {
  return robot::mixOpenLoopMecanum(
      vx, vy, 0.0F,
      clampFloat(duty, 0.0F, rearMotionDutyCap(context)), now_ms,
      context.rear_config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeImuTurnCommand(
    const RuntimeContext& context, const float rotation_command,
    const robot::Milliseconds now_ms) {
  const float signed_yaw_command =
      rotation_command *
      static_cast<float>(context.imu_turn_config.yaw_command_polarity);
  return robot::mixOpenLoopMecanum(
      0.0F, 0.0F, signed_yaw_command < 0.0F ? -1.0F : 1.0F,
      std::fabs(signed_yaw_command), now_ms,
      context.rear_config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makePegFinderBackwardCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.peg_finder_config.reverse_duty, 0.0F,
                 rearMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(
      0.0F, -1.0F, 0.0F, duty, now_ms,
      context.rear_config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeTowerPiecesFinalBackwardCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.tower_pieces_config.final_reverse_duty, 0.0F,
                 rearMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(
      0.0F, -1.0F, 0.0F, duty, now_ms,
      context.rear_config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makePegFinderForwardCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.peg_finder_config.forward_duty, 0.0F,
                 rearMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(
      0.0F, 1.0F, 0.0F, duty, now_ms,
      context.rear_config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeSolarForwardCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.solar_contact_config.retry_forward_duty, 0.0F,
                 activeMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(0.0F, 1.0F, 0.0F, duty, now_ms,
                                   context.config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeSolarPostContactForwardCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.solar_contact_config.post_contact_forward_duty,
                 0.0F, activeMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(0.0F, 1.0F, 0.0F, duty, now_ms,
                                   context.config.remoteCommandTimeoutMs);
}

bool sendStoppedRearCommand(RearCommandLink& rear_link,
                            const robot::LineFollowerConfig& config,
                            const robot::Milliseconds now_ms,
                            const robot::LaserDistanceProfile laser_profile =
                                robot::LaserDistanceProfile::HighAccuracy) {
  robot::RearDriveCommand command{};
  command.enabled = false;
  command.sender_timestamp_ms = now_ms;
  command.timeout_ms = config.remoteCommandTimeoutMs;
  command.laser_profile = laser_profile;
  return rear_link.send(command);
}

bool sendFunnelMotorCommand(RearCommandLink& rear_link,
                            const robot::MotorCommand& motor,
                            const robot::LineFollowerConfig& config,
                            const robot::Milliseconds now_ms) {
  robot::FunnelCommand command{};
  command.enabled = motor.enabled;
  command.command_milli = motor.duty_command_milli;
  command.sender_timestamp_ms = now_ms;
  command.timeout_ms = config.remoteCommandTimeoutMs;
  return rear_link.send(command);
}

bool sendStoppedFunnelCommand(RearCommandLink& rear_link,
                              const robot::LineFollowerConfig& config,
                              const robot::Milliseconds now_ms) {
  return sendFunnelMotorCommand(rear_link, robot::disabledMotorCommand(), config,
                                now_ms);
}

bool sendStoppedSolarHookCommand(RearCommandLink& rear_link) {
  robot::SolarHookCommand command{};
  command.enabled = false;
  command.angle_deg = 0U;
  return rear_link.send(command);
}

bool disableMotionActuators(RuntimeContext& context,
                            robot::IMotorOutput& front_left,
                            robot::IMotorOutput& front_right,
                            RearCommandLink& rear_link,
                            const robot::Milliseconds now_ms,
                            const robot::LaserDistanceProfile laser_profile =
                                robot::LaserDistanceProfile::HighAccuracy) {
  robot::stopLineFollower(context.follower_state);
  context.requested_command = robot::disabledFourWheelCommand();
  context.last_commanded_wheels = robot::disabledFourWheelCommand();
  context.command_deadman_armed = false;
  context.mode_expires_at_ms = 0U;
  front_left.disable();
  front_right.disable();
  return sendStoppedRearCommand(rear_link, context.config, now_ms,
                                laser_profile);
}

bool stopAutonomyDriveAndFunnel(RuntimeContext& context,
                                robot::IMotorOutput& front_left,
                                robot::IMotorOutput& front_right,
                                RearCommandLink& rear_link,
                                const robot::Milliseconds now_ms) {
  const bool rear_stopped =
      disableMotionActuators(context, front_left, front_right, rear_link,
                             now_ms);
  context.requested_funnel_command = robot::disabledMotorCommand();
  const bool funnel_stopped =
      sendStoppedFunnelCommand(rear_link, context.rear_config, now_ms);
  return rear_stopped && funnel_stopped;
}

void disableActuators(RuntimeContext& context, robot::IMotorOutput& front_left,
                      robot::IMotorOutput& front_right,
                      RearCommandLink& rear_link,
                      const robot::Milliseconds now_ms) {
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  if (g_runtime.stepper != nullptr) {
    g_runtime.stepper->stop();
  }
  context.requested_funnel_command = robot::disabledMotorCommand();
  sendStoppedFunnelCommand(rear_link, context.config, now_ms);
  sendStoppedSolarHookCommand(rear_link);
  if (g_runtime.claws != nullptr) {
    g_runtime.claws->disable();
  }
}

void emergencyStop(RuntimeContext& context, robot::IMotorOutput& front_left,
                   robot::IMotorOutput& front_right,
                   RearCommandLink& rear_link,
                   const robot::Milliseconds now_ms,
                   const robot::EventSource source) {
  disableActuators(context, front_left, front_right, rear_link, now_ms);
  context.modes.emergencyStop(now_ms);
  if (g_runtime.stepper != nullptr) g_runtime.stepper->stop();
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  resetImuTurn(context);
  resetImuHeadingHold(context);
  setFault(context, robot::FaultCode::None, "");
  logEvent(context, now_ms, robot::EventSeverity::Warn, source,
           "emergency stop requested");
}

bool allWheelCommandsDisabled(const robot::FourWheelCommand& command) {
  return !command.front_left.enabled && !command.front_right.enabled &&
         !command.back_left.enabled && !command.back_right.enabled;
}

std::uint16_t commandMagnitudeMilli(const robot::MotorCommand& command) {
  return static_cast<std::uint16_t>(std::abs(command.duty_command_milli));
}

std::uint16_t driveCommandMagnitudeMilli(
    const robot::FourWheelCommand& command) {
  std::uint16_t magnitude = commandMagnitudeMilli(command.front_left);
  const std::uint16_t front_right =
      commandMagnitudeMilli(command.front_right);
  const std::uint16_t back_left = commandMagnitudeMilli(command.back_left);
  const std::uint16_t back_right = commandMagnitudeMilli(command.back_right);
  magnitude = front_right > magnitude ? front_right : magnitude;
  magnitude = back_left > magnitude ? back_left : magnitude;
  return back_right > magnitude ? back_right : magnitude;
}

void recordMotionDiagnostic(
    RuntimeContext& context, const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right, const RearCommandLink& rear_link,
    const robot::esp2::ImuAcquisitionSnapshot* imu,
    const robot::MotionDiagnosticEvent event,
    const robot::Milliseconds now_ms) {
  robot::MotionDiagnosticSample sample{};
  sample.timestamp_ms = now_ms;
  sample.event = event;
  sample.mode = context.modes.currentMode();
  sample.loop_interval_us = context.diagnostic_loop_interval_us;
  sample.loop_work_us =
      context.diagnostic_loop_started_us == 0U
          ? 0U
          : static_cast<std::uint32_t>(
                micros() - context.diagnostic_loop_started_us);
  sample.imu_update_us = context.diagnostic_imu_update_us;
  sample.web_handle_us = context.diagnostic_web_handle_us;

  sample.requested_command_milli[0] =
      context.requested_command.front_left.duty_command_milli;
  sample.requested_command_milli[1] =
      context.requested_command.front_right.duty_command_milli;
  sample.requested_command_milli[2] =
      context.requested_command.back_left.duty_command_milli;
  sample.requested_command_milli[3] =
      context.requested_command.back_right.duty_command_milli;
  sample.front_driver_desired_command_milli[0] =
      front_left.lastDesiredCommand().duty_command_milli;
  sample.front_driver_desired_command_milli[1] =
      front_right.lastDesiredCommand().duty_command_milli;
  sample.front_command_expires_at_ms[0] =
      front_left.lastDesiredCommand().expires_at_ms;
  sample.front_command_expires_at_ms[1] =
      front_right.lastDesiredCommand().expires_at_ms;
  sample.applied_command_milli[0] =
      front_left.lastAppliedCommand().duty_command_milli;
  sample.applied_command_milli[1] =
      front_right.lastAppliedCommand().duty_command_milli;
  sample.front_pwm_readback[0] = front_left.pwm0DutyReadback();
  sample.front_pwm_readback[1] = front_left.pwm1DutyReadback();
  sample.front_pwm_readback[2] = front_right.pwm0DutyReadback();
  sample.front_pwm_readback[3] = front_right.pwm1DutyReadback();

  sample.rear_sequence = rear_link.lastSequenceSent();
  sample.rear_command_age_ms =
      rear_link.lastSentAtMs() == 0U
          ? 0U
          : elapsedSince(now_ms, rear_link.lastSentAtMs());
  sample.esp1_status_age_ms =
      rear_link.lastStatusReceivedAtMs() == 0U
          ? 0U
          : elapsedSince(now_ms, rear_link.lastStatusReceivedAtMs());
  sample.esp1_status_fresh = rear_link.remoteStatusFresh(
      now_ms, remoteStatusTimeoutMs(context.config));
  sample.esp1_packet_error_count = rear_link.packetErrorCount();
  if (rear_link.statusAvailable()) {
    sample.applied_command_milli[2] =
        rear_link.latestStatus().back_left_applied_command_milli;
    sample.applied_command_milli[3] =
        rear_link.latestStatus().back_right_applied_command_milli;
  }

  sample.command_deadman_armed = context.command_deadman_armed;
  sample.command_deadline_ms = context.mode_expires_at_ms;
  sample.last_command_age_ms =
      context.last_command_ms == 0U
          ? 0U
          : elapsedSince(now_ms, context.last_command_ms);

  sample.imu_turn_state = context.imu_turn_state.state;
  if (context.modes.currentMode() ==
      robot::RobotTestMode::ImuStrafeTest) {
    sample.target_heading_deg =
        context.imu_heading_hold_state.target_heading_deg;
    sample.rotation_command =
        context.last_imu_heading_hold_update.yaw_correction_duty;
  } else {
    sample.target_heading_deg =
        context.imu_turn_state.target_heading_deg;
    sample.rotation_command =
        context.last_imu_turn_update.rotation_command;
  }
  if (imu != nullptr) {
    sample.heading_deg = imu->state.heading_deg;
    sample.angle_error_deg =
        sample.target_heading_deg - sample.heading_deg;
    sample.yaw_rate_dps = imu->state.yaw_rate_dps;
  }
  context.motion_diagnostics.record(sample);
}

void scheduleMotionDiagnosticFreeze(
    RuntimeContext& context,
    const robot::MotionDiagnosticTrigger trigger) {
  if (context.diagnostic_freeze_pending ==
      robot::MotionDiagnosticTrigger::None) {
    context.diagnostic_freeze_pending = trigger;
  }
}

void resetMotionDiagnosticCapture(RuntimeContext& context,
                                  const robot::Milliseconds now_ms) {
  context.motion_diagnostics.reset(now_ms);
  context.last_motion_diagnostic_sample_ms = 0U;
  context.diagnostic_motion_was_active = false;
  context.diagnostic_web_stop_seen = false;
  context.diagnostic_freeze_pending =
      robot::MotionDiagnosticTrigger::None;
}

bool diagnosticMotionActive(const RuntimeContext& context,
                            const DualPwmMotorOutput& front_left,
                            const DualPwmMotorOutput& front_right,
                            const RearCommandLink& rear_link) {
  if (context.command_deadman_armed ||
      robot::imuTurnActive(context.imu_turn_state) ||
      robot::imuHeadingHoldActive(context.imu_heading_hold_state) ||
      driveCommandMagnitudeMilli(context.requested_command) > 0U ||
      front_left.lastAppliedCommand().enabled ||
      front_right.lastAppliedCommand().enabled) {
    return true;
  }
  if (!rear_link.statusAvailable()) {
    return false;
  }
  const robot::Esp1StatusReport& status = rear_link.latestStatus();
  return status.back_left_applied_command_milli != 0 ||
         status.back_right_applied_command_milli != 0;
}

bool startRequirementsMet(const DigitalFrontLineSensorReader& sensors,
                          const DualPwmMotorOutput& front_left,
                          const DualPwmMotorOutput& front_right,
                          const RearCommandLink& rear_link,
                          const robot::Milliseconds now_ms,
                          const RuntimeContext& context) {
  return sensors.configured() && front_left.configured() &&
         front_right.configured() && rear_link.configured() &&
         rear_link.remoteStatusFresh(now_ms,
                                     remoteStatusTimeoutMs(context.config)) &&
         context.config.maxDuty > 0.0F && hardwareDutyCap() > 0.0F;
}

bool habitatPiecesMotionRequirementsMet(
    const DigitalFrontLineSensorReader& sensors,
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  const robot::LineFollowerConfig line_config =
      habitatPiecesLineFollowerConfig(context);
  return startRequirementsMet(sensors, front_left, front_right, rear_link,
                              now_ms, context) &&
         robot::validateLineFollowerConfig(line_config, hardwareDutyCap())
             .accepted &&
         stepper.maximumPositionSteps() > 0 &&
         stepper.maximumPositionSteps() <=
             static_cast<std::int64_t>(UINT32_MAX) &&
         robot::habitatPiecesConfigValid(context.habitat_pieces_config,
                                         activeMotionDutyCap(context),
                                         kMaxTimedTestDurationMs,
                                         stepper.maximumSpeedStepsPerSecond(),
                                         static_cast<std::uint32_t>(
                                             stepper.maximumPositionSteps())) &&
         gpioAssigned(robot::esp2::kPins.stepper_sleep) &&
         gpioAssigned(robot::esp2::kPins.stepper_dir) &&
         gpioAssigned(robot::esp2::kPins.stepper_step) &&
         gpioAssigned(robot::esp2::kPins.limit_switch_stepper_bottom) &&
         gpioAssigned(robot::esp2::kPins.limit_switch_stepper_top);
}

bool habitatPiecesStartRequirementsMet(
    const DigitalFrontLineSensorReader& sensors,
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  return habitatPiecesMotionRequirementsMet(
             sensors, front_left, front_right, rear_link, stepper, now_ms,
             context) &&
         rear_link.rearLineSnapshotFresh(
             now_ms, context.config.remoteCommandTimeoutMs) &&
         rear_link.latestRearLineSnapshot().side_2_configured &&
         rear_link.latestRearLineSnapshot().side_3_configured &&
         rear_link.latestRearLineSnapshot().configured;
}

bool rearLineStartRequirementsMet(
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  return front_left.configured() && front_right.configured() &&
         rear_link.configured() &&
         rear_link.remoteStatusFresh(
             now_ms, remoteStatusTimeoutMs(context.rear_config)) &&
         rear_link.rearLineSnapshotFresh(
             now_ms, context.rear_config.remoteCommandTimeoutMs) &&
         rear_link.latestRearLineSnapshot().configured &&
         context.rear_config.maxDuty > 0.0F && hardwareDutyCap() > 0.0F;
}

bool towerPiecesStartRequirementsMet(
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const ClawServoBank& claws,
    const robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  return rearLineStartRequirementsMet(front_left, front_right, rear_link,
                                      now_ms, context) &&
         rear_link.latestRearLineSnapshot().side_configured &&
         claws.allTargetsConfigured() &&
         gpioAssigned(robot::esp2::kPins.stepper_sleep) &&
         gpioAssigned(robot::esp2::kPins.stepper_dir) &&
         gpioAssigned(robot::esp2::kPins.stepper_step) &&
         gpioAssigned(robot::esp2::kPins.limit_switch_stepper_bottom) &&
         gpioAssigned(robot::esp2::kPins.limit_switch_stepper_top) &&
         stepper.maximumPositionSteps() > 0 &&
         !(stepper.lowerLimitActive() && stepper.upperLimitActive()) &&
         imuReadyForAutonomousMotion(context) &&
         imuHeadingHoldRuntimeConfigValid(
             context.imu_heading_hold_config) &&
         autonomousStrafeDutyValid(
             context, context.tower_pieces_config.strafe_right_duty) &&
         autonomousStrafeDutyValid(
             context, context.tower_pieces_config.shimmy_duty) &&
         imuTurnRuntimeConfigValid(context.imu_turn_config) &&
         robot::towerPiecesConfigValid(context.tower_pieces_config,
                                       rearMotionDutyCap(context),
                                       stepper.maximumSpeedStepsPerSecond());
}

bool habitatPlacementStartRequirementsMet(
    const DigitalFrontLineSensorReader& sensors,
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const ClawServoBank& claws,
    const robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  const robot::HabitatPlacementConfig& config =
      context.habitat_placement_config;
  const bool bounded_timings =
      config.lss1_timeout_ms <= kMaxTimedTestDurationMs &&
      config.post_lss1_delay_ms <= kMaxTimedTestDurationMs &&
      config.initial_heading_turn_timeout_ms <=
          kMaxTimedTestDurationMs &&
      config.pre_counter_clockwise_strafe_right_duration_ms <=
          kMaxTimedTestDurationMs &&
      config.counter_clockwise_timeout_ms <= kMaxTimedTestDurationMs &&
      config.forward_to_slide_duration_ms <= kMaxTimedTestDurationMs &&
      config.stepper_down_timeout_ms <= kMaxTimedTestDurationMs &&
      config.pusher_open_settle_ms <= kMaxTimedTestDurationMs &&
      config.push_forward_duration_ms <= kMaxTimedTestDurationMs &&
      config.reverse_retreat_duration_ms <= kMaxTimedTestDurationMs &&
      config.clockwise_timeout_ms <= kMaxTimedTestDurationMs &&
      config.post_clockwise_reverse_duration_ms <=
          kMaxTimedTestDurationMs &&
      config.post_clockwise_strafe_left_duration_ms <=
          kMaxTimedTestDurationMs &&
      config.post_clockwise_delay_ms <= kMaxTimedTestDurationMs &&
      config.exit_forward_duration_ms <= kMaxTimedTestDurationMs &&
      config.post_forward_delay_ms <= kMaxTimedTestDurationMs &&
      config.strafe_right_timeout_ms <= kMaxTimedTestDurationMs;
  robot::ImuTurnConfig ccw_turn_config = context.imu_turn_config;
  ccw_turn_config.timeout_ms = config.counter_clockwise_timeout_ms;
  robot::ImuTurnConfig initial_heading_turn_config =
      context.imu_turn_config;
  initial_heading_turn_config.timeout_ms =
      config.initial_heading_turn_timeout_ms;
  robot::ImuTurnConfig cw_turn_config = context.imu_turn_config;
  cw_turn_config.timeout_ms = config.clockwise_timeout_ms;
  return sensors.configured() &&
         rearLineStartRequirementsMet(front_left, front_right, rear_link,
                                      now_ms, context) &&
         rear_link.latestRearLineSnapshot().side_configured &&
         claws.habitatPusherTargetsConfigured() &&
         gpioAssigned(robot::esp2::kPins.stepper_sleep) &&
         gpioAssigned(robot::esp2::kPins.stepper_dir) &&
         gpioAssigned(robot::esp2::kPins.stepper_step) &&
         gpioAssigned(robot::esp2::kPins.limit_switch_stepper_bottom) &&
         !(stepper.lowerLimitActive() && stepper.upperLimitActive()) &&
         imuReadyForAutonomousMotion(context) &&
         imuTurnRuntimeConfigValid(initial_heading_turn_config) &&
         imuTurnRuntimeConfigValid(ccw_turn_config) &&
         imuTurnRuntimeConfigValid(cw_turn_config) && bounded_timings &&
         robot::validateLineFollowerConfig(
             habitatPlacementLineFollowerConfig(context),
             hardwareDutyCap())
             .accepted &&
         robot::habitatPlacementConfigValid(
             config, rearMotionDutyCap(context),
             stepper.maximumSpeedStepsPerSecond());
}

bool pegFinderStartRequirementsMet(
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const ClawServoBank& claws,
    const DigitalActiveHighLimitSwitch& funnel_limit,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  return front_left.configured() && front_right.configured() &&
         rear_link.configured() &&
         rear_link.remoteStatusFresh(
             now_ms, remoteStatusTimeoutMs(context.rear_config)) &&
         rear_link.latestStatus().funnel_configured &&
         funnel_limit.configured() &&
         claws.allClawOpenTargetsConfigured() &&
         context.imu_heading_reset_pending_sequence == 0U &&
         std::strcmp(robot::esp2::imuDisconnectReason(
                         context.latest_imu_snapshot, micros(),
                         kImuFreshnessTimeoutUs),
                     "NONE") == 0 &&
         imuTurnRuntimeConfigValid(context.imu_turn_config) &&
         context.rear_config.maxDuty > 0.0F && hardwareDutyCap() > 0.0F &&
         robot::pegFinderConfigValid(context.peg_finder_config,
                                     rearMotionDutyCap(context),
                                     funnelMotionDutyCap());
}

bool solarPanelStartRequirementsMet(
    const DigitalFrontLineSensorReader& sensors,
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  return startRequirementsMet(sensors, front_left, front_right, rear_link,
                              now_ms, context) &&
         solarPanelLimitSwitchesReady(rear_link, now_ms, context) &&
         rear_link.rearLineSnapshotFresh(
             now_ms, context.rear_config.remoteCommandTimeoutMs) &&
         rear_link.latestRearLineSnapshot().configured &&
         imuReadyForAutonomousMotion(context) &&
         imuHeadingHoldRuntimeConfigValid(
             context.imu_heading_hold_config) &&
         robot::solarPanelContactConfigValid(
             context.solar_contact_config) &&
         autonomousStrafeDutyValid(
             context,
             context.solar_contact_config.initial_strafe_right_duty) &&
         autonomousStrafeDutyValid(
             context,
             context.solar_contact_config.retry_strafe_left_duty) &&
         autonomousStrafeDutyValid(
             context,
             context.solar_contact_config.retry_strafe_right_duty) &&
         autonomousStrafeDutyValid(
             context,
             context.solar_contact_config
                 .line_reacquire_strafe_duty);
}

bool timeTrialStartRequirementsMet(
    const DigitalFrontLineSensorReader& sensors,
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link, const ClawServoBank& claws,
    const DigitalActiveHighLimitSwitch& funnel_limit,
    const robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms, const RuntimeContext& context) {
  return solarPanelStartRequirementsMet(sensors, front_left, front_right,
                                        rear_link, now_ms, context) &&
         towerPiecesStartRequirementsMet(front_left, front_right, rear_link,
                                         claws, stepper, now_ms, context) &&
         pegFinderStartRequirementsMet(front_left, front_right, rear_link,
                                       claws, funnel_limit, now_ms, context) &&
         robot::timeTrialConfigValid(context.time_trial_config,
                                     rearMotionDutyCap(context));
}

bool sendRearWheelCommand(RearCommandLink& rear_link,
                          const robot::FourWheelCommand& wheels,
                          const robot::LineFollowerConfig& config,
                          const robot::Milliseconds now_ms,
                          const robot::LaserDistanceProfile laser_profile =
                              robot::LaserDistanceProfile::HighAccuracy) {
  robot::RearDriveCommand rear{};
  rear.enabled = wheels.back_left.enabled || wheels.back_right.enabled;
  rear.back_left_command_milli = wheels.back_left.duty_command_milli;
  rear.back_right_command_milli = wheels.back_right.duty_command_milli;
  rear.sender_timestamp_ms = now_ms;
  rear.timeout_ms = config.remoteCommandTimeoutMs;
  rear.laser_profile = laser_profile;
  return rear_link.send(rear);
}

bool applyWheelCommand(RuntimeContext& context,
                       robot::IMotorOutput& front_left,
                       robot::IMotorOutput& front_right,
                       RearCommandLink& rear_link,
                       const robot::FourWheelCommand& wheels,
                       const robot::LineFollowerConfig& command_config,
                       const robot::Milliseconds now_ms,
                       const robot::LaserDistanceProfile laser_profile =
                           robot::LaserDistanceProfile::HighAccuracy) {
  context.last_commanded_wheels = wheels;
  front_left.apply(wheels.front_left);
  front_right.apply(wheels.front_right);
  return sendRearWheelCommand(rear_link, wheels, command_config, now_ms,
                              laser_profile);
}

enum class AutonomousImuMotionResult {
  Running,
  Complete,
  InvalidConfiguration,
  ImuUnavailable,
  MotorUnavailable,
  RearLinkUnavailable,
  ControllerFault,
  Timeout,
  CommandFailed,
};

enum class AutonomousImuRecoveryResult {
  Continue,
  Paused,
  TimedOut,
  StopCommandFailed,
};

struct AutonomousImuRecoveryUpdate {
  AutonomousImuRecoveryResult result{
      AutonomousImuRecoveryResult::Continue};
  robot::Milliseconds timer_adjustment_ms{0U};
};

const char* imuUnavailableReason(const RuntimeContext& context) {
  const robot::esp2::ImuState& imu_state =
      context.latest_imu_snapshot.state;
  if (context.imu_heading_reset_pending_sequence != 0U) {
    return "HEADING_RESET_PENDING";
  }
  if (!imu_state.configured || !imu_state.initialized ||
      !imu_state.calibrated) {
    return robot::esp2::imuDisconnectReason(
        context.latest_imu_snapshot, micros(),
        kImuFreshnessTimeoutUs);
  }
  if (!imu_state.runtime_configuration_valid) {
    return "RUNTIME_CONFIGURATION_INVALID";
  }
  return robot::esp2::imuDisconnectReason(
      context.latest_imu_snapshot, micros(),
      kImuFreshnessTimeoutUs);
}

bool imuReadyForAutonomousMotion(const RuntimeContext& context) {
  return std::strcmp(imuUnavailableReason(context), "NONE") == 0;
}

void recordImuAvailabilityDiagnostics(
    RuntimeContext& context, const robot::Milliseconds now_ms) {
  const char* const reason = imuUnavailableReason(context);
  const bool disconnected = std::strcmp(reason, "NONE") != 0;
  const bool reason_changed =
      std::strcmp(context.last_imu_disconnect_reason, reason) != 0;

  if (disconnected &&
      (!context.imu_disconnect_observed ||
       !context.imu_disconnect_active || reason_changed)) {
    ++context.imu_disconnect_count;
    context.last_imu_disconnect_at_ms = now_ms;
    copyText(context.last_imu_disconnect_reason,
             sizeof(context.last_imu_disconnect_reason), reason);

    char event_message[robot::kEventMessageSize]{};
    std::snprintf(event_message, sizeof(event_message),
                  "IMU disconnected: %s", reason);
    logEvent(context, now_ms, robot::EventSeverity::Warn,
             robot::EventSource::System, event_message);

    const robot::esp2::ImuAcquisitionSnapshot& snapshot =
        context.latest_imu_snapshot;
    const std::uint32_t now_us = micros();
    const std::uint32_t sample_age_us =
        snapshot.state.last_successful_read_us == 0U
            ? 0U
            : now_us - snapshot.state.last_successful_read_us;
    const std::uint32_t snapshot_age_us =
        snapshot.published_at_us == 0U
            ? 0U
            : now_us - snapshot.published_at_us;
    Serial.printf(
        "IMU_DISCONNECT reason=%s wire=%d sample_age_us=%lu "
        "snapshot_age_us=%lu reads_ok=%lu reads_failed=%lu "
        "consecutive_failures=%lu\n",
        reason, snapshot.state.last_wire_status,
        static_cast<unsigned long>(sample_age_us),
        static_cast<unsigned long>(snapshot_age_us),
        static_cast<unsigned long>(
            snapshot.lifetime_successful_acquisitions),
        static_cast<unsigned long>(
            snapshot.lifetime_failed_acquisitions),
        static_cast<unsigned long>(
            snapshot.consecutive_acquisition_failures));
  } else if (!disconnected && context.imu_disconnect_observed &&
             context.imu_disconnect_active) {
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::System, "IMU recovered");
    Serial.printf("IMU_RECOVERED previous_reason=%s\n",
                  context.last_imu_disconnect_reason);
  }

  context.imu_disconnect_observed = true;
  context.imu_disconnect_active = disconnected;
}

AutonomousImuRecoveryUpdate serviceAutonomousImuRecovery(
    RuntimeContext& context, robot::ImuRecoveryState& recovery,
    const bool is_turn, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::LineFollowerConfig& command_config,
    const robot::Milliseconds now_ms) {
  AutonomousImuRecoveryUpdate service_update{};
  const robot::esp2::ImuState& imu_state =
      context.latest_imu_snapshot.state;
  const robot::ImuRecoveryUpdate recovery_update =
      robot::updateImuRecovery(
          recovery, kAutonomousImuRecoveryConfig,
          imuReadyForAutonomousMotion(context),
          imu_state.successful_read_count, imu_state.heading_deg,
          now_ms);
  service_update.timer_adjustment_ms =
      recovery_update.timer_adjustment_ms;

  if (is_turn) {
    robot::deferImuTurnTimers(
        context.imu_turn_state,
        recovery_update.timer_adjustment_ms);
  } else {
    robot::deferImuHeadingHoldTimer(
        context.imu_heading_hold_state,
        recovery_update.timer_adjustment_ms);
  }

  if (recovery_update.pause_started) {
    logEvent(
        context, now_ms, robot::EventSeverity::Warn,
        robot::EventSource::System,
        is_turn
            ? "IMU I2C reads lost: autonomous turn paused with wheels disabled"
            : "IMU I2C reads lost: autonomous strafe paused with wheels disabled");
  }
  if (recovery_update.decision ==
      robot::ImuRecoveryDecision::Recovered) {
    logEvent(
        context, now_ms, robot::EventSeverity::Info,
        robot::EventSource::System,
        is_turn
            ? "IMU I2C reads recovered: autonomous turn resumed"
            : "IMU I2C reads recovered: autonomous strafe resumed");
    return service_update;
  }
  if (recovery_update.decision ==
      robot::ImuRecoveryDecision::Continue) {
    return service_update;
  }

  const bool outputs_stopped =
      stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
  const bool rear_link_healthy =
      rear_link.configured() &&
      rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(command_config)) &&
      (!rear_link.statusAvailable() ||
       !rear_link.latestStatus().fault_active ||
       rear_link.latestStatus().fault_code !=
           robot::FaultCode::CommunicationStale);
  if (!outputs_stopped || !rear_link_healthy) {
    service_update.result =
        AutonomousImuRecoveryResult::StopCommandFailed;
    return service_update;
  }
  service_update.result =
      recovery_update.decision ==
              robot::ImuRecoveryDecision::TimedOut
          ? AutonomousImuRecoveryResult::TimedOut
          : AutonomousImuRecoveryResult::Paused;
  return service_update;
}

void stopAutonomousImuStrafe(RuntimeContext& context) {
  robot::cancelImuRecovery(context.imu_strafe_recovery);
  if (robot::imuHeadingHoldActive(context.imu_heading_hold_state)) {
    robot::stopImuHeadingHold(context.imu_heading_hold_state);
    context.last_imu_heading_hold_update = {};
    context.last_imu_heading_hold_update.state =
        context.imu_heading_hold_state.state;
  }
}

AutonomousImuMotionResult runAutonomousImuStrafe(
    RuntimeContext& context, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::LineFollowerConfig& command_config,
    const int lateral_direction, const float strafe_duty,
    const robot::Milliseconds now_ms) {
  robot::ImuHeadingHoldConfig active_config =
      context.imu_heading_hold_config;
  active_config.maximum_strafe_duty = strafe_duty;
  if (!imuHeadingHoldRuntimeConfigValid(active_config)) {
    return AutonomousImuMotionResult::InvalidConfiguration;
  }
  if (!imuReadyForAutonomousMotion(context)) {
    return AutonomousImuMotionResult::ImuUnavailable;
  }
  if (!front_left.configured() || !front_right.configured()) {
    return AutonomousImuMotionResult::MotorUnavailable;
  }
  if (!rear_link.configured() ||
      !rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(command_config))) {
    return AutonomousImuMotionResult::RearLinkUnavailable;
  }

  if (!robot::imuHeadingHoldActive(context.imu_heading_hold_state) ||
      context.imu_heading_hold_state.lateral_direction !=
          lateral_direction) {
    const robot::esp2::ImuState& imu_state =
        context.latest_imu_snapshot.state;
    if (!robot::startImuHeadingHold(
            context.imu_heading_hold_state, imu_state.heading_deg,
            lateral_direction, active_config,
            hardwareDutyCap(), now_ms)) {
      return AutonomousImuMotionResult::ControllerFault;
    }
    context.last_imu_heading_hold_update = {};
    context.last_imu_heading_hold_update.state =
        context.imu_heading_hold_state.state;
    context.last_imu_heading_hold_update.current_heading_deg =
        imu_state.heading_deg;
    context.last_imu_heading_hold_update.target_heading_deg =
        context.imu_heading_hold_state.target_heading_deg;
    context.last_imu_heading_hold_update.yaw_rate_dps =
        imu_state.yaw_rate_dps;
    context.last_imu_heading_hold_update.lateral_direction =
        lateral_direction;
    context.last_imu_heading_hold_update.should_strafe = true;
  }

  const robot::esp2::ImuState& imu_state =
      context.latest_imu_snapshot.state;
  context.last_imu_heading_hold_update =
      robot::updateImuHeadingHold(
          context.imu_heading_hold_state, imu_state.heading_deg,
          imu_state.yaw_rate_dps, active_config,
          hardwareDutyCap(), now_ms);
  if (context.last_imu_heading_hold_update.faulted) {
    return AutonomousImuMotionResult::ControllerFault;
  }

  const float lateral_duty =
      static_cast<float>(lateral_direction) *
      active_config.maximum_strafe_duty;
  const float signed_yaw_correction =
      context.last_imu_heading_hold_update.yaw_correction_duty *
      static_cast<float>(
          active_config.yaw_command_polarity);
  const robot::FourWheelCommand wheels = robot::mixOpenLoopMecanum(
      lateral_duty, 0.0F, signed_yaw_correction, 1.0F, now_ms,
      command_config.remoteCommandTimeoutMs);
  context.requested_command = wheels;
  if (!applyWheelCommand(context, front_left, front_right, rear_link,
                         wheels, command_config, now_ms)) {
    robot::faultImuHeadingHold(
        context.imu_heading_hold_state,
        robot::ImuHeadingHoldFaultReason::CommandFailed);
    return AutonomousImuMotionResult::CommandFailed;
  }

  context.last_command_ms = now_ms;
  context.command_deadman_armed = true;
  context.mode_expires_at_ms = 0U;
  return AutonomousImuMotionResult::Running;
}

void stopAutonomousImuTurn(RuntimeContext& context) {
  robot::cancelImuRecovery(context.imu_turn_recovery);
  if (robot::imuTurnActive(context.imu_turn_state)) {
    robot::stopImuTurn(context.imu_turn_state);
    context.last_imu_turn_update = {};
    context.last_imu_turn_update.state =
        context.imu_turn_state.state;
  }
}

AutonomousImuMotionResult runAutonomousImuTurnImpl(
    RuntimeContext& context, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::LineFollowerConfig& command_config,
    const float requested_heading_deg,
    const bool target_is_absolute,
    const robot::Milliseconds now_ms,
    const robot::Milliseconds timeout_override_ms) {
  robot::ImuTurnConfig active_config = context.imu_turn_config;
  if (timeout_override_ms > 0U) {
    active_config.timeout_ms = timeout_override_ms;
  }
  if (!imuTurnRuntimeConfigValid(active_config)) {
    return AutonomousImuMotionResult::InvalidConfiguration;
  }
  if (!imuReadyForAutonomousMotion(context)) {
    return AutonomousImuMotionResult::ImuUnavailable;
  }
  if (!front_left.configured() || !front_right.configured()) {
    return AutonomousImuMotionResult::MotorUnavailable;
  }
  if (!rear_link.configured() ||
      !rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(command_config))) {
    return AutonomousImuMotionResult::RearLinkUnavailable;
  }

  if (context.imu_turn_state.state == robot::ImuTurnState::Complete) {
    return AutonomousImuMotionResult::Complete;
  }
  if (context.imu_turn_state.state == robot::ImuTurnState::Fault) {
    return context.imu_turn_state.fault_reason ==
                   robot::ImuTurnFaultReason::Timeout
               ? AutonomousImuMotionResult::Timeout
               : AutonomousImuMotionResult::ControllerFault;
  }
  if (!robot::imuTurnActive(context.imu_turn_state)) {
    const robot::esp2::ImuState& imu_state =
        context.latest_imu_snapshot.state;
    const bool started = target_is_absolute
                             ? robot::startImuTurnToHeading(
                                   context.imu_turn_state,
                                   imu_state.heading_deg,
                                   requested_heading_deg, active_config,
                                   hardwareDutyCap(), now_ms)
                             : robot::startImuTurn(
                                   context.imu_turn_state,
                                   imu_state.heading_deg,
                                   requested_heading_deg, active_config,
                                   hardwareDutyCap(), now_ms);
    if (!started) {
      return AutonomousImuMotionResult::ControllerFault;
    }
    context.last_imu_turn_update = {};
    context.last_imu_turn_update.state =
        context.imu_turn_state.state;
    context.last_imu_turn_update.current_heading_deg =
        imu_state.heading_deg;
    context.last_imu_turn_update.target_heading_deg =
        context.imu_turn_state.target_heading_deg;
    context.last_imu_turn_update.angle_error_deg =
        context.imu_turn_state.target_heading_deg -
        imu_state.heading_deg;
    context.last_imu_turn_update.yaw_rate_dps =
        imu_state.yaw_rate_dps;
  }

  const robot::esp2::ImuState& imu_state =
      context.latest_imu_snapshot.state;
  context.last_imu_turn_update = robot::updateImuTurn(
      context.imu_turn_state, imu_state.heading_deg,
      imu_state.yaw_rate_dps, active_config,
      hardwareDutyCap(), now_ms);
  if (context.last_imu_turn_update.faulted) {
    return context.imu_turn_state.fault_reason ==
                   robot::ImuTurnFaultReason::Timeout
               ? AutonomousImuMotionResult::Timeout
               : AutonomousImuMotionResult::ControllerFault;
  }
  if (context.last_imu_turn_update.completed) {
    const robot::FourWheelCommand stopped =
        robot::disabledFourWheelCommand();
    context.requested_command = stopped;
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           stopped, command_config, now_ms)) {
      robot::faultImuTurn(context.imu_turn_state,
                          robot::ImuTurnFaultReason::CommandFailed);
      return AutonomousImuMotionResult::CommandFailed;
    }
    return AutonomousImuMotionResult::Complete;
  }
  if (!context.last_imu_turn_update.should_rotate) {
    const robot::FourWheelCommand stopped =
        robot::disabledFourWheelCommand();
    context.requested_command = stopped;
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           stopped, command_config, now_ms)) {
      robot::faultImuTurn(context.imu_turn_state,
                          robot::ImuTurnFaultReason::CommandFailed);
      return AutonomousImuMotionResult::CommandFailed;
    }
    return AutonomousImuMotionResult::Running;
  }

  const robot::FourWheelCommand wheels = makeImuTurnCommand(
      context, context.last_imu_turn_update.rotation_command, now_ms);
  context.requested_command = wheels;
  if (!applyWheelCommand(context, front_left, front_right, rear_link,
                         wheels, command_config, now_ms)) {
    robot::faultImuTurn(context.imu_turn_state,
                        robot::ImuTurnFaultReason::CommandFailed);
    return AutonomousImuMotionResult::CommandFailed;
  }

  context.last_command_ms = now_ms;
  context.command_deadman_armed = true;
  context.mode_expires_at_ms = 0U;
  return AutonomousImuMotionResult::Running;
}

AutonomousImuMotionResult runAutonomousImuTurn(
    RuntimeContext& context, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::LineFollowerConfig& command_config,
    const float relative_angle_deg, const robot::Milliseconds now_ms,
    const robot::Milliseconds timeout_override_ms = 0U) {
  return runAutonomousImuTurnImpl(
      context, front_left, front_right, rear_link, command_config,
      relative_angle_deg, false, now_ms, timeout_override_ms);
}

AutonomousImuMotionResult runAutonomousImuTurnToHeading(
    RuntimeContext& context, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::LineFollowerConfig& command_config,
    const float target_heading_deg, const robot::Milliseconds now_ms,
    const robot::Milliseconds timeout_override_ms) {
  return runAutonomousImuTurnImpl(
      context, front_left, front_right, rear_link, command_config,
      target_heading_deg, true, now_ms, timeout_override_ms);
}

void enterImuTurnFault(RuntimeContext& context,
                       DualPwmMotorOutput& front_left,
                       DualPwmMotorOutput& front_right,
                       RearCommandLink& rear_link,
                       const robot::ImuTurnFaultReason reason,
                       const robot::FaultCode fault_code,
                       const char* message,
                       const robot::EventSource source,
                       const robot::Milliseconds now_ms) {
  if (context.imu_turn_state.state == robot::ImuTurnState::Fault) {
    return;
  }
  recordMotionDiagnostic(
      context, front_left, front_right, rear_link,
      &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::ImuTurnFaulted, now_ms);
  robot::faultImuTurn(context.imu_turn_state, reason);
  context.last_imu_turn_update = {};
  context.last_imu_turn_update.state = context.imu_turn_state.state;
  context.last_imu_turn_update.fault_reason =
      context.imu_turn_state.fault_reason;
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  recordMotionDiagnostic(
      context, front_left, front_right, rear_link,
      &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::OutputsDisabled, now_ms);
  scheduleMotionDiagnosticFreeze(
      context, robot::MotionDiagnosticTrigger::ImuTurnFaulted);
  setFault(context, fault_code, message);
  logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
}

void captureImuTurnAvailabilityFault(
    RuntimeContext& context,
    const robot::esp2::ImuAcquisitionSnapshot& imu,
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const std::uint32_t now_us, const char* const origin,
    const char* const reason) {
  ImuTurnAvailabilityFaultCapture& capture =
      context.imu_turn_availability_fault;
  capture = {};
  capture.valid = true;
  capture.configured = imu.state.configured;
  capture.initialized = imu.state.initialized;
  capture.calibrated = imu.state.calibrated;
  capture.healthy = imu.state.healthy;
  capture.sample_valid = imu.state.last_successful_read_us != 0U;
  capture.data_fresh = robot::esp2::imuSnapshotFresh(
      imu, now_us, kImuFreshnessTimeoutUs);
  capture.acquisition_running = imu.acquisition_running;
  capture.shared_snapshot_available =
      context.imu_shared_snapshot_available;
  capture.front_left_configured = front_left.configured();
  capture.front_right_configured = front_right.configured();
  capture.rear_link_configured = rear_link.configured();
  capture.rear_status_available = rear_link.statusAvailable();
  capture.rear_status_fresh = rear_link.remoteStatusFresh(
      now_ms, remoteStatusTimeoutMs(context.config));
  copyText(capture.origin, sizeof(capture.origin), origin);
  copyText(capture.reason, sizeof(capture.reason), reason);
  capture.evaluated_at_us = now_us;
  capture.evaluated_at_ms = now_ms;
  capture.published_at_us = imu.published_at_us;
  capture.last_successful_read_us =
      imu.state.last_successful_read_us;
  capture.sample_age_us =
      capture.sample_valid
          ? now_us - imu.state.last_successful_read_us
          : 0U;
  capture.snapshot_age_us =
      imu.published_at_us == 0U ? 0U
                               : now_us - imu.published_at_us;
  capture.freshness_timeout_us = kImuFreshnessTimeoutUs;
  capture.cached_snapshot_sequence = imu.publication_sequence;
  capture.cached_successful_sample_sequence =
      imu.successful_sample_sequence;
  capture.cached_snapshot_fetched_at_us =
      context.imu_shared_snapshot_fetched_at_us;
  capture.cached_snapshot_fetch_to_gate_us =
      context.imu_shared_snapshot_fetched_at_us == 0U
          ? 0U
          : now_us -
                context.imu_shared_snapshot_fetched_at_us;

  robot::esp2::ImuAcquisitionSnapshot newest_snapshot{};
  capture.newest_snapshot_available =
      g_runtime.imu_acquisition != nullptr &&
      g_runtime.imu_acquisition->latest(newest_snapshot);
  capture.newest_snapshot_sequence =
      capture.newest_snapshot_available
          ? newest_snapshot.publication_sequence
          : 0U;
  capture.newest_successful_sample_sequence =
      capture.newest_snapshot_available
          ? newest_snapshot.successful_sample_sequence
          : 0U;
  capture.cached_snapshot_matches_newest =
      capture.newest_snapshot_available &&
      capture.cached_snapshot_sequence ==
          capture.newest_snapshot_sequence;
  const robot::esp2::ImuAcquisitionSnapshot& timing_snapshot =
      capture.newest_snapshot_available ? newest_snapshot : imu;
  capture.successful_sample_publication_gap_us =
      timing_snapshot.successful_sample_publication_gap_us;
  capture.maximum_successful_sample_publication_gap_us =
      timing_snapshot
          .maximum_successful_sample_publication_gap_us;
  capture.current_observed_publication_gap_us =
      imu.last_successful_sample_published_at_us == 0U
          ? 0U
          : now_us -
                imu.last_successful_sample_published_at_us;
  if (capture.current_observed_publication_gap_us >
      context.maximum_observed_imu_publication_gap_us) {
    context.maximum_observed_imu_publication_gap_us =
        capture.current_observed_publication_gap_us;
  }
  capture.maximum_observed_publication_gap_us =
      context.maximum_observed_imu_publication_gap_us;
  capture.rear_last_status_received_at_ms =
      rear_link.lastStatusReceivedAtMs();
  capture.rear_status_age_ms =
      capture.rear_status_available
          ? elapsedSince(now_ms,
                         capture.rear_last_status_received_at_ms)
          : 0U;

  Serial.printf(
      "IMU_TURN_AVAILABILITY_FAULT origin=%s reason=%s "
      "configured=%u initialized=%u calibrated=%u healthy=%u "
      "sample_valid=%u data_fresh=%u acquisition_running=%u "
      "shared_snapshot_available=%u evaluated_at_us=%lu "
      "published_at_us=%lu last_successful_read_us=%lu "
      "sample_age_us=%lu snapshot_age_us=%lu freshness_timeout_us=%lu "
      "front_left_configured=%u front_right_configured=%u "
      "rear_link_configured=%u rear_status_available=%u "
      "rear_status_fresh=%u evaluated_at_ms=%lu "
      "rear_last_status_received_at_ms=%lu rear_status_age_ms=%lu "
      "acq_loop_us=%lu acq_loop_max_us=%lu sync_us=%lu "
      "sync_max_us=%lu i2c_read_us=%lu i2c_read_max_us=%lu "
      "wire_lock_us=%lu wire_lock_max_us=%lu "
      "read_to_publish_us=%lu read_to_publish_max_us=%lu "
      "publish_queue_us=%lu publish_queue_max_us=%lu "
      "publication_gap_us=%lu publication_gap_max_us=%lu "
      "observed_open_publication_gap_us=%lu "
      "observed_publication_gap_max_us=%lu "
      "cached_seq=%lu newest_available=%u newest_seq=%lu "
      "cached_matches_newest=%u cached_fetched_at_us=%lu "
      "cached_fetch_to_gate_us=%lu\n",
      capture.origin, capture.reason,
      static_cast<unsigned>(capture.configured),
      static_cast<unsigned>(capture.initialized),
      static_cast<unsigned>(capture.calibrated),
      static_cast<unsigned>(capture.healthy),
      static_cast<unsigned>(capture.sample_valid),
      static_cast<unsigned>(capture.data_fresh),
      static_cast<unsigned>(capture.acquisition_running),
      static_cast<unsigned>(capture.shared_snapshot_available),
      static_cast<unsigned long>(capture.evaluated_at_us),
      static_cast<unsigned long>(capture.published_at_us),
      static_cast<unsigned long>(capture.last_successful_read_us),
      static_cast<unsigned long>(capture.sample_age_us),
      static_cast<unsigned long>(capture.snapshot_age_us),
      static_cast<unsigned long>(capture.freshness_timeout_us),
      static_cast<unsigned>(capture.front_left_configured),
      static_cast<unsigned>(capture.front_right_configured),
      static_cast<unsigned>(capture.rear_link_configured),
      static_cast<unsigned>(capture.rear_status_available),
      static_cast<unsigned>(capture.rear_status_fresh),
      static_cast<unsigned long>(capture.evaluated_at_ms),
      static_cast<unsigned long>(
          capture.rear_last_status_received_at_ms),
      static_cast<unsigned long>(capture.rear_status_age_ms),
      static_cast<unsigned long>(imu.acquisition_loop_interval_us),
      static_cast<unsigned long>(
          imu.maximum_acquisition_loop_interval_us),
      static_cast<unsigned long>(imu.synchronization_duration_us),
      static_cast<unsigned long>(
          imu.maximum_synchronization_duration_us),
      static_cast<unsigned long>(
          imu.state.last_measurement_read_duration_us),
      static_cast<unsigned long>(
          imu.state.maximum_measurement_read_duration_us),
      static_cast<unsigned long>(
          imu.state.last_wire_lock_acquire_duration_us),
      static_cast<unsigned long>(
          imu.state.maximum_wire_lock_acquire_duration_us),
      static_cast<unsigned long>(
          imu.successful_read_to_publication_us),
      static_cast<unsigned long>(
          imu.maximum_successful_read_to_publication_us),
      static_cast<unsigned long>(
          imu.publication_queue_duration_us),
      static_cast<unsigned long>(
          imu.maximum_publication_queue_duration_us),
      static_cast<unsigned long>(
          capture.successful_sample_publication_gap_us),
      static_cast<unsigned long>(
          capture.maximum_successful_sample_publication_gap_us),
      static_cast<unsigned long>(
          capture.current_observed_publication_gap_us),
      static_cast<unsigned long>(
          capture.maximum_observed_publication_gap_us),
      static_cast<unsigned long>(
          capture.cached_snapshot_sequence),
      static_cast<unsigned>(capture.newest_snapshot_available),
      static_cast<unsigned long>(
          capture.newest_snapshot_sequence),
      static_cast<unsigned>(
          capture.cached_snapshot_matches_newest),
      static_cast<unsigned long>(
          capture.cached_snapshot_fetched_at_us),
      static_cast<unsigned long>(
          capture.cached_snapshot_fetch_to_gate_us));
}

void runImuTurnTest(RuntimeContext& context,
                    DualPwmMotorOutput& front_left,
                    DualPwmMotorOutput& front_right,
                    RearCommandLink& rear_link,
                    robot::esp2::ImuAcquisitionService& imu_acquisition,
                    const robot::Milliseconds now_ms) {
  if (!robot::imuTurnActive(context.imu_turn_state)) {
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    return;
  }

  if (!imuTurnRuntimeConfigValid(context.imu_turn_config)) {
    enterImuTurnFault(
        context, front_left, front_right, rear_link,
        robot::ImuTurnFaultReason::InvalidConfiguration,
        robot::FaultCode::InvalidCommand,
        "IMU turn stopped: invalid configuration",
        robot::EventSource::System, now_ms);
    return;
  }

  context.imu_shared_snapshot_available =
      imu_acquisition.latest(context.latest_imu_snapshot);
  context.imu_shared_snapshot_fetched_at_us = micros();
  const robot::esp2::ImuAcquisitionSnapshot& imu =
      context.latest_imu_snapshot;
  const robot::esp2::ImuState& imu_state = imu.state;
  const std::uint32_t imu_availability_now_us = micros();
  const char* const imu_availability_reason =
      robot::esp2::imuDisconnectReason(
          imu, imu_availability_now_us, kImuFreshnessTimeoutUs);
  if (std::strcmp(imu_availability_reason, "NONE") != 0) {
    captureImuTurnAvailabilityFault(
        context, imu, front_left, front_right, rear_link, now_ms,
        imu_availability_now_us, "ACTIVE_TURN_RUNTIME_GATE",
        imu_availability_reason);
    enterImuTurnFault(context, front_left, front_right, rear_link,
                      robot::ImuTurnFaultReason::ImuUnavailable,
                      robot::FaultCode::HardwareNotConfigured,
                      "IMU turn stopped: IMU unavailable",
                      robot::EventSource::System, now_ms);
    return;
  }

  if (!front_left.configured() || !front_right.configured()) {
    enterImuTurnFault(context, front_left, front_right, rear_link,
                      robot::ImuTurnFaultReason::CommandFailed,
                      robot::FaultCode::HardwareNotConfigured,
                      "IMU turn stopped: front motors unavailable",
                      robot::EventSource::Motor, now_ms);
    return;
  }
  if (!rear_link.configured() ||
      !rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(context.config))) {
    enterImuTurnFault(context, front_left, front_right, rear_link,
                      robot::ImuTurnFaultReason::RearLinkUnavailable,
                      robot::FaultCode::CommunicationStale,
                      "IMU turn stopped: rear link unavailable",
                      robot::EventSource::Uart, now_ms);
    return;
  }

  context.last_imu_turn_update = robot::updateImuTurn(
      context.imu_turn_state, imu_state.heading_deg,
      imu_state.yaw_rate_dps, context.imu_turn_config,
      hardwareDutyCap(), now_ms);
  if (context.last_imu_turn_update.faulted) {
    const bool timed_out =
        context.imu_turn_state.fault_reason ==
        robot::ImuTurnFaultReason::Timeout;
    recordMotionDiagnostic(
        context, front_left, front_right, rear_link, &imu,
        timed_out ? robot::MotionDiagnosticEvent::ImuTurnTimedOut
                  : robot::MotionDiagnosticEvent::ImuTurnFaulted,
        now_ms);
    // updateImuTurn already put the controller in Fault, so stop explicitly.
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    recordMotionDiagnostic(
        context, front_left, front_right, rear_link, &imu,
        robot::MotionDiagnosticEvent::OutputsDisabled, now_ms);
    scheduleMotionDiagnosticFreeze(
        context, timed_out
                     ? robot::MotionDiagnosticTrigger::ImuTurnTimedOut
                     : robot::MotionDiagnosticTrigger::ImuTurnFaulted);
    setFault(context,
             timed_out ? robot::FaultCode::SearchTimeout
                       : robot::FaultCode::InvalidCommand,
             timed_out ? "IMU turn stopped: overall timeout"
                       : "IMU turn stopped: controller fault");
    logEvent(context, now_ms, robot::EventSeverity::Fault,
             robot::EventSource::System,
             timed_out ? "IMU turn stopped: overall timeout"
                       : "IMU turn stopped: controller fault");
    return;
  }

  if (context.last_imu_turn_update.completed) {
    recordMotionDiagnostic(
        context, front_left, front_right, rear_link, &imu,
        robot::MotionDiagnosticEvent::ImuTurnCompleted, now_ms);
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    recordMotionDiagnostic(
        context, front_left, front_right, rear_link, &imu,
        robot::MotionDiagnosticEvent::OutputsDisabled, now_ms);
    scheduleMotionDiagnosticFreeze(
        context, robot::MotionDiagnosticTrigger::ImuTurnCompleted);
    clearFault(context);
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::System, "IMU turn complete");
    return;
  }

  if (!context.last_imu_turn_update.should_rotate) {
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    return;
  }

  const float signed_yaw_command =
      context.last_imu_turn_update.rotation_command *
      static_cast<float>(context.imu_turn_config.yaw_command_polarity);
  const robot::FourWheelCommand wheels = robot::mixOpenLoopMecanum(
      0.0F, 0.0F, signed_yaw_command < 0.0F ? -1.0F : 1.0F,
      std::fabs(signed_yaw_command), now_ms,
      context.config.remoteCommandTimeoutMs);
  context.requested_command = wheels;
  if (!applyWheelCommand(context, front_left, front_right, rear_link,
                         wheels, context.config, now_ms)) {
    enterImuTurnFault(context, front_left, front_right, rear_link,
                      robot::ImuTurnFaultReason::CommandFailed,
                      robot::FaultCode::CommunicationStale,
                      "IMU turn stopped: rear command failed",
                      robot::EventSource::Uart, now_ms);
    return;
  }
  context.last_command_ms = now_ms;
  context.command_deadman_armed = false;
  context.mode_expires_at_ms = 0U;
}

void enterImuHeadingHoldFault(
    RuntimeContext& context, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::ImuHeadingHoldFaultReason reason,
    const robot::FaultCode fault_code, const char* message,
    const robot::EventSource source,
    const robot::Milliseconds now_ms) {
  const bool already_faulted =
      context.imu_heading_hold_state.state ==
      robot::ImuHeadingHoldState::Fault;
  robot::faultImuHeadingHold(context.imu_heading_hold_state, reason);
  context.last_imu_heading_hold_update = {};
  context.last_imu_heading_hold_update.state =
      context.imu_heading_hold_state.state;
  context.last_imu_heading_hold_update.fault_reason =
      context.imu_heading_hold_state.fault_reason;
  disableMotionActuators(context, front_left, front_right, rear_link,
                         now_ms);
  setFault(context, fault_code, message);
  if (!already_faulted) {
    logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
  }
}

void runImuHeadingHoldTest(
    RuntimeContext& context, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    robot::esp2::ImuAcquisitionService& imu_acquisition,
    const robot::Milliseconds now_ms) {
  if (!robot::imuHeadingHoldActive(context.imu_heading_hold_state)) {
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    return;
  }

  if (!imuHeadingHoldRuntimeConfigValid(
          context.imu_heading_hold_config)) {
    enterImuHeadingHoldFault(
        context, front_left, front_right, rear_link,
        robot::ImuHeadingHoldFaultReason::InvalidConfiguration,
        robot::FaultCode::InvalidCommand,
        "IMU strafe stopped: invalid configuration",
        robot::EventSource::System, now_ms);
    return;
  }

  context.imu_shared_snapshot_available =
      imu_acquisition.latest(context.latest_imu_snapshot);
  context.imu_shared_snapshot_fetched_at_us = micros();
  const robot::esp2::ImuAcquisitionSnapshot& imu =
      context.latest_imu_snapshot;
  const robot::esp2::ImuState& imu_state = imu.state;
  if (std::strcmp(robot::esp2::imuDisconnectReason(
                      imu, micros(), kImuFreshnessTimeoutUs),
                  "NONE") != 0) {
    enterImuHeadingHoldFault(
        context, front_left, front_right, rear_link,
        robot::ImuHeadingHoldFaultReason::ImuUnavailable,
        robot::FaultCode::HardwareNotConfigured,
        "IMU strafe stopped: IMU unavailable",
        robot::EventSource::System, now_ms);
    return;
  }
  if (!front_left.configured() || !front_right.configured()) {
    enterImuHeadingHoldFault(
        context, front_left, front_right, rear_link,
        robot::ImuHeadingHoldFaultReason::CommandFailed,
        robot::FaultCode::HardwareNotConfigured,
        "IMU strafe stopped: front motors unavailable",
        robot::EventSource::Motor, now_ms);
    return;
  }
  if (!rear_link.configured() ||
      !rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(context.config))) {
    enterImuHeadingHoldFault(
        context, front_left, front_right, rear_link,
        robot::ImuHeadingHoldFaultReason::RearLinkUnavailable,
        robot::FaultCode::CommunicationStale,
        "IMU strafe stopped: rear link unavailable",
        robot::EventSource::Uart, now_ms);
    return;
  }

  context.last_imu_heading_hold_update =
      robot::updateImuHeadingHold(
          context.imu_heading_hold_state, imu_state.heading_deg,
          imu_state.yaw_rate_dps, context.imu_heading_hold_config,
          hardwareDutyCap(), now_ms);
  if (context.last_imu_heading_hold_update.faulted) {
    enterImuHeadingHoldFault(
        context, front_left, front_right, rear_link,
        context.imu_heading_hold_state.fault_reason,
        robot::FaultCode::InvalidCommand,
        "IMU strafe stopped: controller fault",
        robot::EventSource::System, now_ms);
    return;
  }

  const float lateral_duty =
      static_cast<float>(
          context.imu_heading_hold_state.lateral_direction) *
      context.imu_heading_hold_config.maximum_strafe_duty;
  const float signed_yaw_correction =
      context.last_imu_heading_hold_update.yaw_correction_duty *
      static_cast<float>(
          context.imu_heading_hold_config.yaw_command_polarity);
  const robot::FourWheelCommand wheels = robot::mixOpenLoopMecanum(
      lateral_duty, 0.0F, signed_yaw_correction, 1.0F, now_ms,
      context.config.remoteCommandTimeoutMs);
  context.requested_command = wheels;
  if (!applyWheelCommand(context, front_left, front_right, rear_link,
                         wheels, context.config, now_ms)) {
    enterImuHeadingHoldFault(
        context, front_left, front_right, rear_link,
        robot::ImuHeadingHoldFaultReason::CommandFailed,
        robot::FaultCode::CommunicationStale,
        "IMU strafe stopped: rear command failed",
        robot::EventSource::Uart, now_ms);
  }
}

void printTelemetry(RuntimeContext& context,
                    const DigitalFrontLineSensorReader& sensors,
                    const RearCommandLink& rear_link,
                    robot::Milliseconds now_ms);

void printRearTelemetry(RuntimeContext& context,
                        const RearCommandLink& rear_link,
                        robot::Milliseconds now_ms);

void requestSolarPanelAutonomyStart(RuntimeContext& context,
                                    robot::IMotorOutput& front_left,
                                    robot::IMotorOutput& front_right,
                                    RearCommandLink& rear_link,
                                    const robot::Milliseconds now_ms,
                                    const robot::EventSource source) {
  disableActuators(context, front_left, front_right, rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::AutonomousSolarPanel, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  context.solar_start_requested = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info, source,
           "solar autonomy start requested");
}

void requestHabitatPiecesStart(RuntimeContext& context,
                               robot::IMotorOutput& front_left,
                               robot::IMotorOutput& front_right,
                               RearCommandLink& rear_link,
                               const robot::Milliseconds now_ms,
                               const robot::EventSource source) {
  disableActuators(context, front_left, front_right, rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::HabitatPieces, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  context.habitat_pieces_start_requested = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info, source,
           "Habitat Pieces start requested");
}

void requestHabitatPlacementStart(
    RuntimeContext& context, robot::IMotorOutput& front_left,
    robot::IMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const robot::EventSource source) {
  (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                   rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::HabitatPlacement, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  context.habitat_placement_start_requested = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info, source,
           "Habitat Placement start requested");
}

void requestTowerPiecesStart(RuntimeContext& context,
                             robot::IMotorOutput& front_left,
                             robot::IMotorOutput& front_right,
                             RearCommandLink& rear_link,
                             const robot::Milliseconds now_ms,
                             const robot::EventSource source) {
  (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                   rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::AutonomousTowerPieces, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  context.tower_pieces_start_requested = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info, source,
           "tower pieces start requested");
}

void requestPegFinderStart(RuntimeContext& context,
                           robot::IMotorOutput& front_left,
                           robot::IMotorOutput& front_right,
                           RearCommandLink& rear_link,
                           const robot::Milliseconds now_ms,
                           const robot::EventSource source) {
  (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                   rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::PegFinder, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  context.peg_finder_start_requested = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info, source,
           "PegFinder start requested");
}

void requestTimeTrialStart(RuntimeContext& context,
                           robot::IMotorOutput& front_left,
                           robot::IMotorOutput& front_right,
                           RearCommandLink& rear_link,
                           const robot::Milliseconds now_ms,
                           const robot::EventSource source) {
  disableActuators(context, front_left, front_right, rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::TimeTrial, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  context.time_trial_start_requested = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info, source,
           "Time Trial start requested");
}

void enterHabitatPiecesFault(
    RuntimeContext& context, robot::IMotorOutput& front_left,
    robot::IMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::HabitatPiecesStopReason stop_reason,
    const robot::FaultCode fault_code, const char* const message,
    const robot::EventSource source,
    const robot::Milliseconds now_ms) {
  if (g_runtime.stepper != nullptr) {
    g_runtime.stepper->stop();
  }
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  robot::failHabitatPiecesAutonomy(context.habitat_pieces, stop_reason,
                                   now_ms);
  context.last_habitat_pieces_update.state =
      robot::HabitatPiecesState::Fault;
  context.last_habitat_pieces_update.stop_reason = stop_reason;
  context.last_habitat_pieces_update.should_stop = true;
  context.last_habitat_pieces_update.should_line_follow = false;
  context.last_habitat_pieces_update.should_align_side_lines = false;
  context.last_habitat_pieces_update.should_drive_left_side = false;
  context.last_habitat_pieces_update.should_drive_right_side = false;
  context.last_habitat_pieces_update.should_reverse = false;
  context.last_habitat_pieces_update.should_distance_strafe = false;
  context.last_habitat_pieces_update.should_compensation_strafe = false;
  context.last_habitat_pieces_update.should_start_slide_down = false;
  context.last_habitat_pieces_update.should_lower_slide = false;
  context.last_habitat_pieces_update.should_drive_forward_to_distance = false;
  context.last_habitat_pieces_update.should_start_slide_lift = false;
  context.last_habitat_pieces_update.should_post_pickup_reverse = false;
  context.last_habitat_pieces_update.should_return_line_strafe = false;
  context.last_habitat_pieces_update.should_wait_for_slide_lift = false;
  setFault(context, fault_code, message);
  logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
}

void runHabitatPieces(
    RuntimeContext& context, DigitalFrontLineSensorReader& sensors,
    DualPwmMotorOutput& front_left, DualPwmMotorOutput& front_right,
    RearCommandLink& rear_link, robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms) {
  if (context.habitat_pieces.state ==
      robot::HabitatPiecesState::WaitForStart) {
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    if (!context.habitat_pieces_start_requested) {
      return;
    }
    context.habitat_pieces_start_requested = false;
    if (!rear_link.rearLineSnapshotFresh(
            now_ms, context.config.remoteCommandTimeoutMs)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::SideSensorDataStale,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces start rejected: LSS2/LSS3 sensor data stale",
          robot::EventSource::Uart, now_ms);
      return;
    }
    if (!rear_link.latestRearLineSnapshot().side_2_configured ||
        !rear_link.latestRearLineSnapshot().side_3_configured) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::SideSensorsUnavailable,
          robot::FaultCode::HardwareNotConfigured,
          "Habitat Pieces start rejected: LSS2/LSS3 GPIO is not configured",
          robot::EventSource::Line, now_ms);
      return;
    }
    if (stepper.lowerLimitActive() && stepper.upperLimitActive()) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::ConflictingSlideLimits,
          robot::FaultCode::LimitSwitchConflict,
          "Habitat Pieces start rejected: slide limit switches conflict",
          robot::EventSource::Motor, now_ms);
      return;
    }
    if (!habitatPiecesStartRequirementsMet(
            sensors, front_left, front_right, rear_link, stepper, now_ms,
            context)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::ConfigurationIncomplete,
          robot::FaultCode::HardwareNotConfigured,
          "Habitat Pieces start rejected: config or hardware incomplete",
          robot::EventSource::System, now_ms);
      return;
    }
    robot::startHabitatPiecesAutonomy(context.habitat_pieces, now_ms);
    robot::startLineFollower(context.follower_state, now_ms);
    context.last_update = {};
    clearFault(context);
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::System,
             "Habitat Pieces front line follow started; LSS2/LSS3 detection delay active");
  }

  if (context.habitat_pieces.state ==
          robot::HabitatPiecesState::Complete ||
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::Fault) {
    stepper.stop();
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    return;
  }

  if (stepper.lowerLimitActive() && stepper.upperLimitActive()) {
    enterHabitatPiecesFault(
        context, front_left, front_right, rear_link,
        robot::HabitatPiecesStopReason::ConflictingSlideLimits,
        robot::FaultCode::LimitSwitchConflict,
        "Habitat Pieces stopped: slide limit switches conflict",
        robot::EventSource::Motor, now_ms);
    return;
  }

  if (!habitatPiecesMotionRequirementsMet(
          sensors, front_left, front_right, rear_link, stepper, now_ms,
          context)) {
    enterHabitatPiecesFault(
        context, front_left, front_right, rear_link,
        robot::HabitatPiecesStopReason::ConfigurationIncomplete,
        robot::FaultCode::CommunicationStale,
        "Habitat Pieces stopped: motion hardware or rear link unavailable",
        robot::EventSource::Uart, now_ms);
    return;
  }

  bool lss2_black = false;
  bool lss3_black = false;
  if (context.habitat_pieces.state ==
          robot::HabitatPiecesState::LineFollowing ||
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::SideLineAligning) {
    if (!rear_link.rearLineSnapshotFresh(
            now_ms, context.config.remoteCommandTimeoutMs)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::SideSensorDataStale,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: LSS2/LSS3 sensor data stale",
          robot::EventSource::Uart, now_ms);
      return;
    }
    const robot::RearLineSensorSnapshot& line_sensors =
        rear_link.latestRearLineSnapshot();
    if (!line_sensors.side_2_configured ||
        !line_sensors.side_3_configured) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::SideSensorsUnavailable,
          robot::FaultCode::HardwareNotConfigured,
          "Habitat Pieces stopped: LSS2/LSS3 is not configured",
          robot::EventSource::Line, now_ms);
      return;
    }
    lss2_black = line_sensors.side_2_electrical_high;
    lss3_black = line_sensors.side_3_electrical_high;
  }

  robot::HabitatPiecesDistanceSample distance_sample{};
  if (rear_link.laserSnapshotAvailable()) {
    const robot::LaserDistanceSnapshot& laser =
        rear_link.latestLaserSnapshot();
    const robot::Milliseconds measurement_received_at_ms =
        rear_link.lastLaserMeasurementReceivedAtMs();
    const robot::Milliseconds maximum_sample_age_ms =
        static_cast<robot::Milliseconds>(
            laser.intermeasurement_period_ms) *
        4U;
    distance_sample.available =
        laser.configured && laser.initialized && laser.ranging &&
        laser.data_valid &&
        laser.profile == robot::LaserDistanceProfile::HighAccuracy &&
        laser.sensor_range_status == robot::kVl53l0xValidRangeStatus &&
        laser.driver_status == 0 && measurement_received_at_ms != 0U &&
        maximum_sample_age_ms > 0U &&
        elapsedSince(now_ms, measurement_received_at_ms) <=
            maximum_sample_age_ms;
    distance_sample.distance_mm = laser.distance_mm;
    distance_sample.measurement_sequence =
        laser.measurement_sequence;
  }

  const robot::HabitatPiecesState previous_habitat_state =
      context.habitat_pieces.state;
  if (previous_habitat_state ==
      robot::HabitatPiecesState::ReturnLineStrafing) {
    if (!rear_link.rearLineSnapshotFresh(
            now_ms, context.rear_config.remoteCommandTimeoutMs)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearLineDataStale,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: rear line data stale during return strafe",
          robot::EventSource::Uart, now_ms);
      return;
    }
    if (!rear_link.latestRearLineSnapshot().configured) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearLineSensorsUnavailable,
          robot::FaultCode::HardwareNotConfigured,
          "Habitat Pieces stopped: rear line sensors unavailable",
          robot::EventSource::Line, now_ms);
      return;
    }
  }

  robot::HabitatPiecesInputs habitat_inputs{};
  habitat_inputs.lss2_black = lss2_black;
  habitat_inputs.lss3_black = lss3_black;
  habitat_inputs.distance_sample = distance_sample;
  habitat_inputs.slide_bottom_limit_active =
      stepper.lowerLimitActive();
  habitat_inputs.slide_bottom_ready =
      stepper.lowerLimitActive() && stepper.isHomed();
  habitat_inputs.slide_top_limit_active = stepper.upperLimitActive();
  habitat_inputs.slide_down_failed =
      previous_habitat_state == robot::HabitatPiecesState::LoweringSlide &&
      !habitat_inputs.slide_bottom_ready &&
      (stepper.motionState() ==
           robot::esp2::StepperMotionState::LimitSearchFailed ||
       stepper.motionState() == robot::esp2::StepperMotionState::Stopped);
  if (context.habitat_pieces.slide_lift_started) {
    habitat_inputs.slide_lift_complete =
        stepper.positionSteps() >=
            static_cast<std::int64_t>(
                context.habitat_pieces_config.slide_lift_steps) &&
        !stepper.isBusy();
    habitat_inputs.slide_lift_failed =
        !habitat_inputs.slide_lift_complete &&
        ((stepper.upperLimitActive() &&
          stepper.positionSteps() <
              static_cast<std::int64_t>(
                  context.habitat_pieces_config.slide_lift_steps)) ||
         !stepper.isBusy());
  }
  if (rear_link.rearLineSnapshotFresh(
          now_ms, context.rear_config.remoteCommandTimeoutMs) &&
      rear_link.latestRearLineSnapshot().configured) {
    const robot::RearLineSensorSnapshot& rear_line =
        rear_link.latestRearLineSnapshot();
    habitat_inputs.rear_left_black = rear_line.left_electrical_high;
    habitat_inputs.rear_right_black = rear_line.right_electrical_high;
  }

  context.last_habitat_pieces_update =
      robot::updateHabitatPiecesAutonomy(
          context.habitat_pieces, context.habitat_pieces_config,
          habitat_inputs, now_ms);

  if (context.last_habitat_pieces_update.should_start_slide_down) {
    const bool accepted = stepper.setLimitSearchSpeed(
                              context.habitat_pieces_config
                                  .slide_down_speed_steps_per_second) &&
                          stepper.moveToLowerLimit();
    if (!accepted) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::SlideCommandFailed,
          robot::FaultCode::InvalidCommand,
          "Habitat Pieces stopped: slide-down command rejected",
          robot::EventSource::Motor, now_ms);
      return;
    }
  }

  if (context.last_habitat_pieces_update.should_start_slide_lift) {
    const bool accepted = stepper.isHomed() &&
                          stepper.setSpeed(
                              context.habitat_pieces_config
                                  .slide_lift_speed_steps_per_second) &&
                          stepper.jogSteps(static_cast<std::int64_t>(
                              context.habitat_pieces_config
                                  .slide_lift_steps));
    if (!accepted) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::SlideCommandFailed,
          robot::FaultCode::InvalidCommand,
          "Habitat Pieces stopped: slide-lift command rejected",
          robot::EventSource::Motor, now_ms);
      return;
    }
  }

  if (context.habitat_pieces.state == robot::HabitatPiecesState::Fault) {
    const robot::HabitatPiecesStopReason reason =
        context.habitat_pieces.stop_reason;
    robot::FaultCode fault_code = robot::FaultCode::InvalidCommand;
    const char* message = "Habitat Pieces stopped: autonomous state fault";
    robot::EventSource source = robot::EventSource::System;
    switch (reason) {
      case robot::HabitatPiecesStopReason::RunTimeout:
        fault_code = robot::FaultCode::SearchTimeout;
        message = "Habitat Pieces stopped: LSS2/LSS3 alignment timeout reached";
        break;
      case robot::HabitatPiecesStopReason::DistanceStrafeTimeout:
        fault_code = robot::FaultCode::SearchTimeout;
        message = "Habitat Pieces stopped: distance strafe timeout reached";
        break;
      case robot::HabitatPiecesStopReason::SlideDownTimeout:
        fault_code = robot::FaultCode::SearchTimeout;
        message = "Habitat Pieces stopped: slide-down timeout reached";
        source = robot::EventSource::Motor;
        break;
      case robot::HabitatPiecesStopReason::ForwardDistanceTimeout:
        fault_code = robot::FaultCode::SearchTimeout;
        message = "Habitat Pieces stopped: forward distance timeout reached";
        break;
      case robot::HabitatPiecesStopReason::SlideLiftTimeout:
        fault_code = robot::FaultCode::SearchTimeout;
        message = "Habitat Pieces stopped: slide-lift timeout reached";
        source = robot::EventSource::Motor;
        break;
      case robot::HabitatPiecesStopReason::ReturnLineTimeout:
        fault_code = robot::FaultCode::SearchTimeout;
        message = "Habitat Pieces stopped: rear-line return timeout reached";
        source = robot::EventSource::Line;
        break;
      case robot::HabitatPiecesStopReason::SlideCommandFailed:
        message = "Habitat Pieces stopped: slide motion failed";
        source = robot::EventSource::Motor;
        break;
      case robot::HabitatPiecesStopReason::ConflictingSlideLimits:
        fault_code = robot::FaultCode::LimitSwitchConflict;
        message = "Habitat Pieces stopped: slide limit switches conflict";
        source = robot::EventSource::Motor;
        break;
      default:
        break;
    }
    enterHabitatPiecesFault(
        context, front_left, front_right, rear_link, reason,
        fault_code, message, source, now_ms);
    return;
  }

  if (context.last_habitat_pieces_update.should_align_side_lines) {
    robot::stopLineFollower(context.follower_state);
    if (context.last_habitat_pieces_update.transitioned) {
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::Line,
               "Habitat Pieces first side line detected; aligning LSS2/LSS3");
    }
    const float alignment_duty = clampFloat(
        context.habitat_pieces_config.line_follow_duty, 0.0F,
        activeMotionDutyCap(context));
    const robot::FourWheelCommand wheels =
        robot::makeHabitatPiecesSideAlignmentCommand(
            alignment_duty,
            context.last_habitat_pieces_update.lss2_latched,
            context.last_habitat_pieces_update.lss3_latched, now_ms,
            context.config.remoteCommandTimeoutMs);
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           wheels, context.config, now_ms)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: side-line alignment command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    printTelemetry(context, sensors, rear_link, now_ms);
    return;
  }

  if (context.last_habitat_pieces_update.should_reverse) {
    robot::stopLineFollower(context.follower_state);
    if (context.last_habitat_pieces_update.transitioned) {
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::Line,
               "Habitat Pieces LSS2 and LSS3 aligned; reversing");
    }
    const robot::FourWheelCommand wheels =
        makeHabitatPiecesBackwardCommand(context, now_ms);
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           wheels, context.config, now_ms)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: reverse command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    printTelemetry(context, sensors, rear_link, now_ms);
    return;
  }

  if (context.last_habitat_pieces_update.should_distance_strafe) {
    robot::stopLineFollower(context.follower_state);
    if (context.last_habitat_pieces_update.transitioned) {
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::System,
               "Habitat Pieces reverse complete; distance-zone strafe started");
    }
    const float strafe_duty = clampFloat(
        context.habitat_pieces_config.distance_strafe_duty, 0.0F,
        activeMotionDutyCap(context));
    const robot::FourWheelCommand wheels =
        robot::makeHabitatPiecesDistanceStrafeCommand(
            context.habitat_pieces_config.distance_strafe_direction,
            strafe_duty, now_ms,
            context.config.remoteCommandTimeoutMs);
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           wheels, context.config, now_ms,
                           robot::LaserDistanceProfile::HighAccuracy)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: distance strafe command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    printTelemetry(context, sensors, rear_link, now_ms);
    return;
  }

  if (context.last_habitat_pieces_update.should_compensation_strafe) {
    robot::stopLineFollower(context.follower_state);
    const robot::HabitatPiecesStrafeDirection direction =
        robot::oppositeHabitatPiecesStrafeDirection(
            context.habitat_pieces_config.distance_strafe_direction);
    const robot::FourWheelCommand wheels =
        robot::makeHabitatPiecesDistanceStrafeCommand(
            direction,
            clampFloat(
                context.habitat_pieces_config.compensation_strafe_duty,
                0.0F, activeMotionDutyCap(context)),
            now_ms, context.config.remoteCommandTimeoutMs);
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           wheels, context.config, now_ms)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: compensation strafe command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    return;
  }

  if (context.last_habitat_pieces_update.should_lower_slide) {
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    return;
  }

  if (context.last_habitat_pieces_update
          .should_drive_forward_to_distance) {
    const robot::FourWheelCommand wheels =
        makeHabitatPiecesTranslationCommand(
            context, 0.0F, 1.0F,
            context.habitat_pieces_config.forward_to_distance_duty,
            now_ms);
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           wheels, context.config, now_ms)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: forward distance command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    return;
  }

  if (context.last_habitat_pieces_update.should_post_pickup_reverse) {
    const robot::FourWheelCommand wheels =
        makeHabitatPiecesTranslationCommand(
            context, 0.0F, -1.0F,
            context.habitat_pieces_config.post_pickup_reverse_duty,
            now_ms);
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           wheels, context.config, now_ms)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: post-pickup reverse command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    return;
  }

  if (context.last_habitat_pieces_update.should_return_line_strafe) {
    const robot::HabitatPiecesStrafeDirection direction =
        robot::oppositeHabitatPiecesStrafeDirection(
            context.habitat_pieces_config.distance_strafe_direction);
    const robot::FourWheelCommand wheels =
        robot::makeHabitatPiecesDistanceStrafeCommand(
            direction,
            clampFloat(context.habitat_pieces_config.return_strafe_duty,
                       0.0F, activeMotionDutyCap(context)),
            now_ms, context.config.remoteCommandTimeoutMs);
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           wheels, context.config, now_ms)) {
      enterHabitatPiecesFault(
          context, front_left, front_right, rear_link,
          robot::HabitatPiecesStopReason::RearCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Pieces stopped: rear-line return strafe command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    return;
  }

  if (context.last_habitat_pieces_update.should_wait_for_slide_lift) {
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    return;
  }

  if (context.last_habitat_pieces_update.transitioned &&
      (context.habitat_pieces.state ==
           robot::HabitatPiecesState::ForwardToDistance ||
       context.habitat_pieces.state ==
           robot::HabitatPiecesState::ReturnLineStrafing)) {
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    return;
  }

  if (!context.last_habitat_pieces_update.should_line_follow) {
    disableMotionActuators(context, front_left, front_right, rear_link,
                           now_ms);
    if (context.habitat_pieces.state ==
            robot::HabitatPiecesState::Complete &&
        context.last_habitat_pieces_update.target_reached) {
      clearFault(context);
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::System,
               "Habitat Pieces rear line reached and slide lift complete; stopped");
    } else if (context.habitat_pieces.stop_reason ==
               robot::HabitatPiecesStopReason::DistanceStrafeTimeout) {
      setFault(context, robot::FaultCode::SearchTimeout,
               "Habitat Pieces stopped: distance strafe timeout reached");
      logEvent(context, now_ms, robot::EventSeverity::Fault,
               robot::EventSource::System,
               "Habitat Pieces stopped: distance strafe timeout reached");
    } else if (context.habitat_pieces.timed_out) {
      setFault(context, robot::FaultCode::SearchTimeout,
               "Habitat Pieces stopped: LSS2/LSS3 alignment timeout reached");
      logEvent(context, now_ms, robot::EventSeverity::Fault,
               robot::EventSource::System,
               "Habitat Pieces stopped: LSS2/LSS3 alignment timeout reached");
    } else {
      setFault(context, robot::FaultCode::InvalidCommand,
               "Habitat Pieces stopped: side-line gate unsafe");
      logEvent(context, now_ms, robot::EventSeverity::Fault,
               robot::EventSource::Line,
               "Habitat Pieces stopped: side-line gate unsafe");
    }
    return;
  }

  const robot::LineFollowerConfig line_config =
      habitatPiecesLineFollowerConfig(context);
  context.last_update = robot::updateLineFollower(
      context.follower_state, context.last_line_observation.left_black,
      context.last_line_observation.right_black, line_config, now_ms);
  if (!context.follower_state.enabled ||
      !context.last_update.observation.safe_to_drive) {
    enterHabitatPiecesFault(
        context, front_left, front_right, rear_link,
        robot::HabitatPiecesStopReason::FrontLineLost,
        robot::FaultCode::InvalidCommand,
        "Habitat Pieces stopped: front line lost without history",
        robot::EventSource::Line, now_ms);
    return;
  }
  if (!applyWheelCommand(context, front_left, front_right, rear_link,
                         context.last_update.wheel_command, line_config,
                         now_ms,
                         robot::LaserDistanceProfile::HighAccuracy)) {
    enterHabitatPiecesFault(
        context, front_left, front_right, rear_link,
        robot::HabitatPiecesStopReason::RearCommandFailed,
        robot::FaultCode::CommunicationStale,
        "Habitat Pieces stopped: rear command failed",
        robot::EventSource::Uart, now_ms);
    return;
  }
  context.last_command_ms = now_ms;
  printTelemetry(context, sensors, rear_link, now_ms);
}

void enterHabitatPlacementFault(
    RuntimeContext& context, robot::IMotorOutput& front_left,
    robot::IMotorOutput& front_right, RearCommandLink& rear_link,
    robot::esp2::StepperAxis& stepper,
    const robot::HabitatPlacementFaultReason reason,
    const robot::FaultCode fault_code, const char* message,
    const robot::EventSource source,
    const robot::Milliseconds now_ms) {
  stopAutonomousImuTurn(context);
  stepper.stop();
  (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                   rear_link, now_ms);
  robot::failHabitatPlacementAutonomy(context.habitat_placement, reason,
                                      now_ms);
  context.last_habitat_placement_update = {};
  context.last_habitat_placement_update.state =
      robot::HabitatPlacementState::Fault;
  context.last_habitat_placement_update.fault_reason = reason;
  context.last_habitat_placement_update.faulted = true;
  setFault(context, fault_code, message);
  logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
}

void runHabitatPlacement(
    RuntimeContext& context, DigitalFrontLineSensorReader& sensors,
    DualPwmMotorOutput& front_left, DualPwmMotorOutput& front_right,
    RearCommandLink& rear_link, ClawServoBank& claws,
    robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms) {
  if (context.habitat_placement.state ==
      robot::HabitatPlacementState::WaitForStart) {
    stepper.stop();
    (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                     rear_link, now_ms);
    if (!context.habitat_placement_start_requested) {
      return;
    }
    context.habitat_placement_start_requested = false;
    if (!habitatPlacementStartRequirementsMet(
            sensors, front_left, front_right, rear_link, claws, stepper,
            now_ms, context)) {
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          robot::HabitatPlacementFaultReason::HardwareNotReady,
          robot::FaultCode::HardwareNotConfigured,
          "Habitat Placement start rejected: configure drive, rear/LSS1/front sensors, IMU turns, stepper, pusher, duties, angles, and timings",
          robot::EventSource::System, now_ms);
      return;
    }
    const float counter_clockwise_relative_angle_deg =
        -robot::clockwiseTurnRelativeAngleDeg(
            context.habitat_placement_config
                .counter_clockwise_angle_deg,
            context.imu_turn_config.yaw_command_polarity);
    if (!robot::startHabitatPlacementAutonomy(
            context.habitat_placement,
            context.latest_imu_snapshot.state.heading_deg,
            counter_clockwise_relative_angle_deg, now_ms)) {
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          robot::HabitatPlacementFaultReason::ImuUnavailable,
          robot::FaultCode::HardwareNotConfigured,
          "Habitat Placement start rejected: initial IMU heading unavailable",
          robot::EventSource::System, now_ms);
      return;
    }
    robot::startLineFollower(context.follower_state, now_ms);
    context.last_rear_update = {};
    context.last_command_ms = now_ms;
    context.command_deadman_armed = true;
    context.mode_expires_at_ms = 0U;
    clearFault(context);
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::Line,
             "Habitat Placement reverse rear-line follow started");
  }

  if (context.habitat_placement.state ==
          robot::HabitatPlacementState::Complete ||
      context.habitat_placement.state ==
          robot::HabitatPlacementState::Fault) {
    stopAutonomousImuTurn(context);
    stepper.stop();
    (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                     rear_link, now_ms);
    return;
  }

  if (!rear_link.configured() ||
      !rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(context.rear_config))) {
    enterHabitatPlacementFault(
        context, front_left, front_right, rear_link, stepper,
        robot::HabitatPlacementFaultReason::RearLinkStale,
        robot::FaultCode::CommunicationStale,
        "Habitat Placement stopped: ESP1 link stale",
        robot::EventSource::Uart, now_ms);
    return;
  }

  if (context.habitat_placement.state ==
      robot::HabitatPlacementState::ReverseLineFollow) {
    if (!rear_link.rearLineSnapshotFresh(
            now_ms, context.rear_config.remoteCommandTimeoutMs)) {
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          robot::HabitatPlacementFaultReason::RearLineDataStale,
          robot::FaultCode::CommunicationStale,
          "Habitat Placement stopped: rear/LSS1 data stale",
          robot::EventSource::Uart, now_ms);
      return;
    }
    const robot::RearLineSensorSnapshot& rear =
        rear_link.latestRearLineSnapshot();
    if (!rear.configured || !rear.side_configured) {
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          robot::HabitatPlacementFaultReason::HardwareNotReady,
          robot::FaultCode::HardwareNotConfigured,
          "Habitat Placement stopped: rear line sensors or LSS1 unavailable",
          robot::EventSource::Line, now_ms);
      return;
    }
  }

  if (stepper.lowerLimitActive() && stepper.upperLimitActive()) {
    enterHabitatPlacementFault(
        context, front_left, front_right, rear_link, stepper,
        robot::HabitatPlacementFaultReason::ConflictingLimitSwitches,
        robot::FaultCode::LimitSwitchConflict,
        "Habitat Placement stopped: stepper limits conflict",
        robot::EventSource::Motor, now_ms);
    return;
  }
  if (context.habitat_placement.state ==
          robot::HabitatPlacementState::LowerSlide &&
      !stepper.lowerLimitActive() &&
      (stepper.motionState() ==
           robot::esp2::StepperMotionState::LimitSearchFailed ||
       stepper.motionState() ==
           robot::esp2::StepperMotionState::Stopped)) {
    enterHabitatPlacementFault(
        context, front_left, front_right, rear_link, stepper,
        robot::HabitatPlacementFaultReason::StepperCommandFailed,
        robot::FaultCode::SearchTimeout,
        "Habitat Placement stopped: slide stopped before bottom limit",
        robot::EventSource::Motor, now_ms);
    return;
  }

  const robot::HabitatPlacementState previous_state =
      context.habitat_placement.state;
  const robot::RearLineSensorSnapshot& rear =
      rear_link.latestRearLineSnapshot();
  robot::HabitatPlacementInputs inputs{};
  inputs.lss1_black =
      rear.side_configured && rear.side_electrical_high;
  inputs.initial_heading_turn_complete =
      previous_state ==
              robot::HabitatPlacementState::TurnToInitialHeading &&
      context.imu_turn_state.state == robot::ImuTurnState::Complete;
  inputs.counter_clockwise_turn_complete =
      previous_state ==
              robot::HabitatPlacementState::TurnCounterClockwise &&
      context.imu_turn_state.state == robot::ImuTurnState::Complete;
  inputs.bottom_limit_active = stepper.lowerLimitActive();
  inputs.top_limit_active = stepper.upperLimitActive();
  inputs.pusher_open_commanded = claws.habitatPusherCommandedOpen();
  inputs.clockwise_turn_complete =
      previous_state == robot::HabitatPlacementState::TurnClockwise &&
      context.imu_turn_state.state == robot::ImuTurnState::Complete;
  inputs.front_line_black =
      context.last_line_observation.left_black ||
      context.last_line_observation.right_black;
  inputs.pusher_closed_commanded =
      claws.habitatPusherCommandedClosed();
  context.last_habitat_placement_update =
      robot::updateHabitatPlacementAutonomy(
          context.habitat_placement, inputs,
          context.habitat_placement_config, now_ms);
  const robot::HabitatPlacementUpdate& update =
      context.last_habitat_placement_update;

  if (update.faulted) {
    const bool lss_timeout =
        update.fault_reason ==
        robot::HabitatPlacementFaultReason::Lss1Timeout;
    const bool front_timeout =
        update.fault_reason ==
        robot::HabitatPlacementFaultReason::FrontLineTimeout;
    const bool stepper_timeout =
        update.fault_reason ==
        robot::HabitatPlacementFaultReason::StepperTimeout;
    enterHabitatPlacementFault(
        context, front_left, front_right, rear_link, stepper,
        update.fault_reason, robot::FaultCode::SearchTimeout,
        lss_timeout
            ? "Habitat Placement stopped: LSS1 search timed out"
            : (front_timeout
                   ? "Habitat Placement stopped: front-line strafe timed out"
                   : (stepper_timeout
                          ? "Habitat Placement stopped: bottom-limit search timed out"
                          : "Habitat Placement stopped: autonomous state fault")),
        robot::EventSource::System, now_ms);
    return;
  }

  if (update.state != previous_state) {
    if (update.state ==
            robot::HabitatPlacementState::TurnToInitialHeading ||
        update.state ==
            robot::HabitatPlacementState::TurnCounterClockwise ||
        update.state == robot::HabitatPlacementState::TurnClockwise) {
      robot::resetImuTurnController(context.imu_turn_state);
      context.last_imu_turn_update = {};
    }
    if (update.state == robot::HabitatPlacementState::LowerSlide) {
      const bool accepted = stepper.setLimitSearchSpeed(
                                context.habitat_placement_config
                                    .stepper_down_speed_steps_per_second) &&
                            (stepper.lowerLimitActive() ||
                             stepper.moveToLowerLimit());
      if (!accepted) {
        enterHabitatPlacementFault(
            context, front_left, front_right, rear_link, stepper,
            robot::HabitatPlacementFaultReason::StepperCommandFailed,
            robot::FaultCode::InvalidCommand,
            "Habitat Placement stopped: slide-down command rejected",
            robot::EventSource::Motor, now_ms);
        return;
      }
    } else if (previous_state ==
               robot::HabitatPlacementState::LowerSlide) {
      stepper.stop();
    }
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::System,
             robot::habitatPlacementStateName(update.state));
  }

  if (update.should_open_pusher || update.should_close_pusher) {
    const ClawServoCommandResult result = claws.command(
        kHabitatPusherServoIndex,
        update.should_open_pusher ? ClawServoPositionRequest::Open
                                  : ClawServoPositionRequest::Closed);
    if (result != ClawServoCommandResult::Accepted) {
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          robot::HabitatPlacementFaultReason::PusherCommandFailed,
          clawFaultCode(result), clawServoResultReason(result),
          robot::EventSource::Motor, now_ms);
      return;
    }
  }

  if (update.should_reverse_line_follow) {
    const robot::LineFollowerConfig line_config =
        habitatPlacementLineFollowerConfig(context);
    context.last_rear_update = robot::updateLineFollower(
        context.follower_state,
        context.last_rear_line_observation.left_black,
        context.last_rear_line_observation.right_black, line_config,
        now_ms);
    if (!context.follower_state.enabled ||
        !context.last_rear_update.observation.safe_to_drive) {
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          robot::HabitatPlacementFaultReason::LineLost,
          robot::FaultCode::InvalidCommand,
          "Habitat Placement stopped: rear line lost without history",
          robot::EventSource::Line, now_ms);
      return;
    }
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           context.last_rear_update.wheel_command,
                           line_config, now_ms)) {
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          robot::HabitatPlacementFaultReason::DriveCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Placement stopped: reverse line command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    return;
  }

  robot::stopLineFollower(context.follower_state);
  if (update.should_turn_to_initial_heading ||
      update.should_turn_counter_clockwise ||
      update.should_turn_clockwise) {
    const bool turn_to_initial_heading =
        update.should_turn_to_initial_heading;
    const bool clockwise = update.should_turn_clockwise;
    const robot::Milliseconds timeout_ms =
        turn_to_initial_heading
            ? context.habitat_placement_config
                  .initial_heading_turn_timeout_ms
            : (clockwise
                   ? context.habitat_placement_config.clockwise_timeout_ms
                   : context.habitat_placement_config
                         .counter_clockwise_timeout_ms);
    AutonomousImuMotionResult result{};
    if (turn_to_initial_heading || update.should_turn_counter_clockwise) {
      const float target_heading_deg =
          turn_to_initial_heading
              ? context.habitat_placement.initial_heading_deg
              : context.habitat_placement
                    .counter_clockwise_target_heading_deg;
      result = runAutonomousImuTurnToHeading(
          context, front_left, front_right, rear_link,
          context.rear_config, target_heading_deg, now_ms, timeout_ms);
    } else {
      const float relative_angle_deg =
          robot::clockwiseTurnRelativeAngleDeg(
              context.habitat_placement_config.clockwise_angle_deg,
              context.imu_turn_config.yaw_command_polarity);
      result = runAutonomousImuTurn(
          context, front_left, front_right, rear_link,
          context.rear_config, relative_angle_deg, now_ms, timeout_ms);
    }
    if (result != AutonomousImuMotionResult::Running &&
        result != AutonomousImuMotionResult::Complete) {
      const bool unavailable =
          result == AutonomousImuMotionResult::ImuUnavailable ||
          result == AutonomousImuMotionResult::MotorUnavailable;
      const bool timed_out = result == AutonomousImuMotionResult::Timeout;
      const bool link_failed =
          result == AutonomousImuMotionResult::RearLinkUnavailable ||
          result == AutonomousImuMotionResult::CommandFailed;
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          unavailable
              ? robot::HabitatPlacementFaultReason::ImuUnavailable
              : (timed_out
                     ? robot::HabitatPlacementFaultReason::ImuTurnTimeout
                     : robot::HabitatPlacementFaultReason::ImuTurnFailed),
          unavailable
              ? robot::FaultCode::HardwareNotConfigured
              : (link_failed ? robot::FaultCode::CommunicationStale
                             : robot::FaultCode::InvalidCommand),
          "Habitat Placement stopped: IMU turn failed",
          link_failed ? robot::EventSource::Uart
                      : robot::EventSource::System,
          now_ms);
    }
    return;
  }

  bool moving = false;
  robot::FourWheelCommand wheels = robot::disabledFourWheelCommand();
  if (update.should_strafe_right_before_counter_clockwise) {
    moving = true;
    wheels = makeHabitatPlacementTranslationCommand(
        context, 1.0F, 0.0F,
        context.habitat_placement_config
            .pre_counter_clockwise_strafe_right_duty,
        now_ms);
  } else if (update.should_drive_forward_to_slide) {
    moving = true;
    wheels = makeHabitatPlacementTranslationCommand(
        context, 0.0F, 1.0F,
        context.habitat_placement_config.forward_to_slide_duty, now_ms);
  } else if (update.should_drive_forward_push) {
    moving = true;
    wheels = makeHabitatPlacementTranslationCommand(
        context, 0.0F, 1.0F,
        context.habitat_placement_config.push_forward_duty, now_ms);
  } else if (update.should_drive_reverse_retreat) {
    moving = true;
    wheels = makeHabitatPlacementTranslationCommand(
        context, 0.0F, -1.0F,
        context.habitat_placement_config.reverse_retreat_duty, now_ms);
  } else if (update.should_drive_reverse_after_clockwise) {
    moving = true;
    wheels = makeHabitatPlacementTranslationCommand(
        context, 0.0F, -1.0F,
        context.habitat_placement_config
            .post_clockwise_reverse_duty,
        now_ms);
  } else if (update.should_strafe_left_after_clockwise) {
    moving = true;
    wheels = makeHabitatPlacementTranslationCommand(
        context, -1.0F, 0.0F,
        context.habitat_placement_config
            .post_clockwise_strafe_left_duty,
        now_ms);
  } else if (update.should_drive_forward_exit) {
    moving = true;
    wheels = makeHabitatPlacementTranslationCommand(
        context, 0.0F, 1.0F,
        context.habitat_placement_config.exit_forward_duty, now_ms);
  } else if (update.should_strafe_right) {
    moving = true;
    wheels = makeHabitatPlacementTranslationCommand(
        context, 1.0F, 0.0F,
        context.habitat_placement_config.strafe_right_duty, now_ms);
  }

  if (moving) {
    stopAutonomousImuTurn(context);
    if (!applyWheelCommand(context, front_left, front_right, rear_link,
                           wheels, context.rear_config, now_ms)) {
      enterHabitatPlacementFault(
          context, front_left, front_right, rear_link, stepper,
          robot::HabitatPlacementFaultReason::DriveCommandFailed,
          robot::FaultCode::CommunicationStale,
          "Habitat Placement stopped: drive command failed",
          robot::EventSource::Uart, now_ms);
      return;
    }
    context.last_command_ms = now_ms;
    return;
  }

  stopAutonomousImuTurn(context);
  if (!stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                  rear_link, now_ms)) {
    enterHabitatPlacementFault(
        context, front_left, front_right, rear_link, stepper,
        robot::HabitatPlacementFaultReason::DriveCommandFailed,
        robot::FaultCode::CommunicationStale,
        "Habitat Placement stopped: stop command failed",
        robot::EventSource::Uart, now_ms);
    return;
  }
  if (update.complete) {
    clearFault(context);
  }
}

void enterTowerPiecesFault(
    RuntimeContext& context, robot::IMotorOutput& front_left,
    robot::IMotorOutput& front_right, RearCommandLink& rear_link,
    robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms,
    const robot::TowerPiecesFaultReason reason,
    const robot::FaultCode fault_code, const char* message,
    const robot::EventSource source) {
  stopAutonomousImuStrafe(context);
  stopAutonomousImuTurn(context);
  stepper.stop();
  (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                   rear_link, now_ms);
  robot::failTowerPiecesAutonomy(context.tower_pieces, reason, now_ms);
  setFault(context, fault_code, message);
  logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
}

void enterTowerPiecesImuMotionFault(
    RuntimeContext& context, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    robot::esp2::StepperAxis& stepper,
    const AutonomousImuMotionResult result, const bool is_turn,
    const robot::Milliseconds now_ms) {
  const bool imu_unavailable =
      result == AutonomousImuMotionResult::ImuUnavailable;
  const bool link_failed =
      result == AutonomousImuMotionResult::RearLinkUnavailable ||
      result == AutonomousImuMotionResult::CommandFailed;
  const bool timed_out =
      result == AutonomousImuMotionResult::Timeout;
  const robot::TowerPiecesFaultReason reason =
      imu_unavailable
          ? robot::TowerPiecesFaultReason::ImuUnavailable
          : (link_failed
                 ? robot::TowerPiecesFaultReason::RearCommandFailed
                 : (is_turn
                        ? (timed_out
                               ? robot::TowerPiecesFaultReason::
                                     ImuTurnTimeout
                               : robot::TowerPiecesFaultReason::
                                     ImuTurnFailed)
                        : robot::TowerPiecesFaultReason::
                              ImuStrafeFailed));
  const robot::FaultCode fault_code =
      imu_unavailable ||
              result == AutonomousImuMotionResult::MotorUnavailable
          ? robot::FaultCode::HardwareNotConfigured
          : (link_failed
                 ? robot::FaultCode::CommunicationStale
                 : (timed_out ? robot::FaultCode::SearchTimeout
                              : robot::FaultCode::InvalidCommand));
  const char* message =
      imu_unavailable
          ? "tower pieces stopped: IMU unavailable"
          : (link_failed
                 ? "tower pieces stopped: IMU motion rear command failed"
                 : (is_turn
                        ? (timed_out
                               ? "tower pieces stopped: IMU clockwise turn timed out"
                               : "tower pieces stopped: IMU clockwise turn failed")
                        : "tower pieces stopped: IMU strafe failed"));
  enterTowerPiecesFault(
      context, front_left, front_right, rear_link, stepper, now_ms,
      reason, fault_code, message,
      link_failed ? robot::EventSource::Uart
                  : robot::EventSource::System);
}

bool stopTowerPiecesOutputsOrFault(
    RuntimeContext& context, robot::IMotorOutput& front_left,
    robot::IMotorOutput& front_right, RearCommandLink& rear_link,
    robot::esp2::StepperAxis& stepper,
    const robot::Milliseconds now_ms) {
  if (stopAutonomyDriveAndFunnel(context, front_left, front_right, rear_link,
                                 now_ms)) {
    return true;
  }
  enterTowerPiecesFault(
      context, front_left, front_right, rear_link, stepper, now_ms,
      robot::TowerPiecesFaultReason::RearCommandFailed,
      robot::FaultCode::CommunicationStale,
      "tower pieces stopped: drive or funnel stop command failed",
      robot::EventSource::Uart);
  return false;
}

void runTowerPiecesAutonomy(RuntimeContext& context,
                            DualPwmMotorOutput& front_left,
                            DualPwmMotorOutput& front_right,
                            RearCommandLink& rear_link,
                            ClawServoBank& claws,
                            robot::esp2::StepperAxis& stepper,
                            const robot::Milliseconds now_ms) {
  switch (context.tower_pieces.state) {
    case robot::TowerPiecesState::WaitForStart: {
      stepper.stop();
      const bool outputs_stopped = stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      if (!context.tower_pieces_start_requested) {
        return;
      }
      context.tower_pieces_start_requested = false;
      if (!outputs_stopped) {
        enterTowerPiecesFault(
            context, front_left, front_right, rear_link, stepper, now_ms,
            robot::TowerPiecesFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "tower pieces start rejected: drive or funnel stop command failed",
            robot::EventSource::Uart);
        return;
      }
      if (!towerPiecesStartRequirementsMet(front_left, front_right, rear_link,
                                           claws, stepper, now_ms, context)) {
        enterTowerPiecesFault(
            context, front_left, front_right, rear_link, stepper, now_ms,
            robot::TowerPiecesFaultReason::HardwareNotReady,
            robot::FaultCode::HardwareNotConfigured,
            "tower pieces start rejected: configure motion, shared IMU turn/strafe tuning, rear/side sensors, servos, stepper, timings, and link",
            robot::EventSource::System);
        return;
      }
      robot::startTowerPiecesAutonomy(
          context.tower_pieces,
          rear_link.latestRearLineSnapshot().side_electrical_high, now_ms);
      robot::startLineFollower(context.follower_state, now_ms);
      context.last_command_ms = now_ms;
      context.command_deadman_armed = true;
      context.mode_expires_at_ms = 0U;
      clearFault(context);
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::Line,
               "tower pieces reverse line follow started");
      return;
    }

    case robot::TowerPiecesState::ReverseLineFollow:
    case robot::TowerPiecesState::PostLineDelay:
    case robot::TowerPiecesState::StrafeRight:
    case robot::TowerPiecesState::PostStrafePause:
    case robot::TowerPiecesState::RotateClockwise:
    case robot::TowerPiecesState::PostRotationPause:
    case robot::TowerPiecesState::ReverseTimed:
    case robot::TowerPiecesState::ShimmyLeft:
    case robot::TowerPiecesState::ShimmyRight:
    case robot::TowerPiecesState::FinalReverse:
    case robot::TowerPiecesState::PostFinalReverseDelay:
    case robot::TowerPiecesState::WinchOpen:
    case robot::TowerPiecesState::PostWinchOpenDelay:
    case robot::TowerPiecesState::ClawsOpen:
    case robot::TowerPiecesState::PostClawsOpenDelay:
    case robot::TowerPiecesState::PreStepperBottomDelay:
    case robot::TowerPiecesState::MoveStepperBottom:
    case robot::TowerPiecesState::PostStepperBottomDelay:
    case robot::TowerPiecesState::ClawsClosed:
    case robot::TowerPiecesState::PostClawsClosedDelay:
    case robot::TowerPiecesState::MoveStepperTop:
    case robot::TowerPiecesState::WinchClosed:
      break;
    case robot::TowerPiecesState::Complete:
    case robot::TowerPiecesState::Fault:
      stopAutonomousImuStrafe(context);
      stopAutonomousImuTurn(context);
      stepper.stop();
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      return;
  }

  const bool imu_turn_phase =
      context.tower_pieces.state ==
      robot::TowerPiecesState::RotateClockwise;
  const bool imu_strafe_phase =
      context.tower_pieces.state ==
          robot::TowerPiecesState::StrafeRight ||
      context.tower_pieces.state ==
          robot::TowerPiecesState::ShimmyLeft ||
      context.tower_pieces.state ==
          robot::TowerPiecesState::ShimmyRight;
  if (imu_turn_phase || imu_strafe_phase) {
    robot::ImuRecoveryState& recovery =
        imu_turn_phase ? context.imu_turn_recovery
                       : context.imu_strafe_recovery;
    const AutonomousImuRecoveryUpdate recovery_update =
        serviceAutonomousImuRecovery(
            context, recovery, imu_turn_phase, front_left, front_right,
            rear_link, context.rear_config, now_ms);
    context.tower_pieces.state_entered_at_ms +=
        recovery_update.timer_adjustment_ms;
    context.tower_pieces.started_at_ms +=
        recovery_update.timer_adjustment_ms;
    if (context.tower_pieces.state ==
            robot::TowerPiecesState::ShimmyLeft ||
        context.tower_pieces.state ==
            robot::TowerPiecesState::ShimmyRight) {
      context.tower_pieces.shimmy_started_at_ms +=
          recovery_update.timer_adjustment_ms;
    }
    if (recovery_update.result ==
        AutonomousImuRecoveryResult::Paused) {
      return;
    }
    if (recovery_update.result ==
            AutonomousImuRecoveryResult::TimedOut ||
        recovery_update.result ==
            AutonomousImuRecoveryResult::StopCommandFailed) {
      enterTowerPiecesImuMotionFault(
          context, front_left, front_right, rear_link, stepper,
          recovery_update.result ==
                  AutonomousImuRecoveryResult::TimedOut
              ? AutonomousImuMotionResult::ImuUnavailable
              : AutonomousImuMotionResult::CommandFailed,
          imu_turn_phase, now_ms);
      return;
    }
  }

  if (!rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(context.rear_config))) {
    enterTowerPiecesFault(
        context, front_left, front_right, rear_link, stepper, now_ms,
        robot::TowerPiecesFaultReason::RearLinkStale,
        robot::FaultCode::CommunicationStale,
        "tower pieces stopped: ESP1 status stale",
        robot::EventSource::Uart);
    return;
  }
  const bool requires_rear_line_data =
      context.tower_pieces.state ==
          robot::TowerPiecesState::ReverseLineFollow ||
      context.tower_pieces.state == robot::TowerPiecesState::PostLineDelay ||
      context.tower_pieces.state == robot::TowerPiecesState::StrafeRight ||
      context.tower_pieces.state ==
          robot::TowerPiecesState::PostStrafePause ||
      context.tower_pieces.state ==
          robot::TowerPiecesState::RotateClockwise ||
      context.tower_pieces.state ==
          robot::TowerPiecesState::PostRotationPause ||
      context.tower_pieces.state == robot::TowerPiecesState::ReverseTimed ||
      context.tower_pieces.state == robot::TowerPiecesState::ShimmyLeft ||
      context.tower_pieces.state == robot::TowerPiecesState::ShimmyRight;
  if (requires_rear_line_data &&
      !rear_link.rearLineSnapshotFresh(
          now_ms, context.rear_config.remoteCommandTimeoutMs)) {
    enterTowerPiecesFault(
        context, front_left, front_right, rear_link, stepper, now_ms,
        robot::TowerPiecesFaultReason::RearLinkStale,
        robot::FaultCode::CommunicationStale,
        "tower pieces stopped: rear line sensor data stale",
        robot::EventSource::Uart);
    return;
  }

  const robot::Esp1StatusReport& esp1 = rear_link.latestStatus();
  const robot::RearLineSensorSnapshot& line_sensors =
      rear_link.latestRearLineSnapshot();
  if (!rear_link.configured() ||
      (requires_rear_line_data &&
       (!line_sensors.configured || !line_sensors.side_configured))) {
    enterTowerPiecesFault(
        context, front_left, front_right, rear_link, stepper, now_ms,
        robot::TowerPiecesFaultReason::HardwareNotReady,
        robot::FaultCode::HardwareNotConfigured,
        "tower pieces stopped: rear or side line sensor unavailable",
        robot::EventSource::Line);
    return;
  }
  if (esp1.fault_active &&
      esp1.fault_code == robot::FaultCode::CommunicationStale) {
    enterTowerPiecesFault(
        context, front_left, front_right, rear_link, stepper, now_ms,
        robot::TowerPiecesFaultReason::RearLinkStale,
        robot::FaultCode::CommunicationStale,
        "tower pieces stopped: ESP1 reported stale commands",
        robot::EventSource::Uart);
    return;
  }

  const bool moving_stepper_bottom =
      context.tower_pieces.state ==
      robot::TowerPiecesState::MoveStepperBottom;
  const bool moving_stepper_top =
      context.tower_pieces.state == robot::TowerPiecesState::MoveStepperTop;
  if ((moving_stepper_bottom || moving_stepper_top) &&
      stepper.motionState() ==
          robot::esp2::StepperMotionState::LimitSearchFailed) {
    enterTowerPiecesFault(
        context, front_left, front_right, rear_link, stepper, now_ms,
        robot::TowerPiecesFaultReason::StepperLimitSearchFailed,
        robot::FaultCode::SearchTimeout,
        moving_stepper_bottom
            ? "tower pieces stopped: bottom limit search failed"
            : "tower pieces stopped: top limit search failed",
        robot::EventSource::Motor);
    return;
  }
  if ((moving_stepper_bottom && !stepper.lowerLimitActive() &&
       stepper.motionState() ==
           robot::esp2::StepperMotionState::Stopped) ||
      (moving_stepper_top && !stepper.upperLimitActive() &&
       stepper.motionState() ==
           robot::esp2::StepperMotionState::Stopped)) {
    enterTowerPiecesFault(
        context, front_left, front_right, rear_link, stepper, now_ms,
        robot::TowerPiecesFaultReason::StepperCommandFailed,
        robot::FaultCode::InvalidCommand,
        "tower pieces stopped: stepper stopped before its target limit",
        robot::EventSource::Motor);
    return;
  }

  const robot::TowerPiecesState previous_state = context.tower_pieces.state;
  const robot::TowerPiecesInputs inputs{
      line_sensors.side_electrical_high,
      line_sensors.left_electrical_high,
      line_sensors.right_electrical_high,
      stepper.lowerLimitActive(),
      stepper.upperLimitActive(),
      context.tower_pieces.state ==
              robot::TowerPiecesState::RotateClockwise &&
          context.imu_turn_state.state ==
              robot::ImuTurnState::Complete};
  const robot::TowerPiecesUpdate tower_update =
      robot::updateTowerPiecesAutonomy(context.tower_pieces, inputs,
                                       context.tower_pieces_config, now_ms);
  context.last_tower_pieces_update = tower_update;
  if (tower_update.state == robot::TowerPiecesState::Fault &&
      tower_update.fault_reason ==
          robot::TowerPiecesFaultReason::ConflictingLimitSwitches) {
    enterTowerPiecesFault(
        context, front_left, front_right, rear_link, stepper, now_ms,
        tower_update.fault_reason, robot::FaultCode::LimitSwitchConflict,
        "tower pieces stopped: bottom and top stepper limits are both active",
        robot::EventSource::Motor);
    return;
  }
  if (tower_update.side_line_rising_edge) {
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::Line,
             tower_update.side_line_count >=
                     robot::kTowerPiecesTargetSideLineCount
                 ? "tower pieces side line count: 2"
                 : "tower pieces side line count: 1");
  }
  if (tower_update.state != previous_state) {
    switch (tower_update.state) {
      case robot::TowerPiecesState::PostLineDelay:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Line,
                 "tower pieces stopped on second side line; delay started");
        break;
      case robot::TowerPiecesState::StrafeRight:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces right strafe started");
        break;
      case robot::TowerPiecesState::PostStrafePause:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces right strafe complete; pause started");
        break;
      case robot::TowerPiecesState::RotateClockwise:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces clockwise rotation started");
        break;
      case robot::TowerPiecesState::PostRotationPause:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces rotation complete; pause started");
        break;
      case robot::TowerPiecesState::ReverseTimed:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces timed reverse started");
        break;
      case robot::TowerPiecesState::ShimmyLeft:
        break;
      case robot::TowerPiecesState::ShimmyRight:
        if (previous_state == robot::TowerPiecesState::ReverseTimed) {
          logEvent(context, now_ms, robot::EventSeverity::Info,
                   robot::EventSource::Motor,
                   "tower pieces shimmy search started right");
        }
        break;
      case robot::TowerPiecesState::FinalReverse:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces back line detected; final backward motion started");
        break;
      case robot::TowerPiecesState::PostFinalReverseDelay:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 previous_state == robot::TowerPiecesState::FinalReverse
                     ? "tower pieces final backward motion complete; delay started"
                     : "tower pieces back line detected; optional final backward motion skipped; delay started");
        break;
      case robot::TowerPiecesState::WinchOpen:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor, "tower pieces winch opened");
        break;
      case robot::TowerPiecesState::PostWinchOpenDelay:
        break;
      case robot::TowerPiecesState::ClawsOpen:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor, "tower pieces claws opened");
        break;
      case robot::TowerPiecesState::PostClawsOpenDelay:
      case robot::TowerPiecesState::PreStepperBottomDelay:
        break;
        break;
      case robot::TowerPiecesState::MoveStepperBottom:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces stepper moving to bottom limit");
        break;
      case robot::TowerPiecesState::PostStepperBottomDelay:
        stepper.stop();
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces bottom limit reached; delay started");
        break;
      case robot::TowerPiecesState::ClawsClosed:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor, "tower pieces claws closed");
        break;
      case robot::TowerPiecesState::PostClawsClosedDelay:
        break;
      case robot::TowerPiecesState::MoveStepperTop:
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces stepper moving to top limit");
        break;
      case robot::TowerPiecesState::WinchClosed:
        stepper.stop();
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::Motor,
                 "tower pieces top limit reached; winch closed");
        break;
      case robot::TowerPiecesState::WaitForStart:
      case robot::TowerPiecesState::ReverseLineFollow:
      case robot::TowerPiecesState::Complete:
      case robot::TowerPiecesState::Fault:
        break;
    }

    ClawServoCommandResult servo_result =
        ClawServoCommandResult::Accepted;
    bool stepper_command_accepted = true;
    switch (tower_update.state) {
      case robot::TowerPiecesState::WinchOpen:
        servo_result = claws.command(kWinchServoIndex,
                                     ClawServoPositionRequest::Open);
        break;
      case robot::TowerPiecesState::ClawsOpen:
        servo_result = claws.commandAll(ClawServoPositionRequest::Open);
        break;
      case robot::TowerPiecesState::MoveStepperBottom:
        stepper_command_accepted = stepper.setLimitSearchSpeed(
            context.tower_pieces_config
                .stepper_down_speed_steps_per_second);
        if (stepper_command_accepted) {
          stepper_command_accepted = stepper.moveToLowerLimit();
        }
        break;
      case robot::TowerPiecesState::ClawsClosed:
        servo_result = claws.commandAll(ClawServoPositionRequest::Closed);
        break;
      case robot::TowerPiecesState::MoveStepperTop:
        stepper_command_accepted = stepper.setLimitSearchSpeed(
            context.tower_pieces_config.stepper_up_speed_steps_per_second);
        if (stepper_command_accepted) {
          stepper_command_accepted = stepper.moveToUpperLimit();
        }
        break;
      case robot::TowerPiecesState::WinchClosed:
        servo_result = claws.command(kWinchServoIndex,
                                     ClawServoPositionRequest::Closed);
        break;
      case robot::TowerPiecesState::WaitForStart:
      case robot::TowerPiecesState::ReverseLineFollow:
      case robot::TowerPiecesState::PostLineDelay:
      case robot::TowerPiecesState::StrafeRight:
      case robot::TowerPiecesState::PostStrafePause:
      case robot::TowerPiecesState::RotateClockwise:
      case robot::TowerPiecesState::PostRotationPause:
      case robot::TowerPiecesState::ReverseTimed:
      case robot::TowerPiecesState::ShimmyLeft:
      case robot::TowerPiecesState::ShimmyRight:
      case robot::TowerPiecesState::FinalReverse:
      case robot::TowerPiecesState::PostFinalReverseDelay:
      case robot::TowerPiecesState::PostWinchOpenDelay:
      case robot::TowerPiecesState::PostClawsOpenDelay:
      case robot::TowerPiecesState::PreStepperBottomDelay:
      case robot::TowerPiecesState::PostStepperBottomDelay:
      case robot::TowerPiecesState::PostClawsClosedDelay:
      case robot::TowerPiecesState::Complete:
      case robot::TowerPiecesState::Fault:
        break;
    }
    if (servo_result != ClawServoCommandResult::Accepted) {
      enterTowerPiecesFault(
          context, front_left, front_right, rear_link, stepper, now_ms,
          robot::TowerPiecesFaultReason::ServoCommandFailed,
          robot::FaultCode::InvalidCommand,
          clawServoResultReason(servo_result), robot::EventSource::Motor);
      return;
    }
    if (!stepper_command_accepted) {
      enterTowerPiecesFault(
          context, front_left, front_right, rear_link, stepper, now_ms,
          robot::TowerPiecesFaultReason::StepperCommandFailed,
          robot::FaultCode::InvalidCommand,
          "tower pieces stopped: stepper command rejected",
          robot::EventSource::Motor);
      return;
    }
  }

  switch (tower_update.state) {
    case robot::TowerPiecesState::ReverseLineFollow: {
      context.requested_funnel_command = robot::disabledMotorCommand();
      if (!sendStoppedFunnelCommand(rear_link, context.rear_config, now_ms)) {
        enterTowerPiecesFault(
            context, front_left, front_right, rear_link, stepper, now_ms,
            robot::TowerPiecesFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "tower pieces stopped: funnel stop command failed",
            robot::EventSource::Uart);
        return;
      }
      const float line_follow_duty =
          context.tower_pieces_config.reverse_line_duty;
      const robot::LineFollowerConfig reverse_config =
          towerPiecesLineFollowerConfig(context, line_follow_duty);
      context.last_rear_update = robot::updateLineFollower(
          context.follower_state,
          context.last_rear_line_observation.left_black,
          context.last_rear_line_observation.right_black, reverse_config,
          now_ms);
      if (!context.follower_state.enabled &&
          !context.last_rear_update.observation.safe_to_drive) {
        enterTowerPiecesFault(
            context, front_left, front_right, rear_link, stepper, now_ms,
            robot::TowerPiecesFaultReason::LineLost,
            robot::FaultCode::InvalidCommand,
            "tower pieces stopped: rear line lost without history",
            robot::EventSource::Line);
        return;
      }
      if (!applyWheelCommand(context, front_left, front_right, rear_link,
                             context.last_rear_update.wheel_command,
                             reverse_config, now_ms)) {
        enterTowerPiecesFault(
            context, front_left, front_right, rear_link, stepper, now_ms,
            robot::TowerPiecesFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "tower pieces stopped: rear command send failed",
            robot::EventSource::Uart);
        return;
      }
      break;
    }

    case robot::TowerPiecesState::PostLineDelay:
    case robot::TowerPiecesState::PostStrafePause:
    case robot::TowerPiecesState::PostRotationPause:
    case robot::TowerPiecesState::PostFinalReverseDelay:
    case robot::TowerPiecesState::WinchOpen:
    case robot::TowerPiecesState::PostWinchOpenDelay:
    case robot::TowerPiecesState::ClawsOpen:
    case robot::TowerPiecesState::PostClawsOpenDelay:
    case robot::TowerPiecesState::PreStepperBottomDelay:
    case robot::TowerPiecesState::MoveStepperBottom:
    case robot::TowerPiecesState::PostStepperBottomDelay:
    case robot::TowerPiecesState::ClawsClosed:
    case robot::TowerPiecesState::PostClawsClosedDelay:
    case robot::TowerPiecesState::MoveStepperTop:
    case robot::TowerPiecesState::WinchClosed:
      stopAutonomousImuStrafe(context);
      stopAutonomousImuTurn(context);
      (void)stopTowerPiecesOutputsOrFault(
          context, front_left, front_right, rear_link, stepper, now_ms);
      return;

    case robot::TowerPiecesState::StrafeRight:
    case robot::TowerPiecesState::ReverseTimed:
    case robot::TowerPiecesState::ShimmyLeft:
    case robot::TowerPiecesState::ShimmyRight:
    case robot::TowerPiecesState::FinalReverse: {
      robot::stopLineFollower(context.follower_state);
      context.requested_funnel_command = robot::disabledMotorCommand();
      if (!sendStoppedFunnelCommand(rear_link, context.rear_config, now_ms)) {
        enterTowerPiecesFault(
            context, front_left, front_right, rear_link, stepper, now_ms,
            robot::TowerPiecesFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "tower pieces stopped: funnel stop command failed",
            robot::EventSource::Uart);
        return;
      }
      if (tower_update.should_initial_strafe_right) {
        const AutonomousImuMotionResult result =
            runAutonomousImuStrafe(
                context, front_left, front_right, rear_link,
                context.rear_config, 1,
                context.tower_pieces_config.strafe_right_duty, now_ms);
        if (result != AutonomousImuMotionResult::Running) {
          enterTowerPiecesImuMotionFault(
              context, front_left, front_right, rear_link, stepper,
              result, false, now_ms);
          return;
        }
        break;
      }

      if (tower_update.should_shimmy_left ||
          tower_update.should_shimmy_right) {
        const AutonomousImuMotionResult result =
            runAutonomousImuStrafe(
                context, front_left, front_right, rear_link,
                context.rear_config,
                tower_update.should_shimmy_right ? 1 : -1,
                context.tower_pieces_config.shimmy_duty, now_ms);
        if (result != AutonomousImuMotionResult::Running) {
          enterTowerPiecesImuMotionFault(
              context, front_left, front_right, rear_link, stepper,
              result, false, now_ms);
          return;
        }
        break;
      }

      stopAutonomousImuStrafe(context);
      const robot::FourWheelCommand wheels =
          tower_update.should_drive_final_reverse
              ? makeTowerPiecesFinalBackwardCommand(context, now_ms)
              : makeTowerPiecesBackwardCommand(context, now_ms);
      if (!applyWheelCommand(context, front_left, front_right, rear_link,
                             wheels, context.rear_config, now_ms)) {
        enterTowerPiecesFault(
            context, front_left, front_right, rear_link, stepper, now_ms,
            robot::TowerPiecesFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "tower pieces stopped: rear command send failed",
            robot::EventSource::Uart);
        return;
      }
      break;
    }

    case robot::TowerPiecesState::RotateClockwise: {
      stopAutonomousImuStrafe(context);
      context.requested_funnel_command = robot::disabledMotorCommand();
      if (!sendStoppedFunnelCommand(rear_link, context.rear_config,
                                    now_ms)) {
        enterTowerPiecesFault(
            context, front_left, front_right, rear_link, stepper, now_ms,
            robot::TowerPiecesFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "tower pieces stopped: funnel stop command failed",
            robot::EventSource::Uart);
        return;
      }
      const AutonomousImuMotionResult result =
          runAutonomousImuTurn(
              context, front_left, front_right, rear_link,
              context.rear_config,
              robot::clockwiseTurnRelativeAngleDeg(
                  context.tower_pieces_config
                      .clockwise_rotation_angle_deg,
                  context.imu_turn_config.yaw_command_polarity),
              now_ms);
      if (result != AutonomousImuMotionResult::Running &&
          result != AutonomousImuMotionResult::Complete) {
        enterTowerPiecesImuMotionFault(
            context, front_left, front_right, rear_link, stepper,
            result, true, now_ms);
        return;
      }
      if (result == AutonomousImuMotionResult::Complete) {
        (void)stopTowerPiecesOutputsOrFault(
            context, front_left, front_right, rear_link, stepper,
            now_ms);
        return;
      }
      break;
    }

    case robot::TowerPiecesState::Complete:
      stopAutonomousImuStrafe(context);
      stopAutonomousImuTurn(context);
      stepper.stop();
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      clearFault(context);
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::Motor, "tower pieces complete");
      return;

    case robot::TowerPiecesState::Fault:
      stopAutonomousImuStrafe(context);
      stopAutonomousImuTurn(context);
      stepper.stop();
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      if (tower_update.fault_reason ==
          robot::TowerPiecesFaultReason::ShimmyTimeout) {
        setFault(context, robot::FaultCode::SearchTimeout,
                 "tower pieces shimmy timeout without back line");
        logEvent(context, now_ms, robot::EventSeverity::Fault,
                 robot::EventSource::Line,
                 "tower pieces shimmy timeout without back line");
      } else {
        setFault(context, robot::FaultCode::SearchTimeout,
                 "tower pieces timeout before second side line");
        logEvent(context, now_ms, robot::EventSeverity::Fault,
                 robot::EventSource::Line,
                 "tower pieces timeout before second side line");
      }
      return;

    case robot::TowerPiecesState::WaitForStart:
      stopAutonomousImuStrafe(context);
      stopAutonomousImuTurn(context);
      stepper.stop();
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      return;
  }

  context.last_command_ms = now_ms;
  context.command_deadman_armed = true;
  context.mode_expires_at_ms = 0U;
  printRearTelemetry(context, rear_link, now_ms);
}

void enterPegFinderFault(
    RuntimeContext& context, robot::IMotorOutput& front_left,
    robot::IMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const robot::PegFinderFaultReason reason,
    const robot::FaultCode fault_code, const char* message,
    const robot::EventSource source) {
  robot::cancelImuRecovery(context.imu_turn_recovery);
  if (robot::imuTurnActive(context.imu_turn_state)) {
    robot::stopImuTurn(context.imu_turn_state);
    context.last_imu_turn_update.state =
        context.imu_turn_state.state;
    context.last_imu_turn_update.fault_reason =
        context.imu_turn_state.fault_reason;
  }
  (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                   rear_link, now_ms);
  robot::failPegFinderAutonomy(context.peg_finder, reason, now_ms);
  setFault(context, fault_code, message);
  logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
}

bool stopPegFinderOutputsOrFault(RuntimeContext& context,
                                 robot::IMotorOutput& front_left,
                                 robot::IMotorOutput& front_right,
                                 RearCommandLink& rear_link,
                                 const robot::Milliseconds now_ms) {
  if (stopAutonomyDriveAndFunnel(context, front_left, front_right, rear_link,
                                 now_ms)) {
    return true;
  }
  enterPegFinderFault(
      context, front_left, front_right, rear_link, now_ms,
      robot::PegFinderFaultReason::RearCommandFailed,
      robot::FaultCode::CommunicationStale,
      "PegFinder stopped: drive or funnel stop command failed",
      robot::EventSource::Uart);
  return false;
}

void runPegFinder(RuntimeContext& context,
                  DualPwmMotorOutput& front_left,
                  DualPwmMotorOutput& front_right,
                  RearCommandLink& rear_link,
                  ClawServoBank& claws,
                  const DigitalActiveHighLimitSwitch& funnel_limit,
                  const robot::Milliseconds now_ms) {
  switch (context.peg_finder.state) {
    case robot::PegFinderState::WaitForStart: {
      const bool outputs_stopped = stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      if (!context.peg_finder_start_requested) {
        return;
      }
      context.peg_finder_start_requested = false;
      if (!outputs_stopped) {
        enterPegFinderFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::PegFinderFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "PegFinder start rejected: drive or funnel stop command failed",
            robot::EventSource::Uart);
        return;
      }
      if (!pegFinderStartRequirementsMet(front_left, front_right, rear_link,
                                         claws, funnel_limit, now_ms,
                                         context)) {
        enterPegFinderFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::PegFinderFaultReason::HardwareNotReady,
            robot::FaultCode::HardwareNotConfigured,
            "PegFinder start rejected: configure drive, funnel, IMU turn tuning, clockwise angle, GPIO 47 limit, claw open angles, timings, and ESP1 link",
            robot::EventSource::System);
        return;
      }
      context.last_imu_turn_update = {};
      robot::resetImuTurnController(context.imu_turn_state);
      robot::startPegFinderAutonomy(context.peg_finder, now_ms);
      context.last_command_ms = now_ms;
      context.command_deadman_armed = true;
      context.mode_expires_at_ms = 0U;
      clearFault(context);
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::Motor,
               "PegFinder clockwise rotation started");
      break;
    }

    case robot::PegFinderState::RotateClockwise:
    case robot::PegFinderState::PostRotationPause:
    case robot::PegFinderState::Reverse:
    case robot::PegFinderState::PostReversePause:
    case robot::PegFinderState::Forward:
    case robot::PegFinderState::FunnelForward:
    case robot::PegFinderState::PostFunnelLimitDelay:
    case robot::PegFinderState::OpenClaw1:
    case robot::PegFinderState::PostClaw1OpenDelay:
    case robot::PegFinderState::OpenClaw2:
    case robot::PegFinderState::PostClaw2OpenDelay:
    case robot::PegFinderState::OpenClaw3:
    case robot::PegFinderState::PostClawsOpenDelay:
    case robot::PegFinderState::FunnelReverse:
      break;
    case robot::PegFinderState::Complete:
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      return;
    case robot::PegFinderState::Fault:
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      return;
  }

  if (context.peg_finder.state ==
      robot::PegFinderState::RotateClockwise) {
    const AutonomousImuRecoveryUpdate recovery_update =
        serviceAutonomousImuRecovery(
            context, context.imu_turn_recovery, true, front_left,
            front_right, rear_link, context.rear_config, now_ms);
    context.peg_finder.state_entered_at_ms +=
        recovery_update.timer_adjustment_ms;
    if (recovery_update.result ==
        AutonomousImuRecoveryResult::Paused) {
      return;
    }
    if (recovery_update.result ==
        AutonomousImuRecoveryResult::TimedOut) {
      enterPegFinderFault(
          context, front_left, front_right, rear_link, now_ms,
          robot::PegFinderFaultReason::ImuUnavailable,
          robot::FaultCode::HardwareNotConfigured,
          "PegFinder stopped: IMU I2C recovery timed out during clockwise turn",
          robot::EventSource::System);
      return;
    }
    if (recovery_update.result ==
        AutonomousImuRecoveryResult::StopCommandFailed) {
      enterPegFinderFault(
          context, front_left, front_right, rear_link, now_ms,
          robot::PegFinderFaultReason::RearCommandFailed,
          robot::FaultCode::CommunicationStale,
          "PegFinder stopped: failed to hold wheel outputs disabled during IMU recovery",
          robot::EventSource::Uart);
      return;
    }
  }

  if (!rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(context.rear_config))) {
    enterPegFinderFault(
        context, front_left, front_right, rear_link, now_ms,
        robot::PegFinderFaultReason::RearLinkStale,
        robot::FaultCode::CommunicationStale,
        "PegFinder stopped: ESP1 status stale", robot::EventSource::Uart);
    return;
  }
  if (!rear_link.configured() ||
      !rear_link.latestStatus().funnel_configured) {
    enterPegFinderFault(
        context, front_left, front_right, rear_link, now_ms,
        robot::PegFinderFaultReason::HardwareNotReady,
        robot::FaultCode::HardwareNotConfigured,
        "PegFinder stopped: drive or funnel hardware unavailable",
        robot::EventSource::Motor);
    return;
  }
  if (rear_link.latestStatus().fault_active &&
      rear_link.latestStatus().fault_code ==
          robot::FaultCode::CommunicationStale) {
    enterPegFinderFault(
        context, front_left, front_right, rear_link, now_ms,
        robot::PegFinderFaultReason::RearLinkStale,
        robot::FaultCode::CommunicationStale,
        "PegFinder stopped: ESP1 reported stale commands",
        robot::EventSource::Uart);
    return;
  }

  bool clockwise_turn_complete = false;
  if (context.peg_finder.state ==
      robot::PegFinderState::RotateClockwise) {
    const AutonomousImuMotionResult turn_result =
        runAutonomousImuTurn(
            context, front_left, front_right, rear_link,
            context.rear_config,
            robot::clockwiseTurnRelativeAngleDeg(
                context.peg_finder_config.clockwise_angle_deg,
                context.imu_turn_config.yaw_command_polarity),
            now_ms);
    if (turn_result != AutonomousImuMotionResult::Running &&
        turn_result != AutonomousImuMotionResult::Complete) {
      const bool timed_out =
          turn_result == AutonomousImuMotionResult::Timeout;
      const bool imu_unavailable =
          turn_result == AutonomousImuMotionResult::ImuUnavailable;
      const bool link_failed =
          turn_result == AutonomousImuMotionResult::RearLinkUnavailable ||
          turn_result == AutonomousImuMotionResult::CommandFailed;
      enterPegFinderFault(
          context, front_left, front_right, rear_link, now_ms,
          imu_unavailable
              ? robot::PegFinderFaultReason::ImuUnavailable
              : (link_failed
                     ? robot::PegFinderFaultReason::RearCommandFailed
                     : (timed_out
                            ? robot::PegFinderFaultReason::ImuTurnTimeout
                            : robot::PegFinderFaultReason::ImuTurnFailed)),
          imu_unavailable ||
                  turn_result ==
                      AutonomousImuMotionResult::MotorUnavailable
              ? robot::FaultCode::HardwareNotConfigured
              : (link_failed
                     ? robot::FaultCode::CommunicationStale
                     : (timed_out ? robot::FaultCode::SearchTimeout
                                  : robot::FaultCode::InvalidCommand)),
          imu_unavailable
              ? "PegFinder stopped: IMU unavailable during clockwise turn"
              : (link_failed
                     ? "PegFinder stopped: IMU turn rear command failed"
                     : (timed_out
                            ? "PegFinder stopped: IMU clockwise turn timed out"
                            : "PegFinder stopped: IMU clockwise turn controller fault")),
          link_failed ? robot::EventSource::Uart
                      : robot::EventSource::System);
      return;
    }
    clockwise_turn_complete =
        turn_result == AutonomousImuMotionResult::Complete;
  }

  const robot::PegFinderState previous_state = context.peg_finder.state;
  const robot::PegFinderInputs inputs{
      funnel_limit.active(), clockwise_turn_complete};
  const robot::PegFinderUpdate update = robot::updatePegFinderAutonomy(
      context.peg_finder, inputs, context.peg_finder_config, now_ms);
  if (update.state == robot::PegFinderState::Fault &&
      update.fault_reason ==
          robot::PegFinderFaultReason::FunnelLimitTimeout) {
    enterPegFinderFault(
        context, front_left, front_right, rear_link, now_ms,
        update.fault_reason, robot::FaultCode::SearchTimeout,
        "PegFinder stopped: funnel limit switch timeout",
        robot::EventSource::Motor);
    return;
  }
  if (update.state != previous_state) {
    ClawServoCommandResult servo_result =
        ClawServoCommandResult::Accepted;
    switch (update.state) {
      case robot::PegFinderState::OpenClaw1:
      case robot::PegFinderState::OpenClaw2:
      case robot::PegFinderState::OpenClaw3: {
        const std::size_t claw_index =
            update.should_open_claw_1
                ? 0U
                : (update.should_open_claw_2 ? 1U : 2U);
        servo_result =
            claws.command(claw_index, ClawServoPositionRequest::Open);
        break;
      }
      case robot::PegFinderState::WaitForStart:
      case robot::PegFinderState::RotateClockwise:
      case robot::PegFinderState::PostRotationPause:
      case robot::PegFinderState::Reverse:
      case robot::PegFinderState::PostReversePause:
      case robot::PegFinderState::Forward:
      case robot::PegFinderState::FunnelForward:
      case robot::PegFinderState::PostFunnelLimitDelay:
      case robot::PegFinderState::PostClaw1OpenDelay:
      case robot::PegFinderState::PostClaw2OpenDelay:
      case robot::PegFinderState::PostClawsOpenDelay:
      case robot::PegFinderState::FunnelReverse:
      case robot::PegFinderState::Complete:
      case robot::PegFinderState::Fault:
        break;
    }
    if (servo_result != ClawServoCommandResult::Accepted) {
      enterPegFinderFault(
          context, front_left, front_right, rear_link, now_ms,
          robot::PegFinderFaultReason::ServoCommandFailed,
          clawFaultCode(servo_result), clawServoResultReason(servo_result),
          robot::EventSource::Motor);
      return;
    }

    const char* message = nullptr;
    switch (update.state) {
      case robot::PegFinderState::PostRotationPause:
        message = "PegFinder clockwise rotation complete; pause started";
        break;
      case robot::PegFinderState::Reverse:
        message = "PegFinder backward motion started";
        break;
      case robot::PegFinderState::PostReversePause:
        message = "PegFinder backward motion complete; pause started";
        break;
      case robot::PegFinderState::Forward:
        message = "PegFinder forward motion started";
        break;
      case robot::PegFinderState::FunnelForward:
        message = "PegFinder forward motion complete; funnel forward started";
        break;
      case robot::PegFinderState::PostFunnelLimitDelay:
        message =
            "PegFinder funnel limit pressed; funnel stopped and delay started";
        break;
      case robot::PegFinderState::OpenClaw1:
      case robot::PegFinderState::OpenClaw2:
      case robot::PegFinderState::OpenClaw3:
        message = update.should_open_claw_1
                      ? "PegFinder claw 1 opened"
                      : (update.should_open_claw_2
                             ? "PegFinder claw 2 opened"
                             : "PegFinder claw 3 opened");
        break;
      case robot::PegFinderState::PostClaw1OpenDelay:
      case robot::PegFinderState::PostClaw2OpenDelay:
        break;
      case robot::PegFinderState::PostClawsOpenDelay:
        message =
            "PegFinder all claws open; funnel reverse delay started";
        break;
      case robot::PegFinderState::FunnelReverse:
        message = "PegFinder funnel reverse started";
        break;
      case robot::PegFinderState::Complete:
        message = "PegFinder complete";
        break;
      case robot::PegFinderState::WaitForStart:
      case robot::PegFinderState::RotateClockwise:
      case robot::PegFinderState::Fault:
        break;
    }
    if (message != nullptr) {
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::Motor, message);
    }
  }

  switch (update.state) {
    case robot::PegFinderState::Reverse:
    case robot::PegFinderState::Forward: {
      context.requested_funnel_command = robot::disabledMotorCommand();
      if (!sendStoppedFunnelCommand(rear_link, context.rear_config,
                                    now_ms)) {
        enterPegFinderFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::PegFinderFaultReason::FunnelCommandFailed,
            robot::FaultCode::CommunicationStale,
            "PegFinder stopped: funnel stop command failed",
            robot::EventSource::Uart);
        return;
      }
      robot::FourWheelCommand wheels{};
      if (update.should_drive_backward) {
        wheels = makePegFinderBackwardCommand(context, now_ms);
      } else {
        wheels = makePegFinderForwardCommand(context, now_ms);
      }
      if (!applyWheelCommand(context, front_left, front_right, rear_link,
                             wheels, context.rear_config, now_ms)) {
        enterPegFinderFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::PegFinderFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "PegFinder stopped: rear drive command failed",
            robot::EventSource::Uart);
        return;
      }
      break;
    }

    case robot::PegFinderState::RotateClockwise:
      context.requested_funnel_command = robot::disabledMotorCommand();
      if (!sendStoppedFunnelCommand(rear_link, context.rear_config,
                                    now_ms)) {
        enterPegFinderFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::PegFinderFaultReason::FunnelCommandFailed,
            robot::FaultCode::CommunicationStale,
            "PegFinder stopped: funnel stop command failed",
            robot::EventSource::Uart);
        return;
      }
      break;

    case robot::PegFinderState::PostRotationPause:
    case robot::PegFinderState::PostReversePause:
    case robot::PegFinderState::PostFunnelLimitDelay:
    case robot::PegFinderState::OpenClaw1:
    case robot::PegFinderState::PostClaw1OpenDelay:
    case robot::PegFinderState::OpenClaw2:
    case robot::PegFinderState::PostClaw2OpenDelay:
    case robot::PegFinderState::OpenClaw3:
    case robot::PegFinderState::PostClawsOpenDelay:
      if (!stopPegFinderOutputsOrFault(
              context, front_left, front_right, rear_link, now_ms)) {
        return;
      }
      return;

    case robot::PegFinderState::FunnelForward:
    case robot::PegFinderState::FunnelReverse: {
      if (!disableMotionActuators(context, front_left, front_right, rear_link,
                                  now_ms)) {
        enterPegFinderFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::PegFinderFaultReason::RearCommandFailed,
            robot::FaultCode::CommunicationStale,
            "PegFinder stopped: rear stop command failed before funnel",
            robot::EventSource::Uart);
        return;
      }
      const bool reversing = update.should_run_funnel_reverse;
      const float duty_magnitude =
          clampFloat(reversing
                         ? context.peg_finder_config.funnel_reverse_duty
                         : context.peg_finder_config.funnel_forward_duty,
                     0.0F, funnelMotionDutyCap());
      const float duty = reversing ? -duty_magnitude : duty_magnitude;
      context.requested_funnel_command = makeTimedMotorCommand(
          duty, now_ms, context.rear_config.remoteCommandTimeoutMs);
      if (!sendFunnelMotorCommand(rear_link,
                                  context.requested_funnel_command,
                                  context.rear_config, now_ms)) {
        enterPegFinderFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::PegFinderFaultReason::FunnelCommandFailed,
            robot::FaultCode::CommunicationStale,
            "PegFinder stopped: funnel command failed",
            robot::EventSource::Uart);
        return;
      }
      break;
    }

    case robot::PegFinderState::Complete:
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      clearFault(context);
      return;

    case robot::PegFinderState::Fault:
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      return;
    case robot::PegFinderState::WaitForStart:
      (void)stopAutonomyDriveAndFunnel(
          context, front_left, front_right, rear_link, now_ms);
      return;
  }

  context.last_command_ms = now_ms;
  context.command_deadman_armed = true;
  context.mode_expires_at_ms = 0U;
}

void enterSolarPanelAligned(RuntimeContext& context,
                            robot::IMotorOutput& front_left,
                            robot::IMotorOutput& front_right,
                            RearCommandLink& rear_link,
                            const robot::Milliseconds now_ms) {
  stopAutonomousImuStrafe(context);
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  enterSolarPanelAutonomyState(
      context, robot::SolarPanelAutonomyState::SolarBeaconAligned, now_ms);
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::System, "solar beacon aligned");
}

void enterSolarPanelContacted(RuntimeContext& context,
                              robot::IMotorOutput& front_left,
                              robot::IMotorOutput& front_right,
                              RearCommandLink& rear_link,
                              const robot::Milliseconds now_ms) {
  stopAutonomousImuStrafe(context);
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  enterSolarPanelAutonomyState(
      context, robot::SolarPanelAutonomyState::SolarPanelContacted, now_ms);
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::System,
           "solar panel limit switches contacted");
}

void enterSolarRearLineReacquired(RuntimeContext& context,
                                  robot::IMotorOutput& front_left,
                                  robot::IMotorOutput& front_right,
                                  RearCommandLink& rear_link,
                                  const robot::Milliseconds now_ms) {
  stopAutonomousImuStrafe(context);
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  enterSolarPanelAutonomyState(
      context, robot::SolarPanelAutonomyState::RearLineReacquired, now_ms);
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Line,
           "solar run complete: rear line reacquired");
}

void enterSolarPanelSearchFault(
    RuntimeContext& context, robot::IMotorOutput& front_left,
    robot::IMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const robot::SolarPanelFaultReason reason,
    const robot::FaultCode fault_code, const char* message,
    const robot::EventSource source) {
  stopAutonomousImuStrafe(context);
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  enterSolarPanelAutonomyState(
      context, robot::SolarPanelAutonomyState::SolarSearchFault, now_ms,
      reason);
  setFault(context, fault_code, message);
  logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
}

void enterSolarImuStrafeFault(
    RuntimeContext& context, DualPwmMotorOutput& front_left,
    DualPwmMotorOutput& front_right, RearCommandLink& rear_link,
    const AutonomousImuMotionResult result,
    const robot::Milliseconds now_ms) {
  const bool imu_unavailable =
      result == AutonomousImuMotionResult::ImuUnavailable;
  const bool link_failed =
      result == AutonomousImuMotionResult::RearLinkUnavailable ||
      result == AutonomousImuMotionResult::CommandFailed;
  enterSolarPanelSearchFault(
      context, front_left, front_right, rear_link, now_ms,
      imu_unavailable
          ? robot::SolarPanelFaultReason::ImuUnavailable
          : (link_failed
                 ? robot::SolarPanelFaultReason::RearLinkStale
                 : robot::SolarPanelFaultReason::ImuStrafeFailed),
      imu_unavailable ||
              result == AutonomousImuMotionResult::MotorUnavailable
          ? robot::FaultCode::HardwareNotConfigured
          : (link_failed ? robot::FaultCode::CommunicationStale
                         : robot::FaultCode::InvalidCommand),
      imu_unavailable
          ? "solar strafe stopped: IMU unavailable"
          : (link_failed
                 ? "solar strafe stopped: rear command failed"
                 : "solar strafe stopped: IMU heading hold failed"),
      link_failed ? robot::EventSource::Uart
                  : robot::EventSource::System);
}

void runSolarPanelAutonomy(RuntimeContext& context,
                           DigitalFrontLineSensorReader& sensors,
                           DualPwmMotorOutput& front_left,
                           DualPwmMotorOutput& front_right,
                           RearCommandLink& rear_link,
                           const robot::Milliseconds now_ms) {
  const bool imu_strafe_phase =
      context.autonomous_state ==
          robot::SolarPanelAutonomyState::StrafeRightToSolarPanel ||
      context.autonomous_state ==
          robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry ||
      context.autonomous_state ==
          robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel ||
      context.autonomous_state ==
          robot::SolarPanelAutonomyState::StrafeLeftToRearLine;
  if (imu_strafe_phase) {
    const AutonomousImuRecoveryUpdate recovery_update =
        serviceAutonomousImuRecovery(
            context, context.imu_strafe_recovery, false, front_left,
            front_right, rear_link, context.config, now_ms);
    context.autonomous_state_entered_at_ms +=
        recovery_update.timer_adjustment_ms;
    if (recovery_update.result ==
        AutonomousImuRecoveryResult::Paused) {
      return;
    }
    if (recovery_update.result ==
            AutonomousImuRecoveryResult::TimedOut ||
        recovery_update.result ==
            AutonomousImuRecoveryResult::StopCommandFailed) {
      enterSolarImuStrafeFault(
          context, front_left, front_right, rear_link,
          recovery_update.result ==
                  AutonomousImuRecoveryResult::TimedOut
              ? AutonomousImuMotionResult::ImuUnavailable
              : AutonomousImuMotionResult::CommandFailed,
          now_ms);
      return;
    }
  }

  const robot::Milliseconds time_in_state_ms =
      elapsedSince(now_ms, context.autonomous_state_entered_at_ms);

  switch (context.autonomous_state) {
    case robot::SolarPanelAutonomyState::WaitForStart:
      disableMotionActuators(context, front_left, front_right, rear_link,
                             now_ms);
      if (!context.solar_start_requested) {
        return;
      }
      context.solar_start_requested = false;
      robot::resetSolarBeaconDetectorState(context.solar_detector);
      context.last_solar_detector_update = {};
      if (!solarPanelStartRequirementsMet(sensors, front_left, front_right,
                                          rear_link, now_ms, context)) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::HardwareNotReady,
            robot::FaultCode::HardwareNotConfigured,
            "solar start rejected: hardware, rear line sensors, or limit switches incomplete",
            robot::EventSource::System);
        return;
      }
      robot::startLineFollower(context.follower_state, now_ms);
      context.last_command_ms = now_ms;
      context.command_deadman_armed = true;
      context.mode_expires_at_ms = 0U;
      clearFault(context);
      enterSolarPanelAutonomyState(
          context, robot::SolarPanelAutonomyState::LineFollowToSolar,
          now_ms);
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::Line, "solar line follow started");
      return;

    case robot::SolarPanelAutonomyState::LineFollowToSolar: {
      const bool remote_fresh = rear_link.remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(context.config));
      if (!sensors.configured() || !rear_link.configured()) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::HardwareNotReady,
            robot::FaultCode::HardwareNotConfigured,
            "solar line follow stopped: hardware invalid",
            robot::EventSource::Line);
        return;
      }
      if (!remote_fresh) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::RearLinkStale,
            robot::FaultCode::CommunicationStale,
            "solar line follow stopped: rear link unhealthy",
            robot::EventSource::Uart);
        return;
      }

      const robot::Esp1StatusReport& esp1 = rear_link.latestStatus();
      if (esp1.fault_active &&
          esp1.fault_code == robot::FaultCode::CommunicationStale) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::RearLinkStale,
            robot::FaultCode::CommunicationStale,
            "solar line follow stopped: ESP1 reported stale commands",
            robot::EventSource::Uart);
        return;
      }
      const bool detection_permitted =
          time_in_state_ms >= context.solar_config.ignore_after_start_ms;
      const robot::SolarPanelAutonomyConfig active_solar_config =
          activeSolarPanelConfig(context, esp1.ir_selected_frequency_hz);
      context.last_solar_detector_update =
          robot::updateSolarBeaconDetector(
              context.solar_detector, esp1.ir_selected_amplitude,
              active_solar_config, now_ms, detection_permitted);
      if (context.last_solar_detector_update.beacon_detected) {
        enterSolarPanelAligned(context, front_left, front_right, rear_link,
                               now_ms);
        return;
      }
      if (time_in_state_ms >= context.solar_config.search_timeout_ms) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::SearchTimeout,
            robot::FaultCode::SearchTimeout,
            "solar search timeout before beacon confirmation",
            robot::EventSource::System);
        return;
      }

      const bool left_black = context.last_line_observation.left_black;
      const bool right_black = context.last_line_observation.right_black;
      const robot::LineFollowerConfig active_line_config =
          activeSolarLineFollowerConfig(context, time_in_state_ms);
      context.last_update = robot::updateLineFollower(
          context.follower_state, left_black, right_black, active_line_config,
          now_ms);
      if (!applyWheelCommand(context, front_left, front_right, rear_link,
                             context.last_update.wheel_command,
                             active_line_config, now_ms)) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::RearLinkStale,
            robot::FaultCode::CommunicationStale,
            "solar line follow stopped: rear command send failed",
            robot::EventSource::Uart);
        return;
      }
      context.last_command_ms = now_ms;
      if (!context.follower_state.enabled &&
          !context.last_update.observation.safe_to_drive) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::LineLost,
            robot::FaultCode::InvalidCommand,
            "solar line follow stopped: line lost without history",
            robot::EventSource::Line);
        return;
      }
      printTelemetry(context, sensors, rear_link, now_ms);
      return;
    }

    case robot::SolarPanelAutonomyState::SolarBeaconAligned:
      disableMotionActuators(context, front_left, front_right, rear_link,
                             now_ms);
      if (!solarPanelLimitSwitchesReady(rear_link, now_ms, context)) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::HardwareNotReady,
            robot::FaultCode::HardwareNotConfigured,
            "solar contact rejected: limit switch status unavailable",
            robot::EventSource::System);
        return;
      }
      if (solarPanelLimitSwitchesAllHit(rear_link.latestStatus())) {
        enterSolarPanelContacted(context, front_left, front_right, rear_link,
                                 now_ms);
        return;
      }
      if (time_in_state_ms <
          context.solar_contact_config.strafe_start_delay_ms) {
        printTelemetry(context, sensors, rear_link, now_ms);
        return;
      }
      enterSolarPanelAutonomyState(
          context,
          robot::SolarPanelAutonomyState::StrafeRightToSolarPanel, now_ms);
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::System,
               "solar panel right strafe started");
      return;

    case robot::SolarPanelAutonomyState::StrafeRightToSolarPanel:
    case robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry:
    case robot::SolarPanelAutonomyState::MoveForwardForSolarRetry:
    case robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel:
    case robot::SolarPanelAutonomyState::SolarPanelContacted:
    case robot::SolarPanelAutonomyState::MoveForwardAfterSolarContact:
    case robot::SolarPanelAutonomyState::
        WaitBeforeStrafeLeftToRearLine:
    case robot::SolarPanelAutonomyState::StrafeLeftToRearLine:
    case robot::SolarPanelAutonomyState::
        BackwardLineFollowAfterRearDetection: {
      if (!rear_link.remoteStatusFresh(now_ms,
                                       remoteStatusTimeoutMs(context.config))) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::RearLinkStale,
            robot::FaultCode::CommunicationStale,
            "solar contact stopped: rear link unhealthy",
            robot::EventSource::Uart);
        return;
      }
      const robot::Esp1StatusReport& esp1 = rear_link.latestStatus();
      if (esp1.fault_active &&
          esp1.fault_code == robot::FaultCode::CommunicationStale) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::RearLinkStale,
            robot::FaultCode::CommunicationStale,
            "solar contact stopped: ESP1 reported stale commands",
            robot::EventSource::Uart);
        return;
      }
      if (!esp1.solar_panel_limit_switches_configured) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::HardwareNotReady,
            robot::FaultCode::HardwareNotConfigured,
            "solar contact stopped: limit switches not configured",
            robot::EventSource::System);
        return;
      }
      if (!rear_link.rearLineSnapshotFresh(
              now_ms, context.rear_config.remoteCommandTimeoutMs)) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::RearLinkStale,
            robot::FaultCode::CommunicationStale,
            "solar contact stopped: rear line sensor data stale",
            robot::EventSource::Uart);
        return;
      }
      if (!rear_link.latestRearLineSnapshot().configured) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::HardwareNotReady,
            robot::FaultCode::HardwareNotConfigured,
            "solar contact stopped: rear line sensors not configured",
            robot::EventSource::Line);
        return;
      }

      const bool back_hit =
          solarPanelLimitSwitchHit(esp1.solar_limit_back_right_high);
      const bool front_hit =
          solarPanelLimitSwitchHit(esp1.solar_limit_front_right_high);
      const robot::SolarPanelContactSequenceUpdate sequence_update =
          robot::updateSolarPanelContactSequence(
              context.autonomous_state, front_hit, back_hit,
              context.last_rear_line_observation.left_black ||
                  context.last_rear_line_observation.right_black,
              time_in_state_ms, context.solar_contact_config);
      if (sequence_update.next_state ==
              robot::SolarPanelAutonomyState::SolarPanelContacted &&
          context.autonomous_state !=
              robot::SolarPanelAutonomyState::SolarPanelContacted) {
        enterSolarPanelContacted(context, front_left, front_right, rear_link,
                                 now_ms);
        return;
      }
      if (sequence_update.next_state ==
          robot::SolarPanelAutonomyState::RearLineReacquired) {
        enterSolarRearLineReacquired(context, front_left, front_right,
                                     rear_link, now_ms);
        return;
      }
      if (sequence_update.next_state ==
          robot::SolarPanelAutonomyState::SolarSearchFault) {
        const char* timeout_message =
            context.autonomous_state ==
                    robot::SolarPanelAutonomyState::
                        RetryStrafeRightToSolarPanel
                ? "solar contact retry timeout before both limit switches hit"
                : "solar contact timeout before both limit switches hit";
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::LimitSwitchTimeout,
            robot::FaultCode::SearchTimeout, timeout_message,
            robot::EventSource::System);
        return;
      }
      if (sequence_update.transitioned) {
        stopAutonomousImuStrafe(context);
        disableMotionActuators(context, front_left, front_right, rear_link,
                               now_ms);
        enterSolarPanelAutonomyState(context, sequence_update.next_state,
                                     now_ms);
        if (sequence_update.next_state ==
            robot::SolarPanelAutonomyState::
                BackwardLineFollowAfterRearDetection) {
          robot::startLineFollower(context.follower_state, now_ms);
          logEvent(context, now_ms, robot::EventSeverity::Info,
                   robot::EventSource::Line,
                   "solar rear tape detected; backward line-follow PID started");
          printTelemetry(context, sensors, rear_link, now_ms);
          return;
        }
        const char* message = "solar contact adjustment state changed";
        if (sequence_update.next_state ==
            robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry) {
          message = "solar contact front-only: left adjustment started";
        } else if (sequence_update.next_state ==
                   robot::SolarPanelAutonomyState::
                       MoveForwardForSolarRetry) {
          message = "solar contact forward adjustment started";
        } else if (sequence_update.next_state ==
                   robot::SolarPanelAutonomyState::
                       RetryStrafeRightToSolarPanel) {
          message = "solar panel right strafe retry started";
        } else if (sequence_update.next_state ==
                   robot::SolarPanelAutonomyState::
                       MoveForwardAfterSolarContact) {
          message = "solar post-contact forward motion started";
        } else if (sequence_update.next_state ==
                   robot::SolarPanelAutonomyState::StrafeLeftToRearLine) {
          message = "solar rear-line reacquisition strafe started";
        } else if (sequence_update.next_state ==
                   robot::SolarPanelAutonomyState::
                       WaitBeforeStrafeLeftToRearLine) {
          message = "solar post-forward left-strafe delay started";
        } else if (sequence_update.next_state ==
                   robot::SolarPanelAutonomyState::
                       BackwardLineFollowAfterRearDetection) {
          message = "solar rear tape detected; backward line following started";
        }
        logEvent(context, now_ms, robot::EventSeverity::Info,
                 robot::EventSource::System, message);
        const bool zero_duration_adjustment =
            (context.autonomous_state ==
                 robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry &&
             context.solar_contact_config.retry_strafe_left_duration_ms ==
                 0U) ||
            (context.autonomous_state ==
                 robot::SolarPanelAutonomyState::MoveForwardForSolarRetry &&
             context.solar_contact_config.retry_forward_duration_ms == 0U) ||
            (context.autonomous_state ==
                 robot::SolarPanelAutonomyState::
                     MoveForwardAfterSolarContact &&
             context.solar_contact_config.post_contact_forward_duration_ms ==
                 0U);
        if (zero_duration_adjustment) {
          printTelemetry(context, sensors, rear_link, now_ms);
          return;
        }
      }

      const bool imu_strafe_state =
          context.autonomous_state ==
              robot::SolarPanelAutonomyState::
                  StrafeRightToSolarPanel ||
          context.autonomous_state ==
              robot::SolarPanelAutonomyState::
                  StrafeLeftForSolarRetry ||
          context.autonomous_state ==
              robot::SolarPanelAutonomyState::
                  RetryStrafeRightToSolarPanel ||
          context.autonomous_state ==
              robot::SolarPanelAutonomyState::StrafeLeftToRearLine;
      if (imu_strafe_state) {
        int strafe_direction = 0;
        float strafe_duty = 0.0F;
        if (context.autonomous_state ==
            robot::SolarPanelAutonomyState::StrafeLeftToRearLine) {
          strafe_direction = -1;
          strafe_duty =
              context.solar_contact_config.line_reacquire_strafe_duty;
        } else if (context.autonomous_state ==
                   robot::SolarPanelAutonomyState::
                       StrafeRightToSolarPanel) {
          strafe_direction = 1;
          strafe_duty =
              context.solar_contact_config.initial_strafe_right_duty;
        } else if (context.autonomous_state ==
                   robot::SolarPanelAutonomyState::
                       StrafeLeftForSolarRetry) {
          strafe_direction = -1;
          strafe_duty =
              context.solar_contact_config.retry_strafe_left_duty;
        } else {
          strafe_direction = 1;
          strafe_duty =
              context.solar_contact_config.retry_strafe_right_duty;
        }
        const AutonomousImuMotionResult result =
            runAutonomousImuStrafe(
                context, front_left, front_right, rear_link,
                context.config, strafe_direction, strafe_duty, now_ms);
        if (result != AutonomousImuMotionResult::Running) {
          enterSolarImuStrafeFault(
              context, front_left, front_right, rear_link, result,
              now_ms);
          return;
        }
        printTelemetry(context, sensors, rear_link, now_ms);
        return;
      }

      stopAutonomousImuStrafe(context);
      if (context.autonomous_state ==
          robot::SolarPanelAutonomyState::
              BackwardLineFollowAfterRearDetection) {
        const robot::LineFollowerConfig reverse_config =
            reverseRearLineFollowerConfig(context);
        context.last_rear_update = robot::updateLineFollower(
            context.follower_state,
            context.last_rear_line_observation.left_black,
            context.last_rear_line_observation.right_black,
            reverse_config, now_ms);
        if (!context.follower_state.enabled &&
            !context.last_rear_update.observation.safe_to_drive) {
          enterSolarPanelSearchFault(
              context, front_left, front_right, rear_link, now_ms,
              robot::SolarPanelFaultReason::LineLost,
              robot::FaultCode::InvalidCommand,
              "solar backward line follow stopped: rear line lost without history",
              robot::EventSource::Line);
          return;
        }
        if (!applyWheelCommand(
                context, front_left, front_right, rear_link,
                context.last_rear_update.wheel_command, reverse_config,
                now_ms)) {
          enterSolarPanelSearchFault(
              context, front_left, front_right, rear_link, now_ms,
              robot::SolarPanelFaultReason::RearLinkStale,
              robot::FaultCode::CommunicationStale,
              "solar backward line follow stopped: rear command failed",
              robot::EventSource::Uart);
          return;
        }
        context.last_command_ms = now_ms;
        context.command_deadman_armed = true;
        context.mode_expires_at_ms = 0U;
        printTelemetry(context, sensors, rear_link, now_ms);
        return;
      }
      robot::FourWheelCommand wheels{};
      switch (context.autonomous_state) {
        case robot::SolarPanelAutonomyState::SolarPanelContacted:
        case robot::SolarPanelAutonomyState::
            WaitBeforeStrafeLeftToRearLine:
          wheels = robot::disabledFourWheelCommand();
          break;
        case robot::SolarPanelAutonomyState::MoveForwardForSolarRetry:
          wheels = makeSolarForwardCommand(context, now_ms);
          break;
        case robot::SolarPanelAutonomyState::MoveForwardAfterSolarContact:
          wheels = makeSolarPostContactForwardCommand(context, now_ms);
          break;
        default:
          wheels = robot::disabledFourWheelCommand();
          break;
      }
      if (!applyWheelCommand(context, front_left, front_right, rear_link,
                             wheels, context.config, now_ms)) {
        enterSolarPanelSearchFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::SolarPanelFaultReason::RearLinkStale,
            robot::FaultCode::CommunicationStale,
            "solar contact stopped: rear command send failed",
            robot::EventSource::Uart);
        return;
      }
      context.last_command_ms = now_ms;
      context.command_deadman_armed = true;
      context.mode_expires_at_ms = 0U;
      printTelemetry(context, sensors, rear_link, now_ms);
      return;
    }

    case robot::SolarPanelAutonomyState::RearLineReacquired:
    case robot::SolarPanelAutonomyState::SolarSearchFault:
      disableMotionActuators(context, front_left, front_right, rear_link,
                             now_ms);
      return;
  }
}

void enterTimeTrialFault(RuntimeContext& context,
                         robot::IMotorOutput& front_left,
                         robot::IMotorOutput& front_right,
                         RearCommandLink& rear_link,
                         const robot::Milliseconds now_ms,
                         const robot::FaultCode fault_code,
                         const char* message,
                         const robot::EventSource source) {
  stopAutonomousImuStrafe(context);
  (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                   rear_link, now_ms);
  robot::failTimeTrialAutonomy(context.time_trial, now_ms);
  setFault(context, fault_code, message);
  logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
}

bool stopTimeTrialOutputsOrFault(RuntimeContext& context,
                                 robot::IMotorOutput& front_left,
                                 robot::IMotorOutput& front_right,
                                 RearCommandLink& rear_link,
                                 const robot::Milliseconds now_ms) {
  if (stopAutonomyDriveAndFunnel(context, front_left, front_right, rear_link,
                                 now_ms)) {
    return true;
  }
  enterTimeTrialFault(
      context, front_left, front_right, rear_link, now_ms,
      robot::FaultCode::CommunicationStale,
      "Time Trial stopped: drive or funnel stop command failed",
      robot::EventSource::Uart);
  return false;
}

void runTimeTrial(RuntimeContext& context,
                  DigitalFrontLineSensorReader& sensors,
                  DualPwmMotorOutput& front_left,
                  DualPwmMotorOutput& front_right,
                  RearCommandLink& rear_link, ClawServoBank& claws,
                  const DigitalActiveHighLimitSwitch& funnel_limit,
                  robot::esp2::StepperAxis& stepper,
                  const robot::Milliseconds now_ms) {
  if (context.time_trial.state == robot::TimeTrialState::WaitForStart) {
    if (!stopTimeTrialOutputsOrFault(context, front_left, front_right,
                                     rear_link, now_ms)) {
      return;
    }
    if (!context.time_trial_start_requested) {
      return;
    }
    context.time_trial_start_requested = false;
    if (!timeTrialStartRequirementsMet(
            sensors, front_left, front_right, rear_link, claws,
            funnel_limit, stepper, now_ms, context)) {
      enterTimeTrialFault(
          context, front_left, front_right, rear_link, now_ms,
          robot::FaultCode::HardwareNotConfigured,
          "Time Trial start rejected: configure all three modes, servos, stepper, sensors, and link",
          robot::EventSource::System);
      return;
    }

    (void)robot::startTimeTrialAutonomy(context.time_trial, now_ms);
    context.solar_start_requested = true;
    clearFault(context);
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::System,
             "Time Trial autonomous solar started");
    return;
  }

  if (context.time_trial.state ==
      robot::TimeTrialState::SolarToTowerStrafeRight) {
    const AutonomousImuRecoveryUpdate recovery_update =
        serviceAutonomousImuRecovery(
            context, context.imu_strafe_recovery, false, front_left,
            front_right, rear_link, context.rear_config, now_ms);
    context.time_trial.state_entered_at_ms +=
        recovery_update.timer_adjustment_ms;
    if (recovery_update.result ==
        AutonomousImuRecoveryResult::Paused) {
      return;
    }
    if (recovery_update.result ==
        AutonomousImuRecoveryResult::TimedOut) {
      enterTimeTrialFault(
          context, front_left, front_right, rear_link, now_ms,
          robot::FaultCode::HardwareNotConfigured,
          "Time Trial stopped: IMU I2C recovery timed out during transition strafe",
          robot::EventSource::System);
      return;
    }
    if (recovery_update.result ==
        AutonomousImuRecoveryResult::StopCommandFailed) {
      enterTimeTrialFault(
          context, front_left, front_right, rear_link, now_ms,
          robot::FaultCode::CommunicationStale,
          "Time Trial stopped: failed to hold wheel outputs disabled during IMU recovery",
          robot::EventSource::Uart);
      return;
    }
  }

  if (context.time_trial.state == robot::TimeTrialState::AutonomousSolar) {
    runSolarPanelAutonomy(context, sensors, front_left, front_right,
                          rear_link, now_ms);
  } else if (context.time_trial.state ==
             robot::TimeTrialState::TowerPieces) {
    runTowerPiecesAutonomy(context, front_left, front_right, rear_link, claws,
                           stepper, now_ms);
  } else if (context.time_trial.state ==
             robot::TimeTrialState::PegFinder) {
    runPegFinder(context, front_left, front_right, rear_link, claws,
                 funnel_limit, now_ms);
  } else if (context.time_trial.state ==
                 robot::TimeTrialState::PostSolarDelay ||
             context.time_trial.state ==
                 robot::TimeTrialState::PostTowerDelay) {
    if (!stopTimeTrialOutputsOrFault(context, front_left, front_right,
                                     rear_link, now_ms)) {
      return;
    }
  }

  const robot::TimeTrialState previous_state = context.time_trial.state;
  robot::TimeTrialInputs inputs{};
  inputs.solar_complete =
      context.autonomous_state ==
      robot::SolarPanelAutonomyState::RearLineReacquired;
  inputs.solar_line_follow_ready =
      context.autonomous_state ==
      robot::SolarPanelAutonomyState::
          BackwardLineFollowAfterRearDetection;
  inputs.solar_fault =
      context.autonomous_state ==
      robot::SolarPanelAutonomyState::SolarSearchFault;
  inputs.tower_pieces_complete =
      context.tower_pieces.state == robot::TowerPiecesState::Complete;
  inputs.tower_pieces_fault =
      context.tower_pieces.state == robot::TowerPiecesState::Fault;
  inputs.peg_finder_complete =
      context.peg_finder.state == robot::PegFinderState::Complete;
  inputs.peg_finder_fault =
      context.peg_finder.state == robot::PegFinderState::Fault;
  const robot::TimeTrialUpdate update = robot::updateTimeTrialAutonomy(
      context.time_trial, inputs, context.time_trial_config, now_ms);

  if (update.state == robot::TimeTrialState::Fault) {
    (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                     rear_link, now_ms);
    if (previous_state != robot::TimeTrialState::Fault) {
      logEvent(context, now_ms, robot::EventSeverity::Fault,
               robot::EventSource::System,
               "Time Trial stopped because an included mode faulted");
    }
    return;
  }

  if (update.should_start_tower_pieces) {
    if (update.should_handoff_solar_line_follow) {
      if (!towerPiecesStartRequirementsMet(
              front_left, front_right, rear_link, claws, stepper, now_ms,
              context)) {
        enterTimeTrialFault(
            context, front_left, front_right, rear_link, now_ms,
            robot::FaultCode::HardwareNotConfigured,
            "Time Trial line-follow handoff rejected: Tower Pieces hardware or configuration is unavailable",
            robot::EventSource::System);
        return;
      }

      stepper.stop();
      robot::startTowerPiecesAutonomy(
          context.tower_pieces,
          rear_link.latestRearLineSnapshot().side_electrical_high, now_ms);
      context.last_tower_pieces_update = {};
      context.tower_pieces_start_requested = false;
      // Solar initialized this same follower when it detected the rear tape.
      // Keep that follower instance under Tower Pieces ownership so there is
      // no second stop/start between the two line-follow phases.
      enterSolarPanelAutonomyState(
          context, robot::SolarPanelAutonomyState::RearLineReacquired,
          now_ms);
      context.last_command_ms = now_ms;
      context.command_deadman_armed = true;
      context.mode_expires_at_ms = 0U;
      clearFault(context);
      logEvent(
          context, now_ms, robot::EventSeverity::Info,
          robot::EventSource::Line,
          "Time Trial solar rear-line PID handed directly to Tower Pieces; Tower reverse line-follow duty is active");
    } else {
      context.tower_pieces_start_requested = true;
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::System,
               "Time Trial tower pieces started");
    }
  }
  if (update.should_start_peg_finder) {
    const ClawServoCommandResult claws_result =
        claws.commandAll(ClawServoPositionRequest::Closed);
    const ClawServoCommandResult winch_result =
        claws.command(kWinchServoIndex, ClawServoPositionRequest::Closed);
    if (claws_result != ClawServoCommandResult::Accepted ||
        winch_result != ClawServoCommandResult::Accepted) {
      const ClawServoCommandResult failed_result =
          claws_result != ClawServoCommandResult::Accepted
              ? claws_result
              : winch_result;
      enterTimeTrialFault(
          context, front_left, front_right, rear_link, now_ms,
          clawFaultCode(failed_result), clawServoResultReason(failed_result),
          robot::EventSource::Motor);
      return;
    }
    // Keep the closed servo PWM outputs enabled across this handoff.
    context.peg_finder_start_requested = true;
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::System,
             "Time Trial PegFinder started with servos held closed");
  }

  if (update.state == robot::TimeTrialState::SolarToTowerStrafeRight) {
    if (!rear_link.configured() ||
        !rear_link.remoteStatusFresh(
            now_ms, remoteStatusTimeoutMs(context.rear_config))) {
      enterTimeTrialFault(
          context, front_left, front_right, rear_link, now_ms,
          robot::FaultCode::CommunicationStale,
          "Time Trial transition strafe stopped: rear link unhealthy",
          robot::EventSource::Uart);
      return;
    }
    const AutonomousImuMotionResult result =
        runAutonomousImuStrafe(
            context, front_left, front_right, rear_link,
            context.rear_config, 1,
            context.imu_heading_hold_config.maximum_strafe_duty, now_ms);
    if (result != AutonomousImuMotionResult::Running) {
      const bool imu_unavailable =
          result == AutonomousImuMotionResult::ImuUnavailable;
      const bool link_failed =
          result == AutonomousImuMotionResult::RearLinkUnavailable ||
          result == AutonomousImuMotionResult::CommandFailed;
      enterTimeTrialFault(
          context, front_left, front_right, rear_link, now_ms,
          imu_unavailable ||
                  result ==
                      AutonomousImuMotionResult::MotorUnavailable
              ? robot::FaultCode::HardwareNotConfigured
              : (link_failed ? robot::FaultCode::CommunicationStale
                             : robot::FaultCode::InvalidCommand),
          imu_unavailable
              ? "Time Trial transition strafe stopped: IMU unavailable"
              : (link_failed
                     ? "Time Trial transition strafe stopped: rear command failed"
                     : "Time Trial transition strafe stopped: IMU heading hold failed"),
          link_failed ? robot::EventSource::Uart
                      : robot::EventSource::System);
      return;
    }
  } else if (previous_state ==
                 robot::TimeTrialState::SolarToTowerStrafeRight &&
             update.state == robot::TimeTrialState::TowerPieces) {
    stopAutonomousImuStrafe(context);
    if (!stopTimeTrialOutputsOrFault(context, front_left, front_right,
                                     rear_link, now_ms)) {
      return;
    }
  }

  if (update.state != previous_state) {
    const char* message = nullptr;
    if (update.state == robot::TimeTrialState::PostSolarDelay) {
      message = "Time Trial solar complete; transition delay started";
    } else if (update.state ==
               robot::TimeTrialState::SolarToTowerStrafeRight) {
      message = "Time Trial solar-to-tower right strafe started";
    } else if (update.state == robot::TimeTrialState::PostTowerDelay) {
      message = "Time Trial tower pieces complete; PegFinder delay started";
    } else if (update.state == robot::TimeTrialState::Complete) {
      message = "Time Trial complete";
      clearFault(context);
    }
    if (message != nullptr) {
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::System, message);
    }
  }

  if (update.state == robot::TimeTrialState::Complete) {
    (void)stopAutonomyDriveAndFunnel(context, front_left, front_right,
                                     rear_link, now_ms);
  }
}

robot::FourWheelCommand makeManualDriveCommand(
    const RuntimeContext& context, const float vx, const float vy,
    const float wz, const float duty, const robot::Milliseconds now_ms) {
  robot::FourWheelCommand command =
      robot::mixOpenLoopMecanum(vx, vy, wz, duty, now_ms, kCommandTimeoutMs);
  if (context.modes.currentMode() !=
      robot::RobotTestMode::DistributedDriveTest) {
    command.back_left = robot::disabledMotorCommand();
    command.back_right = robot::disabledMotorCommand();
  }
  return command;
}

void refreshLineObservation(RuntimeContext& context,
                            DigitalFrontLineSensorReader& sensors,
                            const robot::Milliseconds now_ms) {
  const robot::FrontLineSensorSnapshot snapshot = sensors.readSnapshot(now_ms);
  const bool left_black = snapshot.left == robot::LineSample::OnTape;
  const bool right_black = snapshot.right == robot::LineSample::OnTape;
  context.last_line_observation = robot::observeDigitalLineSensors(
      left_black, right_black, context.line_sensor_last_known_side, now_ms);
  context.line_sensor_last_known_side =
      context.last_line_observation.last_known_side;
}

void refreshRearLineObservation(RuntimeContext& context,
                                const RearCommandLink& rear_link,
                                const robot::Milliseconds now_ms) {
  if (!rear_link.rearLineSnapshotFresh(
          now_ms, context.rear_config.remoteCommandTimeoutMs) ||
      !rear_link.latestRearLineSnapshot().configured) {
    context.last_rear_line_observation = {};
    context.last_rear_line_observation.timestampMs = now_ms;
    context.last_rear_line_observation.timestamp_ms = now_ms;
    context.last_rear_line_observation.observed_at_ms = now_ms;
    return;
  }

  const robot::RearLineSensorSnapshot& snapshot =
      rear_link.latestRearLineSnapshot();
  context.last_rear_line_observation =
      robot::observeRearLineSensorsForReverseTravel(
          snapshot.left_electrical_high, snapshot.right_electrical_high,
          context.rear_line_sensor_last_known_side, now_ms);
  context.rear_line_sensor_last_known_side =
      context.last_rear_line_observation.last_known_side;
}

void fillMotorTelemetry(robot::MotorTelemetry& output,
                        const DualPwmMotorOutput& motor) {
  output.desired_command_milli =
      motor.lastDesiredCommand().duty_command_milli;
  output.applied_command_milli =
      motor.lastAppliedCommand().duty_command_milli;
  output.enabled = motor.lastAppliedCommand().enabled;
  output.inverted = motor.runtimeInverted();
  output.configured = motor.configured();
}

void fillTelemetrySnapshot(const RuntimeContext& context,
                           const DigitalFrontLineSensorReader& sensors,
                           const DualPwmMotorOutput& front_left,
                           const DualPwmMotorOutput& front_right,
                           const RearCommandLink& rear_link,
                           const ClawServoBank& claws,
                           const DigitalActiveHighLimitSwitch& funnel_limit,
                           const robot::esp2::ImuAcquisitionSnapshot& imu,
                           robot::TelemetrySnapshot& snapshot,
                           const robot::Milliseconds now_ms) {
  snapshot = {};
  snapshot.uptime_ms = now_ms;
  snapshot.current_mode = context.modes.currentMode();
  snapshot.previous_mode = context.modes.previousMode();
  snapshot.enabled = context.modes.motorsMayBeCommanded();
  snapshot.fault_active = context.fault_active;
  snapshot.fault_code = context.fault_code;
  copyText(snapshot.fault_message, sizeof(snapshot.fault_message),
           context.fault_message);
  snapshot.last_command_age_ms =
      context.last_command_ms == 0U ? 0U
                                    : elapsedSince(now_ms,
                                                   context.last_command_ms);
  snapshot.deadman_remaining_ms =
      snapshot.last_command_age_ms >= kCommandTimeoutMs
          ? 0U
          : kCommandTimeoutMs - snapshot.last_command_age_ms;
  snapshot.wifi_clients =
      static_cast<std::uint8_t>(WiFi.softAPgetStationNum());
  copyText(snapshot.ip_address, sizeof(snapshot.ip_address),
           WiFi.softAPIP().toString().c_str());
  snapshot.free_heap_bytes = ESP.getFreeHeap();
  copyText(snapshot.reset_reason, sizeof(snapshot.reset_reason),
           resetReasonName(esp_reset_reason()));

  const robot::esp2::ImuState& imu_state = imu.state;
  const std::uint32_t now_us = micros();
  snapshot.imu.configured = imu_state.configured;
  snapshot.imu.initialized = imu_state.initialized;
  snapshot.imu.calibrated = imu_state.calibrated;
  snapshot.imu.data_fresh = robot::esp2::imuSnapshotFresh(
      imu, now_us, kImuFreshnessTimeoutUs);
  snapshot.imu.healthy = imu_state.healthy && snapshot.imu.data_fresh;
  snapshot.imu.acquisition_running = imu.acquisition_running;
  snapshot.imu.device_acknowledged =
      imu_state.device_acknowledged;
  snapshot.imu.runtime_configuration_valid =
      imu_state.runtime_configuration_valid;
  snapshot.imu.register_reads_use_repeated_start =
      imu_state.register_reads_use_repeated_start;
  snapshot.imu.i2c_address = imu_state.address;
  snapshot.imu.who_am_i = imu_state.who_am_i;
  snapshot.imu.sda_gpio = imu_state.sda_gpio;
  snapshot.imu.scl_gpio = imu_state.scl_gpio;
  snapshot.imu.last_wire_status = imu_state.last_wire_status;
  copyText(snapshot.imu.initialization_error,
           sizeof(snapshot.imu.initialization_error),
           robot::esp2::imuInitializationErrorName(
               imu_state.initialization_error));
  copyText(snapshot.imu.last_read_failure_reason,
           sizeof(snapshot.imu.last_read_failure_reason),
           robot::esp2::imuReadFailureReasonName(
               imu_state.last_read_failure_reason));
  copyText(snapshot.imu.disconnect_reason,
           sizeof(snapshot.imu.disconnect_reason),
           robot::esp2::imuDisconnectReason(
               imu, now_us, kImuFreshnessTimeoutUs));
  copyText(snapshot.imu.last_disconnect_reason,
           sizeof(snapshot.imu.last_disconnect_reason),
           context.last_imu_disconnect_reason[0] == '\0'
               ? "NONE"
               : context.last_imu_disconnect_reason);
  snapshot.imu.raw_gyro_z = imu_state.raw_gyro_z;
  snapshot.imu.gyro_z_bias_dps = imu_state.gyro_z_bias_dps;
  snapshot.imu.yaw_rate_dps = imu_state.yaw_rate_dps;
  snapshot.imu.heading_deg = imu_state.heading_deg;
  snapshot.imu.sample_age_ms =
      imu_state.last_successful_read_us == 0U
          ? 0U
          : static_cast<robot::Milliseconds>(
                (now_us - imu_state.last_successful_read_us) / 1000U);
  snapshot.imu.snapshot_age_ms =
      imu.published_at_us == 0U
          ? 0U
          : static_cast<robot::Milliseconds>(
                (now_us - imu.published_at_us) / 1000U);
  snapshot.imu.acquisition_duration_us =
      imu.acquisition_duration_us;
  snapshot.imu.maximum_completed_acquisition_duration_us =
      imu.maximum_completed_acquisition_duration_us;
  snapshot.imu.total_acquisition_attempts =
      imu.total_acquisition_attempts;
  snapshot.imu.last_successful_read_us =
      imu_state.last_successful_read_us;
  snapshot.imu.last_sample_interval_us =
      imu_state.last_sample_interval_us;
  snapshot.imu.last_read_failure_us =
      imu_state.last_read_failure_us;
  snapshot.imu.successful_read_count =
      imu.lifetime_successful_acquisitions;
  snapshot.imu.failed_read_count =
      imu.lifetime_failed_acquisitions;
  snapshot.imu.consecutive_failed_reads =
      imu.consecutive_acquisition_failures;
  snapshot.imu.disconnect_count = context.imu_disconnect_count;
  snapshot.imu.last_disconnect_at_ms =
      context.last_imu_disconnect_at_ms;
  snapshot.imu.acquisition_loop_interval_us =
      imu.acquisition_loop_interval_us;
  snapshot.imu.maximum_acquisition_loop_interval_us =
      imu.maximum_acquisition_loop_interval_us;
  snapshot.imu.synchronization_duration_us =
      imu.synchronization_duration_us;
  snapshot.imu.maximum_synchronization_duration_us =
      imu.maximum_synchronization_duration_us;
  snapshot.imu.wire_lock_acquire_duration_us =
      imu_state.last_wire_lock_acquire_duration_us;
  snapshot.imu.maximum_wire_lock_acquire_duration_us =
      imu_state.maximum_wire_lock_acquire_duration_us;
  snapshot.imu.measurement_read_duration_us =
      imu_state.last_measurement_read_duration_us;
  snapshot.imu.maximum_measurement_read_duration_us =
      imu_state.maximum_measurement_read_duration_us;
  snapshot.imu.successful_read_to_publication_us =
      imu.successful_read_to_publication_us;
  snapshot.imu.maximum_successful_read_to_publication_us =
      imu.maximum_successful_read_to_publication_us;
  snapshot.imu.publication_queue_duration_us =
      imu.publication_queue_duration_us;
  snapshot.imu.maximum_publication_queue_duration_us =
      imu.maximum_publication_queue_duration_us;
  snapshot.imu.successful_sample_publication_gap_us =
      imu.successful_sample_publication_gap_us;
  snapshot.imu.maximum_successful_sample_publication_gap_us =
      imu.maximum_successful_sample_publication_gap_us;
  snapshot.imu.current_observed_publication_gap_us =
      imu.last_successful_sample_published_at_us == 0U
          ? 0U
          : now_us -
                imu.last_successful_sample_published_at_us;
  snapshot.imu.maximum_observed_publication_gap_us =
      snapshot.imu.current_observed_publication_gap_us >
              context.maximum_observed_imu_publication_gap_us
          ? snapshot.imu.current_observed_publication_gap_us
          : context.maximum_observed_imu_publication_gap_us;
  snapshot.imu.publication_sequence = imu.publication_sequence;
  snapshot.imu.successful_sample_sequence =
      imu.successful_sample_sequence;
  snapshot.imu.delayed_iteration_count =
      imu.delayed_iteration_count;

  snapshot.imu_turn.configuration_valid =
      imuTurnRuntimeConfigValid(context.imu_turn_config);
  snapshot.imu_turn.active =
      robot::imuTurnActive(context.imu_turn_state);
  snapshot.imu_turn.state = context.imu_turn_state.state;
  snapshot.imu_turn.fault_reason =
      context.imu_turn_state.fault_reason;
  const ImuTurnAvailabilityFaultCapture& availability_capture =
      context.imu_turn_availability_fault;
  snapshot.imu_turn.availability_fault_capture_valid =
      availability_capture.valid;
  snapshot.imu_turn.availability_fault_latched =
      availability_capture.valid &&
      context.imu_turn_state.state == robot::ImuTurnState::Fault &&
      context.imu_turn_state.fault_reason ==
          robot::ImuTurnFaultReason::ImuUnavailable;
  snapshot.imu_turn.imu_currently_available =
      std::strcmp(robot::esp2::imuDisconnectReason(
                      imu, now_us, kImuFreshnessTimeoutUs),
                  "NONE") == 0;
  snapshot.imu_turn.captured_configured =
      availability_capture.configured;
  snapshot.imu_turn.captured_initialized =
      availability_capture.initialized;
  snapshot.imu_turn.captured_calibrated =
      availability_capture.calibrated;
  snapshot.imu_turn.captured_healthy = availability_capture.healthy;
  snapshot.imu_turn.captured_sample_valid =
      availability_capture.sample_valid;
  snapshot.imu_turn.captured_data_fresh =
      availability_capture.data_fresh;
  snapshot.imu_turn.captured_acquisition_running =
      availability_capture.acquisition_running;
  snapshot.imu_turn.captured_shared_snapshot_available =
      availability_capture.shared_snapshot_available;
  snapshot.imu_turn.captured_front_left_configured =
      availability_capture.front_left_configured;
  snapshot.imu_turn.captured_front_right_configured =
      availability_capture.front_right_configured;
  snapshot.imu_turn.captured_rear_link_configured =
      availability_capture.rear_link_configured;
  snapshot.imu_turn.captured_rear_status_available =
      availability_capture.rear_status_available;
  snapshot.imu_turn.captured_rear_status_fresh =
      availability_capture.rear_status_fresh;
  snapshot.imu_turn.captured_newest_snapshot_available =
      availability_capture.newest_snapshot_available;
  snapshot.imu_turn.captured_cached_snapshot_matches_newest =
      availability_capture.cached_snapshot_matches_newest;
  copyText(snapshot.imu_turn.availability_fault_origin,
           sizeof(snapshot.imu_turn.availability_fault_origin),
           availability_capture.origin);
  copyText(snapshot.imu_turn.captured_availability_reason,
           sizeof(snapshot.imu_turn.captured_availability_reason),
           availability_capture.reason);
  snapshot.imu_turn.availability_evaluated_at_us =
      availability_capture.evaluated_at_us;
  snapshot.imu_turn.availability_evaluated_at_ms =
      availability_capture.evaluated_at_ms;
  snapshot.imu_turn.captured_published_at_us =
      availability_capture.published_at_us;
  snapshot.imu_turn.captured_last_successful_read_us =
      availability_capture.last_successful_read_us;
  snapshot.imu_turn.captured_sample_age_us =
      availability_capture.sample_age_us;
  snapshot.imu_turn.captured_snapshot_age_us =
      availability_capture.snapshot_age_us;
  snapshot.imu_turn.captured_freshness_timeout_us =
      availability_capture.freshness_timeout_us;
  snapshot.imu_turn.captured_cached_snapshot_sequence =
      availability_capture.cached_snapshot_sequence;
  snapshot.imu_turn.captured_newest_snapshot_sequence =
      availability_capture.newest_snapshot_sequence;
  snapshot.imu_turn.captured_cached_successful_sample_sequence =
      availability_capture.cached_successful_sample_sequence;
  snapshot.imu_turn.captured_newest_successful_sample_sequence =
      availability_capture.newest_successful_sample_sequence;
  snapshot.imu_turn.captured_cached_snapshot_fetched_at_us =
      availability_capture.cached_snapshot_fetched_at_us;
  snapshot.imu_turn.captured_cached_snapshot_fetch_to_gate_us =
      availability_capture.cached_snapshot_fetch_to_gate_us;
  snapshot.imu_turn.captured_successful_sample_publication_gap_us =
      availability_capture.successful_sample_publication_gap_us;
  snapshot.imu_turn
      .captured_maximum_successful_sample_publication_gap_us =
      availability_capture
          .maximum_successful_sample_publication_gap_us;
  snapshot.imu_turn.captured_current_observed_publication_gap_us =
      availability_capture.current_observed_publication_gap_us;
  snapshot.imu_turn.captured_maximum_observed_publication_gap_us =
      availability_capture.maximum_observed_publication_gap_us;
  snapshot.imu_turn.captured_rear_last_status_received_at_ms =
      availability_capture.rear_last_status_received_at_ms;
  snapshot.imu_turn.captured_rear_status_age_ms =
      availability_capture.rear_status_age_ms;
  snapshot.imu_turn.maximum_rotation_duty =
      context.imu_turn_config.maximum_rotation_duty;
  snapshot.imu_turn.kp = context.imu_turn_config.kp;
  snapshot.imu_turn.kd = context.imu_turn_config.kd;
  snapshot.imu_turn.angle_tolerance_deg =
      context.imu_turn_config.angle_tolerance_deg;
  snapshot.imu_turn.maximum_finishing_yaw_rate_dps =
      context.imu_turn_config.maximum_finishing_yaw_rate_dps;
  snapshot.imu_turn.settling_time_ms =
      context.imu_turn_config.settling_time_ms;
  snapshot.imu_turn.timeout_ms = context.imu_turn_config.timeout_ms;
  snapshot.imu_turn.yaw_command_polarity =
      context.imu_turn_config.yaw_command_polarity;
  snapshot.imu_turn.start_heading_deg =
      context.imu_turn_state.start_heading_deg;
  snapshot.imu_turn.current_heading_deg = imu_state.heading_deg;
  snapshot.imu_turn.target_heading_deg =
      context.imu_turn_state.target_heading_deg;
  snapshot.imu_turn.relative_angle_deg =
      context.imu_turn_state.relative_angle_deg;
  snapshot.imu_turn.angle_error_deg =
      context.imu_turn_state.target_heading_deg - imu_state.heading_deg;
  snapshot.imu_turn.yaw_rate_dps = imu_state.yaw_rate_dps;
  snapshot.imu_turn.proportional_term =
      context.last_imu_turn_update.proportional_term;
  snapshot.imu_turn.damping_term =
      context.last_imu_turn_update.damping_term;
  snapshot.imu_turn.rotation_command =
      context.last_imu_turn_update.rotation_command;
  snapshot.imu_turn.elapsed_ms =
      context.last_imu_turn_update.elapsed_ms;
  snapshot.imu_turn.settling_elapsed_ms =
      context.last_imu_turn_update.settling_elapsed_ms;

  snapshot.imu_heading_hold.configuration_valid =
      imuHeadingHoldRuntimeConfigValid(
          context.imu_heading_hold_config);
  snapshot.imu_heading_hold.active =
      robot::imuHeadingHoldActive(context.imu_heading_hold_state);
  snapshot.imu_heading_hold.state =
      context.imu_heading_hold_state.state;
  snapshot.imu_heading_hold.fault_reason =
      context.imu_heading_hold_state.fault_reason;
  snapshot.imu_heading_hold.maximum_strafe_duty =
      context.imu_heading_hold_config.maximum_strafe_duty;
  snapshot.imu_heading_hold.kp =
      context.imu_heading_hold_config.kp;
  snapshot.imu_heading_hold.kd =
      context.imu_heading_hold_config.kd;
  snapshot.imu_heading_hold.maximum_yaw_correction_duty =
      context.imu_heading_hold_config.maximum_yaw_correction_duty;
  snapshot.imu_heading_hold.yaw_command_polarity =
      context.imu_heading_hold_config.yaw_command_polarity;
  snapshot.imu_heading_hold.start_heading_deg =
      context.imu_heading_hold_state.start_heading_deg;
  snapshot.imu_heading_hold.current_heading_deg =
      imu_state.heading_deg;
  snapshot.imu_heading_hold.target_heading_deg =
      context.imu_heading_hold_state.target_heading_deg;
  snapshot.imu_heading_hold.angle_error_deg =
      context.imu_heading_hold_state.target_heading_deg -
      imu_state.heading_deg;
  snapshot.imu_heading_hold.yaw_rate_dps =
      imu_state.yaw_rate_dps;
  snapshot.imu_heading_hold.proportional_term =
      context.last_imu_heading_hold_update.proportional_term;
  snapshot.imu_heading_hold.damping_term =
      context.last_imu_heading_hold_update.damping_term;
  snapshot.imu_heading_hold.yaw_correction_duty =
      context.last_imu_heading_hold_update.yaw_correction_duty;
  snapshot.imu_heading_hold.lateral_direction =
      context.imu_heading_hold_state.lateral_direction;
  snapshot.imu_heading_hold.elapsed_ms =
      context.last_imu_heading_hold_update.elapsed_ms;

  snapshot.imu_recovery.turn_paused =
      context.imu_turn_recovery.active;
  snapshot.imu_recovery.strafe_paused =
      context.imu_strafe_recovery.active;
  snapshot.imu_recovery.turn_saved_heading_deg =
      context.imu_turn_recovery.saved_heading_deg;
  snapshot.imu_recovery.strafe_saved_heading_deg =
      context.imu_strafe_recovery.saved_heading_deg;
  snapshot.imu_recovery.turn_pause_elapsed_ms =
      context.imu_turn_recovery.active
          ? elapsedSince(
                now_ms,
                context.imu_turn_recovery.pause_started_at_ms)
          : context.imu_turn_recovery.last_pause_duration_ms;
  snapshot.imu_recovery.strafe_pause_elapsed_ms =
      context.imu_strafe_recovery.active
          ? elapsedSince(
                now_ms,
                context.imu_strafe_recovery.pause_started_at_ms)
          : context.imu_strafe_recovery.last_pause_duration_ms;
  snapshot.imu_recovery.maximum_pause_ms =
      kAutonomousImuRecoveryConfig.maximum_pause_ms;
  snapshot.imu_recovery.consecutive_fresh_samples_required =
      kAutonomousImuRecoveryConfig
          .consecutive_fresh_samples_required;
  snapshot.imu_recovery.turn_consecutive_fresh_samples =
      context.imu_turn_recovery.consecutive_fresh_samples;
  snapshot.imu_recovery.strafe_consecutive_fresh_samples =
      context.imu_strafe_recovery.consecutive_fresh_samples;
  snapshot.imu_recovery.turn_pause_count =
      context.imu_turn_recovery.pause_count;
  snapshot.imu_recovery.strafe_pause_count =
      context.imu_strafe_recovery.pause_count;
  const std::uint64_t total_imu_paused_ms =
      static_cast<std::uint64_t>(
          context.imu_turn_recovery.total_paused_ms) +
      static_cast<std::uint64_t>(
          context.imu_strafe_recovery.total_paused_ms);
  snapshot.imu_recovery.total_paused_ms =
      total_imu_paused_ms >
              std::numeric_limits<robot::Milliseconds>::max()
          ? std::numeric_limits<robot::Milliseconds>::max()
          : static_cast<robot::Milliseconds>(
                total_imu_paused_ms);

  const bool time_trial_tower_active =
      context.modes.currentMode() == robot::RobotTestMode::TimeTrial &&
      context.time_trial.state == robot::TimeTrialState::TowerPieces;
  const bool tower_line_config_active =
      context.modes.currentMode() ==
          robot::RobotTestMode::AutonomousTowerPieces ||
      time_trial_tower_active;
  const bool rear_line_following =
      (context.modes.currentMode() ==
           robot::RobotTestMode::RearLineFollowTest ||
       tower_line_config_active) &&
      context.follower_state.enabled;
  const bool front_line_following =
      context.follower_state.enabled && !rear_line_following;
  const robot::LineObservation& observation =
      front_line_following ? context.last_update.observation
                           : context.last_line_observation;
  snapshot.lsfl_raw_level = sensors.lastLeftLevel();
  snapshot.lsfr_raw_level = sensors.lastRightLevel();
  snapshot.lsfl_black = observation.left_black;
  snapshot.lsfr_black = observation.right_black;
  snapshot.line_error = observation.error;
  snapshot.line_visible = observation.line_visible;
  snapshot.line_has_history = observation.hasHistory;
  snapshot.last_known_line_side = observation.last_known_side;
  snapshot.line_follower_enabled = front_line_following;

  const robot::LineObservation& rear_observation =
      rear_line_following ? context.last_rear_update.observation
                          : context.last_rear_line_observation;
  snapshot.rear_line_data_fresh = rear_link.rearLineSnapshotFresh(
      now_ms, context.rear_config.remoteCommandTimeoutMs);
  if (rear_link.rearLineSnapshotAvailable()) {
    const robot::RearLineSensorSnapshot& rear_snapshot =
        rear_link.latestRearLineSnapshot();
    snapshot.rear_line_configured = rear_snapshot.configured;
    snapshot.rear_line_sequence = rear_link.lastRearLineSequence();
    snapshot.rear_line_sample_age_ms =
        elapsedSince(now_ms, rear_link.lastRearLineReceivedAtMs());
    snapshot.rear_line_captured_at_ms = rear_snapshot.captured_at_ms;
    if (snapshot.rear_line_data_fresh && rear_snapshot.configured) {
      snapshot.lsbl_raw_level = rear_snapshot.left_electrical_high ? 1 : 0;
      snapshot.lsbr_raw_level = rear_snapshot.right_electrical_high ? 1 : 0;
      snapshot.lsbl_black = rear_snapshot.left_electrical_high;
      snapshot.lsbr_black = rear_snapshot.right_electrical_high;
    }
  }
  snapshot.rear_line_error = rear_observation.error;
  snapshot.rear_line_visible = rear_observation.line_visible;
  snapshot.rear_line_has_history = rear_observation.hasHistory;
  snapshot.rear_last_known_line_side = rear_observation.last_known_side;
  snapshot.rear_line_follower_enabled = rear_line_following;
  snapshot.rear_logical_left_black = rear_observation.left_black;
  snapshot.rear_logical_right_black = rear_observation.right_black;

  const robot::LineFollowerConfig reverse_config =
      tower_line_config_active
          ? towerPiecesLineFollowerConfig(
                context,
                context.tower_pieces_config.reverse_line_duty)
          : reverseRearLineFollowerConfig(context);
  snapshot.rear_kp = context.rear_config.kp;
  snapshot.rear_ki = context.rear_config.ki;
  snapshot.rear_kd = context.rear_config.kd;
  snapshot.rear_base_duty = std::fabs(context.rear_config.baseDuty);
  snapshot.rear_effective_base_duty = reverse_config.baseDuty;
  snapshot.rear_maximum_duty = context.rear_config.maxDuty;
  snapshot.rear_maximum_correction = context.rear_config.maxCorrection;
  snapshot.rear_integral_limit = context.rear_config.integralLimit;
  snapshot.rear_derivative_limit = context.rear_config.derivativeLimit;
  snapshot.rear_derivative_filter_alpha =
      context.rear_config.derivativeFilterAlpha;
  snapshot.rear_steering_polarity = context.rear_config.steeringPolarity;
  snapshot.rear_control_period_ms = context.rear_config.controlPeriodMs;
  snapshot.rear_remote_command_timeout_ms =
      context.rear_config.remoteCommandTimeoutMs;
  snapshot.rear_line_telemetry_enabled =
      context.rear_config.telemetryEnabled;
  snapshot.rear_pid_p_term =
      context.last_rear_update.pid_terms.proportional_term;
  snapshot.rear_pid_i_term =
      context.last_rear_update.pid_terms.integral_term;
  snapshot.rear_pid_d_term =
      context.last_rear_update.pid_terms.derivative_term;
  snapshot.rear_pid_correction =
      context.last_rear_update.pid_terms.correction;

  snapshot.kp = context.config.kp;
  snapshot.ki = context.config.ki;
  snapshot.kd = context.config.kd;
  snapshot.base_duty = context.config.baseDuty;
  snapshot.maximum_duty = context.config.maxDuty;
  snapshot.maximum_correction = context.config.maxCorrection;
  snapshot.integral_limit = context.config.integralLimit;
  snapshot.derivative_limit = context.config.derivativeLimit;
  snapshot.derivative_filter_alpha = context.config.derivativeFilterAlpha;
  snapshot.steering_polarity = context.config.steeringPolarity;
  snapshot.control_period_ms = context.config.controlPeriodMs;
  snapshot.remote_command_timeout_ms = context.config.remoteCommandTimeoutMs;
  snapshot.line_telemetry_enabled = context.config.telemetryEnabled;
  snapshot.pid_p_term = context.last_update.pid_terms.proportional_term;
  snapshot.pid_i_term = context.last_update.pid_terms.integral_term;
  snapshot.pid_d_term = context.last_update.pid_terms.derivative_term;
  snapshot.pid_correction = context.last_update.pid_terms.correction;
  const std::uint32_t selected_frequency_hz =
      rear_link.statusAvailable()
          ? rear_link.latestStatus().ir_selected_frequency_hz
          : kIrBeaconFrequency1Khz;
  const robot::SolarPanelAutonomyConfig active_solar_config =
      activeSolarPanelConfig(context, selected_frequency_hz);
  snapshot.autonomous_state = context.autonomous_state;
  snapshot.autonomous_fault_reason = context.autonomous_fault_reason;
  snapshot.autonomous_time_in_state_ms =
      elapsedSince(now_ms, context.autonomous_state_entered_at_ms);
  snapshot.solar_ir_raw_amplitude =
      context.last_solar_detector_update.raw_amplitude;
  snapshot.solar_ir_filtered_amplitude =
      context.last_solar_detector_update.filtered_amplitude;
  snapshot.solar_ir_detection_threshold =
      active_solar_config.detection_threshold;
  snapshot.solar_ir_release_threshold =
      active_solar_config.release_threshold;
  snapshot.solar_ir_detection_threshold_1khz =
      context.solar_thresholds.detect_1khz;
  snapshot.solar_ir_release_threshold_1khz =
      context.solar_thresholds.release_1khz;
  snapshot.solar_ir_detection_threshold_10khz =
      context.solar_thresholds.detect_10khz;
  snapshot.solar_ir_release_threshold_10khz =
      context.solar_thresholds.release_10khz;
  snapshot.solar_ir_confirmation_progress_ms =
      context.last_solar_detector_update.confirmation_progress_ms;
  snapshot.solar_ir_confirmation_time_ms =
      context.solar_config.confirmation_time_ms;
  snapshot.solar_ir_filter_alpha = context.solar_config.filter_alpha;
  snapshot.solar_ir_ignore_after_start_ms =
      context.solar_config.ignore_after_start_ms;
  snapshot.solar_search_timeout_ms =
      context.solar_config.search_timeout_ms;
  snapshot.solar_start_base_duty =
      context.solar_speed_config.start_base_duty;
  snapshot.solar_slow_after_ms =
      context.solar_speed_config.slow_after_ms;
  snapshot.solar_slow_base_duty =
      context.solar_speed_config.slow_base_duty;
  snapshot.solar_slow_mode_active =
      solarSlowModeActive(context, snapshot.autonomous_time_in_state_ms);
  snapshot.solar_ir_confirmation_active =
      context.last_solar_detector_update.confirmation_active;
  snapshot.solar_beacon_confirmed =
      context.last_solar_detector_update.beacon_detected;
  snapshot.solar_contact_timeout_ms =
      context.solar_contact_config.timeout_ms;
  snapshot.solar_contact_strafe_duty =
      context.solar_contact_config.initial_strafe_right_duty;
  snapshot.solar_retry_left_strafe_duty =
      context.solar_contact_config.retry_strafe_left_duty;
  snapshot.solar_retry_right_strafe_duty =
      context.solar_contact_config.retry_strafe_right_duty;
  snapshot.solar_strafe_start_delay_ms =
      context.solar_contact_config.strafe_start_delay_ms;
  snapshot.solar_retry_strafe_left_duration_ms =
      context.solar_contact_config.retry_strafe_left_duration_ms;
  snapshot.solar_retry_forward_duration_ms =
      context.solar_contact_config.retry_forward_duration_ms;
  snapshot.solar_retry_forward_duty =
      context.solar_contact_config.retry_forward_duty;
  snapshot.solar_retry_strafe_timeout_ms =
      context.solar_contact_config.retry_strafe_timeout_ms;
  snapshot.solar_post_contact_forward_duration_ms =
      context.solar_contact_config.post_contact_forward_duration_ms;
  snapshot.solar_line_reacquire_strafe_duty =
      context.solar_contact_config.line_reacquire_strafe_duty;
  snapshot.solar_rear_line_follow_duration_ms =
      context.solar_contact_config.rear_line_follow_duration_ms;
  snapshot.solar_post_contact_forward_start_delay_ms =
      context.solar_contact_config.post_contact_forward_start_delay_ms;
  snapshot.solar_line_reacquire_strafe_start_delay_ms =
      context.solar_contact_config.line_reacquire_strafe_start_delay_ms;
  snapshot.solar_post_contact_forward_duty =
      context.solar_contact_config.post_contact_forward_duty;

  const robot::LineFollowerConfig habitat_line_config =
      habitatPiecesLineFollowerConfig(context);
  snapshot.habitat_pieces_state = context.habitat_pieces.state;
  snapshot.habitat_pieces_stop_reason =
      context.habitat_pieces.stop_reason;
  snapshot.habitat_pieces_time_in_state_ms =
      elapsedSince(now_ms, context.habitat_pieces.state_entered_at_ms);
  snapshot.habitat_pieces_line_follow_duty =
      context.habitat_pieces_config.line_follow_duty;
  snapshot.habitat_pieces_lss2_detection_delay_ms =
      context.habitat_pieces_config.lss2_detection_delay_ms;
  snapshot.habitat_pieces_lss2_detection_remaining_ms =
      context.habitat_pieces_config.lss2_detection_delay_ms >
              context.habitat_pieces.run_elapsed_ms
          ? context.habitat_pieces_config.lss2_detection_delay_ms -
                context.habitat_pieces.run_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_run_timeout_ms =
      context.habitat_pieces_config.run_timeout_ms;
  snapshot.habitat_pieces_run_elapsed_ms =
      context.habitat_pieces.run_elapsed_ms;
  snapshot.habitat_pieces_timeout_remaining_ms =
      context.habitat_pieces_config.run_timeout_ms >
              context.habitat_pieces.run_elapsed_ms
          ? context.habitat_pieces_config.run_timeout_ms -
                context.habitat_pieces.run_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_reverse_duty =
      context.habitat_pieces_config.reverse_duty;
  snapshot.habitat_pieces_reverse_duration_ms =
      context.habitat_pieces_config.reverse_duration_ms;
  snapshot.habitat_pieces_reverse_elapsed_ms =
      context.habitat_pieces.reverse_elapsed_ms;
  snapshot.habitat_pieces_reverse_remaining_ms =
      context.habitat_pieces_config.reverse_duration_ms >
              context.habitat_pieces.reverse_elapsed_ms
          ? context.habitat_pieces_config.reverse_duration_ms -
                context.habitat_pieces.reverse_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_distance_strafe_direction =
      context.habitat_pieces_config.distance_strafe_direction;
  snapshot.habitat_pieces_distance_threshold_mm =
      context.habitat_pieces_config.distance_threshold_mm;
  snapshot.habitat_pieces_distance_zone_target_count =
      context.habitat_pieces_config.distance_zone_target_count;
  snapshot.habitat_pieces_distance_strafe_duty =
      context.habitat_pieces_config.distance_strafe_duty;
  snapshot.habitat_pieces_distance_strafe_timeout_ms =
      context.habitat_pieces_config.distance_strafe_timeout_ms;
  snapshot.habitat_pieces_distance_strafe_elapsed_ms =
      context.habitat_pieces.distance_strafe_elapsed_ms;
  snapshot.habitat_pieces_distance_strafe_remaining_ms =
      context.habitat_pieces_config.distance_strafe_timeout_ms >
              context.habitat_pieces.distance_strafe_elapsed_ms
          ? context.habitat_pieces_config.distance_strafe_timeout_ms -
                context.habitat_pieces.distance_strafe_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_distance_mm =
      context.habitat_pieces.latest_distance_mm;
  snapshot.habitat_pieces_distance_zone_count =
      context.habitat_pieces.distance_zone_count;
  snapshot.habitat_pieces_compensation_strafe_duty =
      context.habitat_pieces_config.compensation_strafe_duty;
  snapshot.habitat_pieces_compensation_strafe_duration_ms =
      context.habitat_pieces_config.compensation_strafe_duration_ms;
  snapshot.habitat_pieces_compensation_strafe_elapsed_ms =
      context.habitat_pieces.compensation_strafe_elapsed_ms;
  snapshot.habitat_pieces_compensation_strafe_remaining_ms =
      context.habitat_pieces_config.compensation_strafe_duration_ms >
              context.habitat_pieces.compensation_strafe_elapsed_ms
          ? context.habitat_pieces_config.compensation_strafe_duration_ms -
                context.habitat_pieces.compensation_strafe_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_slide_down_speed_steps_per_second =
      context.habitat_pieces_config.slide_down_speed_steps_per_second;
  snapshot.habitat_pieces_slide_down_timeout_ms =
      context.habitat_pieces_config.slide_down_timeout_ms;
  snapshot.habitat_pieces_slide_down_elapsed_ms =
      context.habitat_pieces.slide_down_elapsed_ms;
  snapshot.habitat_pieces_slide_down_remaining_ms =
      context.habitat_pieces_config.slide_down_timeout_ms >
              context.habitat_pieces.slide_down_elapsed_ms
          ? context.habitat_pieces_config.slide_down_timeout_ms -
                context.habitat_pieces.slide_down_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_forward_to_distance_duty =
      context.habitat_pieces_config.forward_to_distance_duty;
  snapshot.habitat_pieces_forward_stop_distance_mm =
      context.habitat_pieces_config.forward_stop_distance_mm;
  snapshot.habitat_pieces_forward_to_distance_timeout_ms =
      context.habitat_pieces_config.forward_to_distance_timeout_ms;
  snapshot.habitat_pieces_forward_to_distance_elapsed_ms =
      context.habitat_pieces.forward_to_distance_elapsed_ms;
  snapshot.habitat_pieces_forward_to_distance_remaining_ms =
      context.habitat_pieces_config.forward_to_distance_timeout_ms >
              context.habitat_pieces.forward_to_distance_elapsed_ms
          ? context.habitat_pieces_config.forward_to_distance_timeout_ms -
                context.habitat_pieces.forward_to_distance_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_slide_lift_steps =
      context.habitat_pieces_config.slide_lift_steps;
  snapshot.habitat_pieces_slide_lift_speed_steps_per_second =
      context.habitat_pieces_config.slide_lift_speed_steps_per_second;
  snapshot.habitat_pieces_slide_lift_timeout_ms =
      context.habitat_pieces_config.slide_lift_timeout_ms;
  snapshot.habitat_pieces_slide_lift_elapsed_ms =
      context.habitat_pieces.slide_lift_elapsed_ms;
  snapshot.habitat_pieces_slide_lift_remaining_ms =
      context.habitat_pieces_config.slide_lift_timeout_ms >
              context.habitat_pieces.slide_lift_elapsed_ms
          ? context.habitat_pieces_config.slide_lift_timeout_ms -
                context.habitat_pieces.slide_lift_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_post_pickup_reverse_duty =
      context.habitat_pieces_config.post_pickup_reverse_duty;
  snapshot.habitat_pieces_post_pickup_reverse_duration_ms =
      context.habitat_pieces_config.post_pickup_reverse_duration_ms;
  snapshot.habitat_pieces_post_pickup_reverse_elapsed_ms =
      context.habitat_pieces.post_pickup_reverse_elapsed_ms;
  snapshot.habitat_pieces_post_pickup_reverse_remaining_ms =
      context.habitat_pieces_config.post_pickup_reverse_duration_ms >
              context.habitat_pieces.post_pickup_reverse_elapsed_ms
          ? context.habitat_pieces_config.post_pickup_reverse_duration_ms -
                context.habitat_pieces.post_pickup_reverse_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_return_strafe_duty =
      context.habitat_pieces_config.return_strafe_duty;
  snapshot.habitat_pieces_return_line_timeout_ms =
      context.habitat_pieces_config.return_line_timeout_ms;
  snapshot.habitat_pieces_return_strafe_elapsed_ms =
      context.habitat_pieces.return_strafe_elapsed_ms;
  snapshot.habitat_pieces_return_strafe_remaining_ms =
      context.habitat_pieces_config.return_line_timeout_ms >
              context.habitat_pieces.return_strafe_elapsed_ms
          ? context.habitat_pieces_config.return_line_timeout_ms -
                context.habitat_pieces.return_strafe_elapsed_ms
          : 0U;
  snapshot.habitat_pieces_configuration_valid =
      g_runtime.stepper != nullptr &&
      g_runtime.stepper->maximumPositionSteps() > 0 &&
      g_runtime.stepper->maximumPositionSteps() <=
          static_cast<std::int64_t>(UINT32_MAX) &&
      robot::habitatPiecesConfigValid(
          context.habitat_pieces_config, activeMotionDutyCap(context),
          kMaxTimedTestDurationMs,
          g_runtime.stepper->maximumSpeedStepsPerSecond(),
          static_cast<std::uint32_t>(
              g_runtime.stepper->maximumPositionSteps())) &&
      robot::validateLineFollowerConfig(habitat_line_config,
                                        hardwareDutyCap())
          .accepted;
  snapshot.habitat_pieces_start_ready =
      g_runtime.stepper != nullptr &&
      habitatPiecesStartRequirementsMet(
          sensors, front_left, front_right, rear_link,
          *g_runtime.stepper, now_ms, context);
  snapshot.habitat_pieces_lss2_data_fresh =
      rear_link.rearLineSnapshotFresh(
          now_ms, context.config.remoteCommandTimeoutMs);
  snapshot.habitat_pieces_lss2_configured =
      rear_link.rearLineSnapshotAvailable() &&
      rear_link.latestRearLineSnapshot().side_2_configured;
  snapshot.habitat_pieces_lss3_data_fresh =
      snapshot.habitat_pieces_lss2_data_fresh;
  snapshot.habitat_pieces_lss3_configured =
      rear_link.rearLineSnapshotAvailable() &&
      rear_link.latestRearLineSnapshot().side_3_configured;
  snapshot.habitat_pieces_lss2_detection_armed =
      context.habitat_pieces.lss2_detection_armed;
  snapshot.habitat_pieces_lss2_black =
      snapshot.habitat_pieces_lss2_data_fresh &&
      snapshot.habitat_pieces_lss2_configured &&
      rear_link.latestRearLineSnapshot().side_2_electrical_high;
  snapshot.habitat_pieces_lss3_black =
      snapshot.habitat_pieces_lss3_data_fresh &&
      snapshot.habitat_pieces_lss3_configured &&
      rear_link.latestRearLineSnapshot().side_3_electrical_high;
  snapshot.habitat_pieces_lss2_latched =
      context.habitat_pieces.lss2_latched;
  snapshot.habitat_pieces_lss3_latched =
      context.habitat_pieces.lss3_latched;
  snapshot.habitat_pieces_should_stop =
      context.last_habitat_pieces_update.should_stop;
  snapshot.habitat_pieces_target_reached =
      context.last_habitat_pieces_update.target_reached;
  snapshot.habitat_pieces_line_following =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::LineFollowing &&
      context.follower_state.enabled;
  snapshot.habitat_pieces_side_line_aligning =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::SideLineAligning;
  snapshot.habitat_pieces_left_side_driving =
      snapshot.habitat_pieces_side_line_aligning &&
      context.last_habitat_pieces_update.should_drive_left_side;
  snapshot.habitat_pieces_right_side_driving =
      snapshot.habitat_pieces_side_line_aligning &&
      context.last_habitat_pieces_update.should_drive_right_side;
  snapshot.habitat_pieces_reversing =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::Reversing;
  snapshot.habitat_pieces_distance_strafing =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::DistanceStrafing;
  snapshot.habitat_pieces_distance_measurement_available =
      context.last_habitat_pieces_update.distance_measurement_available;
  snapshot.habitat_pieces_distance_sample_new =
      context.last_habitat_pieces_update.distance_sample_new;
  snapshot.habitat_pieces_distance_zone_active =
      context.habitat_pieces.distance_zone_active;
  snapshot.habitat_pieces_distance_zone_entered =
      context.last_habitat_pieces_update.distance_zone_entered;
  snapshot.habitat_pieces_compensation_strafing =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::CompensationStrafing;
  snapshot.habitat_pieces_lowering_slide =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::LoweringSlide;
  snapshot.habitat_pieces_slide_bottom_ready =
      g_runtime.stepper != nullptr &&
      g_runtime.stepper->lowerLimitActive() &&
      g_runtime.stepper->isHomed();
  snapshot.habitat_pieces_forward_to_distance =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::ForwardToDistance;
  snapshot.habitat_pieces_forward_distance_reached =
      context.habitat_pieces.slide_lift_started;
  snapshot.habitat_pieces_slide_lift_started =
      context.habitat_pieces.slide_lift_started;
  snapshot.habitat_pieces_slide_lift_complete =
      context.habitat_pieces.slide_lift_complete;
  snapshot.habitat_pieces_post_pickup_reversing =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::PostPickupReversing;
  snapshot.habitat_pieces_return_line_strafing =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::ReturnLineStrafing;
  snapshot.habitat_pieces_waiting_for_slide_lift =
      context.modes.currentMode() == robot::RobotTestMode::HabitatPieces &&
      context.habitat_pieces.state ==
          robot::HabitatPiecesState::WaitForSlideLift;
  snapshot.habitat_pieces_rear_line_data_fresh =
      rear_link.rearLineSnapshotFresh(
          now_ms, context.rear_config.remoteCommandTimeoutMs);
  snapshot.habitat_pieces_rear_line_configured =
      rear_link.rearLineSnapshotAvailable() &&
      rear_link.latestRearLineSnapshot().configured;
  snapshot.habitat_pieces_rear_left_black =
      snapshot.habitat_pieces_rear_line_data_fresh &&
      snapshot.habitat_pieces_rear_line_configured &&
      rear_link.latestRearLineSnapshot().left_electrical_high;
  snapshot.habitat_pieces_rear_right_black =
      snapshot.habitat_pieces_rear_line_data_fresh &&
      snapshot.habitat_pieces_rear_line_configured &&
      rear_link.latestRearLineSnapshot().right_electrical_high;
  snapshot.habitat_pieces_rear_line_detected =
      context.habitat_pieces.rear_line_detected;
  snapshot.habitat_pieces_timed_out = context.habitat_pieces.timed_out;

  snapshot.habitat_placement_state = context.habitat_placement.state;
  snapshot.habitat_placement_fault_reason =
      context.habitat_placement.fault_reason;
  snapshot.habitat_placement_config =
      context.habitat_placement_config;
  snapshot.habitat_placement_time_in_state_ms = elapsedSince(
      now_ms, context.habitat_placement.state_entered_at_ms);
  snapshot.habitat_placement_configuration_valid =
      g_runtime.stepper != nullptr &&
      robot::habitatPlacementConfigValid(
          context.habitat_placement_config,
          rearMotionDutyCap(context),
          g_runtime.stepper->maximumSpeedStepsPerSecond());
  snapshot.habitat_placement_start_ready =
      g_runtime.stepper != nullptr &&
      habitatPlacementStartRequirementsMet(
          sensors, front_left, front_right, rear_link, claws,
          *g_runtime.stepper, now_ms, context);
  snapshot.habitat_placement_initial_heading_captured =
      context.habitat_placement.initial_heading_captured;
  snapshot.habitat_placement_initial_heading_deg =
      context.habitat_placement.initial_heading_deg;
  snapshot.habitat_placement_counter_clockwise_target_heading_deg =
      context.habitat_placement.counter_clockwise_target_heading_deg;

  snapshot.tower_pieces_state = context.tower_pieces.state;
  snapshot.tower_pieces_fault_reason =
      context.tower_pieces.fault_reason;
  snapshot.tower_pieces_time_in_state_ms =
      elapsedSince(now_ms, context.tower_pieces.state_entered_at_ms);
  snapshot.tower_pieces_reverse_line_duty =
      context.tower_pieces_config.reverse_line_duty;
  snapshot.tower_pieces_side_line_timeout_ms =
      context.tower_pieces_config.side_line_timeout_ms;
  snapshot.tower_pieces_side_line_cooldown_ms =
      context.tower_pieces_config.side_line_cooldown_ms;
  snapshot.tower_pieces_side_line_rearm_ms =
      context.tower_pieces_config.side_line_rearm_ms;
  snapshot.tower_pieces_post_line_delay_ms =
      context.tower_pieces_config.post_line_delay_ms;
  snapshot.tower_pieces_strafe_right_duty =
      context.tower_pieces_config.strafe_right_duty;
  snapshot.tower_pieces_strafe_right_duration_ms =
      context.tower_pieces_config.strafe_right_duration_ms;
  snapshot.tower_pieces_post_strafe_pause_ms =
      context.tower_pieces_config.post_strafe_pause_ms;
  snapshot.tower_pieces_clockwise_rotation_duty =
      context.imu_turn_config.maximum_rotation_duty;
  snapshot.tower_pieces_clockwise_rotation_angle_deg =
      context.tower_pieces_config.clockwise_rotation_angle_deg;
  snapshot.tower_pieces_post_rotation_pause_ms =
      context.tower_pieces_config.post_rotation_pause_ms;
  snapshot.tower_pieces_reverse_duty =
      context.tower_pieces_config.reverse_duty;
  snapshot.tower_pieces_reverse_duration_ms =
      context.tower_pieces_config.reverse_duration_ms;
  snapshot.tower_pieces_shimmy_duty =
      context.tower_pieces_config.shimmy_duty;
  snapshot.tower_pieces_shimmy_right_duration_ms =
      context.tower_pieces_config.shimmy_right_duration_ms;
  snapshot.tower_pieces_shimmy_left_duration_ms =
      context.tower_pieces_config.shimmy_left_duration_ms;
  snapshot.tower_pieces_shimmy_timeout_ms =
      context.tower_pieces_config.shimmy_timeout_ms;
  snapshot.tower_pieces_final_reverse_duty =
      context.tower_pieces_config.final_reverse_duty;
  snapshot.tower_pieces_final_reverse_duration_ms =
      context.tower_pieces_config.final_reverse_duration_ms;
  snapshot.tower_pieces_post_final_reverse_delay_ms =
      context.tower_pieces_config.post_final_reverse_delay_ms;
  snapshot.tower_pieces_post_winch_open_delay_ms =
      context.tower_pieces_config.post_winch_open_delay_ms;
  snapshot.tower_pieces_post_claws_open_delay_ms =
      context.tower_pieces_config.post_claws_open_delay_ms;
  snapshot.tower_pieces_pre_stepper_bottom_delay_ms =
      context.tower_pieces_config.pre_stepper_bottom_delay_ms;
  snapshot.tower_pieces_stepper_down_speed_steps_per_second =
      context.tower_pieces_config.stepper_down_speed_steps_per_second;
  snapshot.tower_pieces_post_stepper_bottom_delay_ms =
      context.tower_pieces_config.post_stepper_bottom_delay_ms;
  snapshot.tower_pieces_post_claws_closed_delay_ms =
      context.tower_pieces_config.post_claws_closed_delay_ms;
  snapshot.tower_pieces_stepper_up_speed_steps_per_second =
      context.tower_pieces_config.stepper_up_speed_steps_per_second;
  snapshot.tower_pieces_side_line_count =
      context.tower_pieces.side_line_count;
  snapshot.tower_pieces_side_line_rejected_count =
      context.tower_pieces.side_line_rejected_count;
  snapshot.tower_pieces_side_line_armed =
      context.tower_pieces.side_line_armed;
  snapshot.tower_pieces_side_line_detection_accepted =
      context.last_tower_pieces_update.side_line_detection_accepted;
  snapshot.tower_pieces_side_line_detection_rejected =
      context.last_tower_pieces_update.side_line_detection_rejected;
  snapshot.tower_pieces_target_side_line_count =
      robot::kTowerPiecesTargetSideLineCount;
  const bool tower_pieces_mode_active =
      context.modes.currentMode() ==
          robot::RobotTestMode::AutonomousTowerPieces ||
      time_trial_tower_active;
  snapshot.tower_pieces_line_following =
      tower_pieces_mode_active &&
      context.tower_pieces.state ==
          robot::TowerPiecesState::ReverseLineFollow &&
      context.follower_state.enabled;
  snapshot.tower_pieces_strafing_right =
      tower_pieces_mode_active &&
      context.tower_pieces.state == robot::TowerPiecesState::StrafeRight;
  snapshot.tower_pieces_rotating_clockwise =
      tower_pieces_mode_active &&
      context.tower_pieces.state ==
          robot::TowerPiecesState::RotateClockwise;
  snapshot.tower_pieces_driving_backward =
      tower_pieces_mode_active &&
      context.tower_pieces.state == robot::TowerPiecesState::ReverseTimed;
  snapshot.tower_pieces_shimmying_left =
      tower_pieces_mode_active &&
      context.tower_pieces.state == robot::TowerPiecesState::ShimmyLeft;
  snapshot.tower_pieces_shimmying_right =
      tower_pieces_mode_active &&
      context.tower_pieces.state == robot::TowerPiecesState::ShimmyRight;
  snapshot.tower_pieces_final_reverse_active =
      tower_pieces_mode_active &&
      context.tower_pieces.state == robot::TowerPiecesState::FinalReverse;
  snapshot.tower_pieces_stepper_moving_down =
      tower_pieces_mode_active &&
      context.tower_pieces.state ==
          robot::TowerPiecesState::MoveStepperBottom;
  snapshot.tower_pieces_stepper_moving_up =
      tower_pieces_mode_active &&
      context.tower_pieces.state == robot::TowerPiecesState::MoveStepperTop;
  snapshot.peg_finder_state = context.peg_finder.state;
  snapshot.peg_finder_fault_reason = context.peg_finder.fault_reason;
  snapshot.peg_finder_time_in_state_ms =
      elapsedSince(now_ms, context.peg_finder.state_entered_at_ms);
  snapshot.peg_finder_clockwise_duty =
      context.imu_turn_config.maximum_rotation_duty;
  snapshot.peg_finder_clockwise_angle_deg =
      context.peg_finder_config.clockwise_angle_deg;
  snapshot.peg_finder_post_rotation_pause_ms =
      context.peg_finder_config.post_rotation_pause_ms;
  snapshot.peg_finder_reverse_duty =
      context.peg_finder_config.reverse_duty;
  snapshot.peg_finder_reverse_duration_ms =
      context.peg_finder_config.reverse_duration_ms;
  snapshot.peg_finder_post_reverse_pause_ms =
      context.peg_finder_config.post_reverse_pause_ms;
  snapshot.peg_finder_forward_duty =
      context.peg_finder_config.forward_duty;
  snapshot.peg_finder_forward_duration_ms =
      context.peg_finder_config.forward_duration_ms;
  snapshot.peg_finder_funnel_forward_duty =
      context.peg_finder_config.funnel_forward_duty;
  snapshot.peg_finder_funnel_forward_timeout_ms =
      context.peg_finder_config.funnel_forward_timeout_ms;
  snapshot.peg_finder_post_funnel_limit_delay_ms =
      context.peg_finder_config.post_funnel_limit_delay_ms;
  snapshot.peg_finder_claw_open_interval_ms =
      context.peg_finder_config.claw_open_interval_ms;
  snapshot.peg_finder_claw_open_order_1 =
      context.peg_finder_config.claw_open_order_1;
  snapshot.peg_finder_claw_open_order_2 =
      context.peg_finder_config.claw_open_order_2;
  snapshot.peg_finder_claw_open_order_3 =
      context.peg_finder_config.claw_open_order_3;
  snapshot.peg_finder_post_claws_open_delay_ms =
      context.peg_finder_config.post_claws_open_delay_ms;
  snapshot.peg_finder_funnel_reverse_duty =
      context.peg_finder_config.funnel_reverse_duty;
  snapshot.peg_finder_funnel_reverse_duration_ms =
      context.peg_finder_config.funnel_reverse_duration_ms;
  snapshot.peg_finder_funnel_limit_configured =
      funnel_limit.configured();
  snapshot.peg_finder_funnel_limit_high = funnel_limit.active();
  const bool peg_finder_mode_active =
      context.modes.currentMode() == robot::RobotTestMode::PegFinder ||
      (context.modes.currentMode() == robot::RobotTestMode::TimeTrial &&
       context.time_trial.state == robot::TimeTrialState::PegFinder);
  snapshot.peg_finder_rotating_clockwise =
      peg_finder_mode_active &&
      context.peg_finder.state == robot::PegFinderState::RotateClockwise;
  snapshot.peg_finder_driving_backward =
      peg_finder_mode_active &&
      context.peg_finder.state == robot::PegFinderState::Reverse;
  snapshot.peg_finder_driving_forward =
      peg_finder_mode_active &&
      context.peg_finder.state == robot::PegFinderState::Forward;
  snapshot.peg_finder_funnel_forward =
      peg_finder_mode_active &&
      context.peg_finder.state == robot::PegFinderState::FunnelForward;
  snapshot.peg_finder_funnel_reverse =
      peg_finder_mode_active &&
      context.peg_finder.state == robot::PegFinderState::FunnelReverse;
  const bool opening_first_claw =
      context.peg_finder.state == robot::PegFinderState::OpenClaw1;
  const bool opening_second_claw =
      context.peg_finder.state == robot::PegFinderState::OpenClaw2;
  const bool opening_third_claw =
      context.peg_finder.state == robot::PegFinderState::OpenClaw3;
  snapshot.peg_finder_opening_claw_1 =
      peg_finder_mode_active &&
      ((opening_first_claw &&
        context.peg_finder_config.claw_open_order_1 == 1U) ||
       (opening_second_claw &&
        context.peg_finder_config.claw_open_order_2 == 1U) ||
       (opening_third_claw &&
        context.peg_finder_config.claw_open_order_3 == 1U));
  snapshot.peg_finder_opening_claw_2 =
      peg_finder_mode_active &&
      ((opening_first_claw &&
        context.peg_finder_config.claw_open_order_1 == 2U) ||
       (opening_second_claw &&
        context.peg_finder_config.claw_open_order_2 == 2U) ||
       (opening_third_claw &&
        context.peg_finder_config.claw_open_order_3 == 2U));
  snapshot.peg_finder_opening_claw_3 =
      peg_finder_mode_active &&
      ((opening_first_claw &&
        context.peg_finder_config.claw_open_order_1 == 3U) ||
       (opening_second_claw &&
        context.peg_finder_config.claw_open_order_2 == 3U) ||
       (opening_third_claw &&
        context.peg_finder_config.claw_open_order_3 == 3U));

  snapshot.time_trial_state = context.time_trial.state;
  snapshot.time_trial_time_in_state_ms =
      elapsedSince(now_ms, context.time_trial.state_entered_at_ms);
  snapshot.time_trial_post_solar_delay_ms =
      context.time_trial_config.post_solar_delay_ms;
  snapshot.time_trial_strafe_right_duty =
      context.imu_heading_hold_config.maximum_strafe_duty;
  snapshot.time_trial_strafe_right_duration_ms =
      context.time_trial_config.solar_to_tower_strafe_right_duration_ms;
  snapshot.time_trial_post_tower_delay_ms =
      context.time_trial_config.post_tower_delay_ms;
  snapshot.time_trial_strafing_right =
      context.modes.currentMode() == robot::RobotTestMode::TimeTrial &&
      context.time_trial.state ==
          robot::TimeTrialState::SolarToTowerStrafeRight;
  snapshot.limit_switch_funnel_left = funnel_limit.active();
  fillMotorTelemetry(snapshot.front_left, front_left);
  fillMotorTelemetry(snapshot.front_right, front_right);
  snapshot.funnel.desired_command_milli =
      context.requested_funnel_command.duty_command_milli;
  snapshot.funnel.enabled = context.requested_funnel_command.enabled;
  claws.fillTelemetry(snapshot.claws);
  snapshot.servo_claw_1_position =
      snapshot.claws.claw_1.commanded_angle_deg;
  snapshot.servo_claw_2_position =
      snapshot.claws.claw_2.commanded_angle_deg;
  snapshot.servo_claw_3_position =
      snapshot.claws.claw_3.commanded_angle_deg;
  snapshot.servo_pusher_position =
      snapshot.claws.habitat_pusher.commanded_angle_deg;
  snapshot.servo_winch_position =
      snapshot.claws.winch.commanded_angle_deg;
  snapshot.solar_hook.open_angle_deg =
      context.solar_hook_servo_settings.open_angle_deg;
  snapshot.solar_hook.closed_angle_deg =
      context.solar_hook_servo_settings.closed_angle_deg;
  snapshot.solar_hook.open_configured =
      snapshot.solar_hook.open_angle_deg >= 0 &&
      snapshot.solar_hook.open_angle_deg <= 180;
  snapshot.solar_hook.closed_configured =
      snapshot.solar_hook.closed_angle_deg >= 0 &&
      snapshot.solar_hook.closed_angle_deg <= 180;
  snapshot.solar_hook.commanded_open =
      context.solar_hook_commanded_open;
  snapshot.rear.back_left_desired_command_milli =
      context.last_commanded_wheels.back_left.duty_command_milli;
  snapshot.rear.back_right_desired_command_milli =
      context.last_commanded_wheels.back_right.duty_command_milli;
  snapshot.rear.sequence = rear_link.lastSequenceSent();
  snapshot.rear.command_age_ms =
      rear_link.lastSentAtMs() == 0U ? 0U
                                     : elapsedSince(now_ms,
                                                    rear_link.lastSentAtMs());
  snapshot.rear.esp1_link_healthy =
      rear_link.remoteStatusFresh(now_ms,
                                  remoteStatusTimeoutMs(context.config));
  snapshot.rear.esp1_link_configured = rear_link.configured();
  snapshot.rear.esp1_last_packet_age_ms =
      rear_link.lastStatusReceivedAtMs() == 0U
          ? 0U
          : elapsedSince(now_ms, rear_link.lastStatusReceivedAtMs());
  snapshot.rear.esp1_packet_error_count = rear_link.packetErrorCount();
  snapshot.motor_command_magnitude_milli =
      driveCommandMagnitudeMilli(context.last_commanded_wheels);
  if (rear_link.statusAvailable()) {
    const robot::Esp1StatusReport& esp1 = rear_link.latestStatus();
    snapshot.esp1.available = true;
    snapshot.esp1.uptime_ms = esp1.uptime_ms;
    snapshot.esp1.mode = esp1.mode;
    snapshot.esp1.fault_active = esp1.fault_active;
    snapshot.esp1.fault_code = esp1.fault_code;
    snapshot.esp1.back_left_applied_command_milli =
        esp1.back_left_applied_command_milli;
    snapshot.esp1.back_right_applied_command_milli =
        esp1.back_right_applied_command_milli;
    snapshot.esp1.funnel_applied_command_milli =
        esp1.funnel_applied_command_milli;
    snapshot.esp1.back_left_inverted = esp1.back_left_inverted;
    snapshot.esp1.back_right_inverted = esp1.back_right_inverted;
    snapshot.esp1.funnel_configured = esp1.funnel_configured;
    snapshot.esp1.solar_panel_limit_switches_configured =
        esp1.solar_panel_limit_switches_configured;
    snapshot.esp1.solar_limit_back_right_high =
        esp1.solar_limit_back_right_high;
    snapshot.esp1.solar_limit_front_right_high =
        esp1.solar_limit_front_right_high;
    snapshot.esp1.side_line_sensor_configured =
        esp1.side_line_sensor_configured;
    snapshot.esp1.side_line_sensor_high =
        esp1.side_line_sensor_high;
    snapshot.esp1.ultrasonic_1_configured =
        esp1.ultrasonic_1_configured;
    snapshot.esp1.ultrasonic_1_echo_valid =
        esp1.ultrasonic_1_echo_valid;
    snapshot.esp1.ultrasonic_1_distance_mm =
        esp1.ultrasonic_1_distance_mm;
    snapshot.esp1.ultrasonic_1_echo_duration_us =
        esp1.ultrasonic_1_echo_duration_us;
    snapshot.esp1.solar_hook_configured =
        esp1.solar_hook_configured;
    snapshot.esp1.solar_hook_output_enabled =
        esp1.solar_hook_output_enabled;
    snapshot.esp1.solar_hook_commanded_angle_deg =
        esp1.solar_hook_commanded_angle_deg;
    snapshot.solar_hook.hardware_configured =
        esp1.solar_hook_configured;
    snapshot.solar_hook.output_enabled =
        esp1.solar_hook_output_enabled;
    snapshot.solar_hook.commanded_angle_deg =
        esp1.solar_hook_commanded_angle_deg;
    snapshot.ultrasonic_1.configured =
        esp1.ultrasonic_1_configured;
    snapshot.ultrasonic_1.data_fresh =
        snapshot.rear.esp1_link_healthy;
    snapshot.ultrasonic_1.echo_valid =
        esp1.ultrasonic_1_echo_valid;
    snapshot.ultrasonic_1.distance_mm =
        esp1.ultrasonic_1_distance_mm;
    snapshot.ultrasonic_1.echo_duration_us =
        esp1.ultrasonic_1_echo_duration_us;
    snapshot.ultrasonic_1.sample_age_ms =
        snapshot.rear.esp1_last_packet_age_ms;
    snapshot.ultrasonic_1_distance_mm =
        esp1.ultrasonic_1_echo_valid
            ? static_cast<int>(esp1.ultrasonic_1_distance_mm)
            : -1;
    snapshot.solar_panel_limit_switches_configured =
        esp1.solar_panel_limit_switches_configured;
    snapshot.solar_limit_back_right_high =
        esp1.solar_limit_back_right_high;
    snapshot.solar_limit_front_right_high =
        esp1.solar_limit_front_right_high;
    snapshot.solar_limit_back_right_hit =
        solarPanelLimitSwitchHit(esp1.solar_limit_back_right_high);
    snapshot.solar_limit_front_right_hit =
        solarPanelLimitSwitchHit(esp1.solar_limit_front_right_high);
    snapshot.solar_limit_all_hit = solarPanelLimitSwitchesAllHit(esp1);
    snapshot.funnel.applied_command_milli =
        esp1.funnel_applied_command_milli;
    snapshot.funnel.enabled = esp1.funnel_applied_command_milli != 0;
    snapshot.funnel.configured = esp1.funnel_configured;
    snapshot.ir_adc_average = esp1.ir_adc_average;
    snapshot.ir_adc_min = esp1.ir_adc_min;
    snapshot.ir_adc_max = esp1.ir_adc_max;
    snapshot.ir_amplitude_pp = esp1.ir_amplitude_pp;
    snapshot.ir_beacon_detected = esp1.ir_beacon_detected;
    snapshot.ir_switch_raw_state = esp1.ir_switch_raw_high;
    snapshot.ir_switch_debounced_state = esp1.ir_switch_debounced_high;
    snapshot.selected_beacon_frequency_hz =
        esp1.ir_selected_frequency_hz;
    snapshot.ir_adc_latest_sample = esp1.ir_adc_latest_sample;
    snapshot.ir_adc_sample_mean = esp1.ir_adc_average;
    snapshot.ir_1khz_goertzel_amplitude = esp1.ir_1khz_amplitude;
    snapshot.ir_10khz_goertzel_amplitude = esp1.ir_10khz_amplitude;
    snapshot.ir_selected_frequency_amplitude =
        esp1.ir_selected_amplitude;
    snapshot.solar_ir_raw_amplitude = esp1.ir_selected_amplitude;
    snapshot.ir_active_threshold = esp1.ir_active_threshold;
    snapshot.ir_consecutive_detection_count =
        esp1.ir_consecutive_detection_count;
    snapshot.ir_adc_sample_rate_hz = esp1.ir_adc_sample_rate_hz;
  }
  if (rear_link.laserSnapshotAvailable()) {
    const robot::LaserDistanceSnapshot& laser =
        rear_link.latestLaserSnapshot();
    robot::LaserDistanceTelemetry& telemetry =
        snapshot.laser_distance;
    telemetry.available = true;
    telemetry.configured = laser.configured;
    telemetry.initialized = laser.initialized;
    telemetry.ranging = laser.ranging;
    telemetry.data_valid = laser.data_valid;
    telemetry.profile = laser.profile;
    telemetry.distance_mm = laser.distance_mm;
    telemetry.measurement_sequence = laser.measurement_sequence;
    telemetry.packet_sequence =
        rear_link.lastLaserPacketSequence();
    telemetry.sensor_range_status = laser.sensor_range_status;
    telemetry.driver_status = laser.driver_status;
    telemetry.sda_gpio = laser.sda_gpio;
    telemetry.scl_gpio = laser.scl_gpio;
    telemetry.i2c_address = laser.i2c_address;
    telemetry.captured_at_ms = laser.captured_at_ms;
    telemetry.intermeasurement_period_ms =
        laser.intermeasurement_period_ms;
    telemetry.successful_measurement_count =
        laser.successful_measurement_count;
    telemetry.failed_measurement_count =
        laser.failed_measurement_count;
    telemetry.consecutive_failed_measurements =
        laser.consecutive_failed_measurements;
    telemetry.acquisition_duration_us =
        laser.acquisition_duration_us;
    telemetry.maximum_acquisition_duration_us =
        laser.maximum_acquisition_duration_us;
    const robot::Milliseconds measurement_received_at_ms =
        rear_link.lastLaserMeasurementReceivedAtMs();
    telemetry.sample_age_ms =
        measurement_received_at_ms == 0U
            ? 0U
            : elapsedSince(now_ms, measurement_received_at_ms);
    const robot::Milliseconds snapshot_received_at_ms =
        rear_link.lastLaserSnapshotReceivedAtMs();
    telemetry.snapshot_age_ms =
        snapshot_received_at_ms == 0U
            ? 0U
            : elapsedSince(now_ms, snapshot_received_at_ms);
    const robot::Milliseconds telemetry_freshness_ms =
        static_cast<robot::Milliseconds>(
            laser.intermeasurement_period_ms) *
        4U;
    telemetry.data_fresh =
        measurement_received_at_ms != 0U &&
        telemetry_freshness_ms > 0U &&
        telemetry.sample_age_ms <= telemetry_freshness_ms;
  }
  if (snapshot.rear_line_data_fresh &&
      rear_link.rearLineSnapshotAvailable()) {
    const robot::RearLineSensorSnapshot& line_sensors =
        rear_link.latestRearLineSnapshot();
    snapshot.tower_pieces_side_line_sensor_configured =
        line_sensors.side_configured;
    snapshot.tower_pieces_side_line_sensor_high =
        line_sensors.side_electrical_high;
    snapshot.tower_pieces_back_line_detected =
        line_sensors.left_electrical_high ||
        line_sensors.right_electrical_high;
    snapshot.lss_configured = line_sensors.side_configured;
    if (line_sensors.side_configured) {
      snapshot.lss_raw_level =
          line_sensors.side_electrical_high ? 1 : 0;
      snapshot.lss_black = line_sensors.side_electrical_high;
    }
    snapshot.lss2_configured = line_sensors.side_2_configured;
    if (line_sensors.side_2_configured) {
      snapshot.lss2_raw_level =
          line_sensors.side_2_electrical_high ? 1 : 0;
      snapshot.lss2_black = line_sensors.side_2_electrical_high;
    }
    snapshot.lss3_configured = line_sensors.side_3_configured;
    if (line_sensors.side_3_configured) {
      snapshot.lss3_raw_level =
          line_sensors.side_3_electrical_high ? 1 : 0;
      snapshot.lss3_black = line_sensors.side_3_electrical_high;
    }
  }
}

bool argFloat(const char* name, float& value, const float fallback,
              const bool required) {
  if (!g_server.hasArg(name)) {
    value = fallback;
    return !required;
  }
  const String text = g_server.arg(name);
  return parseFloat(text.c_str(), value);
}

bool argUnsigned(const char* name, robot::Milliseconds& value,
                 const robot::Milliseconds fallback,
                 const bool required) {
  if (!g_server.hasArg(name)) {
    value = fallback;
    return !required;
  }
  const String text = g_server.arg(name);
  return parseUnsigned(text.c_str(), value);
}

bool argSigned(const char* name, int& value, const int fallback,
               const bool required) {
  if (!g_server.hasArg(name)) {
    value = fallback;
    return !required;
  }
  const String text = g_server.arg(name);
  return parseSignedInteger(text.c_str(), value);
}

bool argPolarity(const char* name, int& value, const int fallback,
                 const bool required) {
  if (!g_server.hasArg(name)) {
    value = fallback;
    return !required;
  }
  const String text = g_server.arg(name);
  return parsePolarity(text.c_str(), value);
}

bool argOnOff(const char* name, bool& value, const bool fallback,
              const bool required) {
  if (!g_server.hasArg(name)) {
    value = fallback;
    return !required;
  }
  const String text = g_server.arg(name);
  return parseOnOff(text.c_str(), value);
}

void sendErrorJson(const int status, const char* reason) {
  std::snprintf(g_json_buffer, sizeof(g_json_buffer),
                "{\"ok\":false,\"error\":\"%s\"}", reason);
  g_server.send(status, "application/json", g_json_buffer);
}

void sendOkJson(const char* message) {
  std::snprintf(g_json_buffer, sizeof(g_json_buffer),
                "{\"ok\":true,\"message\":\"%s\"}", message);
  g_server.send(200, "application/json", g_json_buffer);
}

bool runtimeReady() {
  return g_runtime.context != nullptr && g_runtime.sensors != nullptr &&
         g_runtime.peg_finder_funnel_limit != nullptr &&
         g_runtime.front_left != nullptr && g_runtime.front_right != nullptr &&
         g_runtime.rear_link != nullptr && g_runtime.claws != nullptr &&
         g_runtime.stepper != nullptr &&
         g_runtime.imu_acquisition != nullptr;
}

robot::TelemetrySnapshot currentSnapshot() {
  robot::TelemetrySnapshot snapshot{};
  if (runtimeReady()) {
    fillTelemetrySnapshot(*g_runtime.context, *g_runtime.sensors,
                          *g_runtime.front_left, *g_runtime.front_right,
                          *g_runtime.rear_link, *g_runtime.claws,
                          *g_runtime.peg_finder_funnel_limit,
                          g_runtime.context->latest_imu_snapshot,
                          snapshot,
                          static_cast<robot::Milliseconds>(millis()));
  }
  return snapshot;
}

void handleRoot();
void handleStatus();
void handleTelemetry();
void handleImuSoakCountersReset();
void handleDiagnostics();
void handleDiagnosticsReset();
void handleDiagnosticsFreeze();
void handleStop();
void handleMode();
void handleDrive();
void handleMotor();
void handleInvert();
void handleSensors();
void handleLine();
void handleRearLine();
void handleHabitatPiecesStart();
void handleHabitatPiecesStop();
void handleHabitatPiecesConfig();
void handleHabitatPlacementStart();
void handleHabitatPlacementStop();
void handleHabitatPlacementConfig();
void handleAutonomousSolarStart();
void handleAutonomousSolarConfig();
void handleTowerPiecesStart();
void handleTowerPiecesConfig();
void handlePegFinderStart();
void handlePegFinderConfig();
void handleTimeTrialStart();
void handleTimeTrialConfig();
void handleImuTurnConfig();
void handleImuTurnStart();
void handleImuTurnStop();
void handleImuAngleReset();
void handleImuTurnSave();
void handleImuHeadingHoldConfig();
void handleImuHeadingHoldStart();
void handleImuHeadingHoldHeartbeat();
void handleImuHeadingHoldStop();
void handleImuHeadingHoldSave();
void handleLineFollowStart();
void handleLineFollowStop();
void handleLineFollowConfig();
void handleRearLineFollowConfig();
void handleRearLineFollowStart();
void handleRearLineFollowStop();
void handleClaw();
void handleClawsAll();
void handleWinch();
void handleHabitatPusher();
void handleClawsConfig();
void handleClawsSave();
void handleSolarHook();
void handleSolarHookConfig();
void handleSolarHookSave();
void handleFunnel();
void handleConfig();
void handleConfigSave();
void handleEvents();

const char kDashboardHtml[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Team14 Robot Test</title>
  <style>
    :root { color-scheme: dark; font-family: Arial, sans-serif; }
    body { margin: 0; background: #101316; color: #edf0f2; }
    header { position: sticky; top: 0; z-index: 2; display: flex; gap: 12px; align-items: center; justify-content: space-between; padding: 12px 16px; background: #181d21; border-bottom: 1px solid #303840; }
    h1 { font-size: 20px; margin: 0; }
    main { padding: 16px; display: grid; gap: 12px; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); }
    section { background: #1b2228; border: 1px solid #303a43; border-radius: 8px; padding: 12px; }
    h2 { margin: 0 0 10px; font-size: 16px; color: #dce7ee; }
    button, select, input { font: inherit; border-radius: 6px; border: 1px solid #53616d; background: #27313a; color: #fff; padding: 9px; }
    button { cursor: pointer; }
    .stop { background: #b4232f; border-color: #f05260; font-weight: 700; min-width: 120px; }
    .run { background: #1f6f53; border-color: #3aa277; }
    .warn { background: #795113; border-color: #b98325; }
    .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
    .two { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 8px; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin: 8px 0; }
    label { display: grid; gap: 4px; font-size: 13px; color: #c5d0d8; }
    .kv { display: grid; grid-template-columns: 1fr auto; gap: 6px 12px; font-size: 14px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .muted { color: #aeb9c2; }
    .bad { color: #ff9ca5; }
    .good { color: #8de0b8; }
    pre { white-space: pre-wrap; max-height: 300px; overflow: auto; background: #0b0e10; padding: 10px; border-radius: 6px; }
    input[type=number] { width: 86px; }
    .wide { grid-column: 1 / -1; }
  </style>
</head>
<body>
<header>
  <div>
    <h1>Team14 Robot Test</h1>
    <div class="muted">TEST ONLY - wheels up first</div>
  </div>
  <button class="stop" onclick="stopAll()">STOP</button>
</header>
<main>
  <section>
    <h2>Status</h2>
    <div class="kv">
      <span>Mode</span><span id="mode" class="mono"></span>
      <span>Fault</span><span id="fault" class="mono"></span>
      <span>ESP1 link</span><span id="link" class="mono"></span>
      <span>Uptime</span><span id="uptime" class="mono"></span>
      <span>AP</span><span id="ap" class="mono"></span>
      <span>Command age</span><span id="deadman" class="mono"></span>
    </div>
  </section>

  <section>
    <h2>Ultrasonic 1</h2>
    <div class="muted">HC-SR04 pins unassigned because GPIO11/GPIO12 now belong to LSS2/LSS3</div>
    <div class="kv">
      <span>Distance</span><span id="ultrasonic1Distance" class="mono"></span>
      <span>Status</span><span id="ultrasonic1Status" class="mono"></span>
      <span>Echo pulse</span><span id="ultrasonic1Echo" class="mono"></span>
      <span>Sample age</span><span id="ultrasonic1Age" class="mono"></span>
    </div>
  </section>

  <section>
    <h2>Laser distance</h2>
    <div class="muted">ESP1 VL53L0X V2 · SDA GPIO10 · SCL GPIO9</div>
    <div class="kv">
      <span>Distance</span><span id="laserDistance" class="mono"></span>
      <span>State</span><span id="laserState" class="mono"></span>
      <span>Profile</span><span id="laserProfile" class="mono"></span>
      <span>Range / driver status</span><span id="laserStatus" class="mono"></span>
      <span>Sample / snapshot age</span><span id="laserAge" class="mono"></span>
      <span>Measurement / packet sequence</span><span id="laserSequence" class="mono"></span>
      <span>Reads OK / failed / consecutive</span><span id="laserCounts" class="mono"></span>
      <span>Acquisition current / max</span><span id="laserTiming" class="mono"></span>
      <span>SDA / SCL / address</span><span id="laserBus" class="mono"></span>
    </div>
  </section>

  <section>
    <h2>Habitat Pieces</h2>
    <div class="muted">Follows the front line, aligns independently on LSS2/LSS3, reverses, and strafes while counting distinct above-threshold laser zones. At the target count it compensates in the opposite direction, lowers the slide to its bottom switch, then drives forward until a valid laser reading is at or below the pickup threshold. The slide lift starts with the timed reverse and continues concurrently while the robot strafes opposite the initial count direction until either rear line sensor sees black. Invalid/no-signal laser samples do not stop the forward approach; every search and open-loop drive remains timeout-bounded.</div>
    <div class="kv">
      <span>Configuration</span><span id="habitatConfig" class="mono"></span>
      <span>Start ready</span><span id="habitatReady" class="mono"></span>
      <span>State</span><span id="habitatState" class="mono"></span>
      <span>Side-line gate</span><span id="habitatGate" class="mono"></span>
      <span>LSS2 left input</span><span id="habitatLss2" class="mono"></span>
      <span>LSS3 right input</span><span id="habitatLss3" class="mono"></span>
      <span>Side alignment</span><span id="habitatAlign" class="mono"></span>
      <span>Detection delay</span><span id="habitatDelay" class="mono"></span>
      <span>Search elapsed / timeout</span><span id="habitatTimeout" class="mono"></span>
      <span>Reverse elapsed / duration</span><span id="habitatReverse" class="mono"></span>
      <span>Distance reading / threshold</span><span id="habitatDistance" class="mono"></span>
      <span>Distance-zone count</span><span id="habitatDistanceCount" class="mono"></span>
      <span>Distance strafe</span><span id="habitatDistanceStrafe" class="mono"></span>
      <span>Overshoot compensation</span><span id="habitatCompensation" class="mono"></span>
      <span>Slide down</span><span id="habitatSlideDown" class="mono"></span>
      <span>Forward pickup approach</span><span id="habitatForwardPickup" class="mono"></span>
      <span>Concurrent slide lift</span><span id="habitatSlideLift" class="mono"></span>
      <span>Post-pickup reverse</span><span id="habitatPostReverse" class="mono"></span>
      <span>Return rear-line strafe</span><span id="habitatReturnStrafe" class="mono"></span>
      <span>Line following</span><span id="habitatFollowing" class="mono"></span>
      <span>Last request</span><span id="habitatResult" class="mono muted">none</span>
    </div>
    <div class="two">
      <label>Front line-follow duty <input id="habitatDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>LSS2/LSS3 detection delay (ms) <input id="habitatDetectionDelay" type="number" min="1" step="100"></label>
      <label>Side-line search/alignment timeout (ms) <input id="habitatRunTimeout" type="number" min="1" step="100"></label>
      <label>Reverse duty <input id="habitatReverseDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Reverse duration (ms) <input id="habitatReverseDuration" type="number" min="1" step="100"></label>
      <label>Distance strafe direction <select id="habitatDistanceDirection"><option value="">Select</option><option value="LEFT">Left</option><option value="RIGHT">Right</option></select></label>
      <label>Distance threshold (mm) <input id="habitatDistanceThreshold" type="number" min="1" max="65535" step="1"></label>
      <label>Distance-zone target count <input id="habitatDistanceCountTarget" type="number" min="1" max="65535" step="1"></label>
      <label>Distance strafe duty <input id="habitatDistanceDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Distance strafe timeout (ms) <input id="habitatDistanceTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>Opposite compensation duty <input id="habitatCompensationDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Opposite compensation duration (ms) <input id="habitatCompensationDuration" type="number" min="1" max="30000" step="100"></label>
      <label>Slide-down speed (microsteps/s) <input id="habitatSlideDownSpeed" type="number" min="1" step="100"></label>
      <label>Slide-down timeout (ms) <input id="habitatSlideDownTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>Forward pickup duty <input id="habitatForwardDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Forward pickup stop distance (mm) <input id="habitatForwardDistance" type="number" min="1" max="65535" step="1"></label>
      <label>Forward pickup timeout (ms) <input id="habitatForwardTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>Slide lift (microsteps) <input id="habitatSlideLiftSteps" type="number" min="1" step="100"></label>
      <label>Slide-lift speed (microsteps/s) <input id="habitatSlideLiftSpeed" type="number" min="1" step="100"></label>
      <label>Slide-lift timeout (ms) <input id="habitatSlideLiftTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>Post-pickup reverse duty <input id="habitatPostReverseDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Post-pickup reverse duration (ms) <input id="habitatPostReverseDuration" type="number" min="1" max="30000" step="100"></label>
      <label>Return strafe duty <input id="habitatReturnDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Return rear-line timeout (ms) <input id="habitatReturnTimeout" type="number" min="1" max="30000" step="100"></label>
    </div>
    <div class="row">
      <button class="run" onclick="habitatStart()">Start</button>
      <button onclick="habitatApply()">Apply</button>
      <button onclick="habitatSave()">Save</button>
      <button class="stop" onclick="habitatStop()">Stop</button>
    </div>
  </section>

  <section>
    <h2>Habitat Placement</h2>
    <div class="muted">Standalone route intended to run after Habitat Pieces pickup: capture the initial IMU heading, rear-line follow to LSS1, return to the captured heading, timed right strafe, fixed-target CCW IMU turn, lower the slide, open/push/retreat, CW IMU turn, timed reverse, timed left strafe, then strafe right until either front line sensor sees black and close the pusher. Every motion has an adjustable bound.</div>
    <div class="kv">
      <span>Configuration</span><span id="placementConfig" class="mono"></span>
      <span>Start ready</span><span id="placementReady" class="mono"></span>
      <span>State</span><span id="placementState" class="mono"></span>
      <span>Fault reason</span><span id="placementFault" class="mono"></span>
      <span>Time in state</span><span id="placementTime" class="mono"></span>
      <span>Initial heading / CCW target</span><span id="placementCcwSaved" class="mono"></span>
      <span>Last request</span><span id="placementResult" class="mono muted">none</span>
    </div>
    <div class="two">
      <label>Rear line-follow duty <input id="placementRearDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>LSS1 timeout ms <input id="placementLssTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>Delay after LSS1 ms <input id="placementLssDelay" type="number" min="1" max="30000" step="100"></label>
      <label>Return-to-initial-heading timeout ms <input id="placementInitialHeadingTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>Pre-CCW strafe-right duty <input id="placementPreCcwStrafeDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Pre-CCW strafe-right ms <input id="placementPreCcwStrafeMs" type="number" min="1" max="30000" step="100"></label>
      <label>CCW angle deg <input id="placementCcwAngle" type="number" min="0.1" step="1"></label>
      <label>CCW timeout ms <input id="placementCcwTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>Forward-to-slide duty <input id="placementSlideDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Forward-to-slide ms <input id="placementSlideMs" type="number" min="1" max="30000" step="100"></label>
      <label>Slide-down speed µsteps/s <input id="placementStepperSpeed" type="number" min="1" step="100"></label>
      <label>Slide-down timeout ms <input id="placementStepperTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>Pusher-open settle ms <input id="placementPusherSettle" type="number" min="1" max="30000" step="100"></label>
      <label>Push-forward duty <input id="placementPushDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Push-forward ms <input id="placementPushMs" type="number" min="1" max="30000" step="100"></label>
      <label>Retreat duty <input id="placementRetreatDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Retreat ms <input id="placementRetreatMs" type="number" min="1" max="30000" step="100"></label>
      <label>CW angle deg <input id="placementCwAngle" type="number" min="0.1" step="1"></label>
      <label>CW timeout ms <input id="placementCwTimeout" type="number" min="1" max="30000" step="100"></label>
      <label>After-CW reverse duty <input id="placementAfterCwReverseDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>After-CW reverse ms <input id="placementAfterCwReverseMs" type="number" min="1" max="30000" step="100"></label>
      <label>After-CW strafe-left duty <input id="placementAfterCwStrafeLeftDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>After-CW strafe-left ms <input id="placementAfterCwStrafeLeftMs" type="number" min="1" max="30000" step="100"></label>
      <label>Delay after CW ms <input id="placementCwDelay" type="number" min="1" max="30000" step="100"></label>
      <label>Exit-forward duty <input id="placementExitDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Exit-forward ms <input id="placementExitMs" type="number" min="1" max="30000" step="100"></label>
      <label>Delay after exit ms <input id="placementExitDelay" type="number" min="1" max="30000" step="100"></label>
      <label>Strafe-right duty <input id="placementStrafeDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Strafe-right timeout ms <input id="placementStrafeTimeout" type="number" min="1" max="30000" step="100"></label>
    </div>
    <div class="row">
      <button class="run" onclick="placementStart()">Start</button>
      <button onclick="placementApply()">Apply</button>
      <button onclick="placementSave()">Save</button>
      <button class="stop" onclick="placementStop()">Stop</button>
    </div>
  </section>

  <section>
    <h2>IMU</h2>
    <div class="muted">ESP2 MPU-6050-compatible sensor · read-only 10 ms soak monitoring. Reset affects only soak counters.</div>
    <div class="kv">
      <span>Configured</span><span id="imuConfigured" class="mono"></span>
      <span>Initialized</span><span id="imuInitialized" class="mono"></span>
      <span>Calibrated</span><span id="imuCalibrated" class="mono"></span>
      <span>Acquisition task</span><span id="imuAcquisitionRunning" class="mono"></span>
      <span>Health</span><span id="imuHealth" class="mono"></span>
      <span>Disconnect reason</span><span id="imuDisconnectReason" class="mono"></span>
      <span>Last disconnect</span><span id="imuLastDisconnect" class="mono"></span>
      <span>Last read failure</span><span id="imuLastReadFailure" class="mono"></span>
      <span>Initialization result</span><span id="imuInitializationError" class="mono"></span>
      <span>SDA / SCL GPIO</span><span id="imuPins" class="mono"></span>
      <span>Device ACK</span><span id="imuDeviceAck" class="mono"></span>
      <span>Runtime register configuration</span><span id="imuRuntimeConfig" class="mono"></span>
      <span>Register-read mode</span><span id="imuReadMode" class="mono"></span>
      <span>Last Wire status</span><span id="imuWireStatus" class="mono"></span>
      <span>I2C address</span><span id="imuAddress" class="mono"></span>
      <span>WHO_AM_I</span><span id="imuWhoAmI" class="mono"></span>
      <span>Raw gyro Z</span><span id="imuRawGyroZ" class="mono"></span>
      <span>Gyro Z bias</span><span id="imuBias" class="mono"></span>
      <span>Yaw rate</span><span id="imuYawRate" class="mono"></span>
      <span>Heading</span><span id="imuHeading" class="mono"></span>
      <span>Sample age</span><span id="imuSampleAge" class="mono"></span>
      <span>Snapshot age</span><span id="imuSnapshotAge" class="mono"></span>
      <span>Acquisition duration</span><span id="imuAcquisitionDuration" class="mono"></span>
      <span>Maximum completed acquisition duration</span><span id="imuMaxAcquisitionDuration" class="mono"></span>
      <span>Total acquisition attempts</span><span id="imuTotalAttempts" class="mono"></span>
      <span>Last successful-read timestamp</span><span id="imuLastSuccessfulReadUs" class="mono"></span>
      <span>Sample interval</span><span id="imuSampleInterval" class="mono"></span>
      <span>Reads OK / failed (lifetime)</span><span id="imuReadCounts" class="mono"></span>
      <span>Consecutive failures</span><span id="imuConsecutiveFailures" class="mono"></span>
      <span>Acquisition loop interval current / max</span><span id="imuLoopTiming" class="mono"></span>
      <span>Queue sync current / max</span><span id="imuSyncTiming" class="mono"></span>
      <span>Wire lock acquisition current / max</span><span id="imuLockTiming" class="mono"></span>
      <span>I2C measurement read current / max</span><span id="imuReadTiming" class="mono"></span>
      <span>Successful read → publication current / max</span><span id="imuPublishTiming" class="mono"></span>
      <span>Snapshot queue publication current / max</span><span id="imuPublishQueueTiming" class="mono"></span>
      <span>Successful sample publication gap current / max</span><span id="imuPublicationGap" class="mono"></span>
      <span>Currently observed open publication gap / max</span><span id="imuObservedPublicationGap" class="mono"></span>
      <span>Snapshot / successful sample sequence</span><span id="imuPublicationSequence" class="mono"></span>
      <span>Delayed acquisition iterations</span><span id="imuDelayedIterations" class="mono"></span>
      <span>Autonomous recovery</span><span id="imuRecoveryState" class="mono"></span>
      <span>Saved turn / strafe heading</span><span id="imuRecoveryHeadings" class="mono"></span>
      <span>Recovery confirmations</span><span id="imuRecoveryConfirmations" class="mono"></span>
      <span>Last/current pause turn / strafe</span><span id="imuRecoveryTimes" class="mono"></span>
      <span>Recovery count / total paused</span><span id="imuRecoveryCounts" class="mono"></span>
    </div>
    <div class="row">
      <button onclick="imuSoakResetCounters()">Reset soak counters</button>
      <button onclick="imuSoakRefresh()">Refresh values</button>
      <span id="imuSoakResult" class="mono muted"></span>
    </div>
  </section>

  <section>
    <h2>IMU Turn Test</h2>
    <div class="muted">Stage 2 manual test only. Motion is locked until every field is valid, the IMU is healthy, and the ESP1 rear-wheel link is fresh. Start with wheels raised.</div>
    <div class="kv">
      <span>Configuration</span><span id="imuTurnConfigValid" class="mono"></span>
      <span>State</span><span id="imuTurnState" class="mono"></span>
      <span>Fault</span><span id="imuTurnFault" class="mono"></span>
      <span>Availability fault timing</span><span id="imuTurnFaultTiming" class="mono"></span>
      <span>Captured availability gate</span><span id="imuTurnAvailabilityGate" class="mono"></span>
      <span>Captured timestamps</span><span id="imuTurnAvailabilityTimes" class="mono"></span>
      <span>Captured publication gap</span><span id="imuTurnPublicationGap" class="mono"></span>
      <span>Cached snapshot vs newest</span><span id="imuTurnSnapshotIdentity" class="mono"></span>
      <span>Captured motors / link</span><span id="imuTurnAvailabilityLink" class="mono"></span>
      <span>Current angle</span><span id="imuTurnCurrentAngle" class="mono"></span>
      <span>Start / target</span><span id="imuTurnHeadings" class="mono"></span>
      <span>Angle error / yaw rate</span><span id="imuTurnErrorRate" class="mono"></span>
      <span>P / damping</span><span id="imuTurnTerms" class="mono"></span>
      <span>Rotation command</span><span id="imuTurnOutput" class="mono"></span>
      <span>Elapsed / settling</span><span id="imuTurnTime" class="mono"></span>
      <span>Last request</span><span id="imuTurnResult" class="mono muted">none</span>
    </div>
    <div class="two">
      <label>Maximum rotation duty <input id="imuTurnMaxDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Kp (duty/degree) <input id="imuTurnKp" type="number" min="0.00001" step="0.001"></label>
      <label>Kd (duty/(degree/second)) <input id="imuTurnKd" type="number" min="0" step="0.001"></label>
      <label>Angle tolerance (degrees) <input id="imuTurnTolerance" type="number" min="0.001" step="0.1"></label>
      <label>Maximum finishing yaw rate (degrees/second) <input id="imuTurnFinishRate" type="number" min="0.001" step="0.1"></label>
      <label>Required settling time (ms) <input id="imuTurnSettleMs" type="number" min="1" step="10"></label>
      <label>Overall turn timeout (ms) <input id="imuTurnTimeoutMs" type="number" min="2" max="30000" step="100"></label>
      <label>Yaw command polarity
        <select id="imuTurnPolarity">
          <option value="0">unconfigured</option>
          <option value="1">+1</option>
          <option value="-1">-1</option>
        </select>
      </label>
    </div>
    <div class="muted">Polarity maps positive IMU heading to drivetrain yaw. Verify it with wheels raised: if a +90 request initially turns heading negative, stop and reverse this value.</div>
    <div class="row">
      <button onclick="imuTurnApply()">Apply</button>
      <button onclick="imuTurnSave()">Save</button>
      <button class="run" onclick="imuTurnStart(90)">Turn +90°</button>
      <button class="run" onclick="imuTurnStart(-90)">Turn -90°</button>
      <button class="warn" onclick="imuAngleReset()">Reset angle</button>
      <button class="stop" onclick="imuTurnStop()">Stop</button>
    </div>
  </section>

  <section>
    <h2>IMU Heading-Held Strafe</h2>
    <div class="muted">Stage 3 manual test only. The target heading is captured once when a held strafe starts. Motion requires valid tuning, a healthy IMU, and a fresh ESP1 rear-wheel link. Start with wheels raised.</div>
    <div class="kv">
      <span>Configuration</span><span id="imuStrafeConfigValid" class="mono"></span>
      <span>State</span><span id="imuStrafeState" class="mono"></span>
      <span>Fault</span><span id="imuStrafeFault" class="mono"></span>
      <span>Direction</span><span id="imuStrafeDirection" class="mono"></span>
      <span>Current / target</span><span id="imuStrafeHeadings" class="mono"></span>
      <span>Angle error / yaw rate</span><span id="imuStrafeErrorRate" class="mono"></span>
      <span>P / damping</span><span id="imuStrafeTerms" class="mono"></span>
      <span>Yaw correction duty</span><span id="imuStrafeOutput" class="mono"></span>
      <span>Elapsed</span><span id="imuStrafeElapsed" class="mono"></span>
      <span>Last request</span><span id="imuStrafeResult" class="mono muted">none</span>
    </div>
    <div class="two">
      <label>Maximum strafe duty <input id="imuStrafeMaxDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Kp (duty/degree) <input id="imuStrafeKp" type="number" min="0.00001" step="0.001"></label>
      <label>Kd (duty/(degree/second)) <input id="imuStrafeKd" type="number" min="0" step="0.001"></label>
      <label>Maximum yaw correction duty <input id="imuStrafeMaxCorrection" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Yaw command polarity
        <select id="imuStrafePolarity">
          <option value="0">unconfigured</option>
          <option value="1">+1</option>
          <option value="-1">-1</option>
        </select>
      </label>
    </div>
    <div class="muted">Maximum strafe duty plus maximum yaw correction duty must not exceed the hardware duty cap. Use the same measured yaw polarity as the Stage 2 turn controller. Apply settings before holding a direction.</div>
    <div class="row">
      <button onclick="imuStrafeApply()">Apply</button>
      <button onclick="imuStrafeSave()">Save</button>
      <button class="run" onpointerdown="imuStrafeHold(-1,event)" onpointerup="imuStrafeRelease(event)" onpointercancel="imuStrafeRelease(event)" onlostpointercapture="imuStrafeRelease(event)">Hold Left</button>
      <button class="run" onpointerdown="imuStrafeHold(1,event)" onpointerup="imuStrafeRelease(event)" onpointercancel="imuStrafeRelease(event)" onlostpointercapture="imuStrafeRelease(event)">Hold Right</button>
      <button class="stop" onclick="imuStrafeStop()">Stop</button>
    </div>
  </section>

  <section>
    <h2>Autonomous Solar</h2>
    <div class="muted">Each strafe has an independent translational duty while sharing the IMU yaw-hold gains above. The final strafe stops when either rear sensor detects tape, then the existing backward line-follow PID runs for the configured duration.</div>
    <div class="kv">
      <span>State</span><span id="autoState" class="mono"></span>
      <span>Fault</span><span id="autoFault" class="mono"></span>
      <span>Time</span><span id="autoTime" class="mono"></span>
      <span>IR raw/filter</span><span id="autoIr" class="mono"></span>
      <span>Thresholds</span><span id="autoThresholds" class="mono"></span>
      <span>Confirm</span><span id="autoConfirm" class="mono"></span>
      <span>Slow mode</span><span id="autoSlow" class="mono"></span>
    </div>
    <div class="two">
      <label>1 kHz detect <input id="solarDetect1" type="number" min="0" max="65535" step="1"></label>
      <label>1 kHz release <input id="solarRelease1" type="number" min="0" max="65535" step="1"></label>
      <label>10 kHz detect <input id="solarDetect10" type="number" min="0" max="65535" step="1"></label>
      <label>10 kHz release <input id="solarRelease10" type="number" min="0" max="65535" step="1"></label>
      <label>Confirm ms <input id="solarConfirmMs" type="number" min="0" step="10"></label>
      <label>Filter alpha <input id="solarFilterAlpha" type="number" min="0" max="0.99" step="0.01"></label>
      <label>Ignore ms <input id="solarIgnoreMs" type="number" min="0" step="50"></label>
      <label>Timeout ms <input id="solarTimeoutMs" type="number" min="1" step="500"></label>
      <label>Start duty <input id="solarStartDuty" type="number" min="0" max="1" step="0.01"></label>
      <label>Slow after ms <input id="solarSlowAfterMs" type="number" min="0" step="500"></label>
      <label>Slow duty <input id="solarSlowDuty" type="number" min="0" max="1" step="0.01"></label>
      <label>Contact timeout ms <input id="solarContactTimeoutMs" type="number" min="1" step="500"></label>
      <label>Initial right strafe duty <input id="solarInitialStrafeDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Retry left strafe duty <input id="solarRetryLeftDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Retry right strafe duty <input id="solarRetryRightDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Strafe delay ms <input id="solarStrafeDelayMs" type="number" min="0" step="50"></label>
      <label>Retry forward duty <input id="solarStrafeDuty" type="number" min="0" max="1" step="0.01"></label>
      <label>Retry left strafe ms <input id="solarRetryLeftMs" type="number" min="0" step="50"></label>
      <label>Retry forward ms <input id="solarRetryForwardMs" type="number" min="0" step="50"></label>
      <label>Retry right timeout ms <input id="solarRetryStrafeTimeoutMs" type="number" min="1" step="500"></label>
      <label>Post-contact forward ms <input id="solarPostContactForwardMs" type="number" min="0" step="50"></label>
      <label>Post-contact forward duty <input id="solarPostContactForwardDuty" type="number" min="0" max="1" step="0.01"></label>
      <label>Post-contact forward delay ms <input id="solarPostContactForwardDelayMs" type="number" min="0" step="50"></label>
      <label>Post-forward rear-line delay ms <input id="solarPostForwardStrafeDelayMs" type="number" min="0" step="50"></label>
      <label>Rear-line strafe duty <input id="solarLineReacquireDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Backward PID duration ms <input id="solarRearLineFollowDurationMs" type="number" min="1" step="100"></label>
    </div>
    <div class="row">
      <button class="run" onclick="autoSolarStart()">Start</button>
      <button onclick="autoSolarApply()">Apply</button>
      <button onclick="autoSolarSave()">Save</button>
    </div>
  </section>

  <section>
    <h2>Solar Limits</h2>
    <div class="kv">
      <span>Configured</span><span id="solarLimitsConfigured" class="mono"></span>
      <span>Back right side</span><span id="solarLimitBackRight" class="mono"></span>
      <span>Front right side</span><span id="solarLimitFrontRight" class="mono"></span>
      <span>Both hit</span><span id="solarLimitsAll" class="mono"></span>
      <span>Timeout</span><span id="solarLimitTimeout" class="mono"></span>
      <span>Strafe delay</span><span id="solarLimitDelay" class="mono"></span>
      <span>IMU strafe duty (shared)</span><span id="solarLimitDuty" class="mono"></span>
      <span>Retry left</span><span id="solarRetryLeft" class="mono"></span>
      <span>Retry forward</span><span id="solarRetryForward" class="mono"></span>
      <span>Retry right timeout</span><span id="solarRetryTimeout" class="mono"></span>
      <span>Post-contact forward</span><span id="solarPostContactForward" class="mono"></span>
      <span>Post-contact forward duty</span><span id="solarPostContactForwardDutyStatus" class="mono"></span>
      <span>Rear-line reacquire IMU duty (shared)</span><span id="solarLineReacquireDutyStatus" class="mono"></span>
      <span>Backward PID duration</span><span id="solarRearLineFollowDuration" class="mono"></span>
      <span>Post-contact forward delay</span><span id="solarPostContactForwardDelay" class="mono"></span>
      <span>Post-forward rear-line delay</span><span id="solarPostForwardStrafeDelay" class="mono"></span>
    </div>
  </section>

  <section>
    <h2>Tower Pieces</h2>
    <div class="muted">The initial timed strafe and timed alternating shimmy have independent duties while sharing IMU yaw-hold gains. The shimmy stops when either rear sensor detects tape.</div>
    <div class="kv">
      <span>State</span><span id="towerState" class="mono"></span>
      <span>Fault</span><span id="towerFault" class="mono"></span>
      <span>Time</span><span id="towerTime" class="mono"></span>
      <span>Line following</span><span id="towerFollowing" class="mono"></span>
      <span>Side sensor</span><span id="towerSideSensor" class="mono"></span>
      <span>Side-line count</span><span id="towerSideCount" class="mono"></span>
      <span>Crossing gate</span><span id="towerCrossingGate" class="mono"></span>
      <span>Back line detected</span><span id="towerBackLineDetected" class="mono"></span>
      <span>Active output</span><span id="towerOutput" class="mono"></span>
      <span>Last command</span><span id="towerCommand" class="mono"></span>
    </div>
    <div class="two">
      <label>Reverse line-follow duty cycle <input id="towerDuty" type="number" min="0.01" max="1" step="0.01"></label>
      <label>Second-line timeout ms <input id="towerTimeoutMs" type="number" min="1" step="500"></label>
      <label>Right-sensor crossing cooldown ms <input id="towerSideCooldownMs" type="number" min="0" step="10"></label>
      <label>Right-sensor off-line re-arm ms <input id="towerSideRearmMs" type="number" min="0" step="10"></label>
      <label>Delay after second line ms <input id="towerPostLineDelayMs" type="number" min="1" step="100"></label>
      <label>Right strafe duration ms <input id="towerStrafeDurationMs" type="number" min="1" step="100"></label>
      <label>Right strafe duty <input id="towerStrafeDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Pause after strafe ms <input id="towerPostStrafePauseMs" type="number" min="1" step="100"></label>
      <label>Clockwise rotation angle (degrees) <input id="towerRotationAngleDeg" type="number" min="0.1" step="1"></label>
      <label>Pause after rotation ms <input id="towerPostRotationPauseMs" type="number" min="1" step="100"></label>
      <label>Timed backward duty cycle <input id="towerReverseDuty" type="number" min="0.01" max="1" step="0.01"></label>
      <label>Timed backward duration ms <input id="towerReverseDurationMs" type="number" min="1" step="100"></label>
      <label>Shimmy right duration ms <input id="towerShimmyRightMs" type="number" min="1" step="100"></label>
      <label>Shimmy left duration ms <input id="towerShimmyLeftMs" type="number" min="1" step="100"></label>
      <label>Shimmy search timeout ms <input id="towerShimmyTimeoutMs" type="number" min="1" step="500"></label>
      <label>Shimmy duty <input id="towerShimmyDuty" type="number" min="0.001" max="1" step="0.01"></label>
      <label>Optional final backward duty cycle <input id="towerFinalReverseDuty" type="number" min="0" max="1" step="0.01"></label>
      <label>Optional final backward duration ms (0 skips) <input id="towerFinalReverseDurationMs" type="number" min="0" step="100"></label>
      <label>Delay after final backward ms <input id="towerPostFinalReverseDelayMs" type="number" min="1" step="100"></label>
      <label>Delay after winch opens ms <input id="towerPostWinchOpenDelayMs" type="number" min="1" step="100"></label>
      <label>Delay after claws open ms <input id="towerPostClawsOpenDelayMs" type="number" min="1" step="100"></label>
      <label>Delay before slide moves down ms <input id="towerPreStepperBottomDelayMs" type="number" min="0" step="100"></label>
      <label>Stepper down speed (microsteps/s) <input id="towerStepperDownSpeed" type="number" min="1" max="200000" step="100"></label>
      <label>Delay at bottom ms <input id="towerPostStepperBottomDelayMs" type="number" min="1" step="100"></label>
      <label>Delay after claws close ms <input id="towerPostClawsClosedDelayMs" type="number" min="1" step="100"></label>
      <label>Stepper up speed (microsteps/s) <input id="towerStepperUpSpeed" type="number" min="1" max="200000" step="100"></label>
    </div>
    <div class="row">
      <button class="run" onclick="towerStart()">Start</button>
      <button onclick="towerApply()">Apply</button>
      <button onclick="towerSave()">Save</button>
      <button class="stop" onclick="stopAll()">Stop</button>
    </div>
  </section>

  <section>
    <h2>PegFinder</h2>
    <div class="muted">IMU-controlled clockwise turn, then timed backward and forward chassis movements, followed by funnel forward until the active-high GPIO 47 limit is pressed. The turn uses the shared IMU Turn tuning and the angle below. After a delay, the claws open in the selected order using the shared Servos panel angles. Once all three are open, PegFinder waits, then reverses the funnel for the configured duration.</div>
    <div class="kv">
      <span>State</span><span id="pegFinderState" class="mono"></span>
      <span>Fault</span><span id="pegFinderFault" class="mono"></span>
      <span>Time</span><span id="pegFinderTime" class="mono"></span>
      <span>Funnel limit GPIO 47</span><span id="pegFinderFunnelLimit" class="mono"></span>
      <span>Active output</span><span id="pegFinderOutput" class="mono"></span>
      <span>Last command</span><span id="pegFinderCommand" class="mono"></span>
    </div>
    <div class="two">
      <label>Clockwise turn angle (degrees) <input id="pegFinderClockwiseAngleDeg" type="number" min="0.1" step="1"></label>
      <label>Pause after clockwise ms <input id="pegFinderPostRotationPauseMs" type="number" min="1" step="100"></label>
      <label>Backward duty cycle <input id="pegFinderReverseDuty" type="number" min="0.01" max="1" step="0.01"></label>
      <label>Backward duration ms <input id="pegFinderReverseDurationMs" type="number" min="1" step="100"></label>
      <label>Pause after backward ms <input id="pegFinderPostReversePauseMs" type="number" min="1" step="100"></label>
      <label>Forward duty cycle <input id="pegFinderForwardDuty" type="number" min="0.01" max="1" step="0.01"></label>
      <label>Forward duration ms <input id="pegFinderForwardDurationMs" type="number" min="1" step="100"></label>
      <label>Funnel forward duty cycle <input id="pegFinderFunnelDuty" type="number" min="0.01" max="1" step="0.01"></label>
      <label>Funnel limit timeout ms <input id="pegFinderFunnelTimeoutMs" type="number" min="1" step="100"></label>
      <label>Delay after funnel limit ms <input id="pegFinderPostFunnelLimitDelayMs" type="number" min="1" step="100"></label>
      <label>Delay between claw openings ms <input id="pegFinderClawOpenIntervalMs" type="number" min="1" step="100"></label>
      <label>First claw to open <select id="pegFinderClawOrder1"><option value="1">Claw 1</option><option value="2">Claw 2</option><option value="3">Claw 3</option></select></label>
      <label>Second claw to open <select id="pegFinderClawOrder2"><option value="1">Claw 1</option><option value="2">Claw 2</option><option value="3">Claw 3</option></select></label>
      <label>Third claw to open <select id="pegFinderClawOrder3"><option value="1">Claw 1</option><option value="2">Claw 2</option><option value="3">Claw 3</option></select></label>
      <label>Delay after all claws open ms <input id="pegFinderPostClawsOpenDelayMs" type="number" min="1" step="100"></label>
      <label>Funnel reverse duty cycle <input id="pegFinderFunnelReverseDuty" type="number" min="0.01" max="1" step="0.01"></label>
      <label>Funnel reverse duration ms <input id="pegFinderFunnelReverseDurationMs" type="number" min="1" step="100"></label>
    </div>
    <div class="row">
      <button class="run" onclick="pegFinderStart()">Start</button>
      <button onclick="pegFinderApply()">Apply</button>
      <button onclick="pegFinderSave()">Save</button>
      <button class="stop" onclick="stopAll()">Stop</button>
    </div>
  </section>

  <section>
    <h2>Time Trial</h2>
    <div class="muted">Runs Autonomous Solar until the rear tape is detected, then hands the active reverse line follower directly to Tower Pieces before running PegFinder. The combined Solar-to-Tower line-follow speed is the Reverse line-follow duty cycle in the Tower Pieces panel.</div>
    <div class="kv">
      <span>State</span><span id="timeTrialState" class="mono"></span>
      <span>Time</span><span id="timeTrialTime" class="mono"></span>
      <span>Active output</span><span id="timeTrialOutput" class="mono"></span>
      <span>Last command</span><span id="timeTrialCommand" class="mono"></span>
    </div>
    <div class="two">
      <label>Legacy delay after solar ms (bypassed) <input id="timeTrialPostSolarDelayMs" type="number" min="0" step="100" disabled></label>
      <label>Legacy solar-to-tower strafe ms (bypassed) <input id="timeTrialStrafeDurationMs" type="number" min="0" step="100" disabled></label>
      <label>Delay after tower pieces ms <input id="timeTrialPostTowerDelayMs" type="number" min="0" step="100"></label>
    </div>
    <div class="row">
      <button class="run" onclick="timeTrialStart()">Start</button>
      <button onclick="timeTrialApply()">Apply</button>
      <button onclick="timeTrialSave()">Save</button>
      <button class="stop" onclick="stopAll()">Stop</button>
    </div>
  </section>

  <section>
    <h2>IR Beacon</h2>
    <div class="kv">
      <span>Selected Hz</span><span id="irSelectedHz" class="mono"></span>
      <span>Amplitude</span><span id="irAmp" class="mono"></span>
      <span>1 kHz</span><span id="ir1k" class="mono"></span>
      <span>10 kHz</span><span id="ir10k" class="mono"></span>
      <span>Selected amp</span><span id="irSelectedAmp" class="mono"></span>
    </div>
  </section>

  <section>
    <h2>Front Line Following</h2>
    <div class="kv">
      <span>Enabled</span><span id="lfEnabled" class="mono"></span>
      <span>LSFL</span><span id="lsfl" class="mono"></span>
      <span>LSFR</span><span id="lsfr" class="mono"></span>
      <span>LSS (side)</span><span id="lss" class="mono"></span>
      <span>LSS2 (side, ESP1 GPIO11)</span><span id="lss2" class="mono"></span>
      <span>LSS3 (side, ESP1 GPIO12)</span><span id="lss3" class="mono"></span>
      <span>Error</span><span id="lfError" class="mono"></span>
      <span>PID</span><span id="lfPid" class="mono"></span>
    </div>
    <div class="two">
      <label>Kp <input id="lfKp" type="number" step="0.01"></label>
      <label>Ki <input id="lfKi" type="number" step="0.01"></label>
      <label>Kd <input id="lfKd" type="number" step="0.01"></label>
      <label>Base <input id="lfBase" type="number" step="0.01"></label>
      <label>Max Duty <input id="lfMaxDuty" type="number" step="0.01"></label>
      <label>Max Corr <input id="lfMaxCorrection" type="number" step="0.01"></label>
      <label>I Limit <input id="lfIntegralLimit" type="number" step="0.01"></label>
      <label>D Limit <input id="lfDerivativeLimit" type="number" step="0.1"></label>
      <label>D Alpha <input id="lfDerivativeAlpha" type="number" step="0.01"></label>
      <label>Polarity <select id="lfPolarity"><option value="1">+1</option><option value="-1">-1</option></select></label>
      <label>Duration ms <input id="lfDuration" type="number" min="1" max="5000" step="100" value="5000"></label>
      <label>Telemetry <select id="lfTelemetry"><option value="on">on</option><option value="off">off</option></select></label>
    </div>
    <div class="row">
      <button onclick="lineSensorMode()">Sensor Test</button>
      <button class="run" onclick="lfStart()">Start</button>
      <button class="stop" onclick="lfStop()">Stop</button>
      <button onclick="lfApply()">Apply</button>
      <button onclick="lfSave()">Save</button>
    </div>
  </section>

  <section>
    <h2>Reverse Rear Line Following</h2>
    <div class="muted">Reverse travel: LSBR is logical left; LSBL is logical right.</div>
    <div class="kv">
      <span>Enabled</span><span id="rlfEnabled" class="mono"></span>
      <span>LSBL (physical)</span><span id="lsbl" class="mono"></span>
      <span>LSBR (physical)</span><span id="lsbr" class="mono"></span>
      <span>Travel mapping</span><span id="rlfMapping" class="mono"></span>
      <span>Sensor stream</span><span id="rearLineStream" class="mono"></span>
      <span>Error</span><span id="rlfError" class="mono"></span>
      <span>PID</span><span id="rlfPid" class="mono"></span>
      <span>Effective base</span><span id="rlfEffectiveBase" class="mono"></span>
    </div>
    <div class="two">
      <label>Kp <input id="rlfKp" type="number" step="0.01"></label>
      <label>Ki <input id="rlfKi" type="number" step="0.01"></label>
      <label>Kd <input id="rlfKd" type="number" step="0.01"></label>
      <label>Base magnitude (reverse) <input id="rlfBase" type="number" min="0" step="0.01"></label>
      <label>Max Duty <input id="rlfMaxDuty" type="number" step="0.01"></label>
      <label>Max Corr <input id="rlfMaxCorrection" type="number" step="0.01"></label>
      <label>I Limit <input id="rlfIntegralLimit" type="number" step="0.01"></label>
      <label>D Limit <input id="rlfDerivativeLimit" type="number" step="0.1"></label>
      <label>D Alpha <input id="rlfDerivativeAlpha" type="number" step="0.01"></label>
      <label>Polarity <select id="rlfPolarity"><option value="1">+1</option><option value="-1">-1</option></select></label>
      <label>Duration ms <input id="rlfDuration" type="number" min="1" max="5000" step="100" value="5000"></label>
      <label>Telemetry <select id="rlfTelemetry"><option value="on">on</option><option value="off">off</option></select></label>
    </div>
    <div class="row">
      <button onclick="rearLineSensorMode()">Rear Sensor Test</button>
      <button class="run" onclick="rlfStart()">Start Reverse</button>
      <button class="stop" onclick="rlfStop()">Stop</button>
      <button onclick="rlfApply()">Apply</button>
      <button onclick="rlfSave()">Save</button>
    </div>
  </section>

  <section>
    <h2>Drive</h2>
    <div class="row">Duty <input id="driveDuty" type="number" min="0" max="1" step="0.01" value="0.30"></div>
    <div class="grid">
      <span></span><button onpointerdown="drive(0,1,0)" onpointerup="stopAll()" onpointerleave="stopAll()" onpointercancel="stopAll()">FWD</button><span></span>
      <button onpointerdown="drive(-1,0,0)" onpointerup="stopAll()" onpointerleave="stopAll()" onpointercancel="stopAll()">LEFT</button><button class="stop" onclick="stopAll()">STOP</button><button onpointerdown="drive(1,0,0)" onpointerup="stopAll()" onpointerleave="stopAll()" onpointercancel="stopAll()">RIGHT</button>
      <button onpointerdown="drive(0,0,-1)" onpointerup="stopAll()" onpointerleave="stopAll()" onpointercancel="stopAll()">CCW</button><button onpointerdown="drive(0,-1,0)" onpointerup="stopAll()" onpointerleave="stopAll()" onpointercancel="stopAll()">BACK</button><button onpointerdown="drive(0,0,1)" onpointerup="stopAll()" onpointerleave="stopAll()" onpointercancel="stopAll()">CW</button>
    </div>
  </section>

  <section>
    <h2>Single Wheel</h2>
    <div class="row">
      Wheel <select id="motorId"><option>FL</option><option>FR</option><option>BL</option><option>BR</option></select>
      Duty <input id="motorSpeed" type="number" min="0" max="1" step="0.01" value="0.30">
    </div>
    <div class="row">
      <button class="run" onpointerdown="motorHold(1)" onpointerup="motorRelease()" onpointerleave="motorRelease()" onpointercancel="motorRelease()">Hold Forward</button>
      <button class="run" onpointerdown="motorHold(-1)" onpointerup="motorRelease()" onpointerleave="motorRelease()" onpointercancel="motorRelease()">Hold Back</button>
      <button class="warn" onclick="invert()">Invert</button>
    </div>
    <pre id="motors"></pre>
  </section>

	  <section>
	    <h2>Vertical Stepper (DRV8425)</h2>
    <div class="kv">
      <span>Position</span><span id="stepperPosition" class="mono"></span>
      <span>Motion</span><span id="stepperMotion" class="mono"></span>
      <span>Speed</span><span id="stepperSpeedNow" class="mono"></span>
      <span>Driver</span><span id="stepperSleep" class="mono"></span>
      <span>Bottom limit</span><span id="stepperBottomLimit" class="mono"></span>
      <span>Top limit</span><span id="stepperTopLimit" class="mono"></span>
    </div>
    <div class="two">
      <label>Up/down speed (µsteps/s) <input id="stepperSpeed" type="number" min="1" max="200000" value="800" oninput="stepperSpeedsDirty()"></label>
      <label>Go-to-limit speed (µsteps/s) <input id="stepperLimitSpeed" type="number" min="1" max="200000" value="200" oninput="stepperSpeedsDirty()"></label>
    </div>
    <div class="row">
      <button class="run" style="touch-action:none" onpointerdown="stepperHold('up')" onpointerup="stepperRelease()" onpointerleave="stepperRelease()" onpointercancel="stepperRelease()">Hold Up</button>
      <button class="run" style="touch-action:none" onpointerdown="stepperHold('down')" onpointerup="stepperRelease()" onpointerleave="stepperRelease()" onpointercancel="stepperRelease()">Hold Down</button>
      <button onclick="stepperCommand('bottom')">Go to Bottom</button>
      <button onclick="stepperCommand('top')">Go to Top</button>
      <button class="stop" onclick="stepperCommand('stop')">Stop</button>
      <button onclick="stepperApplySpeeds()">Apply Speeds</button>
      <span id="stepperSpeedStatus" class="mono muted"></span>
    </div>
  </section>

	  <section>
	    <h2>Servos</h2>
    <div class="kv">
      <span>Claw hardware</span><span id="clawHardware" class="mono"></span>
      <span>Claw commands</span><span id="clawCommanded" class="mono"></span>
      <span>Winch hardware</span><span id="winchHardware" class="mono"></span>
      <span>Winch command</span><span id="winchCommanded" class="mono"></span>
      <span>Habitat Pusher hardware</span><span id="pusherHardware" class="mono"></span>
      <span>Habitat Pusher command</span><span id="pusherCommanded" class="mono"></span>
    </div>
    <div class="two">
      <label>Claw 1 open angle <input id="claw1Open" type="number" min="0" max="180" step="1"></label>
      <label>Claw 1 closed angle <input id="claw1Closed" type="number" min="0" max="180" step="1"></label>
      <label>Claw 2 open angle <input id="claw2Open" type="number" min="0" max="180" step="1"></label>
      <label>Claw 2 closed angle <input id="claw2Closed" type="number" min="0" max="180" step="1"></label>
      <label>Claw 3 open angle <input id="claw3Open" type="number" min="0" max="180" step="1"></label>
      <label>Claw 3 closed angle <input id="claw3Closed" type="number" min="0" max="180" step="1"></label>
      <label>Habitat Pusher open angle <input id="pusherOpen" type="number" min="0" max="180" step="1"></label>
      <label>Habitat Pusher closed angle <input id="pusherClosed" type="number" min="0" max="180" step="1"></label>
      <label>Winch open angle <input id="winchOpen" type="number" min="0" max="180" step="1"></label>
      <label>Winch closed angle <input id="winchClosed" type="number" min="0" max="180" step="1"></label>
    </div>
    <div class="row">
      <button onclick="clawsApply()">Apply</button>
      <button onclick="clawsSave()">Save</button>
      <button onclick="clawAll('close')">Close All</button>
      <button class="run" onclick="clawAll('open')">Open All</button>
    </div>
    <div class="row">
      <span>Claw 1</span><button onclick="claw(1,'close')">Close</button><button class="run" onclick="claw(1,'open')">Open</button>
      <span>Claw 2</span><button onclick="claw(2,'close')">Close</button><button class="run" onclick="claw(2,'open')">Open</button>
      <span>Claw 3</span><button onclick="claw(3,'close')">Close</button><button class="run" onclick="claw(3,'open')">Open</button>
      <span>Habitat Pusher</span><button onclick="habitatPusher('close')">Close</button><button class="run" onclick="habitatPusher('open')">Open</button>
      <span>Winch</span><button onclick="winch('close')">Close</button><button class="run" onclick="winch('open')">Open</button>
    </div>
    <div class="muted">Habitat Pusher: ESP2 GPIO5 / LEDC 7. Winch: ESP2 GPIO6 / MCPWM unit 0, timer 0, generator A. Both start disabled. Apply both pusher angles before Habitat Placement and calibrate close → open for clockwise arm rotation.</div>
	    <pre id="claws"></pre>
	  </section>

  <section>
    <h2>Solar Hook Servo Control</h2>
    <div class="muted">ESP1 GPIO3 · LEDC channel 6 · output starts disabled</div>
    <div class="kv">
      <span>ESP1 hardware</span><span id="solarHookHardware" class="mono"></span>
      <span>Output</span><span id="solarHookOutput" class="mono"></span>
      <span>Command</span><span id="solarHookCommandStatus" class="mono"></span>
    </div>
    <div class="two">
      <label>Open angle <input id="solarHookOpen" type="number" min="0" max="180" step="1"></label>
      <label>Closed angle <input id="solarHookClosed" type="number" min="0" max="180" step="1"></label>
    </div>
    <div class="row">
      <button onclick="solarHookApply()">Apply</button>
      <button onclick="solarHookSave()">Save</button>
      <button class="run" onclick="solarHookCommand('open')">Open</button>
      <button onclick="solarHookCommand('close')">Close</button>
      <button class="stop" onclick="solarHookCommand('disable')">Disable</button>
      <span id="solarHookResult" class="mono muted"></span>
    </div>
  </section>

	  <section>
	    <h2>Funnel Motor</h2>
	    <div class="kv">
	      <span>Configured</span><span id="funnelConfigured" class="mono"></span>
	      <span>Desired</span><span id="funnelDesired" class="mono"></span>
	      <span>Applied</span><span id="funnelApplied" class="mono"></span>
	    </div>
	    <div class="row">
	      Duty <input id="funnelSpeed" type="number" min="0" max="1" step="0.01" value="0.25">
	    </div>
	    <div class="row">
	      <button class="run" onpointerdown="funnelHold(1)" onpointerup="funnelRelease()" onpointerleave="funnelRelease()" onpointercancel="funnelRelease()">Hold Forward</button>
	      <button class="run" onpointerdown="funnelHold(-1)" onpointerup="funnelRelease()" onpointerleave="funnelRelease()" onpointercancel="funnelRelease()">Hold Reverse</button>
	    </div>
	    <pre id="funnel"></pre>
	  </section>

  <section class="wide">
    <h2>Motion Failure Diagnostics</h2>
    <div class="muted">Reset before a raised-wheel reproduction. The trace is collected only while motion is active and is not polled automatically.</div>
    <div class="row">
      <button onclick="diagnosticsReset()">Reset before run</button>
      <button onclick="diagnosticsRefresh()">Refresh after failure</button>
      <button class="warn" onclick="diagnosticsFreeze()">Freeze capture</button>
    </div>
    <pre id="motionDiagnostics">No diagnostic report loaded.</pre>
  </section>

  <section class="wide">
	    <h2>Recent Events</h2>
	    <pre id="events"></pre>
	  </section>
</main>
<script>
let holdTimer = null;
let lfLoaded = false;
let rlfLoaded = false;
let clawsLoaded = false;
let solarHookLoaded = false;
let solarLoaded = false;
let habitatLoaded = false;
let placementLoaded = false;
let towerLoaded = false;
let pegFinderLoaded = false;
let timeTrialLoaded = false;
let imuTurnLoaded = false;
let imuStrafeLoaded = false;
let imuStrafeHeartbeat = null;
let imuStrafeHeartbeatPending = false;
let imuStrafePressed = false;
let imuStrafeGeneration = 0;
let imuStrafePointerId = null;
let imuStrafeRequestChain = Promise.resolve();
function qs(id){ return document.getElementById(id); }
function api(path){ return fetch(path).then(r => r.json().catch(() => ({})).then(j => ({ok:r.ok, status:r.status, json:j}))); }
function stepperCommand(command, extra=''){ return api(`/api/stepper/command?command=${command}${extra}`); }
let stepperHeartbeat = null;
let stepperSettingsLoaded = false;
function stepperHold(direction){
  stepperRelease(false);
  stepperCommand(direction);
  stepperHeartbeat = setInterval(() => stepperCommand('hold'), 150);
}
function stepperRelease(sendStop=true){
  if(stepperHeartbeat){ clearInterval(stepperHeartbeat); stepperHeartbeat=null; }
  if(sendStop) stepperCommand('stop');
}
function stepperSpeedsDirty(){
  stepperSettingsLoaded=true;
  qs('stepperSpeedStatus').textContent='not applied';
  qs('stepperSpeedStatus').className='mono muted';
}
async function stepperApplySpeeds(){
  const p=new URLSearchParams({
    command:'config',
    speed:qs('stepperSpeed').value,
    limitSpeed:qs('stepperLimitSpeed').value
  });
  const result=await api(`/api/stepper/command?${p.toString()}`);
  const status=qs('stepperSpeedStatus');
  if(result.ok){
    status.textContent='applied';
    status.className='mono good';
    stepperSettingsLoaded=true;
  }else{
    status.textContent=result.json.error || `rejected (${result.status})`;
    status.className='mono bad';
  }
  return result;
}
function updateStepper(){ fetch('/api/stepper').then(r=>r.json()).then(s=>{
  qs('stepperPosition').textContent=`${s.positionSteps} µsteps`;
  qs('stepperMotion').textContent=s.motionState;
  qs('stepperSpeedNow').textContent=`${s.speedStepsPerSecond} µsteps/s`;
  qs('stepperSleep').textContent=s.sleeping?'asleep':'awake / holding';
  qs('stepperBottomLimit').textContent=s.lowerLimitActive?'PRESSED':'released';
  qs('stepperBottomLimit').className=s.lowerLimitActive?'mono bad':'mono good';
  qs('stepperTopLimit').textContent=s.upperLimitActive?'PRESSED':'released';
  qs('stepperTopLimit').className=s.upperLimitActive?'mono bad':'mono good';
  if(!stepperSettingsLoaded){
    qs('stepperSpeed').value=s.configuredSpeedStepsPerSecond;
    qs('stepperLimitSpeed').value=s.limitSearchSpeedStepsPerSecond;
    stepperSettingsLoaded=true;
  }
}).catch(()=>{ qs('stepperMotion').textContent='disconnected'; }); }
function stopAll(){
  if (holdTimer) clearInterval(holdTimer);
  holdTimer=null;
  ++imuStrafeGeneration;
  imuStrafePressed=false;
  if (imuStrafeHeartbeat) clearInterval(imuStrafeHeartbeat);
  imuStrafeHeartbeat=null;
  imuStrafeHeartbeatPending=false;
  imuStrafePointerId=null;
  api('/api/stop');
}
function drive(vx,vy,wz){
  const duty = Number(qs('driveDuty').value || 0);
  const send = start => api(`/api/drive?vx=${vx}&vy=${vy}&wz=${wz}&duty=${duty}&hold-start=${start ? 1 : 0}`);
  send(true); if (holdTimer) clearInterval(holdTimer); holdTimer=setInterval(() => send(false), 200);
}
function diagnosticsRefresh(){
  return fetch('/api/diagnostics').then(r => r.json()).then(j => {
    qs('motionDiagnostics').textContent = JSON.stringify(j, null, 2);
    return j;
  }).catch(() => { qs('motionDiagnostics').textContent = 'diagnostic request failed'; });
}
function diagnosticsReset(){
  return api('/api/diagnostics/reset').then(r => {
    qs('motionDiagnostics').textContent =
      r.ok ? 'Capture reset. Run one test, then press Refresh after failure.' :
             (r.json.error || `reset rejected (${r.status})`);
    return r;
  });
}
function diagnosticsFreeze(){
  return api('/api/diagnostics/freeze').then(() => diagnosticsRefresh());
}
function motorCommand(speed){ api(`/api/motor?id=${qs('motorId').value}&speed=${speed}`); }
function motorHold(sign){
  const speed = Math.abs(Number(qs('motorSpeed').value || 0)) * sign;
  const send = () => motorCommand(speed);
  send(); if (holdTimer) clearInterval(holdTimer); holdTimer=setInterval(send, 200);
}
function motorRelease(){ if (holdTimer) clearInterval(holdTimer); holdTimer=null; motorCommand(0); }
function invert(){ api(`/api/invert?id=${qs('motorId').value}`); }
function funnelCommand(speed){ api(`/api/funnel?speed=${speed}`); }
function funnelHold(sign){
  const speed = Math.abs(Number(qs('funnelSpeed').value || 0)) * sign;
  const send = () => funnelCommand(speed);
  send(); if (holdTimer) clearInterval(holdTimer); holdTimer=setInterval(send, 200);
}
function funnelRelease(){ if (holdTimer) clearInterval(holdTimer); holdTimer=null; funnelCommand(0); }
function setLfValue(id, value){ const el = qs(id); if (el && document.activeElement !== el) el.value = value; }
function loadHabitatControls(j){
  if (habitatLoaded || !j.habitat_pieces) return;
  const h = j.habitat_pieces;
  setLfValue('habitatDuty', h.line_follow_duty);
  setLfValue('habitatDetectionDelay', h.lss2_detection_delay_ms || '');
  setLfValue('habitatRunTimeout', h.run_timeout_ms || '');
  setLfValue('habitatReverseDuty', h.reverse_duty || '');
  setLfValue('habitatReverseDuration', h.reverse_duration_ms || '');
  setLfValue('habitatDistanceDirection',
             h.distance_strafe_direction === 'NONE'
               ? '' : h.distance_strafe_direction);
  setLfValue('habitatDistanceThreshold', h.distance_threshold_mm || '');
  setLfValue('habitatDistanceCountTarget',
             h.distance_zone_target_count || '');
  setLfValue('habitatDistanceDuty', h.distance_strafe_duty || '');
  setLfValue('habitatDistanceTimeout',
             h.distance_strafe_timeout_ms || '');
  setLfValue('habitatCompensationDuty',
             h.compensation_strafe_duty || '');
  setLfValue('habitatCompensationDuration',
             h.compensation_strafe_duration_ms || '');
  setLfValue('habitatSlideDownSpeed',
             h.slide_down_speed_steps_per_second || '');
  setLfValue('habitatSlideDownTimeout',
             h.slide_down_timeout_ms || '');
  setLfValue('habitatForwardDuty',
             h.forward_to_distance_duty || '');
  setLfValue('habitatForwardDistance',
             h.forward_stop_distance_mm || '');
  setLfValue('habitatForwardTimeout',
             h.forward_to_distance_timeout_ms || '');
  setLfValue('habitatSlideLiftSteps', h.slide_lift_steps || '');
  setLfValue('habitatSlideLiftSpeed',
             h.slide_lift_speed_steps_per_second || '');
  setLfValue('habitatSlideLiftTimeout',
             h.slide_lift_timeout_ms || '');
  setLfValue('habitatPostReverseDuty',
             h.post_pickup_reverse_duty || '');
  setLfValue('habitatPostReverseDuration',
             h.post_pickup_reverse_duration_ms || '');
  setLfValue('habitatReturnDuty', h.return_strafe_duty || '');
  setLfValue('habitatReturnTimeout', h.return_line_timeout_ms || '');
  habitatLoaded = true;
}
function habitatParams(){
  return new URLSearchParams({
    duty: qs('habitatDuty').value,
    'lss2-detection-delay-ms': qs('habitatDetectionDelay').value,
    'run-timeout-ms': qs('habitatRunTimeout').value,
    'reverse-duty': qs('habitatReverseDuty').value,
    'reverse-duration-ms': qs('habitatReverseDuration').value,
    'distance-strafe-direction': qs('habitatDistanceDirection').value,
    'distance-threshold-mm': qs('habitatDistanceThreshold').value,
    'distance-zone-count': qs('habitatDistanceCountTarget').value,
    'distance-strafe-duty': qs('habitatDistanceDuty').value,
    'distance-strafe-timeout-ms': qs('habitatDistanceTimeout').value,
    'compensation-strafe-duty': qs('habitatCompensationDuty').value,
    'compensation-strafe-duration-ms': qs('habitatCompensationDuration').value,
    'slide-down-speed-steps-per-second': qs('habitatSlideDownSpeed').value,
    'slide-down-timeout-ms': qs('habitatSlideDownTimeout').value,
    'forward-to-distance-duty': qs('habitatForwardDuty').value,
    'forward-stop-distance-mm': qs('habitatForwardDistance').value,
    'forward-to-distance-timeout-ms': qs('habitatForwardTimeout').value,
    'slide-lift-steps': qs('habitatSlideLiftSteps').value,
    'slide-lift-speed-steps-per-second': qs('habitatSlideLiftSpeed').value,
    'slide-lift-timeout-ms': qs('habitatSlideLiftTimeout').value,
    'post-pickup-reverse-duty': qs('habitatPostReverseDuty').value,
    'post-pickup-reverse-duration-ms': qs('habitatPostReverseDuration').value,
    'return-strafe-duty': qs('habitatReturnDuty').value,
    'return-line-timeout-ms': qs('habitatReturnTimeout').value
  });
}
function habitatShowResult(result){
  const message = result.json.message || result.json.error ||
    `request ${result.status}`;
  qs('habitatResult').textContent = message;
  qs('habitatResult').className = result.ok ? 'mono good' : 'mono bad';
  return result;
}
function habitatApply(){
  return api(`/api/autonomous/habitat-pieces/config?${habitatParams().toString()}`)
    .then(habitatShowResult);
}
async function habitatStart(){
  const applied = await habitatApply();
  if (!applied.ok) return applied;
  return api('/api/autonomous/habitat-pieces/start')
    .then(habitatShowResult);
}
async function habitatSave(){
  const applied = await habitatApply();
  if (!applied.ok) return applied;
  return api('/api/config/save').then(habitatShowResult);
}
function habitatStop(){
  return api('/api/autonomous/habitat-pieces/stop')
    .then(habitatShowResult);
}
function loadPlacementControls(j){
  if (placementLoaded || !j.habitat_placement) return;
  const p = j.habitat_placement;
  const values = {
    placementRearDuty:p.reverse_line_follow_duty,
    placementLssTimeout:p.lss1_timeout_ms,
    placementLssDelay:p.post_lss1_delay_ms,
    placementInitialHeadingTimeout:p.initial_heading_turn_timeout_ms,
    placementPreCcwStrafeDuty:p.pre_counter_clockwise_strafe_right_duty,
    placementPreCcwStrafeMs:p.pre_counter_clockwise_strafe_right_duration_ms,
    placementCcwAngle:p.counter_clockwise_angle_deg,
    placementCcwTimeout:p.counter_clockwise_timeout_ms,
    placementSlideDuty:p.forward_to_slide_duty,
    placementSlideMs:p.forward_to_slide_duration_ms,
    placementStepperSpeed:p.stepper_down_speed_steps_per_second,
    placementStepperTimeout:p.stepper_down_timeout_ms,
    placementPusherSettle:p.pusher_open_settle_ms,
    placementPushDuty:p.push_forward_duty,
    placementPushMs:p.push_forward_duration_ms,
    placementRetreatDuty:p.reverse_retreat_duty,
    placementRetreatMs:p.reverse_retreat_duration_ms,
    placementCwAngle:p.clockwise_angle_deg,
    placementCwTimeout:p.clockwise_timeout_ms,
    placementAfterCwReverseDuty:p.post_clockwise_reverse_duty,
    placementAfterCwReverseMs:p.post_clockwise_reverse_duration_ms,
    placementAfterCwStrafeLeftDuty:p.post_clockwise_strafe_left_duty,
    placementAfterCwStrafeLeftMs:p.post_clockwise_strafe_left_duration_ms,
    placementCwDelay:p.post_clockwise_delay_ms,
    placementExitDuty:p.exit_forward_duty,
    placementExitMs:p.exit_forward_duration_ms,
    placementExitDelay:p.post_forward_delay_ms,
    placementStrafeDuty:p.strafe_right_duty,
    placementStrafeTimeout:p.strafe_right_timeout_ms
  };
  Object.entries(values).forEach(([id,value]) =>
    setLfValue(id, value || ''));
  placementLoaded = true;
}
function placementParams(){
  return new URLSearchParams({
    'reverse-line-duty':qs('placementRearDuty').value,
    'lss1-timeout-ms':qs('placementLssTimeout').value,
    'post-lss1-delay-ms':qs('placementLssDelay').value,
    'initial-heading-timeout-ms':qs('placementInitialHeadingTimeout').value,
    'pre-ccw-strafe-right-duty':qs('placementPreCcwStrafeDuty').value,
    'pre-ccw-strafe-right-ms':qs('placementPreCcwStrafeMs').value,
    'ccw-angle-deg':qs('placementCcwAngle').value,
    'ccw-timeout-ms':qs('placementCcwTimeout').value,
    'forward-to-slide-duty':qs('placementSlideDuty').value,
    'forward-to-slide-ms':qs('placementSlideMs').value,
    'stepper-down-speed':qs('placementStepperSpeed').value,
    'stepper-down-timeout-ms':qs('placementStepperTimeout').value,
    'pusher-open-settle-ms':qs('placementPusherSettle').value,
    'push-forward-duty':qs('placementPushDuty').value,
    'push-forward-ms':qs('placementPushMs').value,
    'reverse-retreat-duty':qs('placementRetreatDuty').value,
    'reverse-retreat-ms':qs('placementRetreatMs').value,
    'cw-angle-deg':qs('placementCwAngle').value,
    'cw-timeout-ms':qs('placementCwTimeout').value,
    'post-cw-reverse-duty':qs('placementAfterCwReverseDuty').value,
    'post-cw-reverse-ms':qs('placementAfterCwReverseMs').value,
    'post-cw-strafe-left-duty':qs('placementAfterCwStrafeLeftDuty').value,
    'post-cw-strafe-left-ms':qs('placementAfterCwStrafeLeftMs').value,
    'post-cw-delay-ms':qs('placementCwDelay').value,
    'exit-forward-duty':qs('placementExitDuty').value,
    'exit-forward-ms':qs('placementExitMs').value,
    'post-forward-delay-ms':qs('placementExitDelay').value,
    'strafe-right-duty':qs('placementStrafeDuty').value,
    'strafe-right-timeout-ms':qs('placementStrafeTimeout').value
  });
}
function placementShowResult(result){
  const message = result.json.message || result.json.error ||
    `request ${result.status}`;
  qs('placementResult').textContent = message;
  qs('placementResult').className = result.ok ? 'mono good' : 'mono bad';
  return result;
}
function placementApply(){
  return api(`/api/autonomous/habitat-placement/config?${placementParams().toString()}`)
    .then(placementShowResult);
}
async function placementStart(){
  const applied = await placementApply();
  if (!applied.ok) return applied;
  return api('/api/autonomous/habitat-placement/start')
    .then(placementShowResult);
}
async function placementSave(){
  const applied = await placementApply();
  if (!applied.ok) return applied;
  return api('/api/config/save').then(placementShowResult);
}
function placementStop(){
  return api('/api/autonomous/habitat-placement/stop')
    .then(placementShowResult);
}
function loadLfControls(j){
  if (lfLoaded || !j.pid) return;
  setLfValue('lfKp', j.pid.kp);
  setLfValue('lfKi', j.pid.ki);
  setLfValue('lfKd', j.pid.kd);
  setLfValue('lfBase', j.pid.baseDuty);
  setLfValue('lfMaxDuty', j.pid.maxDuty ?? j.pid.maximumDuty);
  setLfValue('lfMaxCorrection', j.pid.maxCorrection ?? j.pid.maximumCorrection);
  setLfValue('lfIntegralLimit', j.pid.integralLimit);
  setLfValue('lfDerivativeLimit', j.pid.derivativeLimit);
  setLfValue('lfDerivativeAlpha', j.pid.derivativeFilterAlpha);
  setLfValue('lfPolarity', j.pid.steeringPolarity);
  setLfValue('lfTelemetry', j.pid.telemetryEnabled ? 'on' : 'off');
  lfLoaded = true;
}
function loadRlfControls(j){
  if (rlfLoaded || !j.rear_pid) return;
  setLfValue('rlfKp', j.rear_pid.kp);
  setLfValue('rlfKi', j.rear_pid.ki);
  setLfValue('rlfKd', j.rear_pid.kd);
  setLfValue('rlfBase', j.rear_pid.baseDuty);
  setLfValue('rlfMaxDuty', j.rear_pid.maxDuty ?? j.rear_pid.maximumDuty);
  setLfValue('rlfMaxCorrection', j.rear_pid.maxCorrection ?? j.rear_pid.maximumCorrection);
  setLfValue('rlfIntegralLimit', j.rear_pid.integralLimit);
  setLfValue('rlfDerivativeLimit', j.rear_pid.derivativeLimit);
  setLfValue('rlfDerivativeAlpha', j.rear_pid.derivativeFilterAlpha);
  setLfValue('rlfPolarity', j.rear_pid.steeringPolarity);
  setLfValue('rlfTelemetry', j.rear_pid.telemetryEnabled ? 'on' : 'off');
  rlfLoaded = true;
}
function lfParams(){
  const p = new URLSearchParams();
  function add(name, id){ const v = qs(id).value; if (v !== '') p.set(name, v); }
  add('kp', 'lfKp');
  add('ki', 'lfKi');
  add('kd', 'lfKd');
  add('base', 'lfBase');
  add('max-duty', 'lfMaxDuty');
  add('max-correction', 'lfMaxCorrection');
  add('integral-limit', 'lfIntegralLimit');
  add('derivative-limit', 'lfDerivativeLimit');
  add('derivative-alpha', 'lfDerivativeAlpha');
  add('polarity', 'lfPolarity');
  add('telemetry', 'lfTelemetry');
  return p;
}
function lfApply(){ return api(`/api/line-follow/config?${lfParams().toString()}`); }
function lfStart(){ lfApply().then(() => api(`/api/line-follow/start?ms=${qs('lfDuration').value || 5000}`)); }
function lfStop(){ api('/api/line-follow/stop'); }
function lfSave(){ lfApply().then(() => api('/api/config/save')); }
function rlfParams(){
  const p = new URLSearchParams();
  function add(name, id){ const v = qs(id).value; if (v !== '') p.set(name, v); }
  add('kp', 'rlfKp');
  add('ki', 'rlfKi');
  add('kd', 'rlfKd');
  add('base', 'rlfBase');
  add('max-duty', 'rlfMaxDuty');
  add('max-correction', 'rlfMaxCorrection');
  add('integral-limit', 'rlfIntegralLimit');
  add('derivative-limit', 'rlfDerivativeLimit');
  add('derivative-alpha', 'rlfDerivativeAlpha');
  add('polarity', 'rlfPolarity');
  add('telemetry', 'rlfTelemetry');
  return p;
}
function rlfApply(){ return api(`/api/rear-line-follow/config?${rlfParams().toString()}`); }
function rlfStart(){ rlfApply().then(() => api(`/api/rear-line-follow/start?ms=${qs('rlfDuration').value || 5000}`)); }
function rlfStop(){ api('/api/rear-line-follow/stop'); }
function rlfSave(){ rlfApply().then(() => api('/api/config/save')); }
function lineSensorMode(){ api('/api/mode?mode=line-sensor'); }
function rearLineSensorMode(){ api('/api/mode?mode=rear-line-sensor'); }
function autoSolarStart(){ api('/api/autonomous/solar/start'); }
function loadTowerControls(j){
  if (towerLoaded || !j.tower_pieces) return;
  const line = j.tower_line_control || {};
  setLfValue('towerDuty', j.tower_pieces.reverse_line_duty);
  setLfValue('towerTimeoutMs', j.tower_pieces.side_line_timeout_ms);
  setLfValue('towerSideCooldownMs', line.side_line_cooldown_ms);
  setLfValue('towerSideRearmMs', line.side_line_rearm_ms);
  setLfValue('towerPostLineDelayMs', j.tower_pieces.post_line_delay_ms);
  setLfValue('towerStrafeDurationMs', j.tower_pieces.strafe_right_duration_ms);
  setLfValue('towerStrafeDuty', line.initial_strafe_duty);
  setLfValue('towerPostStrafePauseMs', j.tower_pieces.post_strafe_pause_ms);
  setLfValue('towerRotationAngleDeg', j.tower_pieces.clockwise_rotation_angle_deg);
  setLfValue('towerPostRotationPauseMs', j.tower_pieces.post_rotation_pause_ms);
  setLfValue('towerReverseDuty', j.tower_pieces.reverse_duty);
  setLfValue('towerReverseDurationMs', j.tower_pieces.reverse_duration_ms);
  setLfValue('towerShimmyRightMs', j.tower_pieces.shimmy_right_duration_ms);
  setLfValue('towerShimmyLeftMs', j.tower_pieces.shimmy_left_duration_ms);
  setLfValue('towerShimmyTimeoutMs', j.tower_pieces.shimmy_timeout_ms);
  setLfValue('towerShimmyDuty', line.shimmy_duty);
  setLfValue('towerFinalReverseDuty', j.tower_pieces.final_reverse_duty);
  setLfValue('towerFinalReverseDurationMs', j.tower_pieces.final_reverse_duration_ms);
  setLfValue('towerPostFinalReverseDelayMs', j.tower_pieces.post_final_reverse_delay_ms);
  setLfValue('towerPostWinchOpenDelayMs', j.tower_pieces.post_winch_open_delay_ms);
  setLfValue('towerPostClawsOpenDelayMs', j.tower_pieces.post_claws_open_delay_ms);
  setLfValue('towerPreStepperBottomDelayMs', line.pre_stepper_bottom_delay_ms);
  setLfValue('towerStepperDownSpeed', j.tower_pieces.stepper_down_speed_steps_per_second);
  setLfValue('towerPostStepperBottomDelayMs', j.tower_pieces.post_stepper_bottom_delay_ms);
  setLfValue('towerPostClawsClosedDelayMs', j.tower_pieces.post_claws_closed_delay_ms);
  setLfValue('towerStepperUpSpeed', j.tower_pieces.stepper_up_speed_steps_per_second);
  towerLoaded = true;
}
function towerParams(){
  const p = new URLSearchParams();
  p.set('duty', qs('towerDuty').value);
  p.set('timeout-ms', qs('towerTimeoutMs').value);
  p.set('side-line-cooldown-ms', qs('towerSideCooldownMs').value);
  p.set('side-line-rearm-ms', qs('towerSideRearmMs').value);
  p.set('post-line-delay-ms', qs('towerPostLineDelayMs').value);
  p.set('strafe-duration-ms', qs('towerStrafeDurationMs').value);
  p.set('strafe-duty', qs('towerStrafeDuty').value);
  p.set('post-strafe-pause-ms', qs('towerPostStrafePauseMs').value);
  p.set('rotation-angle-deg', qs('towerRotationAngleDeg').value);
  p.set('post-rotation-pause-ms', qs('towerPostRotationPauseMs').value);
  p.set('reverse-duty', qs('towerReverseDuty').value);
  p.set('reverse-duration-ms', qs('towerReverseDurationMs').value);
  p.set('shimmy-right-ms', qs('towerShimmyRightMs').value);
  p.set('shimmy-left-ms', qs('towerShimmyLeftMs').value);
  p.set('shimmy-timeout-ms', qs('towerShimmyTimeoutMs').value);
  p.set('shimmy-duty', qs('towerShimmyDuty').value);
  p.set('final-reverse-duty', qs('towerFinalReverseDuty').value);
  p.set('final-reverse-duration-ms', qs('towerFinalReverseDurationMs').value);
  p.set('post-final-reverse-delay-ms', qs('towerPostFinalReverseDelayMs').value);
  p.set('post-winch-open-delay-ms', qs('towerPostWinchOpenDelayMs').value);
  p.set('post-claws-open-delay-ms', qs('towerPostClawsOpenDelayMs').value);
  p.set('pre-stepper-bottom-delay-ms', qs('towerPreStepperBottomDelayMs').value);
  p.set('stepper-down-speed-steps-per-second', qs('towerStepperDownSpeed').value);
  p.set('post-stepper-bottom-delay-ms', qs('towerPostStepperBottomDelayMs').value);
  p.set('post-claws-closed-delay-ms', qs('towerPostClawsClosedDelayMs').value);
  p.set('stepper-up-speed-steps-per-second', qs('towerStepperUpSpeed').value);
  return p;
}
function towerResult(r){
  qs('towerCommand').textContent = r.ok ? (r.json.message || 'ok') : (r.json.error || `HTTP ${r.status}`);
  qs('towerCommand').className = r.ok ? 'mono good' : 'mono bad';
  return r;
}
function towerApply(){ return api(`/api/autonomous/tower-pieces/config?${towerParams().toString()}`).then(towerResult); }
function towerStart(){ towerApply().then(r => { if (r.ok) api('/api/autonomous/tower-pieces/start').then(towerResult); }); }
function towerSave(){ towerApply().then(r => { if (r.ok) api('/api/config/save').then(towerResult); }); }
function loadPegFinderControls(j){
  if (pegFinderLoaded || !j.peg_finder) return;
  const p = j.peg_finder;
  setLfValue('pegFinderClockwiseAngleDeg', p.clockwise_angle_deg);
  setLfValue('pegFinderPostRotationPauseMs', p.post_rotation_pause_ms);
  setLfValue('pegFinderReverseDuty', p.reverse_duty);
  setLfValue('pegFinderReverseDurationMs', p.reverse_duration_ms);
  setLfValue('pegFinderPostReversePauseMs', p.post_reverse_pause_ms);
  setLfValue('pegFinderForwardDuty', p.forward_duty);
  setLfValue('pegFinderForwardDurationMs', p.forward_duration_ms);
  setLfValue('pegFinderFunnelDuty', p.funnel_forward_duty);
  setLfValue('pegFinderFunnelTimeoutMs', p.funnel_forward_timeout_ms);
  setLfValue('pegFinderPostFunnelLimitDelayMs', p.post_funnel_limit_delay_ms);
  setLfValue('pegFinderClawOpenIntervalMs', p.claw_open_interval_ms);
  const clawOrder = p.claw_open_order || [1, 2, 3];
  setLfValue('pegFinderClawOrder1', clawOrder[0]);
  setLfValue('pegFinderClawOrder2', clawOrder[1]);
  setLfValue('pegFinderClawOrder3', clawOrder[2]);
  setLfValue('pegFinderPostClawsOpenDelayMs', p.post_claws_open_delay_ms);
  setLfValue('pegFinderFunnelReverseDuty', p.funnel_reverse_duty);
  setLfValue('pegFinderFunnelReverseDurationMs', p.funnel_reverse_duration_ms);
  pegFinderLoaded = true;
}
function pegFinderParams(){
  const p = new URLSearchParams();
  p.set('clockwise-angle-deg', qs('pegFinderClockwiseAngleDeg').value);
  p.set('post-rotation-pause-ms', qs('pegFinderPostRotationPauseMs').value);
  p.set('reverse-duty', qs('pegFinderReverseDuty').value);
  p.set('reverse-duration-ms', qs('pegFinderReverseDurationMs').value);
  p.set('post-reverse-pause-ms', qs('pegFinderPostReversePauseMs').value);
  p.set('forward-duty', qs('pegFinderForwardDuty').value);
  p.set('forward-duration-ms', qs('pegFinderForwardDurationMs').value);
  p.set('funnel-duty', qs('pegFinderFunnelDuty').value);
  p.set('funnel-timeout-ms', qs('pegFinderFunnelTimeoutMs').value);
  p.set('post-funnel-limit-delay-ms', qs('pegFinderPostFunnelLimitDelayMs').value);
  p.set('claw-open-interval-ms', qs('pegFinderClawOpenIntervalMs').value);
  p.set('claw-order-1', qs('pegFinderClawOrder1').value);
  p.set('claw-order-2', qs('pegFinderClawOrder2').value);
  p.set('claw-order-3', qs('pegFinderClawOrder3').value);
  p.set('post-claws-open-delay-ms', qs('pegFinderPostClawsOpenDelayMs').value);
  p.set('funnel-reverse-duty', qs('pegFinderFunnelReverseDuty').value);
  p.set('funnel-reverse-duration-ms', qs('pegFinderFunnelReverseDurationMs').value);
  return p;
}
function pegFinderResult(r){
  qs('pegFinderCommand').textContent = r.ok ? (r.json.message || 'ok') : (r.json.error || `HTTP ${r.status}`);
  qs('pegFinderCommand').className = r.ok ? 'mono good' : 'mono bad';
  return r;
}
function pegFinderApply(){ return api(`/api/autonomous/peg-finder/config?${pegFinderParams().toString()}`).then(pegFinderResult); }
function pegFinderStart(){ pegFinderApply().then(r => { if (r.ok) api('/api/autonomous/peg-finder/start').then(pegFinderResult); }); }
function pegFinderSave(){ pegFinderApply().then(r => { if (r.ok) api('/api/config/save').then(pegFinderResult); }); }
function loadTimeTrialControls(j){
  if (timeTrialLoaded || !j.time_trial) return;
  const t = j.time_trial;
  setLfValue('timeTrialPostSolarDelayMs', t.post_solar_delay_ms);
  setLfValue('timeTrialStrafeDurationMs', t.strafe_right_duration_ms);
  setLfValue('timeTrialPostTowerDelayMs', t.post_tower_delay_ms);
  timeTrialLoaded = true;
}
function timeTrialParams(){
  const p = new URLSearchParams();
  p.set('post-solar-delay-ms', qs('timeTrialPostSolarDelayMs').value);
  p.set('strafe-right-duration-ms', qs('timeTrialStrafeDurationMs').value);
  p.set('post-tower-delay-ms', qs('timeTrialPostTowerDelayMs').value);
  return p;
}
function timeTrialResult(r){
  qs('timeTrialCommand').textContent = r.ok ? (r.json.message || 'ok') : (r.json.error || `HTTP ${r.status}`);
  qs('timeTrialCommand').className = r.ok ? 'mono good' : 'mono bad';
  return r;
}
async function timeTrialApply(){
  const includedApplies = [autoSolarApply, towerApply, pegFinderApply, clawsApply];
  for (const apply of includedApplies) {
    const result = await apply();
    if (!result.ok) return timeTrialResult(result);
  }
  return api(`/api/autonomous/time-trial/config?${timeTrialParams().toString()}`).then(timeTrialResult);
}
async function timeTrialStart(){
  const applied = await timeTrialApply();
  if (applied.ok) {
    return api('/api/autonomous/time-trial/start').then(timeTrialResult);
  }
  return applied;
}
async function timeTrialSave(){
  const applied = await timeTrialApply();
  if (applied.ok) {
    return api('/api/config/save').then(timeTrialResult);
  }
  return applied;
}
function loadImuTurnControls(j){
  if (imuTurnLoaded || !j.imu_turn) return;
  const t = j.imu_turn;
  setLfValue('imuTurnMaxDuty', t.maximum_rotation_duty);
  setLfValue('imuTurnKp', t.kp);
  setLfValue('imuTurnKd', t.kd);
  setLfValue('imuTurnTolerance', t.angle_tolerance_deg);
  setLfValue('imuTurnFinishRate', t.maximum_finishing_yaw_rate_dps);
  setLfValue('imuTurnSettleMs', t.settling_time_ms);
  setLfValue('imuTurnTimeoutMs', t.timeout_ms);
  setLfValue('imuTurnPolarity', t.yaw_command_polarity);
  imuTurnLoaded = true;
}
function imuTurnParams(){
  const p = new URLSearchParams();
  p.set('max-duty', qs('imuTurnMaxDuty').value);
  p.set('kp', qs('imuTurnKp').value);
  p.set('kd', qs('imuTurnKd').value);
  p.set('tolerance-deg', qs('imuTurnTolerance').value);
  p.set('finish-rate-dps', qs('imuTurnFinishRate').value);
  p.set('settle-ms', qs('imuTurnSettleMs').value);
  p.set('timeout-ms', qs('imuTurnTimeoutMs').value);
  p.set('polarity', qs('imuTurnPolarity').value);
  return p;
}
function imuTurnShowResult(result){
  qs('imuTurnResult').textContent =
    result.ok ? (result.json.message || 'ok')
              : (result.json.error || `HTTP ${result.status}`);
  qs('imuTurnResult').className =
    result.ok ? 'mono good' : 'mono bad';
  return result;
}
function imuTurnApply(){
  return api(`/api/imu-turn/config?${imuTurnParams().toString()}`)
    .then(imuTurnShowResult);
}
async function imuTurnStart(degrees){
  const applied = await imuTurnApply();
  if (!applied.ok) return applied;
  return api(`/api/imu-turn/start?degrees=${degrees}`)
    .then(imuTurnShowResult);
}
async function imuTurnSave(){
  const applied = await imuTurnApply();
  if (!applied.ok) return applied;
  return api('/api/imu-turn/save').then(imuTurnShowResult);
}
function imuTurnStop(){
  return api('/api/imu-turn/stop').then(imuTurnShowResult);
}
function imuAngleReset(){
  return api('/api/imu-turn/reset-angle').then(imuTurnShowResult);
}
function imuSoakRefresh(){
  qs('imuSoakResult').textContent = 'refresh requested';
  qs('imuSoakResult').className = 'mono muted';
  update();
}
function imuSoakResetCounters(){
  qs('imuSoakResult').textContent = 'reset request pending';
  qs('imuSoakResult').className = 'mono muted';
  return api('/api/imu/soak/reset-counters').then(result => {
    qs('imuSoakResult').textContent =
      result.ok ? (result.json.message || 'reset queued')
                : (result.json.error || `HTTP ${result.status}`);
    qs('imuSoakResult').className =
      result.ok ? 'mono good' : 'mono bad';
    if (result.ok) setTimeout(update, 30);
    return result;
  });
}
function loadImuStrafeControls(j){
  if (imuStrafeLoaded || !j.imu_heading_hold) return;
  const h = j.imu_heading_hold;
  setLfValue('imuStrafeMaxDuty', h.maximum_strafe_duty);
  setLfValue('imuStrafeKp', h.kp);
  setLfValue('imuStrafeKd', h.kd);
  setLfValue('imuStrafeMaxCorrection', h.maximum_yaw_correction_duty);
  setLfValue('imuStrafePolarity', h.yaw_command_polarity);
  imuStrafeLoaded = true;
}
function imuStrafeParams(){
  const p = new URLSearchParams();
  p.set('strafe-duty', qs('imuStrafeMaxDuty').value);
  p.set('kp', qs('imuStrafeKp').value);
  p.set('kd', qs('imuStrafeKd').value);
  p.set('max-correction-duty', qs('imuStrafeMaxCorrection').value);
  p.set('polarity', qs('imuStrafePolarity').value);
  return p;
}
function imuStrafeShowResult(result){
  qs('imuStrafeResult').textContent =
    result.ok ? (result.json.message || 'ok')
              : (result.json.error || `HTTP ${result.status}`);
  qs('imuStrafeResult').className =
    result.ok ? 'mono good' : 'mono bad';
  return result;
}
function imuStrafeRequest(path){
  const request = imuStrafeRequestChain
    .catch(() => {})
    .then(() => api(path));
  imuStrafeRequestChain = request.catch(() => {});
  return request;
}
function imuStrafeRequestError(){
  return {
    ok: false,
    status: 0,
    json: {error: 'IMU strafe request failed'}
  };
}
function imuStrafeClearHeartbeat(){
  if (imuStrafeHeartbeat) clearInterval(imuStrafeHeartbeat);
  imuStrafeHeartbeat = null;
}
function imuStrafeStartHeartbeat(generation){
  imuStrafeClearHeartbeat();
  imuStrafeHeartbeatPending = false;
  imuStrafeHeartbeat = setInterval(() => {
    if (!imuStrafePressed ||
        generation !== imuStrafeGeneration ||
        imuStrafeHeartbeatPending) {
      return;
    }
    imuStrafeHeartbeatPending = true;
    imuStrafeRequest('/api/imu-strafe/heartbeat')
      .then(heartbeat => {
        if (!imuStrafePressed ||
            generation !== imuStrafeGeneration) {
          return;
        }
        if (!heartbeat.ok) {
          imuStrafeShowResult(heartbeat);
          imuStrafeRelease();
        }
      })
      .catch(() => {
        if (!imuStrafePressed ||
            generation !== imuStrafeGeneration) {
          return;
        }
        imuStrafeShowResult(imuStrafeRequestError());
        imuStrafeRelease();
      })
      .finally(() => {
        if (generation === imuStrafeGeneration) {
          imuStrafeHeartbeatPending = false;
        }
      });
  }, 200);
}
function imuStrafeApply(){
  return imuStrafeRequest(
      `/api/imu-strafe/config?${imuStrafeParams().toString()}`)
    .then(imuStrafeShowResult);
}
async function imuStrafeSave(){
  const applied = await imuStrafeApply();
  if (!applied.ok) return applied;
  return imuStrafeRequest('/api/imu-strafe/save')
    .then(imuStrafeShowResult);
}
function imuStrafeHold(direction,event){
  if (event && event.currentTarget &&
      event.currentTarget.setPointerCapture) {
    try {
      event.currentTarget.setPointerCapture(event.pointerId);
    } catch (_) {}
  }
  const generation = ++imuStrafeGeneration;
  imuStrafePressed = true;
  imuStrafePointerId = event ? event.pointerId : null;
  imuStrafeClearHeartbeat();
  imuStrafeHeartbeatPending = false;
  imuStrafeRequest(`/api/imu-strafe/start?direction=${direction}`)
    .then(result => {
      if (generation !== imuStrafeGeneration) {
        return;
      }
      imuStrafeShowResult(result);
      if (!result.ok) {
        imuStrafePressed = false;
        imuStrafePointerId = null;
        return;
      }
      if (!imuStrafePressed) return;
      imuStrafeStartHeartbeat(generation);
    })
    .catch(() => {
      if (generation !== imuStrafeGeneration) {
        return;
      }
      imuStrafePressed = false;
      imuStrafePointerId = null;
      imuStrafeShowResult(imuStrafeRequestError());
    });
}
function imuStrafeRelease(event=null,sendStop=true){
  if (event && imuStrafePointerId !== null &&
      event.pointerId !== imuStrafePointerId) {
    return;
  }
  const wasActive = imuStrafePressed || imuStrafeHeartbeat !== null;
  if (!wasActive) return;
  ++imuStrafeGeneration;
  imuStrafePressed = false;
  imuStrafePointerId = null;
  imuStrafeClearHeartbeat();
  imuStrafeHeartbeatPending = false;
  if (sendStop && wasActive) {
    return imuStrafeRequest('/api/imu-strafe/stop')
      .then(imuStrafeShowResult)
      .catch(() => imuStrafeShowResult(imuStrafeRequestError()));
  }
}
function imuStrafeStop(){
  ++imuStrafeGeneration;
  imuStrafePressed = false;
  imuStrafePointerId = null;
  imuStrafeClearHeartbeat();
  imuStrafeHeartbeatPending = false;
  return imuStrafeRequest('/api/imu-strafe/stop')
    .then(imuStrafeShowResult)
    .catch(() => imuStrafeShowResult(imuStrafeRequestError()));
}
function loadSolarControls(j){
  if (solarLoaded || !j.autonomous) return;
  const a = j.autonomous;
  const speeds = j.solar_strafe_speeds || {};
  setLfValue('solarDetect1', a.ir_detection_threshold_1khz);
  setLfValue('solarRelease1', a.ir_release_threshold_1khz);
  setLfValue('solarDetect10', a.ir_detection_threshold_10khz);
  setLfValue('solarRelease10', a.ir_release_threshold_10khz);
  setLfValue('solarConfirmMs', a.confirmation_time_ms);
  setLfValue('solarFilterAlpha', a.filter_alpha);
  setLfValue('solarIgnoreMs', a.ignore_after_start_ms);
  setLfValue('solarTimeoutMs', a.search_timeout_ms);
  setLfValue('solarStartDuty', a.start_base_duty);
  setLfValue('solarSlowAfterMs', a.slow_after_ms);
  setLfValue('solarSlowDuty', a.slow_base_duty);
  setLfValue('solarContactTimeoutMs', a.contact_timeout_ms);
  setLfValue('solarInitialStrafeDuty', speeds.initial_right_duty);
  setLfValue('solarRetryLeftDuty', speeds.retry_left_duty);
  setLfValue('solarRetryRightDuty', speeds.retry_right_duty);
  setLfValue('solarStrafeDelayMs', a.strafe_start_delay_ms);
  setLfValue('solarStrafeDuty', a.retry_forward_duty);
  setLfValue('solarRetryLeftMs', a.retry_strafe_left_duration_ms);
  setLfValue('solarRetryForwardMs', a.retry_forward_duration_ms);
  setLfValue('solarRetryStrafeTimeoutMs', a.retry_strafe_timeout_ms);
  setLfValue('solarPostContactForwardMs', a.post_contact_forward_duration_ms);
  setLfValue('solarPostContactForwardDuty', a.post_contact_forward_duty);
  setLfValue('solarPostContactForwardDelayMs', a.post_contact_forward_start_delay_ms);
  setLfValue('solarPostForwardStrafeDelayMs', a.line_reacquire_strafe_start_delay_ms);
  setLfValue('solarLineReacquireDuty', speeds.rear_line_strafe_duty);
  setLfValue('solarRearLineFollowDurationMs', speeds.backward_pid_duration_ms);
  solarLoaded = true;
}
function solarParams(){
  const p = new URLSearchParams();
  function add(name, id){ const v = qs(id).value; if (v !== '') p.set(name, v); }
  add('detect-1khz', 'solarDetect1');
  add('release-1khz', 'solarRelease1');
  add('detect-10khz', 'solarDetect10');
  add('release-10khz', 'solarRelease10');
  add('confirm-ms', 'solarConfirmMs');
  add('filter-alpha', 'solarFilterAlpha');
  add('ignore-ms', 'solarIgnoreMs');
  add('timeout-ms', 'solarTimeoutMs');
  add('start-duty', 'solarStartDuty');
  add('slow-after-ms', 'solarSlowAfterMs');
  add('slow-duty', 'solarSlowDuty');
  add('contact-timeout-ms', 'solarContactTimeoutMs');
  add('initial-strafe-duty', 'solarInitialStrafeDuty');
  add('retry-left-strafe-duty', 'solarRetryLeftDuty');
  add('retry-right-strafe-duty', 'solarRetryRightDuty');
  add('strafe-delay-ms', 'solarStrafeDelayMs');
  add('retry-forward-duty', 'solarStrafeDuty');
  add('retry-left-ms', 'solarRetryLeftMs');
  add('retry-forward-ms', 'solarRetryForwardMs');
  add('retry-strafe-timeout-ms', 'solarRetryStrafeTimeoutMs');
  add('post-contact-forward-ms', 'solarPostContactForwardMs');
  add('post-contact-forward-duty', 'solarPostContactForwardDuty');
  add('post-contact-forward-delay-ms', 'solarPostContactForwardDelayMs');
  add('post-forward-strafe-delay-ms', 'solarPostForwardStrafeDelayMs');
  add('line-reacquire-strafe-duty', 'solarLineReacquireDuty');
  add('rear-line-follow-duration-ms', 'solarRearLineFollowDurationMs');
  return p;
}
function autoSolarApply(){ return api(`/api/autonomous/solar/config?${solarParams().toString()}`); }
function autoSolarSave(){ autoSolarApply().then(r => { if (r.ok) api('/api/config/save'); }); }
function setClawAngle(id, value){
  const el = qs(id);
  if (!el || document.activeElement === el) return;
  el.value = value >= 0 ? value : '';
}
function loadClawControls(j){
  if (clawsLoaded || !j.claws) return;
  const claws = [j.claws.claw_1, j.claws.claw_2, j.claws.claw_3];
  claws.forEach((claw, index) => {
    const n = index + 1;
    setClawAngle(`claw${n}Open`, claw.openAngleDeg);
    setClawAngle(`claw${n}Closed`, claw.closedAngleDeg);
  });
  const winchServo = j.claws.winch || {};
  const pusherServo = j.claws.habitat_pusher || {};
  setClawAngle('pusherOpen', pusherServo.openAngleDeg ?? -1);
  setClawAngle('pusherClosed', pusherServo.closedAngleDeg ?? -1);
  setClawAngle('winchOpen', winchServo.openAngleDeg ?? -1);
  setClawAngle('winchClosed', winchServo.closedAngleDeg ?? -1);
  clawsLoaded = true;
}
function clawParams(){
  const p = new URLSearchParams();
  for (let n = 1; n <= 3; n++) {
    const open = qs(`claw${n}Open`).value;
    const closed = qs(`claw${n}Closed`).value;
    if (open !== '') p.set(`claw${n}-open`, open);
    if (closed !== '') p.set(`claw${n}-closed`, closed);
  }
  const winchOpen = qs('winchOpen').value;
  const winchClosed = qs('winchClosed').value;
  const pusherOpen = qs('pusherOpen').value;
  const pusherClosed = qs('pusherClosed').value;
  if (pusherOpen !== '') p.set('pusher-open', pusherOpen);
  if (pusherClosed !== '') p.set('pusher-closed', pusherClosed);
  if (winchOpen !== '') p.set('winch-open', winchOpen);
  if (winchClosed !== '') p.set('winch-closed', winchClosed);
  return p;
}
function clawsApply(){ return api(`/api/claws/config?${clawParams().toString()}`); }
function clawsSave(){ clawsApply().then(r => { if (r.ok) api('/api/claws/save'); }); }
function claw(id,state){ clawsApply().then(r => { if (r.ok) api(`/api/claw?id=${id}&state=${state}`); }); }
function clawAll(state){ clawsApply().then(r => { if (r.ok) api(`/api/claws?state=${state}`); }); }
function winch(state){ clawsApply().then(r => { if (r.ok) api(`/api/winch?state=${state}`); }); }
function habitatPusher(state){ clawsApply().then(r => { if (r.ok) api(`/api/habitat-pusher?state=${state}`); }); }
function clawSummary(c){
  if (!c) return 'n/a';
  const open = c.openConfigured ? c.openAngleDeg : 'unset';
  const closed = c.closedConfigured ? c.closedAngleDeg : 'unset';
  const target = c.outputEnabled ? c.commandedAngleDeg : 'off';
  return `open=${open} closed=${closed} target=${target}`;
}
function servoHardwareSummary(c){
  if (!c) return 'n/a';
  const configured = yn(c.hardwareConfigured);
  if (c.pwmBackend === 'MCPWM') {
    const generator = c.mcpwmGenerator === 0 ? 'A' : 'B';
    return `${configured} · GPIO${c.gpio} · MCPWM ${c.mcpwmUnit}/${c.mcpwmTimer}/${generator}`;
  }
  if (c.pwmBackend === 'LEDC') {
    return `${configured} · GPIO${c.gpio} · LEDC ${c.ledcChannel}`;
  }
  return `${configured} · unconfigured`;
}
function loadSolarHookControls(j){
  if (solarHookLoaded || !j.solar_hook) return;
  setClawAngle('solarHookOpen', j.solar_hook.openAngleDeg ?? -1);
  setClawAngle('solarHookClosed', j.solar_hook.closedAngleDeg ?? -1);
  solarHookLoaded = true;
}
function solarHookParams(){
  const p = new URLSearchParams();
  const open = qs('solarHookOpen').value;
  const closed = qs('solarHookClosed').value;
  if (open !== '') p.set('open-angle', open);
  if (closed !== '') p.set('closed-angle', closed);
  return p;
}
function solarHookShowResult(result){
  const message = result.json.message || result.json.error ||
    `request ${result.status}`;
  qs('solarHookResult').textContent = message;
  qs('solarHookResult').className =
    result.ok ? 'mono good' : 'mono bad';
  return result;
}
function solarHookApply(){
  return api(`/api/solar-hook/config?${solarHookParams().toString()}`)
    .then(solarHookShowResult);
}
async function solarHookSave(){
  const applied = await solarHookApply();
  if (!applied.ok) return applied;
  return api('/api/solar-hook/save').then(solarHookShowResult);
}
async function solarHookCommand(state){
  if (state !== 'disable') {
    const applied = await solarHookApply();
    if (!applied.ok) return applied;
  }
  return api(`/api/solar-hook?state=${state}`)
    .then(solarHookShowResult);
}
function yn(v){ return v ? 'yes' : 'no'; }
function level(v){ return v ? 'HIGH' : 'LOW'; }
function update(){
  fetch('/api/telemetry').then(r => r.json()).then(j => {
    qs('mode').textContent = j.current_mode;
    qs('fault').textContent = j.fault_active ? `${j.fault_code}: ${j.fault_message}` : 'none';
    qs('fault').className = j.fault_active ? 'mono bad' : 'mono good';
    qs('link').textContent = `${yn(j.rear.esp1_link_healthy)} configured=${yn(j.rear.esp1_link_configured)}`;
    qs('uptime').textContent = `${j.uptime_ms} ms`;
    qs('ap').textContent = `${j.ip_address} clients=${j.wifi_clients}`;
    qs('deadman').textContent = `${j.last_command_age_ms} ms, ${j.deadman_remaining_ms} ms left`;
    const ultrasonic1 = j.ultrasonic_1 || {};
    const ultrasonic1Fresh = ultrasonic1.data_fresh === true;
    const ultrasonic1Valid = ultrasonic1Fresh &&
      ultrasonic1.configured === true && ultrasonic1.echo_valid === true;
    if (ultrasonic1Valid) {
      const distanceMm = ultrasonic1.distance_mm ?? 0;
      qs('ultrasonic1Distance').textContent =
        `${distanceMm} mm (${(distanceMm / 10).toFixed(1)} cm)`;
      qs('ultrasonic1Distance').className = 'mono good';
      qs('ultrasonic1Status').textContent = 'valid echo';
      qs('ultrasonic1Status').className = 'mono good';
    } else {
      qs('ultrasonic1Distance').textContent = '—';
      qs('ultrasonic1Distance').className = 'mono muted';
      qs('ultrasonic1Status').textContent =
        !ultrasonic1Fresh ? 'ESP1 data stale' :
        (ultrasonic1.configured !== true ? 'not configured' : 'no valid echo');
      qs('ultrasonic1Status').className =
        ultrasonic1Fresh ? 'mono muted' : 'mono bad';
    }
    qs('ultrasonic1Echo').textContent =
      ultrasonic1Fresh ? `${ultrasonic1.echo_duration_us ?? 0} µs` : '—';
    qs('ultrasonic1Age').textContent =
      ultrasonic1Fresh ? `${ultrasonic1.sample_age_ms ?? 0} ms` : '—';
    const laser = j.laser_distance || {};
    const laserUsable = laser.available === true &&
      laser.configured === true && laser.initialized === true &&
      laser.ranging === true && laser.data_fresh === true &&
      laser.data_valid === true && laser.sensor_range_status === 0;
    qs('laserDistance').textContent = laserUsable
      ? `${laser.distance_mm ?? 0} mm (${((laser.distance_mm ?? 0) / 10).toFixed(1)} cm)`
      : '—';
    qs('laserDistance').className =
      laserUsable ? 'mono good' : 'mono muted';
    qs('laserState').textContent =
      `available=${yn(laser.available)} configured=${yn(laser.configured)} initialized=${yn(laser.initialized)} ranging=${yn(laser.ranging)} fresh=${yn(laser.data_fresh)} valid=${yn(laser.data_valid)}`;
    qs('laserState').className =
      laserUsable ? 'mono good' : 'mono bad';
    qs('laserProfile').textContent = laser.profile || 'DEFAULT';
    qs('laserProfile').className =
      laser.profile === 'HIGH_ACCURACY' ? 'mono good' : 'mono muted';
    qs('laserStatus').textContent =
      `${laser.sensor_range_status ?? 255} / ${laser.driver_status ?? 0}`;
    qs('laserAge').textContent =
      `${laser.sample_age_ms ?? 0} / ${laser.snapshot_age_ms ?? 0} ms`;
    qs('laserSequence').textContent =
      `${laser.measurement_sequence ?? 0} / ${laser.packet_sequence ?? 0}`;
    qs('laserCounts').textContent =
      `${laser.successful_measurement_count ?? 0} / ${laser.failed_measurement_count ?? 0} / ${laser.consecutive_failed_measurements ?? 0}`;
    qs('laserTiming').textContent =
      `${laser.acquisition_duration_us ?? 0} / ${laser.maximum_acquisition_duration_us ?? 0} µs`;
    qs('laserBus').textContent =
      `${laser.sda_gpio ?? -1} / ${laser.scl_gpio ?? -1} / 0x${Number(laser.i2c_address ?? 0).toString(16).toUpperCase().padStart(2, '0')}`;
    const habitat = j.habitat_pieces || {};
    loadHabitatControls(j);
    qs('habitatConfig').textContent = habitat.configuration_valid
      ? 'valid' : 'incomplete / locked';
    qs('habitatConfig').className =
      habitat.configuration_valid ? 'mono good' : 'mono bad';
    qs('habitatReady').textContent = yn(habitat.start_ready);
    qs('habitatReady').className =
      habitat.start_ready ? 'mono good' : 'mono bad';
    qs('habitatState').textContent = habitat.state || 'WAIT_FOR_START';
    const habitatReason = habitat.stop_reason || 'CONFIGURATION_INCOMPLETE';
    qs('habitatGate').textContent =
      `${habitatReason} · armed=${yn(habitat.lss2_detection_armed)} · stop=${yn(habitat.should_stop)} · target=${yn(habitat.target_reached)}`;
    qs('habitatGate').className =
      (!habitat.timed_out &&
      (habitatReason === 'NONE' ||
       habitatReason === 'BOTH_SIDE_LINES_DETECTED' ||
       habitatReason === 'DISTANCE_ZONE_COUNT_REACHED' ||
       habitatReason === 'RETURN_LINE_DETECTED')
        ? 'mono good' : 'mono bad');
    const habitatLss2Level =
      habitat.lss2_configured && habitat.lss2_data_fresh
        ? (habitat.lss2_black ? 'BLACK' : 'WHITE') : 'UNKNOWN';
    qs('habitatLss2').textContent =
      `${habitatLss2Level} · configured=${yn(habitat.lss2_configured)} · fresh=${yn(habitat.lss2_data_fresh)}`;
    qs('habitatLss2').className =
      habitat.lss2_configured && habitat.lss2_data_fresh ? 'mono good' : 'mono bad';
    const habitatLss3Level =
      habitat.lss3_configured && habitat.lss3_data_fresh
        ? (habitat.lss3_black ? 'BLACK' : 'WHITE') : 'UNKNOWN';
    qs('habitatLss3').textContent =
      `${habitatLss3Level} · configured=${yn(habitat.lss3_configured)} · fresh=${yn(habitat.lss3_data_fresh)}`;
    qs('habitatLss3').className =
      habitat.lss3_configured && habitat.lss3_data_fresh ? 'mono good' : 'mono bad';
    qs('habitatAlign').textContent =
      `active=${yn(habitat.side_line_aligning)} · LSS2/left latched=${yn(habitat.lss2_latched)} drive=${yn(habitat.left_side_driving)} · LSS3/right latched=${yn(habitat.lss3_latched)} drive=${yn(habitat.right_side_driving)}`;
    qs('habitatAlign').className =
      habitat.side_line_aligning ? 'mono good' : 'mono muted';
    qs('habitatDelay').textContent =
      `${habitat.run_elapsed_ms ?? 0} / ${habitat.lss2_detection_delay_ms ?? 0} ms (${habitat.lss2_detection_remaining_ms ?? 0} ms remaining)`;
    qs('habitatTimeout').textContent =
      `${habitat.run_elapsed_ms ?? 0} / ${habitat.run_timeout_ms ?? 0} ms (${habitat.timeout_remaining_ms ?? 0} ms remaining)`;
    qs('habitatReverse').textContent =
      `${habitat.reverse_elapsed_ms ?? 0} / ${habitat.reverse_duration_ms ?? 0} ms at duty ${Number(habitat.reverse_duty ?? 0).toFixed(3)} · active=${yn(habitat.reversing)}`;
    qs('habitatDistance').textContent = habitat.distance_measurement_available
      ? `${habitat.distance_mm ?? 0} / ${habitat.distance_threshold_mm ?? 0} mm · zone=${habitat.distance_zone_active ? 'ABOVE' : 'AT/BELOW'} · new=${yn(habitat.distance_sample_new)}`
      : `NO VALID READING / ${habitat.distance_threshold_mm ?? 0} mm · zone latched=${yn(habitat.distance_zone_active)}`;
    qs('habitatDistance').className = habitat.distance_measurement_available
      ? 'mono good' : 'mono bad';
    qs('habitatDistanceCount').textContent =
      `${habitat.distance_zone_count ?? 0} / ${habitat.distance_zone_target_count ?? 0} · entered this update=${yn(habitat.distance_zone_entered)}`;
    qs('habitatDistanceStrafe').textContent =
      `${habitat.distance_strafe_direction || 'NONE'} at duty ${Number(habitat.distance_strafe_duty ?? 0).toFixed(3)} · ${habitat.distance_strafe_elapsed_ms ?? 0} / ${habitat.distance_strafe_timeout_ms ?? 0} ms (${habitat.distance_strafe_remaining_ms ?? 0} ms remaining) · active=${yn(habitat.distance_strafing)}`;
    qs('habitatDistanceStrafe').className =
      habitat.distance_strafing ? 'mono good' : 'mono muted';
    qs('habitatCompensation').textContent =
      `${habitat.return_strafe_direction || 'NONE'} at duty ${Number(habitat.compensation_strafe_duty ?? 0).toFixed(3)} · ${habitat.compensation_strafe_elapsed_ms ?? 0} / ${habitat.compensation_strafe_duration_ms ?? 0} ms (${habitat.compensation_strafe_remaining_ms ?? 0} ms remaining) · active=${yn(habitat.compensation_strafing)}`;
    qs('habitatCompensation').className =
      habitat.compensation_strafing ? 'mono good' : 'mono muted';
    qs('habitatSlideDown').textContent =
      `${habitat.slide_down_speed_steps_per_second ?? 0} microsteps/s · ${habitat.slide_down_elapsed_ms ?? 0} / ${habitat.slide_down_timeout_ms ?? 0} ms (${habitat.slide_down_remaining_ms ?? 0} ms remaining) · bottom ready=${yn(habitat.slide_bottom_ready)} · active=${yn(habitat.lowering_slide)}`;
    qs('habitatSlideDown').className =
      habitat.lowering_slide ? 'mono good' : 'mono muted';
    qs('habitatForwardPickup').textContent = habitat.distance_measurement_available
      ? `${habitat.distance_mm ?? 0} / ${habitat.forward_stop_distance_mm ?? 0} mm at duty ${Number(habitat.forward_to_distance_duty ?? 0).toFixed(3)} · ${habitat.forward_to_distance_elapsed_ms ?? 0} / ${habitat.forward_to_distance_timeout_ms ?? 0} ms (${habitat.forward_to_distance_remaining_ms ?? 0} ms remaining) · reached=${yn(habitat.forward_distance_reached)} · active=${yn(habitat.forward_to_distance)}`
      : `NO VALID READING / ${habitat.forward_stop_distance_mm ?? 0} mm at duty ${Number(habitat.forward_to_distance_duty ?? 0).toFixed(3)} · ${habitat.forward_to_distance_elapsed_ms ?? 0} / ${habitat.forward_to_distance_timeout_ms ?? 0} ms (${habitat.forward_to_distance_remaining_ms ?? 0} ms remaining) · active=${yn(habitat.forward_to_distance)}`;
    qs('habitatForwardPickup').className =
      habitat.forward_to_distance ? 'mono good' : 'mono muted';
    qs('habitatSlideLift').textContent =
      `${habitat.slide_lift_steps ?? 0} microsteps at ${habitat.slide_lift_speed_steps_per_second ?? 0} microsteps/s · ${habitat.slide_lift_elapsed_ms ?? 0} / ${habitat.slide_lift_timeout_ms ?? 0} ms (${habitat.slide_lift_remaining_ms ?? 0} ms remaining) · started=${yn(habitat.slide_lift_started)} complete=${yn(habitat.slide_lift_complete)} wait=${yn(habitat.waiting_for_slide_lift)}`;
    qs('habitatSlideLift').className = habitat.slide_lift_complete
      ? 'mono good' : (habitat.slide_lift_started ? 'mono' : 'mono muted');
    qs('habitatPostReverse').textContent =
      `duty ${Number(habitat.post_pickup_reverse_duty ?? 0).toFixed(3)} · ${habitat.post_pickup_reverse_elapsed_ms ?? 0} / ${habitat.post_pickup_reverse_duration_ms ?? 0} ms (${habitat.post_pickup_reverse_remaining_ms ?? 0} ms remaining) · active=${yn(habitat.post_pickup_reversing)}`;
    qs('habitatPostReverse').className =
      habitat.post_pickup_reversing ? 'mono good' : 'mono muted';
    const habitatRearLine = habitat.rear_line_configured && habitat.rear_line_data_fresh
      ? `left=${habitat.rear_left_black ? 'BLACK' : 'WHITE'} right=${habitat.rear_right_black ? 'BLACK' : 'WHITE'}`
      : 'rear line UNKNOWN';
    qs('habitatReturnStrafe').textContent =
      `${habitat.return_strafe_direction || 'NONE'} at duty ${Number(habitat.return_strafe_duty ?? 0).toFixed(3)} · ${habitat.return_strafe_elapsed_ms ?? 0} / ${habitat.return_line_timeout_ms ?? 0} ms (${habitat.return_strafe_remaining_ms ?? 0} ms remaining) · ${habitatRearLine} · detected=${yn(habitat.rear_line_detected)} active=${yn(habitat.return_line_strafing)}`;
    qs('habitatReturnStrafe').className =
      habitat.return_line_strafing ? 'mono good' : 'mono muted';
    qs('habitatFollowing').textContent = yn(habitat.line_following);
    qs('habitatFollowing').className =
      habitat.line_following ? 'mono good' : 'mono muted';
    const placement = j.habitat_placement || {};
    loadPlacementControls(j);
    qs('placementConfig').textContent = placement.configuration_valid
      ? 'valid' : 'incomplete / locked';
    qs('placementConfig').className =
      placement.configuration_valid ? 'mono good' : 'mono bad';
    qs('placementReady').textContent = yn(placement.start_ready);
    qs('placementReady').className =
      placement.start_ready ? 'mono good' : 'mono bad';
    qs('placementState').textContent = placement.state || 'WAIT_FOR_START';
    qs('placementFault').textContent = placement.fault_reason || 'NONE';
    qs('placementFault').className =
      placement.fault_reason === 'NONE' ? 'mono good' : 'mono bad';
    qs('placementTime').textContent =
      `${placement.time_in_state_ms ?? 0} ms`;
    qs('placementCcwSaved').textContent =
      placement.initial_heading_captured
        ? `${Number(placement.initial_heading_deg ?? 0).toFixed(2)}\u00b0 / ${Number(placement.counter_clockwise_target_heading_deg ?? 0).toFixed(2)}\u00b0`
        : 'not captured';
    const imu = j.imu || {};
    const hexByte = value =>
      `0x${Number(value ?? 0).toString(16).toUpperCase().padStart(2, '0')}`;
    qs('imuConfigured').textContent = yn(imu.configured);
    qs('imuInitialized').textContent = yn(imu.initialized);
    qs('imuCalibrated').textContent = yn(imu.calibrated);
    qs('imuAcquisitionRunning').textContent = yn(imu.acquisition_running);
    qs('imuAcquisitionRunning').className =
      imu.acquisition_running ? 'mono good' : 'mono bad';
    qs('imuHealth').textContent =
      `${imu.healthy ? 'healthy' : 'unhealthy'} / ${imu.data_fresh ? 'fresh' : 'stale'}`;
    qs('imuHealth').className =
      imu.healthy && imu.data_fresh ? 'mono good' : 'mono bad';
    const disconnectReason = imu.disconnect_reason || 'UNKNOWN';
    qs('imuDisconnectReason').textContent = disconnectReason;
    qs('imuDisconnectReason').className =
      disconnectReason === 'NONE' ? 'mono good' : 'mono bad';
    const lastDisconnectReason = imu.last_disconnect_reason || 'NONE';
    qs('imuLastDisconnect').textContent =
      `${lastDisconnectReason} · count ${imu.disconnect_count ?? 0} · at ${imu.last_disconnect_at_ms ?? 0} ms`;
    qs('imuLastDisconnect').className =
      lastDisconnectReason === 'NONE' ? 'mono muted' : 'mono bad';
    const lastReadFailure = imu.last_read_failure_reason || 'NONE';
    qs('imuLastReadFailure').textContent =
      `${lastReadFailure} · at ${imu.last_read_failure_us ?? 0} µs`;
    qs('imuLastReadFailure').className =
      lastReadFailure === 'NONE' ? 'mono good' : 'mono bad';
    qs('imuInitializationError').textContent =
      imu.initialization_error || 'UNKNOWN';
    qs('imuInitializationError').className =
      imu.initialization_error === 'NONE' ? 'mono good' : 'mono bad';
    qs('imuPins').textContent =
      `${imu.sda_gpio ?? -1} / ${imu.scl_gpio ?? -1}`;
    qs('imuDeviceAck').textContent = yn(imu.device_acknowledged);
    qs('imuDeviceAck').className =
      imu.device_acknowledged ? 'mono good' : 'mono bad';
    qs('imuRuntimeConfig').textContent =
      imu.runtime_configuration_valid ? 'verified' : 'not verified / lost';
    qs('imuRuntimeConfig').className =
      imu.runtime_configuration_valid ? 'mono good' : 'mono bad';
    qs('imuReadMode').textContent =
      imu.register_reads_use_repeated_start ? 'repeated start' : 'stop / start';
    const wireStatus = imu.last_wire_status ?? -1;
    const wireStatusNames = {
      '-1':'no transaction', 0:'success', 1:'buffer overflow',
      2:'address NACK', 3:'data NACK', 4:'other error', 5:'timeout'
    };
    qs('imuWireStatus').textContent =
      `${wireStatus} (${wireStatusNames[wireStatus] || 'unknown'})`;
    qs('imuAddress').textContent = hexByte(imu.i2c_address);
    qs('imuWhoAmI').textContent = hexByte(imu.who_am_i);
    qs('imuRawGyroZ').textContent = imu.raw_gyro_z ?? 0;
    qs('imuBias').textContent = `${Number(imu.gyro_z_bias_dps ?? 0).toFixed(4)} °/s`;
    qs('imuYawRate').textContent = `${Number(imu.yaw_rate_dps ?? 0).toFixed(3)} °/s`;
    qs('imuHeading').textContent = `${Number(imu.heading_deg ?? 0).toFixed(2)}°`;
    qs('imuSampleAge').textContent = `${imu.sample_age_ms ?? 0} ms`;
    qs('imuSnapshotAge').textContent = `${imu.snapshot_age_ms ?? 0} ms`;
    qs('imuAcquisitionDuration').textContent =
      `${imu.acquisition_duration_us ?? 0} µs`;
    qs('imuMaxAcquisitionDuration').textContent =
      `${imu.maximum_completed_acquisition_duration_us ?? 0} µs`;
    qs('imuTotalAttempts').textContent =
      imu.total_acquisition_attempts ?? 0;
    qs('imuLastSuccessfulReadUs').textContent =
      `${imu.last_successful_read_us ?? 0} µs`;
    qs('imuSampleInterval').textContent =
      `${imu.last_sample_interval_us ?? 0} µs`;
    qs('imuReadCounts').textContent =
      `${imu.successful_read_count ?? 0} / ${imu.failed_read_count ?? 0}`;
    qs('imuConsecutiveFailures').textContent =
      imu.consecutive_failed_reads ?? 0;
    const imuTiming = imu.acquisition_timing || {};
    qs('imuLoopTiming').textContent =
      `${imuTiming.loop_interval_us ?? 0} / ${imuTiming.maximum_loop_interval_us ?? 0} µs`;
    qs('imuSyncTiming').textContent =
      `${imuTiming.synchronization_duration_us ?? 0} / ${imuTiming.maximum_synchronization_duration_us ?? 0} µs`;
    qs('imuLockTiming').textContent =
      `${imuTiming.wire_lock_acquire_duration_us ?? 0} / ${imuTiming.maximum_wire_lock_acquire_duration_us ?? 0} µs`;
    qs('imuReadTiming').textContent =
      `${imuTiming.measurement_read_duration_us ?? 0} / ${imuTiming.maximum_measurement_read_duration_us ?? 0} µs`;
    qs('imuPublishTiming').textContent =
      `${imuTiming.successful_read_to_publication_us ?? 0} / ${imuTiming.maximum_successful_read_to_publication_us ?? 0} µs`;
    qs('imuPublishQueueTiming').textContent =
      `${imuTiming.publication_queue_duration_us ?? 0} / ${imuTiming.maximum_publication_queue_duration_us ?? 0} µs`;
    qs('imuPublicationGap').textContent =
      `${imuTiming.successful_sample_publication_gap_us ?? 0} / ${imuTiming.maximum_successful_sample_publication_gap_us ?? 0} µs`;
    qs('imuObservedPublicationGap').textContent =
      `${imuTiming.current_observed_publication_gap_us ?? 0} / ${imuTiming.maximum_observed_publication_gap_us ?? 0} µs`;
    qs('imuPublicationSequence').textContent =
      `${imuTiming.publication_sequence ?? 0} / ${imuTiming.successful_sample_sequence ?? 0}`;
    qs('imuDelayedIterations').textContent =
      imuTiming.delayed_iteration_count ?? 0;
    const imuRecovery = j.imu_recovery || {};
    const turnRecoveryPaused = imuRecovery.turn_paused === true;
    const strafeRecoveryPaused = imuRecovery.strafe_paused === true;
    qs('imuRecoveryState').textContent =
      turnRecoveryPaused ? 'TURN PAUSED — wheels disabled' :
      (strafeRecoveryPaused ? 'STRAFE PAUSED — wheels disabled' : 'ready');
    qs('imuRecoveryState').className =
      turnRecoveryPaused || strafeRecoveryPaused ? 'mono bad' : 'mono good';
    qs('imuRecoveryHeadings').textContent =
      `${Number(imuRecovery.turn_saved_heading_deg ?? 0).toFixed(2)}° / ${Number(imuRecovery.strafe_saved_heading_deg ?? 0).toFixed(2)}°`;
    qs('imuRecoveryConfirmations').textContent =
      `${imuRecovery.turn_consecutive_fresh_samples ?? 0} / ${imuRecovery.strafe_consecutive_fresh_samples ?? 0} of ${imuRecovery.consecutive_fresh_samples_required ?? 0}`;
    qs('imuRecoveryTimes').textContent =
      `${imuRecovery.turn_pause_elapsed_ms ?? 0} ms / ${imuRecovery.strafe_pause_elapsed_ms ?? 0} ms (limit ${imuRecovery.maximum_pause_ms ?? 0} ms)`;
    qs('imuRecoveryCounts').textContent =
      `${imuRecovery.turn_pause_count ?? 0} turn / ${imuRecovery.strafe_pause_count ?? 0} strafe; ${imuRecovery.total_paused_ms ?? 0} ms`;
    const imuTurn = j.imu_turn || {};
    loadImuTurnControls(j);
    qs('imuTurnConfigValid').textContent =
      imuTurn.configuration_valid ? 'valid' : 'incomplete / locked';
    qs('imuTurnConfigValid').className =
      imuTurn.configuration_valid ? 'mono good' : 'mono bad';
    qs('imuTurnState').textContent =
      `${imuTurn.state || 'IDLE'} active=${yn(imuTurn.active)}`;
    qs('imuTurnFault').textContent =
      imuTurn.fault_reason || 'NONE';
    qs('imuTurnFault').className =
      (imuTurn.fault_reason || 'NONE') === 'NONE' ? 'mono good' : 'mono bad';
    const turnAvailability = imuTurn.availability_fault || {};
    const availabilityCaptured = turnAvailability.capture_valid === true;
    const availabilityLatched = turnAvailability.latched === true;
    const imuAvailableNow = turnAvailability.imu_currently_available === true;
    qs('imuTurnFaultTiming').textContent =
      !availabilityCaptured ? 'no IMU availability fault captured' :
      `${availabilityLatched ? 'LATCHED' : 'historical'} at ${turnAvailability.evaluated_at_ms ?? 0} ms · IMU currently ${imuAvailableNow ? 'AVAILABLE' : 'UNAVAILABLE'} · ${turnAvailability.origin || 'UNKNOWN'} / ${turnAvailability.reason || 'UNKNOWN'}`;
    qs('imuTurnFaultTiming').className =
      availabilityLatched ? 'mono bad' : 'mono muted';
    qs('imuTurnAvailabilityGate').textContent =
      !availabilityCaptured ? '—' :
      `configured=${yn(turnAvailability.configured)} initialized=${yn(turnAvailability.initialized)} calibrated=${yn(turnAvailability.calibrated)} healthy=${yn(turnAvailability.healthy)} sample_valid=${yn(turnAvailability.sample_valid)} fresh=${yn(turnAvailability.data_fresh)} acquisition_running=${yn(turnAvailability.acquisition_running)} shared_snapshot=${yn(turnAvailability.shared_snapshot_available)}`;
    qs('imuTurnAvailabilityTimes').textContent =
      !availabilityCaptured ? '—' :
      `evaluated_us=${turnAvailability.evaluated_at_us ?? 0} published_us=${turnAvailability.published_at_us ?? 0} last_success_us=${turnAvailability.last_successful_read_us ?? 0} sample_age_us=${turnAvailability.sample_age_us ?? 0} snapshot_age_us=${turnAvailability.snapshot_age_us ?? 0} timeout_us=${turnAvailability.freshness_timeout_us ?? 0}`;
    qs('imuTurnPublicationGap').textContent =
      !availabilityCaptured ? '—' :
      `completed=${turnAvailability.successful_sample_publication_gap_us ?? 0} max_completed=${turnAvailability.maximum_successful_sample_publication_gap_us ?? 0} open_at_fault=${turnAvailability.current_observed_publication_gap_us ?? 0} max_observed=${turnAvailability.maximum_observed_publication_gap_us ?? 0} µs`;
    qs('imuTurnSnapshotIdentity').textContent =
      !availabilityCaptured ? '—' :
      `cached_seq=${turnAvailability.cached_snapshot_sequence ?? 0}/${turnAvailability.cached_successful_sample_sequence ?? 0} newest_available=${yn(turnAvailability.newest_snapshot_available)} newest_seq=${turnAvailability.newest_snapshot_sequence ?? 0}/${turnAvailability.newest_successful_sample_sequence ?? 0} match=${yn(turnAvailability.cached_snapshot_matches_newest)} fetched_at_us=${turnAvailability.cached_snapshot_fetched_at_us ?? 0} fetch_to_gate_us=${turnAvailability.cached_snapshot_fetch_to_gate_us ?? 0}`;
    qs('imuTurnAvailabilityLink').textContent =
      !availabilityCaptured ? '—' :
      `front_left=${yn(turnAvailability.front_left_configured)} front_right=${yn(turnAvailability.front_right_configured)} rear_link=${yn(turnAvailability.rear_link_configured)} rear_status_available=${yn(turnAvailability.rear_status_available)} rear_status_fresh=${yn(turnAvailability.rear_status_fresh)} rear_last_ms=${turnAvailability.rear_last_status_received_at_ms ?? 0} rear_age_ms=${turnAvailability.rear_status_age_ms ?? 0}`;
    qs('imuTurnCurrentAngle').textContent =
      `${Number(imuTurn.current_heading_deg ?? 0).toFixed(2)}°`;
    qs('imuTurnHeadings').textContent =
      `${Number(imuTurn.start_heading_deg ?? 0).toFixed(2)}° / ${Number(imuTurn.target_heading_deg ?? 0).toFixed(2)}°`;
    qs('imuTurnErrorRate').textContent =
      `${Number(imuTurn.angle_error_deg ?? 0).toFixed(2)}° / ${Number(imuTurn.yaw_rate_dps ?? 0).toFixed(2)} °/s`;
    qs('imuTurnTerms').textContent =
      `${Number(imuTurn.proportional_term ?? 0).toFixed(4)} / ${Number(imuTurn.damping_term ?? 0).toFixed(4)}`;
    qs('imuTurnOutput').textContent =
      Number(imuTurn.rotation_command ?? 0).toFixed(4);
    qs('imuTurnTime').textContent =
      `${imuTurn.elapsed_ms ?? 0} ms / ${imuTurn.settling_elapsed_ms ?? 0} ms`;
    const imuStrafe = j.imu_heading_hold || {};
    loadImuStrafeControls(j);
    qs('imuStrafeConfigValid').textContent =
      imuStrafe.configuration_valid ? 'valid' : 'incomplete / locked';
    qs('imuStrafeConfigValid').className =
      imuStrafe.configuration_valid ? 'mono good' : 'mono bad';
    qs('imuStrafeState').textContent =
      `${imuStrafe.state || 'IDLE'} active=${yn(imuStrafe.active)}`;
    qs('imuStrafeFault').textContent =
      imuStrafe.fault_reason || 'NONE';
    qs('imuStrafeFault').className =
      (imuStrafe.fault_reason || 'NONE') === 'NONE'
        ? 'mono good' : 'mono bad';
    const imuStrafeDirection = Number(imuStrafe.lateral_direction ?? 0);
    qs('imuStrafeDirection').textContent =
      imuStrafeDirection < 0 ? 'left' :
      (imuStrafeDirection > 0 ? 'right' : 'none');
    qs('imuStrafeHeadings').textContent =
      `${Number(imuStrafe.current_heading_deg ?? 0).toFixed(2)}° / ${Number(imuStrafe.target_heading_deg ?? 0).toFixed(2)}°`;
    qs('imuStrafeErrorRate').textContent =
      `${Number(imuStrafe.angle_error_deg ?? 0).toFixed(2)}° / ${Number(imuStrafe.yaw_rate_dps ?? 0).toFixed(2)} °/s`;
    qs('imuStrafeTerms').textContent =
      `${Number(imuStrafe.proportional_term ?? 0).toFixed(4)} / ${Number(imuStrafe.damping_term ?? 0).toFixed(4)}`;
    qs('imuStrafeOutput').textContent =
      Number(imuStrafe.yaw_correction_duty ?? 0).toFixed(4);
    qs('imuStrafeElapsed').textContent =
      `${imuStrafe.elapsed_ms ?? 0} ms`;
    const a = j.autonomous || {};
    qs('autoState').textContent = a.state || j.autonomous_state || 'WAIT_FOR_START';
    qs('autoFault').textContent = a.fault_reason || 'NONE';
    qs('autoTime').textContent = `${a.time_in_state_ms ?? 0} ms`;
    qs('autoIr').textContent = `${a.ir_raw_amplitude ?? 0} / ${(a.ir_filtered_amplitude ?? 0).toFixed ? (a.ir_filtered_amplitude ?? 0).toFixed(1) : 0}`;
    qs('autoThresholds').textContent = `${a.ir_detection_threshold ?? 0} / ${a.ir_release_threshold ?? 0}`;
    qs('autoConfirm').textContent = `${a.confirmation_progress_ms ?? 0} / ${a.confirmation_time_ms ?? 0} ms detected=${yn(a.beacon_detected)}`;
    qs('autoSlow').textContent = `${yn(a.slow_mode_active)} start=${a.start_base_duty ?? 0} after=${a.slow_after_ms ?? 0} ms slow=${a.slow_base_duty ?? 0}`;
    const limits = j.solarLimitSwitches || a.limit_switches || {};
    qs('solarLimitsConfigured').textContent = yn(limits.configured);
    qs('solarLimitsConfigured').className = limits.configured ? 'mono good' : 'mono bad';
    qs('solarLimitBackRight').textContent = `${level(limits.backRightHigh ?? limits.back_right_high)} hit=${yn(limits.backRightHit ?? limits.back_right_hit)}`;
    qs('solarLimitFrontRight').textContent = `${level(limits.frontRightHigh ?? limits.front_right_high)} hit=${yn(limits.frontRightHit ?? limits.front_right_hit)}`;
    qs('solarLimitsAll').textContent = yn(limits.allHit ?? limits.all_hit);
    qs('solarLimitsAll').className = (limits.allHit ?? limits.all_hit) ? 'mono good' : 'mono';
    qs('solarLimitTimeout').textContent = `${a.contact_timeout_ms ?? 0} ms`;
    qs('solarLimitDelay').textContent = `${a.strafe_start_delay_ms ?? 0} ms`;
    qs('solarLimitDuty').textContent = a.strafe_duty ?? 0;
    qs('solarRetryLeft').textContent = `${a.retry_strafe_left_duration_ms ?? 0} ms`;
    qs('solarRetryForward').textContent = `${a.retry_forward_duration_ms ?? 0} ms`;
    qs('solarRetryTimeout').textContent = `${a.retry_strafe_timeout_ms ?? 0} ms`;
    qs('solarPostContactForward').textContent = `${a.post_contact_forward_duration_ms ?? 0} ms`;
    qs('solarPostContactForwardDutyStatus').textContent = a.post_contact_forward_duty ?? 0;
    qs('solarLineReacquireDutyStatus').textContent = a.line_reacquire_strafe_duty ?? 0;
    qs('solarRearLineFollowDuration').textContent = `${a.rear_line_follow_duration_ms ?? 0} ms`;
    qs('solarPostContactForwardDelay').textContent = `${a.post_contact_forward_start_delay_ms ?? 0} ms`;
    qs('solarPostForwardStrafeDelay').textContent = `${a.line_reacquire_strafe_start_delay_ms ?? 0} ms`;
    const tower = j.tower_pieces || {};
    const towerLine = j.tower_line_control || {};
    qs('towerState').textContent = tower.state || 'WAIT_FOR_START';
    qs('towerFault').textContent = tower.fault_reason || 'NONE';
    qs('towerFault').className = tower.fault_reason && tower.fault_reason !== 'NONE' ? 'mono bad' : 'mono good';
    qs('towerTime').textContent = `${tower.time_in_state_ms ?? 0} ms`;
    qs('towerFollowing').textContent = yn(tower.line_following);
    qs('towerSideSensor').textContent = `${level(tower.side_line_sensor_high)} configured=${yn(tower.side_line_sensor_configured)}`;
    qs('towerSideSensor').className = tower.side_line_sensor_configured ? 'mono good' : 'mono bad';
    qs('towerSideCount').textContent = `${tower.side_line_count ?? 0} / ${tower.target_side_line_count ?? 2}`;
    qs('towerSideCount').className = (tower.side_line_count ?? 0) >= (tower.target_side_line_count ?? 2) ? 'mono good' : 'mono';
    qs('towerCrossingGate').textContent =
      `armed=${yn(towerLine.side_line_armed)} accepted=${yn(towerLine.detection_accepted)} rejected=${yn(towerLine.detection_rejected)} count=${towerLine.crossing_count ?? 0} rejectedTotal=${towerLine.rejected_detection_count ?? 0}`;
    qs('towerBackLineDetected').textContent = yn(tower.back_line_detected);
    qs('towerBackLineDetected').className = tower.back_line_detected ? 'mono good' : 'mono';
    qs('towerOutput').textContent = tower.final_reverse_active ? 'final backward' : (tower.stepper_moving_down ? 'stepper down' : (tower.stepper_moving_up ? 'stepper up' : (tower.line_following ? 'line following' : (tower.strafing_right ? 'strafe right' : (tower.rotating_clockwise ? 'clockwise' : (tower.driving_backward ? 'backward' : (tower.shimmying_left ? 'shimmy left' : (tower.shimmying_right ? 'shimmy right' : 'stopped'))))))));
    const pegFinder = j.peg_finder || {};
    qs('pegFinderState').textContent = pegFinder.state || 'WAIT_FOR_START';
    qs('pegFinderFault').textContent = pegFinder.fault_reason || 'NONE';
    qs('pegFinderFault').className = pegFinder.fault_reason && pegFinder.fault_reason !== 'NONE' ? 'mono bad' : 'mono good';
    qs('pegFinderTime').textContent = `${pegFinder.time_in_state_ms ?? 0} ms`;
    qs('pegFinderFunnelLimit').textContent = `${level(pegFinder.funnel_limit_high)} ${pegFinder.funnel_limit_high ? 'PRESSED' : 'released'} configured=${yn(pegFinder.funnel_limit_configured)}`;
    qs('pegFinderFunnelLimit').className = pegFinder.funnel_limit_configured ? (pegFinder.funnel_limit_high ? 'mono good' : 'mono') : 'mono bad';
    qs('pegFinderOutput').textContent = pegFinder.rotating_clockwise ? 'clockwise' : (pegFinder.driving_backward ? 'backward' : (pegFinder.driving_forward ? 'forward' : (pegFinder.funnel_forward ? 'funnel forward' : (pegFinder.funnel_reverse ? 'funnel reverse' : (pegFinder.opening_claw_1 ? 'opening claw 1' : (pegFinder.opening_claw_2 ? 'opening claw 2' : (pegFinder.opening_claw_3 ? 'opening claw 3' : 'stopped')))))));
    const timeTrial = j.time_trial || {};
    qs('timeTrialState').textContent = timeTrial.state || 'WAIT_FOR_START';
    qs('timeTrialTime').textContent = `${timeTrial.time_in_state_ms ?? 0} ms`;
    qs('timeTrialOutput').textContent = timeTrial.strafing_right ? 'right strafe' : 'included mode / stopped';
    loadTowerControls(j);
    loadPegFinderControls(j);
    loadTimeTrialControls(j);
    loadSolarControls(j);
    loadLfControls(j);
    loadRlfControls(j);
    loadClawControls(j);
    loadSolarHookControls(j);
    qs('lfEnabled').textContent = yn(j.line.line_follower_enabled);
    qs('lsfl').textContent = `${j.line.lsfl_level} black=${yn(j.line.lsfl_black)}`;
    qs('lsfr').textContent = `${j.line.lsfr_level} black=${yn(j.line.lsfr_black)}`;
    qs('lss').textContent = `${j.line.lss_level} black=${yn(j.line.lss_black)} configured=${yn(j.line.lss_configured)}`;
    qs('lss2').textContent = `${j.line.lss2_level} black=${yn(j.line.lss2_black)} configured=${yn(j.line.lss2_configured)}`;
    qs('lss3').textContent = `${j.line.lss3_level} black=${yn(j.line.lss3_black)} configured=${yn(j.line.lss3_configured)}`;
    qs('lfError').textContent = `${j.line.line_error}, side=${j.line.last_known_line_side}, visible=${yn(j.line.line_visible)}, hist=${yn(j.line.has_history)}`;
    const rearLine = j.rear_line || {};
    qs('rlfEnabled').textContent = yn(rearLine.line_follower_enabled);
    qs('lsbl').textContent = `${rearLine.lsbl_level || 'UNKNOWN'} black=${yn(rearLine.lsbl_black)}`;
    qs('lsbr').textContent = `${rearLine.lsbr_level || 'UNKNOWN'} black=${yn(rearLine.lsbr_black)}`;
    qs('rlfMapping').textContent = `${rearLine.logical_left_source || 'LSBR'}=${yn(rearLine.logical_left_black)} / ${rearLine.logical_right_source || 'LSBL'}=${yn(rearLine.logical_right_black)}`;
    qs('rearLineStream').textContent = `configured=${yn(rearLine.configured)} fresh=${yn(rearLine.data_fresh)} seq=${rearLine.sequence ?? 0} age=${rearLine.sample_age_ms ?? 0} ms`;
    qs('rlfError').textContent = `${rearLine.line_error ?? 0}, side=${rearLine.last_known_line_side ?? 0}, visible=${yn(rearLine.line_visible)}, hist=${yn(rearLine.has_history)}`;
    qs('lfPid').textContent = `P=${j.pid.p_term.toFixed(3)} I=${j.pid.i_term.toFixed(3)} D=${j.pid.d_term.toFixed(3)} C=${j.pid.correction.toFixed(3)}`;
    const rearPid = j.rear_pid || {};
    qs('rlfPid').textContent = `P=${(rearPid.p_term ?? 0).toFixed(3)} I=${(rearPid.i_term ?? 0).toFixed(3)} D=${(rearPid.d_term ?? 0).toFixed(3)} C=${(rearPid.correction ?? 0).toFixed(3)}`;
    qs('rlfEffectiveBase').textContent = `${rearPid.effectiveBaseDuty ?? 0} (reverse)`;
    qs('irSelectedHz').textContent = j.selectedBeaconFrequencyHz ?? 0;
    qs('irAmp').textContent = j.ir_amplitude_pp ?? 0;
    qs('ir1k').textContent = j.ir_1khz_goertzel_amplitude ?? 0;
	    qs('ir10k').textContent = j.ir_10khz_goertzel_amplitude ?? 0;
	    qs('irSelectedAmp').textContent = j.ir_selected_frequency_amplitude ?? 0;
	    qs('motors').textContent = JSON.stringify({motors:j.motors, rear:j.rear}, null, 2);
	    const funnel = (j.motors && j.motors.funnel) || {};
	    qs('funnelConfigured').textContent = yn(funnel.configured);
	    qs('funnelConfigured').className = funnel.configured ? 'mono good' : 'mono bad';
	    qs('funnelDesired').textContent = funnel.desired_command_milli ?? 0;
	    qs('funnelApplied').textContent = funnel.applied_command_milli ?? 0;
	    qs('funnel').textContent = JSON.stringify({funnel:funnel, esp1:j.esp1}, null, 2);
	    const claws = j.claws || {};
    const clawList = [claws.claw_1, claws.claw_2, claws.claw_3];
    qs('clawHardware').textContent = clawList.map(c => yn(c && c.hardwareConfigured)).join(' / ');
    qs('clawCommanded').textContent = clawList.map(clawSummary).join(' | ');
    qs('winchHardware').textContent = servoHardwareSummary(claws.winch);
    qs('winchHardware').className = claws.winch && claws.winch.hardwareConfigured ? 'mono good' : 'mono bad';
    qs('winchCommanded').textContent = clawSummary(claws.winch);
    qs('pusherHardware').textContent = servoHardwareSummary(claws.habitat_pusher);
    qs('pusherHardware').className = claws.habitat_pusher && claws.habitat_pusher.hardwareConfigured ? 'mono good' : 'mono bad';
    qs('pusherCommanded').textContent = clawSummary(claws.habitat_pusher);
    qs('claws').textContent = JSON.stringify(claws, null, 2);
    const solarHook = j.solar_hook || {};
    qs('solarHookHardware').textContent =
      yn(solarHook.hardwareConfigured);
    qs('solarHookHardware').className =
      solarHook.hardwareConfigured ? 'mono good' : 'mono bad';
    qs('solarHookOutput').textContent =
      solarHook.outputEnabled
        ? `enabled at ${solarHook.commandedAngleDeg ?? 'unknown'}°`
        : 'disabled';
    qs('solarHookOutput').className =
      solarHook.outputEnabled ? 'mono good' : 'mono muted';
    qs('solarHookCommandStatus').textContent =
      `open=${solarHook.openConfigured ? solarHook.openAngleDeg : 'unset'}° closed=${solarHook.closedConfigured ? solarHook.closedAngleDeg : 'unset'}° last=${solarHook.commandedOpen ? 'open' : 'closed/off'}`;
  }).catch(() => { qs('fault').textContent = 'telemetry disconnected'; qs('fault').className = 'mono bad'; });
  fetch('/api/events').then(r => r.json()).then(j => { qs('events').textContent = JSON.stringify(j.events, null, 2); });
}
setInterval(update, 300); setInterval(updateStepper, 300); update(); updateStepper();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  g_server.sendHeader("Cache-Control", "no-store, max-age=0");
  g_server.send_P(200, "text/html", kDashboardHtml);
}

void handleStatus() {
  g_server.sendHeader("Cache-Control", "no-store, max-age=0");
  const robot::TelemetrySnapshot snapshot = currentSnapshot();
  if (!robot::writeTelemetryJson(snapshot, g_json_buffer,
                                 sizeof(g_json_buffer), true)) {
    sendErrorJson(500, "telemetry json overflow");
    return;
  }
  g_server.send(200, "application/json", g_json_buffer);
}

void handleTelemetry() {
  g_server.sendHeader("Cache-Control", "no-store, max-age=0");
  const robot::TelemetrySnapshot snapshot = currentSnapshot();
  if (!robot::writeTelemetryJson(snapshot, g_json_buffer,
                                 sizeof(g_json_buffer), false)) {
    sendErrorJson(500, "telemetry json overflow");
    return;
  }
  g_server.send(200, "application/json", g_json_buffer);
}

void handleImuSoakCountersReset() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  if (!g_runtime.imu_acquisition->requestSoakCountersReset()) {
    sendErrorJson(503, "IMU acquisition command queue unavailable");
    return;
  }
  sendOkJson("IMU soak counter reset queued");
}

void handleDiagnostics() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  if (!context.motion_diagnostics.frozen()) {
    const robot::Milliseconds now_ms =
        static_cast<robot::Milliseconds>(millis());
    recordMotionDiagnostic(
        context, *g_runtime.front_left, *g_runtime.front_right,
        *g_runtime.rear_link, &context.latest_imu_snapshot,
        robot::MotionDiagnosticEvent::Periodic, now_ms);
    context.motion_diagnostics.freeze(
        robot::MotionDiagnosticTrigger::ManualFreeze, now_ms);
  }
  if (!robot::writeMotionDiagnosticsJson(
          context.motion_diagnostics, g_json_buffer,
          sizeof(g_json_buffer))) {
    sendErrorJson(500, "diagnostic json overflow");
    return;
  }
  g_server.sendHeader("Cache-Control", "no-store, max-age=0");
  g_server.send(200, "application/json", g_json_buffer);
}

void handleDiagnosticsReset() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  resetMotionDiagnosticCapture(context, now_ms);
  recordMotionDiagnostic(
      context, *g_runtime.front_left, *g_runtime.front_right,
      *g_runtime.rear_link, &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::Periodic, now_ms);
  sendOkJson("motion diagnostics reset");
}

void handleDiagnosticsFreeze() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  recordMotionDiagnostic(
      context, *g_runtime.front_left, *g_runtime.front_right,
      *g_runtime.rear_link, &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::Periodic, now_ms);
  context.motion_diagnostics.freeze(
      robot::MotionDiagnosticTrigger::ManualFreeze, now_ms);
  sendOkJson("motion diagnostics frozen");
}

void handleStop() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  RuntimeContext& context = *g_runtime.context;
  context.motion_diagnostics.noteWebStop(now_ms);
  recordMotionDiagnostic(
      context, *g_runtime.front_left, *g_runtime.front_right,
      *g_runtime.rear_link, &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::WebStop, now_ms);
  context.diagnostic_web_stop_seen = true;
  emergencyStop(*g_runtime.context, *g_runtime.front_left,
                *g_runtime.front_right, *g_runtime.rear_link, now_ms,
                robot::EventSource::Web);
  recordMotionDiagnostic(
      context, *g_runtime.front_left, *g_runtime.front_right,
      *g_runtime.rear_link, &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::OutputsDisabled, now_ms);
  sendOkJson("stopped");
}

void handleMode() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  if (!g_server.hasArg("mode")) {
    sendErrorJson(400, "missing mode");
    return;
  }
  robot::RobotTestMode mode{};
  const String mode_arg = g_server.arg("mode");
  if (!robot::parseRobotTestMode(mode_arg.c_str(), mode)) {
    sendErrorJson(400, "invalid mode");
    return;
  }

  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  disableActuators(*g_runtime.context, *g_runtime.front_left,
                   *g_runtime.front_right, *g_runtime.rear_link, now_ms);
  g_runtime.context->modes.setMode(mode, now_ms);
  resetSolarPanelAutonomy(*g_runtime.context, now_ms);
  resetHabitatPieces(*g_runtime.context, now_ms);
  resetHabitatPlacement(*g_runtime.context, now_ms);
  resetTowerPieces(*g_runtime.context, now_ms);
  resetPegFinder(*g_runtime.context, now_ms);
  resetTimeTrial(*g_runtime.context, now_ms);
  resetImuTurn(*g_runtime.context);
  resetImuHeadingHold(*g_runtime.context);
  clearFault(*g_runtime.context);
  logEvent(*g_runtime.context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "mode changed");
  sendOkJson("mode changed");
}

void handleDrive() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  float vx = 0.0F;
  float vy = 0.0F;
  float wz = 0.0F;
  float duty = 0.0F;
  int hold_start_value = 0;
  if (!argFloat("vx", vx, 0.0F, true) ||
      !argFloat("vy", vy, 0.0F, true) ||
      !argFloat("wz", wz, 0.0F, true) ||
      !argFloat("duty", duty, 0.0F, true) ||
      !argSigned("hold-start", hold_start_value, 0, false) ||
      (hold_start_value != 0 && hold_start_value != 1)) {
    sendErrorJson(400, "malformed drive argument");
    return;
  }

  if (context.modes.currentMode() !=
      robot::RobotTestMode::DistributedDriveTest) {
    resetTowerPieces(context, now_ms);
    resetPegFinder(context, now_ms);
    resetTimeTrial(context, now_ms);
    disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                     *g_runtime.rear_link, now_ms);
    context.modes.setMode(robot::RobotTestMode::DistributedDriveTest, now_ms);
  }

  const robot::CommandValidationResult validation =
      robot::validateDriveCommand(context.modes.currentMode(), vx, vy, wz,
                                  duty, validationLimits(context));
  if (!validation.accepted) {
    disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                     *g_runtime.rear_link,
                     now_ms);
    setFault(context, robot::FaultCode::InvalidCommand, validation.reason);
    logEvent(context, now_ms, robot::EventSeverity::Warn,
             robot::EventSource::Web, validation.reason);
    sendErrorJson(409, validation.reason);
    return;
  }
  if (!g_runtime.front_left->configured() ||
      !g_runtime.front_right->configured()) {
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "front motors are not configured");
    sendErrorJson(409, "front motors are not configured");
    return;
  }
  if (context.modes.currentMode() ==
          robot::RobotTestMode::DistributedDriveTest &&
      !g_runtime.rear_link->configured()) {
    setFault(context, robot::FaultCode::CommunicationStale,
             "rear UART is not configured");
    sendErrorJson(409, "rear UART is not configured");
    return;
  }

  const bool starts_new_hold = hold_start_value == 1;
  if (starts_new_hold) {
    resetMotionDiagnosticCapture(context, now_ms);
  }
  const bool arrived_after_stop =
      !starts_new_hold && context.diagnostic_web_stop_seen;
  context.motion_diagnostics.noteWebDrive(
      now_ms, starts_new_hold, arrived_after_stop);

  context.requested_command =
      makeManualDriveCommand(context, vx, vy, wz, duty, now_ms);
  context.last_command_ms = now_ms;
  context.command_deadman_armed = true;
  context.mode_expires_at_ms = now_ms + kCommandTimeoutMs;
  if (starts_new_hold || arrived_after_stop) {
    recordMotionDiagnostic(
        context, *g_runtime.front_left, *g_runtime.front_right,
        *g_runtime.rear_link, &context.latest_imu_snapshot,
        starts_new_hold
            ? robot::MotionDiagnosticEvent::WebDriveStart
            : robot::MotionDiagnosticEvent::WebDriveHeartbeatAfterStop,
        now_ms);
  }
  clearFault(context);
  sendOkJson("drive command accepted");
}

void handleMotor() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  if (!g_server.hasArg("id")) {
    sendErrorJson(400, "missing motor id");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  robot::WheelId wheel{};
  const String id_arg = g_server.arg("id");
  float speed = 0.0F;
  if (!robot::parseWheelId(id_arg.c_str(), wheel) ||
      !argFloat("speed", speed, 0.0F, true)) {
    sendErrorJson(400, "malformed motor command");
    return;
  }

  if (context.modes.currentMode() != robot::RobotTestMode::SingleMotorTest) {
    resetTowerPieces(context, now_ms);
    resetPegFinder(context, now_ms);
    resetTimeTrial(context, now_ms);
    disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                     *g_runtime.rear_link, now_ms);
    context.modes.setMode(robot::RobotTestMode::SingleMotorTest, now_ms);
  }

  const robot::CommandValidationResult validation =
      robot::validateSingleMotorCommand(context.modes.currentMode(), speed,
                                        kCommandTimeoutMs,
                                        validationLimits(context));
  if (!validation.accepted) {
    setFault(context, robot::FaultCode::InvalidCommand, validation.reason);
    logEvent(context, now_ms, robot::EventSeverity::Warn,
             robot::EventSource::Web, validation.reason);
    sendErrorJson(409, validation.reason);
    return;
  }

  const bool front_wheel =
      wheel == robot::WheelId::FrontLeft ||
      wheel == robot::WheelId::FrontRight;
  if (front_wheel &&
      ((wheel == robot::WheelId::FrontLeft &&
        !g_runtime.front_left->configured()) ||
       (wheel == robot::WheelId::FrontRight &&
        !g_runtime.front_right->configured()))) {
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "selected front motor is not configured on ESP2");
    sendErrorJson(409, "selected front motor is not configured on ESP2");
    return;
  }
  if (!front_wheel && !g_runtime.rear_link->configured()) {
    setFault(context, robot::FaultCode::CommunicationStale,
             "back motor UART to ESP1 is not configured");
    sendErrorJson(409, "back motor UART to ESP1 is not configured");
    return;
  }

  context.requested_command = robot::disabledFourWheelCommand();
  if (wheel == robot::WheelId::FrontLeft) {
    context.requested_command.front_left =
        makeTimedMotorCommand(speed, now_ms, kCommandTimeoutMs);
  } else if (wheel == robot::WheelId::FrontRight) {
    context.requested_command.front_right =
        makeTimedMotorCommand(speed, now_ms, kCommandTimeoutMs);
  } else if (wheel == robot::WheelId::BackLeft) {
    context.requested_command.back_left =
        makeTimedMotorCommand(speed, now_ms, kCommandTimeoutMs);
  } else {
    context.requested_command.back_right =
        makeTimedMotorCommand(speed, now_ms, kCommandTimeoutMs);
  }
  context.last_command_ms = now_ms;
  context.mode_expires_at_ms = now_ms + kCommandTimeoutMs;
  context.command_deadman_armed = std::fabs(speed) > 0.0001F;
  clearFault(context);
  if (context.command_deadman_armed) {
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::Web, "single motor hold command");
  }
  sendOkJson("motor command accepted");
}

void handleInvert() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  if (!g_server.hasArg("id")) {
    sendErrorJson(400, "missing motor id");
    return;
  }
  robot::WheelId wheel{};
  const String id_arg = g_server.arg("id");
  if (!robot::parseWheelId(id_arg.c_str(), wheel)) {
    sendErrorJson(400, "invalid motor id");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  if (!allWheelCommandsDisabled(context.last_commanded_wheels) ||
      context.command_deadman_armed) {
    sendErrorJson(409, "motor inversion is only allowed while stopped");
    return;
  }
  if (wheel == robot::WheelId::BackLeft || wheel == robot::WheelId::BackRight) {
    sendErrorJson(501, "back inversion must be changed on ESP1");
    return;
  }

  DualPwmMotorOutput& motor =
      wheel == robot::WheelId::FrontLeft ? *g_runtime.front_left
                                         : *g_runtime.front_right;
  motor.setRuntimeInverted(!motor.runtimeInverted());
  if (g_runtime.preferences != nullptr) {
    const char* key = wheel == robot::WheelId::FrontLeft ? "inv_fl" : "inv_fr";
    g_runtime.preferences->putBool(key, motor.runtimeInverted());
  }
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "front motor inversion toggled");
  sendOkJson("motor inversion toggled");
}

void handleSensors() {
  const robot::TelemetrySnapshot snapshot = currentSnapshot();
  std::snprintf(g_json_buffer, sizeof(g_json_buffer),
                "{\"lsfl_raw_level\":%d,\"lsfr_raw_level\":%d,"
                "\"lss_raw_level\":%d,\"lss2_raw_level\":%d,"
                "\"lss3_raw_level\":%d,"
                "\"lsfl_level\":\"%s\",\"lsfr_level\":\"%s\","
                "\"lss_level\":\"%s\",\"lss2_level\":\"%s\","
                "\"lss3_level\":\"%s\","
                "\"lsfl_black\":%s,\"lsfr_black\":%s,"
                "\"lss_black\":%s,\"lss_configured\":%s,"
                "\"lss2_black\":%s,\"lss2_configured\":%s,"
                "\"lss3_black\":%s,\"lss3_configured\":%s,"
                "\"limit_switch_stepper_bottom\":%s,"
                "\"limit_switch_stepper_middle\":%s,"
                "\"limit_switch_stepper_top\":%s,"
                "\"limit_switch_funnel_left\":%s,"
                "\"limit_switch_funnel_right\":%s,"
                "\"solar_limit_switches_configured\":%s,"
                "\"solar_limit_back_right_high\":%s,"
                "\"solar_limit_front_right_high\":%s,"
                "\"solar_limit_back_right_hit\":%s,"
                "\"solar_limit_front_right_hit\":%s,"
                "\"solar_limit_all_hit\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                snapshot.lss_raw_level, snapshot.lss2_raw_level,
                snapshot.lss3_raw_level,
                digitalLevelName(snapshot.lsfl_raw_level),
                digitalLevelName(snapshot.lsfr_raw_level),
                digitalLevelName(snapshot.lss_raw_level),
                digitalLevelName(snapshot.lss2_raw_level),
                digitalLevelName(snapshot.lss3_raw_level),
                snapshot.lsfl_black ? "true" : "false",
                snapshot.lsfr_black ? "true" : "false",
                snapshot.lss_black ? "true" : "false",
                snapshot.lss_configured ? "true" : "false",
                snapshot.lss2_black ? "true" : "false",
                snapshot.lss2_configured ? "true" : "false",
                snapshot.lss3_black ? "true" : "false",
                snapshot.lss3_configured ? "true" : "false",
                snapshot.limit_switch_stepper_bottom ? "true" : "false",
                snapshot.limit_switch_stepper_middle ? "true" : "false",
                snapshot.limit_switch_stepper_top ? "true" : "false",
                snapshot.limit_switch_funnel_left ? "true" : "false",
                snapshot.limit_switch_funnel_right ? "true" : "false",
                snapshot.solar_panel_limit_switches_configured ? "true"
                                                               : "false",
                snapshot.solar_limit_back_right_high ? "true" : "false",
                snapshot.solar_limit_front_right_high ? "true" : "false",
                snapshot.solar_limit_back_right_hit ? "true" : "false",
                snapshot.solar_limit_front_right_hit ? "true" : "false",
                snapshot.solar_limit_all_hit ? "true" : "false");
  g_server.send(200, "application/json", g_json_buffer);
}

void handleLine() {
  const robot::TelemetrySnapshot snapshot = currentSnapshot();
  std::snprintf(g_json_buffer, sizeof(g_json_buffer),
                "{\"LSFL\":%d,\"LSFR\":%d,\"LSS\":%d,\"LSS2\":%d,"
                "\"LSS3\":%d,"
                "\"LSFLLevel\":\"%s\",\"LSFRLevel\":\"%s\","
                "\"LSSLevel\":\"%s\",\"LSS2Level\":\"%s\","
                "\"LSS3Level\":\"%s\",\"leftBlack\":%s,"
                "\"rightBlack\":%s,\"sideBlack\":%s,"
                "\"sideConfigured\":%s,\"side2Black\":%s,"
                "\"side2Configured\":%s,\"side3Black\":%s,"
                "\"side3Configured\":%s,\"error\":%d,"
                "\"lastKnownSide\":%d,"
                "\"lineVisible\":%s,\"hasHistory\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                snapshot.lss_raw_level, snapshot.lss2_raw_level,
                snapshot.lss3_raw_level,
                digitalLevelName(snapshot.lsfl_raw_level),
                digitalLevelName(snapshot.lsfr_raw_level),
                digitalLevelName(snapshot.lss_raw_level),
                digitalLevelName(snapshot.lss2_raw_level),
                digitalLevelName(snapshot.lss3_raw_level),
                snapshot.lsfl_black ? "true" : "false",
                snapshot.lsfr_black ? "true" : "false",
                snapshot.lss_black ? "true" : "false",
                snapshot.lss_configured ? "true" : "false",
                snapshot.lss2_black ? "true" : "false",
                snapshot.lss2_configured ? "true" : "false",
                snapshot.lss3_black ? "true" : "false",
                snapshot.lss3_configured ? "true" : "false",
                static_cast<int>(snapshot.line_error),
                static_cast<int>(snapshot.last_known_line_side),
                snapshot.line_visible ? "true" : "false",
                snapshot.line_has_history ? "true" : "false");
  g_server.send(200, "application/json", g_json_buffer);
}

void handleRearLine() {
  const robot::TelemetrySnapshot snapshot = currentSnapshot();
  std::snprintf(
      g_json_buffer, sizeof(g_json_buffer),
      "{\"LSBL\":%d,\"LSBR\":%d,\"LSBLLevel\":\"%s\","
      "\"LSBRLevel\":\"%s\",\"leftBlack\":%s,\"rightBlack\":%s,"
      "\"configured\":%s,\"dataFresh\":%s,\"sequence\":%u,"
      "\"sampleAgeMs\":%u,\"capturedAtMs\":%u,\"error\":%d,"
      "\"lastKnownSide\":%d,\"lineVisible\":%s,\"hasHistory\":%s,"
      "\"travelDirection\":\"REVERSE\","
      "\"logicalLeftSource\":\"LSBR\","
      "\"logicalRightSource\":\"LSBL\","
      "\"logicalLeftBlack\":%s,\"logicalRightBlack\":%s}",
      snapshot.lsbl_raw_level, snapshot.lsbr_raw_level,
      digitalLevelName(snapshot.lsbl_raw_level),
      digitalLevelName(snapshot.lsbr_raw_level),
      snapshot.lsbl_black ? "true" : "false",
      snapshot.lsbr_black ? "true" : "false",
      snapshot.rear_line_configured ? "true" : "false",
      snapshot.rear_line_data_fresh ? "true" : "false",
      static_cast<unsigned>(snapshot.rear_line_sequence),
      static_cast<unsigned>(snapshot.rear_line_sample_age_ms),
      static_cast<unsigned>(snapshot.rear_line_captured_at_ms),
      static_cast<int>(snapshot.rear_line_error),
      static_cast<int>(snapshot.rear_last_known_line_side),
      snapshot.rear_line_visible ? "true" : "false",
      snapshot.rear_line_has_history ? "true" : "false",
      snapshot.rear_logical_left_black ? "true" : "false",
      snapshot.rear_logical_right_black ? "true" : "false");
  g_server.send(200, "application/json", g_json_buffer);
}

void handleHabitatPiecesStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  requestHabitatPiecesStart(*g_runtime.context, *g_runtime.front_left,
                            *g_runtime.front_right, *g_runtime.rear_link,
                            now_ms, robot::EventSource::Web);
  sendOkJson("Habitat Pieces start requested");
}

void handleHabitatPiecesStop() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  disableActuators(*g_runtime.context, *g_runtime.front_left,
                   *g_runtime.front_right, *g_runtime.rear_link, now_ms);
  resetHabitatPieces(*g_runtime.context, now_ms);
  resetHabitatPlacement(*g_runtime.context, now_ms);
  clearFault(*g_runtime.context);
  logEvent(*g_runtime.context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "Habitat Pieces stopped");
  sendOkJson("Habitat Pieces stopped");
}

void handleHabitatPiecesConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  if (context.habitat_pieces.state !=
          robot::HabitatPiecesState::WaitForStart &&
      context.habitat_pieces.state !=
          robot::HabitatPiecesState::Complete &&
      context.habitat_pieces.state != robot::HabitatPiecesState::Fault) {
    sendErrorJson(409, "stop Habitat Pieces before changing its config");
    return;
  }

  robot::HabitatPiecesConfig next = context.habitat_pieces_config;
  float duty = next.line_follow_duty;
  float reverse_duty = next.reverse_duty;
  float distance_strafe_duty = next.distance_strafe_duty;
  float compensation_strafe_duty = next.compensation_strafe_duty;
  float forward_to_distance_duty = next.forward_to_distance_duty;
  float post_pickup_reverse_duty = next.post_pickup_reverse_duty;
  float return_strafe_duty = next.return_strafe_duty;
  robot::Milliseconds value = 0U;
  if (g_server.hasArg("duty") &&
      !argFloat("duty", duty, next.line_follow_duty, true)) {
    sendErrorJson(400, "malformed duty");
    return;
  }
  next.line_follow_duty = duty;
  if (g_server.hasArg("lss2-detection-delay-ms")) {
    if (!argUnsigned("lss2-detection-delay-ms", value,
                     next.lss2_detection_delay_ms, true)) {
      sendErrorJson(400, "malformed lss2-detection-delay-ms");
      return;
    }
    next.lss2_detection_delay_ms = value;
  }
  if (g_server.hasArg("run-timeout-ms")) {
    if (!argUnsigned("run-timeout-ms", value, next.run_timeout_ms,
                     true)) {
      sendErrorJson(400, "malformed run-timeout-ms");
      return;
    }
    next.run_timeout_ms = value;
  }
  if (g_server.hasArg("reverse-duty") &&
      !argFloat("reverse-duty", reverse_duty, next.reverse_duty, true)) {
    sendErrorJson(400, "malformed reverse-duty");
    return;
  }
  next.reverse_duty = reverse_duty;
  if (g_server.hasArg("reverse-duration-ms")) {
    if (!argUnsigned("reverse-duration-ms", value,
                     next.reverse_duration_ms, true)) {
      sendErrorJson(400, "malformed reverse-duration-ms");
      return;
    }
    next.reverse_duration_ms = value;
  }
  if (g_server.hasArg("distance-strafe-direction")) {
    const String direction =
        g_server.arg("distance-strafe-direction");
    if (direction == "LEFT" || direction == "left" || direction == "L" ||
        direction == "l") {
      next.distance_strafe_direction =
          robot::HabitatPiecesStrafeDirection::Left;
    } else if (direction == "RIGHT" || direction == "right" ||
               direction == "R" || direction == "r") {
      next.distance_strafe_direction =
          robot::HabitatPiecesStrafeDirection::Right;
    } else {
      sendErrorJson(400, "distance-strafe-direction must be LEFT or RIGHT");
      return;
    }
  }
  if (g_server.hasArg("distance-threshold-mm")) {
    if (!argUnsigned("distance-threshold-mm", value,
                     next.distance_threshold_mm, true) ||
        value > UINT16_MAX) {
      sendErrorJson(400, "malformed distance-threshold-mm");
      return;
    }
    next.distance_threshold_mm = static_cast<std::uint16_t>(value);
  }
  if (g_server.hasArg("distance-zone-count")) {
    if (!argUnsigned("distance-zone-count", value,
                     next.distance_zone_target_count, true) ||
        value > UINT16_MAX) {
      sendErrorJson(400, "malformed distance-zone-count");
      return;
    }
    next.distance_zone_target_count =
        static_cast<std::uint16_t>(value);
  }
  if (g_server.hasArg("distance-strafe-duty") &&
      !argFloat("distance-strafe-duty", distance_strafe_duty,
                next.distance_strafe_duty, true)) {
    sendErrorJson(400, "malformed distance-strafe-duty");
    return;
  }
  next.distance_strafe_duty = distance_strafe_duty;
  if (g_server.hasArg("distance-strafe-timeout-ms")) {
    if (!argUnsigned("distance-strafe-timeout-ms", value,
                     next.distance_strafe_timeout_ms, true)) {
      sendErrorJson(400, "malformed distance-strafe-timeout-ms");
      return;
    }
    next.distance_strafe_timeout_ms = value;
  }
  if (g_server.hasArg("compensation-strafe-duty") &&
      !argFloat("compensation-strafe-duty", compensation_strafe_duty,
                next.compensation_strafe_duty, true)) {
    sendErrorJson(400, "malformed compensation-strafe-duty");
    return;
  }
  next.compensation_strafe_duty = compensation_strafe_duty;
  if (g_server.hasArg("compensation-strafe-duration-ms")) {
    if (!argUnsigned("compensation-strafe-duration-ms", value,
                     next.compensation_strafe_duration_ms, true)) {
      sendErrorJson(400, "malformed compensation-strafe-duration-ms");
      return;
    }
    next.compensation_strafe_duration_ms = value;
  }
  if (g_server.hasArg("slide-down-speed-steps-per-second")) {
    if (!argUnsigned("slide-down-speed-steps-per-second", value,
                     next.slide_down_speed_steps_per_second, true)) {
      sendErrorJson(400, "malformed slide-down-speed-steps-per-second");
      return;
    }
    next.slide_down_speed_steps_per_second = value;
  }
  if (g_server.hasArg("slide-down-timeout-ms")) {
    if (!argUnsigned("slide-down-timeout-ms", value,
                     next.slide_down_timeout_ms, true)) {
      sendErrorJson(400, "malformed slide-down-timeout-ms");
      return;
    }
    next.slide_down_timeout_ms = value;
  }
  if (g_server.hasArg("forward-to-distance-duty") &&
      !argFloat("forward-to-distance-duty", forward_to_distance_duty,
                next.forward_to_distance_duty, true)) {
    sendErrorJson(400, "malformed forward-to-distance-duty");
    return;
  }
  next.forward_to_distance_duty = forward_to_distance_duty;
  if (g_server.hasArg("forward-stop-distance-mm")) {
    if (!argUnsigned("forward-stop-distance-mm", value,
                     next.forward_stop_distance_mm, true) ||
        value > UINT16_MAX) {
      sendErrorJson(400, "malformed forward-stop-distance-mm");
      return;
    }
    next.forward_stop_distance_mm = static_cast<std::uint16_t>(value);
  }
  if (g_server.hasArg("forward-to-distance-timeout-ms")) {
    if (!argUnsigned("forward-to-distance-timeout-ms", value,
                     next.forward_to_distance_timeout_ms, true)) {
      sendErrorJson(400, "malformed forward-to-distance-timeout-ms");
      return;
    }
    next.forward_to_distance_timeout_ms = value;
  }
  if (g_server.hasArg("slide-lift-steps")) {
    if (!argUnsigned("slide-lift-steps", value,
                     next.slide_lift_steps, true)) {
      sendErrorJson(400, "malformed slide-lift-steps");
      return;
    }
    next.slide_lift_steps = value;
  }
  if (g_server.hasArg("slide-lift-speed-steps-per-second")) {
    if (!argUnsigned("slide-lift-speed-steps-per-second", value,
                     next.slide_lift_speed_steps_per_second, true)) {
      sendErrorJson(400, "malformed slide-lift-speed-steps-per-second");
      return;
    }
    next.slide_lift_speed_steps_per_second = value;
  }
  if (g_server.hasArg("slide-lift-timeout-ms")) {
    if (!argUnsigned("slide-lift-timeout-ms", value,
                     next.slide_lift_timeout_ms, true)) {
      sendErrorJson(400, "malformed slide-lift-timeout-ms");
      return;
    }
    next.slide_lift_timeout_ms = value;
  }
  if (g_server.hasArg("post-pickup-reverse-duty") &&
      !argFloat("post-pickup-reverse-duty", post_pickup_reverse_duty,
                next.post_pickup_reverse_duty, true)) {
    sendErrorJson(400, "malformed post-pickup-reverse-duty");
    return;
  }
  next.post_pickup_reverse_duty = post_pickup_reverse_duty;
  if (g_server.hasArg("post-pickup-reverse-duration-ms")) {
    if (!argUnsigned("post-pickup-reverse-duration-ms", value,
                     next.post_pickup_reverse_duration_ms, true)) {
      sendErrorJson(400, "malformed post-pickup-reverse-duration-ms");
      return;
    }
    next.post_pickup_reverse_duration_ms = value;
  }
  if (g_server.hasArg("return-strafe-duty") &&
      !argFloat("return-strafe-duty", return_strafe_duty,
                next.return_strafe_duty, true)) {
    sendErrorJson(400, "malformed return-strafe-duty");
    return;
  }
  next.return_strafe_duty = return_strafe_duty;
  if (g_server.hasArg("return-line-timeout-ms")) {
    if (!argUnsigned("return-line-timeout-ms", value,
                     next.return_line_timeout_ms, true)) {
      sendErrorJson(400, "malformed return-line-timeout-ms");
      return;
    }
    next.return_line_timeout_ms = value;
  }

  robot::LineFollowerConfig line_config = context.config;
  line_config.baseDuty = next.line_follow_duty;
  if (g_runtime.stepper == nullptr ||
      g_runtime.stepper->maximumPositionSteps() <= 0 ||
      g_runtime.stepper->maximumPositionSteps() >
          static_cast<std::int64_t>(UINT32_MAX) ||
      !robot::habitatPiecesConfigValid(
          next, activeMotionDutyCap(context), kMaxTimedTestDurationMs,
          g_runtime.stepper->maximumSpeedStepsPerSecond(),
          static_cast<std::uint32_t>(
              g_runtime.stepper->maximumPositionSteps())) ||
      !robot::validateLineFollowerConfig(line_config, hardwareDutyCap())
           .accepted) {
    sendErrorJson(
        409,
        "Habitat Pieces requires every duty, duration, timeout, distance, count, and slide setting; LEFT/RIGHT initial direction; duties within the active cap; time bounds no greater than 30000 ms; and slide values within hardware limits");
    return;
  }

  context.habitat_pieces_config = next;
  resetHabitatPieces(context,
                     static_cast<robot::Milliseconds>(millis()));
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "Habitat Pieces config updated");
  sendOkJson("Habitat Pieces config updated");
}

void handleHabitatPlacementStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  requestHabitatPlacementStart(
      *g_runtime.context, *g_runtime.front_left, *g_runtime.front_right,
      *g_runtime.rear_link, now_ms, robot::EventSource::Web);
  sendOkJson("Habitat Placement start requested");
}

void handleHabitatPlacementStop() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  disableActuators(*g_runtime.context, *g_runtime.front_left,
                   *g_runtime.front_right, *g_runtime.rear_link, now_ms);
  g_runtime.stepper->stop();
  resetHabitatPlacement(*g_runtime.context, now_ms);
  clearFault(*g_runtime.context);
  logEvent(*g_runtime.context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "Habitat Placement stopped");
  sendOkJson("Habitat Placement stopped");
}

void handleHabitatPlacementConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  const robot::HabitatPlacementState state =
      context.habitat_placement.state;
  if (state != robot::HabitatPlacementState::WaitForStart &&
      state != robot::HabitatPlacementState::Complete &&
      state != robot::HabitatPlacementState::Fault) {
    sendErrorJson(409,
                  "stop Habitat Placement before changing its config");
    return;
  }

  robot::HabitatPlacementConfig next =
      context.habitat_placement_config;
  const auto readFloat = [](const char* name, float& field) {
    if (!g_server.hasArg(name)) {
      return true;
    }
    float value = field;
    if (!argFloat(name, value, field, true)) {
      return false;
    }
    field = value;
    return true;
  };
  const auto readMilliseconds = [](const char* name,
                                   robot::Milliseconds& field) {
    if (!g_server.hasArg(name)) {
      return true;
    }
    robot::Milliseconds value = field;
    if (!argUnsigned(name, value, field, true)) {
      return false;
    }
    field = value;
    return true;
  };
  const auto readSpeed = [](const char* name, std::uint32_t& field) {
    if (!g_server.hasArg(name)) {
      return true;
    }
    robot::Milliseconds value = field;
    if (!argUnsigned(name, value, field, true)) {
      return false;
    }
    field = value;
    return true;
  };

  if (!readFloat("reverse-line-duty", next.reverse_line_follow_duty) ||
      !readMilliseconds("lss1-timeout-ms", next.lss1_timeout_ms) ||
      !readMilliseconds("post-lss1-delay-ms",
                        next.post_lss1_delay_ms) ||
      !readMilliseconds(
          "initial-heading-timeout-ms",
          next.initial_heading_turn_timeout_ms) ||
      !readFloat(
          "pre-ccw-strafe-right-duty",
          next.pre_counter_clockwise_strafe_right_duty) ||
      !readMilliseconds(
          "pre-ccw-strafe-right-ms",
          next.pre_counter_clockwise_strafe_right_duration_ms) ||
      !readFloat("ccw-angle-deg", next.counter_clockwise_angle_deg) ||
      !readMilliseconds("ccw-timeout-ms",
                        next.counter_clockwise_timeout_ms) ||
      !readFloat("forward-to-slide-duty",
                 next.forward_to_slide_duty) ||
      !readMilliseconds("forward-to-slide-ms",
                        next.forward_to_slide_duration_ms) ||
      !readSpeed("stepper-down-speed",
                 next.stepper_down_speed_steps_per_second) ||
      !readMilliseconds("stepper-down-timeout-ms",
                        next.stepper_down_timeout_ms) ||
      !readMilliseconds("pusher-open-settle-ms",
                        next.pusher_open_settle_ms) ||
      !readFloat("push-forward-duty", next.push_forward_duty) ||
      !readMilliseconds("push-forward-ms",
                        next.push_forward_duration_ms) ||
      !readFloat("reverse-retreat-duty",
                 next.reverse_retreat_duty) ||
      !readMilliseconds("reverse-retreat-ms",
                        next.reverse_retreat_duration_ms) ||
      !readFloat("cw-angle-deg", next.clockwise_angle_deg) ||
      !readMilliseconds("cw-timeout-ms", next.clockwise_timeout_ms) ||
      !readFloat("post-cw-reverse-duty",
                 next.post_clockwise_reverse_duty) ||
      !readMilliseconds("post-cw-reverse-ms",
                        next.post_clockwise_reverse_duration_ms) ||
      !readFloat("post-cw-strafe-left-duty",
                 next.post_clockwise_strafe_left_duty) ||
      !readMilliseconds(
          "post-cw-strafe-left-ms",
          next.post_clockwise_strafe_left_duration_ms) ||
      !readMilliseconds("post-cw-delay-ms",
                        next.post_clockwise_delay_ms) ||
      !readFloat("exit-forward-duty", next.exit_forward_duty) ||
      !readMilliseconds("exit-forward-ms",
                        next.exit_forward_duration_ms) ||
      !readMilliseconds("post-forward-delay-ms",
                        next.post_forward_delay_ms) ||
      !readFloat("strafe-right-duty", next.strafe_right_duty) ||
      !readMilliseconds("strafe-right-timeout-ms",
                        next.strafe_right_timeout_ms)) {
    sendErrorJson(400, "malformed Habitat Placement config value");
    return;
  }

  const bool timings_bounded =
      next.lss1_timeout_ms <= kMaxTimedTestDurationMs &&
      next.post_lss1_delay_ms <= kMaxTimedTestDurationMs &&
      next.initial_heading_turn_timeout_ms <=
          kMaxTimedTestDurationMs &&
      next.pre_counter_clockwise_strafe_right_duration_ms <=
          kMaxTimedTestDurationMs &&
      next.counter_clockwise_timeout_ms <= kMaxTimedTestDurationMs &&
      next.forward_to_slide_duration_ms <= kMaxTimedTestDurationMs &&
      next.stepper_down_timeout_ms <= kMaxTimedTestDurationMs &&
      next.pusher_open_settle_ms <= kMaxTimedTestDurationMs &&
      next.push_forward_duration_ms <= kMaxTimedTestDurationMs &&
      next.reverse_retreat_duration_ms <= kMaxTimedTestDurationMs &&
      next.clockwise_timeout_ms <= kMaxTimedTestDurationMs &&
      next.post_clockwise_reverse_duration_ms <=
          kMaxTimedTestDurationMs &&
      next.post_clockwise_strafe_left_duration_ms <=
          kMaxTimedTestDurationMs &&
      next.post_clockwise_delay_ms <= kMaxTimedTestDurationMs &&
      next.exit_forward_duration_ms <= kMaxTimedTestDurationMs &&
      next.post_forward_delay_ms <= kMaxTimedTestDurationMs &&
      next.strafe_right_timeout_ms <= kMaxTimedTestDurationMs;
  robot::LineFollowerConfig line_config = context.rear_config;
  line_config.baseDuty = next.reverse_line_follow_duty;
  line_config = robot::makeReverseTravelLineFollowerConfig(line_config);
  if (!timings_bounded ||
      !robot::habitatPlacementConfigValid(
          next, rearMotionDutyCap(context),
          g_runtime.stepper->maximumSpeedStepsPerSecond()) ||
      !robot::validateLineFollowerConfig(line_config, hardwareDutyCap())
           .accepted) {
    sendErrorJson(
        409,
        "Habitat Placement needs nonzero duties and durations (including pre-CCW right strafe and after-CW reverse/left strafe), search/turn timeouts, positive turn angles, a valid slide speed, and every timing at or below 30000 ms");
    return;
  }

  context.habitat_placement_config = next;
  resetHabitatPlacement(
      context, static_cast<robot::Milliseconds>(millis()));
  clearFault(context);
  sendOkJson("Habitat Placement config updated");
}

void handleAutonomousSolarStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  requestSolarPanelAutonomyStart(*g_runtime.context, *g_runtime.front_left,
                                 *g_runtime.front_right,
                                 *g_runtime.rear_link, now_ms,
                                 robot::EventSource::Web);
  sendOkJson("solar autonomy start requested");
}

void handleAutonomousSolarConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  robot::SolarPanelAutonomyConfig next_config = context.solar_config;
  SolarIrThresholds next_thresholds = context.solar_thresholds;
  SolarLineFollowSpeedConfig next_speed = context.solar_speed_config;
  robot::SolarPanelContactConfig next_contact =
      context.solar_contact_config;
  robot::Milliseconds milliseconds_value = 0U;
  float float_value = 0.0F;

  if (g_server.hasArg("detect-1khz")) {
    if (!argUnsigned("detect-1khz", milliseconds_value,
                     next_thresholds.detect_1khz, true) ||
        milliseconds_value > UINT16_MAX) {
      sendErrorJson(400, "malformed detect-1khz");
      return;
    }
    next_thresholds.detect_1khz =
        static_cast<std::uint16_t>(milliseconds_value);
  }
  if (g_server.hasArg("release-1khz")) {
    if (!argUnsigned("release-1khz", milliseconds_value,
                     next_thresholds.release_1khz, true) ||
        milliseconds_value > UINT16_MAX) {
      sendErrorJson(400, "malformed release-1khz");
      return;
    }
    next_thresholds.release_1khz =
        static_cast<std::uint16_t>(milliseconds_value);
  }
  if (g_server.hasArg("detect-10khz")) {
    if (!argUnsigned("detect-10khz", milliseconds_value,
                     next_thresholds.detect_10khz, true) ||
        milliseconds_value > UINT16_MAX) {
      sendErrorJson(400, "malformed detect-10khz");
      return;
    }
    next_thresholds.detect_10khz =
        static_cast<std::uint16_t>(milliseconds_value);
  }
  if (g_server.hasArg("release-10khz")) {
    if (!argUnsigned("release-10khz", milliseconds_value,
                     next_thresholds.release_10khz, true) ||
        milliseconds_value > UINT16_MAX) {
      sendErrorJson(400, "malformed release-10khz");
      return;
    }
    next_thresholds.release_10khz =
        static_cast<std::uint16_t>(milliseconds_value);
  }
  if (g_server.hasArg("confirm-ms")) {
    if (!argUnsigned("confirm-ms", milliseconds_value,
                     next_config.confirmation_time_ms, true)) {
      sendErrorJson(400, "malformed confirm-ms");
      return;
    }
    next_config.confirmation_time_ms = milliseconds_value;
  }
  if (g_server.hasArg("filter-alpha")) {
    if (!argFloat("filter-alpha", float_value, next_config.filter_alpha,
                  true)) {
      sendErrorJson(400, "malformed filter-alpha");
      return;
    }
    next_config.filter_alpha = float_value;
  }
  if (g_server.hasArg("ignore-ms")) {
    if (!argUnsigned("ignore-ms", milliseconds_value,
                     next_config.ignore_after_start_ms, true)) {
      sendErrorJson(400, "malformed ignore-ms");
      return;
    }
    next_config.ignore_after_start_ms = milliseconds_value;
  }
  if (g_server.hasArg("timeout-ms")) {
    if (!argUnsigned("timeout-ms", milliseconds_value,
                     next_config.search_timeout_ms, true)) {
      sendErrorJson(400, "malformed timeout-ms");
      return;
    }
    next_config.search_timeout_ms = milliseconds_value;
  }
  if (g_server.hasArg("start-duty")) {
    if (!argFloat("start-duty", float_value, next_speed.start_base_duty,
                  true)) {
      sendErrorJson(400, "malformed start-duty");
      return;
    }
    next_speed.start_base_duty = float_value;
  }
  if (g_server.hasArg("slow-after-ms")) {
    if (!argUnsigned("slow-after-ms", milliseconds_value,
                     next_speed.slow_after_ms, true)) {
      sendErrorJson(400, "malformed slow-after-ms");
      return;
    }
    next_speed.slow_after_ms = milliseconds_value;
  }
  if (g_server.hasArg("slow-duty")) {
    if (!argFloat("slow-duty", float_value, next_speed.slow_base_duty,
                  true)) {
      sendErrorJson(400, "malformed slow-duty");
      return;
    }
    next_speed.slow_base_duty = float_value;
  }
  if (g_server.hasArg("contact-timeout-ms")) {
    if (!argUnsigned("contact-timeout-ms", milliseconds_value,
                     next_contact.timeout_ms, true)) {
      sendErrorJson(400, "malformed contact-timeout-ms");
      return;
    }
    next_contact.timeout_ms = milliseconds_value;
  }
  if (g_server.hasArg("initial-strafe-duty") &&
      !argFloat("initial-strafe-duty",
                next_contact.initial_strafe_right_duty,
                next_contact.initial_strafe_right_duty, true)) {
    sendErrorJson(400, "malformed initial-strafe-duty");
    return;
  }
  if (g_server.hasArg("retry-left-strafe-duty") &&
      !argFloat("retry-left-strafe-duty",
                next_contact.retry_strafe_left_duty,
                next_contact.retry_strafe_left_duty, true)) {
    sendErrorJson(400, "malformed retry-left-strafe-duty");
    return;
  }
  if (g_server.hasArg("retry-right-strafe-duty") &&
      !argFloat("retry-right-strafe-duty",
                next_contact.retry_strafe_right_duty,
                next_contact.retry_strafe_right_duty, true)) {
    sendErrorJson(400, "malformed retry-right-strafe-duty");
    return;
  }
  if (g_server.hasArg("line-reacquire-strafe-duty") &&
      !argFloat("line-reacquire-strafe-duty",
                next_contact.line_reacquire_strafe_duty,
                next_contact.line_reacquire_strafe_duty, true)) {
    sendErrorJson(400, "malformed line-reacquire-strafe-duty");
    return;
  }
  if (g_server.hasArg("rear-line-follow-duration-ms")) {
    if (!argUnsigned("rear-line-follow-duration-ms", milliseconds_value,
                     next_contact.rear_line_follow_duration_ms, true)) {
      sendErrorJson(400, "malformed rear-line-follow-duration-ms");
      return;
    }
    next_contact.rear_line_follow_duration_ms = milliseconds_value;
  }
  if (g_server.hasArg("strafe-delay-ms")) {
    if (!argUnsigned("strafe-delay-ms", milliseconds_value,
                     next_contact.strafe_start_delay_ms, true)) {
      sendErrorJson(400, "malformed strafe-delay-ms");
      return;
    }
    next_contact.strafe_start_delay_ms = milliseconds_value;
  }
  const char* retry_forward_duty_arg =
      g_server.hasArg("retry-forward-duty")
          ? "retry-forward-duty"
          : (g_server.hasArg("strafe-duty") ? "strafe-duty" : nullptr);
  if (retry_forward_duty_arg != nullptr) {
    if (!argFloat(retry_forward_duty_arg, float_value,
                  next_contact.retry_forward_duty, true)) {
      sendErrorJson(400, "malformed retry-forward-duty");
      return;
    }
    next_contact.retry_forward_duty = float_value;
  }
  if (g_server.hasArg("retry-left-ms")) {
    if (!argUnsigned("retry-left-ms", milliseconds_value,
                     next_contact.retry_strafe_left_duration_ms, true)) {
      sendErrorJson(400, "malformed retry-left-ms");
      return;
    }
    next_contact.retry_strafe_left_duration_ms = milliseconds_value;
  }
  if (g_server.hasArg("retry-forward-ms")) {
    if (!argUnsigned("retry-forward-ms", milliseconds_value,
                     next_contact.retry_forward_duration_ms, true)) {
      sendErrorJson(400, "malformed retry-forward-ms");
      return;
    }
    next_contact.retry_forward_duration_ms = milliseconds_value;
  }
  if (g_server.hasArg("retry-strafe-timeout-ms")) {
    if (!argUnsigned("retry-strafe-timeout-ms", milliseconds_value,
                     next_contact.retry_strafe_timeout_ms, true)) {
      sendErrorJson(400, "malformed retry-strafe-timeout-ms");
      return;
    }
    next_contact.retry_strafe_timeout_ms = milliseconds_value;
  }
  if (g_server.hasArg("post-contact-forward-ms")) {
    if (!argUnsigned("post-contact-forward-ms", milliseconds_value,
                     next_contact.post_contact_forward_duration_ms, true)) {
      sendErrorJson(400, "malformed post-contact-forward-ms");
      return;
    }
    next_contact.post_contact_forward_duration_ms = milliseconds_value;
  }
  if (g_server.hasArg("post-contact-forward-duty")) {
    if (!argFloat("post-contact-forward-duty", float_value,
                  next_contact.post_contact_forward_duty, true)) {
      sendErrorJson(400, "malformed post-contact-forward-duty");
      return;
    }
    next_contact.post_contact_forward_duty = float_value;
  }
  if (g_server.hasArg("post-contact-forward-delay-ms")) {
    if (!argUnsigned(
            "post-contact-forward-delay-ms", milliseconds_value,
            next_contact.post_contact_forward_start_delay_ms, true)) {
      sendErrorJson(400, "malformed post-contact-forward-delay-ms");
      return;
    }
    next_contact.post_contact_forward_start_delay_ms = milliseconds_value;
  }
  if (g_server.hasArg("post-forward-strafe-delay-ms")) {
    if (!argUnsigned(
            "post-forward-strafe-delay-ms", milliseconds_value,
            next_contact.line_reacquire_strafe_start_delay_ms, true)) {
      sendErrorJson(400, "malformed post-forward-strafe-delay-ms");
      return;
    }
    next_contact.line_reacquire_strafe_start_delay_ms = milliseconds_value;
  }

  robot::SolarPanelAutonomyConfig validate_1khz = next_config;
  validate_1khz.detection_threshold = next_thresholds.detect_1khz;
  validate_1khz.release_threshold = next_thresholds.release_1khz;
  robot::SolarPanelAutonomyConfig validate_10khz = next_config;
  validate_10khz.detection_threshold = next_thresholds.detect_10khz;
  validate_10khz.release_threshold = next_thresholds.release_10khz;
  if (!solarThresholdsValid(next_thresholds) ||
      !robot::solarPanelAutonomyConfigValid(validate_1khz) ||
      !robot::solarPanelAutonomyConfigValid(validate_10khz) ||
      !solarSpeedConfigValid(next_speed) ||
      !robot::solarPanelContactConfigValid(next_contact) ||
      !autonomousStrafeDutyValid(
          context, next_contact.initial_strafe_right_duty) ||
      !autonomousStrafeDutyValid(
          context, next_contact.retry_strafe_left_duty) ||
      !autonomousStrafeDutyValid(
          context, next_contact.retry_strafe_right_duty) ||
      !autonomousStrafeDutyValid(
          context, next_contact.line_reacquire_strafe_duty)) {
    sendErrorJson(409,
                  "solar config requires release <= detect, alpha [0,1), contact timeouts > 0, duties [0,1]");
    return;
  }

  context.solar_config = next_config;
  context.solar_thresholds = next_thresholds;
  context.solar_speed_config = next_speed;
  context.solar_contact_config = next_contact;
  robot::resetSolarBeaconDetectorState(context.solar_detector);
  context.last_solar_detector_update = {};
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "solar autonomy config updated");
  sendOkJson("solar autonomy config updated");
}

void handleTowerPiecesStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  requestTowerPiecesStart(*g_runtime.context, *g_runtime.front_left,
                          *g_runtime.front_right, *g_runtime.rear_link,
                          now_ms, robot::EventSource::Web);
  sendOkJson("tower pieces start requested");
}

void handleTowerPiecesConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  if (context.tower_pieces.state ==
          robot::TowerPiecesState::RotateClockwise &&
      robot::imuTurnActive(context.imu_turn_state)) {
    sendErrorJson(
        409, "stop the active Tower Pieces IMU turn before changing its angle");
    return;
  }
  robot::TowerPiecesConfig next = context.tower_pieces_config;
  float duty = next.reverse_line_duty;
  robot::Milliseconds timeout_ms = next.side_line_timeout_ms;
  robot::Milliseconds side_line_cooldown_ms =
      next.side_line_cooldown_ms;
  robot::Milliseconds side_line_rearm_ms = next.side_line_rearm_ms;
  robot::Milliseconds post_line_delay_ms = next.post_line_delay_ms;
  robot::Milliseconds strafe_duration_ms =
      next.strafe_right_duration_ms;
  robot::Milliseconds post_strafe_pause_ms =
      next.post_strafe_pause_ms;
  float rotation_angle_deg =
      next.clockwise_rotation_angle_deg;
  robot::Milliseconds post_rotation_pause_ms =
      next.post_rotation_pause_ms;
  float reverse_duty = next.reverse_duty;
  robot::Milliseconds reverse_duration_ms = next.reverse_duration_ms;
  robot::Milliseconds shimmy_right_ms = next.shimmy_right_duration_ms;
  robot::Milliseconds shimmy_left_ms = next.shimmy_left_duration_ms;
  robot::Milliseconds shimmy_timeout_ms = next.shimmy_timeout_ms;
  if (g_server.hasArg("duty") &&
      !argFloat("duty", duty, next.reverse_line_duty, true)) {
    sendErrorJson(400, "malformed duty");
    return;
  }
  if (g_server.hasArg("timeout-ms") &&
      !argUnsigned("timeout-ms", timeout_ms,
                   next.side_line_timeout_ms, true)) {
    sendErrorJson(400, "malformed timeout-ms");
    return;
  }
  if (g_server.hasArg("side-line-cooldown-ms") &&
      !argUnsigned("side-line-cooldown-ms", side_line_cooldown_ms,
                   next.side_line_cooldown_ms, true)) {
    sendErrorJson(400, "malformed side-line-cooldown-ms");
    return;
  }
  if (g_server.hasArg("side-line-rearm-ms") &&
      !argUnsigned("side-line-rearm-ms", side_line_rearm_ms,
                   next.side_line_rearm_ms, true)) {
    sendErrorJson(400, "malformed side-line-rearm-ms");
    return;
  }
  if (g_server.hasArg("post-line-delay-ms") &&
      !argUnsigned("post-line-delay-ms", post_line_delay_ms,
                   next.post_line_delay_ms, true)) {
    sendErrorJson(400, "malformed post-line-delay-ms");
    return;
  }
  if (g_server.hasArg("strafe-duration-ms") &&
      !argUnsigned("strafe-duration-ms", strafe_duration_ms,
                   next.strafe_right_duration_ms, true)) {
    sendErrorJson(400, "malformed strafe-duration-ms");
    return;
  }
  if (g_server.hasArg("strafe-duty") &&
      !argFloat("strafe-duty", next.strafe_right_duty,
                next.strafe_right_duty, true)) {
    sendErrorJson(400, "malformed strafe-duty");
    return;
  }
  if (g_server.hasArg("post-strafe-pause-ms") &&
      !argUnsigned("post-strafe-pause-ms", post_strafe_pause_ms,
                   next.post_strafe_pause_ms, true)) {
    sendErrorJson(400, "malformed post-strafe-pause-ms");
    return;
  }
  if (g_server.hasArg("rotation-angle-deg") &&
      !argFloat("rotation-angle-deg", rotation_angle_deg,
                next.clockwise_rotation_angle_deg, true)) {
    sendErrorJson(400, "malformed rotation-angle-deg");
    return;
  }
  if (g_server.hasArg("post-rotation-pause-ms") &&
      !argUnsigned("post-rotation-pause-ms", post_rotation_pause_ms,
                   next.post_rotation_pause_ms, true)) {
    sendErrorJson(400, "malformed post-rotation-pause-ms");
    return;
  }
  if (g_server.hasArg("reverse-duty") &&
      !argFloat("reverse-duty", reverse_duty, next.reverse_duty, true)) {
    sendErrorJson(400, "malformed reverse-duty");
    return;
  }
  if (g_server.hasArg("reverse-duration-ms") &&
      !argUnsigned("reverse-duration-ms", reverse_duration_ms,
                   next.reverse_duration_ms, true)) {
    sendErrorJson(400, "malformed reverse-duration-ms");
    return;
  }
  if (g_server.hasArg("shimmy-direction-ms")) {
    robot::Milliseconds shared_shimmy_ms = shimmy_right_ms;
    if (!argUnsigned("shimmy-direction-ms", shared_shimmy_ms,
                     shimmy_right_ms, true)) {
      sendErrorJson(400, "malformed shimmy-direction-ms");
      return;
    }
    shimmy_right_ms = shared_shimmy_ms;
    shimmy_left_ms = shared_shimmy_ms;
  }
  if (g_server.hasArg("shimmy-right-ms") &&
      !argUnsigned("shimmy-right-ms", shimmy_right_ms,
                   next.shimmy_right_duration_ms, true)) {
    sendErrorJson(400, "malformed shimmy-right-ms");
    return;
  }
  if (g_server.hasArg("shimmy-left-ms") &&
      !argUnsigned("shimmy-left-ms", shimmy_left_ms,
                   next.shimmy_left_duration_ms, true)) {
    sendErrorJson(400, "malformed shimmy-left-ms");
    return;
  }
  if (g_server.hasArg("shimmy-timeout-ms") &&
      !argUnsigned("shimmy-timeout-ms", shimmy_timeout_ms,
                   next.shimmy_timeout_ms, true)) {
    sendErrorJson(400, "malformed shimmy-timeout-ms");
    return;
  }
  if (g_server.hasArg("shimmy-duty") &&
      !argFloat("shimmy-duty", next.shimmy_duty, next.shimmy_duty, true)) {
    sendErrorJson(400, "malformed shimmy-duty");
    return;
  }
  if (g_server.hasArg("final-reverse-duty") &&
      !argFloat("final-reverse-duty", next.final_reverse_duty,
                context.tower_pieces_config.final_reverse_duty, true)) {
    sendErrorJson(400, "malformed final-reverse-duty");
    return;
  }
  if (g_server.hasArg("final-reverse-duration-ms") &&
      !argUnsigned(
          "final-reverse-duration-ms", next.final_reverse_duration_ms,
          context.tower_pieces_config.final_reverse_duration_ms, true)) {
    sendErrorJson(400, "malformed final-reverse-duration-ms");
    return;
  }
  if (g_server.hasArg("post-final-reverse-delay-ms") &&
      !argUnsigned(
          "post-final-reverse-delay-ms", next.post_final_reverse_delay_ms,
          context.tower_pieces_config.post_final_reverse_delay_ms, true)) {
    sendErrorJson(400, "malformed post-final-reverse-delay-ms");
    return;
  }
  if (g_server.hasArg("post-winch-open-delay-ms") &&
      !argUnsigned(
          "post-winch-open-delay-ms", next.post_winch_open_delay_ms,
          context.tower_pieces_config.post_winch_open_delay_ms, true)) {
    sendErrorJson(400, "malformed post-winch-open-delay-ms");
    return;
  }
  if (g_server.hasArg("pre-stepper-bottom-delay-ms") &&
      !argUnsigned(
          "pre-stepper-bottom-delay-ms",
          next.pre_stepper_bottom_delay_ms,
          context.tower_pieces_config.pre_stepper_bottom_delay_ms, true)) {
    sendErrorJson(400, "malformed pre-stepper-bottom-delay-ms");
    return;
  }
  if (g_server.hasArg("post-claws-open-delay-ms") &&
      !argUnsigned(
          "post-claws-open-delay-ms", next.post_claws_open_delay_ms,
          context.tower_pieces_config.post_claws_open_delay_ms, true)) {
    sendErrorJson(400, "malformed post-claws-open-delay-ms");
    return;
  }
  if (g_server.hasArg("stepper-down-speed-steps-per-second") &&
      !argUnsigned(
          "stepper-down-speed-steps-per-second",
          next.stepper_down_speed_steps_per_second,
          context.tower_pieces_config.stepper_down_speed_steps_per_second,
          true)) {
    sendErrorJson(400,
                  "malformed stepper-down-speed-steps-per-second");
    return;
  }
  if (g_server.hasArg("post-stepper-bottom-delay-ms") &&
      !argUnsigned(
          "post-stepper-bottom-delay-ms",
          next.post_stepper_bottom_delay_ms,
          context.tower_pieces_config.post_stepper_bottom_delay_ms, true)) {
    sendErrorJson(400, "malformed post-stepper-bottom-delay-ms");
    return;
  }
  if (g_server.hasArg("post-claws-closed-delay-ms") &&
      !argUnsigned(
          "post-claws-closed-delay-ms", next.post_claws_closed_delay_ms,
          context.tower_pieces_config.post_claws_closed_delay_ms, true)) {
    sendErrorJson(400, "malformed post-claws-closed-delay-ms");
    return;
  }
  if (g_server.hasArg("stepper-up-speed-steps-per-second") &&
      !argUnsigned(
          "stepper-up-speed-steps-per-second",
          next.stepper_up_speed_steps_per_second,
          context.tower_pieces_config.stepper_up_speed_steps_per_second,
          true)) {
    sendErrorJson(400, "malformed stepper-up-speed-steps-per-second");
    return;
  }
  next.reverse_line_duty = duty;
  next.side_line_timeout_ms = timeout_ms;
  next.side_line_cooldown_ms = side_line_cooldown_ms;
  next.side_line_rearm_ms = side_line_rearm_ms;
  next.post_line_delay_ms = post_line_delay_ms;
  next.strafe_right_duration_ms = strafe_duration_ms;
  next.post_strafe_pause_ms = post_strafe_pause_ms;
  next.clockwise_rotation_angle_deg = rotation_angle_deg;
  next.post_rotation_pause_ms = post_rotation_pause_ms;
  next.reverse_duty = reverse_duty;
  next.reverse_duration_ms = reverse_duration_ms;
  next.shimmy_right_duration_ms = shimmy_right_ms;
  next.shimmy_left_duration_ms = shimmy_left_ms;
  next.shimmy_timeout_ms = shimmy_timeout_ms;
  if (!robot::towerPiecesConfigValid(
          next, rearMotionDutyCap(context),
          g_runtime.stepper->maximumSpeedStepsPerSecond()) ||
      !autonomousStrafeDutyValid(context, next.strafe_right_duty) ||
      !autonomousStrafeDutyValid(context, next.shimmy_duty)) {
    sendErrorJson(
        409,
        "tower pieces config requires a positive clockwise angle, safe nonzero drive duties/timings, positive delays/speeds, and a valid optional final reverse");
    return;
  }

  context.tower_pieces_config = next;
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "tower pieces config updated");
  sendOkJson("tower pieces config updated");
}

void handlePegFinderStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  requestPegFinderStart(*g_runtime.context, *g_runtime.front_left,
                        *g_runtime.front_right, *g_runtime.rear_link,
                        now_ms, robot::EventSource::Web);
  sendOkJson("PegFinder start requested");
}

void handlePegFinderConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  if (context.peg_finder.state ==
          robot::PegFinderState::RotateClockwise &&
      robot::imuTurnActive(context.imu_turn_state)) {
    sendErrorJson(
        409, "stop the active PegFinder IMU turn before changing config");
    return;
  }
  robot::PegFinderConfig next = context.peg_finder_config;
  if (g_server.hasArg("clockwise-angle-deg") &&
      !argFloat("clockwise-angle-deg", next.clockwise_angle_deg,
                context.peg_finder_config.clockwise_angle_deg, true)) {
    sendErrorJson(400, "malformed clockwise-angle-deg");
    return;
  }
  if (g_server.hasArg("post-rotation-pause-ms") &&
      !argUnsigned("post-rotation-pause-ms",
                   next.post_rotation_pause_ms,
                   context.peg_finder_config.post_rotation_pause_ms,
                   true)) {
    sendErrorJson(400, "malformed post-rotation-pause-ms");
    return;
  }
  if (g_server.hasArg("reverse-duty") &&
      !argFloat("reverse-duty", next.reverse_duty,
                context.peg_finder_config.reverse_duty, true)) {
    sendErrorJson(400, "malformed reverse-duty");
    return;
  }
  if (g_server.hasArg("reverse-duration-ms") &&
      !argUnsigned("reverse-duration-ms", next.reverse_duration_ms,
                   context.peg_finder_config.reverse_duration_ms, true)) {
    sendErrorJson(400, "malformed reverse-duration-ms");
    return;
  }
  if (g_server.hasArg("post-reverse-pause-ms") &&
      !argUnsigned("post-reverse-pause-ms", next.post_reverse_pause_ms,
                   context.peg_finder_config.post_reverse_pause_ms, true)) {
    sendErrorJson(400, "malformed post-reverse-pause-ms");
    return;
  }
  if (g_server.hasArg("forward-duty") &&
      !argFloat("forward-duty", next.forward_duty,
                context.peg_finder_config.forward_duty, true)) {
    sendErrorJson(400, "malformed forward-duty");
    return;
  }
  if (g_server.hasArg("forward-duration-ms") &&
      !argUnsigned("forward-duration-ms", next.forward_duration_ms,
                   context.peg_finder_config.forward_duration_ms, true)) {
    sendErrorJson(400, "malformed forward-duration-ms");
    return;
  }
  if (g_server.hasArg("funnel-duty") &&
      !argFloat("funnel-duty", next.funnel_forward_duty,
                context.peg_finder_config.funnel_forward_duty, true)) {
    sendErrorJson(400, "malformed funnel-duty");
    return;
  }
  const char* funnel_timeout_arg =
      g_server.hasArg("funnel-timeout-ms")
          ? "funnel-timeout-ms"
          : (g_server.hasArg("funnel-duration-ms")
                 ? "funnel-duration-ms"
                 : nullptr);
  if (funnel_timeout_arg != nullptr &&
      !argUnsigned(funnel_timeout_arg, next.funnel_forward_timeout_ms,
                   context.peg_finder_config.funnel_forward_timeout_ms,
                   true)) {
    sendErrorJson(400, "malformed funnel-timeout-ms");
    return;
  }
  if (g_server.hasArg("post-funnel-limit-delay-ms") &&
      !argUnsigned(
          "post-funnel-limit-delay-ms",
          next.post_funnel_limit_delay_ms,
          context.peg_finder_config.post_funnel_limit_delay_ms, true)) {
    sendErrorJson(400, "malformed post-funnel-limit-delay-ms");
    return;
  }
  if (g_server.hasArg("claw-open-interval-ms") &&
      !argUnsigned("claw-open-interval-ms",
                   next.claw_open_interval_ms,
                   context.peg_finder_config.claw_open_interval_ms, true)) {
    sendErrorJson(400, "malformed claw-open-interval-ms");
    return;
  }
  robot::Milliseconds claw_order_1 = next.claw_open_order_1;
  robot::Milliseconds claw_order_2 = next.claw_open_order_2;
  robot::Milliseconds claw_order_3 = next.claw_open_order_3;
  if (g_server.hasArg("claw-order-1") &&
      (!argUnsigned("claw-order-1", claw_order_1,
                    context.peg_finder_config.claw_open_order_1, true) ||
       claw_order_1 < 1U || claw_order_1 > 3U)) {
    sendErrorJson(400, "claw-order-1 must be 1, 2, or 3");
    return;
  }
  if (g_server.hasArg("claw-order-2") &&
      (!argUnsigned("claw-order-2", claw_order_2,
                    context.peg_finder_config.claw_open_order_2, true) ||
       claw_order_2 < 1U || claw_order_2 > 3U)) {
    sendErrorJson(400, "claw-order-2 must be 1, 2, or 3");
    return;
  }
  if (g_server.hasArg("claw-order-3") &&
      (!argUnsigned("claw-order-3", claw_order_3,
                    context.peg_finder_config.claw_open_order_3, true) ||
       claw_order_3 < 1U || claw_order_3 > 3U)) {
    sendErrorJson(400, "claw-order-3 must be 1, 2, or 3");
    return;
  }
  next.claw_open_order_1 = static_cast<std::uint8_t>(claw_order_1);
  next.claw_open_order_2 = static_cast<std::uint8_t>(claw_order_2);
  next.claw_open_order_3 = static_cast<std::uint8_t>(claw_order_3);
  if (g_server.hasArg("post-claws-open-delay-ms") &&
      !argUnsigned(
          "post-claws-open-delay-ms", next.post_claws_open_delay_ms,
          context.peg_finder_config.post_claws_open_delay_ms, true)) {
    sendErrorJson(400, "malformed post-claws-open-delay-ms");
    return;
  }
  if (g_server.hasArg("funnel-reverse-duty") &&
      !argFloat("funnel-reverse-duty", next.funnel_reverse_duty,
                context.peg_finder_config.funnel_reverse_duty, true)) {
    sendErrorJson(400, "malformed funnel-reverse-duty");
    return;
  }
  if (g_server.hasArg("funnel-reverse-duration-ms") &&
      !argUnsigned(
          "funnel-reverse-duration-ms", next.funnel_reverse_duration_ms,
          context.peg_finder_config.funnel_reverse_duration_ms, true)) {
    sendErrorJson(400, "malformed funnel-reverse-duration-ms");
    return;
  }
  if (!robot::pegFinderConfigValid(next, rearMotionDutyCap(context),
                                    funnelMotionDutyCap())) {
    sendErrorJson(
        409,
        "PegFinder config requires a positive clockwise angle, safe nonzero duties, motion timings, a unique claw order containing 1, 2, and 3, and both funnel phases");
    return;
  }

  context.peg_finder_config = next;
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "PegFinder config updated");
  sendOkJson("PegFinder config updated");
}

void handleTimeTrialStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  requestTimeTrialStart(*g_runtime.context, *g_runtime.front_left,
                        *g_runtime.front_right, *g_runtime.rear_link, now_ms,
                        robot::EventSource::Web);
  sendOkJson("Time Trial start requested");
}

void handleTimeTrialConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  robot::TimeTrialConfig next = context.time_trial_config;
  if (g_server.hasArg("post-solar-delay-ms") &&
      !argUnsigned("post-solar-delay-ms", next.post_solar_delay_ms,
                   context.time_trial_config.post_solar_delay_ms, true)) {
    sendErrorJson(400, "malformed post-solar-delay-ms");
    return;
  }
  if (g_server.hasArg("strafe-right-duration-ms") &&
      !argUnsigned(
          "strafe-right-duration-ms",
          next.solar_to_tower_strafe_right_duration_ms,
          context.time_trial_config
              .solar_to_tower_strafe_right_duration_ms,
          true)) {
    sendErrorJson(400, "malformed strafe-right-duration-ms");
    return;
  }
  if (g_server.hasArg("post-tower-delay-ms") &&
      !argUnsigned("post-tower-delay-ms", next.post_tower_delay_ms,
                   context.time_trial_config.post_tower_delay_ms, true)) {
    sendErrorJson(400, "malformed post-tower-delay-ms");
    return;
  }
  if (!robot::timeTrialConfigValid(next, rearMotionDutyCap(context))) {
    sendErrorJson(
        409,
        "Time Trial transition configuration is invalid");
    return;
  }

  context.time_trial_config = next;
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "Time Trial transition config updated");
  sendOkJson("Time Trial transition config updated");
}

void handleImuTurnConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  if (robot::imuTurnActive(context.imu_turn_state)) {
    sendErrorJson(409, "stop the active IMU turn before changing tuning");
    return;
  }

  robot::ImuTurnConfig next = context.imu_turn_config;
  if (g_server.hasArg("max-duty") &&
      !argFloat("max-duty", next.maximum_rotation_duty,
                next.maximum_rotation_duty, true)) {
    sendErrorJson(400, "malformed max-duty");
    return;
  }
  if (g_server.hasArg("kp") &&
      !argFloat("kp", next.kp, next.kp, true)) {
    sendErrorJson(400, "malformed kp");
    return;
  }
  if (g_server.hasArg("kd") &&
      !argFloat("kd", next.kd, next.kd, true)) {
    sendErrorJson(400, "malformed kd");
    return;
  }
  if (g_server.hasArg("tolerance-deg") &&
      !argFloat("tolerance-deg", next.angle_tolerance_deg,
                next.angle_tolerance_deg, true)) {
    sendErrorJson(400, "malformed tolerance-deg");
    return;
  }
  if (g_server.hasArg("finish-rate-dps") &&
      !argFloat("finish-rate-dps",
                next.maximum_finishing_yaw_rate_dps,
                next.maximum_finishing_yaw_rate_dps, true)) {
    sendErrorJson(400, "malformed finish-rate-dps");
    return;
  }
  if (g_server.hasArg("settle-ms") &&
      !argUnsigned("settle-ms", next.settling_time_ms,
                   next.settling_time_ms, true)) {
    sendErrorJson(400, "malformed settle-ms");
    return;
  }
  if (g_server.hasArg("timeout-ms") &&
      !argUnsigned("timeout-ms", next.timeout_ms, next.timeout_ms, true)) {
    sendErrorJson(400, "malformed timeout-ms");
    return;
  }
  if (g_server.hasArg("polarity") &&
      !argPolarity("polarity", next.yaw_command_polarity,
                   next.yaw_command_polarity, true)) {
    sendErrorJson(400, "polarity must be +1 or -1");
    return;
  }

  if (!imuTurnRuntimeConfigValid(next)) {
    sendErrorJson(
        409,
        "IMU turn tuning incomplete: duty/Kp/tolerances/times must be positive, Kd nonnegative, timeout greater than settling and at most 30000 ms, duty within hardware cap, and polarity +1 or -1");
    return;
  }

  context.imu_turn_config = next;
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "IMU turn tuning updated");
  sendOkJson("IMU turn tuning updated");
}

void handleImuTurnStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  float relative_angle_deg = 0.0F;
  if (!argFloat("degrees", relative_angle_deg, 0.0F, true) ||
      (relative_angle_deg != 90.0F &&
       relative_angle_deg != -90.0F)) {
    sendErrorJson(400, "degrees must be +90 or -90");
    return;
  }

  // A rejected turn request must not allow a previously active mode to resume.
  disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                   *g_runtime.rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::ImuTurnTest, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  resetImuTurn(context);

  if (!imuTurnRuntimeConfigValid(context.imu_turn_config)) {
    robot::faultImuTurn(
        context.imu_turn_state,
        robot::ImuTurnFaultReason::InvalidConfiguration);
    setFault(context, robot::FaultCode::InvalidCommand,
             "IMU turn tuning is incomplete");
    sendErrorJson(409, "IMU turn tuning is incomplete");
    return;
  }

  const robot::esp2::ImuAcquisitionSnapshot& imu_snapshot =
      context.latest_imu_snapshot;
  const robot::esp2::ImuState& imu_state = imu_snapshot.state;
  if (context.imu_heading_reset_pending_sequence != 0U) {
    sendErrorJson(409, "IMU angle reset is still being applied");
    return;
  }
  const std::uint32_t imu_availability_now_us = micros();
  const char* const imu_availability_reason =
      robot::esp2::imuDisconnectReason(
          imu_snapshot, imu_availability_now_us,
          kImuFreshnessTimeoutUs);
  if (std::strcmp(imu_availability_reason, "NONE") != 0) {
    captureImuTurnAvailabilityFault(
        context, imu_snapshot, *g_runtime.front_left,
        *g_runtime.front_right, *g_runtime.rear_link, now_ms,
        imu_availability_now_us, "TURN_START_GATE",
        imu_availability_reason);
    robot::faultImuTurn(context.imu_turn_state,
                        robot::ImuTurnFaultReason::ImuUnavailable);
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "IMU turn unavailable: IMU unhealthy");
    sendErrorJson(409, "IMU turn unavailable: IMU unhealthy or stale");
    return;
  }
  if (!g_runtime.front_left->configured() ||
      !g_runtime.front_right->configured()) {
    robot::faultImuTurn(context.imu_turn_state,
                        robot::ImuTurnFaultReason::CommandFailed);
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "IMU turn unavailable: front motors invalid");
    sendErrorJson(409, "IMU turn unavailable: front motors invalid");
    return;
  }
  if (!g_runtime.rear_link->configured() ||
      !g_runtime.rear_link->remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(context.config))) {
    robot::faultImuTurn(
        context.imu_turn_state,
        robot::ImuTurnFaultReason::RearLinkUnavailable);
    setFault(context, robot::FaultCode::CommunicationStale,
             "IMU turn unavailable: rear link unhealthy");
    sendErrorJson(409, "IMU turn unavailable: rear link unhealthy");
    return;
  }

  if (!robot::startImuTurn(
          context.imu_turn_state, imu_state.heading_deg,
          relative_angle_deg, context.imu_turn_config,
          hardwareDutyCap(), now_ms)) {
    setFault(context, robot::FaultCode::InvalidCommand,
             "IMU turn controller rejected start");
    sendErrorJson(409, "IMU turn controller rejected start");
    return;
  }

  context.last_imu_turn_update = {};
  context.last_imu_turn_update.state = context.imu_turn_state.state;
  context.last_imu_turn_update.current_heading_deg =
      imu_state.heading_deg;
  context.last_imu_turn_update.target_heading_deg =
      context.imu_turn_state.target_heading_deg;
  context.last_imu_turn_update.angle_error_deg = relative_angle_deg;
  context.last_imu_turn_update.yaw_rate_dps = imu_state.yaw_rate_dps;
  context.last_command_ms = now_ms;
  context.command_deadman_armed = false;
  context.mode_expires_at_ms = 0U;
  resetMotionDiagnosticCapture(context, now_ms);
  recordMotionDiagnostic(
      context, *g_runtime.front_left, *g_runtime.front_right,
      *g_runtime.rear_link, &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::ImuTurnStarted, now_ms);
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web,
           relative_angle_deg > 0.0F
               ? "IMU +90 degree turn started"
               : "IMU -90 degree turn started");
  sendOkJson(relative_angle_deg > 0.0F
                 ? "IMU +90 degree turn started"
                 : "IMU -90 degree turn started");
}

void handleImuTurnStop() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  recordMotionDiagnostic(
      context, *g_runtime.front_left, *g_runtime.front_right,
      *g_runtime.rear_link, &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::ImuTurnStopped, now_ms);
  disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                   *g_runtime.rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::ImuTurnTest, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  robot::stopImuTurn(context.imu_turn_state);
  context.last_imu_turn_update = {};
  context.last_imu_turn_update.state = context.imu_turn_state.state;
  recordMotionDiagnostic(
      context, *g_runtime.front_left, *g_runtime.front_right,
      *g_runtime.rear_link, &context.latest_imu_snapshot,
      robot::MotionDiagnosticEvent::OutputsDisabled, now_ms);
  scheduleMotionDiagnosticFreeze(
      context, robot::MotionDiagnosticTrigger::ManualFreeze);
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Warn,
           robot::EventSource::Web, "IMU turn stopped");
  sendOkJson("IMU turn stopped");
}

void handleImuAngleReset() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  if (robot::imuTurnActive(context.imu_turn_state)) {
    sendErrorJson(409, "stop the active IMU turn before resetting angle");
    return;
  }
  if (robot::imuHeadingHoldActive(context.imu_heading_hold_state)) {
    sendErrorJson(
        409, "stop the active IMU strafe before resetting angle");
    return;
  }

  std::uint32_t reset_sequence = 0U;
  if (!g_runtime.imu_acquisition->requestHeadingReset(
          0.0F, reset_sequence)) {
    sendErrorJson(503, "IMU acquisition task is unavailable");
    return;
  }
  context.imu_heading_reset_pending_sequence = reset_sequence;
  resetImuTurn(context);
  resetImuHeadingHold(context);
  if (context.modes.currentMode() ==
          robot::RobotTestMode::ImuTurnTest ||
      context.modes.currentMode() ==
          robot::RobotTestMode::ImuStrafeTest) {
    clearFault(context);
  }
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "IMU relative heading reset requested");
  sendOkJson("IMU angle reset queued");
}

void handleImuTurnSave() {
  if (!runtimeReady() || g_runtime.preferences == nullptr) {
    sendErrorJson(503, "preferences unavailable");
    return;
  }
  const robot::ImuTurnConfig& config =
      g_runtime.context->imu_turn_config;
  if (!imuTurnRuntimeConfigValid(config)) {
    sendErrorJson(409, "apply valid IMU turn tuning before saving");
    return;
  }
  g_runtime.preferences->putFloat("itmax",
                                  config.maximum_rotation_duty);
  g_runtime.preferences->putFloat("itkp", config.kp);
  g_runtime.preferences->putFloat("itkd", config.kd);
  g_runtime.preferences->putFloat("ittol",
                                  config.angle_tolerance_deg);
  g_runtime.preferences->putFloat(
      "itrate", config.maximum_finishing_yaw_rate_dps);
  g_runtime.preferences->putUInt("itsettle",
                                 config.settling_time_ms);
  g_runtime.preferences->putUInt("ittimeout", config.timeout_ms);
  g_runtime.preferences->putInt("itpol", config.yaw_command_polarity);
  sendOkJson("IMU turn tuning saved");
}

void handleImuHeadingHoldConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  if (robot::imuHeadingHoldActive(context.imu_heading_hold_state)) {
    sendErrorJson(
        409, "stop the active IMU strafe before changing tuning");
    return;
  }

  robot::ImuHeadingHoldConfig next =
      context.imu_heading_hold_config;
  if (g_server.hasArg("strafe-duty") &&
      !argFloat("strafe-duty", next.maximum_strafe_duty,
                next.maximum_strafe_duty, true)) {
    sendErrorJson(400, "malformed strafe-duty");
    return;
  }
  if (g_server.hasArg("kp") &&
      !argFloat("kp", next.kp, next.kp, true)) {
    sendErrorJson(400, "malformed kp");
    return;
  }
  if (g_server.hasArg("kd") &&
      !argFloat("kd", next.kd, next.kd, true)) {
    sendErrorJson(400, "malformed kd");
    return;
  }
  if (g_server.hasArg("max-correction-duty") &&
      !argFloat("max-correction-duty",
                next.maximum_yaw_correction_duty,
                next.maximum_yaw_correction_duty, true)) {
    sendErrorJson(400, "malformed max-correction-duty");
    return;
  }
  if (g_server.hasArg("polarity") &&
      !argPolarity("polarity", next.yaw_command_polarity,
                   next.yaw_command_polarity, true)) {
    sendErrorJson(400, "polarity must be +1 or -1");
    return;
  }

  if (!imuHeadingHoldRuntimeConfigValid(next)) {
    sendErrorJson(
        409,
        "IMU strafe tuning incomplete: strafe duty, Kp, and correction duty must be positive, Kd nonnegative, polarity +1 or -1, and strafe plus correction duty must not exceed the hardware cap");
    return;
  }

  context.imu_heading_hold_config = next;
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "IMU strafe tuning updated");
  sendOkJson("IMU strafe tuning updated");
}

void handleImuHeadingHoldStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  int lateral_direction = 0;
  if (!argSigned("direction", lateral_direction, 0, true) ||
      (lateral_direction != -1 && lateral_direction != 1)) {
    sendErrorJson(400, "direction must be -1 (left) or +1 (right)");
    return;
  }

  // A rejected request must not allow a previous motion mode to resume.
  disableActuators(context, *g_runtime.front_left,
                   *g_runtime.front_right, *g_runtime.rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::ImuStrafeTest, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  resetImuTurn(context);
  resetImuHeadingHold(context);

  if (!imuHeadingHoldRuntimeConfigValid(
          context.imu_heading_hold_config)) {
    robot::faultImuHeadingHold(
        context.imu_heading_hold_state,
        robot::ImuHeadingHoldFaultReason::InvalidConfiguration);
    setFault(context, robot::FaultCode::InvalidCommand,
             "IMU strafe tuning is incomplete");
    sendErrorJson(409, "IMU strafe tuning is incomplete");
    return;
  }

  const robot::esp2::ImuAcquisitionSnapshot& imu_snapshot =
      context.latest_imu_snapshot;
  const robot::esp2::ImuState& imu_state = imu_snapshot.state;
  if (context.imu_heading_reset_pending_sequence != 0U) {
    sendErrorJson(409, "IMU angle reset is still being applied");
    return;
  }
  if (std::strcmp(robot::esp2::imuDisconnectReason(
                      imu_snapshot, micros(), kImuFreshnessTimeoutUs),
                  "NONE") != 0) {
    robot::faultImuHeadingHold(
        context.imu_heading_hold_state,
        robot::ImuHeadingHoldFaultReason::ImuUnavailable);
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "IMU strafe unavailable: IMU unhealthy");
    sendErrorJson(409, "IMU strafe unavailable: IMU unhealthy or stale");
    return;
  }
  if (!g_runtime.front_left->configured() ||
      !g_runtime.front_right->configured()) {
    robot::faultImuHeadingHold(
        context.imu_heading_hold_state,
        robot::ImuHeadingHoldFaultReason::CommandFailed);
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "IMU strafe unavailable: front motors invalid");
    sendErrorJson(409, "IMU strafe unavailable: front motors invalid");
    return;
  }
  if (!g_runtime.rear_link->configured() ||
      !g_runtime.rear_link->remoteStatusFresh(
          now_ms, remoteStatusTimeoutMs(context.config))) {
    robot::faultImuHeadingHold(
        context.imu_heading_hold_state,
        robot::ImuHeadingHoldFaultReason::RearLinkUnavailable);
    setFault(context, robot::FaultCode::CommunicationStale,
             "IMU strafe unavailable: rear link unhealthy");
    sendErrorJson(409, "IMU strafe unavailable: rear link unhealthy");
    return;
  }

  if (!robot::startImuHeadingHold(
          context.imu_heading_hold_state, imu_state.heading_deg,
          lateral_direction, context.imu_heading_hold_config,
          hardwareDutyCap(), now_ms)) {
    setFault(context, robot::FaultCode::InvalidCommand,
             "IMU strafe controller rejected start");
    sendErrorJson(409, "IMU strafe controller rejected start");
    return;
  }

  context.last_imu_heading_hold_update = {};
  context.last_imu_heading_hold_update.state =
      context.imu_heading_hold_state.state;
  context.last_imu_heading_hold_update.current_heading_deg =
      imu_state.heading_deg;
  context.last_imu_heading_hold_update.target_heading_deg =
      context.imu_heading_hold_state.target_heading_deg;
  context.last_imu_heading_hold_update.yaw_rate_dps =
      imu_state.yaw_rate_dps;
  context.last_imu_heading_hold_update.lateral_direction =
      lateral_direction;
  context.last_imu_heading_hold_update.should_strafe = true;
  context.last_command_ms = now_ms;
  context.command_deadman_armed = true;
  context.mode_expires_at_ms = now_ms + kCommandTimeoutMs;
  resetMotionDiagnosticCapture(context, now_ms);
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web,
           lateral_direction < 0
               ? "IMU heading-held left strafe started"
               : "IMU heading-held right strafe started");
  sendOkJson(lateral_direction < 0
                 ? "IMU heading-held left strafe started"
                 : "IMU heading-held right strafe started");
}

void handleImuHeadingHoldHeartbeat() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  if (context.modes.currentMode() !=
          robot::RobotTestMode::ImuStrafeTest ||
      !robot::imuHeadingHoldActive(context.imu_heading_hold_state)) {
    sendErrorJson(409, "no active IMU strafe");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  context.last_command_ms = now_ms;
  context.command_deadman_armed = true;
  context.mode_expires_at_ms = now_ms + kCommandTimeoutMs;
  sendOkJson("IMU strafe heartbeat accepted");
}

void handleImuHeadingHoldStop() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  disableMotionActuators(context, *g_runtime.front_left,
                         *g_runtime.front_right,
                         *g_runtime.rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::ImuStrafeTest, now_ms);
  robot::stopImuHeadingHold(context.imu_heading_hold_state);
  context.last_imu_heading_hold_update = {};
  context.last_imu_heading_hold_update.state =
      context.imu_heading_hold_state.state;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Warn,
           robot::EventSource::Web, "IMU strafe stopped");
  sendOkJson("IMU strafe stopped");
}

void handleImuHeadingHoldSave() {
  if (!runtimeReady() || g_runtime.preferences == nullptr) {
    sendErrorJson(503, "preferences unavailable");
    return;
  }
  const robot::ImuHeadingHoldConfig& config =
      g_runtime.context->imu_heading_hold_config;
  if (!imuHeadingHoldRuntimeConfigValid(config)) {
    sendErrorJson(409, "apply valid IMU strafe tuning before saving");
    return;
  }
  g_runtime.preferences->putFloat(
      "ihmax", config.maximum_strafe_duty);
  g_runtime.preferences->putFloat("ihkp", config.kp);
  g_runtime.preferences->putFloat("ihkd", config.kd);
  g_runtime.preferences->putFloat(
      "ihcorr", config.maximum_yaw_correction_duty);
  g_runtime.preferences->putInt(
      "ihpol", config.yaw_command_polarity);
  sendOkJson("IMU strafe tuning saved");
}

bool parseClawId(std::size_t& claw_index) {
  int claw_id = 0;
  if (!argSigned("id", claw_id, 0, true) || claw_id < 1 ||
      claw_id > static_cast<int>(kClawServoCount)) {
    return false;
  }
  claw_index = static_cast<std::size_t>(claw_id - 1);
  return true;
}

bool parseClawRequest(ClawServoPositionRequest& request) {
  if (!g_server.hasArg("state")) {
    return false;
  }
  const String state = g_server.arg("state");
  if (state == "open") {
    request = ClawServoPositionRequest::Open;
    return true;
  }
  if (state == "close" || state == "closed") {
    request = ClawServoPositionRequest::Closed;
    return true;
  }
  return false;
}

robot::FaultCode clawFaultCode(const ClawServoCommandResult result) {
  return result == ClawServoCommandResult::HardwareUnconfigured ||
                 result == ClawServoCommandResult::PwmWriteFailed
             ? robot::FaultCode::HardwareNotConfigured
             : robot::FaultCode::InvalidCommand;
}

bool readOptionalClawInt(const char* primary_name, const char* alternate_name,
                         int& value, const char* malformed_reason) {
  const char* name = nullptr;
  if (g_server.hasArg(primary_name)) {
    name = primary_name;
  } else if (g_server.hasArg(alternate_name)) {
    name = alternate_name;
  }
  if (name == nullptr) {
    return true;
  }
  if (!argSigned(name, value, value, true)) {
    sendErrorJson(400, malformed_reason);
    return false;
  }
  return true;
}

bool parseClawSettings(ClawServoSettings& settings) {
  constexpr const char* kOpenNames[kClawServoCount][2] = {
      {"claw1-open", "claw1Open"},
      {"claw2-open", "claw2Open"},
      {"claw3-open", "claw3Open"},
  };
  constexpr const char* kClosedNames[kClawServoCount][2] = {
      {"claw1-closed", "claw1Closed"},
      {"claw2-closed", "claw2Closed"},
      {"claw3-closed", "claw3Closed"},
  };
  constexpr const char* kLegacyStartNames[kClawServoCount][2] = {
      {"claw1-start", "claw1Start"},
      {"claw2-start", "claw2Start"},
      {"claw3-start", "claw3Start"},
  };
  constexpr const char* kLegacyDirectionNames[kClawServoCount][2] = {
      {"claw1-dir", "claw1Direction"},
      {"claw2-dir", "claw2Direction"},
      {"claw3-dir", "claw3Direction"},
  };

  for (std::size_t index = 0U; index < kClawServoCount; ++index) {
    const bool has_open = g_server.hasArg(kOpenNames[index][0]) ||
                          g_server.hasArg(kOpenNames[index][1]);
    const bool has_closed = g_server.hasArg(kClosedNames[index][0]) ||
                            g_server.hasArg(kClosedNames[index][1]);
    const bool has_legacy_start =
        g_server.hasArg(kLegacyStartNames[index][0]) ||
        g_server.hasArg(kLegacyStartNames[index][1]);
    const bool has_legacy_direction =
        g_server.hasArg(kLegacyDirectionNames[index][0]) ||
        g_server.hasArg(kLegacyDirectionNames[index][1]);

    int value = settings.open_angle_deg[index];
    if (!readOptionalClawInt(kOpenNames[index][0], kOpenNames[index][1],
                             value, "malformed claw open angle")) {
      return false;
    }
    settings.open_angle_deg[index] = value;

    value = settings.closed_angle_deg[index];
    if (!readOptionalClawInt(kClosedNames[index][0], kClosedNames[index][1],
                             value, "malformed claw closed angle")) {
      return false;
    }
    settings.closed_angle_deg[index] = value;

    // Backward compatibility for the previous start + direction API. Start
    // represented closed, and open was derived as start +/- 90 degrees.
    if (!has_closed && has_legacy_start) {
      value = settings.closed_angle_deg[index];
      if (!readOptionalClawInt(kLegacyStartNames[index][0],
                               kLegacyStartNames[index][1], value,
                               "malformed legacy claw start angle")) {
        return false;
      }
      settings.closed_angle_deg[index] = value;
    }
    if (!has_open && (has_legacy_start || has_legacy_direction)) {
      int direction = 1;
      if (!readOptionalClawInt(kLegacyDirectionNames[index][0],
                               kLegacyDirectionNames[index][1], direction,
                               "malformed legacy claw direction")) {
        return false;
      }
      if (direction != 1 && direction != -1) {
        sendErrorJson(400, "legacy claw direction must be 1 or -1");
        return false;
      }
      if (settings.closed_angle_deg[index] != kClawServoUnsetAngleDeg) {
        settings.open_angle_deg[index] =
            settings.closed_angle_deg[index] +
            (direction * kLegacyClawServoRotationDeg);
      }
    }
  }

  int value = settings.open_angle_deg[kWinchServoIndex];
  if (!readOptionalClawInt("pusher-open", "pusherOpen",
                           settings.open_angle_deg[kHabitatPusherServoIndex],
                           "malformed Habitat Pusher open angle")) {
    return false;
  }
  if (!readOptionalClawInt("pusher-closed", "pusherClosed",
                           settings.closed_angle_deg[kHabitatPusherServoIndex],
                           "malformed Habitat Pusher closed angle")) {
    return false;
  }

  value = settings.open_angle_deg[kWinchServoIndex];
  if (!readOptionalClawInt("winch-open", "winchOpen", value,
                           "malformed winch open angle")) {
    return false;
  }
  settings.open_angle_deg[kWinchServoIndex] = value;

  value = settings.closed_angle_deg[kWinchServoIndex];
  if (!readOptionalClawInt("winch-closed", "winchClosed", value,
                           "malformed winch closed angle")) {
    return false;
  }
  settings.closed_angle_deg[kWinchServoIndex] = value;
  return true;
}

void enterMechanismTestIfNeeded(RuntimeContext& context,
                                const robot::Milliseconds now_ms) {
  if (context.modes.currentMode() == robot::RobotTestMode::MechanismTest) {
    return;
  }
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                   *g_runtime.rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::MechanismTest, now_ms);
}

bool solarHookAngleSettingValid(const int angle_deg) {
  return angle_deg == kClawServoUnsetAngleDeg ||
         (angle_deg >= 0 && angle_deg <= 180);
}

bool parseSolarHookSettings(SolarHookServoSettings& settings) {
  int open_angle_deg = settings.open_angle_deg;
  int closed_angle_deg = settings.closed_angle_deg;
  if (!argSigned("open-angle", open_angle_deg, open_angle_deg, false) ||
      !argSigned("closed-angle", closed_angle_deg, closed_angle_deg,
                 false)) {
    sendErrorJson(400, "malformed solar hook angle");
    return false;
  }
  if (!solarHookAngleSettingValid(open_angle_deg) ||
      !solarHookAngleSettingValid(closed_angle_deg)) {
    sendErrorJson(409, "solar hook angles must be 0..180 degrees");
    return false;
  }
  settings.open_angle_deg = open_angle_deg;
  settings.closed_angle_deg = closed_angle_deg;
  return true;
}

void handleSolarHookConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  SolarHookServoSettings next =
      g_runtime.context->solar_hook_servo_settings;
  if (!parseSolarHookSettings(next)) {
    return;
  }
  g_runtime.context->solar_hook_servo_settings = next;
  clearFault(*g_runtime.context);
  logEvent(*g_runtime.context,
           static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "solar hook settings updated");
  sendOkJson("solar hook settings updated");
}

void handleSolarHook() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  if (!g_server.hasArg("state")) {
    sendErrorJson(400, "missing solar hook state");
    return;
  }

  const String state = g_server.arg("state");
  const bool disable_requested =
      state == "disable" || state == "off" || state == "stop";
  const bool open_requested = state == "open";
  const bool closed_requested =
      state == "close" || state == "closed";
  if (!disable_requested && !open_requested && !closed_requested) {
    sendErrorJson(400, "solar hook state must be open, close, or disable");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  if (!g_runtime.rear_link->configured()) {
    setFault(context, robot::FaultCode::CommunicationStale,
             "solar hook UART to ESP1 is not configured");
    sendErrorJson(409, "solar hook UART to ESP1 is not configured");
    return;
  }

  robot::SolarHookCommand command{};
  if (!disable_requested) {
    if (!g_runtime.rear_link->remoteStatusFresh(
            now_ms, remoteStatusTimeoutMs(context.config))) {
      setFault(context, robot::FaultCode::CommunicationStale,
               "solar hook ESP1 status is stale");
      sendErrorJson(409, "solar hook ESP1 status is stale");
      return;
    }
    if (!g_runtime.rear_link->latestStatus().solar_hook_configured) {
      setFault(context, robot::FaultCode::HardwareNotConfigured,
               "solar hook servo PWM hardware is not configured on ESP1");
      sendErrorJson(
          409, "solar hook servo PWM hardware is not configured on ESP1");
      return;
    }
    const int target_angle_deg =
        open_requested
            ? context.solar_hook_servo_settings.open_angle_deg
            : context.solar_hook_servo_settings.closed_angle_deg;
    if (target_angle_deg < 0 || target_angle_deg > 180) {
      setFault(context, robot::FaultCode::InvalidCommand,
               open_requested ? "solar hook open angle is not set"
                              : "solar hook closed angle is not set");
      sendErrorJson(409, open_requested
                             ? "solar hook open angle is not set"
                             : "solar hook closed angle is not set");
      return;
    }
    enterMechanismTestIfNeeded(context, now_ms);
    command.enabled = true;
    command.angle_deg = static_cast<std::uint8_t>(target_angle_deg);
  }

  if (!g_runtime.rear_link->send(command)) {
    setFault(context, robot::FaultCode::CommunicationStale,
             "solar hook command failed to send");
    sendErrorJson(503, "solar hook command failed to send");
    return;
  }

  context.solar_hook_commanded_open =
      command.enabled && open_requested;
  context.last_command_ms = now_ms;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web,
           command.enabled ? "solar hook command accepted"
                           : "solar hook output disabled");
  sendOkJson(command.enabled ? "solar hook command accepted"
                             : "solar hook output disabled");
}

void handleSolarHookSave() {
  if (!runtimeReady() || g_runtime.preferences == nullptr) {
    sendErrorJson(503, "preferences unavailable");
    return;
  }
  const SolarHookServoSettings& settings =
      g_runtime.context->solar_hook_servo_settings;
  g_runtime.preferences->putInt("shopen", settings.open_angle_deg);
  g_runtime.preferences->putInt("shclosed", settings.closed_angle_deg);
  sendOkJson("solar hook settings saved");
}

void handleClawsConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  ClawServoSettings next = g_runtime.claws->settings();
  if (!parseClawSettings(next)) {
    return;
  }

  const ClawServoCommandResult result = g_runtime.claws->applySettings(next);
  if (result != ClawServoCommandResult::Accepted) {
    const char* reason = clawServoResultReason(result);
    setFault(context, clawFaultCode(result), reason);
    logEvent(context, static_cast<robot::Milliseconds>(millis()),
             robot::EventSeverity::Warn, robot::EventSource::Web, reason);
    sendErrorJson(409, reason);
    return;
  }

  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "servo settings updated");
  sendOkJson("servo settings updated");
}

void handleClaw() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  std::size_t claw_index = 0U;
  ClawServoPositionRequest request{};
  if (!parseClawId(claw_index) || !parseClawRequest(request)) {
    sendErrorJson(400, "malformed claw command");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  enterMechanismTestIfNeeded(context, now_ms);
  const ClawServoCommandResult result =
      g_runtime.claws->command(claw_index, request);
  if (result != ClawServoCommandResult::Accepted) {
    const char* reason = clawServoResultReason(result);
    setFault(context, clawFaultCode(result), reason);
    logEvent(context, now_ms, robot::EventSeverity::Warn,
             robot::EventSource::Web, reason);
    sendErrorJson(409, reason);
    return;
  }

  context.last_command_ms = now_ms;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "claw command accepted");
  sendOkJson("claw command accepted");
}

void handleClawsAll() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  ClawServoPositionRequest request{};
  if (!parseClawRequest(request)) {
    sendErrorJson(400, "malformed claw command");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  enterMechanismTestIfNeeded(context, now_ms);
  const ClawServoCommandResult result = g_runtime.claws->commandAll(request);
  if (result != ClawServoCommandResult::Accepted) {
    const char* reason = clawServoResultReason(result);
    setFault(context, clawFaultCode(result), reason);
    logEvent(context, now_ms, robot::EventSeverity::Warn,
             robot::EventSource::Web, reason);
    sendErrorJson(409, reason);
    return;
  }

  context.last_command_ms = now_ms;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "all claw command accepted");
  sendOkJson("all claw command accepted");
}

void handleWinch() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  ClawServoPositionRequest request{};
  if (!parseClawRequest(request)) {
    sendErrorJson(400, "malformed winch command");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  enterMechanismTestIfNeeded(context, now_ms);
  const ClawServoCommandResult result =
      g_runtime.claws->command(kWinchServoIndex, request);
  if (result != ClawServoCommandResult::Accepted) {
    const char* reason = clawServoResultReason(result);
    setFault(context, clawFaultCode(result), reason);
    logEvent(context, now_ms, robot::EventSeverity::Warn,
             robot::EventSource::Web, reason);
    sendErrorJson(409, reason);
    return;
  }

  context.last_command_ms = now_ms;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "winch command accepted");
  sendOkJson("winch command accepted");
}

void handleHabitatPusher() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  ClawServoPositionRequest request{};
  if (!parseClawRequest(request)) {
    sendErrorJson(400, "malformed Habitat Pusher command");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  enterMechanismTestIfNeeded(context, now_ms);
  const ClawServoCommandResult result = g_runtime.claws->command(
      kHabitatPusherServoIndex, request);
  if (result != ClawServoCommandResult::Accepted) {
    const char* reason = clawServoResultReason(result);
    setFault(context, clawFaultCode(result), reason);
    logEvent(context, now_ms, robot::EventSeverity::Warn,
             robot::EventSource::Web, reason);
    sendErrorJson(409, reason);
    return;
  }

  context.last_command_ms = now_ms;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web,
           request == ClawServoPositionRequest::Open
               ? "Habitat Pusher opened"
               : "Habitat Pusher closed");
  sendOkJson("Habitat Pusher command accepted");
}

void handleFunnel() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }

  float speed = 0.0F;
  if (!argFloat("speed", speed, 0.0F, true)) {
    sendErrorJson(400, "malformed funnel command");
    return;
  }

  RuntimeContext& context = *g_runtime.context;
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  enterMechanismTestIfNeeded(context, now_ms);

  robot::CommandValidationResult validation =
      robot::validateModeAllowsMechanism(context.modes.currentMode());
  if (validation.accepted) {
    validation = robot::validateNormalizedDuty(
        speed, clampFloat(kSingleMotorDutyCap, 0.0F, hardwareDutyCap()));
  }
  if (validation.accepted) {
    validation = robot::validateTimedDuration(
        kCommandTimeoutMs, validationLimits(context).maximum_duration_ms);
  }
  if (!validation.accepted) {
    setFault(context, robot::FaultCode::InvalidCommand, validation.reason);
    logEvent(context, now_ms, robot::EventSeverity::Warn,
             robot::EventSource::Web, validation.reason);
    sendErrorJson(409, validation.reason);
    return;
  }

  const bool moving = std::fabs(speed) > 0.0001F;
  if (!g_runtime.rear_link->configured()) {
    setFault(context, robot::FaultCode::CommunicationStale,
             "funnel UART to ESP1 is not configured");
    sendErrorJson(409, "funnel UART to ESP1 is not configured");
    return;
  }
  if (moving && g_runtime.rear_link->remoteStatusFresh(
                    now_ms, remoteStatusTimeoutMs(context.config)) &&
      !g_runtime.rear_link->latestStatus().funnel_configured) {
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "funnel motor PWM hardware is not configured on ESP1");
    sendErrorJson(409, "funnel motor PWM hardware is not configured on ESP1");
    return;
  }

  context.requested_funnel_command =
      moving ? makeTimedMotorCommand(speed, now_ms, kCommandTimeoutMs)
             : robot::disabledMotorCommand();
  if (!sendFunnelMotorCommand(*g_runtime.rear_link,
                              context.requested_funnel_command,
                              context.config, now_ms)) {
    setFault(context, robot::FaultCode::CommunicationStale,
             "funnel command failed to send");
    sendErrorJson(503, "funnel command failed to send");
    return;
  }

  context.last_command_ms = now_ms;
  clearFault(context);
  if (moving) {
    logEvent(context, now_ms, robot::EventSeverity::Info,
             robot::EventSource::Web, "funnel hold command");
  }
  sendOkJson("funnel command accepted");
}

void handleClawsSave() {
  if (!runtimeReady() || g_runtime.preferences == nullptr) {
    sendErrorJson(503, "preferences unavailable");
    return;
  }
  const ClawServoSettings& settings = g_runtime.claws->settings();
  g_runtime.preferences->putInt("c1open", settings.open_angle_deg[0]);
  g_runtime.preferences->putInt("c2open", settings.open_angle_deg[1]);
  g_runtime.preferences->putInt("c3open", settings.open_angle_deg[2]);
  g_runtime.preferences->putInt("c1closed", settings.closed_angle_deg[0]);
  g_runtime.preferences->putInt("c2closed", settings.closed_angle_deg[1]);
  g_runtime.preferences->putInt("c3closed", settings.closed_angle_deg[2]);
  g_runtime.preferences->putInt(
      "wopen", settings.open_angle_deg[kWinchServoIndex]);
  g_runtime.preferences->putInt(
      "wclosed", settings.closed_angle_deg[kWinchServoIndex]);
  g_runtime.preferences->putInt(
      "pushopen", settings.open_angle_deg[kHabitatPusherServoIndex]);
  g_runtime.preferences->putInt(
      "pushclose", settings.closed_angle_deg[kHabitatPusherServoIndex]);
  sendOkJson("servo settings saved");
}

void handleLineFollowStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  robot::Milliseconds duration_ms = kMaxTimedTestDurationMs;
  if (!argUnsigned("ms", duration_ms, kMaxTimedTestDurationMs, false)) {
    sendErrorJson(400, "malformed duration");
    return;
  }
  const robot::CommandValidationResult duration_validation =
      robot::validateTimedDuration(duration_ms, kMaxTimedTestDurationMs);
  if (!duration_validation.accepted) {
    sendErrorJson(409, duration_validation.reason);
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  if (context.modes.currentMode() != robot::RobotTestMode::LineFollowTest) {
    resetTowerPieces(context, now_ms);
    resetPegFinder(context, now_ms);
    resetTimeTrial(context, now_ms);
    disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                     *g_runtime.rear_link, now_ms);
    context.modes.setMode(robot::RobotTestMode::LineFollowTest, now_ms);
  }
  if (!startRequirementsMet(*g_runtime.sensors, *g_runtime.front_left,
                            *g_runtime.front_right, *g_runtime.rear_link,
                            now_ms,
                            context)) {
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "line follower hardware requirements are incomplete");
    logEvent(context, now_ms, robot::EventSeverity::Fault,
             robot::EventSource::Line,
             "line follower start rejected: hardware incomplete");
    sendErrorJson(409,
                  "configure sensors, motors, UART, ESP1 status, max-duty, hardware cap");
    return;
  }

  robot::startLineFollower(context.follower_state, now_ms);
  context.last_command_ms = now_ms;
  context.mode_expires_at_ms = now_ms + duration_ms;
  context.command_deadman_armed = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "line follower started");
  sendOkJson("line follower started");
}

void handleLineFollowStop() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  disableActuators(*g_runtime.context, *g_runtime.front_left,
                   *g_runtime.front_right, *g_runtime.rear_link, now_ms);
  logEvent(*g_runtime.context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "line follower stopped");
  sendOkJson("line follower stopped");
}

void handleRearLineFollowStart() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  robot::Milliseconds duration_ms = kMaxTimedTestDurationMs;
  if (!argUnsigned("ms", duration_ms, kMaxTimedTestDurationMs, false)) {
    sendErrorJson(400, "malformed duration");
    return;
  }
  const robot::CommandValidationResult duration_validation =
      robot::validateTimedDuration(duration_ms, kMaxTimedTestDurationMs);
  if (!duration_validation.accepted) {
    sendErrorJson(409, duration_validation.reason);
    return;
  }

  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  if (context.modes.currentMode() !=
      robot::RobotTestMode::RearLineFollowTest) {
    resetTowerPieces(context, now_ms);
    resetPegFinder(context, now_ms);
    resetTimeTrial(context, now_ms);
    disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                     *g_runtime.rear_link, now_ms);
    context.modes.setMode(robot::RobotTestMode::RearLineFollowTest, now_ms);
  }
  if (!rearLineStartRequirementsMet(
          *g_runtime.front_left, *g_runtime.front_right,
          *g_runtime.rear_link, now_ms, context)) {
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "rear line follower hardware or sensor stream is incomplete");
    logEvent(context, now_ms, robot::EventSeverity::Fault,
             robot::EventSource::Line,
             "rear line follower start rejected: hardware or data incomplete");
    sendErrorJson(
        409,
        "configure rear sensors, motors, UART, fresh ESP1 data, max-duty, hardware cap");
    return;
  }

  robot::startLineFollower(context.follower_state, now_ms);
  context.last_command_ms = now_ms;
  context.mode_expires_at_ms = now_ms + duration_ms;
  context.command_deadman_armed = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "rear line follower started");
  sendOkJson("rear line follower started");
}

void handleRearLineFollowStop() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  disableActuators(*g_runtime.context, *g_runtime.front_left,
                   *g_runtime.front_right, *g_runtime.rear_link, now_ms);
  logEvent(*g_runtime.context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Web, "rear line follower stopped");
  sendOkJson("rear line follower stopped");
}

void handleLineFollowConfigImpl(const bool rear) {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  robot::LineFollowerConfig next =
      rear ? context.rear_config : context.config;
  float value = 0.0F;
  robot::Milliseconds milliseconds_value = 0U;
  int polarity_value = next.steeringPolarity;
  bool bool_value = false;
  if (g_server.hasArg("kp")) {
    if (!argFloat("kp", value, next.kp, true)) {
      sendErrorJson(400, "malformed kp");
      return;
    }
    next.kp = value;
  }
  if (g_server.hasArg("ki")) {
    if (!argFloat("ki", value, next.ki, true)) {
      sendErrorJson(400, "malformed ki");
      return;
    }
    next.ki = value;
  }
  if (g_server.hasArg("kd")) {
    if (!argFloat("kd", value, next.kd, true)) {
      sendErrorJson(400, "malformed kd");
      return;
    }
    next.kd = value;
  }
  if (g_server.hasArg("base")) {
    if (!argFloat("base", value, next.baseDuty, true)) {
      sendErrorJson(400, "malformed base");
      return;
    }
    next.baseDuty = value;
  } else if (g_server.hasArg("baseDuty")) {
    if (!argFloat("baseDuty", value, next.baseDuty, true)) {
      sendErrorJson(400, "malformed baseDuty");
      return;
    }
    next.baseDuty = value;
  }
  if (g_server.hasArg("max")) {
    if (!argFloat("max", value, next.maxDuty, true)) {
      sendErrorJson(400, "malformed max");
      return;
    }
    next.maxDuty = value;
  } else if (g_server.hasArg("max-duty")) {
    if (!argFloat("max-duty", value, next.maxDuty, true)) {
      sendErrorJson(400, "malformed max-duty");
      return;
    }
    next.maxDuty = value;
  } else if (g_server.hasArg("maxDuty")) {
    if (!argFloat("maxDuty", value, next.maxDuty, true)) {
      sendErrorJson(400, "malformed maxDuty");
      return;
    }
    next.maxDuty = value;
  }
  if (g_server.hasArg("max-correction")) {
    if (!argFloat("max-correction", value, next.maxCorrection, true)) {
      sendErrorJson(400, "malformed max-correction");
      return;
    }
    next.maxCorrection = value;
  } else if (g_server.hasArg("maxCorrection")) {
    if (!argFloat("maxCorrection", value, next.maxCorrection, true)) {
      sendErrorJson(400, "malformed maxCorrection");
      return;
    }
    next.maxCorrection = value;
  }
  if (g_server.hasArg("integral-limit")) {
    if (!argFloat("integral-limit", value, next.integralLimit, true)) {
      sendErrorJson(400, "malformed integral-limit");
      return;
    }
    next.integralLimit = value;
  } else if (g_server.hasArg("integralLimit")) {
    if (!argFloat("integralLimit", value, next.integralLimit, true)) {
      sendErrorJson(400, "malformed integralLimit");
      return;
    }
    next.integralLimit = value;
  }
  if (g_server.hasArg("derivative-limit")) {
    if (!argFloat("derivative-limit", value, next.derivativeLimit, true)) {
      sendErrorJson(400, "malformed derivative-limit");
      return;
    }
    next.derivativeLimit = value;
  } else if (g_server.hasArg("derivativeLimit")) {
    if (!argFloat("derivativeLimit", value, next.derivativeLimit, true)) {
      sendErrorJson(400, "malformed derivativeLimit");
      return;
    }
    next.derivativeLimit = value;
  }
  if (g_server.hasArg("derivative-alpha")) {
    if (!argFloat("derivative-alpha", value, next.derivativeFilterAlpha,
                  true)) {
      sendErrorJson(400, "malformed derivative-alpha");
      return;
    }
    next.derivativeFilterAlpha = value;
  } else if (g_server.hasArg("derivativeFilterAlpha")) {
    if (!argFloat("derivativeFilterAlpha", value,
                  next.derivativeFilterAlpha, true)) {
      sendErrorJson(400, "malformed derivativeFilterAlpha");
      return;
    }
    next.derivativeFilterAlpha = value;
  }
  if (g_server.hasArg("period-ms")) {
    if (!argUnsigned("period-ms", milliseconds_value, next.controlPeriodMs,
                     true)) {
      sendErrorJson(400, "malformed period-ms");
      return;
    }
    next.controlPeriodMs = milliseconds_value;
  } else if (g_server.hasArg("controlPeriodMs")) {
    if (!argUnsigned("controlPeriodMs", milliseconds_value,
                     next.controlPeriodMs, true)) {
      sendErrorJson(400, "malformed controlPeriodMs");
      return;
    }
    next.controlPeriodMs = milliseconds_value;
  }
  if (g_server.hasArg("timeout-ms")) {
    if (!argUnsigned("timeout-ms", milliseconds_value,
                     next.remoteCommandTimeoutMs, true)) {
      sendErrorJson(400, "malformed timeout-ms");
      return;
    }
    next.remoteCommandTimeoutMs = milliseconds_value;
  } else if (g_server.hasArg("remoteCommandTimeoutMs")) {
    if (!argUnsigned("remoteCommandTimeoutMs", milliseconds_value,
                     next.remoteCommandTimeoutMs, true)) {
      sendErrorJson(400, "malformed remoteCommandTimeoutMs");
      return;
    }
    next.remoteCommandTimeoutMs = milliseconds_value;
  }
  if (g_server.hasArg("telemetry")) {
    if (!argOnOff("telemetry", bool_value, next.telemetryEnabled, true)) {
      sendErrorJson(400, "malformed telemetry");
      return;
    }
    next.telemetryEnabled = bool_value;
  } else if (g_server.hasArg("telemetryEnabled")) {
    if (!argOnOff("telemetryEnabled", bool_value, next.telemetryEnabled,
                  true)) {
      sendErrorJson(400, "malformed telemetryEnabled");
      return;
    }
    next.telemetryEnabled = bool_value;
  }
  if (g_server.hasArg("polarity")) {
    if (!argPolarity("polarity", polarity_value, next.steeringPolarity,
                     true)) {
      sendErrorJson(400, "malformed polarity");
      return;
    }
    next.steeringPolarity = polarity_value;
  } else if (g_server.hasArg("steeringPolarity")) {
    if (!argPolarity("steeringPolarity", polarity_value,
                     next.steeringPolarity, true)) {
      sendErrorJson(400, "malformed steeringPolarity");
      return;
    }
    next.steeringPolarity = polarity_value;
  }

  if (rear) {
    next.baseDuty = std::fabs(next.baseDuty);
  }
  const robot::CommandValidationResult validation =
      robot::validateLineFollowerConfig(next, hardwareDutyCap());
  if (!validation.accepted) {
    setFault(context, robot::FaultCode::InvalidCommand, validation.reason);
    logEvent(context, static_cast<robot::Milliseconds>(millis()),
             robot::EventSeverity::Warn, robot::EventSource::Web,
             validation.reason);
    sendErrorJson(409, validation.reason);
    return;
  }

  if (rear) {
    context.rear_config = next;
  } else {
    context.config = next;
  }
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           rear ? "rear line follower config updated"
                : "line follower config updated");
  sendOkJson(rear ? "rear line follower config updated"
                  : "line follower config updated");
}

void handleLineFollowConfig() {
  handleLineFollowConfigImpl(false);
}

void handleRearLineFollowConfig() {
  handleLineFollowConfigImpl(true);
}

void handleConfig() {
  const robot::TelemetrySnapshot snapshot = currentSnapshot();
  std::snprintf(g_json_buffer, sizeof(g_json_buffer),
                "{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f,"
                "\"baseDuty\":%.5f,\"maxDuty\":%.5f,"
                "\"maxCorrection\":%.5f,\"integralLimit\":%.5f,"
                "\"derivativeLimit\":%.5f,\"derivativeFilterAlpha\":%.5f,"
                "\"steeringPolarity\":%d,\"controlPeriodMs\":%u,"
                "\"remoteCommandTimeoutMs\":%u,\"telemetryEnabled\":%s,"
                "\"hardwareDutyCap\":%.5f,\"singleMotorDutyCap\":%.5f}",
                snapshot.kp, snapshot.ki, snapshot.kd, snapshot.base_duty,
                snapshot.maximum_duty, snapshot.maximum_correction,
                snapshot.integral_limit, snapshot.derivative_limit,
                snapshot.derivative_filter_alpha, snapshot.steering_polarity,
                static_cast<unsigned>(snapshot.control_period_ms),
                static_cast<unsigned>(snapshot.remote_command_timeout_ms),
                snapshot.line_telemetry_enabled ? "true" : "false",
                hardwareDutyCap(),
                kSingleMotorDutyCap);
  g_server.send(200, "application/json", g_json_buffer);
}

void handleConfigSave() {
  if (!runtimeReady() || g_runtime.preferences == nullptr) {
    sendErrorJson(503, "preferences unavailable");
    return;
  }
  const RuntimeContext& context = *g_runtime.context;
  g_runtime.preferences->putInt(
      "shopen", context.solar_hook_servo_settings.open_angle_deg);
  g_runtime.preferences->putInt(
      "shclosed", context.solar_hook_servo_settings.closed_angle_deg);
  g_runtime.preferences->putFloat("kp", context.config.kp);
  g_runtime.preferences->putFloat("ki", context.config.ki);
  g_runtime.preferences->putFloat("kd", context.config.kd);
  g_runtime.preferences->putFloat("base", context.config.baseDuty);
  g_runtime.preferences->putFloat("max", context.config.maxDuty);
  g_runtime.preferences->putFloat("corr", context.config.maxCorrection);
  g_runtime.preferences->putFloat("ilim", context.config.integralLimit);
  g_runtime.preferences->putFloat("dlim", context.config.derivativeLimit);
  g_runtime.preferences->putFloat("dalpha",
                                  context.config.derivativeFilterAlpha);
  g_runtime.preferences->putUInt("period", context.config.controlPeriodMs);
  g_runtime.preferences->putUInt("rto",
                                 context.config.remoteCommandTimeoutMs);
  g_runtime.preferences->putBool("lftele", context.config.telemetryEnabled);
  g_runtime.preferences->putInt("pol", context.config.steeringPolarity);
  g_runtime.preferences->putFloat(
      "hpduty", context.habitat_pieces_config.line_follow_duty);
  g_runtime.preferences->putUInt(
      "hpdetect",
      context.habitat_pieces_config.lss2_detection_delay_ms);
  g_runtime.preferences->putUInt(
      "hptimeout", context.habitat_pieces_config.run_timeout_ms);
  g_runtime.preferences->putFloat(
      "hprevduty", context.habitat_pieces_config.reverse_duty);
  g_runtime.preferences->putUInt(
      "hprevms", context.habitat_pieces_config.reverse_duration_ms);
  g_runtime.preferences->putInt(
      "hpdsdir", static_cast<int>(
                     context.habitat_pieces_config
                         .distance_strafe_direction));
  g_runtime.preferences->putUInt(
      "hpdsthr", context.habitat_pieces_config.distance_threshold_mm);
  g_runtime.preferences->putUInt(
      "hpdscnt",
      context.habitat_pieces_config.distance_zone_target_count);
  g_runtime.preferences->putFloat(
      "hpdsduty", context.habitat_pieces_config.distance_strafe_duty);
  g_runtime.preferences->putUInt(
      "hpdstmo",
      context.habitat_pieces_config.distance_strafe_timeout_ms);
  g_runtime.preferences->putFloat(
      "hpcmpd",
      context.habitat_pieces_config.compensation_strafe_duty);
  g_runtime.preferences->putUInt(
      "hpcmpms",
      context.habitat_pieces_config.compensation_strafe_duration_ms);
  g_runtime.preferences->putUInt(
      "hpkdnspd",
      context.habitat_pieces_config.slide_down_speed_steps_per_second);
  g_runtime.preferences->putUInt(
      "hpkdntmo", context.habitat_pieces_config.slide_down_timeout_ms);
  g_runtime.preferences->putFloat(
      "hpkfwdd", context.habitat_pieces_config.forward_to_distance_duty);
  g_runtime.preferences->putUInt(
      "hpkfwdmm", context.habitat_pieces_config.forward_stop_distance_mm);
  g_runtime.preferences->putUInt(
      "hpkfwdtmo",
      context.habitat_pieces_config.forward_to_distance_timeout_ms);
  g_runtime.preferences->putUInt(
      "hpklftstp", context.habitat_pieces_config.slide_lift_steps);
  g_runtime.preferences->putUInt(
      "hpklftspd",
      context.habitat_pieces_config.slide_lift_speed_steps_per_second);
  g_runtime.preferences->putUInt(
      "hpklfttmo", context.habitat_pieces_config.slide_lift_timeout_ms);
  g_runtime.preferences->putFloat(
      "hpkrevd", context.habitat_pieces_config.post_pickup_reverse_duty);
  g_runtime.preferences->putUInt(
      "hpkrevms",
      context.habitat_pieces_config.post_pickup_reverse_duration_ms);
  g_runtime.preferences->putFloat(
      "hpkretd", context.habitat_pieces_config.return_strafe_duty);
  g_runtime.preferences->putUInt(
      "hpkrettmo", context.habitat_pieces_config.return_line_timeout_ms);
  const robot::HabitatPlacementConfig& placement =
      context.habitat_placement_config;
  g_runtime.preferences->putFloat("hprlduty", placement.reverse_line_follow_duty);
  g_runtime.preferences->putUInt("hplsstmo", placement.lss1_timeout_ms);
  g_runtime.preferences->putUInt("hplssdel", placement.post_lss1_delay_ms);
  g_runtime.preferences->putUInt(
      "hpihdtmo", placement.initial_heading_turn_timeout_ms);
  g_runtime.preferences->putFloat(
      "hpprestrd",
      placement.pre_counter_clockwise_strafe_right_duty);
  g_runtime.preferences->putUInt(
      "hpprestrms",
      placement.pre_counter_clockwise_strafe_right_duration_ms);
  g_runtime.preferences->putFloat("hpccwdeg", placement.counter_clockwise_angle_deg);
  g_runtime.preferences->putUInt("hpccwtmo", placement.counter_clockwise_timeout_ms);
  g_runtime.preferences->putFloat("hpfwd1d", placement.forward_to_slide_duty);
  g_runtime.preferences->putUInt("hpfwd1ms", placement.forward_to_slide_duration_ms);
  g_runtime.preferences->putUInt("hpstpspd", placement.stepper_down_speed_steps_per_second);
  g_runtime.preferences->putUInt("hpstptmo", placement.stepper_down_timeout_ms);
  g_runtime.preferences->putUInt("hpopenset", placement.pusher_open_settle_ms);
  g_runtime.preferences->putFloat("hppushd", placement.push_forward_duty);
  g_runtime.preferences->putUInt("hppushms", placement.push_forward_duration_ms);
  g_runtime.preferences->putFloat("hpretd", placement.reverse_retreat_duty);
  g_runtime.preferences->putUInt("hpretms", placement.reverse_retreat_duration_ms);
  g_runtime.preferences->putFloat("hpcwdeg", placement.clockwise_angle_deg);
  g_runtime.preferences->putUInt("hpcwtmo", placement.clockwise_timeout_ms);
  g_runtime.preferences->putFloat(
      "hpcwrevd", placement.post_clockwise_reverse_duty);
  g_runtime.preferences->putUInt(
      "hpcwrevms", placement.post_clockwise_reverse_duration_ms);
  g_runtime.preferences->putFloat(
      "hpcwstrld", placement.post_clockwise_strafe_left_duty);
  g_runtime.preferences->putUInt(
      "hpcwstrlms",
      placement.post_clockwise_strafe_left_duration_ms);
  g_runtime.preferences->putUInt("hpcwdel", placement.post_clockwise_delay_ms);
  g_runtime.preferences->putFloat("hpexitd", placement.exit_forward_duty);
  g_runtime.preferences->putUInt("hpexitms", placement.exit_forward_duration_ms);
  g_runtime.preferences->putUInt("hpexitdel", placement.post_forward_delay_ms);
  g_runtime.preferences->putFloat("hpstrd", placement.strafe_right_duty);
  g_runtime.preferences->putUInt("hpstrtmo", placement.strafe_right_timeout_ms);
  g_runtime.preferences->putFloat("rkp", context.rear_config.kp);
  g_runtime.preferences->putFloat("rki", context.rear_config.ki);
  g_runtime.preferences->putFloat("rkd", context.rear_config.kd);
  g_runtime.preferences->putFloat(
      "rbase", std::fabs(context.rear_config.baseDuty));
  g_runtime.preferences->putFloat("rmax", context.rear_config.maxDuty);
  g_runtime.preferences->putFloat("rcorr",
                                  context.rear_config.maxCorrection);
  g_runtime.preferences->putFloat("rilim",
                                  context.rear_config.integralLimit);
  g_runtime.preferences->putFloat("rdlim",
                                  context.rear_config.derivativeLimit);
  g_runtime.preferences->putFloat(
      "rdalpha", context.rear_config.derivativeFilterAlpha);
  g_runtime.preferences->putUInt("rperiod",
                                 context.rear_config.controlPeriodMs);
  g_runtime.preferences->putUInt(
      "rrto", context.rear_config.remoteCommandTimeoutMs);
  g_runtime.preferences->putBool(
      "rlftele", context.rear_config.telemetryEnabled);
  g_runtime.preferences->putInt("rpol",
                                context.rear_config.steeringPolarity);
  g_runtime.preferences->putFloat(
      "tpduty", context.tower_pieces_config.reverse_line_duty);
  g_runtime.preferences->putUInt(
      "tptmo", context.tower_pieces_config.side_line_timeout_ms);
  g_runtime.preferences->putUInt(
      "tpcool", context.tower_pieces_config.side_line_cooldown_ms);
  g_runtime.preferences->putUInt(
      "tprearm", context.tower_pieces_config.side_line_rearm_ms);
  g_runtime.preferences->putUInt(
      "tpdelay", context.tower_pieces_config.post_line_delay_ms);
  g_runtime.preferences->putUInt(
      "tpsdur", context.tower_pieces_config.strafe_right_duration_ms);
  g_runtime.preferences->putFloat(
      "tpsduty", context.tower_pieces_config.strafe_right_duty);
  g_runtime.preferences->putUInt(
      "tppause", context.tower_pieces_config.post_strafe_pause_ms);
  g_runtime.preferences->putFloat(
      "tprdeg", context.tower_pieces_config.clockwise_rotation_angle_deg);
  g_runtime.preferences->putUInt(
      "tprpause", context.tower_pieces_config.post_rotation_pause_ms);
  g_runtime.preferences->putFloat(
      "tpbkduty", context.tower_pieces_config.reverse_duty);
  g_runtime.preferences->putUInt(
      "tpbkdur", context.tower_pieces_config.reverse_duration_ms);
  g_runtime.preferences->putUInt(
      "tpshrdur", context.tower_pieces_config.shimmy_right_duration_ms);
  g_runtime.preferences->putUInt(
      "tpshldur", context.tower_pieces_config.shimmy_left_duration_ms);
  g_runtime.preferences->putUInt(
      "tpshtmo", context.tower_pieces_config.shimmy_timeout_ms);
  g_runtime.preferences->putFloat(
      "tpshduty", context.tower_pieces_config.shimmy_duty);
  g_runtime.preferences->putFloat(
      "tp_end_d", context.tower_pieces_config.final_reverse_duty);
  g_runtime.preferences->putUInt(
      "tp_end_ms", context.tower_pieces_config.final_reverse_duration_ms);
  g_runtime.preferences->putUInt(
      "tp_end_p", context.tower_pieces_config.post_final_reverse_delay_ms);
  g_runtime.preferences->putUInt(
      "tp_wo_p", context.tower_pieces_config.post_winch_open_delay_ms);
  g_runtime.preferences->putUInt(
      "tp_co_p", context.tower_pieces_config.post_claws_open_delay_ms);
  g_runtime.preferences->putUInt(
      "tp_pre_dn",
      context.tower_pieces_config.pre_stepper_bottom_delay_ms);
  g_runtime.preferences->putUInt(
      "tp_dn_spd",
      context.tower_pieces_config.stepper_down_speed_steps_per_second);
  g_runtime.preferences->putUInt(
      "tp_bot_p", context.tower_pieces_config.post_stepper_bottom_delay_ms);
  g_runtime.preferences->putUInt(
      "tp_cc_p", context.tower_pieces_config.post_claws_closed_delay_ms);
  g_runtime.preferences->putUInt(
      "tp_up_spd",
      context.tower_pieces_config.stepper_up_speed_steps_per_second);
  g_runtime.preferences->putFloat(
      "pf_cw_deg", context.peg_finder_config.clockwise_angle_deg);
  g_runtime.preferences->putUInt(
      "pf_cw_p", context.peg_finder_config.post_rotation_pause_ms);
  g_runtime.preferences->putFloat(
      "pf_rev_d", context.peg_finder_config.reverse_duty);
  g_runtime.preferences->putUInt(
      "pf_rev_ms", context.peg_finder_config.reverse_duration_ms);
  g_runtime.preferences->putUInt(
      "pf_rev_p", context.peg_finder_config.post_reverse_pause_ms);
  g_runtime.preferences->putFloat(
      "pf_fwd_d", context.peg_finder_config.forward_duty);
  g_runtime.preferences->putUInt(
      "pf_fwd_ms", context.peg_finder_config.forward_duration_ms);
  g_runtime.preferences->putFloat(
      "pf_fun_d", context.peg_finder_config.funnel_forward_duty);
  g_runtime.preferences->putUInt(
      "pf_fun_ms", context.peg_finder_config.funnel_forward_timeout_ms);
  g_runtime.preferences->putUInt(
      "pf_fun_p", context.peg_finder_config.post_funnel_limit_delay_ms);
  g_runtime.preferences->putUInt(
      "pf_claw_p", context.peg_finder_config.claw_open_interval_ms);
  g_runtime.preferences->putUChar(
      "pf_claw_1", context.peg_finder_config.claw_open_order_1);
  g_runtime.preferences->putUChar(
      "pf_claw_2", context.peg_finder_config.claw_open_order_2);
  g_runtime.preferences->putUChar(
      "pf_claw_3", context.peg_finder_config.claw_open_order_3);
  g_runtime.preferences->putUInt(
      "pf_claw_d", context.peg_finder_config.post_claws_open_delay_ms);
  g_runtime.preferences->putFloat(
      "pf_revfun_d", context.peg_finder_config.funnel_reverse_duty);
  g_runtime.preferences->putUInt(
      "pf_revfun_ms", context.peg_finder_config.funnel_reverse_duration_ms);
  g_runtime.preferences->putUInt(
      "tt_sdly", context.time_trial_config.post_solar_delay_ms);
  g_runtime.preferences->putUInt(
      "tt_sms",
      context.time_trial_config.solar_to_tower_strafe_right_duration_ms);
  g_runtime.preferences->putUInt(
      "tt_tdly", context.time_trial_config.post_tower_delay_ms);
  g_runtime.preferences->putUInt("sdet1", context.solar_thresholds.detect_1khz);
  g_runtime.preferences->putUInt("srel1",
                                 context.solar_thresholds.release_1khz);
  g_runtime.preferences->putUInt("sdet10",
                                 context.solar_thresholds.detect_10khz);
  g_runtime.preferences->putUInt("srel10",
                                 context.solar_thresholds.release_10khz);
  g_runtime.preferences->putUInt("scfm",
                                 context.solar_config.confirmation_time_ms);
  g_runtime.preferences->putFloat("salpha",
                                  context.solar_config.filter_alpha);
  g_runtime.preferences->putUInt("signore",
                                 context.solar_config.ignore_after_start_ms);
  g_runtime.preferences->putUInt("stimeout",
                                 context.solar_config.search_timeout_ms);
  g_runtime.preferences->putFloat("sstart",
                                  context.solar_speed_config.start_base_duty);
  g_runtime.preferences->putUInt("sslowms",
                                 context.solar_speed_config.slow_after_ms);
  g_runtime.preferences->putFloat("sslow",
                                  context.solar_speed_config.slow_base_duty);
  g_runtime.preferences->putUInt("sctmo",
                                 context.solar_contact_config.timeout_ms);
  g_runtime.preferences->putFloat(
      "si_str_d",
      context.solar_contact_config.initial_strafe_right_duty);
  g_runtime.preferences->putFloat(
      "sl_str_d",
      context.solar_contact_config.retry_strafe_left_duty);
  g_runtime.preferences->putFloat(
      "sr_str_d",
      context.solar_contact_config.retry_strafe_right_duty);
  g_runtime.preferences->putUInt(
      "sdelay",
      context.solar_contact_config.strafe_start_delay_ms);
  g_runtime.preferences->putFloat(
      "sstrfd", context.solar_contact_config.retry_forward_duty);
  g_runtime.preferences->putUInt(
      "srleft",
      context.solar_contact_config.retry_strafe_left_duration_ms);
  g_runtime.preferences->putUInt(
      "srfwd", context.solar_contact_config.retry_forward_duration_ms);
  g_runtime.preferences->putUInt(
      "srtmo", context.solar_contact_config.retry_strafe_timeout_ms);
  g_runtime.preferences->putUInt(
      "spcfwd",
      context.solar_contact_config.post_contact_forward_duration_ms);
  g_runtime.preferences->putFloat(
      "spcfduty", context.solar_contact_config.post_contact_forward_duty);
  g_runtime.preferences->putUInt(
      "sfdly",
      context.solar_contact_config.post_contact_forward_start_delay_ms);
  g_runtime.preferences->putUInt(
      "slfdly",
      context.solar_contact_config.line_reacquire_strafe_start_delay_ms);
  g_runtime.preferences->putFloat(
      "slstrd", context.solar_contact_config.line_reacquire_strafe_duty);
  g_runtime.preferences->putUInt(
      "slpidms", context.solar_contact_config.rear_line_follow_duration_ms);
  sendOkJson("config saved");
}

void handleEvents() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  if (!robot::writeEventLogJson(g_runtime.context->events, g_json_buffer,
                                sizeof(g_json_buffer))) {
    sendErrorJson(500, "event json overflow");
    return;
  }
  g_server.send(200, "application/json", g_json_buffer);
}

void handleStepperStatus() {
  if (!runtimeReady() || g_runtime.stepper == nullptr) { sendErrorJson(503, "runtime not ready"); return; }
  const auto& axis = *g_runtime.stepper;
  std::snprintf(g_json_buffer, sizeof(g_json_buffer),
      "{\"positionSteps\":%lld,\"motionState\":\"%s\",\"speedStepsPerSecond\":%u,"
      "\"configuredSpeedStepsPerSecond\":%u,\"limitSearchSpeedStepsPerSecond\":%u,\"accelerationStepsPerSecond2\":%u,"
      "\"sleeping\":%s,\"lowerLimitActive\":%s,\"upperLimitActive\":%s,\"homed\":%s,\"busy\":%s,"
      "\"maximumPositionSteps\":%lld,\"microstepsPerRevolution\":1600}",
      static_cast<long long>(axis.positionSteps()), axis.motionStateName(),
      axis.speedStepsPerSecond(), axis.configuredSpeedStepsPerSecond(),
      axis.limitSearchSpeedStepsPerSecond(), axis.accelerationStepsPerSecond2(),
      axis.sleeping() ? "true" : "false",
      axis.lowerLimitActive() ? "true" : "false",
      axis.upperLimitActive() ? "true" : "false", axis.isHomed() ? "true" : "false",
      axis.isBusy() ? "true" : "false", static_cast<long long>(axis.maximumPositionSteps()));
  g_server.send(200, "application/json", g_json_buffer);
}

void handleStepperCommand() {
  if (!runtimeReady() || g_runtime.stepper == nullptr || !g_server.hasArg("command")) { sendErrorJson(400, "missing stepper command"); return; }
  auto& axis = *g_runtime.stepper;
  const String command = g_server.arg("command");
  if ((g_runtime.context->modes.currentMode() ==
           robot::RobotTestMode::AutonomousTowerPieces ||
       g_runtime.context->modes.currentMode() ==
           robot::RobotTestMode::HabitatPlacement ||
       g_runtime.context->modes.currentMode() ==
           robot::RobotTestMode::TimeTrial) &&
      command != "stop") {
    sendErrorJson(409,
                  "manual stepper commands are locked in autonomous modes");
    return;
  }
  bool accepted = true;
  if (command == "stop") axis.stop();
  else if (command == "up" || command == "down") accepted = axis.moveContinuous(command == "up" ? robot::esp2::StepperDirection::Up : robot::esp2::StepperDirection::Down);
  else if (command == "hold") axis.refreshHoldCommand();
  else if (command == "bottom") accepted = axis.moveToLowerLimit();
  else if (command == "top") accepted = axis.moveToUpperLimit();
  else if (command == "config") {
    if (!g_server.hasArg("speed") || !g_server.hasArg("limitSpeed")) {
      accepted = false;
    } else {
      char* speed_end = nullptr;
      char* limit_speed_end = nullptr;
      const String speed_text = g_server.arg("speed");
      const String limit_speed_text = g_server.arg("limitSpeed");
      const unsigned long speed = strtoul(speed_text.c_str(), &speed_end, 10);
      const unsigned long limit_speed =
          strtoul(limit_speed_text.c_str(), &limit_speed_end, 10);
      accepted = speed_end != speed_text.c_str() && *speed_end == '\0' &&
                 limit_speed_end != limit_speed_text.c_str() &&
                 *limit_speed_end == '\0' && speed <= UINT32_MAX &&
                 limit_speed <= UINT32_MAX && axis.setMotionSpeeds(
                     static_cast<std::uint32_t>(speed),
                     static_cast<std::uint32_t>(limit_speed));
    }
  }
  else accepted = false;
  if (!accepted) { sendErrorJson(409, "stepper command rejected by limits or state"); return; }
  sendOkJson("stepper command accepted");
}

void setupWebHandlers() {
  g_server.on("/", HTTP_GET, handleRoot);
  g_server.on("/api/status", HTTP_GET, handleStatus);
  g_server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  g_server.on("/api/imu/soak/reset-counters", HTTP_ANY,
              handleImuSoakCountersReset);
  g_server.on("/api/diagnostics", HTTP_GET, handleDiagnostics);
  g_server.on("/api/diagnostics/reset", HTTP_ANY,
              handleDiagnosticsReset);
  g_server.on("/api/diagnostics/freeze", HTTP_ANY,
              handleDiagnosticsFreeze);
  g_server.on("/api/stop", HTTP_ANY, handleStop);
  g_server.on("/api/mode", HTTP_ANY, handleMode);
  g_server.on("/api/drive", HTTP_ANY, handleDrive);
  g_server.on("/api/motor", HTTP_ANY, handleMotor);
  g_server.on("/api/invert", HTTP_ANY, handleInvert);
  g_server.on("/api/sensors", HTTP_GET, handleSensors);
  g_server.on("/api/line", HTTP_GET, handleLine);
  g_server.on("/api/rear-line", HTTP_GET, handleRearLine);
  g_server.on("/api/autonomous/habitat-pieces/start", HTTP_ANY,
              handleHabitatPiecesStart);
  g_server.on("/api/autonomous/habitat-pieces/stop", HTTP_ANY,
              handleHabitatPiecesStop);
  g_server.on("/api/autonomous/habitat-pieces/config", HTTP_ANY,
              handleHabitatPiecesConfig);
  g_server.on("/api/autonomous/habitat-placement/start", HTTP_ANY,
              handleHabitatPlacementStart);
  g_server.on("/api/autonomous/habitat-placement/stop", HTTP_ANY,
              handleHabitatPlacementStop);
  g_server.on("/api/autonomous/habitat-placement/config", HTTP_ANY,
              handleHabitatPlacementConfig);
  g_server.on("/api/autonomous/solar/start", HTTP_ANY,
              handleAutonomousSolarStart);
  g_server.on("/api/autonomous/solar/config", HTTP_ANY,
              handleAutonomousSolarConfig);
  g_server.on("/api/autonomous/tower-pieces/start", HTTP_ANY,
              handleTowerPiecesStart);
  g_server.on("/api/autonomous/tower-pieces/config", HTTP_ANY,
              handleTowerPiecesConfig);
  g_server.on("/api/autonomous/peg-finder/start", HTTP_ANY,
              handlePegFinderStart);
  g_server.on("/api/autonomous/peg-finder/config", HTTP_ANY,
              handlePegFinderConfig);
  g_server.on("/api/autonomous/time-trial/start", HTTP_ANY,
              handleTimeTrialStart);
  g_server.on("/api/autonomous/time-trial/config", HTTP_ANY,
              handleTimeTrialConfig);
  g_server.on("/api/imu-turn/config", HTTP_ANY, handleImuTurnConfig);
  g_server.on("/api/imu-turn/start", HTTP_ANY, handleImuTurnStart);
  g_server.on("/api/imu-turn/stop", HTTP_ANY, handleImuTurnStop);
  g_server.on("/api/imu-turn/reset-angle", HTTP_ANY,
              handleImuAngleReset);
  g_server.on("/api/imu-turn/save", HTTP_ANY, handleImuTurnSave);
  g_server.on("/api/imu-strafe/config", HTTP_ANY,
              handleImuHeadingHoldConfig);
  g_server.on("/api/imu-strafe/start", HTTP_ANY,
              handleImuHeadingHoldStart);
  g_server.on("/api/imu-strafe/heartbeat", HTTP_ANY,
              handleImuHeadingHoldHeartbeat);
  g_server.on("/api/imu-strafe/stop", HTTP_ANY,
              handleImuHeadingHoldStop);
  g_server.on("/api/imu-strafe/save", HTTP_ANY,
              handleImuHeadingHoldSave);
  g_server.on("/api/line-follow/start", HTTP_ANY, handleLineFollowStart);
  g_server.on("/api/line-follow/stop", HTTP_ANY, handleLineFollowStop);
  g_server.on("/api/line-follow/config", HTTP_ANY, handleLineFollowConfig);
  g_server.on("/api/rear-line-follow/start", HTTP_ANY,
              handleRearLineFollowStart);
  g_server.on("/api/rear-line-follow/stop", HTTP_ANY,
              handleRearLineFollowStop);
  g_server.on("/api/rear-line-follow/config", HTTP_ANY,
              handleRearLineFollowConfig);
  g_server.on("/api/claw", HTTP_ANY, handleClaw);
  g_server.on("/api/claws", HTTP_ANY, handleClawsAll);
  g_server.on("/api/winch", HTTP_ANY, handleWinch);
  g_server.on("/api/habitat-pusher", HTTP_ANY,
              handleHabitatPusher);
  g_server.on("/api/claws/config", HTTP_ANY, handleClawsConfig);
  g_server.on("/api/claws/save", HTTP_ANY, handleClawsSave);
  g_server.on("/api/solar-hook", HTTP_ANY, handleSolarHook);
  g_server.on("/api/solar-hook/config", HTTP_ANY,
              handleSolarHookConfig);
  g_server.on("/api/solar-hook/save", HTTP_ANY,
              handleSolarHookSave);
  g_server.on("/api/funnel", HTTP_ANY, handleFunnel);
  g_server.on("/api/config", HTTP_GET, handleConfig);
  g_server.on("/api/config/save", HTTP_ANY, handleConfigSave);
  g_server.on("/api/events", HTTP_GET, handleEvents);
  g_server.on("/api/stepper", HTTP_GET, handleStepperStatus);
  g_server.on("/api/stepper/command", HTTP_ANY, handleStepperCommand);
  g_server.onNotFound([]() { sendErrorJson(404, "not found"); });
}

void printRejected(const char* reason) {
  Serial.print("rejected: ");
  Serial.println(reason);
}

void printOk(const char* message) {
  Serial.print("ok: ");
  Serial.println(message);
}

void printStatus(const RuntimeContext& context, const RearCommandLink& rear_link,
                 const DigitalFrontLineSensorReader& sensors,
                 const DualPwmMotorOutput& front_left,
                 const DualPwmMotorOutput& front_right,
                 const robot::Milliseconds now_ms) {
  Serial.print("status mode=");
  Serial.print(robot::robotTestModeName(context.modes.currentMode()));
  Serial.print(", fault=");
  Serial.print(context.fault_active ? robot::faultCodeName(context.fault_code)
                                    : "NONE");
  Serial.print(", lf-enabled=");
  Serial.print(context.follower_state.enabled ? 1 : 0);
  Serial.print(", kp=");
  Serial.print(context.config.kp, 4);
  Serial.print(", ki=");
  Serial.print(context.config.ki, 4);
  Serial.print(", kd=");
  Serial.print(context.config.kd, 4);
  Serial.print(", base=");
  Serial.print(context.config.baseDuty, 4);
  Serial.print(", max-duty=");
  Serial.print(context.config.maxDuty, 4);
  Serial.print(", hardware-cap=");
  Serial.print(hardwareDutyCap(), 4);
  Serial.print(", max-correction=");
  Serial.print(context.config.maxCorrection, 4);
  Serial.print(", integral-limit=");
  Serial.print(context.config.integralLimit, 4);
  Serial.print(", derivative-limit=");
  Serial.print(context.config.derivativeLimit, 4);
  Serial.print(", derivative-alpha=");
  Serial.print(context.config.derivativeFilterAlpha, 4);
  Serial.print(", polarity=");
  Serial.print(context.config.steeringPolarity);
  Serial.print(", period-ms=");
  Serial.print(context.config.controlPeriodMs);
  Serial.print(", timeout-ms=");
  Serial.print(context.config.remoteCommandTimeoutMs);
  Serial.print(", telemetry=");
  Serial.print(context.config.telemetryEnabled ? 1 : 0);
  Serial.print(", auto-state=");
  Serial.print(robot::solarPanelAutonomyStateName(context.autonomous_state));
  Serial.print(", auto-fault=");
  Serial.print(
      robot::solarPanelFaultReasonName(context.autonomous_fault_reason));
  Serial.print(", solar-ir-raw=");
  Serial.print(context.last_solar_detector_update.raw_amplitude);
  Serial.print(", solar-ir-filtered=");
  Serial.print(context.last_solar_detector_update.filtered_amplitude, 2);
  Serial.print(", solar-confirm-ms=");
  Serial.print(context.last_solar_detector_update.confirmation_progress_ms);
  Serial.print(", solar-start-duty=");
  Serial.print(context.solar_speed_config.start_base_duty, 4);
  Serial.print(", solar-slow-after-ms=");
  Serial.print(context.solar_speed_config.slow_after_ms);
  Serial.print(", solar-slow-duty=");
  Serial.print(context.solar_speed_config.slow_base_duty, 4);
  Serial.print(", solar-contact-timeout-ms=");
  Serial.print(context.solar_contact_config.timeout_ms);
  Serial.print(", solar-strafe-delay-ms=");
  Serial.print(context.solar_contact_config.strafe_start_delay_ms);
  Serial.print(", solar-imu-strafe-duty=");
  Serial.print(context.imu_heading_hold_config.maximum_strafe_duty, 4);
  Serial.print(", solar-retry-forward-duty=");
  Serial.print(context.solar_contact_config.retry_forward_duty, 4);
  Serial.print(", solar-retry-left-ms=");
  Serial.print(
      context.solar_contact_config.retry_strafe_left_duration_ms);
  Serial.print(", solar-retry-forward-ms=");
  Serial.print(context.solar_contact_config.retry_forward_duration_ms);
  Serial.print(", solar-retry-strafe-timeout-ms=");
  Serial.print(context.solar_contact_config.retry_strafe_timeout_ms);
  Serial.print(", solar-post-contact-forward-ms=");
  Serial.print(
      context.solar_contact_config.post_contact_forward_duration_ms);
  Serial.print(", solar-post-contact-forward-duty=");
  Serial.print(context.solar_contact_config.post_contact_forward_duty, 4);
  Serial.print(", solar-post-contact-forward-delay-ms=");
  Serial.print(
      context.solar_contact_config.post_contact_forward_start_delay_ms);
  Serial.print(", solar-post-forward-strafe-delay-ms=");
  Serial.print(
      context.solar_contact_config.line_reacquire_strafe_start_delay_ms);
  if (rear_link.statusAvailable()) {
    const robot::Esp1StatusReport& esp1 = rear_link.latestStatus();
    Serial.print(", solar-limit-configured=");
    Serial.print(esp1.solar_panel_limit_switches_configured ? 1 : 0);
    Serial.print(", solar-limit-br-high=");
    Serial.print(esp1.solar_limit_back_right_high ? 1 : 0);
    Serial.print(", solar-limit-fr-high=");
    Serial.print(esp1.solar_limit_front_right_high ? 1 : 0);
  }
  Serial.print(", rear-line-configured=");
  Serial.print(rear_link.rearLineSnapshotAvailable() &&
                       rear_link.latestRearLineSnapshot().configured
                   ? 1
                   : 0);
  Serial.print(", rear-line-fresh=");
  Serial.print(rear_link.rearLineSnapshotFresh(
                   now_ms, context.rear_config.remoteCommandTimeoutMs)
                   ? 1
                   : 0);
  Serial.print(", rear-line-sequence=");
  Serial.print(rear_link.lastRearLineSequence());
  Serial.print(", sensors-configured=");
  Serial.print(sensors.configured() ? 1 : 0);
  Serial.print(", front-left-configured=");
  Serial.print(front_left.configured() ? 1 : 0);
  Serial.print(", front-right-configured=");
  Serial.print(front_right.configured() ? 1 : 0);
  Serial.print(", rear-link-configured=");
  Serial.print(rear_link.configured() ? 1 : 0);
  Serial.print(", rear-link-healthy=");
  Serial.println(rear_link.remoteStatusFresh(
                     now_ms, remoteStatusTimeoutMs(context.config))
                     ? 1
                     : 0);
}

void printCommands() {
  Serial.println("commands:");
  Serial.println("  help | status | stop");
  Serial.println("  motor test FL|FR|BL|BR <speed -1..1> <ms>");
  Serial.println("  drive fwd|back|left|right|cw|ccw <duty> <ms>");
  Serial.println("  motor invert FL|FR|BL|BR");
  Serial.println("  mode ..., sensor status, line status");
  Serial.println("  rear-line status");
  Serial.println("  habitat start|stop|status");
  Serial.println(
      "  habitat config <duty> <side-delay-ms> <search-align-timeout-ms> <reverse-duty> <reverse-duration-ms> <left|right> <distance-threshold-mm> <zone-count> <strafe-duty> <strafe-timeout-ms>");
  Serial.println("  auto solar|status");
  Serial.println("  lf start|stop|status|reset");
  Serial.println("  lf kp|ki|kd|base|max-duty|max-correction <value>");
  Serial.println("  lf integral-limit|derivative-limit|derivative-alpha <value>");
  Serial.println("  lf polarity <1|-1> | lf telemetry on|off");
  Serial.println("  rlf start|stop|status|reset (independent reverse tuning)");
  Serial.println("  rlf kp|ki|kd|base|max-duty|max-correction <value>");
  Serial.println("  rlf integral-limit|derivative-limit|derivative-alpha <value>");
  Serial.println("  rlf polarity <1|-1> | rlf telemetry on|off");
}

bool serialSetMode(RuntimeContext& context, const char* mode_text,
                   DigitalFrontLineSensorReader& sensors,
                   DualPwmMotorOutput& front_left,
                   DualPwmMotorOutput& front_right,
                   RearCommandLink& rear_link,
                   const robot::Milliseconds now_ms) {
  (void)sensors;
  robot::RobotTestMode mode{};
  if (!robot::parseRobotTestMode(mode_text, mode)) {
    printRejected("invalid mode");
    return false;
  }
  disableActuators(context, front_left, front_right, rear_link, now_ms);
  context.modes.setMode(mode, now_ms);
  resetSolarPanelAutonomy(context, now_ms);
  resetHabitatPieces(context, now_ms);
  resetHabitatPlacement(context, now_ms);
  resetTowerPieces(context, now_ms);
  resetPegFinder(context, now_ms);
  resetTimeTrial(context, now_ms);
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Serial, "mode changed");
  printOk("mode changed");
  return true;
}

bool updateTuningValue(RuntimeContext& context, const char* name,
                       const char* value_text, const bool rear) {
  robot::LineFollowerConfig next =
      rear ? context.rear_config : context.config;
  if (std::strcmp(name, "kp") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.kp = value;
  } else if (std::strcmp(name, "ki") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.ki = value;
  } else if (std::strcmp(name, "kd") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.kd = value;
  } else if (std::strcmp(name, "base") == 0 ||
             std::strcmp(name, "speed") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.baseDuty = value;
  } else if (std::strcmp(name, "max-duty") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.maxDuty = value;
  } else if (std::strcmp(name, "max-correction") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.maxCorrection = value;
  } else if (std::strcmp(name, "integral-limit") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.integralLimit = value;
  } else if (std::strcmp(name, "derivative-limit") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.derivativeLimit = value;
  } else if (std::strcmp(name, "derivative-alpha") == 0) {
    float value = 0.0F;
    if (!parseFloat(value_text, value)) {
      printRejected("malformed tuning value");
      return true;
    }
    next.derivativeFilterAlpha = value;
  } else if (std::strcmp(name, "period-ms") == 0) {
    robot::Milliseconds value = 0U;
    if (!parseUnsigned(value_text, value)) {
      printRejected("malformed period-ms");
      return true;
    }
    next.controlPeriodMs = value;
  } else if (std::strcmp(name, "timeout-ms") == 0) {
    robot::Milliseconds value = 0U;
    if (!parseUnsigned(value_text, value)) {
      printRejected("malformed timeout-ms");
      return true;
    }
    next.remoteCommandTimeoutMs = value;
  } else if (std::strcmp(name, "polarity") == 0) {
    int polarity = 0;
    if (!parsePolarity(value_text, polarity)) {
      printRejected("polarity must be 1 or -1");
      return true;
    }
    next.steeringPolarity = polarity;
  } else {
    return false;
  }

  if (rear) {
    next.baseDuty = std::fabs(next.baseDuty);
  }
  const robot::CommandValidationResult validation =
      robot::validateLineFollowerConfig(next, hardwareDutyCap());
  if (!validation.accepted) {
    printRejected(validation.reason);
    return true;
  }
  if (rear) {
    context.rear_config = next;
  } else {
    context.config = next;
  }
  printOk(name);
  return true;
}

bool requestSerialMotorTest(RuntimeContext& context, const char* wheel_text,
                            const char* speed_text,
                            const char* duration_text,
                            DualPwmMotorOutput& front_left,
                            DualPwmMotorOutput& front_right,
                            RearCommandLink& rear_link,
                            const robot::Milliseconds now_ms) {
  robot::WheelId wheel{};
  float speed = 0.0F;
  robot::Milliseconds duration_ms = 0U;
  if (!robot::parseWheelId(wheel_text, wheel) ||
      !parseFloat(speed_text, speed) ||
      !parseUnsigned(duration_text, duration_ms)) {
    printRejected("motor test syntax: motor test FL|FR|BL|BR <speed> <ms>");
    return false;
  }
  if (context.modes.currentMode() != robot::RobotTestMode::SingleMotorTest) {
    resetTowerPieces(context, now_ms);
    resetPegFinder(context, now_ms);
    resetTimeTrial(context, now_ms);
    disableActuators(context, front_left, front_right, rear_link, now_ms);
    context.modes.setMode(robot::RobotTestMode::SingleMotorTest, now_ms);
  }
  const robot::CommandValidationResult validation =
      robot::validateSingleMotorCommand(context.modes.currentMode(), speed,
                                        duration_ms,
                                        validationLimits(context));
  if (!validation.accepted) {
    printRejected(validation.reason);
    return false;
  }
  disableActuators(context, front_left, front_right, rear_link, now_ms);
  context.requested_command = robot::disabledFourWheelCommand();
  if (wheel == robot::WheelId::FrontLeft) {
    context.requested_command.front_left =
        makeTimedMotorCommand(speed, now_ms, duration_ms);
  } else if (wheel == robot::WheelId::FrontRight) {
    context.requested_command.front_right =
        makeTimedMotorCommand(speed, now_ms, duration_ms);
  } else if (wheel == robot::WheelId::BackLeft) {
    context.requested_command.back_left =
        makeTimedMotorCommand(speed, now_ms, duration_ms);
  } else {
    context.requested_command.back_right =
        makeTimedMotorCommand(speed, now_ms, duration_ms);
  }
  context.last_command_ms = now_ms;
  context.mode_expires_at_ms = now_ms + duration_ms;
  context.command_deadman_armed = true;
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Serial, "single motor test started");
  printOk("motor test started");
  return true;
}

bool requestSerialDrive(RuntimeContext& context, const char* direction,
                        const char* duty_text, const char* duration_text,
                        const robot::Milliseconds now_ms) {
  float duty = 0.0F;
  robot::Milliseconds duration_ms = 0U;
  if (!parseFloat(duty_text, duty) ||
      !parseUnsigned(duration_text, duration_ms)) {
    printRejected("drive syntax: drive fwd|back|left|right|cw|ccw <duty> <ms>");
    return false;
  }
  float vx = 0.0F;
  float vy = 0.0F;
  float wz = 0.0F;
  if (std::strcmp(direction, "fwd") == 0) {
    vy = 1.0F;
  } else if (std::strcmp(direction, "back") == 0) {
    vy = -1.0F;
  } else if (std::strcmp(direction, "left") == 0) {
    vx = -1.0F;
  } else if (std::strcmp(direction, "right") == 0) {
    vx = 1.0F;
  } else if (std::strcmp(direction, "cw") == 0) {
    wz = 1.0F;
  } else if (std::strcmp(direction, "ccw") == 0) {
    wz = -1.0F;
  } else {
    printRejected("unknown drive direction");
    return false;
  }

  if (context.modes.currentMode() !=
      robot::RobotTestMode::DistributedDriveTest) {
    resetTowerPieces(context, now_ms);
    resetPegFinder(context, now_ms);
    resetTimeTrial(context, now_ms);
    context.modes.setMode(robot::RobotTestMode::DistributedDriveTest, now_ms);
  }

  robot::CommandValidationResult validation =
      robot::validateDriveCommand(context.modes.currentMode(), vx, vy, wz,
                                  duty, validationLimits(context));
  if (validation.accepted) {
    validation = robot::validateTimedDuration(duration_ms,
                                             kMaxTimedTestDurationMs);
  }
  if (!validation.accepted) {
    printRejected(validation.reason);
    return false;
  }
  context.requested_command =
      makeManualDriveCommand(context, vx, vy, wz, duty, now_ms);
  context.last_command_ms = now_ms;
  context.mode_expires_at_ms = now_ms + duration_ms;
  context.command_deadman_armed = true;
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Serial, "drive test started");
  printOk("drive test started");
  return true;
}

void printLineStatus(const RuntimeContext& context,
                     const DigitalFrontLineSensorReader& sensors) {
  const robot::LineObservation& observation = context.last_line_observation;
  Serial.print("line LSFL=");
  Serial.print(observation.left_black ? "black" : "white");
  Serial.print("(raw=");
  Serial.print(digitalLevelName(sensors.lastLeftLevel()));
  Serial.print(')');
  Serial.print(", LSFR=");
  Serial.print(observation.right_black ? "black" : "white");
  Serial.print("(raw=");
  Serial.print(digitalLevelName(sensors.lastRightLevel()));
  Serial.print(')');
  Serial.print(", error=");
  Serial.print(observation.error);
  Serial.print(", visible=");
  Serial.print(observation.line_visible ? 1 : 0);
  Serial.print(", has-history=");
  Serial.print(observation.hasHistory ? 1 : 0);
  Serial.print(", last-side=");
  Serial.println(observation.last_known_side);
}

void printRearLineStatus(const RuntimeContext& context,
                         const RearCommandLink& rear_link,
                         const robot::Milliseconds now_ms) {
  const robot::LineObservation& observation =
      context.last_rear_line_observation;
  const bool available = rear_link.rearLineSnapshotAvailable();
  const bool fresh = rear_link.rearLineSnapshotFresh(
      now_ms, context.rear_config.remoteCommandTimeoutMs);
  const robot::RearLineSensorSnapshot snapshot =
      available ? rear_link.latestRearLineSnapshot()
                : robot::RearLineSensorSnapshot{};
  Serial.print("rear-line LSBL=");
  Serial.print(available && fresh && snapshot.configured
                   ? (snapshot.left_electrical_high ? "black" : "white")
                   : "UNKNOWN");
  Serial.print("(raw=");
  Serial.print(available && fresh && snapshot.configured
                   ? digitalLevelName(snapshot.left_electrical_high ? HIGH
                                                                    : LOW)
                   : "UNKNOWN");
  Serial.print("), LSBR=");
  Serial.print(available && fresh && snapshot.configured
                   ? (snapshot.right_electrical_high ? "black" : "white")
                   : "UNKNOWN");
  Serial.print("(raw=");
  Serial.print(available && fresh && snapshot.configured
                   ? digitalLevelName(snapshot.right_electrical_high ? HIGH
                                                                     : LOW)
                   : "UNKNOWN");
  Serial.print("), configured=");
  Serial.print(available && snapshot.configured ? 1 : 0);
  Serial.print(", fresh=");
  Serial.print(fresh ? 1 : 0);
  Serial.print(", sequence=");
  Serial.print(rear_link.lastRearLineSequence());
  Serial.print(", age-ms=");
  Serial.print(available
                   ? elapsedSince(now_ms,
                                  rear_link.lastRearLineReceivedAtMs())
                   : 0U);
  Serial.print(", error=");
  Serial.print(observation.error);
  Serial.print(", logical-left=LSBR(");
  Serial.print(observation.left_black ? "black" : "white");
  Serial.print("), logical-right=LSBL(");
  Serial.print(observation.right_black ? "black" : "white");
  Serial.print(')');
  Serial.print(", visible=");
  Serial.print(observation.line_visible ? 1 : 0);
  Serial.print(", has-history=");
  Serial.print(observation.hasHistory ? 1 : 0);
  Serial.print(", last-side=");
  Serial.print(observation.last_known_side);
  Serial.print(", reverse-kp=");
  Serial.print(context.rear_config.kp, 4);
  Serial.print(", reverse-ki=");
  Serial.print(context.rear_config.ki, 4);
  Serial.print(", reverse-kd=");
  Serial.print(context.rear_config.kd, 4);
  Serial.print(", base-magnitude=");
  Serial.print(context.rear_config.baseDuty, 4);
  Serial.print(", effective-base=");
  Serial.print(-std::fabs(context.rear_config.baseDuty), 4);
  Serial.print(", max-duty=");
  Serial.print(context.rear_config.maxDuty, 4);
  Serial.print(", max-correction=");
  Serial.print(context.rear_config.maxCorrection, 4);
  Serial.print(", polarity=");
  Serial.println(context.rear_config.steeringPolarity);
}

void processCommand(RuntimeContext& context, char* line,
                    DigitalFrontLineSensorReader& sensors,
                    DualPwmMotorOutput& front_left,
                    DualPwmMotorOutput& front_right,
                    RearCommandLink& rear_link,
                    const robot::Milliseconds now_ms) {
  char* token = strtok(line, " \t\r\n");
  if (token == nullptr) {
    return;
  }
  if (std::strcmp(token, "help") == 0) {
    printCommands();
    return;
  }
  if (std::strcmp(token, "status") == 0) {
    printStatus(context, rear_link, sensors, front_left, front_right, now_ms);
    return;
  }
  if (std::strcmp(token, "stop") == 0) {
    emergencyStop(context, front_left, front_right, rear_link, now_ms,
                  robot::EventSource::Serial);
    printOk("stopped");
    return;
  }
  if (std::strcmp(token, "mode") == 0) {
    serialSetMode(context, strtok(nullptr, " \t\r\n"), sensors, front_left,
                  front_right, rear_link, now_ms);
    return;
  }
  if (std::strcmp(token, "habitat") == 0 ||
      std::strcmp(token, "habitat-pieces") == 0) {
    char* command = strtok(nullptr, " \t\r\n");
    if (command != nullptr && std::strcmp(command, "start") == 0) {
      requestHabitatPiecesStart(context, front_left, front_right, rear_link,
                                now_ms, robot::EventSource::Serial);
      printOk("Habitat Pieces start requested");
      return;
    }
    if (command != nullptr && std::strcmp(command, "stop") == 0) {
      disableActuators(context, front_left, front_right, rear_link, now_ms);
      resetHabitatPieces(context, now_ms);
      clearFault(context);
      printOk("Habitat Pieces stopped");
      return;
    }
    if (command != nullptr && std::strcmp(command, "status") == 0) {
      printStatus(context, rear_link, sensors, front_left, front_right,
                  now_ms);
      return;
    }
    if (command != nullptr && std::strcmp(command, "config") == 0) {
      float duty = 0.0F;
      robot::Milliseconds lss2_detection_delay_ms = 0U;
      robot::Milliseconds run_timeout_ms = 0U;
      float reverse_duty = 0.0F;
      robot::Milliseconds reverse_duration_ms = 0U;
      char* distance_strafe_direction_text = nullptr;
      robot::Milliseconds distance_threshold_mm = 0U;
      robot::Milliseconds distance_zone_count = 0U;
      float distance_strafe_duty = 0.0F;
      robot::Milliseconds distance_strafe_timeout_ms = 0U;
      if (!parseFloat(strtok(nullptr, " \t\r\n"), duty) ||
          !parseUnsigned(strtok(nullptr, " \t\r\n"),
                         lss2_detection_delay_ms) ||
          !parseUnsigned(strtok(nullptr, " \t\r\n"), run_timeout_ms) ||
          !parseFloat(strtok(nullptr, " \t\r\n"), reverse_duty) ||
          !parseUnsigned(strtok(nullptr, " \t\r\n"),
                         reverse_duration_ms) ||
          (distance_strafe_direction_text =
               strtok(nullptr, " \t\r\n")) == nullptr ||
          !parseUnsigned(strtok(nullptr, " \t\r\n"),
                         distance_threshold_mm) ||
          !parseUnsigned(strtok(nullptr, " \t\r\n"),
                         distance_zone_count) ||
          !parseFloat(strtok(nullptr, " \t\r\n"),
                      distance_strafe_duty) ||
          !parseUnsigned(strtok(nullptr, " \t\r\n"),
                         distance_strafe_timeout_ms)) {
        printRejected(
            "habitat config <duty> <side-delay-ms> <search-align-timeout-ms> <reverse-duty> <reverse-duration-ms> <left|right> <distance-threshold-mm> <zone-count> <strafe-duty> <strafe-timeout-ms>");
        return;
      }
      robot::HabitatPiecesStrafeDirection distance_strafe_direction =
          robot::HabitatPiecesStrafeDirection::None;
      if (std::strcmp(distance_strafe_direction_text, "left") == 0 ||
          std::strcmp(distance_strafe_direction_text, "LEFT") == 0 ||
          std::strcmp(distance_strafe_direction_text, "l") == 0 ||
          std::strcmp(distance_strafe_direction_text, "L") == 0) {
        distance_strafe_direction =
            robot::HabitatPiecesStrafeDirection::Left;
      } else if (
          std::strcmp(distance_strafe_direction_text, "right") == 0 ||
          std::strcmp(distance_strafe_direction_text, "RIGHT") == 0 ||
          std::strcmp(distance_strafe_direction_text, "r") == 0 ||
          std::strcmp(distance_strafe_direction_text, "R") == 0) {
        distance_strafe_direction =
            robot::HabitatPiecesStrafeDirection::Right;
      }
      // The compact serial command retains the pickup-extension settings;
      // use the web form/API to adjust the full route configuration.
      robot::HabitatPiecesConfig next = context.habitat_pieces_config;
      next.line_follow_duty = duty;
      next.lss2_detection_delay_ms = lss2_detection_delay_ms;
      next.run_timeout_ms = run_timeout_ms;
      next.reverse_duty = reverse_duty;
      next.reverse_duration_ms = reverse_duration_ms;
      next.distance_strafe_direction = distance_strafe_direction;
      if (distance_threshold_mm <= UINT16_MAX) {
        next.distance_threshold_mm =
            static_cast<std::uint16_t>(distance_threshold_mm);
      }
      if (distance_zone_count <= UINT16_MAX) {
        next.distance_zone_target_count =
            static_cast<std::uint16_t>(distance_zone_count);
      }
      next.distance_strafe_duty = distance_strafe_duty;
      next.distance_strafe_timeout_ms =
          distance_strafe_timeout_ms;
      robot::LineFollowerConfig line_config = context.config;
      line_config.baseDuty = duty;
      if (g_runtime.stepper == nullptr ||
          g_runtime.stepper->maximumPositionSteps() <= 0 ||
          g_runtime.stepper->maximumPositionSteps() >
              static_cast<std::int64_t>(UINT32_MAX) ||
          !robot::habitatPiecesConfigValid(
              next, activeMotionDutyCap(context),
              kMaxTimedTestDurationMs,
              g_runtime.stepper->maximumSpeedStepsPerSecond(),
              static_cast<std::uint32_t>(
                  g_runtime.stepper->maximumPositionSteps())) ||
          distance_threshold_mm > UINT16_MAX ||
          distance_zone_count > UINT16_MAX ||
          !robot::validateLineFollowerConfig(line_config, hardwareDutyCap())
               .accepted) {
        printRejected("Habitat Pieces config is outside safe limits");
        return;
      }
      if (context.habitat_pieces.state !=
              robot::HabitatPiecesState::WaitForStart &&
          context.habitat_pieces.state !=
              robot::HabitatPiecesState::Complete &&
          context.habitat_pieces.state !=
              robot::HabitatPiecesState::Fault) {
        printRejected("stop Habitat Pieces before changing config");
        return;
      }
      context.habitat_pieces_config = next;
      resetHabitatPieces(context, now_ms);
      clearFault(context);
      printOk("Habitat Pieces config updated");
      return;
    }
    printRejected("habitat start|stop|status|config");
    return;
  }
  if (std::strcmp(token, "auto") == 0 ||
      std::strcmp(token, "autonomous") == 0) {
    char* command = strtok(nullptr, " \t\r\n");
    if (command != nullptr &&
        (std::strcmp(command, "solar") == 0 ||
         std::strcmp(command, "start") == 0)) {
      requestSolarPanelAutonomyStart(context, front_left, front_right,
                                     rear_link, now_ms,
                                     robot::EventSource::Serial);
      printOk("solar autonomy start requested");
      return;
    }
    if (command != nullptr && std::strcmp(command, "status") == 0) {
      printStatus(context, rear_link, sensors, front_left, front_right,
                  now_ms);
      return;
    }
    printRejected("auto solar|status");
    return;
  }
  if (std::strcmp(token, "sensor") == 0) {
    char* command = strtok(nullptr, " \t\r\n");
    if (command != nullptr && std::strcmp(command, "status") == 0) {
      printLineStatus(context, sensors);
    } else {
      printRejected("sensor status");
    }
    return;
  }
  if (std::strcmp(token, "line") == 0) {
    char* command = strtok(nullptr, " \t\r\n");
    if (command != nullptr && std::strcmp(command, "status") == 0) {
      printLineStatus(context, sensors);
    } else {
      printRejected("line status");
    }
    return;
  }
  if (std::strcmp(token, "rear-line") == 0) {
    char* command = strtok(nullptr, " \t\r\n");
    if (command != nullptr && std::strcmp(command, "status") == 0) {
      printRearLineStatus(context, rear_link, now_ms);
    } else {
      printRejected("rear-line status");
    }
    return;
  }
  if (std::strcmp(token, "motor") == 0) {
    char* command = strtok(nullptr, " \t\r\n");
    if (command != nullptr && std::strcmp(command, "test") == 0) {
      char* wheel_text = strtok(nullptr, " \t\r\n");
      char* speed_text = strtok(nullptr, " \t\r\n");
      char* duration_text = strtok(nullptr, " \t\r\n");
      requestSerialMotorTest(context, wheel_text, speed_text, duration_text,
                             front_left, front_right, rear_link, now_ms);
      return;
    }
    if (command != nullptr && std::strcmp(command, "invert") == 0) {
      robot::WheelId wheel{};
      if (!robot::parseWheelId(strtok(nullptr, " \t\r\n"), wheel)) {
        printRejected("invalid motor id");
        return;
      }
      if (!allWheelCommandsDisabled(context.last_commanded_wheels) ||
          context.command_deadman_armed) {
        printRejected("motor inversion is only allowed while stopped");
        return;
      }
      if (wheel == robot::WheelId::FrontLeft) {
        front_left.setRuntimeInverted(!front_left.runtimeInverted());
        printOk("FL inversion toggled");
      } else if (wheel == robot::WheelId::FrontRight) {
        front_right.setRuntimeInverted(!front_right.runtimeInverted());
        printOk("FR inversion toggled");
      } else {
        printRejected("back inversion must be changed on ESP1");
      }
      return;
    }
    printRejected("motor test|invert");
    return;
  }
  if (std::strcmp(token, "drive") == 0) {
    char* direction = strtok(nullptr, " \t\r\n");
    char* duty_text = strtok(nullptr, " \t\r\n");
    char* duration_text = strtok(nullptr, " \t\r\n");
    requestSerialDrive(context, direction, duty_text, duration_text, now_ms);
    return;
  }
  if (std::strcmp(token, "lf") == 0 ||
      std::strcmp(token, "rlf") == 0) {
    const bool rear_line_command = std::strcmp(token, "rlf") == 0;
    char* command = strtok(nullptr, " \t\r\n");
    if (command == nullptr) {
      printCommands();
      return;
    }
    if (std::strcmp(command, "start") == 0) {
      robot::Milliseconds duration_ms = kMaxTimedTestDurationMs;
      char* duration_text = strtok(nullptr, " \t\r\n");
      if (duration_text != nullptr &&
          !parseUnsigned(duration_text, duration_ms)) {
        printRejected("malformed duration");
        return;
      }
      const robot::CommandValidationResult duration_validation =
          robot::validateTimedDuration(duration_ms, kMaxTimedTestDurationMs);
      if (!duration_validation.accepted) {
        printRejected(duration_validation.reason);
        return;
      }
      const robot::RobotTestMode requested_mode =
          rear_line_command ? robot::RobotTestMode::RearLineFollowTest
                            : robot::RobotTestMode::LineFollowTest;
      if (context.modes.currentMode() != requested_mode) {
        resetTowerPieces(context, now_ms);
        resetPegFinder(context, now_ms);
        resetTimeTrial(context, now_ms);
        disableActuators(context, front_left, front_right, rear_link, now_ms);
        context.modes.setMode(requested_mode, now_ms);
      }
      const bool requirements_met =
          rear_line_command
              ? rearLineStartRequirementsMet(front_left, front_right,
                                             rear_link, now_ms, context)
              : startRequirementsMet(sensors, front_left, front_right,
                                     rear_link, now_ms, context);
      if (!requirements_met) {
        setFault(context, robot::FaultCode::HardwareNotConfigured,
                 rear_line_command
                     ? "rear line follower hardware or sensor stream is incomplete"
                     : "line follower hardware requirements are incomplete");
        printRejected(
            rear_line_command
                ? "configure rear sensors, motors, UART, fresh ESP1 data, max-duty, hardware cap"
                : "configure sensors, motors, UART, ESP1 status, max-duty, hardware cap");
        return;
      }
      robot::startLineFollower(context.follower_state, now_ms);
      context.last_command_ms = now_ms;
      context.mode_expires_at_ms = now_ms + duration_ms;
      context.command_deadman_armed = true;
      printOk(rear_line_command ? "rear line follower started"
                                : "line follower started");
      return;
    }
    if (std::strcmp(command, "stop") == 0) {
      disableActuators(context, front_left, front_right, rear_link, now_ms);
      printOk(rear_line_command ? "rear line follower stopped"
                                : "line follower stopped");
      return;
    }
    if (std::strcmp(command, "status") == 0) {
      if (rear_line_command) {
        printRearLineStatus(context, rear_link, now_ms);
      } else {
        printStatus(context, rear_link, sensors, front_left, front_right,
                    now_ms);
      }
      return;
    }
    if (std::strcmp(command, "reset") == 0) {
      disableActuators(context, front_left, front_right, rear_link, now_ms);
      if (rear_line_command) {
        context.last_rear_update = {};
        context.last_rear_line_observation = {};
        context.rear_line_sensor_last_known_side = 0;
      } else {
        context.last_update = {};
        context.last_line_observation = {};
        context.line_sensor_last_known_side = 0;
      }
      printOk(rear_line_command ? "rear line follower reset"
                                : "line follower reset");
      return;
    }
    if (std::strcmp(command, "telemetry") == 0) {
      char* value = strtok(nullptr, " \t\r\n");
      bool telemetry_enabled = false;
      if (!parseOnOff(value, telemetry_enabled)) {
        printRejected("telemetry must be on or off");
        return;
      }
      if (rear_line_command) {
        context.rear_config.telemetryEnabled = telemetry_enabled;
      } else {
        context.config.telemetryEnabled = telemetry_enabled;
      }
      printOk("telemetry");
      return;
    }
    if (updateTuningValue(context, command, strtok(nullptr, " \t\r\n"),
                          rear_line_command)) {
      return;
    }
    printRejected(rear_line_command ? "unknown rlf command"
                                    : "unknown lf command");
    return;
  }

  printRejected("unknown command");
}

void pollSerialCommands(RuntimeContext& context,
                        DigitalFrontLineSensorReader& sensors,
                        DualPwmMotorOutput& front_left,
                        DualPwmMotorOutput& front_right,
                        RearCommandLink& rear_link,
                        const robot::Milliseconds now_ms) {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (context.command_length > 0U) {
        context.command_buffer[context.command_length] = '\0';
        processCommand(context, context.command_buffer, sensors, front_left,
                       front_right, rear_link, now_ms);
        context.command_length = 0U;
      }
    } else if (context.command_length + 1U < kSerialCommandBufferSize) {
      context.command_buffer[context.command_length++] = ch;
    } else {
      context.command_length = 0U;
      printRejected("command too long");
    }
  }
}

void printTelemetry(RuntimeContext& context,
                    const DigitalFrontLineSensorReader& sensors,
                    const RearCommandLink& rear_link,
                    const robot::Milliseconds now_ms) {
  if (!context.config.telemetryEnabled ||
      now_ms - context.last_telemetry_at_ms < kTelemetryPeriodMs ||
      Serial.availableForWrite() < 160) {
    return;
  }
  context.last_telemetry_at_ms = now_ms;

  const robot::LineFollowerUpdate& update = context.last_update;
  const robot::LineObservation& observation = update.observation;
  const robot::FourWheelCommand& wheels = update.wheel_command;
  const robot::Milliseconds rear_age =
      rear_link.lastSentAtMs() == 0U ? 0U
                                     : elapsedSince(now_ms,
                                                    rear_link.lastSentAtMs());
  const bool remote_healthy =
      rear_link.remoteStatusFresh(now_ms, remoteStatusTimeoutMs(context.config));

  Serial.print("lf_csv,");
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(robot::robotTestModeName(context.modes.currentMode()));
  Serial.print(',');
  Serial.print(context.follower_state.enabled ? 1 : 0);
  Serial.print(',');
  Serial.print(sensors.lastLeftLevel());
  Serial.print(',');
  Serial.print(sensors.lastRightLevel());
  Serial.print(',');
  Serial.print(observation.left_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.right_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.error);
  Serial.print(',');
  Serial.print(observation.last_known_side);
  Serial.print(',');
  Serial.print(observation.line_visible ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.hasHistory ? 1 : 0);
  Serial.print(',');
  Serial.print(context.config.kp, 4);
  Serial.print(',');
  Serial.print(context.config.ki, 4);
  Serial.print(',');
  Serial.print(context.config.kd, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.proportional_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.integral_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.derivative_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.correction, 4);
  Serial.print(',');
  Serial.print(context.config.steeringPolarity);
  Serial.print(',');
  Serial.print(context.config.baseDuty, 4);
  Serial.print(',');
  Serial.print(context.config.maxDuty, 4);
  Serial.print(',');
  Serial.print(context.config.maxCorrection, 4);
  Serial.print(',');
  Serial.print(wheels.front_left.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.front_right.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.back_left.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.back_right.duty_command_milli);
  Serial.print(',');
  Serial.print(remote_healthy ? 1 : 0);
  Serial.print(',');
  Serial.print(rear_link.lastSequenceSent());
  Serial.print(',');
  Serial.print(rear_age);
  Serial.print(',');
  Serial.print(robot::solarPanelAutonomyStateName(
      context.autonomous_state));
  Serial.print(',');
  Serial.print(context.last_solar_detector_update.raw_amplitude);
  Serial.print(',');
  Serial.print(context.last_solar_detector_update.filtered_amplitude, 2);
  Serial.print(',');
  Serial.print(
      context.last_solar_detector_update.confirmation_progress_ms);
  Serial.print(',');
  Serial.print(
      context.last_solar_detector_update.beacon_detected ? 1 : 0);
  Serial.print(',');
  Serial.print(solarSlowModeActive(
                   context,
                   elapsedSince(now_ms, context.autonomous_state_entered_at_ms))
                   ? 1
                   : 0);
  Serial.print(',');
  Serial.print(context.solar_speed_config.start_base_duty, 4);
  Serial.print(',');
  Serial.print(context.solar_speed_config.slow_after_ms);
  Serial.print(',');
  Serial.print(context.solar_speed_config.slow_base_duty, 4);
  Serial.print(',');
  Serial.print(robot::solarPanelFaultReasonName(
      context.autonomous_fault_reason));
  Serial.println();
}

void runSensorOnlyTelemetry(RuntimeContext& context,
                            const DigitalFrontLineSensorReader& sensors,
                            const robot::Milliseconds now_ms) {
  if (now_ms - context.last_telemetry_at_ms < kTelemetryPeriodMs ||
      Serial.availableForWrite() < 80) {
    return;
  }
  context.last_telemetry_at_ms = now_ms;
  const robot::LineObservation& observation = context.last_line_observation;
  Serial.print("sensor,");
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(digitalLevelName(sensors.lastLeftLevel()));
  Serial.print(',');
  Serial.print(digitalLevelName(sensors.lastRightLevel()));
  Serial.print(',');
  Serial.print(observation.left_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.right_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.error);
  Serial.print(',');
  Serial.println(observation.line_visible ? 1 : 0);
}

void printRearTelemetry(RuntimeContext& context,
                        const RearCommandLink& rear_link,
                        const robot::Milliseconds now_ms) {
  if (!context.rear_config.telemetryEnabled ||
      now_ms - context.last_telemetry_at_ms < kTelemetryPeriodMs ||
      Serial.availableForWrite() < 160) {
    return;
  }
  context.last_telemetry_at_ms = now_ms;

  const robot::LineFollowerUpdate& update = context.last_rear_update;
  const robot::LineObservation& observation = update.observation;
  const robot::FourWheelCommand& wheels = update.wheel_command;
  const robot::RearLineSensorSnapshot& sensors =
      rear_link.latestRearLineSnapshot();
  const robot::Milliseconds rear_command_age =
      rear_link.lastSentAtMs() == 0U
          ? 0U
          : elapsedSince(now_ms, rear_link.lastSentAtMs());
  const robot::Milliseconds sensor_age =
      rear_link.lastRearLineReceivedAtMs() == 0U
          ? 0U
          : elapsedSince(now_ms, rear_link.lastRearLineReceivedAtMs());
  const bool status_fresh = rear_link.remoteStatusFresh(
      now_ms, remoteStatusTimeoutMs(context.rear_config));
  const bool sensor_fresh = rear_link.rearLineSnapshotFresh(
      now_ms, context.rear_config.remoteCommandTimeoutMs);

  Serial.print("rlf_csv,");
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(robot::robotTestModeName(context.modes.currentMode()));
  Serial.print(',');
  Serial.print(context.follower_state.enabled ? 1 : 0);
  Serial.print(',');
  Serial.print(sensors.left_electrical_high ? HIGH : LOW);
  Serial.print(',');
  Serial.print(sensors.right_electrical_high ? HIGH : LOW);
  Serial.print(',');
  Serial.print(observation.left_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.right_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.error);
  Serial.print(',');
  Serial.print(observation.last_known_side);
  Serial.print(',');
  Serial.print(observation.line_visible ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.hasHistory ? 1 : 0);
  Serial.print(',');
  Serial.print(context.rear_config.kp, 4);
  Serial.print(',');
  Serial.print(context.rear_config.ki, 4);
  Serial.print(',');
  Serial.print(context.rear_config.kd, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.proportional_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.integral_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.derivative_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.correction, 4);
  Serial.print(',');
  Serial.print(context.rear_config.steeringPolarity);
  Serial.print(',');
  Serial.print(-std::fabs(context.rear_config.baseDuty), 4);
  Serial.print(',');
  Serial.print(context.rear_config.maxDuty, 4);
  Serial.print(',');
  Serial.print(context.rear_config.maxCorrection, 4);
  Serial.print(',');
  Serial.print(wheels.front_left.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.front_right.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.back_left.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.back_right.duty_command_milli);
  Serial.print(',');
  Serial.print(status_fresh ? 1 : 0);
  Serial.print(',');
  Serial.print(rear_link.lastSequenceSent());
  Serial.print(',');
  Serial.print(rear_command_age);
  Serial.print(',');
  Serial.print(sensors.configured ? 1 : 0);
  Serial.print(',');
  Serial.print(sensor_fresh ? 1 : 0);
  Serial.print(',');
  Serial.print(rear_link.lastRearLineSequence());
  Serial.print(',');
  Serial.println(sensor_age);
}

void runRearSensorOnlyTelemetry(RuntimeContext& context,
                                const RearCommandLink& rear_link,
                                const robot::Milliseconds now_ms) {
  if (now_ms - context.last_telemetry_at_ms < kTelemetryPeriodMs ||
      Serial.availableForWrite() < 96) {
    return;
  }
  context.last_telemetry_at_ms = now_ms;
  const bool available = rear_link.rearLineSnapshotAvailable();
  const bool fresh = rear_link.rearLineSnapshotFresh(
      now_ms, context.rear_config.remoteCommandTimeoutMs);
  const robot::RearLineSensorSnapshot snapshot =
      available ? rear_link.latestRearLineSnapshot()
                : robot::RearLineSensorSnapshot{};
  const robot::LineObservation& observation =
      context.last_rear_line_observation;
  Serial.print("rear_sensor,");
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(available && fresh && snapshot.configured
                   ? digitalLevelName(snapshot.left_electrical_high ? HIGH
                                                                    : LOW)
                   : "UNKNOWN");
  Serial.print(',');
  Serial.print(available && fresh && snapshot.configured
                   ? digitalLevelName(snapshot.right_electrical_high ? HIGH
                                                                     : LOW)
                   : "UNKNOWN");
  Serial.print(',');
  Serial.print(observation.left_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.right_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.error);
  Serial.print(',');
  Serial.print(observation.line_visible ? 1 : 0);
  Serial.print(',');
  Serial.print(snapshot.configured ? 1 : 0);
  Serial.print(',');
  Serial.print(fresh ? 1 : 0);
  Serial.print(',');
  Serial.print(rear_link.lastRearLineSequence());
  Serial.print(',');
  Serial.println(available
                     ? elapsedSince(now_ms,
                                    rear_link.lastRearLineReceivedAtMs())
                     : 0U);
}

void loadPreferences(Preferences& preferences, RuntimeContext& context,
                     DualPwmMotorOutput& front_left,
                     DualPwmMotorOutput& front_right,
                     ClawServoBank& claws) {
  front_left.setRuntimeInverted(preferences.getBool("inv_fl", false));
  front_right.setRuntimeInverted(preferences.getBool("inv_fr", false));
  context.config.kp = preferences.getFloat("kp", context.config.kp);
  context.config.ki = preferences.getFloat("ki", context.config.ki);
  context.config.kd = preferences.getFloat("kd", context.config.kd);
  context.config.baseDuty =
      preferences.getFloat("base", context.config.baseDuty);
  context.config.maxDuty =
      preferences.getFloat("max", context.config.maxDuty);
  context.config.maxCorrection =
      preferences.getFloat("corr", context.config.maxCorrection);
  context.config.integralLimit =
      preferences.getFloat("ilim", context.config.integralLimit);
  context.config.derivativeLimit =
      preferences.getFloat("dlim", context.config.derivativeLimit);
  context.config.derivativeFilterAlpha =
      preferences.getFloat("dalpha", context.config.derivativeFilterAlpha);
  context.config.controlPeriodMs =
      preferences.getUInt("period", context.config.controlPeriodMs);
  context.config.remoteCommandTimeoutMs =
      preferences.getUInt("rto", context.config.remoteCommandTimeoutMs);
  context.config.telemetryEnabled =
      preferences.getBool("lftele", context.config.telemetryEnabled);
  context.config.steeringPolarity =
      preferences.getInt("pol", context.config.steeringPolarity) < 0 ? -1 : 1;
  context.habitat_pieces_config.line_follow_duty = preferences.getFloat(
      "hpduty", context.habitat_pieces_config.line_follow_duty);
  context.habitat_pieces_config.lss2_detection_delay_ms =
      preferences.getUInt(
          "hpdetect",
          context.habitat_pieces_config.lss2_detection_delay_ms);
  context.habitat_pieces_config.run_timeout_ms = preferences.getUInt(
      "hptimeout", context.habitat_pieces_config.run_timeout_ms);
  context.habitat_pieces_config.reverse_duty = preferences.getFloat(
      "hprevduty", context.habitat_pieces_config.reverse_duty);
  context.habitat_pieces_config.reverse_duration_ms = preferences.getUInt(
      "hprevms", context.habitat_pieces_config.reverse_duration_ms);
  const int habitat_distance_strafe_direction = preferences.getInt(
      "hpdsdir",
      static_cast<int>(context.habitat_pieces_config
                           .distance_strafe_direction));
  context.habitat_pieces_config.distance_strafe_direction =
      habitat_distance_strafe_direction == -1
          ? robot::HabitatPiecesStrafeDirection::Left
          : (habitat_distance_strafe_direction == 1
                 ? robot::HabitatPiecesStrafeDirection::Right
                 : robot::HabitatPiecesStrafeDirection::None);
  const std::uint32_t habitat_distance_threshold_mm =
      preferences.getUInt(
          "hpdsthr",
          context.habitat_pieces_config.distance_threshold_mm);
  context.habitat_pieces_config.distance_threshold_mm =
      habitat_distance_threshold_mm <= UINT16_MAX
          ? static_cast<std::uint16_t>(habitat_distance_threshold_mm)
          : 0U;
  const std::uint32_t habitat_distance_zone_target_count =
      preferences.getUInt(
          "hpdscnt",
          context.habitat_pieces_config.distance_zone_target_count);
  context.habitat_pieces_config.distance_zone_target_count =
      habitat_distance_zone_target_count <= UINT16_MAX
          ? static_cast<std::uint16_t>(
                habitat_distance_zone_target_count)
          : 0U;
  context.habitat_pieces_config.distance_strafe_duty =
      preferences.getFloat(
          "hpdsduty",
          context.habitat_pieces_config.distance_strafe_duty);
  context.habitat_pieces_config.distance_strafe_timeout_ms =
      preferences.getUInt(
          "hpdstmo",
          context.habitat_pieces_config.distance_strafe_timeout_ms);
  context.habitat_pieces_config.compensation_strafe_duty =
      preferences.getFloat(
          "hpcmpd",
          context.habitat_pieces_config.compensation_strafe_duty);
  context.habitat_pieces_config.compensation_strafe_duration_ms =
      preferences.getUInt(
          "hpcmpms",
          context.habitat_pieces_config.compensation_strafe_duration_ms);
  context.habitat_pieces_config.slide_down_speed_steps_per_second =
      preferences.getUInt(
          "hpkdnspd",
          context.habitat_pieces_config.slide_down_speed_steps_per_second);
  context.habitat_pieces_config.slide_down_timeout_ms =
      preferences.getUInt(
          "hpkdntmo",
          context.habitat_pieces_config.slide_down_timeout_ms);
  context.habitat_pieces_config.forward_to_distance_duty =
      preferences.getFloat(
          "hpkfwdd",
          context.habitat_pieces_config.forward_to_distance_duty);
  const std::uint32_t habitat_forward_stop_distance_mm =
      preferences.getUInt(
          "hpkfwdmm",
          context.habitat_pieces_config.forward_stop_distance_mm);
  context.habitat_pieces_config.forward_stop_distance_mm =
      habitat_forward_stop_distance_mm <= UINT16_MAX
          ? static_cast<std::uint16_t>(
                habitat_forward_stop_distance_mm)
          : 0U;
  context.habitat_pieces_config.forward_to_distance_timeout_ms =
      preferences.getUInt(
          "hpkfwdtmo",
          context.habitat_pieces_config.forward_to_distance_timeout_ms);
  context.habitat_pieces_config.slide_lift_steps =
      preferences.getUInt(
          "hpklftstp", context.habitat_pieces_config.slide_lift_steps);
  context.habitat_pieces_config.slide_lift_speed_steps_per_second =
      preferences.getUInt(
          "hpklftspd",
          context.habitat_pieces_config.slide_lift_speed_steps_per_second);
  context.habitat_pieces_config.slide_lift_timeout_ms =
      preferences.getUInt(
          "hpklfttmo",
          context.habitat_pieces_config.slide_lift_timeout_ms);
  context.habitat_pieces_config.post_pickup_reverse_duty =
      preferences.getFloat(
          "hpkrevd",
          context.habitat_pieces_config.post_pickup_reverse_duty);
  context.habitat_pieces_config.post_pickup_reverse_duration_ms =
      preferences.getUInt(
          "hpkrevms",
          context.habitat_pieces_config.post_pickup_reverse_duration_ms);
  context.habitat_pieces_config.return_strafe_duty =
      preferences.getFloat(
          "hpkretd", context.habitat_pieces_config.return_strafe_duty);
  context.habitat_pieces_config.return_line_timeout_ms =
      preferences.getUInt(
          "hpkrettmo",
          context.habitat_pieces_config.return_line_timeout_ms);
  robot::HabitatPlacementConfig& placement =
      context.habitat_placement_config;
  placement.reverse_line_follow_duty = preferences.getFloat(
      "hprlduty", placement.reverse_line_follow_duty);
  placement.lss1_timeout_ms = preferences.getUInt(
      "hplsstmo", placement.lss1_timeout_ms);
  placement.post_lss1_delay_ms = preferences.getUInt(
      "hplssdel", placement.post_lss1_delay_ms);
  placement.initial_heading_turn_timeout_ms = preferences.getUInt(
      "hpihdtmo", placement.initial_heading_turn_timeout_ms);
  placement.pre_counter_clockwise_strafe_right_duty =
      preferences.getFloat(
          "hpprestrd",
          placement.pre_counter_clockwise_strafe_right_duty);
  placement.pre_counter_clockwise_strafe_right_duration_ms =
      preferences.getUInt(
          "hpprestrms",
          placement.pre_counter_clockwise_strafe_right_duration_ms);
  placement.counter_clockwise_angle_deg = preferences.getFloat(
      "hpccwdeg", placement.counter_clockwise_angle_deg);
  placement.counter_clockwise_timeout_ms = preferences.getUInt(
      "hpccwtmo", placement.counter_clockwise_timeout_ms);
  placement.forward_to_slide_duty = preferences.getFloat(
      "hpfwd1d", placement.forward_to_slide_duty);
  placement.forward_to_slide_duration_ms = preferences.getUInt(
      "hpfwd1ms", placement.forward_to_slide_duration_ms);
  placement.stepper_down_speed_steps_per_second = preferences.getUInt(
      "hpstpspd", placement.stepper_down_speed_steps_per_second);
  placement.stepper_down_timeout_ms = preferences.getUInt(
      "hpstptmo", placement.stepper_down_timeout_ms);
  placement.pusher_open_settle_ms = preferences.getUInt(
      "hpopenset", placement.pusher_open_settle_ms);
  placement.push_forward_duty = preferences.getFloat(
      "hppushd", placement.push_forward_duty);
  placement.push_forward_duration_ms = preferences.getUInt(
      "hppushms", placement.push_forward_duration_ms);
  placement.reverse_retreat_duty = preferences.getFloat(
      "hpretd", placement.reverse_retreat_duty);
  placement.reverse_retreat_duration_ms = preferences.getUInt(
      "hpretms", placement.reverse_retreat_duration_ms);
  placement.clockwise_angle_deg = preferences.getFloat(
      "hpcwdeg", placement.clockwise_angle_deg);
  placement.clockwise_timeout_ms = preferences.getUInt(
      "hpcwtmo", placement.clockwise_timeout_ms);
  placement.post_clockwise_reverse_duty = preferences.getFloat(
      "hpcwrevd", placement.post_clockwise_reverse_duty);
  placement.post_clockwise_reverse_duration_ms =
      preferences.getUInt(
          "hpcwrevms",
          placement.post_clockwise_reverse_duration_ms);
  placement.post_clockwise_strafe_left_duty =
      preferences.getFloat(
          "hpcwstrld", placement.post_clockwise_strafe_left_duty);
  placement.post_clockwise_strafe_left_duration_ms =
      preferences.getUInt(
          "hpcwstrlms",
          placement.post_clockwise_strafe_left_duration_ms);
  placement.post_clockwise_delay_ms = preferences.getUInt(
      "hpcwdel", placement.post_clockwise_delay_ms);
  placement.exit_forward_duty = preferences.getFloat(
      "hpexitd", placement.exit_forward_duty);
  placement.exit_forward_duration_ms = preferences.getUInt(
      "hpexitms", placement.exit_forward_duration_ms);
  placement.post_forward_delay_ms = preferences.getUInt(
      "hpexitdel", placement.post_forward_delay_ms);
  placement.strafe_right_duty = preferences.getFloat(
      "hpstrd", placement.strafe_right_duty);
  placement.strafe_right_timeout_ms = preferences.getUInt(
      "hpstrtmo", placement.strafe_right_timeout_ms);
  context.imu_turn_config.maximum_rotation_duty =
      preferences.getFloat(
          "itmax", context.imu_turn_config.maximum_rotation_duty);
  context.imu_turn_config.kp =
      preferences.getFloat("itkp", context.imu_turn_config.kp);
  context.imu_turn_config.kd =
      preferences.getFloat("itkd", context.imu_turn_config.kd);
  context.imu_turn_config.angle_tolerance_deg =
      preferences.getFloat(
          "ittol", context.imu_turn_config.angle_tolerance_deg);
  context.imu_turn_config.maximum_finishing_yaw_rate_dps =
      preferences.getFloat(
          "itrate",
          context.imu_turn_config.maximum_finishing_yaw_rate_dps);
  context.imu_turn_config.settling_time_ms =
      preferences.getUInt(
          "itsettle", context.imu_turn_config.settling_time_ms);
  context.imu_turn_config.timeout_ms =
      preferences.getUInt(
          "ittimeout", context.imu_turn_config.timeout_ms);
  context.imu_turn_config.yaw_command_polarity =
      preferences.getInt(
          "itpol", context.imu_turn_config.yaw_command_polarity);
  context.imu_heading_hold_config.maximum_strafe_duty =
      preferences.getFloat(
          "ihmax",
          context.imu_heading_hold_config.maximum_strafe_duty);
  context.imu_heading_hold_config.kp =
      preferences.getFloat(
          "ihkp", context.imu_heading_hold_config.kp);
  context.imu_heading_hold_config.kd =
      preferences.getFloat(
          "ihkd", context.imu_heading_hold_config.kd);
  context.imu_heading_hold_config.maximum_yaw_correction_duty =
      preferences.getFloat(
          "ihcorr",
          context.imu_heading_hold_config
              .maximum_yaw_correction_duty);
  context.imu_heading_hold_config.yaw_command_polarity =
      preferences.getInt(
          "ihpol",
          context.imu_heading_hold_config.yaw_command_polarity);
  // First-time rear settings inherit the current front settings, then become
  // independently persistent under their own NVS keys.
  context.rear_config = context.config;
  context.rear_config.kp =
      preferences.getFloat("rkp", context.rear_config.kp);
  context.rear_config.ki =
      preferences.getFloat("rki", context.rear_config.ki);
  context.rear_config.kd =
      preferences.getFloat("rkd", context.rear_config.kd);
  context.rear_config.baseDuty = std::fabs(
      preferences.getFloat("rbase", context.rear_config.baseDuty));
  context.rear_config.maxDuty =
      preferences.getFloat("rmax", context.rear_config.maxDuty);
  context.rear_config.maxCorrection =
      preferences.getFloat("rcorr", context.rear_config.maxCorrection);
  context.rear_config.integralLimit =
      preferences.getFloat("rilim", context.rear_config.integralLimit);
  context.rear_config.derivativeLimit =
      preferences.getFloat("rdlim", context.rear_config.derivativeLimit);
  context.rear_config.derivativeFilterAlpha = preferences.getFloat(
      "rdalpha", context.rear_config.derivativeFilterAlpha);
  context.rear_config.controlPeriodMs =
      preferences.getUInt("rperiod", context.rear_config.controlPeriodMs);
  context.rear_config.remoteCommandTimeoutMs = preferences.getUInt(
      "rrto", context.rear_config.remoteCommandTimeoutMs);
  context.rear_config.telemetryEnabled =
      preferences.getBool("rlftele", context.rear_config.telemetryEnabled);
  context.rear_config.steeringPolarity =
      preferences.getInt("rpol", context.rear_config.steeringPolarity) < 0
          ? -1
          : 1;
  context.tower_pieces_config.reverse_line_duty = std::fabs(
      preferences.getFloat("tpduty", context.rear_config.baseDuty));
  context.tower_pieces_config.side_line_timeout_ms =
      preferences.getUInt(
          "tptmo", context.tower_pieces_config.side_line_timeout_ms);
  context.tower_pieces_config.side_line_cooldown_ms =
      preferences.getUInt(
          "tpcool", context.tower_pieces_config.side_line_cooldown_ms);
  context.tower_pieces_config.side_line_rearm_ms =
      preferences.getUInt(
          "tprearm", context.tower_pieces_config.side_line_rearm_ms);
  context.tower_pieces_config.post_line_delay_ms = preferences.getUInt(
      "tpdelay", context.tower_pieces_config.post_line_delay_ms);
  context.tower_pieces_config.strafe_right_duration_ms =
      preferences.getUInt(
          "tpsdur", context.tower_pieces_config.strafe_right_duration_ms);
  context.tower_pieces_config.strafe_right_duty =
      preferences.getFloat(
          "tpsduty", context.imu_heading_hold_config.maximum_strafe_duty);
  context.tower_pieces_config.post_strafe_pause_ms = preferences.getUInt(
      "tppause", context.tower_pieces_config.post_strafe_pause_ms);
  context.tower_pieces_config.clockwise_rotation_angle_deg = std::fabs(
      preferences.getFloat(
          "tprdeg",
          context.tower_pieces_config.clockwise_rotation_angle_deg));
  context.tower_pieces_config.post_rotation_pause_ms = preferences.getUInt(
      "tprpause", context.tower_pieces_config.post_rotation_pause_ms);
  context.tower_pieces_config.reverse_duty = std::fabs(
      preferences.getFloat("tpbkduty",
                           context.tower_pieces_config.reverse_duty));
  context.tower_pieces_config.reverse_duration_ms = preferences.getUInt(
      "tpbkdur", context.tower_pieces_config.reverse_duration_ms);
  const robot::Milliseconds legacy_shimmy_duration_ms =
      preferences.getUInt("tpshdur", 0U);
  context.tower_pieces_config.shimmy_right_duration_ms =
      preferences.getUInt(
          "tpshrdur",
          legacy_shimmy_duration_ms > 0U
              ? legacy_shimmy_duration_ms
              : context.tower_pieces_config.shimmy_right_duration_ms);
  context.tower_pieces_config.shimmy_left_duration_ms =
      preferences.getUInt(
          "tpshldur",
          legacy_shimmy_duration_ms > 0U
              ? legacy_shimmy_duration_ms
              : context.tower_pieces_config.shimmy_left_duration_ms);
  context.tower_pieces_config.shimmy_timeout_ms = preferences.getUInt(
      "tpshtmo",
      preferences.getUInt(
          "tprtmo", context.tower_pieces_config.shimmy_timeout_ms));
  context.tower_pieces_config.shimmy_duty =
      preferences.getFloat(
          "tpshduty",
          preferences.getFloat(
              "tplapp",
              context.imu_heading_hold_config.maximum_strafe_duty));
  context.tower_pieces_config.final_reverse_duty = std::fabs(
      preferences.getFloat(
          "tp_end_d",
          preferences.getFloat(
              "pf_end_d", context.tower_pieces_config.final_reverse_duty)));
  context.tower_pieces_config.final_reverse_duration_ms = preferences.getUInt(
      "tp_end_ms",
      preferences.getUInt(
          "pf_end_ms",
          context.tower_pieces_config.final_reverse_duration_ms));
  context.tower_pieces_config.post_final_reverse_delay_ms =
      preferences.getUInt(
          "tp_end_p",
          preferences.getUInt(
              "pf_end_p",
              context.tower_pieces_config.post_final_reverse_delay_ms));
  context.tower_pieces_config.post_winch_open_delay_ms = preferences.getUInt(
      "tp_wo_p",
      preferences.getUInt(
          "pf_wo_p", context.tower_pieces_config.post_winch_open_delay_ms));
  context.tower_pieces_config.post_claws_open_delay_ms = preferences.getUInt(
      "tp_co_p",
      preferences.getUInt(
          "pf_co_p", context.tower_pieces_config.post_claws_open_delay_ms));
  context.tower_pieces_config.pre_stepper_bottom_delay_ms =
      preferences.getUInt(
          "tp_pre_dn",
          context.tower_pieces_config.pre_stepper_bottom_delay_ms);
  context.tower_pieces_config.stepper_down_speed_steps_per_second =
      preferences.getUInt(
          "tp_dn_spd",
          preferences.getUInt(
              "pf_dn_spd",
              context.tower_pieces_config
                  .stepper_down_speed_steps_per_second));
  context.tower_pieces_config.post_stepper_bottom_delay_ms =
      preferences.getUInt(
          "tp_bot_p",
          preferences.getUInt(
              "pf_bot_p",
              context.tower_pieces_config.post_stepper_bottom_delay_ms));
  context.tower_pieces_config.post_claws_closed_delay_ms =
      preferences.getUInt(
          "tp_cc_p",
          preferences.getUInt(
              "pf_cc_p",
              context.tower_pieces_config.post_claws_closed_delay_ms));
  context.tower_pieces_config.stepper_up_speed_steps_per_second =
      preferences.getUInt(
          "tp_up_spd",
          preferences.getUInt(
              "pf_up_spd",
              context.tower_pieces_config.stepper_up_speed_steps_per_second));
  context.peg_finder_config.clockwise_angle_deg = std::fabs(
      preferences.getFloat(
          "pf_cw_deg", context.peg_finder_config.clockwise_angle_deg));
  context.peg_finder_config.post_rotation_pause_ms = preferences.getUInt(
      "pf_cw_p", context.peg_finder_config.post_rotation_pause_ms);
  context.peg_finder_config.reverse_duty = std::fabs(
      preferences.getFloat("pf_rev_d",
                           context.peg_finder_config.reverse_duty));
  context.peg_finder_config.reverse_duration_ms = preferences.getUInt(
      "pf_rev_ms", context.peg_finder_config.reverse_duration_ms);
  context.peg_finder_config.post_reverse_pause_ms = preferences.getUInt(
      "pf_rev_p", context.peg_finder_config.post_reverse_pause_ms);
  context.peg_finder_config.forward_duty = std::fabs(
      preferences.getFloat("pf_fwd_d",
                           context.peg_finder_config.forward_duty));
  context.peg_finder_config.forward_duration_ms = preferences.getUInt(
      "pf_fwd_ms", context.peg_finder_config.forward_duration_ms);
  context.peg_finder_config.funnel_forward_duty = std::fabs(
      preferences.getFloat("pf_fun_d",
                           context.peg_finder_config.funnel_forward_duty));
  context.peg_finder_config.funnel_forward_timeout_ms = preferences.getUInt(
      "pf_fun_ms", context.peg_finder_config.funnel_forward_timeout_ms);
  context.peg_finder_config.post_funnel_limit_delay_ms =
      preferences.getUInt(
          "pf_fun_p",
          context.peg_finder_config.post_funnel_limit_delay_ms);
  context.peg_finder_config.claw_open_interval_ms = preferences.getUInt(
      "pf_claw_p", context.peg_finder_config.claw_open_interval_ms);
  context.peg_finder_config.claw_open_order_1 = preferences.getUChar(
      "pf_claw_1", context.peg_finder_config.claw_open_order_1);
  context.peg_finder_config.claw_open_order_2 = preferences.getUChar(
      "pf_claw_2", context.peg_finder_config.claw_open_order_2);
  context.peg_finder_config.claw_open_order_3 = preferences.getUChar(
      "pf_claw_3", context.peg_finder_config.claw_open_order_3);
  context.peg_finder_config.post_claws_open_delay_ms = preferences.getUInt(
      "pf_claw_d", context.peg_finder_config.post_claws_open_delay_ms);
  context.peg_finder_config.funnel_reverse_duty = std::fabs(
      preferences.getFloat("pf_revfun_d",
                           context.peg_finder_config.funnel_reverse_duty));
  context.peg_finder_config.funnel_reverse_duration_ms = preferences.getUInt(
      "pf_revfun_ms",
      context.peg_finder_config.funnel_reverse_duration_ms);
  context.time_trial_config.post_solar_delay_ms = preferences.getUInt(
      "tt_sdly", context.time_trial_config.post_solar_delay_ms);
  context.time_trial_config.solar_to_tower_strafe_right_duration_ms =
      preferences.getUInt(
          "tt_sms",
          context.time_trial_config
              .solar_to_tower_strafe_right_duration_ms);
  context.time_trial_config.post_tower_delay_ms = preferences.getUInt(
      "tt_tdly", context.time_trial_config.post_tower_delay_ms);
  context.solar_thresholds.detect_1khz = static_cast<std::uint16_t>(
      preferences.getUInt("sdet1", context.solar_thresholds.detect_1khz));
  context.solar_thresholds.release_1khz = static_cast<std::uint16_t>(
      preferences.getUInt("srel1", context.solar_thresholds.release_1khz));
  context.solar_thresholds.detect_10khz = static_cast<std::uint16_t>(
      preferences.getUInt("sdet10", context.solar_thresholds.detect_10khz));
  context.solar_thresholds.release_10khz = static_cast<std::uint16_t>(
      preferences.getUInt("srel10", context.solar_thresholds.release_10khz));
  context.solar_config.confirmation_time_ms =
      preferences.getUInt("scfm", context.solar_config.confirmation_time_ms);
  context.solar_config.filter_alpha =
      preferences.getFloat("salpha", context.solar_config.filter_alpha);
  context.solar_config.ignore_after_start_ms =
      preferences.getUInt("signore",
                          context.solar_config.ignore_after_start_ms);
  context.solar_config.search_timeout_ms =
      preferences.getUInt("stimeout", context.solar_config.search_timeout_ms);
  context.solar_speed_config.start_base_duty =
      preferences.getFloat("sstart",
                           context.solar_speed_config.start_base_duty);
  context.solar_speed_config.slow_after_ms =
      preferences.getUInt("sslowms",
                          context.solar_speed_config.slow_after_ms);
  context.solar_speed_config.slow_base_duty =
      preferences.getFloat("sslow",
                           context.solar_speed_config.slow_base_duty);
  context.solar_contact_config.timeout_ms =
      preferences.getUInt("sctmo",
                          context.solar_contact_config.timeout_ms);
  context.solar_contact_config.initial_strafe_right_duty =
      preferences.getFloat(
          "si_str_d", context.imu_heading_hold_config.maximum_strafe_duty);
  context.solar_contact_config.retry_strafe_left_duty =
      preferences.getFloat(
          "sl_str_d", context.imu_heading_hold_config.maximum_strafe_duty);
  context.solar_contact_config.retry_strafe_right_duty =
      preferences.getFloat(
          "sr_str_d", context.imu_heading_hold_config.maximum_strafe_duty);
  context.solar_contact_config.strafe_start_delay_ms =
      preferences.getUInt(
          "sdelay",
          context.solar_contact_config.strafe_start_delay_ms);
  context.solar_contact_config.retry_forward_duty =
      preferences.getFloat("sstrfd",
                           context.solar_contact_config.retry_forward_duty);
  context.solar_contact_config.retry_strafe_left_duration_ms =
      preferences.getUInt(
          "srleft",
          context.solar_contact_config.retry_strafe_left_duration_ms);
  context.solar_contact_config.retry_forward_duration_ms =
      preferences.getUInt(
          "srfwd", context.solar_contact_config.retry_forward_duration_ms);
  context.solar_contact_config.retry_strafe_timeout_ms =
      preferences.getUInt(
          "srtmo", context.solar_contact_config.retry_strafe_timeout_ms);
  context.solar_contact_config.post_contact_forward_duration_ms =
      preferences.getUInt(
          "spcfwd",
          context.solar_contact_config.post_contact_forward_duration_ms);
  context.solar_contact_config.post_contact_forward_duty =
      preferences.getFloat(
          "spcfduty",
          context.solar_contact_config.post_contact_forward_duty);
  context.solar_contact_config.post_contact_forward_start_delay_ms =
      preferences.getUInt(
          "sfdly",
          context.solar_contact_config.post_contact_forward_start_delay_ms);
  context.solar_contact_config.line_reacquire_strafe_start_delay_ms =
      preferences.getUInt(
          "slfdly",
          context.solar_contact_config.line_reacquire_strafe_start_delay_ms);
  context.solar_contact_config.line_reacquire_strafe_duty =
      preferences.getFloat(
          "slstrd",
          preferences.getFloat(
              "slapp",
              context.imu_heading_hold_config.maximum_strafe_duty));
  context.solar_contact_config.rear_line_follow_duration_ms =
      preferences.getUInt(
          "slpidms",
          context.solar_contact_config.rear_line_follow_duration_ms);
  context.solar_hook_servo_settings.open_angle_deg =
      preferences.getInt(
          "shopen",
          context.solar_hook_servo_settings.open_angle_deg);
  context.solar_hook_servo_settings.closed_angle_deg =
      preferences.getInt(
          "shclosed",
          context.solar_hook_servo_settings.closed_angle_deg);
  if (!solarHookAngleSettingValid(
          context.solar_hook_servo_settings.open_angle_deg) ||
      !solarHookAngleSettingValid(
          context.solar_hook_servo_settings.closed_angle_deg)) {
    context.solar_hook_servo_settings = {};
  }

  ClawServoSettings claw_settings = claws.settings();
  constexpr const char* kOpenPreferenceKeys[kClawServoCount] = {
      "c1open", "c2open", "c3open"};
  constexpr const char* kClosedPreferenceKeys[kClawServoCount] = {
      "c1closed", "c2closed", "c3closed"};
  constexpr const char* kLegacyStartPreferenceKeys[kClawServoCount] = {
      "c1start", "c2start", "c3start"};
  constexpr const char* kLegacyDirectionPreferenceKeys[kClawServoCount] = {
      "c1dir", "c2dir", "c3dir"};
  for (std::size_t index = 0U; index < kClawServoCount; ++index) {
    const int legacy_closed_angle_deg = preferences.getInt(
        kLegacyStartPreferenceKeys[index], kClawServoUnsetAngleDeg);
    const int legacy_direction =
        preferences.getInt(kLegacyDirectionPreferenceKeys[index], 1) < 0 ? -1
                                                                         : 1;
    const int legacy_open_angle_deg =
        legacy_closed_angle_deg == kClawServoUnsetAngleDeg
            ? kClawServoUnsetAngleDeg
            : legacy_closed_angle_deg +
                  (legacy_direction * kLegacyClawServoRotationDeg);
    const int open_fallback =
        legacy_open_angle_deg == kClawServoUnsetAngleDeg
            ? claw_settings.open_angle_deg[index]
            : legacy_open_angle_deg;
    const int closed_fallback =
        legacy_closed_angle_deg == kClawServoUnsetAngleDeg
            ? claw_settings.closed_angle_deg[index]
            : legacy_closed_angle_deg;
    claw_settings.open_angle_deg[index] = preferences.getInt(
        kOpenPreferenceKeys[index], open_fallback);
    claw_settings.closed_angle_deg[index] = preferences.getInt(
        kClosedPreferenceKeys[index], closed_fallback);
  }
  claw_settings.open_angle_deg[kWinchServoIndex] = preferences.getInt(
      "wopen", claw_settings.open_angle_deg[kWinchServoIndex]);
  claw_settings.closed_angle_deg[kWinchServoIndex] = preferences.getInt(
      "wclosed", claw_settings.closed_angle_deg[kWinchServoIndex]);
  claw_settings.open_angle_deg[kHabitatPusherServoIndex] =
      preferences.getInt(
          "pushopen",
          claw_settings.open_angle_deg[kHabitatPusherServoIndex]);
  claw_settings.closed_angle_deg[kHabitatPusherServoIndex] =
      preferences.getInt(
          "pushclose",
          claw_settings.closed_angle_deg[kHabitatPusherServoIndex]);
  claws.applySettings(claw_settings);

  const robot::CommandValidationResult validation =
      robot::validateLineFollowerConfig(context.config, hardwareDutyCap());
  if (!validation.accepted) {
    const float cap = hardwareDutyCap();
    context.config = {};
    context.config.maxDuty = cap;
    context.config.maxCorrection =
        clampFloat(context.config.maxCorrection, 0.0F, cap);
  }
  if (!std::isfinite(context.habitat_pieces_config.line_follow_duty) ||
      context.habitat_pieces_config.line_follow_duty <= 0.0F ||
      context.habitat_pieces_config.line_follow_duty >
          activeMotionDutyCap(context)) {
    context.habitat_pieces_config.line_follow_duty =
        robot::kDefaultHabitatPiecesLineFollowDuty <=
                activeMotionDutyCap(context)
            ? robot::kDefaultHabitatPiecesLineFollowDuty
            : 0.0F;
  }
  const robot::CommandValidationResult rear_validation =
      robot::validateLineFollowerConfig(context.rear_config,
                                        hardwareDutyCap());
  if (!rear_validation.accepted) {
    context.rear_config = context.config;
    context.rear_config.baseDuty = std::fabs(context.rear_config.baseDuty);
  }
  if (!solarThresholdsValid(context.solar_thresholds) ||
      !robot::solarPanelAutonomyConfigValid(
          activeSolarPanelConfig(context, kIrBeaconFrequency1Khz)) ||
      !robot::solarPanelAutonomyConfigValid(
          activeSolarPanelConfig(context, kIrBeaconFrequency10Khz)) ||
      !solarSpeedConfigValid(context.solar_speed_config) ||
      !robot::solarPanelContactConfigValid(context.solar_contact_config)) {
    context.solar_config = kSolarPanelAutonomyConfig;
    context.solar_thresholds = {};
    context.solar_speed_config = {};
    context.solar_contact_config = kSolarPanelContactConfig;
  }
  if (!robot::timeTrialConfigValid(context.time_trial_config,
                                   rearMotionDutyCap(context))) {
    context.time_trial_config = {};
  }
}

void motionControlTask(void* parameters) {
  (void)parameters;

  RuntimeContext context{};
  const float cap = hardwareDutyCap();
  context.config.maxDuty = cap;
  context.config.maxCorrection =
      clampFloat(context.config.maxCorrection, 0.0F, cap);
  context.rear_config = context.config;
  context.rear_config.baseDuty = std::fabs(context.config.baseDuty);

  DigitalFrontLineSensorReader line_sensor_reader{
      robot::esp2::kHardwareConfig.pins};
  DigitalActiveHighLimitSwitch peg_finder_funnel_limit{
      robot::esp2::kPins.limit_switch_funnel_left};
  DualPwmMotorOutput front_left_motor{
      robot::esp2::kHardwareConfig.front_left_motor};
  DualPwmMotorOutput front_right_motor{
      robot::esp2::kHardwareConfig.front_right_motor};
  RearCommandLink rear_link{robot::esp2::kHardwareConfig.uart_to_esp1};
  robot::esp2::StepperAxis stepper{{robot::esp2::kPins.stepper_sleep,
      robot::esp2::kPins.stepper_dir, robot::esp2::kPins.stepper_step,
      robot::esp2::kPins.limit_switch_stepper_bottom,
      robot::esp2::kPins.limit_switch_stepper_top, 800U, 1200U, 200U,
      3U, 3U, 2000U, 500U, 15U,
      100000, 0U, 0U}};  // Initial vertical-axis software maximum, adjustable from the dashboard.
  ClawServoBank claws{robot::esp2::kHardwareConfig.servo_claw_1,
                      robot::esp2::kHardwareConfig.servo_claw_2,
                      robot::esp2::kHardwareConfig.servo_claw_3,
                      robot::esp2::kHardwareConfig.servo_habitat_pusher,
                      robot::esp2::kHardwareConfig.servo_winch};
  static robot::esp2::ImuAcquisitionService imu_acquisition{};
  Preferences preferences{};

  line_sensor_reader.initialize();
  peg_finder_funnel_limit.initialize();
  front_left_motor.initializeDisabled();
  front_right_motor.initializeDisabled();
  rear_link.initialize();
  stepper.begin();
  claws.initializeDisabled();

  // An ESP2-only reboot can occur while ESP1 still holds a fresh rear-wheel or
  // funnel command. Send disabled commands twice, separated by two ESP1 drive
  // task periods, before measuring a stationary gyro bias. Front motors and
  // ESP2 mechanisms have already been initialized disabled above.
  robot::Milliseconds startup_now_ms =
      static_cast<robot::Milliseconds>(millis());
  (void)sendStoppedRearCommand(rear_link, context.config, startup_now_ms);
  (void)sendStoppedFunnelCommand(rear_link, context.config, startup_now_ms);
  (void)sendStoppedSolarHookCommand(rear_link);
  delay(kPreCalibrationStopSettleMs);
  startup_now_ms = static_cast<robot::Milliseconds>(millis());
  (void)sendStoppedRearCommand(rear_link, context.config, startup_now_ms);
  (void)sendStoppedFunnelCommand(rear_link, context.config, startup_now_ms);
  (void)sendStoppedSolarHookCommand(rear_link);

  const bool imu_ready = imu_acquisition.initialize(
      Wire, robot::esp2::kPins.imu_sda, robot::esp2::kPins.imu_scl,
      kImuI2cAddress, kImuCalibrationSampleCount);
  context.imu_shared_snapshot_available =
      imu_acquisition.latest(context.latest_imu_snapshot);
  context.imu_shared_snapshot_fetched_at_us = micros();
  const robot::esp2::ImuState& startup_imu_state =
      context.latest_imu_snapshot.state;
  char imu_event_message[robot::kEventMessageSize]{};
  if (startup_imu_state.initialized) {
    std::snprintf(imu_event_message, sizeof(imu_event_message),
                  "IMU initialized: WHO_AM_I=0x%02X",
                  static_cast<unsigned>(startup_imu_state.who_am_i));
    logEvent(context, static_cast<robot::Milliseconds>(millis()),
             robot::EventSeverity::Info, robot::EventSource::System,
             imu_event_message);
    if (startup_imu_state.calibrated) {
      logEvent(context, static_cast<robot::Milliseconds>(millis()),
               robot::EventSeverity::Info, robot::EventSource::System,
               "IMU gyro Z calibration succeeded");
    } else {
      logEvent(context, static_cast<robot::Milliseconds>(millis()),
             robot::EventSeverity::Warn, robot::EventSource::System,
             "IMU gyro Z calibration failed");
    }
  } else {
    if (!startup_imu_state.configured) {
      std::snprintf(imu_event_message, sizeof(imu_event_message),
                    "IMU not configured: SDA/SCL unassigned");
    } else {
      std::snprintf(imu_event_message, sizeof(imu_event_message),
                    "IMU init failed: %s id=0x%02X wire=%d",
                    robot::esp2::imuInitializationErrorName(
                        startup_imu_state.initialization_error),
                    static_cast<unsigned>(startup_imu_state.who_am_i),
                    startup_imu_state.last_wire_status);
    }
    logEvent(context, static_cast<robot::Milliseconds>(millis()),
             robot::EventSeverity::Warn, robot::EventSource::System,
             imu_event_message);
  }
  const bool imu_acquisition_started =
      imu_ready &&
      imu_acquisition.start(kDefaultMotionTaskPeriodMs,
                            kSensorAcquisitionTaskPriority,
                            kSensorAcquisitionTaskCore);
  if (imu_ready && !imu_acquisition_started) {
    logEvent(context, static_cast<robot::Milliseconds>(millis()),
             robot::EventSeverity::Warn, robot::EventSource::System,
             "IMU sensor acquisition task failed to start");
  }
  context.imu_shared_snapshot_available =
      imu_acquisition.latest(context.latest_imu_snapshot);
  context.imu_shared_snapshot_fetched_at_us = micros();

  preferences.begin("telemetry", false);
  loadPreferences(preferences, context, front_left_motor, front_right_motor,
                  claws);
  resetSolarPanelAutonomy(context, static_cast<robot::Milliseconds>(millis()));
  resetHabitatPieces(context, static_cast<robot::Milliseconds>(millis()));
  resetHabitatPlacement(context,
                        static_cast<robot::Milliseconds>(millis()));
  resetTowerPieces(context, static_cast<robot::Milliseconds>(millis()));
  resetPegFinder(context, static_cast<robot::Milliseconds>(millis()));
  resetTimeTrial(context, static_cast<robot::Milliseconds>(millis()));
  resetImuTurn(context);

  g_runtime = {&context, &line_sensor_reader, &peg_finder_funnel_limit,
               &front_left_motor, &front_right_motor, &rear_link, &claws,
               &stepper, &imu_acquisition, &preferences};

  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  setupWebHandlers();
  g_server.begin();
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::System,
           "telemetry web server started");
  resetMotionDiagnosticCapture(
      context, static_cast<robot::Milliseconds>(millis()));

  TickType_t last_wake_tick = xTaskGetTickCount();
  std::uint32_t previous_loop_started_us = 0U;

  for (;;) {
    const std::uint32_t loop_started_us = micros();
    context.diagnostic_loop_started_us = loop_started_us;
    context.diagnostic_loop_interval_us =
        previous_loop_started_us == 0U
            ? 0U
            : static_cast<std::uint32_t>(
                  loop_started_us - previous_loop_started_us);
    previous_loop_started_us = loop_started_us;
    const robot::Milliseconds now_ms =
        static_cast<robot::Milliseconds>(millis());
    context.imu_shared_snapshot_available =
        imu_acquisition.latest(context.latest_imu_snapshot);
    context.imu_shared_snapshot_fetched_at_us = micros();
    if (context.imu_shared_snapshot_available &&
        context.latest_imu_snapshot
                .last_successful_sample_published_at_us != 0U) {
      const std::uint32_t observed_publication_gap_us =
          context.imu_shared_snapshot_fetched_at_us -
          context.latest_imu_snapshot
              .last_successful_sample_published_at_us;
      if (observed_publication_gap_us >
          context.maximum_observed_imu_publication_gap_us) {
        context.maximum_observed_imu_publication_gap_us =
            observed_publication_gap_us;
      }
    }
    context.diagnostic_imu_update_us =
        context.latest_imu_snapshot.acquisition_duration_us;
    if (context.imu_heading_reset_pending_sequence != 0U &&
        context.latest_imu_snapshot.last_heading_reset_sequence ==
            context.imu_heading_reset_pending_sequence) {
      context.imu_heading_reset_pending_sequence = 0U;
      logEvent(context, now_ms, robot::EventSeverity::Info,
               robot::EventSource::System,
               "IMU relative heading reset applied");
    }
    recordImuAvailabilityDiagnostics(context, now_ms);
    const std::uint32_t web_started_us = micros();
    g_server.handleClient();
    context.diagnostic_web_handle_us =
        static_cast<std::uint32_t>(micros() - web_started_us);
    stepper.update();
    peg_finder_funnel_limit.update();
    rear_link.pollReceive(now_ms);
    refreshLineObservation(context, line_sensor_reader, now_ms);
    refreshRearLineObservation(context, rear_link, now_ms);
    pollSerialCommands(context, line_sensor_reader, front_left_motor,
                       front_right_motor, rear_link, now_ms);

    if (context.requested_funnel_command.enabled &&
        context.requested_funnel_command.expires_at_ms != 0U &&
        now_ms >= context.requested_funnel_command.expires_at_ms) {
      context.requested_funnel_command = robot::disabledMotorCommand();
      sendStoppedFunnelCommand(rear_link, context.config, now_ms);
      logEvent(context, now_ms, robot::EventSeverity::Warn,
               robot::EventSource::System, "funnel command deadman expired");
    }

    const bool timed_mode_expired =
        context.command_deadman_armed && context.mode_expires_at_ms != 0U &&
        now_ms >= context.mode_expires_at_ms;
    const bool stale_command =
        context.command_deadman_armed &&
        elapsedSince(now_ms, context.last_command_ms) > kCommandTimeoutMs &&
        (context.modes.currentMode() ==
             robot::RobotTestMode::SingleMotorTest ||
         context.modes.currentMode() ==
             robot::RobotTestMode::ManualDriveTest ||
         context.modes.currentMode() ==
             robot::RobotTestMode::DistributedDriveTest ||
         context.modes.currentMode() ==
             robot::RobotTestMode::ImuStrafeTest);
    if (timed_mode_expired || stale_command) {
      const robot::Milliseconds diagnostic_now_ms =
          static_cast<robot::Milliseconds>(millis());
      recordMotionDiagnostic(
          context, front_left_motor, front_right_motor, rear_link,
          &context.latest_imu_snapshot,
          robot::MotionDiagnosticEvent::CommandDeadmanExpired,
          diagnostic_now_ms);
      if (context.modes.currentMode() ==
          robot::RobotTestMode::ImuStrafeTest) {
        robot::stopImuHeadingHold(context.imu_heading_hold_state);
        context.last_imu_heading_hold_update = {};
        context.last_imu_heading_hold_update.state =
            context.imu_heading_hold_state.state;
      }
      disableActuators(context, front_left_motor, front_right_motor, rear_link,
                       now_ms);
      recordMotionDiagnostic(
          context, front_left_motor, front_right_motor, rear_link,
          &context.latest_imu_snapshot,
          robot::MotionDiagnosticEvent::OutputsDisabled,
          diagnostic_now_ms);
      scheduleMotionDiagnosticFreeze(
          context,
          context.motion_diagnostics.driveAfterStopCount() > 0U
              ? robot::MotionDiagnosticTrigger::DriveHeartbeatAfterStop
              : robot::MotionDiagnosticTrigger::CommandDeadmanExpired);
      logEvent(context, now_ms, robot::EventSeverity::Warn,
               robot::EventSource::System,
               stale_command ? "command deadman expired"
                             : "timed test auto-stopped");
    }

    const robot::RobotTestMode mode = context.modes.currentMode();
    if (mode != robot::RobotTestMode::AutonomousSolarPanel &&
        mode != robot::RobotTestMode::TimeTrial &&
        context.autonomous_state !=
            robot::SolarPanelAutonomyState::WaitForStart) {
      resetSolarPanelAutonomy(context, now_ms);
    }
    if (mode != robot::RobotTestMode::HabitatPieces &&
        (context.habitat_pieces.state !=
             robot::HabitatPiecesState::WaitForStart ||
         context.habitat_pieces_start_requested)) {
      resetHabitatPieces(context, now_ms);
    }
    if (mode != robot::RobotTestMode::HabitatPlacement &&
        (context.habitat_placement.state !=
             robot::HabitatPlacementState::WaitForStart ||
         context.habitat_placement_start_requested)) {
      resetHabitatPlacement(context, now_ms);
    }
    if (mode != robot::RobotTestMode::AutonomousTowerPieces &&
        mode != robot::RobotTestMode::TimeTrial &&
        context.tower_pieces.state !=
            robot::TowerPiecesState::WaitForStart) {
      resetTowerPieces(context, now_ms);
    }
    if (mode != robot::RobotTestMode::PegFinder &&
        mode != robot::RobotTestMode::TimeTrial &&
        context.peg_finder.state != robot::PegFinderState::WaitForStart) {
      resetPegFinder(context, now_ms);
    }
    if (mode != robot::RobotTestMode::TimeTrial &&
        context.time_trial.state != robot::TimeTrialState::WaitForStart) {
      resetTimeTrial(context, now_ms);
    }
    if (mode != robot::RobotTestMode::ImuTurnTest &&
        mode != robot::RobotTestMode::AutonomousTowerPieces &&
        mode != robot::RobotTestMode::HabitatPlacement &&
        mode != robot::RobotTestMode::PegFinder &&
        mode != robot::RobotTestMode::TimeTrial &&
        context.imu_turn_state.state != robot::ImuTurnState::Idle) {
      resetImuTurn(context);
    }
    if (mode != robot::RobotTestMode::ImuStrafeTest &&
        mode != robot::RobotTestMode::AutonomousSolarPanel &&
        mode != robot::RobotTestMode::AutonomousTowerPieces &&
        mode != robot::RobotTestMode::TimeTrial &&
        context.imu_heading_hold_state.state !=
            robot::ImuHeadingHoldState::Idle) {
      resetImuHeadingHold(context);
    }
    if (robot::robotTestModeIsSensorOnly(mode)) {
      disableActuators(context, front_left_motor, front_right_motor, rear_link,
                       now_ms);
      if (mode == robot::RobotTestMode::SensorMonitor ||
          mode == robot::RobotTestMode::LineSensorTest) {
        runSensorOnlyTelemetry(context, line_sensor_reader, now_ms);
      } else if (mode == robot::RobotTestMode::RearLineSensorTest) {
        runRearSensorOnlyTelemetry(context, rear_link, now_ms);
      }
    } else if (mode == robot::RobotTestMode::ImuTurnTest) {
      runImuTurnTest(context, front_left_motor, front_right_motor,
                     rear_link, imu_acquisition, now_ms);
    } else if (mode == robot::RobotTestMode::ImuStrafeTest) {
      runImuHeadingHoldTest(
          context, front_left_motor, front_right_motor, rear_link,
          imu_acquisition, now_ms);
    } else if (mode == robot::RobotTestMode::AutonomousSolarPanel) {
      runSolarPanelAutonomy(context, line_sensor_reader, front_left_motor,
                            front_right_motor, rear_link, now_ms);
    } else if (mode == robot::RobotTestMode::HabitatPieces) {
      runHabitatPieces(context, line_sensor_reader, front_left_motor,
                       front_right_motor, rear_link, stepper, now_ms);
    } else if (mode == robot::RobotTestMode::HabitatPlacement) {
      runHabitatPlacement(context, line_sensor_reader, front_left_motor,
                          front_right_motor, rear_link, claws, stepper,
                          now_ms);
    } else if (mode == robot::RobotTestMode::AutonomousTowerPieces) {
      runTowerPiecesAutonomy(context, front_left_motor, front_right_motor,
                             rear_link, claws, stepper, now_ms);
    } else if (mode == robot::RobotTestMode::PegFinder) {
      runPegFinder(context, front_left_motor, front_right_motor, rear_link,
                   claws, peg_finder_funnel_limit, now_ms);
    } else if (mode == robot::RobotTestMode::TimeTrial) {
      runTimeTrial(context, line_sensor_reader, front_left_motor,
                   front_right_motor, rear_link, claws,
                   peg_finder_funnel_limit, stepper, now_ms);
    } else if (mode == robot::RobotTestMode::LineFollowTest &&
               context.follower_state.enabled) {
      const bool left_black = context.last_line_observation.left_black;
      const bool right_black = context.last_line_observation.right_black;
      if (!line_sensor_reader.configured() || !rear_link.configured()) {
        disableActuators(context, front_left_motor, front_right_motor,
                         rear_link, now_ms);
        setFault(context, robot::FaultCode::HardwareNotConfigured,
                 "line follower stopped: sensor or rear link invalid");
        logEvent(context, now_ms, robot::EventSeverity::Fault,
                 robot::EventSource::Line,
                 "line follower stopped: sensor or rear link invalid");
      } else {
        context.last_update = robot::updateLineFollower(
            context.follower_state, left_black, right_black, context.config,
            now_ms);
        applyWheelCommand(context, front_left_motor, front_right_motor,
                          rear_link, context.last_update.wheel_command,
                          context.config, now_ms);
        if (!context.follower_state.enabled &&
            !context.last_update.observation.safe_to_drive) {
          disableActuators(context, front_left_motor, front_right_motor,
                           rear_link, now_ms);
          setFault(context, robot::FaultCode::InvalidCommand,
                   "line follower stopped: line lost without history");
          logEvent(context, now_ms, robot::EventSeverity::Fault,
                   robot::EventSource::Line,
                   "line follower stopped: line lost without history");
        } else if (!rear_link.remoteStatusFresh(
                now_ms, remoteStatusTimeoutMs(context.config))) {
          disableActuators(context, front_left_motor, front_right_motor,
                           rear_link, now_ms);
          setFault(context, robot::FaultCode::CommunicationStale,
                   "line follower stopped: rear link unhealthy");
          logEvent(context, now_ms, robot::EventSeverity::Fault,
                   robot::EventSource::Uart,
                   "line follower stopped: rear link unhealthy");
        }
        printTelemetry(context, line_sensor_reader, rear_link, now_ms);
      }
    } else if (mode == robot::RobotTestMode::RearLineFollowTest &&
               context.follower_state.enabled) {
      const bool rear_sensor_fresh = rear_link.rearLineSnapshotFresh(
          now_ms, context.rear_config.remoteCommandTimeoutMs);
      if (!rear_sensor_fresh) {
        disableActuators(context, front_left_motor, front_right_motor,
                         rear_link, now_ms);
        setFault(context, robot::FaultCode::CommunicationStale,
                 "rear line follower stopped: sensor data stale");
        logEvent(context, now_ms, robot::EventSeverity::Fault,
                 robot::EventSource::Uart,
                 "rear line follower stopped: sensor data stale");
      } else if (!rear_link.latestRearLineSnapshot().configured ||
                 !rear_link.configured()) {
        disableActuators(context, front_left_motor, front_right_motor,
                         rear_link, now_ms);
        setFault(context, robot::FaultCode::HardwareNotConfigured,
                 "rear line follower stopped: sensor or link invalid");
        logEvent(context, now_ms, robot::EventSeverity::Fault,
                 robot::EventSource::Line,
                 "rear line follower stopped: sensor or link invalid");
      } else {
        const bool left_black =
            context.last_rear_line_observation.left_black;
        const bool right_black =
            context.last_rear_line_observation.right_black;
        const robot::LineFollowerConfig reverse_config =
            reverseRearLineFollowerConfig(context);
        context.last_rear_update = robot::updateLineFollower(
            context.follower_state, left_black, right_black, reverse_config,
            now_ms);
        applyWheelCommand(context, front_left_motor, front_right_motor,
                          rear_link,
                          context.last_rear_update.wheel_command,
                          reverse_config, now_ms);
        if (!context.follower_state.enabled &&
            !context.last_rear_update.observation.safe_to_drive) {
          disableActuators(context, front_left_motor, front_right_motor,
                           rear_link, now_ms);
          setFault(context, robot::FaultCode::InvalidCommand,
                   "rear line follower stopped: line lost without history");
          logEvent(
              context, now_ms, robot::EventSeverity::Fault,
              robot::EventSource::Line,
              "rear line follower stopped: line lost without history");
        } else if (!rear_link.remoteStatusFresh(
                       now_ms, remoteStatusTimeoutMs(context.rear_config))) {
          disableActuators(context, front_left_motor, front_right_motor,
                           rear_link, now_ms);
          setFault(context, robot::FaultCode::CommunicationStale,
                   "rear line follower stopped: ESP1 status stale");
          logEvent(context, now_ms, robot::EventSeverity::Fault,
                   robot::EventSource::Uart,
                   "rear line follower stopped: ESP1 status stale");
        }
        printRearTelemetry(context, rear_link, now_ms);
      }
    } else if (mode == robot::RobotTestMode::MechanismTest) {
      disableMotionActuators(context, front_left_motor, front_right_motor,
                             rear_link, now_ms);
    } else if (mode == robot::RobotTestMode::SingleMotorTest ||
               mode == robot::RobotTestMode::ManualDriveTest ||
               mode == robot::RobotTestMode::DistributedDriveTest) {
      applyWheelCommand(context, front_left_motor, front_right_motor, rear_link,
                        context.requested_command, context.config, now_ms);
    } else {
      disableActuators(context, front_left_motor, front_right_motor, rear_link,
                       now_ms);
    }

    const bool time_trial_rear_phase =
        mode == robot::RobotTestMode::TimeTrial &&
        context.time_trial.state != robot::TimeTrialState::WaitForStart &&
        context.time_trial.state !=
            robot::TimeTrialState::AutonomousSolar;
    const bool rear_line_mode =
        mode == robot::RobotTestMode::RearLineFollowTest ||
        mode == robot::RobotTestMode::RearLineSensorTest ||
        mode == robot::RobotTestMode::AutonomousTowerPieces ||
        mode == robot::RobotTestMode::PegFinder ||
        time_trial_rear_phase;
    const robot::Milliseconds configured_period_ms =
        (mode == robot::RobotTestMode::ImuTurnTest ||
         mode == robot::RobotTestMode::ImuStrafeTest)
            ? kDefaultMotionTaskPeriodMs
            : (rear_line_mode ? context.rear_config.controlPeriodMs
                              : context.config.controlPeriodMs);
    const robot::Milliseconds effective_period_ms =
        configured_period_ms == 0U ? kDefaultMotionTaskPeriodMs
                                   : configured_period_ms;
    const std::uint32_t loop_work_us =
        static_cast<std::uint32_t>(micros() - loop_started_us);
    context.motion_diagnostics.observeLoop(
        context.diagnostic_loop_interval_us, loop_work_us,
        context.diagnostic_imu_update_us,
        context.diagnostic_web_handle_us, effective_period_ms);

    const robot::Milliseconds diagnostic_now_ms =
        static_cast<robot::Milliseconds>(millis());
    const bool diagnostic_motion_active = diagnosticMotionActive(
        context, front_left_motor, front_right_motor, rear_link);
    if (diagnostic_motion_active &&
        (context.last_motion_diagnostic_sample_ms == 0U ||
         elapsedSince(diagnostic_now_ms,
                      context.last_motion_diagnostic_sample_ms) >=
             kMotionDiagnosticSamplePeriodMs)) {
      recordMotionDiagnostic(
          context, front_left_motor, front_right_motor, rear_link,
          &context.latest_imu_snapshot,
          robot::MotionDiagnosticEvent::Periodic, diagnostic_now_ms);
      context.last_motion_diagnostic_sample_ms = diagnostic_now_ms;
    } else if (!diagnostic_motion_active &&
               context.diagnostic_motion_was_active) {
      recordMotionDiagnostic(
          context, front_left_motor, front_right_motor, rear_link,
          &context.latest_imu_snapshot,
          robot::MotionDiagnosticEvent::OutputsDisabled,
          diagnostic_now_ms);
    }
    context.diagnostic_motion_was_active = diagnostic_motion_active;

    if (context.diagnostic_freeze_pending !=
        robot::MotionDiagnosticTrigger::None) {
      context.motion_diagnostics.freeze(
          context.diagnostic_freeze_pending, diagnostic_now_ms);
      context.diagnostic_freeze_pending =
          robot::MotionDiagnosticTrigger::None;
    }
    vTaskDelayUntil(
        &last_wake_tick, pdMS_TO_TICKS(effective_period_ms));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  xTaskCreatePinnedToCore(motionControlTask, "esp2_telemetry",
                          kTaskStackBytes, nullptr, kTaskPriority, nullptr,
                          kTaskCore);
}

void loop() {
  vTaskSuspend(nullptr);
}
