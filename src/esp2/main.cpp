#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
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
#include "common/LineFollower.h"
#include "common/LineObservation.h"
#include "common/LineSensor.h"
#include "common/MotorOutput.h"
#include "common/RearDriveCommand.h"
#include "common/RobotCommandValidation.h"
#include "common/RobotTestModeManager.h"
#include "common/SolarPanelAutonomy.h"
#include "common/TelemetrySnapshot.h"
#include "common/UartProtocol.h"
#include "esp2/MechanismControllers.h"
#include "esp2/PinConfig.h"
#include "esp2/StepperAxis.h"

namespace {

constexpr const char* kApSsid = "Team14Robot";
constexpr const char* kApPassword = "robotdebug";
constexpr robot::Milliseconds kDefaultMotionTaskPeriodMs = 10U;
constexpr robot::Milliseconds kTelemetryPeriodMs = 100U;
constexpr robot::Milliseconds kCommandTimeoutMs = 700U;
constexpr robot::Milliseconds kMaxTimedTestDurationMs = 30000U;
constexpr float kSingleMotorDutyCap = 1.0F;
constexpr std::uint32_t kTaskStackBytes = 12288U;
constexpr UBaseType_t kTaskPriority = 1U;
constexpr BaseType_t kTaskCore = 1;
constexpr std::size_t kSerialCommandBufferSize = 128U;
constexpr std::size_t kJsonBufferSize = 8192U;
constexpr std::size_t kClawServoCount = 3U;
constexpr int kClawServoUnsetAngleDeg = -1;
constexpr int kClawServoRotationDeg = 90;

// Solar-panel first-stage tuning. TEMPORARY DEFAULTS: tune with the real
// beacon, sensor, lighting, and motors running before driving at speed.
constexpr std::uint32_t kIrBeaconFrequency1Khz = 1000U;
constexpr std::uint32_t kIrBeaconFrequency10Khz = 10000U;
constexpr std::uint16_t IR_BEACON_DETECT_THRESHOLD_1KHZ = 15U;
constexpr std::uint16_t IR_BEACON_RELEASE_THRESHOLD_1KHZ = 8U;
constexpr std::uint16_t IR_BEACON_DETECT_THRESHOLD_10KHZ = 10U;
constexpr std::uint16_t IR_BEACON_RELEASE_THRESHOLD_10KHZ = 6U;
constexpr robot::Milliseconds IR_BEACON_CONFIRM_TIME_MS = 50U;
constexpr float IR_FILTER_ALPHA = 0.5F;
constexpr robot::Milliseconds IR_IGNORE_AFTER_START_MS = 7000U;
constexpr robot::Milliseconds SOLAR_SEARCH_TIMEOUT_MS = 13000U;
constexpr float SOLAR_START_BASE_DUTY = 0.3F;
constexpr robot::Milliseconds SOLAR_SLOW_AFTER_MS = 7500U;
constexpr float SOLAR_SLOW_BASE_DUTY = 0.1F;
constexpr robot::Milliseconds SOLAR_CONTACT_TIMEOUT_MS =
    SOLAR_SEARCH_TIMEOUT_MS;
constexpr float SOLAR_CONTACT_STRAFE_DUTY = SOLAR_SLOW_BASE_DUTY;
constexpr robot::Milliseconds SOLAR_STRAFE_START_DELAY_MS = 300U;
// TODO(team): tune both adjustment durations on the real robot. Zero keeps the
// new motion phases disabled until values are applied through telemetry.
constexpr robot::Milliseconds SOLAR_RETRY_STRAFE_LEFT_DURATION_MS = 0U;
constexpr robot::Milliseconds SOLAR_RETRY_FORWARD_DURATION_MS = 0U;
constexpr robot::Milliseconds SOLAR_RETRY_STRAFE_TIMEOUT_MS =
    SOLAR_CONTACT_TIMEOUT_MS;
// Team wiring report: raw HIGH means the solar side switch has been hit.
constexpr bool SOLAR_LIMIT_SWITCH_HIT_WHEN_HIGH = true;
constexpr robot::SolarPanelAutonomyConfig kSolarPanelAutonomyConfig{
    IR_BEACON_DETECT_THRESHOLD_1KHZ, IR_BEACON_RELEASE_THRESHOLD_1KHZ,
    IR_BEACON_CONFIRM_TIME_MS, IR_FILTER_ALPHA, IR_IGNORE_AFTER_START_MS,
    SOLAR_SEARCH_TIMEOUT_MS};
constexpr robot::SolarPanelContactConfig kSolarPanelContactConfig{
    SOLAR_CONTACT_TIMEOUT_MS,
    SOLAR_CONTACT_STRAFE_DUTY,
    SOLAR_STRAFE_START_DELAY_MS,
    SOLAR_RETRY_STRAFE_LEFT_DURATION_MS,
    SOLAR_RETRY_FORWARD_DURATION_MS,
    SOLAR_RETRY_STRAFE_TIMEOUT_MS};

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
static_assert(SOLAR_CONTACT_STRAFE_DUTY >= 0.0F &&
                  SOLAR_CONTACT_STRAFE_DUTY <= 1.0F,
              "solar contact strafe duty must be in [0, 1]");
static_assert(SOLAR_STRAFE_START_DELAY_MS > 0U,
              "solar strafe start delay must be nonzero");
static_assert(SOLAR_RETRY_STRAFE_TIMEOUT_MS > 0U,
              "solar retry strafe timeout must be nonzero");

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
  if (!gpioAssigned(config.gpio) || config.pwm_channel < 0 ||
      config.pwm_frequency_hz == 0U || config.pwm_resolution_bits == 0U ||
      config.pwm_resolution_bits >= 31U ||
      config.minimum_pulse_us == 0U ||
      config.maximum_pulse_us <= config.minimum_pulse_us) {
    return false;
  }

  const std::uint32_t period_us = 1000000UL / config.pwm_frequency_hz;
  return period_us > config.maximum_pulse_us;
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
        robot::Esp1StatusReport report{};
        if (robot::decodeEsp1StatusPacket(packet, report)) {
          latest_status_ = report;
          last_status_received_at_ms_ = now_ms;
          status_available_ = true;
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

 private:
  const robot::esp2::UartConfig& config_;
  robot::UartFrameParser parser_{};
  std::uint16_t next_sequence_{0};
  std::uint16_t last_rear_sequence_sent_{0};
  bool configured_{false};
  bool healthy_{false};
  bool status_available_{false};
  robot::Milliseconds last_rear_sent_at_ms_{0};
  robot::Milliseconds last_status_received_at_ms_{0};
  std::uint32_t packet_error_count_{0};
  robot::Esp1StatusReport latest_status_{};
};

enum class ClawServoPositionRequest : std::uint8_t {
  Closed = 0,
  Open = 1,
};

enum class ClawServoCommandResult : std::uint8_t {
  Accepted = 0,
  InvalidClaw = 1,
  HardwareUnconfigured = 2,
  StartAngleUnset = 3,
  StartAngleOutOfRange = 4,
  DirectionInvalid = 5,
  OpenAngleOutOfRange = 6,
};

struct ClawServoSettings {
  std::array<int, kClawServoCount> start_angle_deg{
      {kClawServoUnsetAngleDeg, kClawServoUnsetAngleDeg,
       kClawServoUnsetAngleDeg}};
  std::array<int, kClawServoCount> open_direction{{1, 1, 1}};
};

const char* clawServoResultReason(const ClawServoCommandResult result) {
  switch (result) {
    case ClawServoCommandResult::Accepted:
      return "claw command accepted";
    case ClawServoCommandResult::InvalidClaw:
      return "invalid claw id";
    case ClawServoCommandResult::HardwareUnconfigured:
      return "claw PWM hardware is not configured";
    case ClawServoCommandResult::StartAngleUnset:
      return "claw start angle is not set";
    case ClawServoCommandResult::StartAngleOutOfRange:
      return "claw start angle must be 0..180 degrees";
    case ClawServoCommandResult::DirectionInvalid:
      return "claw direction must be 1 or -1";
    case ClawServoCommandResult::OpenAngleOutOfRange:
      return "claw open angle must stay within 0..180 degrees";
  }
  return "claw command rejected";
}

class ClawServoBank {
 public:
  ClawServoBank(const robot::esp2::ServoOutputConfig& claw_1,
                const robot::esp2::ServoOutputConfig& claw_2,
                const robot::esp2::ServoOutputConfig& claw_3)
      : configs_{{&claw_1, &claw_2, &claw_3}} {}

  void initializeDisabled() {
    for (std::size_t index = 0U; index < kClawServoCount; ++index) {
      hardware_configured_[index] =
          servoOutputConfigComplete(*configs_[index]);
      if (hardware_configured_[index]) {
        ledcSetup(configs_[index]->pwm_channel,
                  configs_[index]->pwm_frequency_hz,
                  configs_[index]->pwm_resolution_bits);
        pinMode(configs_[index]->gpio, INPUT);
      }
    }
    disable();
  }

  void disable() {
    for (std::size_t index = 0U; index < kClawServoCount; ++index) {
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
    return ClawServoCommandResult::Accepted;
  }

  const ClawServoSettings& settings() const { return settings_; }

  ClawServoCommandResult command(
      const std::size_t index,
      const ClawServoPositionRequest request) {
    int target_angle_deg = kClawServoUnsetAngleDeg;
    const ClawServoCommandResult result =
        targetAngle(index, request, target_angle_deg);
    if (result != ClawServoCommandResult::Accepted) {
      return result;
    }
    writeAngle(index, target_angle_deg);
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
      writeAngle(index, target_angles_deg[index]);
      commanded_open_[index] = request == ClawServoPositionRequest::Open;
    }
    return ClawServoCommandResult::Accepted;
  }

  void fillTelemetry(robot::ServoClawBankTelemetry& output) const {
    output = {};
    output.rotation_deg = kClawServoRotationDeg;
    fillClawTelemetry(output.claw_1, 0U);
    fillClawTelemetry(output.claw_2, 1U);
    fillClawTelemetry(output.claw_3, 2U);
  }

 private:
  static bool angleConfigured(const int angle_deg) {
    return angle_deg != kClawServoUnsetAngleDeg;
  }

  static bool angleInRange(const int angle_deg) {
    return angle_deg >= 0 && angle_deg <= 180;
  }

  static bool directionValid(const int direction) {
    return direction == 1 || direction == -1;
  }

  static int openAngle(const int start_angle_deg, const int direction) {
    return start_angle_deg + (direction * kClawServoRotationDeg);
  }

  static ClawServoCommandResult validateSettings(
      const ClawServoSettings& settings) {
    for (std::size_t index = 0U; index < kClawServoCount; ++index) {
      const int start_angle_deg = settings.start_angle_deg[index];
      const int direction = settings.open_direction[index];
      if (!directionValid(direction)) {
        return ClawServoCommandResult::DirectionInvalid;
      }
      if (!angleConfigured(start_angle_deg)) {
        continue;
      }
      if (!angleInRange(start_angle_deg)) {
        return ClawServoCommandResult::StartAngleOutOfRange;
      }
      if (!angleInRange(openAngle(start_angle_deg, direction))) {
        return ClawServoCommandResult::OpenAngleOutOfRange;
      }
    }
    return ClawServoCommandResult::Accepted;
  }

  ClawServoCommandResult targetAngle(
      const std::size_t index, const ClawServoPositionRequest request,
      int& target_angle_deg) const {
    if (index >= kClawServoCount) {
      return ClawServoCommandResult::InvalidClaw;
    }
    if (!hardware_configured_[index]) {
      return ClawServoCommandResult::HardwareUnconfigured;
    }
    const int start_angle_deg = settings_.start_angle_deg[index];
    if (!angleConfigured(start_angle_deg)) {
      return ClawServoCommandResult::StartAngleUnset;
    }
    target_angle_deg =
        request == ClawServoPositionRequest::Closed
            ? start_angle_deg
            : openAngle(start_angle_deg, settings_.open_direction[index]);
    if (!angleInRange(target_angle_deg)) {
      return ClawServoCommandResult::OpenAngleOutOfRange;
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
    return ((pulse_us * pwmMaxDuty(config.pwm_resolution_bits)) +
            (period_us / 2U)) /
           period_us;
  }

  void writeAngle(const std::size_t index, const int angle_deg) {
    const robot::esp2::ServoOutputConfig& config = *configs_[index];
    ledcSetup(config.pwm_channel, config.pwm_frequency_hz,
              config.pwm_resolution_bits);
    ledcAttachPin(config.gpio, config.pwm_channel);
    ledcWrite(config.pwm_channel,
              dutyForPulseUs(index, pulseUsForAngle(index, angle_deg)));
    output_enabled_[index] = true;
    commanded_angle_deg_[index] = angle_deg;
  }

  void disableOutput(const std::size_t index) {
    if (index >= kClawServoCount) {
      return;
    }
    if (hardware_configured_[index]) {
      ledcWrite(configs_[index]->pwm_channel, 0U);
      ledcDetachPin(configs_[index]->gpio);
      pinMode(configs_[index]->gpio, INPUT);
    }
    output_enabled_[index] = false;
    commanded_angle_deg_[index] = kClawServoUnsetAngleDeg;
    commanded_open_[index] = false;
  }

  void fillClawTelemetry(robot::ServoClawTelemetry& output,
                         const std::size_t index) const {
    const int start_angle_deg = settings_.start_angle_deg[index];
    output.hardware_configured = hardware_configured_[index];
    output.start_configured = angleConfigured(start_angle_deg);
    output.output_enabled = output_enabled_[index];
    output.start_angle_deg = start_angle_deg;
    output.open_angle_deg =
        output.start_configured
            ? openAngle(start_angle_deg, settings_.open_direction[index])
            : kClawServoUnsetAngleDeg;
    output.open_direction = settings_.open_direction[index];
    output.commanded_angle_deg = commanded_angle_deg_[index];
    output.commanded_open = commanded_open_[index];
  }

  std::array<const robot::esp2::ServoOutputConfig*, kClawServoCount> configs_;
  ClawServoSettings settings_{};
  std::array<bool, kClawServoCount> hardware_configured_{{false, false, false}};
  std::array<bool, kClawServoCount> output_enabled_{{false, false, false}};
  std::array<int, kClawServoCount> commanded_angle_deg_{
      {kClawServoUnsetAngleDeg, kClawServoUnsetAngleDeg,
       kClawServoUnsetAngleDeg}};
  std::array<bool, kClawServoCount> commanded_open_{{false, false, false}};
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

struct RuntimeContext {
  robot::LineFollowerConfig config{};
  robot::LineFollowerState follower_state{};
  robot::RobotTestModeManager modes{};
  robot::EventLog events{};
  robot::FourWheelCommand requested_command{};
  robot::FourWheelCommand last_commanded_wheels{};
  robot::MotorCommand requested_funnel_command{};
  robot::LineFollowerUpdate last_update{};
  robot::LineObservation last_line_observation{};
  robot::SolarPanelAutonomyConfig solar_config{kSolarPanelAutonomyConfig};
  SolarIrThresholds solar_thresholds{};
  SolarLineFollowSpeedConfig solar_speed_config{};
  robot::SolarPanelContactConfig solar_contact_config{
      kSolarPanelContactConfig};
  robot::SolarBeaconDetectorState solar_detector{};
  robot::SolarBeaconDetectorUpdate last_solar_detector_update{};
  robot::SolarPanelAutonomyState autonomous_state{
      robot::SolarPanelAutonomyState::WaitForStart};
  robot::SolarPanelFaultReason autonomous_fault_reason{
      robot::SolarPanelFaultReason::None};
  robot::Milliseconds autonomous_state_entered_at_ms{0};
  char command_buffer[kSerialCommandBufferSize]{};
  std::size_t command_length{0};
  robot::Milliseconds last_telemetry_at_ms{0};
  robot::Milliseconds mode_expires_at_ms{0};
  robot::Milliseconds last_command_ms{0};
  std::int8_t line_sensor_last_known_side{0};
  bool command_deadman_armed{false};
  bool solar_start_requested{false};
  bool fault_active{false};
  robot::FaultCode fault_code{robot::FaultCode::None};
  char fault_message[robot::kTelemetryFaultMessageSize]{};
};

struct RuntimeBindings {
  RuntimeContext* context{nullptr};
  DigitalFrontLineSensorReader* sensors{nullptr};
  DualPwmMotorOutput* front_left{nullptr};
  DualPwmMotorOutput* front_right{nullptr};
  RearCommandLink* rear_link{nullptr};
  ClawServoBank* claws{nullptr};
  robot::esp2::StepperAxis* stepper{nullptr};
  Preferences* preferences{nullptr};
};

RuntimeBindings g_runtime{};

float hardwareDutyCap() {
  return clampFloat(robot::esp2::kHardwareConfig.maximum_safe_test_duty, 0.0F,
                    1.0F);
}

float activeMotionDutyCap(const RuntimeContext& context) {
  return clampFloat(context.config.maxDuty, 0.0F, hardwareDutyCap());
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

robot::FourWheelCommand makeSolarStrafeRightCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.solar_contact_config.strafe_duty, 0.0F,
                 activeMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(1.0F, 0.0F, 0.0F, duty, now_ms,
                                   context.config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeSolarStrafeLeftCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.solar_contact_config.strafe_duty, 0.0F,
                 activeMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(-1.0F, 0.0F, 0.0F, duty, now_ms,
                                   context.config.remoteCommandTimeoutMs);
}

robot::FourWheelCommand makeSolarForwardCommand(
    const RuntimeContext& context, const robot::Milliseconds now_ms) {
  const float duty =
      clampFloat(context.solar_contact_config.strafe_duty, 0.0F,
                 activeMotionDutyCap(context));
  return robot::mixOpenLoopMecanum(0.0F, 1.0F, 0.0F, duty, now_ms,
                                   context.config.remoteCommandTimeoutMs);
}

void sendStoppedRearCommand(RearCommandLink& rear_link,
                            const robot::LineFollowerConfig& config,
                            const robot::Milliseconds now_ms) {
  robot::RearDriveCommand command{};
  command.enabled = false;
  command.sender_timestamp_ms = now_ms;
  command.timeout_ms = config.remoteCommandTimeoutMs;
  rear_link.send(command);
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

void disableMotionActuators(RuntimeContext& context,
                            robot::IMotorOutput& front_left,
                            robot::IMotorOutput& front_right,
                            RearCommandLink& rear_link,
                            const robot::Milliseconds now_ms) {
  robot::stopLineFollower(context.follower_state);
  context.requested_command = robot::disabledFourWheelCommand();
  context.last_commanded_wheels = robot::disabledFourWheelCommand();
  context.command_deadman_armed = false;
  context.mode_expires_at_ms = 0U;
  front_left.disable();
  front_right.disable();
  sendStoppedRearCommand(rear_link, context.config, now_ms);
}

void disableActuators(RuntimeContext& context, robot::IMotorOutput& front_left,
                      robot::IMotorOutput& front_right,
                      RearCommandLink& rear_link,
                      const robot::Milliseconds now_ms) {
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  context.requested_funnel_command = robot::disabledMotorCommand();
  sendStoppedFunnelCommand(rear_link, context.config, now_ms);
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

bool solarPanelStartRequirementsMet(
    const DigitalFrontLineSensorReader& sensors,
    const DualPwmMotorOutput& front_left,
    const DualPwmMotorOutput& front_right,
    const RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const RuntimeContext& context) {
  return startRequirementsMet(sensors, front_left, front_right, rear_link,
                              now_ms, context) &&
         solarPanelLimitSwitchesReady(rear_link, now_ms, context);
}

bool sendRearWheelCommand(RearCommandLink& rear_link,
                          const robot::FourWheelCommand& wheels,
                          const robot::LineFollowerConfig& config,
                          const robot::Milliseconds now_ms) {
  robot::RearDriveCommand rear{};
  rear.enabled = wheels.back_left.enabled || wheels.back_right.enabled;
  rear.back_left_command_milli = wheels.back_left.duty_command_milli;
  rear.back_right_command_milli = wheels.back_right.duty_command_milli;
  rear.sender_timestamp_ms = now_ms;
  rear.timeout_ms = config.remoteCommandTimeoutMs;
  return rear_link.send(rear);
}

bool applyWheelCommand(RuntimeContext& context,
                       robot::IMotorOutput& front_left,
                       robot::IMotorOutput& front_right,
                       RearCommandLink& rear_link,
                       const robot::FourWheelCommand& wheels,
                       const robot::Milliseconds now_ms) {
  context.last_commanded_wheels = wheels;
  front_left.apply(wheels.front_left);
  front_right.apply(wheels.front_right);
  return sendRearWheelCommand(rear_link, wheels, context.config, now_ms);
}

void printTelemetry(RuntimeContext& context,
                    const DigitalFrontLineSensorReader& sensors,
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
  context.solar_start_requested = true;
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info, source,
           "solar autonomy start requested");
}

void enterSolarPanelAligned(RuntimeContext& context,
                            robot::IMotorOutput& front_left,
                            robot::IMotorOutput& front_right,
                            RearCommandLink& rear_link,
                            const robot::Milliseconds now_ms) {
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
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  enterSolarPanelAutonomyState(
      context, robot::SolarPanelAutonomyState::SolarPanelContacted, now_ms);
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::System,
           "solar panel limit switches contacted");
}

void enterSolarPanelSearchFault(
    RuntimeContext& context, robot::IMotorOutput& front_left,
    robot::IMotorOutput& front_right, RearCommandLink& rear_link,
    const robot::Milliseconds now_ms,
    const robot::SolarPanelFaultReason reason,
    const robot::FaultCode fault_code, const char* message,
    const robot::EventSource source) {
  disableMotionActuators(context, front_left, front_right, rear_link, now_ms);
  enterSolarPanelAutonomyState(
      context, robot::SolarPanelAutonomyState::SolarSearchFault, now_ms,
      reason);
  setFault(context, fault_code, message);
  logEvent(context, now_ms, robot::EventSeverity::Fault, source, message);
}

void runSolarPanelAutonomy(RuntimeContext& context,
                           DigitalFrontLineSensorReader& sensors,
                           DualPwmMotorOutput& front_left,
                           DualPwmMotorOutput& front_right,
                           RearCommandLink& rear_link,
                           const robot::Milliseconds now_ms) {
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
            "solar start rejected: hardware or limit switches incomplete",
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
                             context.last_update.wheel_command, now_ms)) {
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
    case robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel: {
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

      const bool back_hit =
          solarPanelLimitSwitchHit(esp1.solar_limit_back_right_high);
      const bool front_hit =
          solarPanelLimitSwitchHit(esp1.solar_limit_front_right_high);
      const robot::SolarPanelContactSequenceUpdate sequence_update =
          robot::updateSolarPanelContactSequence(
              context.autonomous_state, front_hit, back_hit,
              time_in_state_ms, context.solar_contact_config);
      if (sequence_update.next_state ==
          robot::SolarPanelAutonomyState::SolarPanelContacted) {
        enterSolarPanelContacted(context, front_left, front_right, rear_link,
                                 now_ms);
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
        disableMotionActuators(context, front_left, front_right, rear_link,
                               now_ms);
        enterSolarPanelAutonomyState(context, sequence_update.next_state,
                                     now_ms);
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
             context.solar_contact_config.retry_forward_duration_ms == 0U);
        if (zero_duration_adjustment) {
          printTelemetry(context, sensors, rear_link, now_ms);
          return;
        }
      }

      robot::FourWheelCommand wheels{};
      switch (context.autonomous_state) {
        case robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry:
          wheels = makeSolarStrafeLeftCommand(context, now_ms);
          break;
        case robot::SolarPanelAutonomyState::MoveForwardForSolarRetry:
          wheels = makeSolarForwardCommand(context, now_ms);
          break;
        case robot::SolarPanelAutonomyState::StrafeRightToSolarPanel:
        case robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel:
          wheels = makeSolarStrafeRightCommand(context, now_ms);
          break;
        default:
          wheels = robot::disabledFourWheelCommand();
          break;
      }
      if (!applyWheelCommand(context, front_left, front_right, rear_link,
                             wheels, now_ms)) {
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

    case robot::SolarPanelAutonomyState::SolarPanelContacted:
    case robot::SolarPanelAutonomyState::SolarSearchFault:
      disableMotionActuators(context, front_left, front_right, rear_link,
                             now_ms);
      return;
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

  const robot::LineObservation& observation =
      context.follower_state.enabled
          ? context.last_update.observation
          : context.last_line_observation;
  snapshot.lsfl_raw_level = sensors.lastLeftLevel();
  snapshot.lsfr_raw_level = sensors.lastRightLevel();
  snapshot.lsfl_black = observation.left_black;
  snapshot.lsfr_black = observation.right_black;
  snapshot.line_error = observation.error;
  snapshot.line_visible = observation.line_visible;
  snapshot.line_has_history = observation.hasHistory;
  snapshot.last_known_line_side = observation.last_known_side;
  snapshot.line_follower_enabled = context.follower_state.enabled;

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
      context.solar_contact_config.strafe_duty;
  snapshot.solar_strafe_start_delay_ms =
      context.solar_contact_config.strafe_start_delay_ms;
  snapshot.solar_retry_strafe_left_duration_ms =
      context.solar_contact_config.retry_strafe_left_duration_ms;
  snapshot.solar_retry_forward_duration_ms =
      context.solar_contact_config.retry_forward_duration_ms;
  snapshot.solar_retry_strafe_timeout_ms =
      context.solar_contact_config.retry_strafe_timeout_ms;

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
    snapshot.lss_configured = esp1.side_line_sensor_configured;
    if (snapshot.rear.esp1_link_healthy && snapshot.lss_configured) {
      snapshot.lss_raw_level = esp1.side_line_sensor_high ? 1 : 0;
      snapshot.lss_black = esp1.side_line_sensor_high;
    }
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
         g_runtime.front_left != nullptr && g_runtime.front_right != nullptr &&
         g_runtime.rear_link != nullptr && g_runtime.claws != nullptr;
}

robot::TelemetrySnapshot currentSnapshot() {
  robot::TelemetrySnapshot snapshot{};
  if (runtimeReady()) {
    fillTelemetrySnapshot(*g_runtime.context, *g_runtime.sensors,
                          *g_runtime.front_left, *g_runtime.front_right,
                          *g_runtime.rear_link, *g_runtime.claws, snapshot,
                          static_cast<robot::Milliseconds>(millis()));
  }
  return snapshot;
}

void handleRoot();
void handleStatus();
void handleTelemetry();
void handleStop();
void handleMode();
void handleDrive();
void handleMotor();
void handleInvert();
void handleSensors();
void handleLine();
void handleAutonomousSolarStart();
void handleAutonomousSolarConfig();
void handleLineFollowStart();
void handleLineFollowStop();
void handleLineFollowConfig();
void handleClaw();
void handleClawsAll();
void handleClawsConfig();
void handleClawsSave();
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
    <h2>Autonomous Solar</h2>
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
      <label>Strafe delay ms <input id="solarStrafeDelayMs" type="number" min="0" step="50"></label>
      <label>Strafe duty <input id="solarStrafeDuty" type="number" min="0" max="1" step="0.01"></label>
      <label>Retry left strafe ms <input id="solarRetryLeftMs" type="number" min="0" step="50"></label>
      <label>Retry forward ms <input id="solarRetryForwardMs" type="number" min="0" step="50"></label>
      <label>Retry right timeout ms <input id="solarRetryStrafeTimeoutMs" type="number" min="1" step="500"></label>
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
      <span>Strafe duty</span><span id="solarLimitDuty" class="mono"></span>
      <span>Retry left</span><span id="solarRetryLeft" class="mono"></span>
      <span>Retry forward</span><span id="solarRetryForward" class="mono"></span>
      <span>Retry right timeout</span><span id="solarRetryTimeout" class="mono"></span>
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
    <h2>Line Sensors</h2>
    <div class="kv">
      <span>Enabled</span><span id="lfEnabled" class="mono"></span>
      <span>LSFL</span><span id="lsfl" class="mono"></span>
      <span>LSFR</span><span id="lsfr" class="mono"></span>
      <span>LSS (side)</span><span id="lss" class="mono"></span>
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
      <span>Lower limit</span><span id="stepperLimit" class="mono"></span>
      <span>Homed</span><span id="stepperHomed" class="mono"></span>
    </div>
    <div class="two">
      <label>Speed steps/s <input id="stepperSpeed" type="number" min="1" value="800"></label>
      <label>Acceleration steps/s² <input id="stepperAcceleration" type="number" min="1" value="1200"></label>
      <label>Maximum position <input id="stepperMaximum" type="number" min="1" placeholder="REQUIRED"></label>
      <label>Jog microsteps <input id="stepperJog" type="number" min="1" value="100"></label>
      <label>Absolute target <input id="stepperTarget" type="number" min="0" value="0"></label>
    </div>
    <div class="row">
      <button class="run" onpointerdown="stepperHold('up')" onpointerup="stepperRelease()" onpointerleave="stepperRelease()" onpointercancel="stepperRelease()">Hold Up</button>
      <button class="run" onpointerdown="stepperHold('down')" onpointerup="stepperRelease()" onpointerleave="stepperRelease()" onpointercancel="stepperRelease()">Hold Down</button>
      <button class="stop" onclick="stepperCommand('stop')">Stop</button>
      <button onclick="stepperCommand('wake')">Wake</button>
      <button onclick="stepperCommand('sleep')">Sleep</button>
      <button class="warn" onclick="stepperCommand('home')">Home</button>
      <button class="warn" title="Debug only: changes software position without moving" onclick="stepperCommand('zero')">Set Zero (DEBUG)</button>
    </div>
    <div class="row">
      <button onclick="stepperJog(1)">Jog Up</button><button onclick="stepperJog(-1)">Jog Down</button>
      <button onclick="stepperMove()">Move Absolute</button><button onclick="stepperConfig()">Apply Config</button>
    </div>
  </section>

	  <section>
	    <h2>Claws</h2>
    <div class="kv">
      <span>Hardware</span><span id="clawHardware" class="mono"></span>
      <span>Commanded</span><span id="clawCommanded" class="mono"></span>
    </div>
    <div class="two">
      <label>Claw 1 start <input id="claw1Start" type="number" min="0" max="180" step="1"></label>
      <label>Claw 1 dir <select id="claw1Dir"><option value="1">+90</option><option value="-1">-90</option></select></label>
      <label>Claw 2 start <input id="claw2Start" type="number" min="0" max="180" step="1"></label>
      <label>Claw 2 dir <select id="claw2Dir"><option value="1">+90</option><option value="-1">-90</option></select></label>
      <label>Claw 3 start <input id="claw3Start" type="number" min="0" max="180" step="1"></label>
      <label>Claw 3 dir <select id="claw3Dir"><option value="1">+90</option><option value="-1">-90</option></select></label>
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
    </div>
	    <pre id="claws"></pre>
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
    <h2>Recent Events</h2>
    <pre id="events"></pre>
  </section>
</main>
<script>
let holdTimer = null;
let lfLoaded = false;
let clawsLoaded = false;
let solarLoaded = false;
function qs(id){ return document.getElementById(id); }
function api(path){ return fetch(path).then(r => r.json().catch(() => ({})).then(j => ({ok:r.ok, status:r.status, json:j}))); }
function stepperCommand(command, extra=''){ return api(`/api/stepper/command?command=${command}${extra}`); }
let stepperHeartbeat = null;
function stepperHold(direction){
  stepperRelease(false);
  stepperCommand(direction);
  stepperHeartbeat = setInterval(() => stepperCommand('hold'), 150);
}
function stepperRelease(sendStop=true){
  if(stepperHeartbeat){ clearInterval(stepperHeartbeat); stepperHeartbeat=null; }
  if(sendStop) stepperCommand('stop');
}
function stepperJog(sign){ const n=Math.abs(Number(qs('stepperJog').value)); return stepperCommand('jog',`&steps=${sign*n}`); }
function stepperMove(){ return stepperCommand('move',`&steps=${qs('stepperTarget').value}`); }
function stepperConfig(){
  const p=new URLSearchParams({command:'config',speed:qs('stepperSpeed').value,acceleration:qs('stepperAcceleration').value});
  if(qs('stepperMaximum').value) p.set('maximum',qs('stepperMaximum').value);
  return api(`/api/stepper/command?${p.toString()}`);
}
function updateStepper(){ fetch('/api/stepper').then(r=>r.json()).then(s=>{
  qs('stepperPosition').textContent=`${s.positionSteps} µsteps`;
  qs('stepperMotion').textContent=s.motionState;
  qs('stepperSpeedNow').textContent=`${s.speedStepsPerSecond} µsteps/s`;
  qs('stepperSleep').textContent=s.sleeping?'asleep':'awake / holding';
  qs('stepperLimit').textContent=s.lowerLimitActive?'PRESSED':'released';
  qs('stepperLimit').className=s.lowerLimitActive?'mono bad':'mono good';
  qs('stepperHomed').textContent=yn(s.homed);
  qs('stepperHomed').className=s.homed?'mono good':'mono bad';
  if(document.activeElement!==qs('stepperMaximum') && s.maximumPositionSteps>0) qs('stepperMaximum').value=s.maximumPositionSteps;
}).catch(()=>{ qs('stepperMotion').textContent='disconnected'; }); }
function stopAll(){ if (holdTimer) clearInterval(holdTimer); holdTimer=null; api('/api/stop'); }
function drive(vx,vy,wz){
  const duty = Number(qs('driveDuty').value || 0);
  const send = () => api(`/api/drive?vx=${vx}&vy=${vy}&wz=${wz}&duty=${duty}`);
  send(); if (holdTimer) clearInterval(holdTimer); holdTimer=setInterval(send, 200);
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
function lineSensorMode(){ api('/api/mode?mode=line-sensor'); }
function autoSolarStart(){ api('/api/autonomous/solar/start'); }
function loadSolarControls(j){
  if (solarLoaded || !j.autonomous) return;
  const a = j.autonomous;
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
  setLfValue('solarStrafeDelayMs', a.strafe_start_delay_ms);
  setLfValue('solarStrafeDuty', a.strafe_duty);
  setLfValue('solarRetryLeftMs', a.retry_strafe_left_duration_ms);
  setLfValue('solarRetryForwardMs', a.retry_forward_duration_ms);
  setLfValue('solarRetryStrafeTimeoutMs', a.retry_strafe_timeout_ms);
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
  add('strafe-delay-ms', 'solarStrafeDelayMs');
  add('strafe-duty', 'solarStrafeDuty');
  add('retry-left-ms', 'solarRetryLeftMs');
  add('retry-forward-ms', 'solarRetryForwardMs');
  add('retry-strafe-timeout-ms', 'solarRetryStrafeTimeoutMs');
  return p;
}
function autoSolarApply(){ return api(`/api/autonomous/solar/config?${solarParams().toString()}`); }
function autoSolarSave(){ autoSolarApply().then(r => { if (r.ok) api('/api/config/save'); }); }
function setClawStart(id, value){
  const el = qs(id);
  if (!el || document.activeElement === el) return;
  el.value = value >= 0 ? value : '';
}
function loadClawControls(j){
  if (clawsLoaded || !j.claws) return;
  const claws = [j.claws.claw_1, j.claws.claw_2, j.claws.claw_3];
  claws.forEach((claw, index) => {
    const n = index + 1;
    setClawStart(`claw${n}Start`, claw.startAngleDeg);
    setLfValue(`claw${n}Dir`, claw.openDirection);
  });
  clawsLoaded = true;
}
function clawParams(){
  const p = new URLSearchParams();
  for (let n = 1; n <= 3; n++) {
    const start = qs(`claw${n}Start`).value;
    if (start !== '') p.set(`claw${n}-start`, start);
    p.set(`claw${n}-dir`, qs(`claw${n}Dir`).value);
  }
  return p;
}
function clawsApply(){ return api(`/api/claws/config?${clawParams().toString()}`); }
function clawsSave(){ clawsApply().then(r => { if (r.ok) api('/api/claws/save'); }); }
function claw(id,state){ clawsApply().then(r => { if (r.ok) api(`/api/claw?id=${id}&state=${state}`); }); }
function clawAll(state){ clawsApply().then(r => { if (r.ok) api(`/api/claws?state=${state}`); }); }
function clawSummary(c){
  if (!c) return 'n/a';
  const start = c.startConfigured ? c.startAngleDeg : 'unset';
  const target = c.outputEnabled ? c.commandedAngleDeg : 'off';
  return `start=${start} open=${c.openAngleDeg} target=${target}`;
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
    loadSolarControls(j);
    loadLfControls(j);
    loadClawControls(j);
    qs('lfEnabled').textContent = yn(j.line.line_follower_enabled);
    qs('lsfl').textContent = `${j.line.lsfl_level} black=${yn(j.line.lsfl_black)}`;
    qs('lsfr').textContent = `${j.line.lsfr_level} black=${yn(j.line.lsfr_black)}`;
    qs('lss').textContent = `${j.line.lss_level} black=${yn(j.line.lss_black)} configured=${yn(j.line.lss_configured)}`;
    qs('lfError').textContent = `${j.line.line_error}, side=${j.line.last_known_line_side}, visible=${yn(j.line.line_visible)}, hist=${yn(j.line.has_history)}`;
    qs('lfPid').textContent = `P=${j.pid.p_term.toFixed(3)} I=${j.pid.i_term.toFixed(3)} D=${j.pid.d_term.toFixed(3)} C=${j.pid.correction.toFixed(3)}`;
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
    qs('claws').textContent = JSON.stringify(claws, null, 2);
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

void handleStop() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
  emergencyStop(*g_runtime.context, *g_runtime.front_left,
                *g_runtime.front_right, *g_runtime.rear_link, now_ms,
                robot::EventSource::Web);
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
  if (!argFloat("vx", vx, 0.0F, true) ||
      !argFloat("vy", vy, 0.0F, true) ||
      !argFloat("wz", wz, 0.0F, true) ||
      !argFloat("duty", duty, 0.0F, true)) {
    sendErrorJson(400, "malformed drive argument");
    return;
  }

  if (context.modes.currentMode() !=
      robot::RobotTestMode::DistributedDriveTest) {
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

  context.requested_command =
      makeManualDriveCommand(context, vx, vy, wz, duty, now_ms);
  context.last_command_ms = now_ms;
  context.command_deadman_armed = true;
  context.mode_expires_at_ms = now_ms + kCommandTimeoutMs;
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
                "\"lss_raw_level\":%d,"
                "\"lsfl_level\":\"%s\",\"lsfr_level\":\"%s\","
                "\"lss_level\":\"%s\","
                "\"lsfl_black\":%s,\"lsfr_black\":%s,"
                "\"lss_black\":%s,\"lss_configured\":%s,"
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
                snapshot.lss_raw_level,
                digitalLevelName(snapshot.lsfl_raw_level),
                digitalLevelName(snapshot.lsfr_raw_level),
                digitalLevelName(snapshot.lss_raw_level),
                snapshot.lsfl_black ? "true" : "false",
                snapshot.lsfr_black ? "true" : "false",
                snapshot.lss_black ? "true" : "false",
                snapshot.lss_configured ? "true" : "false",
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
                "{\"LSFL\":%d,\"LSFR\":%d,\"LSS\":%d,"
                "\"LSFLLevel\":\"%s\",\"LSFRLevel\":\"%s\","
                "\"LSSLevel\":\"%s\",\"leftBlack\":%s,"
                "\"rightBlack\":%s,\"sideBlack\":%s,"
                "\"sideConfigured\":%s,\"error\":%d,"
                "\"lastKnownSide\":%d,"
                "\"lineVisible\":%s,\"hasHistory\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                snapshot.lss_raw_level,
                digitalLevelName(snapshot.lsfl_raw_level),
                digitalLevelName(snapshot.lsfr_raw_level),
                digitalLevelName(snapshot.lss_raw_level),
                snapshot.lsfl_black ? "true" : "false",
                snapshot.lsfr_black ? "true" : "false",
                snapshot.lss_black ? "true" : "false",
                snapshot.lss_configured ? "true" : "false",
                static_cast<int>(snapshot.line_error),
                static_cast<int>(snapshot.last_known_line_side),
                snapshot.line_visible ? "true" : "false",
                snapshot.line_has_history ? "true" : "false");
  g_server.send(200, "application/json", g_json_buffer);
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
  if (g_server.hasArg("strafe-delay-ms")) {
    if (!argUnsigned("strafe-delay-ms", milliseconds_value,
                     next_contact.strafe_start_delay_ms, true)) {
      sendErrorJson(400, "malformed strafe-delay-ms");
      return;
    }
    next_contact.strafe_start_delay_ms = milliseconds_value;
  }
  if (g_server.hasArg("strafe-duty")) {
    if (!argFloat("strafe-duty", float_value, next_contact.strafe_duty,
                  true)) {
      sendErrorJson(400, "malformed strafe-duty");
      return;
    }
    next_contact.strafe_duty = float_value;
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
      !robot::solarPanelContactConfigValid(next_contact)) {
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
  return result == ClawServoCommandResult::HardwareUnconfigured
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
  int value = 0;

  value = settings.start_angle_deg[0];
  if (!readOptionalClawInt("claw1-start", "claw1Start", value,
                           "malformed claw1 start angle")) {
    return false;
  }
  settings.start_angle_deg[0] = value;

  value = settings.start_angle_deg[1];
  if (!readOptionalClawInt("claw2-start", "claw2Start", value,
                           "malformed claw2 start angle")) {
    return false;
  }
  settings.start_angle_deg[1] = value;

  value = settings.start_angle_deg[2];
  if (!readOptionalClawInt("claw3-start", "claw3Start", value,
                           "malformed claw3 start angle")) {
    return false;
  }
  settings.start_angle_deg[2] = value;

  value = settings.open_direction[0];
  if (!readOptionalClawInt("claw1-dir", "claw1Direction", value,
                           "malformed claw1 direction")) {
    return false;
  }
  settings.open_direction[0] = value;

  value = settings.open_direction[1];
  if (!readOptionalClawInt("claw2-dir", "claw2Direction", value,
                           "malformed claw2 direction")) {
    return false;
  }
  settings.open_direction[1] = value;

  value = settings.open_direction[2];
  if (!readOptionalClawInt("claw3-dir", "claw3Direction", value,
                           "malformed claw3 direction")) {
    return false;
  }
  settings.open_direction[2] = value;
  return true;
}

void enterMechanismTestIfNeeded(RuntimeContext& context,
                                const robot::Milliseconds now_ms) {
  if (context.modes.currentMode() == robot::RobotTestMode::MechanismTest) {
    return;
  }
  disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                   *g_runtime.rear_link, now_ms);
  context.modes.setMode(robot::RobotTestMode::MechanismTest, now_ms);
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
           "claw settings updated");
  sendOkJson("claw settings updated");
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
  g_runtime.preferences->putInt("c1start", settings.start_angle_deg[0]);
  g_runtime.preferences->putInt("c2start", settings.start_angle_deg[1]);
  g_runtime.preferences->putInt("c3start", settings.start_angle_deg[2]);
  g_runtime.preferences->putInt("c1dir", settings.open_direction[0]);
  g_runtime.preferences->putInt("c2dir", settings.open_direction[1]);
  g_runtime.preferences->putInt("c3dir", settings.open_direction[2]);
  sendOkJson("claw settings saved");
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

void handleLineFollowConfig() {
  if (!runtimeReady()) {
    sendErrorJson(503, "runtime not ready");
    return;
  }
  RuntimeContext& context = *g_runtime.context;
  robot::LineFollowerConfig next = context.config;
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

  context.config = next;
  clearFault(context);
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::Web,
           "line follower config updated");
  sendOkJson("line follower config updated");
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
  g_runtime.preferences->putUInt(
      "sdelay",
      context.solar_contact_config.strafe_start_delay_ms);
  g_runtime.preferences->putFloat("sstrfd",
                                  context.solar_contact_config.strafe_duty);
  g_runtime.preferences->putUInt(
      "srleft",
      context.solar_contact_config.retry_strafe_left_duration_ms);
  g_runtime.preferences->putUInt(
      "srfwd", context.solar_contact_config.retry_forward_duration_ms);
  g_runtime.preferences->putUInt(
      "srtmo", context.solar_contact_config.retry_strafe_timeout_ms);
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
      "\"configuredSpeedStepsPerSecond\":%u,\"accelerationStepsPerSecond2\":%u,"
      "\"sleeping\":%s,\"lowerLimitActive\":%s,\"homed\":%s,\"busy\":%s,"
      "\"maximumPositionSteps\":%lld,\"microstepsPerRevolution\":1600}",
      static_cast<long long>(axis.positionSteps()), axis.motionStateName(),
      axis.speedStepsPerSecond(), axis.configuredSpeedStepsPerSecond(),
      axis.accelerationStepsPerSecond2(), axis.sleeping() ? "true" : "false",
      axis.lowerLimitActive() ? "true" : "false", axis.isHomed() ? "true" : "false",
      axis.isBusy() ? "true" : "false", static_cast<long long>(axis.maximumPositionSteps()));
  g_server.send(200, "application/json", g_json_buffer);
}

bool stepperInt64Arg(const char* name, std::int64_t& value) {
  if (!g_server.hasArg(name)) return false;
  char* end = nullptr;
  const String text = g_server.arg(name);
  const long long parsed = strtoll(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') return false;
  value = static_cast<std::int64_t>(parsed); return true;
}

void handleStepperCommand() {
  if (!runtimeReady() || g_runtime.stepper == nullptr || !g_server.hasArg("command")) { sendErrorJson(400, "missing stepper command"); return; }
  auto& axis = *g_runtime.stepper;
  const String command = g_server.arg("command");
  bool accepted = true;
  if (command == "stop") axis.stop();
  else if (command == "wake") accepted = axis.wake();
  else if (command == "sleep") axis.sleep();
  else if (command == "home") accepted = axis.home();
  else if (command == "zero") axis.setZeroDebug();
  else if (command == "up" || command == "down") accepted = axis.moveContinuous(command == "up" ? robot::esp2::StepperDirection::Up : robot::esp2::StepperDirection::Down);
  else if (command == "hold") axis.refreshHoldCommand();
  else if (command == "jog") { std::int64_t value; accepted = stepperInt64Arg("steps", value) && axis.jogSteps(value); }
  else if (command == "move") { std::int64_t value; accepted = stepperInt64Arg("steps", value) && axis.moveToSteps(value); }
  else if (command == "config") {
    std::int64_t value;
    if (stepperInt64Arg("speed", value)) accepted = value > 0 && value <= UINT32_MAX && axis.setSpeed(static_cast<std::uint32_t>(value));
    if (accepted && stepperInt64Arg("acceleration", value)) accepted = value > 0 && value <= UINT32_MAX && axis.setAcceleration(static_cast<std::uint32_t>(value));
    if (accepted && stepperInt64Arg("maximum", value)) accepted = axis.setMaximumPositionSteps(value);
  } else accepted = false;
  if (!accepted) { sendErrorJson(409, "stepper command rejected by limits or state"); return; }
  sendOkJson(command == "zero" ? "debug software zero set" : "stepper command accepted");
}

void setupWebHandlers() {
  g_server.on("/", HTTP_GET, handleRoot);
  g_server.on("/api/status", HTTP_GET, handleStatus);
  g_server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  g_server.on("/api/stop", HTTP_ANY, handleStop);
  g_server.on("/api/mode", HTTP_ANY, handleMode);
  g_server.on("/api/drive", HTTP_ANY, handleDrive);
  g_server.on("/api/motor", HTTP_ANY, handleMotor);
  g_server.on("/api/invert", HTTP_ANY, handleInvert);
  g_server.on("/api/sensors", HTTP_GET, handleSensors);
  g_server.on("/api/line", HTTP_GET, handleLine);
  g_server.on("/api/autonomous/solar/start", HTTP_ANY,
              handleAutonomousSolarStart);
  g_server.on("/api/autonomous/solar/config", HTTP_ANY,
              handleAutonomousSolarConfig);
  g_server.on("/api/line-follow/start", HTTP_ANY, handleLineFollowStart);
  g_server.on("/api/line-follow/stop", HTTP_ANY, handleLineFollowStop);
  g_server.on("/api/line-follow/config", HTTP_ANY, handleLineFollowConfig);
  g_server.on("/api/claw", HTTP_ANY, handleClaw);
  g_server.on("/api/claws", HTTP_ANY, handleClawsAll);
  g_server.on("/api/claws/config", HTTP_ANY, handleClawsConfig);
  g_server.on("/api/claws/save", HTTP_ANY, handleClawsSave);
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
  Serial.print(", solar-strafe-duty=");
  Serial.print(context.solar_contact_config.strafe_duty, 4);
  Serial.print(", solar-retry-left-ms=");
  Serial.print(
      context.solar_contact_config.retry_strafe_left_duration_ms);
  Serial.print(", solar-retry-forward-ms=");
  Serial.print(context.solar_contact_config.retry_forward_duration_ms);
  Serial.print(", solar-retry-strafe-timeout-ms=");
  Serial.print(context.solar_contact_config.retry_strafe_timeout_ms);
  if (rear_link.statusAvailable()) {
    const robot::Esp1StatusReport& esp1 = rear_link.latestStatus();
    Serial.print(", solar-limit-configured=");
    Serial.print(esp1.solar_panel_limit_switches_configured ? 1 : 0);
    Serial.print(", solar-limit-br-high=");
    Serial.print(esp1.solar_limit_back_right_high ? 1 : 0);
    Serial.print(", solar-limit-fr-high=");
    Serial.print(esp1.solar_limit_front_right_high ? 1 : 0);
  }
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
  Serial.println("  auto solar|status");
  Serial.println("  lf start|stop|status|reset");
  Serial.println("  lf kp|ki|kd|base|max-duty|max-correction <value>");
  Serial.println("  lf integral-limit|derivative-limit|derivative-alpha <value>");
  Serial.println("  lf polarity <1|-1> | lf telemetry on|off");
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
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Serial, "mode changed");
  printOk("mode changed");
  return true;
}

bool updateTuningValue(RuntimeContext& context, const char* name,
                       const char* value_text) {
  robot::LineFollowerConfig next = context.config;
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

  const robot::CommandValidationResult validation =
      robot::validateLineFollowerConfig(next, hardwareDutyCap());
  if (!validation.accepted) {
    printRejected(validation.reason);
    return true;
  }
  context.config = next;
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
  if (std::strcmp(token, "lf") == 0) {
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
      if (context.modes.currentMode() !=
          robot::RobotTestMode::LineFollowTest) {
        disableActuators(context, front_left, front_right, rear_link, now_ms);
        context.modes.setMode(robot::RobotTestMode::LineFollowTest, now_ms);
      }
      if (!startRequirementsMet(sensors, front_left, front_right, rear_link,
                                now_ms, context)) {
        setFault(context, robot::FaultCode::HardwareNotConfigured,
                 "line follower hardware requirements are incomplete");
        printRejected(
            "configure sensors, motors, UART, ESP1 status, max-duty, hardware cap");
        return;
      }
      robot::startLineFollower(context.follower_state, now_ms);
      context.last_command_ms = now_ms;
      context.mode_expires_at_ms = now_ms + duration_ms;
      context.command_deadman_armed = true;
      printOk("line follower started");
      return;
    }
    if (std::strcmp(command, "stop") == 0) {
      disableActuators(context, front_left, front_right, rear_link, now_ms);
      printOk("line follower stopped");
      return;
    }
    if (std::strcmp(command, "status") == 0) {
      printStatus(context, rear_link, sensors, front_left, front_right,
                  now_ms);
      return;
    }
    if (std::strcmp(command, "reset") == 0) {
      disableActuators(context, front_left, front_right, rear_link, now_ms);
      context.last_update = {};
      context.last_line_observation = {};
      context.line_sensor_last_known_side = 0;
      printOk("line follower reset");
      return;
    }
    if (std::strcmp(command, "telemetry") == 0) {
      char* value = strtok(nullptr, " \t\r\n");
      bool telemetry_enabled = false;
      if (!parseOnOff(value, telemetry_enabled)) {
        printRejected("telemetry must be on or off");
        return;
      }
      context.config.telemetryEnabled = telemetry_enabled;
      printOk("telemetry");
      return;
    }
    if (updateTuningValue(context, command, strtok(nullptr, " \t\r\n"))) {
      return;
    }
    printRejected("unknown lf command");
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
  context.solar_contact_config.strafe_start_delay_ms =
      preferences.getUInt(
          "sdelay",
          context.solar_contact_config.strafe_start_delay_ms);
  context.solar_contact_config.strafe_duty =
      preferences.getFloat("sstrfd",
                           context.solar_contact_config.strafe_duty);
  context.solar_contact_config.retry_strafe_left_duration_ms =
      preferences.getUInt(
          "srleft",
          context.solar_contact_config.retry_strafe_left_duration_ms);
  context.solar_contact_config.retry_forward_duration_ms =
      preferences.getUInt(
          "srfwd", context.solar_contact_config.retry_forward_duration_ms);
  context.solar_contact_config.retry_strafe_timeout_ms =
      preferences.getUInt(
          "srtmo", context.solar_contact_config.timeout_ms);

  ClawServoSettings claw_settings = claws.settings();
  claw_settings.start_angle_deg[0] =
      preferences.getInt("c1start", claw_settings.start_angle_deg[0]);
  claw_settings.start_angle_deg[1] =
      preferences.getInt("c2start", claw_settings.start_angle_deg[1]);
  claw_settings.start_angle_deg[2] =
      preferences.getInt("c3start", claw_settings.start_angle_deg[2]);
  claw_settings.open_direction[0] =
      preferences.getInt("c1dir", claw_settings.open_direction[0]) < 0 ? -1
                                                                       : 1;
  claw_settings.open_direction[1] =
      preferences.getInt("c2dir", claw_settings.open_direction[1]) < 0 ? -1
                                                                       : 1;
  claw_settings.open_direction[2] =
      preferences.getInt("c3dir", claw_settings.open_direction[2]) < 0 ? -1
                                                                       : 1;
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
}

void motionControlTask(void* parameters) {
  (void)parameters;

  RuntimeContext context{};
  const float cap = hardwareDutyCap();
  context.config.maxDuty = cap;
  context.config.maxCorrection =
      clampFloat(context.config.maxCorrection, 0.0F, cap);

  DigitalFrontLineSensorReader line_sensor_reader{
      robot::esp2::kHardwareConfig.pins};
  DualPwmMotorOutput front_left_motor{
      robot::esp2::kHardwareConfig.front_left_motor};
  DualPwmMotorOutput front_right_motor{
      robot::esp2::kHardwareConfig.front_right_motor};
  RearCommandLink rear_link{robot::esp2::kHardwareConfig.uart_to_esp1};
  robot::esp2::StepperAxis stepper{{robot::esp2::kPins.stepper_sleep,
      robot::esp2::kPins.stepper_dir, robot::esp2::kPins.stepper_step,
      robot::esp2::kPins.limit_switch_stepper_bottom, 800U, 1200U, 200U,
      3U, 3U, 2000U, 500U, 15U,
      0, 0U, 0U}};  // Maximum is intentionally unset until supplied from the dashboard.
  ClawServoBank claws{robot::esp2::kHardwareConfig.servo_claw_1,
                      robot::esp2::kHardwareConfig.servo_claw_2,
                      robot::esp2::kHardwareConfig.servo_claw_3};
  Preferences preferences{};

  line_sensor_reader.initialize();
  front_left_motor.initializeDisabled();
  front_right_motor.initializeDisabled();
  rear_link.initialize();
  stepper.begin();
  claws.initializeDisabled();

  preferences.begin("telemetry", false);
  loadPreferences(preferences, context, front_left_motor, front_right_motor,
                  claws);
  resetSolarPanelAutonomy(context, static_cast<robot::Milliseconds>(millis()));

  g_runtime = {&context, &line_sensor_reader, &front_left_motor,
               &front_right_motor, &rear_link, &claws, &stepper, &preferences};

  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  setupWebHandlers();
  g_server.begin();
  logEvent(context, static_cast<robot::Milliseconds>(millis()),
           robot::EventSeverity::Info, robot::EventSource::System,
           "telemetry web server started");

  TickType_t last_wake_tick = xTaskGetTickCount();

  for (;;) {
    const robot::Milliseconds now_ms =
        static_cast<robot::Milliseconds>(millis());
    g_server.handleClient();
    stepper.update();
    rear_link.pollReceive(now_ms);
    refreshLineObservation(context, line_sensor_reader, now_ms);
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
             robot::RobotTestMode::DistributedDriveTest);
    if (timed_mode_expired || stale_command) {
      disableActuators(context, front_left_motor, front_right_motor, rear_link,
                       now_ms);
      logEvent(context, now_ms, robot::EventSeverity::Warn,
               robot::EventSource::System,
               stale_command ? "command deadman expired"
                             : "timed test auto-stopped");
    }

    const robot::RobotTestMode mode = context.modes.currentMode();
    if (mode != robot::RobotTestMode::AutonomousSolarPanel &&
        context.autonomous_state !=
            robot::SolarPanelAutonomyState::WaitForStart) {
      resetSolarPanelAutonomy(context, now_ms);
    }
    if (robot::robotTestModeIsSensorOnly(mode)) {
      disableActuators(context, front_left_motor, front_right_motor, rear_link,
                       now_ms);
      if (mode == robot::RobotTestMode::SensorMonitor ||
          mode == robot::RobotTestMode::LineSensorTest) {
        runSensorOnlyTelemetry(context, line_sensor_reader, now_ms);
      }
    } else if (mode == robot::RobotTestMode::AutonomousSolarPanel) {
      runSolarPanelAutonomy(context, line_sensor_reader, front_left_motor,
                            front_right_motor, rear_link, now_ms);
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
                          now_ms);
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
    } else if (mode == robot::RobotTestMode::MechanismTest) {
      disableMotionActuators(context, front_left_motor, front_right_motor,
                             rear_link, now_ms);
    } else if (mode == robot::RobotTestMode::SingleMotorTest ||
               mode == robot::RobotTestMode::ManualDriveTest ||
               mode == robot::RobotTestMode::DistributedDriveTest) {
      applyWheelCommand(context, front_left_motor, front_right_motor, rear_link,
                        context.requested_command, now_ms);
    } else {
      disableActuators(context, front_left_motor, front_right_motor, rear_link,
                       now_ms);
    }

    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(context.config.controlPeriodMs == 0U
                                      ? kDefaultMotionTaskPeriodMs
                                      : context.config.controlPeriodMs));
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
