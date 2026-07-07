#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_system.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "common/ChassisMixer.h"
#include "common/Esp1Status.h"
#include "common/EventLog.h"
#include "common/LineFollower.h"
#include "common/LineObservation.h"
#include "common/LineSensor.h"
#include "common/MotorOutput.h"
#include "common/RearDriveCommand.h"
#include "common/RobotCommandValidation.h"
#include "common/RobotTestModeManager.h"
#include "common/TelemetrySnapshot.h"
#include "common/UartProtocol.h"
#include "esp2/MechanismControllers.h"
#include "esp2/PinConfig.h"

namespace {

constexpr const char* kApSsid = "Team14Robot";
constexpr const char* kApPassword = "robotdebug";
constexpr robot::Milliseconds kDefaultMotionTaskPeriodMs = 10U;
constexpr robot::Milliseconds kTelemetryPeriodMs = 100U;
constexpr robot::Milliseconds kCommandTimeoutMs = 700U;
constexpr robot::Milliseconds kMaxTimedTestDurationMs = 5000U;
constexpr float kSingleMotorDutyCap = 1.0F;
constexpr std::uint32_t kTaskStackBytes = 12288U;
constexpr UBaseType_t kTaskPriority = 1U;
constexpr BaseType_t kTaskCore = 1;
constexpr std::size_t kSerialCommandBufferSize = 128U;
constexpr std::size_t kJsonBufferSize = 8192U;

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
  if (text == nullptr) {
    return false;
  }
  char* end = nullptr;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  value = static_cast<robot::Milliseconds>(parsed);
  return true;
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
    return {last_left_level_ == LOW ? robot::LineSample::OnTape
                                    : robot::LineSample::OffTape,
            last_right_level_ == LOW ? robot::LineSample::OnTape
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
    last_sequence_sent_ = sequence;
    last_sent_at_ms_ = command.sender_timestamp_ms;
    return healthy_;
  }

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
           elapsedSince(now_ms, last_sent_at_ms_) <= timeout_ms;
  }
  robot::Milliseconds lastSentAtMs() const { return last_sent_at_ms_; }
  robot::Milliseconds lastStatusReceivedAtMs() const {
    return last_status_received_at_ms_;
  }
  std::uint16_t lastSequenceSent() const { return last_sequence_sent_; }
  std::uint32_t packetErrorCount() const { return packet_error_count_; }
  bool statusAvailable() const { return status_available_; }
  const robot::Esp1StatusReport& latestStatus() const {
    return latest_status_;
  }

 private:
  const robot::esp2::UartConfig& config_;
  robot::UartFrameParser parser_{};
  std::uint16_t next_sequence_{0};
  std::uint16_t last_sequence_sent_{0};
  bool configured_{false};
  bool healthy_{false};
  bool status_available_{false};
  robot::Milliseconds last_sent_at_ms_{0};
  robot::Milliseconds last_status_received_at_ms_{0};
  std::uint32_t packet_error_count_{0};
  robot::Esp1StatusReport latest_status_{};
};

struct RuntimeContext {
  robot::LineFollowerConfig config{};
  robot::LineFollowerState follower_state{};
  robot::RobotTestModeManager modes{};
  robot::EventLog events{};
  robot::FourWheelCommand requested_command{};
  robot::FourWheelCommand last_commanded_wheels{};
  robot::LineFollowerUpdate last_update{};
  robot::LineObservation last_line_observation{};
  char command_buffer[kSerialCommandBufferSize]{};
  std::size_t command_length{0};
  robot::Milliseconds last_telemetry_at_ms{0};
  robot::Milliseconds mode_expires_at_ms{0};
  robot::Milliseconds last_command_ms{0};
  std::int8_t line_sensor_last_known_side{0};
  bool command_deadman_armed{false};
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
  Preferences* preferences{nullptr};
};

RuntimeBindings g_runtime{};

float hardwareDutyCap() {
  return clampFloat(robot::esp2::kHardwareConfig.maximum_safe_test_duty, 0.0F,
                    1.0F);
}

float activeMotionDutyCap(const RuntimeContext& context) {
  return clampFloat(context.config.maximumDuty, 0.0F, hardwareDutyCap());
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

void sendStoppedRearCommand(RearCommandLink& rear_link,
                            const robot::LineFollowerConfig& config,
                            const robot::Milliseconds now_ms) {
  robot::RearDriveCommand command{};
  command.enabled = false;
  command.sender_timestamp_ms = now_ms;
  command.timeout_ms = config.rearCommandTimeoutMs;
  rear_link.send(command);
}

void disableActuators(RuntimeContext& context, robot::IMotorOutput& front_left,
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

void emergencyStop(RuntimeContext& context, robot::IMotorOutput& front_left,
                   robot::IMotorOutput& front_right,
                   RearCommandLink& rear_link,
                   const robot::Milliseconds now_ms,
                   const robot::EventSource source) {
  disableActuators(context, front_left, front_right, rear_link, now_ms);
  context.modes.emergencyStop(now_ms);
  setFault(context, robot::FaultCode::None, "");
  logEvent(context, now_ms, robot::EventSeverity::Warn, source,
           "emergency stop requested");
}

bool allWheelCommandsDisabled(const robot::FourWheelCommand& command) {
  return !command.front_left.enabled && !command.front_right.enabled &&
         !command.back_left.enabled && !command.back_right.enabled;
}

bool startRequirementsMet(const DigitalFrontLineSensorReader& sensors,
                          const DualPwmMotorOutput& front_left,
                          const DualPwmMotorOutput& front_right,
                          const RearCommandLink& rear_link,
                          const RuntimeContext& context) {
  return sensors.configured() && front_left.configured() &&
         front_right.configured() && rear_link.configured() &&
         context.config.maximumDuty > 0.0F && hardwareDutyCap() > 0.0F;
}

void sendRearWheelCommand(RearCommandLink& rear_link,
                          const robot::FourWheelCommand& wheels,
                          const robot::LineFollowerConfig& config,
                          const robot::Milliseconds now_ms) {
  robot::RearDriveCommand rear{};
  rear.enabled = wheels.back_left.enabled || wheels.back_right.enabled;
  rear.back_left_command_milli = wheels.back_left.duty_command_milli;
  rear.back_right_command_milli = wheels.back_right.duty_command_milli;
  rear.sender_timestamp_ms = now_ms;
  rear.timeout_ms = config.rearCommandTimeoutMs;
  rear_link.send(rear);
}

void applyWheelCommand(RuntimeContext& context, robot::IMotorOutput& front_left,
                       robot::IMotorOutput& front_right,
                       RearCommandLink& rear_link,
                       const robot::FourWheelCommand& wheels,
                       const robot::Milliseconds now_ms) {
  context.last_commanded_wheels = wheels;
  front_left.apply(wheels.front_left);
  front_right.apply(wheels.front_right);
  sendRearWheelCommand(rear_link, wheels, context.config, now_ms);
}

robot::FourWheelCommand makeManualDriveCommand(
    const RuntimeContext& context, const float vx, const float vy,
    const float wz, const float duty, const robot::Milliseconds now_ms) {
  float fl = vy + vx + wz;
  float fr = vy - vx - wz;
  float bl = vy - vx + wz;
  float br = vy + vx - wz;
  const float peak = std::fmax(
      std::fmax(std::fabs(fl), std::fabs(fr)),
      std::fmax(std::fabs(bl), std::fabs(br)));
  if (peak > 1.0F) {
    fl /= peak;
    fr /= peak;
    bl /= peak;
    br /= peak;
  }

  const robot::Milliseconds expires_at_ms = now_ms + kCommandTimeoutMs;
  robot::FourWheelCommand command{};
  command.front_left = makeTimedMotorCommand(fl * duty, now_ms,
                                             kCommandTimeoutMs);
  command.front_right = makeTimedMotorCommand(fr * duty, now_ms,
                                              kCommandTimeoutMs);
  if (context.modes.currentMode() ==
      robot::RobotTestMode::DistributedDriveTest) {
    command.back_left = makeTimedMotorCommand(bl * duty, now_ms,
                                              kCommandTimeoutMs);
    command.back_right = makeTimedMotorCommand(br * duty, now_ms,
                                               kCommandTimeoutMs);
  }
  command.front_left.expires_at_ms = expires_at_ms;
  command.front_right.expires_at_ms = expires_at_ms;
  command.back_left.expires_at_ms = expires_at_ms;
  command.back_right.expires_at_ms = expires_at_ms;
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
      context.modes.currentMode() == robot::RobotTestMode::LineFollowTest
          ? context.last_update.observation
          : context.last_line_observation;
  snapshot.lsfl_raw_level = sensors.lastLeftLevel();
  snapshot.lsfr_raw_level = sensors.lastRightLevel();
  snapshot.lsfl_black = observation.left_black;
  snapshot.lsfr_black = observation.right_black;
  snapshot.line_error = observation.error;
  snapshot.line_visible = observation.line_visible;
  snapshot.last_known_line_side = observation.last_known_side;
  snapshot.line_follower_enabled = context.follower_state.enabled;

  snapshot.kp = context.config.kp;
  snapshot.ki = context.config.ki;
  snapshot.kd = context.config.kd;
  snapshot.base_duty = context.config.baseDuty;
  snapshot.maximum_duty = context.config.maximumDuty;
  snapshot.maximum_correction = context.config.maximumCorrection;
  snapshot.steering_polarity = context.config.steeringPolarity;
  snapshot.pid_p_term = context.last_update.pid_terms.proportional_term;
  snapshot.pid_i_term = context.last_update.pid_terms.integral_term;
  snapshot.pid_d_term = context.last_update.pid_terms.derivative_term;
  snapshot.pid_correction = context.last_update.pid_terms.correction;

  fillMotorTelemetry(snapshot.front_left, front_left);
  fillMotorTelemetry(snapshot.front_right, front_right);
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
      rear_link.healthy(now_ms, context.config.rearCommandTimeoutMs);
  snapshot.rear.esp1_link_configured = rear_link.configured();
  snapshot.rear.esp1_last_packet_age_ms =
      rear_link.lastStatusReceivedAtMs() == 0U
          ? 0U
          : elapsedSince(now_ms, rear_link.lastStatusReceivedAtMs());
  snapshot.rear.esp1_packet_error_count = rear_link.packetErrorCount();
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
    snapshot.esp1.back_left_inverted = esp1.back_left_inverted;
    snapshot.esp1.back_right_inverted = esp1.back_right_inverted;
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
         g_runtime.rear_link != nullptr;
}

robot::TelemetrySnapshot currentSnapshot() {
  robot::TelemetrySnapshot snapshot{};
  if (runtimeReady()) {
    fillTelemetrySnapshot(*g_runtime.context, *g_runtime.sensors,
                          *g_runtime.front_left, *g_runtime.front_right,
                          *g_runtime.rear_link, snapshot,
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
void handleLineFollowStart();
void handleLineFollowStop();
void handleLineFollowConfig();
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
    main { padding: 16px; display: grid; gap: 12px; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); }
    section { background: #1b2228; border: 1px solid #303a43; border-radius: 8px; padding: 12px; }
    h2 { margin: 0 0 10px; font-size: 16px; color: #dce7ee; }
    button, select, input { font: inherit; border-radius: 6px; border: 1px solid #53616d; background: #27313a; color: #fff; padding: 9px; }
    button { cursor: pointer; }
    .stop { background: #b4232f; border-color: #f05260; font-weight: 700; min-width: 120px; }
    .run { background: #1f6f53; border-color: #3aa277; }
    .warn { background: #795113; border-color: #b98325; }
    .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin: 8px 0; }
    .kv { display: grid; grid-template-columns: 1fr auto; gap: 6px 12px; font-size: 14px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .muted { color: #aeb9c2; }
    .bad { color: #ff9ca5; }
    .good { color: #8de0b8; }
    pre { white-space: pre-wrap; max-height: 300px; overflow: auto; background: #0b0e10; padding: 10px; border-radius: 6px; }
    input[type=number] { width: 86px; }
  </style>
</head>
<body>
<header>
  <div>
    <h1>Team14 Robot Test Interface</h1>
    <div class="muted">TEST ONLY bench telemetry and tuning</div>
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
    <div class="row">
      <select id="modePick">
        <option value="disabled">DISABLED</option>
        <option value="sensors">SENSOR_MONITOR</option>
        <option value="single-motor">SINGLE_MOTOR_TEST</option>
        <option value="manual-drive">MANUAL_DRIVE_TEST</option>
        <option value="distributed-drive">DISTRIBUTED_DRIVE_TEST</option>
        <option value="line-sensor">LINE_SENSOR_TEST</option>
        <option value="line-follow">LINE_FOLLOW_TEST</option>
        <option value="mechanism">MECHANISM_TEST</option>
        <option value="autonomous-dry-run">AUTONOMOUS_DRY_RUN</option>
      </select>
      <button onclick="setMode()">Set Mode</button>
    </div>
  </section>

  <section>
    <h2>Manual Drive</h2>
    <div class="row">Duty <input id="driveDuty" type="number" min="0" max="1" step="0.01" value="0.10"></div>
    <div class="grid">
      <span></span><button onpointerdown="drive(0,1,0)" onpointerup="stopAll()">FWD</button><span></span>
      <button onpointerdown="drive(-1,0,0)" onpointerup="stopAll()">LEFT</button><button class="stop" onclick="stopAll()">STOP</button><button onpointerdown="drive(1,0,0)" onpointerup="stopAll()">RIGHT</button>
      <button onpointerdown="drive(0,0,-1)" onpointerup="stopAll()">CCW</button><button onpointerdown="drive(0,-1,0)" onpointerup="stopAll()">BACK</button><button onpointerdown="drive(0,0,1)" onpointerup="stopAll()">CW</button>
    </div>
  </section>

  <section>
    <h2>Single Motor</h2>
    <div class="row">
      Wheel <select id="motorId"><option>FL</option><option>FR</option><option>BL</option><option>BR</option></select>
      Speed <input id="motorSpeed" type="number" step="0.01" value="0.10">
    </div>
    <div class="row">
      <button class="run" onpointerdown="motorHold()" onpointerup="motorRelease()" onpointerleave="motorRelease()" onpointercancel="motorRelease()">Hold To Spin</button>
      <button class="warn" onclick="invert()">Invert</button>
    </div>
    <pre id="motors"></pre>
  </section>

  <section>
    <h2>Line Sensors</h2>
    <div class="kv">
      <span>LSFL level</span><span id="lsfl" class="mono"></span>
      <span>LSFR level</span><span id="lsfr" class="mono"></span>
      <span>Left black</span><span id="lb" class="mono"></span>
      <span>Right black</span><span id="rb" class="mono"></span>
      <span>Error</span><span id="err" class="mono"></span>
      <span>Visible</span><span id="vis" class="mono"></span>
    </div>
  </section>

  <section>
    <h2>Line Follow</h2>
    <div class="row">
      Kp <input id="kp" type="number" step="0.01">
      Ki <input id="ki" type="number" step="0.01">
      Kd <input id="kd" type="number" step="0.01">
    </div>
    <div class="row">
      Base <input id="base" type="number" step="0.01">
      Max <input id="maxDuty" type="number" step="0.01">
      Corr <input id="maxCorr" type="number" step="0.01">
      Pol <input id="pol" type="number" step="1">
    </div>
    <div class="row">
      <button onclick="saveLf()">Apply PID</button>
      <button class="run" onclick="lfStart()">Start 5s</button>
      <button onclick="lfStop()">Stop LF</button>
    </div>
    <pre id="pid"></pre>
  </section>

  <section>
    <h2>Sensors / Future Mechanisms</h2>
    <pre id="future"></pre>
  </section>

  <section>
    <h2>Events</h2>
    <pre id="events"></pre>
  </section>

  <section>
    <h2>Raw Telemetry</h2>
    <pre id="raw"></pre>
  </section>
</main>
<script>
let holdTimer = null;
let lastPidFill = false;
function qs(id){ return document.getElementById(id); }
function api(path){ return fetch(path).then(r => r.json().catch(() => ({})).then(j => ({ok:r.ok, status:r.status, json:j}))); }
function stopAll(){ if (holdTimer) clearInterval(holdTimer); holdTimer=null; api('/api/stop'); }
function setMode(){ api('/api/mode?mode=' + encodeURIComponent(qs('modePick').value)); }
function drive(vx,vy,wz){
  const duty = Number(qs('driveDuty').value || 0);
  const send = () => api(`/api/drive?vx=${vx}&vy=${vy}&wz=${wz}&duty=${duty}`);
  send(); if (holdTimer) clearInterval(holdTimer); holdTimer=setInterval(send, 200);
}
function motorCommand(speed){ api(`/api/motor?id=${qs('motorId').value}&speed=${speed}`); }
function motorHold(){
  const send = () => motorCommand(qs('motorSpeed').value);
  send(); if (holdTimer) clearInterval(holdTimer); holdTimer=setInterval(send, 200);
}
function motorRelease(){ if (holdTimer) clearInterval(holdTimer); holdTimer=null; motorCommand(0); }
function invert(){ api(`/api/invert?id=${qs('motorId').value}`); }
function saveLf(){
  api(`/api/line-follow/config?kp=${qs('kp').value}&ki=${qs('ki').value}&kd=${qs('kd').value}&base=${qs('base').value}&max=${qs('maxDuty').value}&max-correction=${qs('maxCorr').value}&polarity=${qs('pol').value}`);
}
function lfStart(){ api('/api/line-follow/start?ms=5000'); }
function lfStop(){ api('/api/line-follow/stop'); }
function yn(v){ return v ? 'yes' : 'no'; }
function update(){
  fetch('/api/telemetry').then(r => r.json()).then(j => {
    qs('mode').textContent = j.current_mode;
    qs('fault').textContent = j.fault_active ? `${j.fault_code}: ${j.fault_message}` : 'none';
    qs('fault').className = j.fault_active ? 'mono bad' : 'mono good';
    qs('link').textContent = `${yn(j.rear.esp1_link_healthy)} configured=${yn(j.rear.esp1_link_configured)}`;
    qs('uptime').textContent = `${j.uptime_ms} ms`;
    qs('ap').textContent = `${j.ip_address} clients=${j.wifi_clients}`;
    qs('deadman').textContent = `${j.last_command_age_ms} ms, ${j.deadman_remaining_ms} ms left`;
    qs('lsfl').textContent = j.line.lsfl_raw_level;
    qs('lsfr').textContent = j.line.lsfr_raw_level;
    qs('lb').textContent = yn(j.line.lsfl_black);
    qs('rb').textContent = yn(j.line.lsfr_black);
    qs('err').textContent = j.line.line_error;
    qs('vis').textContent = yn(j.line.line_visible);
    qs('motors').textContent = JSON.stringify({motors:j.motors, rear:j.rear}, null, 2);
    qs('pid').textContent = JSON.stringify(j.pid, null, 2);
    qs('future').textContent = JSON.stringify(j.future, null, 2);
    qs('raw').textContent = JSON.stringify(j, null, 2);
    if (!lastPidFill) {
      qs('kp').value = j.pid.kp; qs('ki').value = j.pid.ki; qs('kd').value = j.pid.kd;
      qs('base').value = j.pid.baseDuty; qs('maxDuty').value = j.pid.maximumDuty;
      qs('maxCorr').value = j.pid.maximumCorrection; qs('pol').value = j.pid.steeringPolarity;
      lastPidFill = true;
    }
  }).catch(() => { qs('raw').textContent = 'telemetry disconnected'; });
  fetch('/api/events').then(r => r.json()).then(j => { qs('events').textContent = JSON.stringify(j.events, null, 2); });
}
setInterval(update, 300); update();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  g_server.send_P(200, "text/html", kDashboardHtml);
}

void handleStatus() {
  const robot::TelemetrySnapshot snapshot = currentSnapshot();
  if (!robot::writeTelemetryJson(snapshot, g_json_buffer,
                                 sizeof(g_json_buffer), true)) {
    sendErrorJson(500, "telemetry json overflow");
    return;
  }
  g_server.send(200, "application/json", g_json_buffer);
}

void handleTelemetry() {
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

  const robot::CommandValidationResult validation =
      robot::validateDriveCommand(context.modes.currentMode(), vx, vy, wz,
                                  duty, validationLimits(context));
  if (!validation.accepted) {
    disableActuators(context, *g_runtime.front_left, *g_runtime.front_right,
                     *g_runtime.rear_link,
                     static_cast<robot::Milliseconds>(millis()));
    setFault(context, robot::FaultCode::InvalidCommand, validation.reason);
    logEvent(context, static_cast<robot::Milliseconds>(millis()),
             robot::EventSeverity::Warn, robot::EventSource::Web,
             validation.reason);
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

  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
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
  robot::WheelId wheel{};
  const String id_arg = g_server.arg("id");
  float speed = 0.0F;
  if (!robot::parseWheelId(id_arg.c_str(), wheel) ||
      !argFloat("speed", speed, 0.0F, true)) {
    sendErrorJson(400, "malformed motor command");
    return;
  }

  const robot::CommandValidationResult validation =
      robot::validateSingleMotorCommand(context.modes.currentMode(), speed,
                                        kCommandTimeoutMs,
                                        validationLimits(context));
  if (!validation.accepted) {
    setFault(context, robot::FaultCode::InvalidCommand, validation.reason);
    logEvent(context, static_cast<robot::Milliseconds>(millis()),
             robot::EventSeverity::Warn, robot::EventSource::Web,
             validation.reason);
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

  const robot::Milliseconds now_ms =
      static_cast<robot::Milliseconds>(millis());
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
                "\"lsfl_black\":%s,\"lsfr_black\":%s,"
                "\"limit_switch_stepper_bottom\":%s,"
                "\"limit_switch_stepper_middle\":%s,"
                "\"limit_switch_stepper_top\":%s,"
                "\"limit_switch_funnel_left\":%s,"
                "\"limit_switch_funnel_right\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                snapshot.lsfl_black ? "true" : "false",
                snapshot.lsfr_black ? "true" : "false",
                snapshot.limit_switch_stepper_bottom ? "true" : "false",
                snapshot.limit_switch_stepper_middle ? "true" : "false",
                snapshot.limit_switch_stepper_top ? "true" : "false",
                snapshot.limit_switch_funnel_left ? "true" : "false",
                snapshot.limit_switch_funnel_right ? "true" : "false");
  g_server.send(200, "application/json", g_json_buffer);
}

void handleLine() {
  const robot::TelemetrySnapshot snapshot = currentSnapshot();
  std::snprintf(g_json_buffer, sizeof(g_json_buffer),
                "{\"LSFL\":%d,\"LSFR\":%d,\"leftBlack\":%s,"
                "\"rightBlack\":%s,\"error\":%d,\"lastKnownSide\":%d,"
                "\"lineVisible\":%s}",
                snapshot.lsfl_raw_level, snapshot.lsfr_raw_level,
                snapshot.lsfl_black ? "true" : "false",
                snapshot.lsfr_black ? "true" : "false",
                static_cast<int>(snapshot.line_error),
                static_cast<int>(snapshot.last_known_line_side),
                snapshot.line_visible ? "true" : "false");
  g_server.send(200, "application/json", g_json_buffer);
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
  if (context.modes.currentMode() != robot::RobotTestMode::LineFollowTest) {
    sendErrorJson(409, "line follower requires LINE_FOLLOW_TEST mode");
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
  if (!startRequirementsMet(*g_runtime.sensors, *g_runtime.front_left,
                            *g_runtime.front_right, *g_runtime.rear_link,
                            context)) {
    setFault(context, robot::FaultCode::HardwareNotConfigured,
             "line follower hardware requirements are incomplete");
    logEvent(context, now_ms, robot::EventSeverity::Fault,
             robot::EventSource::Line,
             "line follower start rejected: hardware incomplete");
    sendErrorJson(409, "configure sensors, motors, UART, max-duty, hardware cap");
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
    if (!argFloat("max", value, next.maximumDuty, true)) {
      sendErrorJson(400, "malformed max");
      return;
    }
    next.maximumDuty = value;
  } else if (g_server.hasArg("maximumDuty")) {
    if (!argFloat("maximumDuty", value, next.maximumDuty, true)) {
      sendErrorJson(400, "malformed maximumDuty");
      return;
    }
    next.maximumDuty = value;
  }
  if (g_server.hasArg("max-correction")) {
    if (!argFloat("max-correction", value, next.maximumCorrection, true)) {
      sendErrorJson(400, "malformed max-correction");
      return;
    }
    next.maximumCorrection = value;
  } else if (g_server.hasArg("maximumCorrection")) {
    if (!argFloat("maximumCorrection", value, next.maximumCorrection, true)) {
      sendErrorJson(400, "malformed maximumCorrection");
      return;
    }
    next.maximumCorrection = value;
  }
  if (g_server.hasArg("polarity")) {
    if (!argFloat("polarity", value,
                  static_cast<float>(next.steeringPolarity), true)) {
      sendErrorJson(400, "malformed polarity");
      return;
    }
    next.steeringPolarity = value < 0.0F ? -1 : 1;
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
                "\"baseDuty\":%.5f,\"maximumDuty\":%.5f,"
                "\"maximumCorrection\":%.5f,\"steeringPolarity\":%d,"
                "\"hardwareDutyCap\":%.5f,\"singleMotorDutyCap\":%.5f}",
                snapshot.kp, snapshot.ki, snapshot.kd, snapshot.base_duty,
                snapshot.maximum_duty, snapshot.maximum_correction,
                snapshot.steering_polarity, hardwareDutyCap(),
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
  g_runtime.preferences->putFloat("max", context.config.maximumDuty);
  g_runtime.preferences->putFloat("corr", context.config.maximumCorrection);
  g_runtime.preferences->putInt("pol", context.config.steeringPolarity);
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
  g_server.on("/api/line-follow/start", HTTP_ANY, handleLineFollowStart);
  g_server.on("/api/line-follow/stop", HTTP_ANY, handleLineFollowStop);
  g_server.on("/api/line-follow/config", HTTP_ANY, handleLineFollowConfig);
  g_server.on("/api/config", HTTP_GET, handleConfig);
  g_server.on("/api/config/save", HTTP_ANY, handleConfigSave);
  g_server.on("/api/events", HTTP_GET, handleEvents);
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
  Serial.print(context.config.maximumDuty, 4);
  Serial.print(", hardware-cap=");
  Serial.print(hardwareDutyCap(), 4);
  Serial.print(", max-correction=");
  Serial.print(context.config.maximumCorrection, 4);
  Serial.print(", polarity=");
  Serial.print(context.config.steeringPolarity);
  Serial.print(", sensors-configured=");
  Serial.print(sensors.configured() ? 1 : 0);
  Serial.print(", front-left-configured=");
  Serial.print(front_left.configured() ? 1 : 0);
  Serial.print(", front-right-configured=");
  Serial.print(front_right.configured() ? 1 : 0);
  Serial.print(", rear-link-configured=");
  Serial.print(rear_link.configured() ? 1 : 0);
  Serial.print(", rear-link-healthy=");
  Serial.println(rear_link.healthy(now_ms, context.config.rearCommandTimeoutMs)
                     ? 1
                     : 0);
}

void printCommands() {
  Serial.println("commands:");
  Serial.println("  help | status | stop");
  Serial.println("  mode disabled|sensors|single-motor|manual-drive|distributed-drive|line-sensor|line-follow|mechanism|autonomous-dry-run");
  Serial.println("  sensor status | line status");
  Serial.println("  motor test FL|FR|BL|BR <speed -1..1> <ms>");
  Serial.println("  motor invert FL|FR|BL|BR");
  Serial.println("  drive fwd|back|left|right|cw|ccw <duty> <ms>");
  Serial.println("  lf start [ms] | lf stop | lf status | lf kp|ki|kd <v>");
  Serial.println("  lf base <v> | lf speed <v> | lf max-duty <v> | lf max-correction <v> | lf polarity <1|-1> | lf telemetry on|off");
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
  clearFault(context);
  logEvent(context, now_ms, robot::EventSeverity::Info,
           robot::EventSource::Serial, "mode changed");
  printOk("mode changed");
  return true;
}

bool updateTuningValue(RuntimeContext& context, const char* name,
                       const char* value_text) {
  float value = 0.0F;
  if (!parseFloat(value_text, value)) {
    printRejected("malformed tuning value");
    return false;
  }
  robot::LineFollowerConfig next = context.config;
  if (std::strcmp(name, "kp") == 0) {
    next.kp = value;
  } else if (std::strcmp(name, "ki") == 0) {
    next.ki = value;
  } else if (std::strcmp(name, "kd") == 0) {
    next.kd = value;
  } else if (std::strcmp(name, "base") == 0 ||
             std::strcmp(name, "speed") == 0) {
    next.baseDuty = value;
  } else if (std::strcmp(name, "max-duty") == 0) {
    next.maximumDuty = value;
  } else if (std::strcmp(name, "max-correction") == 0) {
    next.maximumCorrection = value;
  } else if (std::strcmp(name, "polarity") == 0) {
    next.steeringPolarity = value < 0.0F ? -1 : 1;
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

void printLineStatus(const RuntimeContext& context) {
  const robot::LineObservation& observation = context.last_line_observation;
  Serial.print("line LSFL=");
  Serial.print(observation.left_black ? "black" : "white");
  Serial.print(", LSFR=");
  Serial.print(observation.right_black ? "black" : "white");
  Serial.print(", error=");
  Serial.print(observation.error);
  Serial.print(", visible=");
  Serial.print(observation.line_visible ? 1 : 0);
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
  if (std::strcmp(token, "sensor") == 0) {
    char* command = strtok(nullptr, " \t\r\n");
    if (command != nullptr && std::strcmp(command, "status") == 0) {
      printLineStatus(context);
    } else {
      printRejected("sensor status");
    }
    return;
  }
  if (std::strcmp(token, "line") == 0) {
    char* command = strtok(nullptr, " \t\r\n");
    if (command != nullptr && std::strcmp(command, "status") == 0) {
      printLineStatus(context);
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
      if (context.modes.currentMode() !=
          robot::RobotTestMode::LineFollowTest) {
        printRejected("line follower requires LINE_FOLLOW_TEST mode");
        return;
      }
      if (!startRequirementsMet(sensors, front_left, front_right, rear_link,
                                context)) {
        setFault(context, robot::FaultCode::HardwareNotConfigured,
                 "line follower hardware requirements are incomplete");
        printRejected("configure sensors, motors, UART, max-duty, hardware cap");
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
    if (std::strcmp(command, "telemetry") == 0) {
      char* value = strtok(nullptr, " \t\r\n");
      if (value == nullptr ||
          !(std::strcmp(value, "on") == 0 || std::strcmp(value, "off") == 0)) {
        printRejected("telemetry must be on or off");
        return;
      }
      context.config.telemetryEnabled = std::strcmp(value, "on") == 0;
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

void printTelemetry(RuntimeContext& context, const RearCommandLink& rear_link,
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

  Serial.print("lf_csv,");
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(observation.left_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.right_black ? 1 : 0);
  Serial.print(',');
  Serial.print(observation.error);
  Serial.print(',');
  Serial.print(observation.line_visible ? 1 : 0);
  Serial.print(',');
  Serial.print(update.pid_terms.proportional_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.integral_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.derivative_term, 4);
  Serial.print(',');
  Serial.print(update.pid_terms.correction, 4);
  Serial.print(',');
  Serial.print(wheels.front_left.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.front_right.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.back_left.duty_command_milli);
  Serial.print(',');
  Serial.print(wheels.back_right.duty_command_milli);
  Serial.print(',');
  Serial.print(rear_age);
  Serial.print(',');
  Serial.println(rear_link.healthy(now_ms, context.config.rearCommandTimeoutMs)
                     ? 1
                     : 0);
}

void runSensorOnlyTelemetry(RuntimeContext& context,
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
                     DualPwmMotorOutput& front_right) {
  front_left.setRuntimeInverted(preferences.getBool("inv_fl", false));
  front_right.setRuntimeInverted(preferences.getBool("inv_fr", false));
  context.config.kp = preferences.getFloat("kp", context.config.kp);
  context.config.ki = preferences.getFloat("ki", context.config.ki);
  context.config.kd = preferences.getFloat("kd", context.config.kd);
  context.config.baseDuty =
      preferences.getFloat("base", context.config.baseDuty);
  context.config.maximumDuty =
      preferences.getFloat("max", context.config.maximumDuty);
  context.config.maximumCorrection =
      preferences.getFloat("corr", context.config.maximumCorrection);
  context.config.steeringPolarity =
      preferences.getInt("pol", context.config.steeringPolarity) < 0 ? -1 : 1;

  const robot::CommandValidationResult validation =
      robot::validateLineFollowerConfig(context.config, hardwareDutyCap());
  if (!validation.accepted) {
    const float cap = hardwareDutyCap();
    context.config = {};
    context.config.maximumDuty = cap;
    context.config.maximumCorrection =
        clampFloat(context.config.maximumCorrection, 0.0F, cap);
  }
}

void motionControlTask(void* parameters) {
  (void)parameters;

  RuntimeContext context{};
  const float cap = hardwareDutyCap();
  context.config.maximumDuty = cap;
  context.config.maximumCorrection =
      clampFloat(context.config.maximumCorrection, 0.0F, cap);

  DigitalFrontLineSensorReader line_sensor_reader{
      robot::esp2::kHardwareConfig.pins};
  DualPwmMotorOutput front_left_motor{
      robot::esp2::kHardwareConfig.front_left_motor};
  DualPwmMotorOutput front_right_motor{
      robot::esp2::kHardwareConfig.front_right_motor};
  RearCommandLink rear_link{robot::esp2::kHardwareConfig.uart_to_esp1};
  robot::esp2::StepperController stepper{};
  robot::esp2::ServoBankController servos{};
  Preferences preferences{};

  line_sensor_reader.initialize();
  front_left_motor.initializeDisabled();
  front_right_motor.initializeDisabled();
  rear_link.initialize();
  stepper.initializeDisabled();
  servos.initializeDisabled();

  preferences.begin("telemetry", false);
  loadPreferences(preferences, context, front_left_motor, front_right_motor);

  g_runtime = {&context, &line_sensor_reader, &front_left_motor,
               &front_right_motor, &rear_link, &preferences};

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
    rear_link.pollReceive(now_ms);
    refreshLineObservation(context, line_sensor_reader, now_ms);
    pollSerialCommands(context, line_sensor_reader, front_left_motor,
                       front_right_motor, rear_link, now_ms);

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
    if (robot::robotTestModeIsSensorOnly(mode)) {
      disableActuators(context, front_left_motor, front_right_motor, rear_link,
                       now_ms);
      if (mode == robot::RobotTestMode::SensorMonitor ||
          mode == robot::RobotTestMode::LineSensorTest) {
        runSensorOnlyTelemetry(context, now_ms);
      }
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
        if (!rear_link.healthy(now_ms, context.config.rearCommandTimeoutMs)) {
          disableActuators(context, front_left_motor, front_right_motor,
                           rear_link, now_ms);
          setFault(context, robot::FaultCode::CommunicationStale,
                   "line follower stopped: rear link unhealthy");
          logEvent(context, now_ms, robot::EventSeverity::Fault,
                   robot::EventSource::Uart,
                   "line follower stopped: rear link unhealthy");
        }
        printTelemetry(context, rear_link, now_ms);
      }
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
