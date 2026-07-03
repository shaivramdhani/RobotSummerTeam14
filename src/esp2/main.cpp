#include <Arduino.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "common/ChassisMixer.h"
#include "common/LineFollower.h"
#include "common/LineObservation.h"
#include "common/LineSensor.h"
#include "common/MotorOutput.h"
#include "common/RearDriveCommand.h"
#include "common/UartProtocol.h"
#include "esp2/MechanismControllers.h"
#include "esp2/PinConfig.h"

namespace {

constexpr robot::Milliseconds kDefaultMotionTaskPeriodMs = 10U;
constexpr robot::Milliseconds kTelemetryPeriodMs = 100U;
constexpr robot::Milliseconds kHardwareTestTimeoutMs = 750U;
constexpr std::uint32_t kTaskStackBytes = 8192U;
constexpr UBaseType_t kTaskPriority = 1U;
constexpr BaseType_t kTaskCore = 1;
constexpr std::size_t kSerialCommandBufferSize = 96U;

bool gpioAssigned(const int gpio) {
  return gpio >= 0;
}

float clampFloat(const float value, const float minimum, const float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool parseFloat(const char* text, float& value) {
  if (text == nullptr) {
    return false;
  }
  char* end = nullptr;
  const float parsed = strtof(text, &end);
  if (end == text || *end != '\0') {
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
      return {robot::LineSample::Unknown, robot::LineSample::Unknown, now_ms,
              false};
    }

    const bool left_black = digitalRead(pins_.line_sensor_front_left) == LOW;
    const bool right_black = digitalRead(pins_.line_sensor_front_right) == LOW;
    return {left_black ? robot::LineSample::OnTape
                       : robot::LineSample::OffTape,
            right_black ? robot::LineSample::OnTape
                        : robot::LineSample::OffTape,
            now_ms, true};
  }

  bool configured() const { return configured_; }

 private:
  const robot::esp2::Esp2Pins& pins_;
  bool configured_{false};
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
        robot::esp2::DualPwmHBridgeMode::Pwm0ForwardPwm1Reverse;
    const bool use_pwm0 = positive ? pwm0_forward : !pwm0_forward;

    ledcWrite(config_.pwm0_channel, use_pwm0 ? pwm_duty : 0U);
    ledcWrite(config_.pwm1_channel, use_pwm0 ? 0U : pwm_duty);
    last_command_ = command;
  }

  void disable() override {
    if (configured_) {
      ledcWrite(config_.pwm0_channel, 0U);
      ledcWrite(config_.pwm1_channel, 0U);
    }
    last_command_ = robot::disabledMotorCommand();
  }

  bool configured() const { return configured_; }

 private:
  const robot::esp2::DualPwmMotorOutputConfig& config_;
  bool configured_{false};
  robot::MotorCommand last_command_{};
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

    const robot::UartPacket packet =
        robot::makeRearDriveCommandPacket(command, sequence_++);
    std::uint8_t frame[robot::kUartFrameOverheadSize +
                       robot::kUartMaxPayloadSize]{};
    std::size_t frame_size = 0U;
    if (!robot::encodeUartFrame(packet, frame, sizeof(frame), frame_size)) {
      healthy_ = false;
      return false;
    }

    const std::size_t written = Serial1.write(frame, frame_size);
    healthy_ = written == frame_size;
    last_sent_at_ms_ = command.sender_timestamp_ms;
    return healthy_;
  }

  bool healthy() const { return healthy_; }
  bool configured() const { return configured_; }
  robot::Milliseconds lastSentAtMs() const { return last_sent_at_ms_; }

 private:
  const robot::esp2::UartConfig& config_;
  std::uint16_t sequence_{0};
  bool configured_{false};
  bool healthy_{false};
  robot::Milliseconds last_sent_at_ms_{0};
};

enum class OperationMode : std::uint8_t {
  Idle,
  SensorOnly,
  MotorDirectionTest,
  DistributedDriveTest,
  LineFollowing,
};

struct RuntimeContext {
  robot::LineFollowerConfig config{};
  robot::LineFollowerState follower_state{};
  OperationMode mode{OperationMode::Idle};
  robot::Milliseconds mode_expires_at_ms{0};
  robot::FourWheelCommand test_command{};
  robot::LineFollowerUpdate last_update{};
  char command_buffer[kSerialCommandBufferSize]{};
  std::size_t command_length{0};
  robot::Milliseconds last_telemetry_at_ms{0};
};

float hardwareDutyCap() {
  return clampFloat(robot::esp2::kHardwareConfig.maximum_safe_test_duty, 0.0F,
                    1.0F);
}

bool dutyWithinConfiguredLimits(const RuntimeContext& context,
                                const float duty) {
  const float absolute_duty = std::fabs(duty);
  return absolute_duty <= context.config.maximumDuty &&
         absolute_duty <= hardwareDutyCap();
}

robot::MotorCommand makeTimedMotorCommand(const float duty,
                                          const robot::Milliseconds now_ms) {
  robot::MotorCommand command{};
  command.enabled = std::fabs(duty) > 0.0001F;
  command.duty_command_milli =
      robot::clampCommandMilli(static_cast<std::int16_t>(duty * 1000.0F));
  command.expires_at_ms = now_ms + kHardwareTestTimeoutMs;
  return command;
}

void printRejected(const char* reason) {
  Serial.print("rejected: ");
  Serial.println(reason);
}

void printOk(const char* message) {
  Serial.print("ok: ");
  Serial.println(message);
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

void stopAll(RuntimeContext& context, robot::IMotorOutput& front_left,
             robot::IMotorOutput& front_right, RearCommandLink& rear_link,
             const robot::Milliseconds now_ms) {
  robot::stopLineFollower(context.follower_state);
  context.mode = OperationMode::Idle;
  context.test_command = robot::disabledFourWheelCommand();
  front_left.disable();
  front_right.disable();
  sendStoppedRearCommand(rear_link, context.config, now_ms);
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

void printStatus(const RuntimeContext& context, const RearCommandLink& rear_link,
                 const DigitalFrontLineSensorReader& sensors,
                 const DualPwmMotorOutput& front_left,
                 const DualPwmMotorOutput& front_right) {
  Serial.print("lf enabled=");
  Serial.print(context.follower_state.enabled ? 1 : 0);
  Serial.print(", mode=");
  Serial.print(static_cast<int>(context.mode));
  Serial.print(", kp=");
  Serial.print(context.config.kp, 4);
  Serial.print(", ki=");
  Serial.print(context.config.ki, 4);
  Serial.print(", kd=");
  Serial.print(context.config.kd, 4);
  Serial.print(", speed=");
  Serial.print(context.config.baseDuty, 4);
  Serial.print(", max-duty=");
  Serial.print(context.config.maximumDuty, 4);
  Serial.print(", hardware-cap=");
  Serial.print(hardwareDutyCap(), 4);
  Serial.print(", max-correction=");
  Serial.print(context.config.maximumCorrection, 4);
  Serial.print(", integral-limit=");
  Serial.print(context.config.integralLimit, 4);
  Serial.print(", derivative-limit=");
  Serial.print(context.config.derivativeLimit, 4);
  Serial.print(", derivative-filter=");
  Serial.print(context.config.derivativeFilterCoefficient, 4);
  Serial.print(", polarity=");
  Serial.print(context.config.steeringPolarity);
  Serial.print(", period-ms=");
  Serial.print(context.config.controlPeriodMs);
  Serial.print(", rear-timeout-ms=");
  Serial.print(context.config.rearCommandTimeoutMs);
  Serial.print(", initial-search-timeout-ms=");
  Serial.print(context.config.initialLineSearchTimeoutMs);
  Serial.print(", telemetry=");
  Serial.print(context.config.telemetryEnabled ? 1 : 0);
  Serial.print(", sensors-configured=");
  Serial.print(sensors.configured() ? 1 : 0);
  Serial.print(", front-left-configured=");
  Serial.print(front_left.configured() ? 1 : 0);
  Serial.print(", front-right-configured=");
  Serial.print(front_right.configured() ? 1 : 0);
  Serial.print(", rear-link-configured=");
  Serial.print(rear_link.configured() ? 1 : 0);
  Serial.print(", rear-link-healthy=");
  Serial.println(rear_link.healthy() ? 1 : 0);
}

void printCommands() {
  Serial.println(
      "commands: lf start | lf stop | lf status | lf kp <v> | lf ki <v> | "
      "lf kd <v> | lf speed <duty> | lf max-duty <duty> | "
      "lf max-correction <v> | lf polarity <1|-1> | lf period-ms <ms> | "
      "lf telemetry <on|off> | lf reset | lf test sensor | "
      "lf test motor <fl|fr|bl|br> <duty> | lf test drive <duty> | "
      "lf test stop");
}

bool applyTuningCommand(RuntimeContext& context, const char* name,
                        const char* value) {
  float parsed = 0.0F;
  if (strcmp(name, "kp") == 0 || strcmp(name, "ki") == 0 ||
      strcmp(name, "kd") == 0) {
    if (!parseFloat(value, parsed) || parsed < 0.0F || parsed > 20.0F) {
      printRejected("gain must be in [0, 20]");
      return true;
    }
    if (strcmp(name, "kp") == 0) {
      context.config.kp = parsed;
    } else if (strcmp(name, "ki") == 0) {
      context.config.ki = parsed;
    } else {
      context.config.kd = parsed;
    }
    printOk(name);
    return true;
  }

  if (strcmp(name, "speed") == 0) {
    if (!parseFloat(value, parsed) || !dutyWithinConfiguredLimits(context, parsed)) {
      printRejected("speed exceeds max-duty or hardware cap");
      return true;
    }
    context.config.baseDuty = parsed;
    printOk("speed");
    return true;
  }

  if (strcmp(name, "max-duty") == 0) {
    if (!parseFloat(value, parsed) || parsed < 0.0F ||
        parsed > hardwareDutyCap()) {
      printRejected("max-duty exceeds verified hardware cap");
      return true;
    }
    context.config.maximumDuty = parsed;
    if (std::fabs(context.config.baseDuty) > parsed) {
      context.config.baseDuty =
          context.config.baseDuty < 0.0F ? -parsed : parsed;
    }
    if (context.config.maximumCorrection > parsed) {
      context.config.maximumCorrection = parsed;
    }
    printOk("max-duty");
    return true;
  }

  if (strcmp(name, "max-correction") == 0) {
    if (!parseFloat(value, parsed) || parsed < 0.0F ||
        parsed > context.config.maximumDuty) {
      printRejected("max-correction must be in [0, max-duty]");
      return true;
    }
    context.config.maximumCorrection = parsed;
    printOk("max-correction");
    return true;
  }

  if (strcmp(name, "polarity") == 0) {
    if (value == nullptr ||
        !(strcmp(value, "1") == 0 || strcmp(value, "-1") == 0)) {
      printRejected("polarity must be 1 or -1");
      return true;
    }
    context.config.steeringPolarity = strcmp(value, "-1") == 0 ? -1 : 1;
    printOk("polarity");
    return true;
  }

  if (strcmp(name, "period-ms") == 0) {
    robot::Milliseconds period_ms = 0U;
    if (!parseUnsigned(value, period_ms) || period_ms < 5U ||
        period_ms > 100U) {
      printRejected("period-ms must be in [5, 100]");
      return true;
    }
    context.config.controlPeriodMs = period_ms;
    printOk("period-ms");
    return true;
  }

  return false;
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
  if (strcmp(token, "help") == 0) {
    printCommands();
    return;
  }
  if (strcmp(token, "lf") != 0) {
    printRejected("commands must start with lf");
    return;
  }

  char* command = strtok(nullptr, " \t\r\n");
  if (command == nullptr) {
    printCommands();
    return;
  }

  if (strcmp(command, "start") == 0) {
    if (!startRequirementsMet(sensors, front_left, front_right, rear_link,
                              context)) {
      printRejected("configure sensors, motors, UART, max-duty, hardware cap");
      return;
    }
    robot::startLineFollower(context.follower_state, now_ms);
    context.mode = OperationMode::LineFollowing;
    printOk("line follower started");
    return;
  }

  if (strcmp(command, "stop") == 0) {
    stopAll(context, front_left, front_right, rear_link, now_ms);
    printOk("stopped");
    return;
  }

  if (strcmp(command, "status") == 0) {
    printStatus(context, rear_link, sensors, front_left, front_right);
    return;
  }

  if (strcmp(command, "telemetry") == 0) {
    char* value = strtok(nullptr, " \t\r\n");
    if (value == nullptr ||
        !(strcmp(value, "on") == 0 || strcmp(value, "off") == 0)) {
      printRejected("telemetry must be on or off");
      return;
    }
    context.config.telemetryEnabled = strcmp(value, "on") == 0;
    printOk("telemetry");
    return;
  }

  if (strcmp(command, "reset") == 0) {
    const float cap = hardwareDutyCap();
    context.config = {};
    context.config.maximumDuty = cap;
    context.config.maximumCorrection =
        cap < context.config.maximumCorrection ? cap
                                               : context.config.maximumCorrection;
    robot::resetLineFollowerState(context.follower_state);
    printOk("reset");
    return;
  }

  if (strcmp(command, "test") == 0) {
    char* test_name = strtok(nullptr, " \t\r\n");
    if (test_name == nullptr) {
      printRejected("missing test name");
      return;
    }

    if (strcmp(test_name, "stop") == 0) {
      stopAll(context, front_left, front_right, rear_link, now_ms);
      printOk("test stopped");
      return;
    }

    if (strcmp(test_name, "sensor") == 0) {
      stopAll(context, front_left, front_right, rear_link, now_ms);
      if (!sensors.configured()) {
        printRejected("LSFL/LSFR GPIOs are not configured");
        return;
      }
      context.mode = OperationMode::SensorOnly;
      printOk("sensor-only mode");
      return;
    }

    if (strcmp(test_name, "motor") == 0) {
      char* wheel = strtok(nullptr, " \t\r\n");
      char* duty_text = strtok(nullptr, " \t\r\n");
      float duty = 0.0F;
      if (wheel == nullptr || !parseFloat(duty_text, duty) ||
          duty < 0.0F || !dutyWithinConfiguredLimits(context, duty)) {
        printRejected("motor test needs wheel and positive limited duty");
        return;
      }
      stopAll(context, front_left, front_right, rear_link, now_ms);
      context.test_command = robot::disabledFourWheelCommand();
      if (strcmp(wheel, "fl") == 0) {
        if (!front_left.configured()) {
          printRejected("front-left motor is not configured");
          return;
        }
        context.test_command.front_left = makeTimedMotorCommand(duty, now_ms);
      } else if (strcmp(wheel, "fr") == 0) {
        if (!front_right.configured()) {
          printRejected("front-right motor is not configured");
          return;
        }
        context.test_command.front_right = makeTimedMotorCommand(duty, now_ms);
      } else if (strcmp(wheel, "bl") == 0) {
        if (!rear_link.configured()) {
          printRejected("rear UART is not configured");
          return;
        }
        context.test_command.back_left = makeTimedMotorCommand(duty, now_ms);
      } else if (strcmp(wheel, "br") == 0) {
        if (!rear_link.configured()) {
          printRejected("rear UART is not configured");
          return;
        }
        context.test_command.back_right = makeTimedMotorCommand(duty, now_ms);
      } else {
        printRejected("wheel must be fl, fr, bl, or br");
        return;
      }
      context.mode = OperationMode::MotorDirectionTest;
      context.mode_expires_at_ms = now_ms + kHardwareTestTimeoutMs;
      Serial.print("ok: motor test ");
      Serial.print(wheel);
      Serial.println(" should move forward briefly");
      return;
    }

    if (strcmp(test_name, "drive") == 0) {
      char* duty_text = strtok(nullptr, " \t\r\n");
      float duty = 0.0F;
      if (!parseFloat(duty_text, duty) || duty < 0.0F ||
          !dutyWithinConfiguredLimits(context, duty)) {
        printRejected("drive test needs positive limited duty");
        return;
      }
      if (!front_left.configured() || !front_right.configured() ||
          !rear_link.configured()) {
        printRejected("front motors and rear UART must be configured");
        return;
      }
      stopAll(context, front_left, front_right, rear_link, now_ms);
      context.test_command.front_left = makeTimedMotorCommand(duty, now_ms);
      context.test_command.front_right = makeTimedMotorCommand(duty, now_ms);
      context.test_command.back_left = makeTimedMotorCommand(duty, now_ms);
      context.test_command.back_right = makeTimedMotorCommand(duty, now_ms);
      context.mode = OperationMode::DistributedDriveTest;
      context.mode_expires_at_ms = now_ms + kHardwareTestTimeoutMs;
      printOk("distributed drive test");
      return;
    }
  }

  if (applyTuningCommand(context, command, strtok(nullptr, " \t\r\n"))) {
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

void applyWheelCommand(robot::IMotorOutput& front_left,
                       robot::IMotorOutput& front_right,
                       RearCommandLink& rear_link,
                       const robot::FourWheelCommand& wheels,
                       const robot::LineFollowerConfig& config,
                       const robot::Milliseconds now_ms) {
  front_left.apply(wheels.front_left);
  front_right.apply(wheels.front_right);
  sendRearWheelCommand(rear_link, wheels, config, now_ms);
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
      now_ms >= rear_link.lastSentAtMs() ? now_ms - rear_link.lastSentAtMs()
                                         : 0U;

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
  Serial.println(rear_link.healthy() ? 1 : 0);
}

void runSensorOnlyTelemetry(RuntimeContext& context,
                            DigitalFrontLineSensorReader& sensors,
                            const robot::Milliseconds now_ms) {
  if (now_ms - context.last_telemetry_at_ms < kTelemetryPeriodMs ||
      Serial.availableForWrite() < 80) {
    return;
  }
  context.last_telemetry_at_ms = now_ms;
  const robot::FrontLineSensorSnapshot snapshot = sensors.readSnapshot(now_ms);
  Serial.print("sensor,");
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(snapshot.left == robot::LineSample::OnTape ? 1 : 0);
  Serial.print(',');
  Serial.println(snapshot.right == robot::LineSample::OnTape ? 1 : 0);
}

void motionControlTask(void* parameters) {
  (void)parameters;

  RuntimeContext context{};
  context.config.maximumDuty = hardwareDutyCap();
  context.config.maximumCorrection =
      clampFloat(context.config.maximumCorrection, 0.0F,
                 context.config.maximumDuty);

  DigitalFrontLineSensorReader line_sensor_reader{
      robot::esp2::kHardwareConfig.pins};
  DualPwmMotorOutput front_left_motor{
      robot::esp2::kHardwareConfig.front_left_motor};
  DualPwmMotorOutput front_right_motor{
      robot::esp2::kHardwareConfig.front_right_motor};
  RearCommandLink rear_link{robot::esp2::kHardwareConfig.uart_to_esp1};
  robot::esp2::StepperController stepper{};
  robot::esp2::ServoBankController servos{};

  line_sensor_reader.initialize();
  front_left_motor.initializeDisabled();
  front_right_motor.initializeDisabled();
  rear_link.initialize();
  stepper.initializeDisabled();
  servos.initializeDisabled();

  TickType_t last_wake_tick = xTaskGetTickCount();

  for (;;) {
    const robot::Milliseconds now_ms =
        static_cast<robot::Milliseconds>(millis());
    pollSerialCommands(context, line_sensor_reader, front_left_motor,
                       front_right_motor, rear_link, now_ms);

    if ((context.mode == OperationMode::MotorDirectionTest ||
         context.mode == OperationMode::DistributedDriveTest) &&
        now_ms >= context.mode_expires_at_ms) {
      stopAll(context, front_left_motor, front_right_motor, rear_link, now_ms);
      printOk("test auto-stopped");
    }

    if (context.mode == OperationMode::SensorOnly) {
      front_left_motor.disable();
      front_right_motor.disable();
      sendStoppedRearCommand(rear_link, context.config, now_ms);
      runSensorOnlyTelemetry(context, line_sensor_reader, now_ms);
    } else if (context.mode == OperationMode::LineFollowing) {
      const robot::FrontLineSensorSnapshot snapshot =
          line_sensor_reader.readSnapshot(now_ms);
      if (!snapshot.valid || !rear_link.configured()) {
        stopAll(context, front_left_motor, front_right_motor, rear_link,
                now_ms);
        printRejected("line follower stopped: sensor or rear link invalid");
      } else {
        context.last_update = robot::updateLineFollower(
            context.follower_state, snapshot.left == robot::LineSample::OnTape,
            snapshot.right == robot::LineSample::OnTape, context.config,
            now_ms);
        applyWheelCommand(front_left_motor, front_right_motor, rear_link,
                          context.last_update.wheel_command, context.config,
                          now_ms);
        if (!rear_link.healthy()) {
          stopAll(context, front_left_motor, front_right_motor, rear_link,
                  now_ms);
          printRejected("line follower stopped: rear link unhealthy");
        }
        printTelemetry(context, rear_link, now_ms);
      }
    } else if (context.mode == OperationMode::MotorDirectionTest ||
               context.mode == OperationMode::DistributedDriveTest) {
      context.last_update.wheel_command = context.test_command;
      applyWheelCommand(front_left_motor, front_right_motor, rear_link,
                        context.test_command, context.config, now_ms);
    } else {
      front_left_motor.disable();
      front_right_motor.disable();
      sendStoppedRearCommand(rear_link, context.config, now_ms);
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
  xTaskCreatePinnedToCore(motionControlTask, "esp2_line_follow",
                          kTaskStackBytes, nullptr, kTaskPriority, nullptr,
                          kTaskCore);
}

void loop() {
  vTaskSuspend(nullptr);
}
