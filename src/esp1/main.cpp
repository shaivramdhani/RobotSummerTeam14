#include <Arduino.h>

#include <cstdlib>

#include "common/Esp1Status.h"
#include "common/FaultHealth.h"
#include "common/MotorOutput.h"
#include "common/RearDriveCommand.h"
#include "common/UartProtocol.h"
#include "esp1/MissionStateMachine.h"
#include "esp1/PinConfig.h"

namespace {

constexpr robot::Milliseconds kRearDriveTaskPeriodMs = 10U;
constexpr robot::Milliseconds kRearStatusPeriodMs = 250U;
constexpr std::uint32_t kTaskStackBytes = 6144U;
constexpr UBaseType_t kTaskPriority = 1U;
constexpr BaseType_t kTaskCore = 1;

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

bool uartConfigComplete(const robot::esp1::UartConfig& config) {
  return gpioAssigned(config.tx_gpio) && gpioAssigned(config.rx_gpio) &&
         config.baud_rate > 0U;
}

std::uint32_t pwmMaxDuty(const std::uint8_t resolution_bits) {
  return (static_cast<std::uint32_t>(1U) << resolution_bits) - 1U;
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

    bool received = false;
    while (Serial1.available() > 0) {
      const robot::UartFrameParserStatus status =
          parser_.push(static_cast<std::uint8_t>(Serial1.read()), packet);
      if (status == robot::UartFrameParserStatus::PacketReady) {
        received = true;
      } else if (status == robot::UartFrameParserStatus::InvalidFrame) {
        invalid_frame_seen_ = true;
      }
    }
    return received;
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
                       const robot::RearDriveStatus& rear_status,
                       const robot::Milliseconds now_ms) {
  static robot::Milliseconds last_publish_ms = 0U;
  static std::uint16_t sequence = 0U;
  if (now_ms - last_publish_ms < kRearStatusPeriodMs) {
    return;
  }
  last_publish_ms = now_ms;

  robot::Esp1StatusReport report{};
  report.uptime_ms = now_ms;
  report.mode = robot::RobotTestMode::Disabled;
  report.fault_active = !rear_status.link_healthy &&
                        rear_status.has_valid_command &&
                        rear_status.command_age_ms >
                            robot::kDefaultCommunicationTimeoutMs;
  report.fault_code = report.fault_active
                          ? robot::FaultCode::CommunicationStale
                          : robot::FaultCode::None;
  report.back_left_applied_command_milli =
      back_left.lastAppliedCommand().duty_command_milli;
  report.back_right_applied_command_milli =
      back_right.lastAppliedCommand().duty_command_milli;
  link.send(robot::makeEsp1StatusPacket(report, sequence++));
}

void printRearStatus(const robot::RearDriveStatus& status,
                     const DualPwmMotorOutput& back_left,
                     const DualPwmMotorOutput& back_right,
                     const RearCommandLink& link,
                     const robot::Milliseconds now_ms) {
  static robot::Milliseconds last_print_ms = 0U;
  if (now_ms - last_print_ms < kRearStatusPeriodMs ||
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

void rearDriveTask(void* parameters) {
  (void)parameters;

  DualPwmMotorOutput back_left_motor{
      robot::esp1::kHardwareConfig.back_left_motor};
  DualPwmMotorOutput back_right_motor{
      robot::esp1::kHardwareConfig.back_right_motor};
  RearCommandLink link{robot::esp1::kHardwareConfig.uart_to_esp2};
  robot::RearDriveCommandReceiver receiver{};

  back_left_motor.initializeDisabled();
  back_right_motor.initializeDisabled();
  link.initialize();

  TickType_t last_wake_tick = xTaskGetTickCount();

  for (;;) {
    const robot::Milliseconds now_ms =
        static_cast<robot::Milliseconds>(millis());

    robot::UartPacket packet{};
    if (link.receive(packet)) {
      receiver.acceptPacket(packet, now_ms);
    }
    if (link.consumeInvalidFrameSeen()) {
      receiver.acceptPacket(packet, now_ms);
    }

    back_left_motor.apply(receiver.backLeftCommand(now_ms));
    back_right_motor.apply(receiver.backRightCommand(now_ms));

    const robot::RearDriveStatus rear_status = receiver.status(now_ms);
    publishEsp1Status(link, back_left_motor, back_right_motor, rear_status,
                      now_ms);
    printRearStatus(rear_status, back_left_motor, back_right_motor, link,
                    now_ms);

    vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(kRearDriveTaskPeriodMs));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  xTaskCreatePinnedToCore(rearDriveTask, "esp1_rear_drive", kTaskStackBytes,
                          nullptr, kTaskPriority, nullptr, kTaskCore);
}

void loop() {
  vTaskSuspend(nullptr);
}
