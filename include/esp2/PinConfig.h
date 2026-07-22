#pragma once

#include <cstdint>

namespace robot::esp2 {

constexpr int kUnassignedGpio = -1;
constexpr int kUnassignedPwmChannel = -1;
constexpr std::uint32_t kUnassignedFrequencyHz = 0U;
constexpr std::uint8_t kUnassignedPwmResolutionBits = 0U;
constexpr std::uint32_t kUnassignedBaudRate = 0U;
constexpr std::uint32_t kDriveTestPwmFrequencyHz = 100U;
constexpr std::uint8_t kDriveTestPwmResolutionBits = 10U;
constexpr std::uint32_t kDriveTestUartBaudRate = 115200U;

enum class DualPwmHBridgeMode : std::uint8_t {
  Unconfigured = 0,
  Pwm0ForwardPwm1Reverse = 1,
  Pwm1ForwardPwm0Reverse = 2,
};

struct DualPwmMotorOutputConfig {
  int pwm0_gpio{kUnassignedGpio};
  int pwm1_gpio{kUnassignedGpio};
  int pwm0_channel{kUnassignedPwmChannel};
  int pwm1_channel{kUnassignedPwmChannel};
  std::uint32_t pwm_frequency_hz{kUnassignedFrequencyHz};
  std::uint8_t pwm_resolution_bits{kUnassignedPwmResolutionBits};
  int forward_sign{0};  // TODO: set to +1 or -1 after wheel direction test.
  DualPwmHBridgeMode h_bridge_mode{DualPwmHBridgeMode::Unconfigured};
};

struct UartConfig {
  int tx_gpio{kUnassignedGpio};
  int rx_gpio{kUnassignedGpio};
  std::uint32_t baud_rate{kUnassignedBaudRate};
};

struct ServoOutputConfig {
  int gpio{kUnassignedGpio};
  int pwm_channel{kUnassignedPwmChannel};
  std::uint32_t pwm_frequency_hz{kUnassignedFrequencyHz};
  std::uint8_t pwm_resolution_bits{kUnassignedPwmResolutionBits};
  std::uint16_t minimum_pulse_us{0U};  // TODO: safe servo pulse range
  std::uint16_t maximum_pulse_us{0U};  // TODO: safe servo pulse range
};

struct Esp2Pins {
  int line_sensor_front_left{8};     // Digital comparator, HIGH = black tape
  int line_sensor_front_right{7};    // Digital comparator, HIGH = black tape                  // GPIO, active level TODO
  int pwm_front_left_0{15};           // TODO: GPIO, PWM resource
  int pwm_front_left_1{16};           // TODO: GPIO, PWM resource
  int pwm_front_right_0{41};          // TODO: GPIO, PWM resource
  int pwm_front_right_1{42};          // TODO: GPIO, PWM resource
  int stepper_step{4};               // DRV8425 STEP, rising edge = one microstep
  int stepper_dir{3};                // DRV8425 DIR, LOW = physical Up
  int stepper_sleep{2};              // DRV8425 nSLEEP, active LOW
  int servo_claw_1{14};               // TODO: GPIO, pulse range
  int servo_claw_2{13};               // TODO: GPIO, pulse range
  int servo_claw_3{12};               // TODO: GPIO, pulse range
  int servo_pusher{kUnassignedGpio};               // TODO: GPIO, pulse range
  int servo_winch{6};                 // Winch servo PWM output
  int limit_switch_stepper_bottom{10};  // NO switch: LOW released, HIGH pressed
  int limit_switch_stepper_middle{kUnassignedGpio};  // TODO: GPIO, active level
  int limit_switch_stepper_top{11};     // NO switch: LOW released, HIGH pressed
  int limit_switch_funnel_left{kUnassignedGpio};     // TODO: GPIO, active level
  int limit_switch_funnel_right{kUnassignedGpio};    // TODO: GPIO, active level
  int uart_tx_to_esp1{21};                         // UART TX to ESP1 GPIO40
  int uart_rx_from_esp1{40};                       // UART RX from ESP1 GPIO21
};

struct Esp2HardwareConfig {
  Esp2Pins pins{};
  DualPwmMotorOutputConfig front_left_motor{};   // TODO: fill from schematic
  DualPwmMotorOutputConfig front_right_motor{};  // TODO: fill from schematic
  ServoOutputConfig servo_claw_1{14};              // TODO: PWM resource/range
  ServoOutputConfig servo_claw_2{13};              // TODO: PWM resource/range
  ServoOutputConfig servo_claw_3{12};              // TODO: PWM resource/range
  ServoOutputConfig servo_winch{6};
  UartConfig uart_to_esp1{};                     // TODO: fill TX/RX/baud
  float maximum_safe_test_duty{0.9F};            // TODO: verified safe duty
};

inline constexpr Esp2Pins kPins{};
inline constexpr Esp2HardwareConfig kHardwareConfig{
    kPins,
    {kPins.pwm_front_left_0, kPins.pwm_front_left_1, 0, 1,
     kDriveTestPwmFrequencyHz, kDriveTestPwmResolutionBits, 1,
     DualPwmHBridgeMode::Pwm0ForwardPwm1Reverse},
    {kPins.pwm_front_right_0, kPins.pwm_front_right_1, 2, 3,
     kDriveTestPwmFrequencyHz, kDriveTestPwmResolutionBits, 1,
     DualPwmHBridgeMode::Pwm0ForwardPwm1Reverse},
    {kPins.servo_claw_1, 4, 50, 12, 1000, 2000},
    {kPins.servo_claw_2, 5, 50, 12, 1000, 2000},
    {kPins.servo_claw_3, 6, 50, 12, 1000, 2000},
    {kPins.servo_winch, 7, 50, 12, 1000, 2000},
    {kPins.uart_tx_to_esp1, kPins.uart_rx_from_esp1,
     kDriveTestUartBaudRate},
    1.0F};

}  // namespace robot::esp2
