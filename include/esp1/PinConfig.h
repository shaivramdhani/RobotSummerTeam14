#pragma once

#include <cstdint>

namespace robot::esp1 {

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

struct Esp1Pins {
  int left_ir_filtered{kUnassignedGpio};           // TODO: GPIO, active level
  int right_ir_filtered{7};  // GPIO7, ADC1_CH6, analog 0-3.3 V max
  int freq{2};  // IR frequency-select switch, INPUT_PULLUP
  int ultrasonic_trigger_1{kUnassignedGpio};       // TODO: GPIO, timing
  int ultrasonic_echo_1{kUnassignedGpio};          // TODO: GPIO, voltage level
  int ultrasonic_trigger_2{kUnassignedGpio};       // TODO: GPIO, timing
  int ultrasonic_echo_2{kUnassignedGpio};          // TODO: GPIO, voltage level
  int line_sensor_side{kUnassignedGpio};           // TODO: GPIO/ADC, active level
  int line_sensor_back_left{kUnassignedGpio};      // TODO: GPIO/ADC, active level
  int line_sensor_back_right{kUnassignedGpio};     // TODO: GPIO/ADC, active level
  int pwm_back_left_0{16};            // TODO: GPIO, PWM resource
  int pwm_back_left_1{15};            // TODO: GPIO, PWM resource
  int pwm_back_right_0{42};           // TODO: GPIO, PWM resource
  int pwm_back_right_1{41};           // TODO: GPIO, PWM resource
  int pwm_funnel_0{kUnassignedGpio};               // TODO: GPIO, PWM resource
  int pwm_funnel_1{kUnassignedGpio};               // TODO: GPIO, PWM resource
  int limit_switch_back_right_side{1};   // TODO: GPIO, active level
  int limit_switch_front_right_side{9};  // TODO: GPIO, active level
  int uart_tx_to_esp2{21};                         // UART TX to ESP2 GPIO40
  int uart_rx_from_esp2{40};                       // UART RX from ESP2 GPIO21
};

struct Esp1HardwareConfig {
  Esp1Pins pins{};
  DualPwmMotorOutputConfig back_left_motor{};   // TODO: fill from schematic
  DualPwmMotorOutputConfig back_right_motor{};  // TODO: fill from schematic
  UartConfig uart_to_esp2{};                    // TODO: fill TX/RX/baud
  float maximum_safe_test_duty{0.8F};           // TODO: verified safe duty
};

inline constexpr Esp1Pins kPins{};
inline constexpr Esp1HardwareConfig kHardwareConfig{
    kPins,
    {kPins.pwm_back_left_0, kPins.pwm_back_left_1, 0, 1,
     kDriveTestPwmFrequencyHz, kDriveTestPwmResolutionBits, 1,
     DualPwmHBridgeMode::Pwm0ForwardPwm1Reverse},
    {kPins.pwm_back_right_0, kPins.pwm_back_right_1, 2, 3,
     kDriveTestPwmFrequencyHz, kDriveTestPwmResolutionBits, 1,
     DualPwmHBridgeMode::Pwm0ForwardPwm1Reverse},
    {kPins.uart_tx_to_esp2, kPins.uart_rx_from_esp2,
     kDriveTestUartBaudRate},
    1.0F};

}  // namespace robot::esp1
