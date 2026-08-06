#pragma once

#include <cstdint>

namespace robot::esp2 {

constexpr int kUnassignedGpio = -1;
constexpr int kUnassignedPwmChannel = -1;
constexpr int kUnassignedMcpwmResource = -1;
constexpr std::uint32_t kUnassignedFrequencyHz = 0U;
constexpr std::uint8_t kUnassignedPwmResolutionBits = 0U;
constexpr std::uint32_t kServoMcpwmTimerResolutionHz = 1000000U;
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

enum class ServoPwmBackend : std::uint8_t {
  Unconfigured = 0,
  Ledc = 1,
  Mcpwm = 2,
};

struct ServoOutputConfig {
  int gpio{kUnassignedGpio};
  ServoPwmBackend backend{ServoPwmBackend::Unconfigured};
  int ledc_channel{kUnassignedPwmChannel};
  int mcpwm_unit{kUnassignedMcpwmResource};
  int mcpwm_timer{kUnassignedMcpwmResource};
  int mcpwm_generator{kUnassignedMcpwmResource};
  std::uint32_t pwm_frequency_hz{kUnassignedFrequencyHz};
  std::uint8_t ledc_resolution_bits{kUnassignedPwmResolutionBits};
  std::uint32_t mcpwm_timer_resolution_hz{0U};
  std::uint16_t minimum_pulse_us{0U};  // TODO: safe servo pulse range
  std::uint16_t maximum_pulse_us{0U};  // TODO: safe servo pulse range
};

constexpr ServoOutputConfig ledcServoOutputConfig(
    const int gpio, const int channel, const std::uint32_t frequency_hz,
    const std::uint8_t resolution_bits,
    const std::uint16_t minimum_pulse_us,
    const std::uint16_t maximum_pulse_us) {
  return {gpio,
          ServoPwmBackend::Ledc,
          channel,
          kUnassignedMcpwmResource,
          kUnassignedMcpwmResource,
          kUnassignedMcpwmResource,
          frequency_hz,
          resolution_bits,
          0U,
          minimum_pulse_us,
          maximum_pulse_us};
}

constexpr ServoOutputConfig mcpwmServoOutputConfig(
    const int gpio, const int unit, const int timer,
    const int generator, const std::uint32_t frequency_hz,
    const std::uint32_t timer_resolution_hz,
    const std::uint16_t minimum_pulse_us,
    const std::uint16_t maximum_pulse_us) {
  return {gpio,
          ServoPwmBackend::Mcpwm,
          kUnassignedPwmChannel,
          unit,
          timer,
          generator,
          frequency_hz,
          kUnassignedPwmResolutionBits,
          timer_resolution_hz,
          minimum_pulse_us,
          maximum_pulse_us};
}

struct Esp2Pins {
  // Current IMU wiring assignment; verify it against the physical PCB.
  // GPIO8 and GPIO9 are already owned by other peripherals.
  int imu_sda{18};
  int imu_scl{17};
  int line_sensor_front_left{8};     // Digital comparator, HIGH = black tape
  int line_sensor_front_right{7};    // Digital comparator, HIGH = black tape                  // GPIO, active level TODO
  int final_competition_start_switch{10};  // LOW = stop, LOW-to-HIGH = start
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
  int servo_habitat_pusher{5};  // Habitat Pusher LEDC servo output
  int servo_winch{6};            // Winch MCPWM servo output
  int limit_switch_habitat_piece{48};  // NO switch: LOW released, HIGH pressed
  int limit_switch_stepper_bottom{9};  // NO switch: LOW released, HIGH pressed
  int limit_switch_stepper_middle{kUnassignedGpio};  // TODO: GPIO, active level
  int limit_switch_stepper_top{11};     // NO switch: LOW released, HIGH pressed
  int limit_switch_funnel_left{47};     // PegFinder funnel: LOW released, HIGH pressed
  int limit_switch_funnel_right{kUnassignedGpio};    // TODO: GPIO, active level
  int uart_tx_to_esp1{21};                         // UART TX to ESP1 GPIO40
  int uart_rx_from_esp1{40};                       // UART RX from ESP1 GPIO21
};

struct Esp2HardwareConfig {
  Esp2Pins pins{};
  DualPwmMotorOutputConfig front_left_motor{};   // TODO: fill from schematic
  DualPwmMotorOutputConfig front_right_motor{};  // TODO: fill from schematic
  ServoOutputConfig servo_claw_1{};
  ServoOutputConfig servo_claw_2{};
  ServoOutputConfig servo_claw_3{};
  // Close-to-open must rotate the physical arm clockwise; the pusher's two
  // target angles remain runtime calibration values.
  ServoOutputConfig servo_habitat_pusher{};
  ServoOutputConfig servo_winch{};
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
    ledcServoOutputConfig(kPins.servo_claw_1, 4, 50, 12, 1000, 2000),
    ledcServoOutputConfig(kPins.servo_claw_2, 5, 50, 12, 1000, 2000),
    ledcServoOutputConfig(kPins.servo_claw_3, 6, 50, 12, 1000, 2000),
    ledcServoOutputConfig(
        kPins.servo_habitat_pusher, 7, 50, 12, 1000, 2000),
    mcpwmServoOutputConfig(
        kPins.servo_winch, 0, 0, 0, 50,
        kServoMcpwmTimerResolutionHz, 1000, 2000),
    {kPins.uart_tx_to_esp1, kPins.uart_rx_from_esp1,
     kDriveTestUartBaudRate},
    1.0F};

static_assert(kPins.servo_habitat_pusher != kPins.servo_winch,
              "Habitat Pusher and winch must use distinct GPIOs");
static_assert(kPins.final_competition_start_switch == 10,
              "Final Competition start switch must use ESP2 GPIO10");
static_assert(
    kPins.final_competition_start_switch != kPins.imu_sda &&
        kPins.final_competition_start_switch != kPins.imu_scl &&
        kPins.final_competition_start_switch !=
            kPins.line_sensor_front_left &&
        kPins.final_competition_start_switch !=
            kPins.line_sensor_front_right &&
        kPins.final_competition_start_switch != kPins.pwm_front_left_0 &&
        kPins.final_competition_start_switch != kPins.pwm_front_left_1 &&
        kPins.final_competition_start_switch != kPins.pwm_front_right_0 &&
        kPins.final_competition_start_switch != kPins.pwm_front_right_1 &&
        kPins.final_competition_start_switch != kPins.stepper_step &&
        kPins.final_competition_start_switch != kPins.stepper_dir &&
        kPins.final_competition_start_switch != kPins.stepper_sleep &&
        kPins.final_competition_start_switch != kPins.servo_claw_1 &&
        kPins.final_competition_start_switch != kPins.servo_claw_2 &&
        kPins.final_competition_start_switch != kPins.servo_claw_3 &&
        kPins.final_competition_start_switch !=
            kPins.servo_habitat_pusher &&
        kPins.final_competition_start_switch != kPins.servo_winch &&
        kPins.final_competition_start_switch !=
            kPins.limit_switch_habitat_piece &&
        kPins.final_competition_start_switch !=
            kPins.limit_switch_stepper_bottom &&
        kPins.final_competition_start_switch !=
            kPins.limit_switch_stepper_top &&
        kPins.final_competition_start_switch !=
            kPins.limit_switch_funnel_left &&
        kPins.final_competition_start_switch != kPins.uart_tx_to_esp1 &&
        kPins.final_competition_start_switch != kPins.uart_rx_from_esp1,
    "Final Competition start switch GPIO must have one software owner");
static_assert(kPins.limit_switch_habitat_piece != kPins.servo_habitat_pusher &&
                  kPins.limit_switch_habitat_piece != kPins.servo_winch,
              "Habitat-piece limit switch must not share a servo GPIO");
static_assert(
    kHardwareConfig.servo_habitat_pusher.backend ==
        ServoPwmBackend::Ledc &&
        kHardwareConfig.servo_habitat_pusher.ledc_channel == 7,
    "Habitat Pusher must exclusively own LEDC channel 7");
static_assert(
    kHardwareConfig.servo_winch.backend == ServoPwmBackend::Mcpwm &&
        kHardwareConfig.servo_winch.mcpwm_unit == 0 &&
        kHardwareConfig.servo_winch.mcpwm_timer == 0 &&
        kHardwareConfig.servo_winch.mcpwm_generator == 0,
    "Winch must exclusively own MCPWM unit 0 timer 0 generator A");

}  // namespace robot::esp2
