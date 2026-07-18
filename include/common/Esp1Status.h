#pragma once

#include <cstdint>

#include "common/FaultHealth.h"
#include "common/RobotTestMode.h"
#include "common/UartProtocol.h"
#include "common/Units.h"

namespace robot {

constexpr std::uint16_t kEsp1StatusPayloadSize = 39U;
constexpr std::uint8_t kEsp1StatusFaultActiveFlag = 0x01U;
constexpr std::uint8_t kEsp1StatusBackLeftInvertedFlag = 0x02U;
constexpr std::uint8_t kEsp1StatusBackRightInvertedFlag = 0x04U;
constexpr std::uint8_t kEsp1StatusIrBeaconDetectedFlag = 0x08U;
constexpr std::uint8_t kEsp1StatusIrSwitchRawHighFlag = 0x10U;
constexpr std::uint8_t kEsp1StatusIrSwitchDebouncedHighFlag = 0x20U;
constexpr std::uint8_t kEsp1StatusFunnelConfiguredFlag = 0x40U;
constexpr std::uint8_t kEsp1StatusSolarLimitConfiguredFlag = 0x01U;
constexpr std::uint8_t kEsp1StatusSolarLimitBackRightHighFlag = 0x02U;
constexpr std::uint8_t kEsp1StatusSolarLimitFrontRightHighFlag = 0x04U;
constexpr std::uint8_t kEsp1StatusSideLineConfiguredFlag = 0x08U;
constexpr std::uint8_t kEsp1StatusSideLineHighFlag = 0x10U;

struct Esp1StatusReport {
  Milliseconds uptime_ms{0};
  RobotTestMode mode{RobotTestMode::Disabled};
  bool fault_active{false};
  FaultCode fault_code{FaultCode::None};
  std::int16_t back_left_applied_command_milli{0};
  std::int16_t back_right_applied_command_milli{0};
  bool back_left_inverted{false};
  bool back_right_inverted{false};
  std::int16_t funnel_applied_command_milli{0};
  bool funnel_configured{false};
  bool solar_panel_limit_switches_configured{false};
  bool solar_limit_back_right_high{false};
  bool solar_limit_front_right_high{false};
  bool side_line_sensor_configured{false};
  bool side_line_sensor_high{false};
  std::uint16_t ir_adc_average{0};
  std::uint16_t ir_adc_min{0};
  std::uint16_t ir_adc_max{0};
  std::uint16_t ir_amplitude_pp{0};
  bool ir_beacon_detected{false};
  bool ir_switch_raw_high{true};
  bool ir_switch_debounced_high{true};
  std::uint16_t ir_selected_frequency_hz{1000};
  std::uint16_t ir_adc_latest_sample{0};
  std::uint16_t ir_1khz_amplitude{0};
  std::uint16_t ir_10khz_amplitude{0};
  std::uint16_t ir_selected_amplitude{0};
  std::uint16_t ir_active_threshold{0};
  std::uint8_t ir_consecutive_detection_count{0};
  std::uint32_t ir_adc_sample_rate_hz{0};
};

UartPacket makeEsp1StatusPacket(const Esp1StatusReport& report,
                                std::uint16_t sequence);
bool decodeEsp1StatusPacket(const UartPacket& packet,
                            Esp1StatusReport& report);

}  // namespace robot
