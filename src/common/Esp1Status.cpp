#include "common/Esp1Status.h"

namespace robot {

namespace {

void putI16(std::uint8_t* output, const std::int16_t value) {
  const std::uint16_t raw = static_cast<std::uint16_t>(value);
  output[0] = static_cast<std::uint8_t>(raw & 0xFFU);
  output[1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
}

void putU16(std::uint8_t* output, const std::uint16_t value) {
  output[0] = static_cast<std::uint8_t>(value & 0xFFU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

std::int16_t getI16(const std::uint8_t* input) {
  const std::uint16_t raw =
      static_cast<std::uint16_t>(input[0]) |
      (static_cast<std::uint16_t>(input[1]) << 8U);
  return static_cast<std::int16_t>(raw);
}

std::uint16_t getU16(const std::uint8_t* input) {
  return static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(input[0]) |
      (static_cast<std::uint16_t>(input[1]) << 8U));
}

void putU32(std::uint8_t* output, const std::uint32_t value) {
  output[0] = static_cast<std::uint8_t>(value & 0xFFU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  output[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  output[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

std::uint32_t getU32(const std::uint8_t* input) {
  return static_cast<std::uint32_t>(input[0]) |
         (static_cast<std::uint32_t>(input[1]) << 8U) |
         (static_cast<std::uint32_t>(input[2]) << 16U) |
         (static_cast<std::uint32_t>(input[3]) << 24U);
}

std::uint8_t modeToByte(const RobotTestMode mode) {
  return static_cast<std::uint8_t>(mode);
}

RobotTestMode byteToMode(const std::uint8_t value) {
  switch (static_cast<RobotTestMode>(value)) {
    case RobotTestMode::Disabled:
    case RobotTestMode::SensorMonitor:
    case RobotTestMode::SingleMotorTest:
    case RobotTestMode::ManualDriveTest:
    case RobotTestMode::DistributedDriveTest:
    case RobotTestMode::LineSensorTest:
    case RobotTestMode::LineFollowTest:
    case RobotTestMode::MechanismTest:
    case RobotTestMode::AutonomousDryRun:
    case RobotTestMode::AutonomousSolarPanel:
    case RobotTestMode::RearLineSensorTest:
    case RobotTestMode::RearLineFollowTest:
    case RobotTestMode::AutonomousTowerPieces:
    case RobotTestMode::PegFinder:
    case RobotTestMode::TimeTrial:
    case RobotTestMode::ImuTurnTest:
    case RobotTestMode::ImuStrafeTest:
    case RobotTestMode::HabitatPieces:
    case RobotTestMode::HabitatPlacement:
    case RobotTestMode::FinalCompetition:
      return static_cast<RobotTestMode>(value);
  }
  return RobotTestMode::Disabled;
}

FaultCode byteToFaultCode(const std::uint8_t value) {
  switch (static_cast<FaultCode>(value)) {
    case FaultCode::None:
    case FaultCode::CommunicationStale:
    case FaultCode::InvalidCommand:
    case FaultCode::LimitSwitchConflict:
    case FaultCode::HardwareNotConfigured:
    case FaultCode::SearchTimeout:
      return static_cast<FaultCode>(value);
  }
  return FaultCode::InvalidCommand;
}

}  // namespace

UartPacket makeEsp1StatusPacket(const Esp1StatusReport& report,
                                const std::uint16_t sequence) {
  UartPacket packet{};
  packet.header.version = kUartProtocolVersion;
  packet.header.message_type = UartMessageType::HealthReport;
  packet.header.sequence = sequence;
  packet.header.payload_size = kEsp1StatusPayloadSize;
  putU32(&packet.payload[0], report.uptime_ms);
  packet.payload[4] = modeToByte(report.mode);
  packet.payload[5] = static_cast<std::uint8_t>(report.fault_code);
  putI16(&packet.payload[6],
         clampCommandMilli(report.back_left_applied_command_milli));
  putI16(&packet.payload[8],
         clampCommandMilli(report.back_right_applied_command_milli));
  std::uint8_t flags = 0U;
  flags |= report.fault_active ? kEsp1StatusFaultActiveFlag : 0U;
  flags |= report.back_left_inverted ? kEsp1StatusBackLeftInvertedFlag : 0U;
  flags |= report.back_right_inverted ? kEsp1StatusBackRightInvertedFlag : 0U;
  flags |= report.ir_beacon_detected ? kEsp1StatusIrBeaconDetectedFlag : 0U;
  flags |= report.ir_switch_raw_high ? kEsp1StatusIrSwitchRawHighFlag : 0U;
  flags |= report.ir_switch_debounced_high
               ? kEsp1StatusIrSwitchDebouncedHighFlag
               : 0U;
  flags |= report.funnel_configured ? kEsp1StatusFunnelConfiguredFlag : 0U;
  flags |= report.ir_acquisition_enabled
               ? kEsp1StatusIrAcquisitionEnabledFlag
               : 0U;
  packet.payload[10] = flags;
  putU16(&packet.payload[11], report.ir_adc_average);
  putU16(&packet.payload[13], report.ir_adc_min);
  putU16(&packet.payload[15], report.ir_adc_max);
  putU16(&packet.payload[17], report.ir_amplitude_pp);
  putU16(&packet.payload[19], report.ir_selected_frequency_hz);
  putU16(&packet.payload[21], report.ir_adc_latest_sample);
  putU16(&packet.payload[23], report.ir_1khz_amplitude);
  putU16(&packet.payload[25], report.ir_10khz_amplitude);
  putU16(&packet.payload[27], report.ir_selected_amplitude);
  putU16(&packet.payload[29], report.ir_active_threshold);
  putU32(&packet.payload[31], report.ir_adc_sample_rate_hz);
  packet.payload[35] = report.ir_consecutive_detection_count;
  putI16(&packet.payload[36],
         clampCommandMilli(report.funnel_applied_command_milli));
  std::uint8_t sensor_flags = 0U;
  sensor_flags |= report.solar_panel_limit_switches_configured
                      ? kEsp1StatusSolarLimitConfiguredFlag
                      : 0U;
  sensor_flags |= report.solar_limit_back_right_high
                      ? kEsp1StatusSolarLimitBackRightHighFlag
                      : 0U;
  sensor_flags |= report.solar_limit_front_right_high
                      ? kEsp1StatusSolarLimitFrontRightHighFlag
                      : 0U;
  sensor_flags |= report.side_line_sensor_configured
                      ? kEsp1StatusSideLineConfiguredFlag
                      : 0U;
  sensor_flags |= report.side_line_sensor_high
                      ? kEsp1StatusSideLineHighFlag
                      : 0U;
  packet.payload[38] = sensor_flags;
  std::uint8_t solar_hook_flags = 0U;
  solar_hook_flags |= report.solar_hook_configured
                          ? kEsp1StatusSolarHookConfiguredFlag
                          : 0U;
  solar_hook_flags |= report.solar_hook_output_enabled
                          ? kEsp1StatusSolarHookOutputEnabledFlag
                          : 0U;
  packet.payload[39] = solar_hook_flags;
  packet.payload[40] =
      report.solar_hook_commanded_angle_deg >= 0 &&
              report.solar_hook_commanded_angle_deg <= 180
          ? static_cast<std::uint8_t>(
                report.solar_hook_commanded_angle_deg)
          : kEsp1StatusSolarHookUnsetAngle;
  packet.header.integrity_crc16 = calculatePacketIntegrity(packet);
  return packet;
}

UartPacket makeEsp1OperationalStatusPacket(
    const Esp1StatusReport& report, const std::uint16_t sequence) {
  UartPacket packet{};
  packet.header.version = kUartProtocolVersion;
  packet.header.message_type = UartMessageType::HealthReport;
  packet.header.sequence = sequence;
  packet.header.payload_size = kEsp1OperationalStatusPayloadSize;
  putU32(&packet.payload[0], report.uptime_ms);
  packet.payload[4] = modeToByte(report.mode);
  packet.payload[5] = static_cast<std::uint8_t>(report.fault_code);
  putI16(&packet.payload[6],
         clampCommandMilli(report.back_left_applied_command_milli));
  putI16(&packet.payload[8],
         clampCommandMilli(report.back_right_applied_command_milli));
  putI16(&packet.payload[10],
         clampCommandMilli(report.funnel_applied_command_milli));

  std::uint8_t operational_flags = 0U;
  operational_flags |= report.fault_active
                           ? kEsp1OperationalFaultActiveFlag
                           : 0U;
  operational_flags |= report.funnel_configured
                           ? kEsp1OperationalFunnelConfiguredFlag
                           : 0U;
  operational_flags |= report.solar_panel_limit_switches_configured
                           ? kEsp1OperationalSolarLimitConfiguredFlag
                           : 0U;
  operational_flags |= report.solar_limit_back_right_high
                           ? kEsp1OperationalSolarLimitBackRightHighFlag
                           : 0U;
  operational_flags |= report.solar_limit_front_right_high
                           ? kEsp1OperationalSolarLimitFrontRightHighFlag
                           : 0U;
  operational_flags |= report.solar_hook_configured
                           ? kEsp1OperationalSolarHookConfiguredFlag
                           : 0U;
  operational_flags |= report.solar_hook_output_enabled
                           ? kEsp1OperationalSolarHookOutputEnabledFlag
                           : 0U;
  operational_flags |= report.ir_acquisition_enabled
                           ? kEsp1OperationalIrAcquisitionEnabledFlag
                           : 0U;
  packet.payload[12] = operational_flags;
  packet.payload[13] =
      report.solar_hook_commanded_angle_deg >= 0 &&
              report.solar_hook_commanded_angle_deg <= 180
          ? static_cast<std::uint8_t>(
                report.solar_hook_commanded_angle_deg)
          : kEsp1StatusSolarHookUnsetAngle;
  putU16(&packet.payload[14], report.ir_selected_frequency_hz);
  putU16(&packet.payload[16], report.ir_selected_amplitude);

  std::uint8_t auxiliary_flags = 0U;
  auxiliary_flags |= report.ir_beacon_detected
                         ? kEsp1OperationalIrBeaconDetectedFlag
                         : 0U;
  auxiliary_flags |= report.back_left_inverted
                         ? kEsp1OperationalBackLeftInvertedFlag
                         : 0U;
  auxiliary_flags |= report.back_right_inverted
                         ? kEsp1OperationalBackRightInvertedFlag
                         : 0U;
  auxiliary_flags |= report.side_line_sensor_configured
                         ? kEsp1OperationalSideLineConfiguredFlag
                         : 0U;
  auxiliary_flags |= report.side_line_sensor_high
                         ? kEsp1OperationalSideLineHighFlag
                         : 0U;
  packet.payload[18] = auxiliary_flags;
  packet.header.integrity_crc16 = calculatePacketIntegrity(packet);
  return packet;
}

UartPacket makeEsp1DiagnosticsPacket(const Esp1StatusReport& report,
                                     const std::uint16_t sequence) {
  UartPacket packet{};
  packet.header.version = kUartProtocolVersion;
  packet.header.message_type = UartMessageType::DiagnosticReport;
  packet.header.sequence = sequence;
  packet.header.payload_size = kEsp1DiagnosticsPayloadSize;
  std::uint8_t flags = 0U;
  flags |= report.ir_switch_raw_high
               ? kEsp1DiagnosticsIrSwitchRawHighFlag
               : 0U;
  flags |= report.ir_switch_debounced_high
               ? kEsp1DiagnosticsIrSwitchDebouncedHighFlag
               : 0U;
  packet.payload[0] = flags;
  putU16(&packet.payload[1], report.ir_adc_average);
  putU16(&packet.payload[3], report.ir_adc_min);
  putU16(&packet.payload[5], report.ir_adc_max);
  putU16(&packet.payload[7], report.ir_amplitude_pp);
  putU16(&packet.payload[9], report.ir_adc_latest_sample);
  putU16(&packet.payload[11], report.ir_1khz_amplitude);
  putU16(&packet.payload[13], report.ir_10khz_amplitude);
  putU16(&packet.payload[15], report.ir_active_threshold);
  packet.payload[17] = report.ir_consecutive_detection_count;
  putU32(&packet.payload[18], report.ir_adc_sample_rate_hz);
  packet.header.integrity_crc16 = calculatePacketIntegrity(packet);
  return packet;
}

bool decodeEsp1StatusPacket(const UartPacket& packet,
                            Esp1StatusReport& report) {
  if (!packetLooksValid(packet) ||
      packet.header.message_type != UartMessageType::HealthReport ||
      (packet.header.payload_size != kEsp1StatusPayloadSize &&
       packet.header.payload_size !=
           kEsp1OperationalStatusPayloadSize)) {
    report = {};
    return false;
  }

  if (packet.header.payload_size ==
      kEsp1OperationalStatusPayloadSize) {
    report.uptime_ms = getU32(&packet.payload[0]);
    report.mode = byteToMode(packet.payload[4]);
    report.fault_code = byteToFaultCode(packet.payload[5]);
    report.back_left_applied_command_milli = getI16(&packet.payload[6]);
    report.back_right_applied_command_milli = getI16(&packet.payload[8]);
    report.funnel_applied_command_milli = getI16(&packet.payload[10]);
    const std::uint8_t operational_flags = packet.payload[12];
    report.fault_active =
        (operational_flags & kEsp1OperationalFaultActiveFlag) != 0U;
    report.funnel_configured =
        (operational_flags & kEsp1OperationalFunnelConfiguredFlag) != 0U;
    report.solar_panel_limit_switches_configured =
        (operational_flags & kEsp1OperationalSolarLimitConfiguredFlag) !=
        0U;
    report.solar_limit_back_right_high =
        (operational_flags &
         kEsp1OperationalSolarLimitBackRightHighFlag) != 0U;
    report.solar_limit_front_right_high =
        (operational_flags &
         kEsp1OperationalSolarLimitFrontRightHighFlag) != 0U;
    report.solar_hook_configured =
        (operational_flags & kEsp1OperationalSolarHookConfiguredFlag) != 0U;
    report.solar_hook_output_enabled =
        (operational_flags &
         kEsp1OperationalSolarHookOutputEnabledFlag) != 0U;
    report.ir_acquisition_enabled =
        (operational_flags &
         kEsp1OperationalIrAcquisitionEnabledFlag) != 0U;
    const std::uint8_t angle_deg = packet.payload[13];
    report.solar_hook_commanded_angle_deg =
        angle_deg <= 180U ? static_cast<std::int16_t>(angle_deg) : -1;
    report.ir_selected_frequency_hz = getU16(&packet.payload[14]);
    report.ir_selected_amplitude = getU16(&packet.payload[16]);
    const std::uint8_t auxiliary_flags = packet.payload[18];
    report.ir_beacon_detected =
        (auxiliary_flags & kEsp1OperationalIrBeaconDetectedFlag) != 0U;
    report.back_left_inverted =
        (auxiliary_flags & kEsp1OperationalBackLeftInvertedFlag) != 0U;
    report.back_right_inverted =
        (auxiliary_flags & kEsp1OperationalBackRightInvertedFlag) != 0U;
    report.side_line_sensor_configured =
        (auxiliary_flags & kEsp1OperationalSideLineConfiguredFlag) != 0U;
    report.side_line_sensor_high =
        (auxiliary_flags & kEsp1OperationalSideLineHighFlag) != 0U;
    return true;
  }

  report.uptime_ms = getU32(&packet.payload[0]);
  report.mode = byteToMode(packet.payload[4]);
  report.fault_code = byteToFaultCode(packet.payload[5]);
  report.back_left_applied_command_milli = getI16(&packet.payload[6]);
  report.back_right_applied_command_milli = getI16(&packet.payload[8]);
  const std::uint8_t flags = packet.payload[10];
  report.fault_active = (flags & kEsp1StatusFaultActiveFlag) != 0U;
  report.back_left_inverted =
      (flags & kEsp1StatusBackLeftInvertedFlag) != 0U;
  report.back_right_inverted =
      (flags & kEsp1StatusBackRightInvertedFlag) != 0U;
  report.ir_beacon_detected =
      (flags & kEsp1StatusIrBeaconDetectedFlag) != 0U;
  report.ir_switch_raw_high =
      (flags & kEsp1StatusIrSwitchRawHighFlag) != 0U;
  report.ir_switch_debounced_high =
      (flags & kEsp1StatusIrSwitchDebouncedHighFlag) != 0U;
  report.ir_acquisition_enabled =
      (flags & kEsp1StatusIrAcquisitionEnabledFlag) != 0U;
  report.funnel_configured =
      (flags & kEsp1StatusFunnelConfiguredFlag) != 0U;
  report.ir_adc_average = getU16(&packet.payload[11]);
  report.ir_adc_min = getU16(&packet.payload[13]);
  report.ir_adc_max = getU16(&packet.payload[15]);
  report.ir_amplitude_pp = getU16(&packet.payload[17]);
  report.ir_selected_frequency_hz = getU16(&packet.payload[19]);
  report.ir_adc_latest_sample = getU16(&packet.payload[21]);
  report.ir_1khz_amplitude = getU16(&packet.payload[23]);
  report.ir_10khz_amplitude = getU16(&packet.payload[25]);
  report.ir_selected_amplitude = getU16(&packet.payload[27]);
  report.ir_active_threshold = getU16(&packet.payload[29]);
  report.ir_adc_sample_rate_hz = getU32(&packet.payload[31]);
  report.ir_consecutive_detection_count = packet.payload[35];
  report.funnel_applied_command_milli = getI16(&packet.payload[36]);
  const std::uint8_t sensor_flags = packet.payload[38];
  report.solar_panel_limit_switches_configured =
      (sensor_flags & kEsp1StatusSolarLimitConfiguredFlag) != 0U;
  report.solar_limit_back_right_high =
      (sensor_flags & kEsp1StatusSolarLimitBackRightHighFlag) != 0U;
  report.solar_limit_front_right_high =
      (sensor_flags & kEsp1StatusSolarLimitFrontRightHighFlag) != 0U;
  report.side_line_sensor_configured =
      (sensor_flags & kEsp1StatusSideLineConfiguredFlag) != 0U;
  report.side_line_sensor_high =
      (sensor_flags & kEsp1StatusSideLineHighFlag) != 0U;
  const std::uint8_t solar_hook_flags = packet.payload[39];
  report.solar_hook_configured =
      (solar_hook_flags & kEsp1StatusSolarHookConfiguredFlag) != 0U;
  report.solar_hook_output_enabled =
      (solar_hook_flags & kEsp1StatusSolarHookOutputEnabledFlag) != 0U;
  const std::uint8_t angle_deg = packet.payload[40];
  report.solar_hook_commanded_angle_deg =
      angle_deg <= 180U ? static_cast<std::int16_t>(angle_deg) : -1;
  return true;
}

bool decodeEsp1DiagnosticsPacket(const UartPacket& packet,
                                 Esp1StatusReport& report) {
  if (!packetLooksValid(packet) ||
      packet.header.message_type != UartMessageType::DiagnosticReport ||
      packet.header.payload_size != kEsp1DiagnosticsPayloadSize) {
    return false;
  }
  const std::uint8_t flags = packet.payload[0];
  report.ir_switch_raw_high =
      (flags & kEsp1DiagnosticsIrSwitchRawHighFlag) != 0U;
  report.ir_switch_debounced_high =
      (flags & kEsp1DiagnosticsIrSwitchDebouncedHighFlag) != 0U;
  report.ir_adc_average = getU16(&packet.payload[1]);
  report.ir_adc_min = getU16(&packet.payload[3]);
  report.ir_adc_max = getU16(&packet.payload[5]);
  report.ir_amplitude_pp = getU16(&packet.payload[7]);
  report.ir_adc_latest_sample = getU16(&packet.payload[9]);
  report.ir_1khz_amplitude = getU16(&packet.payload[11]);
  report.ir_10khz_amplitude = getU16(&packet.payload[13]);
  report.ir_active_threshold = getU16(&packet.payload[15]);
  report.ir_consecutive_detection_count = packet.payload[17];
  report.ir_adc_sample_rate_hz = getU32(&packet.payload[18]);
  return true;
}

}  // namespace robot
