#include <cmath>
#include <cstring>

#include <unity.h>

#include "common/ChassisMixer.h"
#include "common/Esp1Status.h"
#include "common/EventLog.h"
#include "common/LineFollower.h"
#include "common/LineObservation.h"
#include "common/RearDriveCommand.h"
#include "common/RobotCommandValidation.h"
#include "common/RobotTestModeManager.h"
#include "common/TelemetrySnapshot.h"

namespace {

void assertNear(const float expected, const float actual,
                const float tolerance) {
  TEST_ASSERT_TRUE(std::fabs(expected - actual) <= tolerance);
}

robot::LineFollowerConfig pidConfig() {
  robot::LineFollowerConfig config{};
  config.maximumDuty = 0.6F;
  config.maximumCorrection = 0.6F;
  config.integralLimit = 1.0F;
  config.derivativeLimit = 100.0F;
  return config;
}

void test_low_low_maps_to_zero_error() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, true, 0, 10U);

  TEST_ASSERT_TRUE(observation.left_black);
  TEST_ASSERT_TRUE(observation.right_black);
  TEST_ASSERT_EQUAL_INT8(0, observation.error);
  TEST_ASSERT_TRUE(observation.line_visible);
  TEST_ASSERT_TRUE(observation.safe_to_drive);
}

void test_low_high_maps_to_positive_one() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, false, 0, 10U);

  TEST_ASSERT_EQUAL_INT8(1, observation.error);
  TEST_ASSERT_EQUAL_INT8(1, observation.last_known_side);
  TEST_ASSERT_TRUE(observation.line_visible);
}

void test_high_low_maps_to_negative_one() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(false, true, 0, 10U);

  TEST_ASSERT_EQUAL_INT8(-1, observation.error);
  TEST_ASSERT_EQUAL_INT8(-1, observation.last_known_side);
  TEST_ASSERT_TRUE(observation.line_visible);
}

void test_high_high_after_positive_history_maps_to_positive_five() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(false, false, 1, 10U);

  TEST_ASSERT_EQUAL_INT8(5, observation.error);
  TEST_ASSERT_EQUAL_INT8(1, observation.last_known_side);
  TEST_ASSERT_FALSE(observation.line_visible);
  TEST_ASSERT_TRUE(observation.safe_to_drive);
}

void test_high_high_after_negative_history_maps_to_negative_five() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(false, false, -1, 10U);

  TEST_ASSERT_EQUAL_INT8(-5, observation.error);
  TEST_ASSERT_EQUAL_INT8(-1, observation.last_known_side);
  TEST_ASSERT_FALSE(observation.line_visible);
  TEST_ASSERT_TRUE(observation.safe_to_drive);
}

void test_high_high_without_history_is_unsafe() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(false, false, 0, 10U);

  TEST_ASSERT_EQUAL_INT8(0, observation.error);
  TEST_ASSERT_EQUAL_INT8(0, observation.last_known_side);
  TEST_ASSERT_FALSE(observation.line_visible);
  TEST_ASSERT_FALSE(observation.safe_to_drive);
}

void test_both_low_preserves_last_known_side() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, true, -1, 10U);

  TEST_ASSERT_EQUAL_INT8(0, observation.error);
  TEST_ASSERT_EQUAL_INT8(-1, observation.last_known_side);
  TEST_ASSERT_TRUE(observation.line_visible);
}

void test_zero_error_gives_zero_correction_after_reset() {
  robot::LineFollowerState state{};
  robot::startLineFollower(state, 100U);
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, true, 0, 100U);

  const robot::PidTerms terms =
      robot::calculatePidTerms(state, observation, pidConfig(), 100U);

  assertNear(0.0F, terms.correction, 0.0001F);
}

void test_proportional_term_has_correct_sign() {
  robot::LineFollowerState state{};
  robot::LineFollowerConfig config = pidConfig();
  config.kp = 0.5F;
  robot::startLineFollower(state, 100U);
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, false, 0, 120U);

  const robot::PidTerms terms =
      robot::calculatePidTerms(state, observation, config, 120U);

  assertNear(0.5F, terms.proportional_term, 0.0001F);
  TEST_ASSERT_TRUE(terms.correction > 0.0F);
}

void test_correction_clamps() {
  robot::LineFollowerState state{};
  robot::LineFollowerConfig config = pidConfig();
  config.kp = 10.0F;
  config.maximumCorrection = 0.25F;
  robot::startLineFollower(state, 100U);
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, false, 0, 120U);

  const robot::PidTerms terms =
      robot::calculatePidTerms(state, observation, config, 120U);

  assertNear(0.25F, terms.correction, 0.0001F);
}

void test_integral_clamps() {
  robot::LineFollowerState state{};
  robot::LineFollowerConfig config = pidConfig();
  config.ki = 1.0F;
  config.integralLimit = 0.2F;
  robot::startLineFollower(state, 100U);

  robot::calculatePidTerms(
      state, robot::observeDigitalLineSensors(true, false, 0, 200U), config,
      200U);
  const robot::PidTerms terms = robot::calculatePidTerms(
      state, robot::observeDigitalLineSensors(true, false, 1, 1000U), config,
      1000U);

  assertNear(0.2F, terms.integral_term, 0.0001F);
}

void test_integral_does_not_accumulate_while_line_is_lost() {
  robot::LineFollowerState state{};
  robot::LineFollowerConfig config = pidConfig();
  config.ki = 1.0F;
  robot::startLineFollower(state, 100U);

  robot::calculatePidTerms(
      state, robot::observeDigitalLineSensors(true, false, 0, 200U), config,
      200U);
  const robot::PidTerms terms = robot::calculatePidTerms(
      state, robot::observeDigitalLineSensors(false, false, 1, 300U), config,
      300U);

  assertNear(0.0F, terms.integral_term, 0.0001F);
}

void test_derivative_uses_elapsed_time() {
  robot::LineFollowerState state{};
  robot::LineFollowerConfig config = pidConfig();
  config.kd = 1.0F;
  robot::startLineFollower(state, 100U);

  robot::calculatePidTerms(
      state, robot::observeDigitalLineSensors(true, true, 0, 100U), config,
      100U);
  const robot::PidTerms terms = robot::calculatePidTerms(
      state, robot::observeDigitalLineSensors(true, false, 0, 200U), config,
      200U);

  assertNear(10.0F, terms.derivative_term, 0.0001F);
}

void test_derivative_clamps() {
  robot::LineFollowerState state{};
  robot::LineFollowerConfig config = pidConfig();
  config.kd = 1.0F;
  config.derivativeLimit = 2.0F;
  robot::startLineFollower(state, 100U);

  robot::calculatePidTerms(
      state, robot::observeDigitalLineSensors(true, true, 0, 100U), config,
      100U);
  const robot::PidTerms terms = robot::calculatePidTerms(
      state, robot::observeDigitalLineSensors(true, false, 0, 110U), config,
      110U);

  assertNear(2.0F, terms.derivative_term, 0.0001F);
}

void test_reset_clears_pid_state() {
  robot::LineFollowerState state{};
  state.enabled = true;
  state.integral = 1.0F;
  state.previous_error = 5.0F;
  state.has_previous_error = true;
  state.last_known_side = 1;

  robot::resetLineFollowerState(state);

  TEST_ASSERT_FALSE(state.enabled);
  assertNear(0.0F, state.integral, 0.0001F);
  TEST_ASSERT_FALSE(state.has_previous_error);
  TEST_ASSERT_EQUAL_INT8(0, state.last_known_side);
}

void test_zero_correction_gives_equal_left_and_right_commands() {
  robot::LineFollowerConfig config{};
  config.baseDuty = 0.2F;
  config.maximumDuty = 0.5F;
  config.maximumCorrection = 0.3F;

  const robot::FourWheelCommand command =
      robot::mixDifferentialLineFollow(0.0F, config, 100U);

  TEST_ASSERT_EQUAL_INT16(200, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(200, command.front_right.duty_command_milli);
}

void test_positive_correction_changes_sides_oppositely() {
  robot::LineFollowerConfig config{};
  config.baseDuty = 0.2F;
  config.maximumDuty = 0.5F;
  config.maximumCorrection = 0.3F;

  const robot::FourWheelCommand command =
      robot::mixDifferentialLineFollow(0.1F, config, 100U);

  TEST_ASSERT_EQUAL_INT16(100, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(300, command.front_right.duty_command_milli);
}

void test_negative_polarity_reverses_correction() {
  robot::LineFollowerConfig config{};
  config.baseDuty = 0.2F;
  config.maximumDuty = 0.5F;
  config.maximumCorrection = 0.3F;
  config.steeringPolarity = -1;

  const robot::FourWheelCommand command =
      robot::mixDifferentialLineFollow(0.1F, config, 100U);

  TEST_ASSERT_EQUAL_INT16(300, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(100, command.front_right.duty_command_milli);
}

void test_final_duties_remain_inside_limits() {
  robot::LineFollowerConfig config{};
  config.baseDuty = 0.5F;
  config.maximumDuty = 0.4F;
  config.maximumCorrection = 0.4F;

  const robot::FourWheelCommand command =
      robot::mixDifferentialLineFollow(0.4F, config, 100U);

  TEST_ASSERT_TRUE(std::abs(command.front_left.duty_command_milli) <= 400);
  TEST_ASSERT_TRUE(std::abs(command.front_right.duty_command_milli) <= 400);
}

void test_valid_rear_command_is_accepted() {
  robot::RearDriveCommandReceiver receiver{};
  const robot::RearDriveCommand command{true, 123, -456, 100U, 250U};
  const robot::UartPacket packet = robot::makeRearDriveCommandPacket(command, 7);

  TEST_ASSERT_TRUE(receiver.acceptPacket(packet, 100U));

  const robot::RearDriveStatus status = receiver.status(120U);
  TEST_ASSERT_TRUE(status.link_healthy);
  TEST_ASSERT_EQUAL_UINT16(7, status.last_sequence);
  TEST_ASSERT_EQUAL_INT16(123, receiver.backLeftCommand(120U).duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-456,
                          receiver.backRightCommand(120U).duty_command_milli);
}

void test_corrupt_rear_packet_is_rejected() {
  robot::RearDriveCommandReceiver receiver{};
  const robot::RearDriveCommand command{true, 100, 100, 100U, 250U};
  robot::UartPacket packet = robot::makeRearDriveCommandPacket(command, 7);
  packet.payload[1] ^= 0x40U;

  TEST_ASSERT_FALSE(receiver.acceptPacket(packet, 100U));
  TEST_ASSERT_FALSE(receiver.status(100U).has_valid_command);
}

void test_stale_rear_command_stops_motors() {
  robot::RearDriveCommandReceiver receiver{};
  const robot::RearDriveCommand command{true, 200, 200, 100U, 50U};
  const robot::UartPacket packet = robot::makeRearDriveCommandPacket(command, 7);

  TEST_ASSERT_TRUE(receiver.acceptPacket(packet, 100U));

  TEST_ASSERT_FALSE(receiver.backLeftCommand(151U).enabled);
  TEST_ASSERT_FALSE(receiver.backRightCommand(151U).enabled);
}

void test_explicit_stop_packet_stops_motors() {
  robot::RearDriveCommandReceiver receiver{};
  const robot::RearDriveCommand command{false, 200, 200, 100U, 250U};
  const robot::UartPacket packet = robot::makeRearDriveCommandPacket(command, 7);

  TEST_ASSERT_TRUE(receiver.acceptPacket(packet, 100U));

  TEST_ASSERT_FALSE(receiver.backLeftCommand(120U).enabled);
  TEST_ASSERT_FALSE(receiver.backRightCommand(120U).enabled);
}

void test_mode_manager_starts_disabled() {
  const robot::RobotTestModeManager manager{};

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::Disabled),
      static_cast<std::uint8_t>(manager.currentMode()));
  TEST_ASSERT_FALSE(manager.motorsMayBeCommanded());
}

void test_mode_manager_rejects_drive_while_disabled() {
  const robot::CommandValidationLimits limits{0.5F, 0.25F, 1000U};
  const robot::CommandValidationResult result = robot::validateDriveCommand(
      robot::RobotTestMode::Disabled, 0.0F, 1.0F, 0.0F, 0.2F, limits);

  TEST_ASSERT_FALSE(result.accepted);
}

void test_mode_manager_accepts_sensor_mode_without_motors() {
  robot::RobotTestModeManager manager{};
  manager.setMode(robot::RobotTestMode::SensorMonitor, 25U);

  TEST_ASSERT_TRUE(
      robot::robotTestModeIsSensorOnly(manager.currentMode()));
  TEST_ASSERT_FALSE(manager.motorsMayBeCommanded());
}

void test_emergency_stop_works_from_any_mode() {
  robot::RobotTestModeManager manager{};
  manager.setMode(robot::RobotTestMode::ManualDriveTest, 25U);
  TEST_ASSERT_TRUE(manager.motorsMayBeCommanded());

  manager.emergencyStop(50U);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::Disabled),
      static_cast<std::uint8_t>(manager.currentMode()));
  TEST_ASSERT_FALSE(manager.motorsMayBeCommanded());
}

void test_command_validation_rejects_out_of_range_duty() {
  const robot::CommandValidationResult result =
      robot::validateNormalizedDuty(0.4F, 0.25F);

  TEST_ASSERT_FALSE(result.accepted);
}

void test_command_validation_accepts_drive_test_duty_0_7() {
  const robot::CommandValidationLimits limits{1.0F, 1.0F, 1000U};

  const robot::CommandValidationResult single_motor_result =
      robot::validateSingleMotorCommand(robot::RobotTestMode::SingleMotorTest,
                                        0.7F, 700U, limits);
  const robot::CommandValidationResult drive_result =
      robot::validateDriveCommand(robot::RobotTestMode::DistributedDriveTest,
                                  0.0F, 1.0F, 0.0F, 0.7F, limits);

  TEST_ASSERT_TRUE(single_motor_result.accepted);
  TEST_ASSERT_TRUE(drive_result.accepted);
}

void test_command_validation_rejects_overlong_duration() {
  const robot::CommandValidationResult result =
      robot::validateTimedDuration(5001U, 5000U);

  TEST_ASSERT_FALSE(result.accepted);
}

void test_command_validation_rejects_malformed_motor_id() {
  robot::WheelId wheel{};

  TEST_ASSERT_FALSE(robot::parseWheelId("bogus", wheel));
}

void test_command_validation_rejects_invalid_pid_value() {
  robot::LineFollowerConfig config{};
  config.maximumDuty = 0.3F;
  config.maximumCorrection = 0.2F;
  config.kp = -0.1F;

  const robot::CommandValidationResult result =
      robot::validateLineFollowerConfig(config, 0.4F);

  TEST_ASSERT_FALSE(result.accepted);
}

void test_command_validation_rejects_mode_incompatible_motor_command() {
  const robot::CommandValidationLimits limits{0.5F, 0.25F, 1000U};
  const robot::CommandValidationResult result =
      robot::validateSingleMotorCommand(robot::RobotTestMode::SensorMonitor,
                                        0.1F, 500U, limits);

  TEST_ASSERT_FALSE(result.accepted);
}

void test_event_log_stores_newest_events_and_wraps() {
  robot::EventLog log{};
  for (std::size_t index = 0; index < log.capacity() + 3U; ++index) {
    log.add(static_cast<robot::Milliseconds>(index),
            robot::EventSeverity::Info, robot::EventSource::System,
            index == log.capacity() + 2U ? "newest" : "older");
  }

  robot::EventRecord newest{};
  TEST_ASSERT_EQUAL_UINT(log.capacity(), log.size());
  TEST_ASSERT_TRUE(log.newest(0U, newest));
  TEST_ASSERT_EQUAL_STRING("newest", newest.message);
}

void test_telemetry_json_contains_required_fields_and_booleans() {
  robot::TelemetrySnapshot snapshot{};
  snapshot.uptime_ms = 123U;
  snapshot.current_mode = robot::RobotTestMode::LineSensorTest;
  snapshot.enabled = false;
  snapshot.lsfl_black = true;
  snapshot.lsfr_black = false;
  snapshot.front_left.desired_command_milli = 100;
  snapshot.front_right.desired_command_milli = -100;

  char output[2048]{};
  TEST_ASSERT_TRUE(
      robot::writeTelemetryJson(snapshot, output, sizeof(output), false));

  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"current_mode\":\"LINE_SENSOR_TEST\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"enabled\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lsfl_black\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"front_left\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"line_error\""));
}

void test_esp1_status_packet_round_trips() {
  const robot::Esp1StatusReport report{
      1234U, robot::RobotTestMode::Disabled, true,
      robot::FaultCode::CommunicationStale, 111, -222, true, false};

  const robot::UartPacket packet = robot::makeEsp1StatusPacket(report, 42U);
  robot::Esp1StatusReport decoded{};

  TEST_ASSERT_TRUE(robot::decodeEsp1StatusPacket(packet, decoded));
  TEST_ASSERT_EQUAL_UINT32(report.uptime_ms, decoded.uptime_ms);
  TEST_ASSERT_TRUE(decoded.fault_active);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::FaultCode::CommunicationStale),
      static_cast<std::uint8_t>(decoded.fault_code));
  TEST_ASSERT_EQUAL_INT16(111, decoded.back_left_applied_command_milli);
  TEST_ASSERT_EQUAL_INT16(-222, decoded.back_right_applied_command_milli);
  TEST_ASSERT_TRUE(decoded.back_left_inverted);
  TEST_ASSERT_FALSE(decoded.back_right_inverted);
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_low_low_maps_to_zero_error);
  RUN_TEST(test_low_high_maps_to_positive_one);
  RUN_TEST(test_high_low_maps_to_negative_one);
  RUN_TEST(test_high_high_after_positive_history_maps_to_positive_five);
  RUN_TEST(test_high_high_after_negative_history_maps_to_negative_five);
  RUN_TEST(test_high_high_without_history_is_unsafe);
  RUN_TEST(test_both_low_preserves_last_known_side);
  RUN_TEST(test_zero_error_gives_zero_correction_after_reset);
  RUN_TEST(test_proportional_term_has_correct_sign);
  RUN_TEST(test_correction_clamps);
  RUN_TEST(test_integral_clamps);
  RUN_TEST(test_integral_does_not_accumulate_while_line_is_lost);
  RUN_TEST(test_derivative_uses_elapsed_time);
  RUN_TEST(test_derivative_clamps);
  RUN_TEST(test_reset_clears_pid_state);
  RUN_TEST(test_zero_correction_gives_equal_left_and_right_commands);
  RUN_TEST(test_positive_correction_changes_sides_oppositely);
  RUN_TEST(test_negative_polarity_reverses_correction);
  RUN_TEST(test_final_duties_remain_inside_limits);
  RUN_TEST(test_valid_rear_command_is_accepted);
  RUN_TEST(test_corrupt_rear_packet_is_rejected);
  RUN_TEST(test_stale_rear_command_stops_motors);
  RUN_TEST(test_explicit_stop_packet_stops_motors);
  RUN_TEST(test_mode_manager_starts_disabled);
  RUN_TEST(test_mode_manager_rejects_drive_while_disabled);
  RUN_TEST(test_mode_manager_accepts_sensor_mode_without_motors);
  RUN_TEST(test_emergency_stop_works_from_any_mode);
  RUN_TEST(test_command_validation_rejects_out_of_range_duty);
  RUN_TEST(test_command_validation_accepts_drive_test_duty_0_7);
  RUN_TEST(test_command_validation_rejects_overlong_duration);
  RUN_TEST(test_command_validation_rejects_malformed_motor_id);
  RUN_TEST(test_command_validation_rejects_invalid_pid_value);
  RUN_TEST(test_command_validation_rejects_mode_incompatible_motor_command);
  RUN_TEST(test_event_log_stores_newest_events_and_wraps);
  RUN_TEST(test_telemetry_json_contains_required_fields_and_booleans);
  RUN_TEST(test_esp1_status_packet_round_trips);
  return UNITY_END();
}
