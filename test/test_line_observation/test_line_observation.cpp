#include <cmath>
#include <cstring>
#include <limits>

#include <unity.h>

#include "common/ChassisMixer.h"
#include "common/Esp1Status.h"
#include "common/EventLog.h"
#include "common/FunnelCommand.h"
#include "common/LineFollower.h"
#include "common/LineObservation.h"
#include "common/RearDriveCommand.h"
#include "common/RobotCommandValidation.h"
#include "common/RobotTestModeManager.h"
#include "common/SolarPanelAutonomy.h"
#include "common/TelemetrySnapshot.h"

namespace {

void assertNear(const float expected, const float actual,
                const float tolerance) {
  TEST_ASSERT_TRUE(std::fabs(expected - actual) <= tolerance);
}

robot::LineFollowerConfig pidConfig() {
  robot::LineFollowerConfig config{};
  config.maxDuty = 0.6F;
  config.maxCorrection = 0.6F;
  config.integralLimit = 1.0F;
  config.derivativeLimit = 100.0F;
  return config;
}

robot::SolarPanelAutonomyConfig solarConfig() {
  return {100U, 60U, 200U, 0.0F, 500U, 5000U};
}

robot::SolarPanelContactConfig solarContactConfig() {
  return {100U, 0.25F, 10U, 20U, 30U, 40U};
}

void assertSolarState(const robot::SolarPanelAutonomyState expected,
                      const robot::SolarPanelAutonomyState actual) {
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(expected),
                          static_cast<std::uint8_t>(actual));
}

void test_both_on_tape_maps_to_zero_error() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, true, 0, 10U);

  TEST_ASSERT_TRUE(observation.leftOnTape);
  TEST_ASSERT_TRUE(observation.rightOnTape);
  TEST_ASSERT_TRUE(observation.left_black);
  TEST_ASSERT_TRUE(observation.right_black);
  TEST_ASSERT_EQUAL_INT8(0, observation.error);
  TEST_ASSERT_TRUE(observation.lineVisible);
  TEST_ASSERT_TRUE(observation.line_visible);
  TEST_ASSERT_TRUE(observation.safe_to_drive);
}

void test_left_on_tape_maps_to_positive_one() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, false, 0, 10U);

  TEST_ASSERT_EQUAL_INT8(1, observation.error);
  TEST_ASSERT_EQUAL_INT8(1, observation.last_known_side);
  TEST_ASSERT_TRUE(observation.line_visible);
}

void test_right_on_tape_maps_to_negative_one() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(false, true, 0, 10U);

  TEST_ASSERT_EQUAL_INT8(-1, observation.error);
  TEST_ASSERT_EQUAL_INT8(-1, observation.last_known_side);
  TEST_ASSERT_TRUE(observation.line_visible);
}

void test_both_off_tape_after_positive_history_maps_to_positive_five() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(false, false, 1, 10U);

  TEST_ASSERT_EQUAL_INT8(5, observation.error);
  TEST_ASSERT_EQUAL_INT8(1, observation.lastKnownSide);
  TEST_ASSERT_EQUAL_INT8(1, observation.last_known_side);
  TEST_ASSERT_FALSE(observation.line_visible);
  TEST_ASSERT_TRUE(observation.hasHistory);
  TEST_ASSERT_TRUE(observation.safe_to_drive);
}

void test_both_off_tape_after_negative_history_maps_to_negative_five() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(false, false, -1, 10U);

  TEST_ASSERT_EQUAL_INT8(-5, observation.error);
  TEST_ASSERT_EQUAL_INT8(-1, observation.last_known_side);
  TEST_ASSERT_FALSE(observation.line_visible);
  TEST_ASSERT_TRUE(observation.safe_to_drive);
}

void test_both_off_tape_without_history_is_unsafe() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(false, false, 0, 10U);

  TEST_ASSERT_EQUAL_INT8(0, observation.error);
  TEST_ASSERT_EQUAL_INT8(0, observation.last_known_side);
  TEST_ASSERT_FALSE(observation.line_visible);
  TEST_ASSERT_FALSE(observation.hasHistory);
  TEST_ASSERT_FALSE(observation.safe_to_drive);
}

void test_both_on_tape_preserves_last_known_side() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensors(true, true, -1, 10U);

  TEST_ASSERT_EQUAL_INT8(0, observation.error);
  TEST_ASSERT_EQUAL_INT8(-1, observation.last_known_side);
  TEST_ASSERT_TRUE(observation.line_visible);
}

void test_electrical_high_high_maps_to_both_on_tape() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensorLevels(true, true, 0, 25U);

  TEST_ASSERT_TRUE(observation.leftOnTape);
  TEST_ASSERT_TRUE(observation.rightOnTape);
  TEST_ASSERT_EQUAL_INT8(0, observation.error);
  TEST_ASSERT_TRUE(observation.lineVisible);
  TEST_ASSERT_EQUAL_UINT32(25U, observation.timestampMs);
}

void test_electrical_low_high_maps_to_right_on_tape() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensorLevels(false, true, 0, 25U);

  TEST_ASSERT_FALSE(observation.leftOnTape);
  TEST_ASSERT_TRUE(observation.rightOnTape);
  TEST_ASSERT_EQUAL_INT8(-1, observation.error);
  TEST_ASSERT_EQUAL_INT8(-1, observation.lastKnownSide);
  TEST_ASSERT_TRUE(observation.hasHistory);
}

void test_electrical_high_low_maps_to_left_on_tape() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensorLevels(true, false, 0, 25U);

  TEST_ASSERT_TRUE(observation.leftOnTape);
  TEST_ASSERT_FALSE(observation.rightOnTape);
  TEST_ASSERT_EQUAL_INT8(1, observation.error);
  TEST_ASSERT_EQUAL_INT8(1, observation.lastKnownSide);
  TEST_ASSERT_TRUE(observation.hasHistory);
}

void test_electrical_low_low_without_history_is_unsafe() {
  const robot::LineObservation observation =
      robot::observeDigitalLineSensorLevels(false, false, 0, 25U);

  TEST_ASSERT_FALSE(observation.leftOnTape);
  TEST_ASSERT_FALSE(observation.rightOnTape);
  TEST_ASSERT_FALSE(observation.lineVisible);
  TEST_ASSERT_FALSE(observation.hasHistory);
  TEST_ASSERT_FALSE(observation.safe_to_drive);
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
  config.maxCorrection = 0.25F;
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

void test_update_stops_when_line_lost_without_history() {
  robot::LineFollowerState state{};
  robot::LineFollowerConfig config = pidConfig();
  robot::startLineFollower(state, 100U);

  const robot::LineFollowerUpdate update =
      robot::updateLineFollower(state, false, false, config, 110U);

  TEST_ASSERT_FALSE(state.enabled);
  TEST_ASSERT_FALSE(update.should_drive);
  TEST_ASSERT_FALSE(update.wheel_command.front_left.enabled);
}

void test_zero_correction_gives_equal_left_and_right_commands() {
  robot::LineFollowerConfig config{};
  config.baseDuty = 0.2F;
  config.maxDuty = 0.5F;
  config.maxCorrection = 0.3F;

  const robot::FourWheelCommand command =
      robot::mixDifferentialLineFollow(0.0F, config, 100U);

  TEST_ASSERT_EQUAL_INT16(200, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(200, command.front_right.duty_command_milli);
}

void test_positive_correction_changes_sides_oppositely() {
  robot::LineFollowerConfig config{};
  config.baseDuty = 0.2F;
  config.maxDuty = 0.5F;
  config.maxCorrection = 0.3F;

  const robot::FourWheelCommand command =
      robot::mixDifferentialLineFollow(0.1F, config, 100U);

  TEST_ASSERT_EQUAL_INT16(100, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(300, command.front_right.duty_command_milli);
}

void test_negative_polarity_reverses_correction() {
  robot::LineFollowerConfig config{};
  config.baseDuty = 0.2F;
  config.maxDuty = 0.5F;
  config.maxCorrection = 0.3F;
  config.steeringPolarity = -1;

  const robot::FourWheelCommand command =
      robot::mixDifferentialLineFollow(0.1F, config, 100U);

  TEST_ASSERT_EQUAL_INT16(300, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(100, command.front_right.duty_command_milli);
}

void test_final_duties_remain_inside_limits() {
  robot::LineFollowerConfig config{};
  config.baseDuty = 0.5F;
  config.maxDuty = 0.4F;
  config.maxCorrection = 0.4F;

  const robot::FourWheelCommand command =
      robot::mixDifferentialLineFollow(0.4F, config, 100U);

  TEST_ASSERT_TRUE(std::abs(command.front_left.duty_command_milli) <= 400);
  TEST_ASSERT_TRUE(std::abs(command.front_right.duty_command_milli) <= 400);
}

void test_open_loop_right_strafe_uses_mecanum_signs() {
  const robot::FourWheelCommand command =
      robot::mixOpenLoopMecanum(1.0F, 0.0F, 0.0F, 0.25F, 100U, 700U);

  TEST_ASSERT_EQUAL_INT16(250, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-250, command.front_right.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-250, command.back_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(250, command.back_right.duty_command_milli);
  TEST_ASSERT_TRUE(command.front_left.enabled);
  TEST_ASSERT_TRUE(command.front_right.enabled);
  TEST_ASSERT_TRUE(command.back_left.enabled);
  TEST_ASSERT_TRUE(command.back_right.enabled);
  TEST_ASSERT_EQUAL_UINT32(800U, command.front_left.expires_at_ms);
}

void test_open_loop_left_strafe_uses_mecanum_signs() {
  const robot::FourWheelCommand command =
      robot::mixOpenLoopMecanum(-1.0F, 0.0F, 0.0F, 0.25F, 100U, 700U);

  TEST_ASSERT_EQUAL_INT16(-250, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(250, command.front_right.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(250, command.back_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-250, command.back_right.duty_command_milli);
  TEST_ASSERT_TRUE(command.front_left.enabled);
  TEST_ASSERT_TRUE(command.front_right.enabled);
  TEST_ASSERT_TRUE(command.back_left.enabled);
  TEST_ASSERT_TRUE(command.back_right.enabled);
  TEST_ASSERT_EQUAL_UINT32(800U, command.front_left.expires_at_ms);
}

void test_open_loop_forward_uses_equal_positive_mecanum_signs() {
  const robot::FourWheelCommand command =
      robot::mixOpenLoopMecanum(0.0F, 1.0F, 0.0F, 0.25F, 100U, 700U);

  TEST_ASSERT_EQUAL_INT16(250, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(250, command.front_right.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(250, command.back_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(250, command.back_right.duty_command_milli);
  TEST_ASSERT_TRUE(command.front_left.enabled);
  TEST_ASSERT_TRUE(command.front_right.enabled);
  TEST_ASSERT_TRUE(command.back_left.enabled);
  TEST_ASSERT_TRUE(command.back_right.enabled);
  TEST_ASSERT_EQUAL_UINT32(800U, command.front_left.expires_at_ms);
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

void test_valid_funnel_command_is_accepted() {
  robot::FunnelCommandReceiver receiver{};
  const robot::FunnelCommand command{true, -375, 100U, 250U};
  const robot::UartPacket packet = robot::makeFunnelCommandPacket(command, 9);

  TEST_ASSERT_TRUE(receiver.acceptPacket(packet, 100U));

  const robot::FunnelStatus status = receiver.status(120U);
  TEST_ASSERT_TRUE(status.link_healthy);
  TEST_ASSERT_TRUE(status.command_enabled);
  TEST_ASSERT_EQUAL_UINT16(9, status.last_sequence);
  TEST_ASSERT_EQUAL_INT16(-375, receiver.motorCommand(120U).duty_command_milli);
}

void test_stale_funnel_command_stops_motor() {
  robot::FunnelCommandReceiver receiver{};
  const robot::FunnelCommand command{true, 250, 100U, 50U};
  const robot::UartPacket packet = robot::makeFunnelCommandPacket(command, 9);

  TEST_ASSERT_TRUE(receiver.acceptPacket(packet, 100U));

  TEST_ASSERT_FALSE(receiver.motorCommand(151U).enabled);

  const robot::FunnelStatus status = receiver.status(351U);
  TEST_ASSERT_TRUE(status.command_enabled);
  TEST_ASSERT_TRUE(robot::enabledFunnelCommandIsStale(status, 250U));
}

void test_stale_disabled_funnel_command_does_not_report_motion_fault() {
  robot::FunnelCommandReceiver receiver{};
  const robot::FunnelCommand command{false, 0, 100U, 50U};
  const robot::UartPacket packet = robot::makeFunnelCommandPacket(command, 9);

  TEST_ASSERT_TRUE(receiver.acceptPacket(packet, 100U));

  const robot::FunnelStatus status = receiver.status(351U);
  TEST_ASSERT_FALSE(status.link_healthy);
  TEST_ASSERT_TRUE(status.has_valid_command);
  TEST_ASSERT_FALSE(status.command_enabled);
  TEST_ASSERT_FALSE(robot::enabledFunnelCommandIsStale(status, 250U));
  TEST_ASSERT_FALSE(receiver.motorCommand(351U).enabled);
}

void test_corrupt_funnel_packet_is_rejected() {
  robot::FunnelCommandReceiver receiver{};
  const robot::FunnelCommand command{true, 250, 100U, 50U};
  robot::UartPacket packet = robot::makeFunnelCommandPacket(command, 9);
  packet.payload[2] ^= 0x20U;

  TEST_ASSERT_FALSE(receiver.acceptPacket(packet, 100U));
  TEST_ASSERT_FALSE(receiver.status(100U).has_valid_command);
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

void test_mechanism_mode_is_not_sensor_only() {
  TEST_ASSERT_FALSE(
      robot::robotTestModeIsSensorOnly(robot::RobotTestMode::MechanismTest));
  TEST_ASSERT_FALSE(robot::robotTestModeAllowsMotion(
      robot::RobotTestMode::MechanismTest));
}

void test_autonomous_solar_mode_allows_motion_and_requires_rear_link() {
  robot::RobotTestMode mode{};

  TEST_ASSERT_TRUE(robot::parseRobotTestMode("autonomous-solar", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::AutonomousSolarPanel),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeIsSensorOnly(mode));
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
  config.maxDuty = 0.3F;
  config.maxCorrection = 0.2F;
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

void test_solar_contact_config_validation() {
  robot::SolarPanelContactConfig config = solarContactConfig();
  TEST_ASSERT_TRUE(robot::solarPanelContactConfigValid(config));

  config.strafe_start_delay_ms = 0U;
  config.retry_strafe_left_duration_ms = 0U;
  config.retry_forward_duration_ms = 0U;
  TEST_ASSERT_TRUE(robot::solarPanelContactConfigValid(config));

  config = solarContactConfig();
  config.timeout_ms = 0U;
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config = solarContactConfig();
  config.retry_strafe_timeout_ms = 0U;
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config = solarContactConfig();
  config.strafe_duty = -0.01F;
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config.strafe_duty = 1.01F;
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config.strafe_duty = std::numeric_limits<float>::quiet_NaN();
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));
}

void test_solar_retry_state_names_are_exposed() {
  TEST_ASSERT_EQUAL_STRING(
      "STRAFE_LEFT_FOR_SOLAR_RETRY",
      robot::solarPanelAutonomyStateName(
          robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry));
  TEST_ASSERT_EQUAL_STRING(
      "MOVE_FORWARD_FOR_SOLAR_RETRY",
      robot::solarPanelAutonomyStateName(
          robot::SolarPanelAutonomyState::MoveForwardForSolarRetry));
  TEST_ASSERT_EQUAL_STRING(
      "RETRY_STRAFE_RIGHT_TO_SOLAR_PANEL",
      robot::solarPanelAutonomyStateName(
          robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel));
}

void test_solar_front_only_at_first_timeout_begins_adjustment() {
  const robot::SolarPanelContactConfig config = solarContactConfig();

  robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::StrafeRightToSolarPanel, true,
          false, config.timeout_ms - 1U, config);
  assertSolarState(robot::SolarPanelAutonomyState::StrafeRightToSolarPanel,
                   update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::StrafeRightToSolarPanel, true, false,
      config.timeout_ms, config);
  assertSolarState(robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);
}

void test_solar_first_timeout_faults_without_front_only_contact() {
  const robot::SolarPanelContactConfig config = solarContactConfig();

  robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::StrafeRightToSolarPanel, false,
          false, config.timeout_ms, config);
  assertSolarState(robot::SolarPanelAutonomyState::SolarSearchFault,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::StrafeRightToSolarPanel, false, true,
      config.timeout_ms, config);
  assertSolarState(robot::SolarPanelAutonomyState::SolarSearchFault,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);
}

void test_solar_adjustment_runs_left_then_forward_then_right() {
  const robot::SolarPanelContactConfig config = solarContactConfig();

  robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry, false,
          false, config.retry_strafe_left_duration_ms - 1U, config);
  assertSolarState(robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry,
                   update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry, false, false,
      config.retry_strafe_left_duration_ms, config);
  assertSolarState(robot::SolarPanelAutonomyState::MoveForwardForSolarRetry,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::MoveForwardForSolarRetry, false, false,
      config.retry_forward_duration_ms - 1U, config);
  assertSolarState(robot::SolarPanelAutonomyState::MoveForwardForSolarRetry,
                   update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::MoveForwardForSolarRetry, false, false,
      config.retry_forward_duration_ms, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel,
      update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);
}

void test_solar_zero_adjustment_durations_transition_immediately() {
  robot::SolarPanelContactConfig config = solarContactConfig();
  config.retry_strafe_left_duration_ms = 0U;
  config.retry_forward_duration_ms = 0U;

  robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry, false,
          false, 0U, config);
  assertSolarState(robot::SolarPanelAutonomyState::MoveForwardForSolarRetry,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::MoveForwardForSolarRetry, false, false,
      0U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel,
      update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);
}

void test_solar_second_strafe_times_out_without_another_adjustment() {
  const robot::SolarPanelContactConfig config = solarContactConfig();

  robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel, true,
          false, config.retry_strafe_timeout_ms - 1U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel,
      update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel, true,
      false, config.retry_strafe_timeout_ms, config);
  assertSolarState(robot::SolarPanelAutonomyState::SolarSearchFault,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, false, config.retry_strafe_timeout_ms, config);
  assertSolarState(robot::SolarPanelAutonomyState::SolarSearchFault,
                   update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);
}

void test_solar_second_strafe_timeout_is_independently_adjustable() {
  robot::SolarPanelContactConfig config = solarContactConfig();
  config.retry_strafe_timeout_ms = 80U;

  robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel, true,
          false, 40U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel,
      update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel, true,
      false, 80U, config);
  assertSolarState(robot::SolarPanelAutonomyState::SolarSearchFault,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);
}

void test_solar_all_hit_has_priority_in_every_contact_motion_state() {
  const robot::SolarPanelContactConfig config = solarContactConfig();
  const robot::SolarPanelAutonomyState states[] = {
      robot::SolarPanelAutonomyState::StrafeRightToSolarPanel,
      robot::SolarPanelAutonomyState::StrafeLeftForSolarRetry,
      robot::SolarPanelAutonomyState::MoveForwardForSolarRetry,
      robot::SolarPanelAutonomyState::RetryStrafeRightToSolarPanel,
  };

  for (const robot::SolarPanelAutonomyState state : states) {
    const robot::SolarPanelContactSequenceUpdate update =
        robot::updateSolarPanelContactSequence(state, true, true, 1000U,
                                               config);
    assertSolarState(robot::SolarPanelAutonomyState::SolarPanelContacted,
                     update.next_state);
    TEST_ASSERT_TRUE(update.transitioned);
  }
}

void test_solar_non_contact_state_is_unchanged() {
  const robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::LineFollowToSolar, true, true,
          1000U, solarContactConfig());

  assertSolarState(robot::SolarPanelAutonomyState::LineFollowToSolar,
                   update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);
}

void test_solar_detector_no_beacon_does_not_confirm() {
  robot::SolarBeaconDetectorState state{};
  const robot::SolarPanelAutonomyConfig config = solarConfig();

  robot::SolarBeaconDetectorUpdate update{};
  for (robot::Milliseconds now = 100U; now <= 600U; now += 100U) {
    update = robot::updateSolarBeaconDetector(state, 20U, config, now, true);
  }

  TEST_ASSERT_FALSE(update.confirmation_active);
  TEST_ASSERT_FALSE(update.beacon_detected);
  TEST_ASSERT_EQUAL_UINT32(0U, update.confirmation_progress_ms);
}

void test_solar_detector_brief_spike_does_not_confirm() {
  robot::SolarBeaconDetectorState state{};
  const robot::SolarPanelAutonomyConfig config = solarConfig();

  robot::SolarBeaconDetectorUpdate update =
      robot::updateSolarBeaconDetector(state, 150U, config, 100U, true);
  TEST_ASSERT_TRUE(update.confirmation_active);
  TEST_ASSERT_FALSE(update.beacon_detected);

  update = robot::updateSolarBeaconDetector(state, 20U, config, 150U, true);
  TEST_ASSERT_FALSE(update.confirmation_active);
  TEST_ASSERT_FALSE(update.beacon_detected);

  update = robot::updateSolarBeaconDetector(state, 20U, config, 350U, true);
  TEST_ASSERT_FALSE(update.beacon_detected);
}

void test_solar_detector_sustained_beacon_confirms() {
  robot::SolarBeaconDetectorState state{};
  const robot::SolarPanelAutonomyConfig config = solarConfig();

  robot::updateSolarBeaconDetector(state, 150U, config, 100U, true);
  robot::SolarBeaconDetectorUpdate update =
      robot::updateSolarBeaconDetector(state, 150U, config, 250U, true);
  TEST_ASSERT_FALSE(update.beacon_detected);

  update = robot::updateSolarBeaconDetector(state, 150U, config, 300U, true);
  TEST_ASSERT_TRUE(update.beacon_detected);
  TEST_ASSERT_EQUAL_UINT32(config.confirmation_time_ms,
                           update.confirmation_progress_ms);
}

void test_solar_detector_hysteresis_holds_until_release_threshold() {
  robot::SolarBeaconDetectorState state{};
  const robot::SolarPanelAutonomyConfig config = solarConfig();

  robot::updateSolarBeaconDetector(state, 150U, config, 100U, true);
  robot::SolarBeaconDetectorUpdate update =
      robot::updateSolarBeaconDetector(state, 90U, config, 200U, true);
  TEST_ASSERT_TRUE(update.confirmation_active);
  TEST_ASSERT_EQUAL_UINT32(100U, update.confirmation_progress_ms);

  update = robot::updateSolarBeaconDetector(state, 50U, config, 250U, true);
  TEST_ASSERT_FALSE(update.confirmation_active);
  TEST_ASSERT_EQUAL_UINT32(0U, update.confirmation_progress_ms);
  TEST_ASSERT_FALSE(update.beacon_detected);
}

void test_solar_detector_ignore_window_blocks_confirmation() {
  robot::SolarBeaconDetectorState state{};
  const robot::SolarPanelAutonomyConfig config = solarConfig();

  robot::SolarBeaconDetectorUpdate update =
      robot::updateSolarBeaconDetector(state, 150U, config, 100U, false);
  TEST_ASSERT_FALSE(update.confirmation_active);
  TEST_ASSERT_FALSE(update.beacon_detected);

  update = robot::updateSolarBeaconDetector(state, 150U, config, 600U, true);
  TEST_ASSERT_TRUE(update.confirmation_active);
  TEST_ASSERT_FALSE(update.beacon_detected);
  TEST_ASSERT_EQUAL_UINT32(0U, update.confirmation_progress_ms);
}

void test_solar_detector_reset_clears_state() {
  robot::SolarBeaconDetectorState state{};
  const robot::SolarPanelAutonomyConfig config = solarConfig();
  robot::updateSolarBeaconDetector(state, 150U, config, 100U, true);
  robot::updateSolarBeaconDetector(state, 150U, config, 300U, true);

  robot::resetSolarBeaconDetectorState(state);

  TEST_ASSERT_FALSE(state.filter_initialized);
  TEST_ASSERT_FALSE(state.confirmation_active);
  TEST_ASSERT_FALSE(state.beacon_detected);
  TEST_ASSERT_EQUAL_UINT32(0U, state.confirmation_progress_ms);
}

void test_telemetry_json_contains_required_fields_and_booleans() {
  robot::TelemetrySnapshot snapshot{};
  snapshot.uptime_ms = 123U;
  snapshot.current_mode = robot::RobotTestMode::LineSensorTest;
  snapshot.enabled = false;
  snapshot.lss_raw_level = 1;
  snapshot.lsfl_black = true;
  snapshot.lsfr_black = false;
  snapshot.lss_black = true;
  snapshot.lss_configured = true;
  snapshot.front_left.desired_command_milli = 100;
  snapshot.front_right.desired_command_milli = -100;
  snapshot.funnel.desired_command_milli = 250;
  snapshot.funnel.applied_command_milli = 250;
  snapshot.funnel.enabled = true;
  snapshot.funnel.configured = true;
  snapshot.esp1.funnel_applied_command_milli = 250;
  snapshot.esp1.funnel_configured = true;
  snapshot.ir_adc_average = 1800U;
  snapshot.ir_adc_min = 1200U;
  snapshot.ir_adc_max = 2200U;
  snapshot.ir_amplitude_pp = 1000U;
  snapshot.ir_beacon_detected = true;
  snapshot.ir_switch_raw_state = true;
  snapshot.ir_switch_debounced_state = true;
  snapshot.selected_beacon_frequency_hz = 1000U;
  snapshot.ir_adc_latest_sample = 1900U;
  snapshot.ir_adc_sample_mean = 1800U;
  snapshot.ir_1khz_goertzel_amplitude = 400U;
  snapshot.ir_10khz_goertzel_amplitude = 30U;
  snapshot.ir_selected_frequency_amplitude = 400U;
  snapshot.ir_active_threshold = 120U;
  snapshot.ir_consecutive_detection_count = 3U;
  snapshot.ir_adc_sample_rate_hz = 50000U;
  snapshot.motor_command_magnitude_milli = 300U;
  snapshot.autonomous_state =
      robot::SolarPanelAutonomyState::SolarBeaconAligned;
  snapshot.autonomous_fault_reason = robot::SolarPanelFaultReason::None;
  snapshot.autonomous_time_in_state_ms = 2500U;
  snapshot.solar_ir_raw_amplitude = 400U;
  snapshot.solar_ir_filtered_amplitude = 380.5F;
  snapshot.solar_ir_detection_threshold = 120U;
  snapshot.solar_ir_release_threshold = 80U;
  snapshot.solar_ir_detection_threshold_1khz = 120U;
  snapshot.solar_ir_release_threshold_1khz = 80U;
  snapshot.solar_ir_detection_threshold_10khz = 220U;
  snapshot.solar_ir_release_threshold_10khz = 160U;
  snapshot.solar_ir_confirmation_progress_ms = 300U;
  snapshot.solar_ir_confirmation_time_ms = 300U;
  snapshot.solar_ir_filter_alpha = 0.75F;
  snapshot.solar_ir_confirmation_active = true;
  snapshot.solar_beacon_confirmed = true;
  snapshot.solar_ir_ignore_after_start_ms = 500U;
  snapshot.solar_search_timeout_ms = 30000U;
  snapshot.solar_start_base_duty = 0.25F;
  snapshot.solar_slow_after_ms = 7000U;
  snapshot.solar_slow_base_duty = 0.15F;
  snapshot.solar_slow_mode_active = true;
  snapshot.solar_contact_timeout_ms = 4000U;
  snapshot.solar_contact_strafe_duty = 0.12F;
  snapshot.solar_strafe_start_delay_ms = 300U;
  snapshot.solar_retry_strafe_left_duration_ms = 321U;
  snapshot.solar_retry_forward_duration_ms = 654U;
  snapshot.solar_retry_strafe_timeout_ms = 987U;
  snapshot.solar_panel_limit_switches_configured = true;
  snapshot.solar_limit_back_right_high = true;
  snapshot.solar_limit_front_right_high = false;
  snapshot.solar_limit_back_right_hit = true;
  snapshot.solar_limit_front_right_hit = false;
  snapshot.solar_limit_all_hit = false;
  snapshot.esp1.solar_panel_limit_switches_configured = true;
  snapshot.esp1.solar_limit_back_right_high = true;
  snapshot.esp1.solar_limit_front_right_high = false;
  snapshot.esp1.side_line_sensor_configured = true;
  snapshot.esp1.side_line_sensor_high = true;
  snapshot.claws.claw_1.hardware_configured = true;
  snapshot.claws.claw_1.start_configured = true;
  snapshot.claws.claw_1.start_angle_deg = 30;
  snapshot.claws.claw_1.open_angle_deg = 120;
  snapshot.claws.claw_1.commanded_angle_deg = 120;
  snapshot.claws.claw_1.commanded_open = true;

  char output[8192]{};
  TEST_ASSERT_TRUE(
      robot::writeTelemetryJson(snapshot, output, sizeof(output), false));

  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"current_mode\":\"LINE_SENSOR_TEST\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"enabled\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lsfl_black\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lsfl_level\":\"UNKNOWN\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_raw_level\":1"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_level\":\"HIGH\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_black\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_configured\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"hasHistory\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"maxDuty\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"front_left\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"funnel\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_applied_command_milli\":250"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"funnel_configured\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"line_error\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"ir_adc_average\":1800"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"ir_beacon_detected\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"selectedBeaconFrequencyHz\":1000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"switchRawState\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"ir_1khz_goertzel_amplitude\":400"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"ir_adc_sample_rate_hz\":50000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"motor_command_magnitude_milli\":300"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"autonomous_state\":\"SOLAR_BEACON_ALIGNED\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"autonomous\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"ir_filtered_amplitude\":380.50"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"ir_detection_threshold_10khz\":220"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"filter_alpha\":0.75000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"start_base_duty\":0.25000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"slow_after_ms\":7000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"slow_base_duty\":0.15000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"slow_mode_active\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"contact_timeout_ms\":4000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"strafe_duty\":0.12000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"strafe_start_delay_ms\":300"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"retry_strafe_left_duration_ms\":321"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"retry_forward_duration_ms\":654"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"retry_strafe_timeout_ms\":987"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"solarLimitSwitches\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"backRightHigh\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"frontRightHigh\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"backRightHit\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"frontRightHit\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"allHit\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output,
                  "\"solar_panel_limit_switches_configured\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"confirmation_progress_ms\":300"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"claws\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"rotation_deg\":90"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"claw_1\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"commandedAngleDeg\":120"));

  snapshot.lss_raw_level = 0;
  snapshot.lss_black = false;
  snapshot.esp1.side_line_sensor_high = false;
  TEST_ASSERT_TRUE(
      robot::writeTelemetryJson(snapshot, output, sizeof(output), false));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_raw_level\":0"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_level\":\"LOW\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_black\":false"));
}

void test_esp1_status_packet_round_trips() {
  robot::Esp1StatusReport report{
      1234U, robot::RobotTestMode::Disabled, true,
      robot::FaultCode::CommunicationStale, 111, -222, true, false};
  report.ir_adc_average = 1800U;
  report.ir_adc_min = 1100U;
  report.ir_adc_max = 2300U;
  report.ir_amplitude_pp = 1200U;
  report.ir_beacon_detected = true;
  report.ir_switch_raw_high = false;
  report.ir_switch_debounced_high = false;
  report.ir_selected_frequency_hz = 10000U;
  report.ir_adc_latest_sample = 1700U;
  report.ir_1khz_amplitude = 25U;
  report.ir_10khz_amplitude = 450U;
  report.ir_selected_amplitude = 450U;
  report.ir_active_threshold = 120U;
  report.ir_consecutive_detection_count = 4U;
  report.ir_adc_sample_rate_hz = 50000U;
  report.funnel_applied_command_milli = -333;
  report.funnel_configured = true;
  report.solar_panel_limit_switches_configured = true;
  report.solar_limit_back_right_high = true;
  report.solar_limit_front_right_high = false;
  report.side_line_sensor_configured = true;
  report.side_line_sensor_high = true;

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
  TEST_ASSERT_EQUAL_UINT16(1800U, decoded.ir_adc_average);
  TEST_ASSERT_EQUAL_UINT16(1100U, decoded.ir_adc_min);
  TEST_ASSERT_EQUAL_UINT16(2300U, decoded.ir_adc_max);
  TEST_ASSERT_EQUAL_UINT16(1200U, decoded.ir_amplitude_pp);
  TEST_ASSERT_TRUE(decoded.ir_beacon_detected);
  TEST_ASSERT_FALSE(decoded.ir_switch_raw_high);
  TEST_ASSERT_FALSE(decoded.ir_switch_debounced_high);
  TEST_ASSERT_EQUAL_UINT16(10000U, decoded.ir_selected_frequency_hz);
  TEST_ASSERT_EQUAL_UINT16(1700U, decoded.ir_adc_latest_sample);
  TEST_ASSERT_EQUAL_UINT16(25U, decoded.ir_1khz_amplitude);
  TEST_ASSERT_EQUAL_UINT16(450U, decoded.ir_10khz_amplitude);
  TEST_ASSERT_EQUAL_UINT16(450U, decoded.ir_selected_amplitude);
  TEST_ASSERT_EQUAL_UINT16(120U, decoded.ir_active_threshold);
  TEST_ASSERT_EQUAL_UINT8(4U, decoded.ir_consecutive_detection_count);
  TEST_ASSERT_EQUAL_UINT32(50000U, decoded.ir_adc_sample_rate_hz);
  TEST_ASSERT_EQUAL_INT16(-333, decoded.funnel_applied_command_milli);
  TEST_ASSERT_TRUE(decoded.funnel_configured);
  TEST_ASSERT_TRUE(decoded.solar_panel_limit_switches_configured);
  TEST_ASSERT_TRUE(decoded.solar_limit_back_right_high);
  TEST_ASSERT_FALSE(decoded.solar_limit_front_right_high);
  TEST_ASSERT_TRUE(decoded.side_line_sensor_configured);
  TEST_ASSERT_TRUE(decoded.side_line_sensor_high);

  report.side_line_sensor_high = false;
  const robot::UartPacket low_packet =
      robot::makeEsp1StatusPacket(report, 43U);
  decoded = {};
  TEST_ASSERT_TRUE(robot::decodeEsp1StatusPacket(low_packet, decoded));
  TEST_ASSERT_TRUE(decoded.side_line_sensor_configured);
  TEST_ASSERT_FALSE(decoded.side_line_sensor_high);
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_both_on_tape_maps_to_zero_error);
  RUN_TEST(test_left_on_tape_maps_to_positive_one);
  RUN_TEST(test_right_on_tape_maps_to_negative_one);
  RUN_TEST(test_both_off_tape_after_positive_history_maps_to_positive_five);
  RUN_TEST(test_both_off_tape_after_negative_history_maps_to_negative_five);
  RUN_TEST(test_both_off_tape_without_history_is_unsafe);
  RUN_TEST(test_both_on_tape_preserves_last_known_side);
  RUN_TEST(test_electrical_high_high_maps_to_both_on_tape);
  RUN_TEST(test_electrical_low_high_maps_to_right_on_tape);
  RUN_TEST(test_electrical_high_low_maps_to_left_on_tape);
  RUN_TEST(test_electrical_low_low_without_history_is_unsafe);
  RUN_TEST(test_zero_error_gives_zero_correction_after_reset);
  RUN_TEST(test_proportional_term_has_correct_sign);
  RUN_TEST(test_correction_clamps);
  RUN_TEST(test_integral_clamps);
  RUN_TEST(test_integral_does_not_accumulate_while_line_is_lost);
  RUN_TEST(test_derivative_uses_elapsed_time);
  RUN_TEST(test_derivative_clamps);
  RUN_TEST(test_reset_clears_pid_state);
  RUN_TEST(test_update_stops_when_line_lost_without_history);
  RUN_TEST(test_zero_correction_gives_equal_left_and_right_commands);
  RUN_TEST(test_positive_correction_changes_sides_oppositely);
  RUN_TEST(test_negative_polarity_reverses_correction);
  RUN_TEST(test_final_duties_remain_inside_limits);
  RUN_TEST(test_open_loop_right_strafe_uses_mecanum_signs);
  RUN_TEST(test_open_loop_left_strafe_uses_mecanum_signs);
  RUN_TEST(test_open_loop_forward_uses_equal_positive_mecanum_signs);
  RUN_TEST(test_valid_rear_command_is_accepted);
  RUN_TEST(test_corrupt_rear_packet_is_rejected);
  RUN_TEST(test_stale_rear_command_stops_motors);
  RUN_TEST(test_explicit_stop_packet_stops_motors);
  RUN_TEST(test_valid_funnel_command_is_accepted);
  RUN_TEST(test_stale_funnel_command_stops_motor);
  RUN_TEST(test_stale_disabled_funnel_command_does_not_report_motion_fault);
  RUN_TEST(test_corrupt_funnel_packet_is_rejected);
  RUN_TEST(test_mode_manager_starts_disabled);
  RUN_TEST(test_mode_manager_rejects_drive_while_disabled);
  RUN_TEST(test_mode_manager_accepts_sensor_mode_without_motors);
  RUN_TEST(test_mechanism_mode_is_not_sensor_only);
  RUN_TEST(test_autonomous_solar_mode_allows_motion_and_requires_rear_link);
  RUN_TEST(test_emergency_stop_works_from_any_mode);
  RUN_TEST(test_command_validation_rejects_out_of_range_duty);
  RUN_TEST(test_command_validation_accepts_drive_test_duty_0_7);
  RUN_TEST(test_command_validation_rejects_overlong_duration);
  RUN_TEST(test_command_validation_rejects_malformed_motor_id);
  RUN_TEST(test_command_validation_rejects_invalid_pid_value);
  RUN_TEST(test_command_validation_rejects_mode_incompatible_motor_command);
  RUN_TEST(test_event_log_stores_newest_events_and_wraps);
  RUN_TEST(test_solar_contact_config_validation);
  RUN_TEST(test_solar_retry_state_names_are_exposed);
  RUN_TEST(test_solar_front_only_at_first_timeout_begins_adjustment);
  RUN_TEST(test_solar_first_timeout_faults_without_front_only_contact);
  RUN_TEST(test_solar_adjustment_runs_left_then_forward_then_right);
  RUN_TEST(test_solar_zero_adjustment_durations_transition_immediately);
  RUN_TEST(test_solar_second_strafe_times_out_without_another_adjustment);
  RUN_TEST(test_solar_second_strafe_timeout_is_independently_adjustable);
  RUN_TEST(test_solar_all_hit_has_priority_in_every_contact_motion_state);
  RUN_TEST(test_solar_non_contact_state_is_unchanged);
  RUN_TEST(test_solar_detector_no_beacon_does_not_confirm);
  RUN_TEST(test_solar_detector_brief_spike_does_not_confirm);
  RUN_TEST(test_solar_detector_sustained_beacon_confirms);
  RUN_TEST(test_solar_detector_hysteresis_holds_until_release_threshold);
  RUN_TEST(test_solar_detector_ignore_window_blocks_confirmation);
  RUN_TEST(test_solar_detector_reset_clears_state);
  RUN_TEST(test_telemetry_json_contains_required_fields_and_booleans);
  RUN_TEST(test_esp1_status_packet_round_trips);
  return UNITY_END();
}
