#include <cmath>
#include <cstring>
#include <limits>

#include <unity.h>

#include "common/ChassisMixer.h"
#include "common/Esp1Status.h"
#include "common/EventLog.h"
#include "common/FunnelCommand.h"
#include "common/HabitatDistanceStop.h"
#include "common/HabitatPiecesAutonomy.h"
#include "common/HabitatPlacementAutonomy.h"
#include "common/ImuHeadingHoldController.h"
#include "common/ImuRecovery.h"
#include "common/ImuTurnController.h"
#include "common/LineFollower.h"
#include "common/LineObservation.h"
#include "common/LaserDistance.h"
#include "common/MotionDiagnostics.h"
#include "common/PegFinderAutonomy.h"
#include "common/RearDriveCommand.h"
#include "common/RearLineSensor.h"
#include "common/RobotCommandValidation.h"
#include "common/RobotTestModeManager.h"
#include "common/SolarHookServo.h"
#include "common/SolarPanelAutonomy.h"
#include "common/TelemetrySnapshot.h"
#include "common/TimeTrialAutonomy.h"
#include "common/TowerPiecesAutonomy.h"
#include "common/Ultrasonic.h"

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
  return {100U, 0.25F, 0.2F, 0.22F, 0.25F, 10U, 20U, 30U, 40U,
          1000U, 0U, 0U, 0.19F, 0.18F, 250U};
}

robot::TowerPiecesConfig towerPiecesConfig() {
  robot::TowerPiecesConfig config{};
  config.reverse_line_duty = 0.2F;
  config.side_line_timeout_ms = 5000U;
  config.side_line_cooldown_ms = 50U;
  config.side_line_rearm_ms = 20U;
  config.post_line_delay_ms = 100U;
  config.strafe_right_duty = 0.25F;
  config.strafe_right_duration_ms = 200U;
  config.post_strafe_pause_ms = 150U;
  config.clockwise_rotation_angle_deg = 90.0F;
  config.post_rotation_pause_ms = 100U;
  config.reverse_duty = 0.2F;
  config.reverse_duration_ms = 300U;
  config.shimmy_right_duration_ms = 80U;
  config.shimmy_left_duration_ms = 120U;
  config.shimmy_timeout_ms = 1000U;
  config.shimmy_duty = 0.2F;
  config.final_reverse_duty = 0.17F;
  config.final_reverse_duration_ms = 90U;
  config.post_final_reverse_delay_ms = 10U;
  config.post_winch_open_delay_ms = 11U;
  config.post_claws_open_delay_ms = 12U;
  config.pre_stepper_bottom_delay_ms = 0U;
  config.stepper_down_speed_steps_per_second = 2000U;
  config.post_stepper_bottom_delay_ms = 13U;
  config.post_claws_closed_delay_ms = 14U;
  config.stepper_up_speed_steps_per_second = 1800U;
  return config;
}

robot::PegFinderConfig pegFinderConfig() {
  robot::PegFinderConfig config{};
  config.clockwise_angle_deg = 10.0F;
  config.post_rotation_pause_ms = 11U;
  config.reverse_duty = 0.25F;
  config.reverse_duration_ms = 12U;
  config.post_reverse_pause_ms = 13U;
  config.forward_duty = 0.22F;
  config.forward_duration_ms = 14U;
  config.funnel_forward_duty = 0.2F;
  config.funnel_forward_timeout_ms = 15U;
  config.post_funnel_limit_delay_ms = 16U;
  config.claw_open_interval_ms = 17U;
  config.claw_open_order_1 = 1U;
  config.claw_open_order_2 = 2U;
  config.claw_open_order_3 = 3U;
  config.post_claws_open_delay_ms = 18U;
  config.funnel_reverse_duty = 0.19F;
  config.funnel_reverse_duration_ms = 20U;
  return config;
}

robot::ImuTurnConfig imuTurnConfig() {
  robot::ImuTurnConfig config{};
  config.maximum_rotation_duty = 0.3F;
  config.kp = 0.01F;
  config.kd = 0.02F;
  config.angle_tolerance_deg = 2.0F;
  config.maximum_finishing_yaw_rate_dps = 3.0F;
  config.settling_time_ms = 100U;
  config.timeout_ms = 2000U;
  config.yaw_command_polarity = 1;
  return config;
}

robot::ImuHeadingHoldConfig imuHeadingHoldConfig() {
  robot::ImuHeadingHoldConfig config{};
  config.maximum_strafe_duty = 0.25F;
  config.kp = 0.01F;
  config.kd = 0.02F;
  config.maximum_yaw_correction_duty = 0.1F;
  config.yaw_command_polarity = 1;
  return config;
}

robot::PegFinderUpdate updatePegFinderForTest(
    robot::PegFinderAutonomy& autonomy,
    const robot::PegFinderConfig& config,
    const robot::Milliseconds now_ms,
    const bool funnel_limit_active = false,
    const bool clockwise_turn_complete = false) {
  const robot::PegFinderInputs inputs{
      funnel_limit_active, clockwise_turn_complete};
  return robot::updatePegFinderAutonomy(autonomy, inputs, config, now_ms);
}

robot::TowerPiecesUpdate updateTowerPiecesForTest(
    robot::TowerPiecesAutonomy& autonomy, const bool side_line_high,
    const bool back_left_line_high, const bool back_right_line_high,
    const robot::TowerPiecesConfig& config,
    const robot::Milliseconds now_ms,
    const bool bottom_limit_active = false,
    const bool top_limit_active = false,
    const bool clockwise_turn_complete = false) {
  const robot::TowerPiecesInputs inputs{
      side_line_high, back_left_line_high, back_right_line_high,
      bottom_limit_active, top_limit_active,
      clockwise_turn_complete};
  return robot::updateTowerPiecesAutonomy(autonomy, inputs, config, now_ms);
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

void test_reverse_rear_observation_swaps_physical_sensor_sides() {
  const robot::LineObservation back_right_on_tape =
      robot::observeRearLineSensorsForReverseTravel(false, true, 0, 25U);

  TEST_ASSERT_TRUE(back_right_on_tape.left_black);
  TEST_ASSERT_FALSE(back_right_on_tape.right_black);
  TEST_ASSERT_EQUAL_INT8(1, back_right_on_tape.error);

  const robot::LineObservation back_left_on_tape =
      robot::observeRearLineSensorsForReverseTravel(true, false, 0, 30U);

  TEST_ASSERT_FALSE(back_left_on_tape.left_black);
  TEST_ASSERT_TRUE(back_left_on_tape.right_black);
  TEST_ASSERT_EQUAL_INT8(-1, back_left_on_tape.error);
}

void test_reverse_rear_config_preserves_pid_and_negates_base() {
  robot::LineFollowerConfig configured = pidConfig();
  configured.kp = 0.31F;
  configured.ki = 0.04F;
  configured.kd = 0.07F;
  configured.baseDuty = 0.22F;
  configured.steeringPolarity = -1;

  const robot::LineFollowerConfig reverse =
      robot::makeReverseTravelLineFollowerConfig(configured);

  assertNear(0.31F, reverse.kp, 0.0001F);
  assertNear(0.04F, reverse.ki, 0.0001F);
  assertNear(0.07F, reverse.kd, 0.0001F);
  assertNear(-0.22F, reverse.baseDuty, 0.0001F);
  TEST_ASSERT_EQUAL_INT(-1, reverse.steeringPolarity);
}

void test_reverse_rear_follow_drives_backward_and_steers_in_travel_frame() {
  robot::LineFollowerConfig configured = pidConfig();
  configured.kp = 0.1F;
  configured.baseDuty = 0.2F;
  const robot::LineFollowerConfig reverse =
      robot::makeReverseTravelLineFollowerConfig(configured);
  const robot::LineObservation observation =
      robot::observeRearLineSensorsForReverseTravel(false, true, 0, 100U);
  robot::LineFollowerState state{};
  robot::startLineFollower(state, 100U);

  const robot::LineFollowerUpdate update = robot::updateLineFollower(
      state, observation.left_black, observation.right_black, reverse, 100U);

  TEST_ASSERT_TRUE(update.should_drive);
  TEST_ASSERT_EQUAL_INT16(-300,
                          update.wheel_command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-100,
                          update.wheel_command.front_right.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-300,
                          update.wheel_command.back_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-100,
                          update.wheel_command.back_right.duty_command_milli);
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

void test_open_loop_clockwise_rotation_uses_mecanum_signs() {
  const robot::FourWheelCommand command =
      robot::mixOpenLoopMecanum(0.0F, 0.0F, 1.0F, 0.25F, 100U, 700U);

  TEST_ASSERT_EQUAL_INT16(250, command.front_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-250, command.front_right.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(250, command.back_left.duty_command_milli);
  TEST_ASSERT_EQUAL_INT16(-250, command.back_right.duty_command_milli);
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
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::LaserDistanceProfile::HighAccuracy),
      static_cast<std::uint8_t>(receiver.laserProfile(120U)));
}

void test_rear_command_keeps_high_accuracy_when_stale() {
  robot::RearDriveCommandReceiver receiver{};
  robot::RearDriveCommand command{};
  command.sender_timestamp_ms = 100U;
  command.timeout_ms = 50U;
  command.laser_profile = robot::LaserDistanceProfile::HighAccuracy;
  const robot::UartPacket packet =
      robot::makeRearDriveCommandPacket(command, 8U);

  TEST_ASSERT_TRUE(receiver.acceptPacket(packet, 100U));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::LaserDistanceProfile::HighAccuracy),
      static_cast<std::uint8_t>(receiver.laserProfile(150U)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::LaserDistanceProfile::HighAccuracy),
      static_cast<std::uint8_t>(receiver.laserProfile(151U)));
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

void test_solar_hook_command_packet_round_trips() {
  const robot::SolarHookCommand expected{true, 137U};
  const robot::UartPacket packet =
      robot::makeSolarHookCommandPacket(expected, 17U);
  robot::SolarHookCommand decoded{};

  TEST_ASSERT_TRUE(robot::decodeSolarHookCommandPacket(packet, decoded));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::UartMessageType::MechanismCommand),
      static_cast<std::uint8_t>(packet.header.message_type));
  TEST_ASSERT_EQUAL_UINT16(17U, packet.header.sequence);
  TEST_ASSERT_TRUE(decoded.enabled);
  TEST_ASSERT_EQUAL_UINT8(137U, decoded.angle_deg);
}

void test_solar_hook_command_rejects_invalid_payload() {
  robot::UartPacket packet =
      robot::makeSolarHookCommandPacket({true, 90U}, 18U);
  robot::SolarHookCommand decoded{};

  packet.payload[0] = robot::kMechanismPayloadTargetFunnel;
  packet.header.integrity_crc16 = robot::calculatePacketIntegrity(packet);
  TEST_ASSERT_FALSE(
      robot::decodeSolarHookCommandPacket(packet, decoded));

  packet = robot::makeSolarHookCommandPacket({true, 90U}, 19U);
  packet.payload[2] = 181U;
  packet.header.integrity_crc16 = robot::calculatePacketIntegrity(packet);
  TEST_ASSERT_FALSE(
      robot::decodeSolarHookCommandPacket(packet, decoded));
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

void test_rear_line_modes_parse_and_enforce_motion_policy() {
  robot::RobotTestMode mode{};
  TEST_ASSERT_TRUE(robot::parseRobotTestMode("rear-line-sensor", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::RearLineSensorTest),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeIsSensorOnly(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_EQUAL_STRING("REAR_LINE_SENSOR_TEST",
                           robot::robotTestModeName(mode));

  TEST_ASSERT_TRUE(robot::parseRobotTestMode("rear-line-follow", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::RearLineFollowTest),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeIsSensorOnly(mode));
  TEST_ASSERT_EQUAL_STRING("REAR_LINE_FOLLOW_TEST",
                           robot::robotTestModeName(mode));

  robot::RobotTestModeManager manager{};
  manager.setMode(mode, 10U);
  TEST_ASSERT_TRUE(manager.acceptsLineFollowerCommand());
}

void test_tower_pieces_mode_parses_and_allows_distributed_motion() {
  robot::RobotTestMode mode{};

  TEST_ASSERT_TRUE(robot::parseRobotTestMode("tower-pieces", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::RobotTestMode::AutonomousTowerPieces),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_EQUAL_STRING("AUTONOMOUS_TOWER_PIECES",
                           robot::robotTestModeName(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeIsSensorOnly(mode));
}

void test_peg_finder_mode_parses_and_allows_distributed_motion() {
  robot::RobotTestMode mode{};

  TEST_ASSERT_TRUE(robot::parseRobotTestMode("PegFinder", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::PegFinder),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_EQUAL_STRING("PEG_FINDER", robot::robotTestModeName(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeIsSensorOnly(mode));

  TEST_ASSERT_TRUE(robot::parseRobotTestMode("peg-finder", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::PegFinder),
      static_cast<std::uint8_t>(mode));
}

void test_time_trial_mode_parses_and_allows_distributed_motion() {
  robot::RobotTestMode mode{};

  TEST_ASSERT_TRUE(robot::parseRobotTestMode("Time-Trial", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::TimeTrial),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_EQUAL_STRING("TIME_TRIAL", robot::robotTestModeName(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeIsSensorOnly(mode));
}

void test_habitat_pieces_mode_parses_and_allows_distributed_motion() {
  robot::RobotTestMode mode{};

  TEST_ASSERT_TRUE(robot::parseRobotTestMode("HabitatPieces", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::HabitatPieces),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_EQUAL_STRING("HABITAT_PIECES",
                           robot::robotTestModeName(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeIsSensorOnly(mode));

  TEST_ASSERT_TRUE(robot::parseRobotTestMode("habitat-pieces", mode));
}

void test_imu_turn_mode_is_explicit_and_requires_rear_link() {
  robot::RobotTestMode mode{};
  TEST_ASSERT_TRUE(robot::parseRobotTestMode("imu-turn", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::ImuTurnTest),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_EQUAL_STRING("IMU_TURN_TEST",
                           robot::robotTestModeName(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeIsSensorOnly(mode));

  robot::RobotTestModeManager manager{};
  manager.setMode(mode, 10U);
  TEST_ASSERT_TRUE(manager.acceptsImuTurnCommand());
  TEST_ASSERT_FALSE(manager.acceptsLineFollowerCommand());
  TEST_ASSERT_FALSE(manager.acceptsDriveCommand());
}

void test_imu_strafe_mode_is_explicit_and_requires_rear_link() {
  robot::RobotTestMode mode{};
  TEST_ASSERT_TRUE(robot::parseRobotTestMode("imu-strafe", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::ImuStrafeTest),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_EQUAL_STRING("IMU_STRAFE_TEST",
                           robot::robotTestModeName(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
  TEST_ASSERT_FALSE(robot::robotTestModeIsSensorOnly(mode));

  robot::RobotTestModeManager manager{};
  manager.setMode(mode, 10U);
  TEST_ASSERT_TRUE(manager.acceptsImuStrafeCommand());
  TEST_ASSERT_FALSE(manager.acceptsImuTurnCommand());
  TEST_ASSERT_FALSE(manager.acceptsLineFollowerCommand());
  TEST_ASSERT_FALSE(manager.acceptsDriveCommand());
}

void test_imu_heading_hold_config_requires_bounded_combined_duty() {
  robot::ImuHeadingHoldConfig config{};
  TEST_ASSERT_FALSE(robot::imuHeadingHoldConfigValid(config, 0.5F));

  config = imuHeadingHoldConfig();
  TEST_ASSERT_TRUE(robot::imuHeadingHoldConfigValid(config, 0.5F));
  config.maximum_yaw_correction_duty = 0.3F;
  TEST_ASSERT_FALSE(robot::imuHeadingHoldConfigValid(config, 0.5F));
  config = imuHeadingHoldConfig();
  config.yaw_command_polarity = 0;
  TEST_ASSERT_FALSE(robot::imuHeadingHoldConfigValid(config, 0.5F));
  config = imuHeadingHoldConfig();
  config.kp = std::numeric_limits<float>::quiet_NaN();
  TEST_ASSERT_FALSE(robot::imuHeadingHoldConfigValid(config, 0.5F));
}

void test_imu_heading_hold_captures_target_once() {
  robot::ImuHeadingHoldControllerState state{};
  const robot::ImuHeadingHoldConfig config = imuHeadingHoldConfig();

  TEST_ASSERT_TRUE(robot::startImuHeadingHold(
      state, 37.5F, -1, config, 0.5F, 100U));
  TEST_ASSERT_TRUE(robot::imuHeadingHoldActive(state));
  assertNear(37.5F, state.start_heading_deg, 0.0001F);
  assertNear(37.5F, state.target_heading_deg, 0.0001F);
  TEST_ASSERT_EQUAL_INT(-1, state.lateral_direction);

  robot::ImuHeadingHoldUpdate update = robot::updateImuHeadingHold(
      state, 35.5F, 0.0F, config, 0.5F, 110U);
  assertNear(37.5F, update.target_heading_deg, 0.0001F);
  assertNear(2.0F, update.angle_error_deg, 0.0001F);
  TEST_ASSERT_TRUE(update.should_strafe);

  update = robot::updateImuHeadingHold(
      state, 36.0F, 0.0F, config, 0.5F, 120U);
  assertNear(37.5F, update.target_heading_deg, 0.0001F);
}

void test_imu_heading_hold_pd_correction_clamps_and_damps_rate() {
  robot::ImuHeadingHoldControllerState state{};
  robot::ImuHeadingHoldConfig config = imuHeadingHoldConfig();
  config.kp = 0.1F;
  config.kd = 0.02F;
  TEST_ASSERT_TRUE(robot::startImuHeadingHold(
      state, 10.0F, 1, config, 0.5F, 100U));

  robot::ImuHeadingHoldUpdate update = robot::updateImuHeadingHold(
      state, 8.0F, 5.0F, config, 0.5F, 110U);
  assertNear(0.2F, update.proportional_term, 0.0001F);
  assertNear(-0.1F, update.damping_term, 0.0001F);
  assertNear(0.1F, update.yaw_correction_duty, 0.0001F);

  update = robot::updateImuHeadingHold(
      state, -20.0F, 0.0F, config, 0.5F, 120U);
  assertNear(config.maximum_yaw_correction_duty,
             update.yaw_correction_duty, 0.0001F);
}

void test_imu_heading_hold_faults_on_nonfinite_measurement_and_stops() {
  robot::ImuHeadingHoldControllerState state{};
  const robot::ImuHeadingHoldConfig config = imuHeadingHoldConfig();
  TEST_ASSERT_TRUE(robot::startImuHeadingHold(
      state, 10.0F, 1, config, 0.5F, 100U));

  const robot::ImuHeadingHoldUpdate update =
      robot::updateImuHeadingHold(
          state, std::numeric_limits<float>::quiet_NaN(), 0.0F,
          config, 0.5F, 110U);
  TEST_ASSERT_TRUE(update.faulted);
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(
          robot::ImuHeadingHoldFaultReason::InvalidMeasurement),
      static_cast<int>(update.fault_reason));
  TEST_ASSERT_FALSE(update.should_strafe);

  robot::stopImuHeadingHold(state);
  TEST_ASSERT_FALSE(robot::imuHeadingHoldActive(state));
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(robot::ImuHeadingHoldState::Stopped),
      static_cast<int>(state.state));
}

void test_imu_turn_config_starts_locked_until_every_value_is_configured() {
  robot::ImuTurnConfig config{};
  TEST_ASSERT_FALSE(robot::imuTurnConfigValid(config, 0.5F));

  config = imuTurnConfig();
  TEST_ASSERT_TRUE(robot::imuTurnConfigValid(config, 0.5F));
  config.maximum_rotation_duty = 0.6F;
  TEST_ASSERT_FALSE(robot::imuTurnConfigValid(config, 0.5F));
  config = imuTurnConfig();
  config.yaw_command_polarity = 0;
  TEST_ASSERT_FALSE(robot::imuTurnConfigValid(config, 0.5F));
  config = imuTurnConfig();
  config.timeout_ms = config.settling_time_ms;
  TEST_ASSERT_FALSE(robot::imuTurnConfigValid(config, 0.5F));
  config = imuTurnConfig();
  config.kp = std::numeric_limits<float>::quiet_NaN();
  TEST_ASSERT_FALSE(robot::imuTurnConfigValid(config, 0.5F));
}

void test_imu_turn_start_captures_a_continuous_relative_target() {
  robot::ImuTurnControllerState state{};
  const robot::ImuTurnConfig config = imuTurnConfig();

  TEST_ASSERT_TRUE(robot::startImuTurn(
      state, 170.0F, 90.0F, config, 0.5F, 25U));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuTurnState::Turning),
      static_cast<std::uint8_t>(state.state));
  assertNear(170.0F, state.start_heading_deg, 0.0001F);
  assertNear(260.0F, state.target_heading_deg, 0.0001F);
  assertNear(90.0F, state.relative_angle_deg, 0.0001F);
}

void test_imu_turn_ccw_target_stays_at_saved_start_minus_offset() {
  robot::ImuTurnControllerState state{};
  const robot::ImuTurnConfig config = imuTurnConfig();

  TEST_ASSERT_TRUE(robot::startImuTurn(
      state, 35.0F, -60.0F, config, 0.5F, 25U));
  assertNear(35.0F, state.start_heading_deg, 0.0001F);
  assertNear(-25.0F, state.target_heading_deg, 0.0001F);

  (void)robot::updateImuTurn(
      state, 10.0F, -5.0F, config, 0.5F, 35U);
  assertNear(35.0F, state.start_heading_deg, 0.0001F);
  assertNear(-25.0F, state.target_heading_deg, 0.0001F);
}

void test_clockwise_angle_uses_measured_imu_polarity() {
  assertNear(
      90.0F,
      robot::clockwiseTurnRelativeAngleDeg(90.0F, 1),
      0.0001F);
  assertNear(
      -90.0F,
      robot::clockwiseTurnRelativeAngleDeg(90.0F, -1),
      0.0001F);
}

void test_imu_turn_pd_output_clamps_and_rate_damping_opposes_motion() {
  robot::ImuTurnControllerState state{};
  const robot::ImuTurnConfig config = imuTurnConfig();
  TEST_ASSERT_TRUE(robot::startImuTurn(
      state, 0.0F, 90.0F, config, 0.5F, 100U));

  robot::ImuTurnUpdate update =
      robot::updateImuTurn(state, 0.0F, 10.0F, config, 0.5F, 110U);
  assertNear(0.9F, update.proportional_term, 0.0001F);
  assertNear(-0.2F, update.damping_term, 0.0001F);
  assertNear(0.3F, update.rotation_command, 0.0001F);
  TEST_ASSERT_TRUE(update.should_rotate);

  robot::ImuTurnControllerState damped_state{};
  TEST_ASSERT_TRUE(robot::startImuTurn(
      damped_state, 0.0F, 10.0F, config, 0.5F, 100U));
  update = robot::updateImuTurn(
      damped_state, 0.0F, 10.0F, config, 0.5F, 110U);
  assertNear(0.1F, update.proportional_term, 0.0001F);
  assertNear(-0.2F, update.damping_term, 0.0001F);
  assertNear(-0.1F, update.rotation_command, 0.0001F);
}

void test_imu_turn_requires_low_angle_and_rate_for_full_settling_time() {
  robot::ImuTurnControllerState state{};
  const robot::ImuTurnConfig config = imuTurnConfig();
  TEST_ASSERT_TRUE(robot::startImuTurn(
      state, 0.0F, 90.0F, config, 0.5F, 100U));

  robot::ImuTurnUpdate update =
      robot::updateImuTurn(state, 89.0F, 4.0F, config, 0.5F, 200U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuTurnState::Turning),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_rotate);

  update = robot::updateImuTurn(
      state, 89.0F, 2.0F, config, 0.5F, 210U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuTurnState::Settling),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_rotate);
  TEST_ASSERT_FALSE(update.completed);

  update = robot::updateImuTurn(
      state, 89.0F, 2.0F, config, 0.5F, 309U);
  TEST_ASSERT_FALSE(update.completed);
  update = robot::updateImuTurn(
      state, 89.0F, 2.0F, config, 0.5F, 310U);
  TEST_ASSERT_TRUE(update.completed);
  TEST_ASSERT_EQUAL_UINT32(210U, update.elapsed_ms);
  TEST_ASSERT_EQUAL_UINT32(100U, update.settling_elapsed_ms);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuTurnState::Complete),
      static_cast<std::uint8_t>(update.state));
}

void test_imu_turn_leaves_settling_if_either_condition_breaks() {
  robot::ImuTurnControllerState state{};
  const robot::ImuTurnConfig config = imuTurnConfig();
  TEST_ASSERT_TRUE(robot::startImuTurn(
      state, 0.0F, 90.0F, config, 0.5F, 100U));
  robot::updateImuTurn(state, 89.0F, 2.0F, config, 0.5F, 200U);

  const robot::ImuTurnUpdate update =
      robot::updateImuTurn(state, 85.0F, 2.0F, config, 0.5F, 220U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuTurnState::Turning),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_EQUAL_UINT32(0U, update.settling_elapsed_ms);
  TEST_ASSERT_TRUE(update.should_rotate);
}

void test_imu_turn_timeout_faults_and_stop_is_terminal() {
  robot::ImuTurnControllerState state{};
  const robot::ImuTurnConfig config = imuTurnConfig();
  TEST_ASSERT_TRUE(robot::startImuTurn(
      state, 0.0F, -90.0F, config, 0.5F, 100U));
  robot::ImuTurnUpdate update =
      robot::updateImuTurn(state, 0.0F, 0.0F, config, 0.5F, 2100U);
  TEST_ASSERT_TRUE(update.faulted);
  TEST_ASSERT_EQUAL_UINT32(2000U, update.elapsed_ms);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuTurnFaultReason::Timeout),
      static_cast<std::uint8_t>(update.fault_reason));
  TEST_ASSERT_FALSE(update.should_rotate);

  robot::stopImuTurn(state);
  update =
      robot::updateImuTurn(state, 0.0F, 0.0F, config, 0.5F, 2200U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuTurnState::Stopped),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_rotate);
  TEST_ASSERT_FALSE(robot::imuTurnActive(state));
}

void test_imu_turn_ignores_a_loop_timestamp_just_before_start() {
  robot::ImuTurnControllerState state{};
  const robot::ImuTurnConfig config = imuTurnConfig();
  TEST_ASSERT_TRUE(robot::startImuTurn(
      state, 0.29F, -90.0F, config, 0.5F, 1000U));

  robot::ImuTurnUpdate update =
      robot::updateImuTurn(state, 0.29F, 0.08F, config, 0.5F, 995U);
  TEST_ASSERT_FALSE(update.faulted);
  TEST_ASSERT_EQUAL_UINT32(0U, update.elapsed_ms);
  TEST_ASSERT_TRUE(update.should_rotate);
  TEST_ASSERT_TRUE(update.rotation_command < 0.0F);

  update =
      robot::updateImuTurn(state, 0.29F, 0.08F, config, 0.5F, 1005U);
  TEST_ASSERT_FALSE(update.faulted);
  TEST_ASSERT_EQUAL_UINT32(5U, update.elapsed_ms);

  robot::ImuTurnControllerState rollover_state{};
  const robot::Milliseconds near_rollover =
      std::numeric_limits<robot::Milliseconds>::max() - 5U;
  TEST_ASSERT_TRUE(robot::startImuTurn(
      rollover_state, 0.0F, 90.0F, config, 0.5F, near_rollover));
  update = robot::updateImuTurn(
      rollover_state, 0.0F, 0.0F, config, 0.5F, 3U);
  TEST_ASSERT_FALSE(update.faulted);
  TEST_ASSERT_EQUAL_UINT32(9U, update.elapsed_ms);
}

void test_imu_turn_faults_instead_of_emitting_nonfinite_output() {
  robot::ImuTurnConfig config = imuTurnConfig();
  config.kp = std::numeric_limits<float>::max();
  robot::ImuTurnControllerState state{};
  TEST_ASSERT_TRUE(robot::startImuTurn(
      state, 0.0F, 1.0e20F, config, 0.5F, 100U));

  const robot::ImuTurnUpdate update =
      robot::updateImuTurn(state, 0.0F, 0.0F, config, 0.5F, 110U);
  TEST_ASSERT_TRUE(update.faulted);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::ImuTurnFaultReason::InvalidConfiguration),
      static_cast<std::uint8_t>(update.fault_reason));
  TEST_ASSERT_FALSE(update.should_rotate);
  TEST_ASSERT_TRUE(std::isfinite(update.rotation_command));
}

void test_imu_recovery_pauses_saves_heading_and_requires_fresh_samples() {
  const robot::ImuRecoveryConfig config{1000U, 3U};
  robot::ImuRecoveryState state{};

  robot::ImuRecoveryUpdate update = robot::updateImuRecovery(
      state, config, true, 10U, 42.0F, 100U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuRecoveryDecision::Continue),
      static_cast<std::uint8_t>(update.decision));

  update = robot::updateImuRecovery(
      state, config, false, 10U, 42.5F, 110U);
  TEST_ASSERT_TRUE(state.active);
  TEST_ASSERT_TRUE(update.pause_started);
  TEST_ASSERT_EQUAL_UINT32(1U, state.pause_count);
  assertNear(42.5F, state.saved_heading_deg, 0.0001F);

  update = robot::updateImuRecovery(
      state, config, true, 11U, 42.5F, 120U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuRecoveryDecision::Paused),
      static_cast<std::uint8_t>(update.decision));
  TEST_ASSERT_EQUAL_UINT8(1U, state.consecutive_fresh_samples);
  TEST_ASSERT_EQUAL_UINT32(10U, update.timer_adjustment_ms);

  update = robot::updateImuRecovery(
      state, config, true, 11U, 42.5F, 130U);
  TEST_ASSERT_EQUAL_UINT8(1U, state.consecutive_fresh_samples);

  update = robot::updateImuRecovery(
      state, config, true, 12U, 42.5F, 140U);
  TEST_ASSERT_EQUAL_UINT8(2U, state.consecutive_fresh_samples);
  update = robot::updateImuRecovery(
      state, config, true, 13U, 42.5F, 150U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuRecoveryDecision::Recovered),
      static_cast<std::uint8_t>(update.decision));
  TEST_ASSERT_FALSE(state.active);
  TEST_ASSERT_EQUAL_UINT32(40U, state.last_pause_duration_ms);
  TEST_ASSERT_EQUAL_UINT32(40U, state.total_paused_ms);
}

void test_imu_recovery_resets_confirmation_and_times_out() {
  const robot::ImuRecoveryConfig config{100U, 2U};
  robot::ImuRecoveryState state{};
  (void)robot::updateImuRecovery(
      state, config, false, 20U, -12.0F, 1000U);
  (void)robot::updateImuRecovery(
      state, config, true, 21U, -12.0F, 1020U);
  TEST_ASSERT_EQUAL_UINT8(1U, state.consecutive_fresh_samples);
  (void)robot::updateImuRecovery(
      state, config, false, 21U, -12.0F, 1040U);
  TEST_ASSERT_EQUAL_UINT8(0U, state.consecutive_fresh_samples);

  const robot::ImuRecoveryUpdate update = robot::updateImuRecovery(
      state, config, false, 21U, -12.0F, 1100U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::ImuRecoveryDecision::TimedOut),
      static_cast<std::uint8_t>(update.decision));
  TEST_ASSERT_FALSE(state.active);
  TEST_ASSERT_EQUAL_UINT32(100U, state.last_pause_duration_ms);
}

void test_imu_recovery_defers_controller_timeouts() {
  const robot::ImuTurnConfig turn_config = imuTurnConfig();
  robot::ImuTurnControllerState turn{};
  TEST_ASSERT_TRUE(robot::startImuTurn(
      turn, 0.0F, 90.0F, turn_config, 0.5F, 100U));
  robot::deferImuTurnTimers(turn, 250U);
  const robot::ImuTurnUpdate turn_update = robot::updateImuTurn(
      turn, 0.0F, 0.0F, turn_config, 0.5F, 350U);
  TEST_ASSERT_EQUAL_UINT32(0U, turn_update.elapsed_ms);

  const robot::ImuHeadingHoldConfig hold_config{
      0.2F, 0.01F, 0.0F, 0.1F, 1};
  robot::ImuHeadingHoldControllerState hold{};
  TEST_ASSERT_TRUE(robot::startImuHeadingHold(
      hold, 5.0F, 1, hold_config, 0.5F, 100U));
  robot::deferImuHeadingHoldTimer(hold, 250U);
  const robot::ImuHeadingHoldUpdate hold_update =
      robot::updateImuHeadingHold(
          hold, 5.0F, 0.0F, hold_config, 0.5F, 350U);
  TEST_ASSERT_EQUAL_UINT32(0U, hold_update.elapsed_ms);
}

void test_time_trial_config_allows_skipped_or_safe_transition_strafe() {
  robot::TimeTrialConfig config{};
  TEST_ASSERT_TRUE(robot::timeTrialConfigValid(config, 0.5F));

  config.solar_to_tower_strafe_right_duration_ms = 250U;
  TEST_ASSERT_TRUE(robot::timeTrialConfigValid(config, 0.5F));

  TEST_ASSERT_FALSE(robot::timeTrialConfigValid(config, 0.0F));
}

void test_time_trial_runs_solar_strafe_tower_and_peg_finder_in_order() {
  const robot::TimeTrialConfig config{100U, 50U, 30U};
  robot::TimeTrialAutonomy autonomy{};

  robot::TimeTrialUpdate update =
      robot::startTimeTrialAutonomy(autonomy, 10U);
  TEST_ASSERT_TRUE(update.should_start_solar);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::AutonomousSolar),
      static_cast<std::uint8_t>(update.state));

  robot::TimeTrialInputs inputs{};
  inputs.solar_complete = true;
  update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 20U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::PostSolarDelay),
      static_cast<std::uint8_t>(update.state));

  inputs = {};
  update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 119U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::PostSolarDelay),
      static_cast<std::uint8_t>(update.state));

  update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 120U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TimeTrialState::SolarToTowerStrafeRight),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_strafe_right);

  update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 169U);
  TEST_ASSERT_TRUE(update.should_strafe_right);
  TEST_ASSERT_FALSE(update.should_start_tower_pieces);

  update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 170U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::TowerPieces),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_start_tower_pieces);

  inputs.tower_pieces_complete = true;
  update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 180U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::PostTowerDelay),
      static_cast<std::uint8_t>(update.state));

  inputs = {};
  update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 210U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::PegFinder),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_start_peg_finder);

  inputs.peg_finder_complete = true;
  update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 220U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::Complete),
      static_cast<std::uint8_t>(update.state));
}

void test_time_trial_hands_solar_line_follow_directly_to_tower() {
  const robot::TimeTrialConfig config{1000U, 500U, 30U};
  robot::TimeTrialAutonomy autonomy{};
  (void)robot::startTimeTrialAutonomy(autonomy, 10U);

  robot::TimeTrialInputs inputs{};
  inputs.solar_line_follow_ready = true;
  const robot::TimeTrialUpdate update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 20U);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::TowerPieces),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_start_tower_pieces);
  TEST_ASSERT_TRUE(update.should_handoff_solar_line_follow);
  TEST_ASSERT_FALSE(update.should_strafe_right);
}

void test_time_trial_propagates_included_mode_faults() {
  const robot::TimeTrialConfig config{};
  robot::TimeTrialAutonomy autonomy{};
  (void)robot::startTimeTrialAutonomy(autonomy, 0U);

  robot::TimeTrialInputs inputs{};
  inputs.solar_fault = true;
  const robot::TimeTrialUpdate update =
      robot::updateTimeTrialAutonomy(autonomy, inputs, config, 5U);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TimeTrialState::Fault),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_strafe_right);
}

void test_peg_finder_config_requires_safe_duties_angle_and_timings() {
  robot::PegFinderConfig config = pegFinderConfig();
  TEST_ASSERT_TRUE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));

  config.reverse_duty = 0.6F;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.forward_duty = 0.6F;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.funnel_forward_duty = 0.5F;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.clockwise_angle_deg = 0.0F;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.post_rotation_pause_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.reverse_duration_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.post_reverse_pause_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.forward_duration_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.funnel_forward_timeout_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.post_funnel_limit_delay_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.claw_open_interval_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.claw_open_order_2 = 1U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.claw_open_order_3 = 4U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.post_claws_open_delay_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.funnel_reverse_duty = 0.5F;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
  config = pegFinderConfig();
  config.funnel_reverse_duration_ms = 0U;
  TEST_ASSERT_FALSE(robot::pegFinderConfigValid(config, 0.5F, 0.4F));
}

void test_peg_finder_clockwise_turn_waits_for_imu_completion() {
  const robot::PegFinderConfig config = pegFinderConfig();
  robot::PegFinderAutonomy autonomy{};
  robot::startPegFinderAutonomy(autonomy, 100U);

  robot::PegFinderUpdate update =
      updatePegFinderForTest(autonomy, config, 10000U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::RotateClockwise),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_rotate_clockwise);

  update =
      updatePegFinderForTest(autonomy, config, 10001U, false, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::PostRotationPause),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_rotate_clockwise);
}

void test_peg_finder_opens_claws_in_configured_order_then_reverses_funnel() {
  robot::PegFinderConfig config = pegFinderConfig();
  config.claw_open_order_1 = 3U;
  config.claw_open_order_2 = 1U;
  config.claw_open_order_3 = 2U;
  robot::PegFinderAutonomy autonomy{};
  robot::startPegFinderAutonomy(autonomy, 100U);

  robot::PegFinderUpdate update =
      updatePegFinderForTest(autonomy, config, 109U);
  TEST_ASSERT_TRUE(update.should_rotate_clockwise);
  TEST_ASSERT_FALSE(update.should_drive_backward);
  TEST_ASSERT_FALSE(update.should_run_funnel_forward);

  update = updatePegFinderForTest(autonomy, config, 110U, false, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::PostRotationPause),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_rotate_clockwise);

  update = updatePegFinderForTest(autonomy, config, 120U);
  TEST_ASSERT_FALSE(update.should_drive_backward);
  update = updatePegFinderForTest(autonomy, config, 121U);
  TEST_ASSERT_TRUE(update.should_drive_backward);

  update = updatePegFinderForTest(autonomy, config, 132U);
  TEST_ASSERT_TRUE(update.should_drive_backward);
  update = updatePegFinderForTest(autonomy, config, 133U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::PostReversePause),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_drive_backward);

  update = updatePegFinderForTest(autonomy, config, 145U);
  TEST_ASSERT_FALSE(update.should_drive_forward);
  TEST_ASSERT_FALSE(update.should_run_funnel_forward);
  update = updatePegFinderForTest(autonomy, config, 146U);
  TEST_ASSERT_TRUE(update.should_drive_forward);
  TEST_ASSERT_FALSE(update.should_run_funnel_forward);

  update = updatePegFinderForTest(autonomy, config, 159U);
  TEST_ASSERT_TRUE(update.should_drive_forward);
  update = updatePegFinderForTest(autonomy, config, 160U);
  TEST_ASSERT_FALSE(update.should_drive_forward);
  TEST_ASSERT_TRUE(update.should_run_funnel_forward);

  update = updatePegFinderForTest(autonomy, config, 174U, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::PostFunnelLimitDelay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.funnel_limit_detected);
  TEST_ASSERT_FALSE(update.should_run_funnel_forward);

  update = updatePegFinderForTest(autonomy, config, 189U, true);
  TEST_ASSERT_FALSE(update.should_open_claw_1);
  update = updatePegFinderForTest(autonomy, config, 190U, true);
  TEST_ASSERT_TRUE(update.should_open_claw_3);
  TEST_ASSERT_FALSE(update.should_open_claw_2);

  update = updatePegFinderForTest(autonomy, config, 191U, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::PostClaw1OpenDelay),
      static_cast<std::uint8_t>(update.state));
  update = updatePegFinderForTest(autonomy, config, 207U, true);
  TEST_ASSERT_FALSE(update.should_open_claw_2);
  update = updatePegFinderForTest(autonomy, config, 208U, true);
  TEST_ASSERT_TRUE(update.should_open_claw_1);
  TEST_ASSERT_FALSE(update.should_open_claw_2);

  update = updatePegFinderForTest(autonomy, config, 209U, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::PostClaw2OpenDelay),
      static_cast<std::uint8_t>(update.state));
  update = updatePegFinderForTest(autonomy, config, 225U, true);
  TEST_ASSERT_FALSE(update.should_open_claw_3);
  update = updatePegFinderForTest(autonomy, config, 226U, true);
  TEST_ASSERT_TRUE(update.should_open_claw_2);

  update = updatePegFinderForTest(autonomy, config, 227U, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::PostClawsOpenDelay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_run_funnel_reverse);
  update = updatePegFinderForTest(autonomy, config, 244U, true);
  TEST_ASSERT_FALSE(update.should_run_funnel_reverse);
  update = updatePegFinderForTest(autonomy, config, 245U, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::PegFinderState::FunnelReverse),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_run_funnel_reverse);
  update = updatePegFinderForTest(autonomy, config, 264U, true);
  TEST_ASSERT_TRUE(update.should_run_funnel_reverse);
  update = updatePegFinderForTest(autonomy, config, 265U, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::PegFinderState::Complete),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_run_funnel_forward);
  TEST_ASSERT_FALSE(update.should_run_funnel_reverse);
}

void test_peg_finder_funnel_timeout_faults_without_limit() {
  const robot::PegFinderConfig config = pegFinderConfig();
  robot::PegFinderAutonomy autonomy{};
  autonomy.state = robot::PegFinderState::FunnelForward;
  autonomy.state_entered_at_ms = 100U;

  robot::PegFinderUpdate update =
      updatePegFinderForTest(autonomy, config, 114U);
  TEST_ASSERT_TRUE(update.should_run_funnel_forward);
  update = updatePegFinderForTest(autonomy, config, 115U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::PegFinderState::Fault),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderFaultReason::FunnelLimitTimeout),
      static_cast<std::uint8_t>(update.fault_reason));
  TEST_ASSERT_FALSE(update.should_run_funnel_forward);
}

void test_peg_finder_does_not_start_funnel_when_limit_already_pressed() {
  const robot::PegFinderConfig config = pegFinderConfig();
  robot::PegFinderAutonomy autonomy{};
  autonomy.state = robot::PegFinderState::Forward;
  autonomy.state_entered_at_ms = 100U;

  const robot::PegFinderUpdate update =
      updatePegFinderForTest(autonomy, config, 114U, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::PegFinderState::PostFunnelLimitDelay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_run_funnel_forward);
}

void test_tower_pieces_config_requires_duties_and_timings() {
  const robot::TowerPiecesConfig defaults{};
  TEST_ASSERT_EQUAL_UINT32(1000U, defaults.post_line_delay_ms);
  TEST_ASSERT_EQUAL_UINT32(1000U, defaults.post_strafe_pause_ms);
  TEST_ASSERT_EQUAL_UINT32(1000U, defaults.post_rotation_pause_ms);
  TEST_ASSERT_EQUAL_UINT32(1000U,
                           defaults.post_final_reverse_delay_ms);
  TEST_ASSERT_EQUAL_UINT32(1000U, defaults.post_winch_open_delay_ms);
  TEST_ASSERT_EQUAL_UINT32(1000U, defaults.post_claws_open_delay_ms);
  TEST_ASSERT_EQUAL_UINT32(1000U,
                           defaults.post_stepper_bottom_delay_ms);
  TEST_ASSERT_EQUAL_UINT32(1000U,
                           defaults.post_claws_closed_delay_ms);
  TEST_ASSERT_EQUAL_UINT32(
      2000U, defaults.stepper_down_speed_steps_per_second);
  TEST_ASSERT_EQUAL_UINT32(2000U,
                           defaults.stepper_up_speed_steps_per_second);

  robot::TowerPiecesConfig config = towerPiecesConfig();
  TEST_ASSERT_TRUE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));

  config.reverse_line_duty = 0.0F;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config.reverse_line_duty = 0.6F;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config.reverse_line_duty = 0.2F;
  config.side_line_timeout_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.post_line_delay_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.strafe_right_duration_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.post_strafe_pause_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.clockwise_rotation_angle_deg = 0.0F;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.post_rotation_pause_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.reverse_duty = 0.0F;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config.reverse_duty = 0.6F;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.reverse_duration_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.shimmy_right_duration_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.shimmy_left_duration_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.shimmy_timeout_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));

  config = towerPiecesConfig();
  config.final_reverse_duty = 0.0F;
  config.final_reverse_duration_ms = 0U;
  TEST_ASSERT_TRUE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config.final_reverse_duration_ms = 1U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.final_reverse_duty = 0.6F;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));

  config = towerPiecesConfig();
  config.post_final_reverse_delay_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.post_winch_open_delay_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.post_claws_open_delay_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.post_stepper_bottom_delay_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.post_claws_closed_delay_ms = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));

  config = towerPiecesConfig();
  config.stepper_down_speed_steps_per_second = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.stepper_down_speed_steps_per_second = 200001U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.stepper_up_speed_steps_per_second = 0U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
  config = towerPiecesConfig();
  config.stepper_up_speed_steps_per_second = 200001U;
  TEST_ASSERT_FALSE(robot::towerPiecesConfigValid(config, 0.5F, 200000U));
}

void test_tower_pieces_counts_distinct_side_line_rising_edges() {
  robot::TowerPiecesAutonomy autonomy{};
  const robot::TowerPiecesConfig config = towerPiecesConfig();
  robot::startTowerPiecesAutonomy(autonomy, false, 100U);

  robot::TowerPiecesUpdate update =
      updateTowerPiecesForTest(autonomy, true, false, false,
                                      config, 200U);
  TEST_ASSERT_TRUE(update.side_line_rising_edge);
  TEST_ASSERT_EQUAL_UINT8(1U, update.side_line_count);
  TEST_ASSERT_TRUE(update.should_line_follow);

  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 300U);
  TEST_ASSERT_FALSE(update.side_line_rising_edge);
  TEST_ASSERT_EQUAL_UINT8(1U, update.side_line_count);
  TEST_ASSERT_TRUE(update.should_line_follow);

  update = updateTowerPiecesForTest(autonomy, false, false, false,
                                            config, 400U);
  TEST_ASSERT_EQUAL_UINT8(1U, update.side_line_count);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 500U);
  TEST_ASSERT_TRUE(update.side_line_rising_edge);
  TEST_ASSERT_EQUAL_UINT8(2U, update.side_line_count);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::PostLineDelay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_line_follow);
  TEST_ASSERT_FALSE(update.should_initial_strafe_right);
  TEST_ASSERT_FALSE(update.should_rotate_clockwise);
}

void test_tower_pieces_does_not_count_a_high_level_present_at_start() {
  robot::TowerPiecesAutonomy autonomy{};
  const robot::TowerPiecesConfig config = towerPiecesConfig();
  robot::startTowerPiecesAutonomy(autonomy, true, 100U);

  robot::TowerPiecesUpdate update =
      updateTowerPiecesForTest(autonomy, true, false, false,
                                      config, 200U);
  TEST_ASSERT_FALSE(update.side_line_rising_edge);
  TEST_ASSERT_EQUAL_UINT8(0U, update.side_line_count);

  updateTowerPiecesForTest(autonomy, false, false, false, config,
                                   300U);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 400U);
  TEST_ASSERT_TRUE(update.side_line_rising_edge);
  TEST_ASSERT_EQUAL_UINT8(1U, update.side_line_count);
}

void test_tower_pieces_timeout_stops_before_second_side_line() {
  robot::TowerPiecesAutonomy autonomy{};
  robot::TowerPiecesConfig config = towerPiecesConfig();
  config.side_line_timeout_ms = 1000U;
  robot::startTowerPiecesAutonomy(autonomy, false, 100U);
  updateTowerPiecesForTest(autonomy, true, false, false, config,
                                   200U);
  updateTowerPiecesForTest(autonomy, false, false, false, config,
                                   300U);

  robot::TowerPiecesUpdate update =
      updateTowerPiecesForTest(autonomy, false, false, false,
                                      config, 1099U);
  TEST_ASSERT_TRUE(update.should_line_follow);
  update = updateTowerPiecesForTest(autonomy, false, false, false,
                                            config, 1100U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::Fault),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesFaultReason::SideLineTimeout),
      static_cast<std::uint8_t>(update.fault_reason));
  TEST_ASSERT_EQUAL_UINT8(1U, update.side_line_count);
  TEST_ASSERT_FALSE(update.should_line_follow);
}

void test_tower_side_crossing_requires_cooldown_and_offline_rearm() {
  robot::TowerPiecesConfig config = towerPiecesConfig();
  config.side_line_cooldown_ms = 100U;
  config.side_line_rearm_ms = 50U;
  robot::TowerPiecesAutonomy autonomy{};
  robot::startTowerPiecesAutonomy(autonomy, false, 0U);

  robot::TowerPiecesUpdate update =
      updateTowerPiecesForTest(autonomy, true, false, false, config, 100U);
  TEST_ASSERT_TRUE(update.side_line_detection_accepted);
  TEST_ASSERT_EQUAL_UINT8(1U, update.side_line_count);
  update =
      updateTowerPiecesForTest(autonomy, false, false, false, config, 110U);
  TEST_ASSERT_FALSE(update.side_line_armed);
  update =
      updateTowerPiecesForTest(autonomy, true, false, false, config, 140U);
  TEST_ASSERT_TRUE(update.side_line_detection_rejected);
  TEST_ASSERT_EQUAL_UINT8(1U, update.side_line_count);
  update =
      updateTowerPiecesForTest(autonomy, false, false, false, config, 150U);
  update =
      updateTowerPiecesForTest(autonomy, false, false, false, config, 260U);
  TEST_ASSERT_TRUE(update.side_line_armed);
  update =
      updateTowerPiecesForTest(autonomy, true, false, false, config, 270U);
  TEST_ASSERT_TRUE(update.side_line_detection_accepted);
  TEST_ASSERT_EQUAL_UINT8(2U, update.side_line_count);
}

void test_tower_pre_stepper_delay_holds_slide_stopped() {
  robot::TowerPiecesConfig config = towerPiecesConfig();
  config.pre_stepper_bottom_delay_ms = 75U;
  robot::TowerPiecesAutonomy autonomy{};
  autonomy.state = robot::TowerPiecesState::PostClawsOpenDelay;
  autonomy.state_entered_at_ms = 100U;

  robot::TowerPiecesUpdate update = updateTowerPiecesForTest(
      autonomy, false, false, false, config, 112U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PreStepperBottomDelay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_move_stepper_bottom);
  update = updateTowerPiecesForTest(
      autonomy, false, false, false, config, 186U);
  TEST_ASSERT_FALSE(update.should_move_stepper_bottom);
  update = updateTowerPiecesForTest(
      autonomy, false, false, false, config, 187U);
  TEST_ASSERT_TRUE(update.should_move_stepper_bottom);
}

void test_tower_pieces_runs_full_sequence_in_order() {
  robot::TowerPiecesAutonomy autonomy{};
  const robot::TowerPiecesConfig config = towerPiecesConfig();
  robot::startTowerPiecesAutonomy(autonomy, false, 100U);
  updateTowerPiecesForTest(autonomy, true, false, false, config,
                                   200U);
  updateTowerPiecesForTest(autonomy, false, false, false, config,
                                   300U);
  robot::TowerPiecesUpdate update = updateTowerPiecesForTest(
      autonomy, true, false, false, config, 500U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::PostLineDelay),
      static_cast<std::uint8_t>(update.state));

  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 599U);
  TEST_ASSERT_FALSE(update.should_initial_strafe_right);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 600U);
  TEST_ASSERT_TRUE(update.should_initial_strafe_right);

  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 799U);
  TEST_ASSERT_TRUE(update.should_initial_strafe_right);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 800U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::PostStrafePause),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_initial_strafe_right);

  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 949U);
  TEST_ASSERT_FALSE(update.should_rotate_clockwise);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 950U);
  TEST_ASSERT_TRUE(update.should_rotate_clockwise);

  update = updateTowerPiecesForTest(autonomy, true, true, true,
                                            config, 1199U);
  TEST_ASSERT_TRUE(update.should_rotate_clockwise);
  TEST_ASSERT_FALSE(update.back_line_detected);
  update = updateTowerPiecesForTest(autonomy, true, true, true,
                                    config, 1200U, false, false, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::PostRotationPause),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_rotate_clockwise);

  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 1299U);
  TEST_ASSERT_FALSE(update.should_drive_backward);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 1300U);
  TEST_ASSERT_TRUE(update.should_drive_backward);

  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 1599U);
  TEST_ASSERT_TRUE(update.should_drive_backward);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 1600U);
  TEST_ASSERT_TRUE(update.should_shimmy_right);

  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 1679U);
  TEST_ASSERT_TRUE(update.should_shimmy_right);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 1680U);
  TEST_ASSERT_TRUE(update.should_shimmy_left);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 1799U);
  TEST_ASSERT_TRUE(update.should_shimmy_left);
  update = updateTowerPiecesForTest(autonomy, true, false, false,
                                            config, 1800U);
  TEST_ASSERT_TRUE(update.should_shimmy_right);

  update = updateTowerPiecesForTest(autonomy, true, true, true,
                                            config, 1801U);
  TEST_ASSERT_TRUE(update.back_line_detected);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::FinalReverse),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_drive_final_reverse);
  TEST_ASSERT_FALSE(update.should_line_follow);
  autonomy.state_entered_at_ms = 1801U;

  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                            config, 1890U);
  TEST_ASSERT_TRUE(update.should_drive_final_reverse);
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                            config, 1891U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostFinalReverseDelay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_drive_final_reverse);

  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1900U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostFinalReverseDelay),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1901U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::WinchOpen),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1902U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostWinchOpenDelay),
      static_cast<std::uint8_t>(update.state));

  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1912U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostWinchOpenDelay),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1913U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::ClawsOpen),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1914U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostClawsOpenDelay),
      static_cast<std::uint8_t>(update.state));

  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1925U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostClawsOpenDelay),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1926U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PreStepperBottomDelay),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1927U);
  TEST_ASSERT_TRUE(update.should_move_stepper_bottom);

  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1928U, false, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::MoveStepperBottom),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_move_stepper_bottom);
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1929U, true, false);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostStepperBottomDelay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_move_stepper_bottom);
  autonomy.state_entered_at_ms = 1928U;

  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1940U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostStepperBottomDelay),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1941U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::ClawsClosed),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1942U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostClawsClosedDelay),
      static_cast<std::uint8_t>(update.state));

  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1955U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostClawsClosedDelay),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1956U);
  TEST_ASSERT_TRUE(update.should_move_stepper_top);
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1957U, true, false);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::MoveStepperTop),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_move_stepper_top);
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1958U, false, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::WinchClosed),
      static_cast<std::uint8_t>(update.state));
  update = updateTowerPiecesForTest(autonomy, true, false, true,
                                    config, 1959U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::Complete),
      static_cast<std::uint8_t>(update.state));
}

void test_tower_pieces_rotation_waits_for_imu_completion() {
  const robot::TowerPiecesConfig config = towerPiecesConfig();
  robot::TowerPiecesAutonomy autonomy{};
  autonomy.state = robot::TowerPiecesState::RotateClockwise;
  autonomy.state_entered_at_ms = 100U;

  robot::TowerPiecesUpdate update = updateTowerPiecesForTest(
      autonomy, false, true, true, config, 10000U);
  TEST_ASSERT_TRUE(update.should_rotate_clockwise);
  TEST_ASSERT_FALSE(update.back_line_detected);
  update = updateTowerPiecesForTest(
      autonomy, false, true, true, config, 10001U, false, false, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::PostRotationPause),
      static_cast<std::uint8_t>(update.state));
}

void test_tower_pieces_shimmy_hands_off_to_tail_and_can_skip_reverse() {
  robot::TowerPiecesConfig config = towerPiecesConfig();
  config.final_reverse_duty = 0.0F;
  config.final_reverse_duration_ms = 0U;
  robot::TowerPiecesAutonomy autonomy{};
  autonomy.state = robot::TowerPiecesState::ShimmyLeft;
  autonomy.state_entered_at_ms = 100U;
  autonomy.shimmy_started_at_ms = 100U;

  robot::TowerPiecesUpdate update = updateTowerPiecesForTest(
      autonomy, false, true, false, config, 200U);
  TEST_ASSERT_TRUE(update.back_line_detected);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesState::PostFinalReverseDelay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_FALSE(update.should_line_follow);
  TEST_ASSERT_FALSE(update.should_drive_final_reverse);
}

void test_tower_pieces_conflicting_stepper_limits_fault() {
  robot::TowerPiecesAutonomy autonomy{};
  autonomy.state = robot::TowerPiecesState::MoveStepperBottom;
  autonomy.state_entered_at_ms = 100U;

  const robot::TowerPiecesUpdate update = updateTowerPiecesForTest(
      autonomy, false, false, false, towerPiecesConfig(), 101U, true, true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::Fault),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesFaultReason::ConflictingLimitSwitches),
      static_cast<std::uint8_t>(update.fault_reason));
  TEST_ASSERT_EQUAL_STRING(
      "CONFLICTING_LIMIT_SWITCHES",
      robot::towerPiecesFaultReasonName(update.fault_reason));
}

void test_tower_pieces_shimmy_timeout_spans_direction_changes() {
  robot::TowerPiecesConfig config = towerPiecesConfig();
  config.shimmy_timeout_ms = 500U;
  robot::TowerPiecesAutonomy autonomy{};
  autonomy.state = robot::TowerPiecesState::ShimmyRight;
  autonomy.state_entered_at_ms = 100U;
  autonomy.shimmy_started_at_ms = 100U;
  robot::TowerPiecesUpdate update = updateTowerPiecesForTest(
      autonomy, false, false, false, config, 179U);
  TEST_ASSERT_TRUE(update.should_shimmy_right);
  update = updateTowerPiecesForTest(autonomy, false, false, false,
                                            config, 180U);
  TEST_ASSERT_TRUE(update.should_shimmy_left);
  update = updateTowerPiecesForTest(autonomy, false, false, false,
                                            config, 299U);
  TEST_ASSERT_TRUE(update.should_shimmy_left);
  update = updateTowerPiecesForTest(autonomy, false, false, false,
                                            config, 300U);
  TEST_ASSERT_TRUE(update.should_shimmy_right);
  update = updateTowerPiecesForTest(autonomy, false, false, false,
                                            config, 599U);
  TEST_ASSERT_TRUE(update.should_shimmy_left || update.should_shimmy_right);
  update = updateTowerPiecesForTest(autonomy, false, false, false,
                                            config, 600U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::TowerPiecesState::Fault),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::TowerPiecesFaultReason::ShimmyTimeout),
      static_cast<std::uint8_t>(update.fault_reason));
  TEST_ASSERT_FALSE(update.should_shimmy_left);
  TEST_ASSERT_FALSE(update.should_shimmy_right);
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

void test_motion_diagnostics_retains_newest_samples_in_time_order() {
  robot::MotionDiagnostics diagnostics{};
  diagnostics.reset(10U);

  for (std::size_t index = 0U;
       index < robot::kMotionDiagnosticSampleCapacity + 3U; ++index) {
    robot::MotionDiagnosticSample sample{};
    sample.timestamp_ms = static_cast<robot::Milliseconds>(index);
    diagnostics.record(sample);
  }

  TEST_ASSERT_EQUAL_UINT(robot::kMotionDiagnosticSampleCapacity,
                         diagnostics.sampleCount());
  TEST_ASSERT_EQUAL_UINT32(
      3U, diagnostics.sampleFromOldest(0U).timestamp_ms);
  TEST_ASSERT_EQUAL_UINT32(
      robot::kMotionDiagnosticSampleCapacity + 2U,
      diagnostics
          .sampleFromOldest(robot::kMotionDiagnosticSampleCapacity - 1U)
          .timestamp_ms);
}

void test_motion_diagnostics_freeze_preserves_trigger_and_samples() {
  robot::MotionDiagnostics diagnostics{};
  diagnostics.reset(100U);
  robot::MotionDiagnosticSample sample{};
  sample.timestamp_ms = 110U;
  diagnostics.record(sample);
  diagnostics.freeze(robot::MotionDiagnosticTrigger::ImuTurnTimedOut, 120U);

  sample.timestamp_ms = 130U;
  diagnostics.record(sample);

  TEST_ASSERT_TRUE(diagnostics.frozen());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::MotionDiagnosticTrigger::ImuTurnTimedOut),
      static_cast<std::uint8_t>(diagnostics.trigger()));
  TEST_ASSERT_EQUAL_UINT32(120U, diagnostics.frozenAtMs());
  TEST_ASSERT_EQUAL_UINT(1U, diagnostics.sampleCount());
  TEST_ASSERT_EQUAL_UINT32(
      110U, diagnostics.sampleFromOldest(0U).timestamp_ms);
}

void test_motion_diagnostics_tracks_loop_and_web_failure_evidence() {
  robot::MotionDiagnostics diagnostics{};
  diagnostics.reset(200U);
  diagnostics.observeLoop(10000U, 3000U, 500U, 1000U, 10U);
  diagnostics.observeLoop(35000U, 32000U, 6000U, 28000U, 10U);
  diagnostics.noteWebDrive(210U, true, false);
  diagnostics.noteWebStop(220U);
  diagnostics.noteWebDrive(230U, false, true);

  TEST_ASSERT_EQUAL_UINT32(35000U, diagnostics.maximumLoopIntervalUs());
  TEST_ASSERT_EQUAL_UINT32(32000U, diagnostics.maximumLoopWorkUs());
  TEST_ASSERT_EQUAL_UINT32(6000U, diagnostics.maximumImuUpdateUs());
  TEST_ASSERT_EQUAL_UINT32(28000U, diagnostics.maximumWebHandleUs());
  TEST_ASSERT_EQUAL_UINT32(2U, diagnostics.missedDeadlineCount());
  TEST_ASSERT_EQUAL_UINT32(2U, diagnostics.webDriveRequestCount());
  TEST_ASSERT_EQUAL_UINT32(1U, diagnostics.webStopRequestCount());
  TEST_ASSERT_EQUAL_UINT32(1U, diagnostics.driveAfterStopCount());
}

void test_motion_diagnostics_json_exposes_commands_pwm_and_timing() {
  robot::MotionDiagnostics diagnostics{};
  diagnostics.reset(300U);
  robot::MotionDiagnosticSample sample{};
  sample.timestamp_ms = 310U;
  sample.event =
      robot::MotionDiagnosticEvent::WebDriveHeartbeatAfterStop;
  sample.mode = robot::RobotTestMode::DistributedDriveTest;
  sample.loop_interval_us = UINT32_MAX;
  sample.loop_work_us = UINT32_MAX;
  sample.imu_update_us = UINT32_MAX;
  sample.web_handle_us = UINT32_MAX;
  sample.requested_command_milli[0] = 250;
  sample.requested_command_milli[1] = -1000;
  sample.requested_command_milli[2] = 1000;
  sample.requested_command_milli[3] = -1000;
  sample.applied_command_milli[0] = 250;
  sample.applied_command_milli[1] = -1000;
  sample.applied_command_milli[2] = 1000;
  sample.applied_command_milli[3] = -1000;
  sample.front_driver_desired_command_milli[0] = -1000;
  sample.front_driver_desired_command_milli[1] = 1000;
  sample.front_command_expires_at_ms[0] = UINT32_MAX;
  sample.front_command_expires_at_ms[1] = UINT32_MAX;
  sample.front_pwm_readback[0] = 255U;
  sample.front_pwm_readback[1] = UINT32_MAX;
  sample.front_pwm_readback[2] = UINT32_MAX;
  sample.front_pwm_readback[3] = UINT32_MAX;
  sample.rear_command_age_ms = UINT32_MAX;
  sample.esp1_status_age_ms = UINT32_MAX;
  sample.esp1_packet_error_count = UINT32_MAX;
  sample.command_deadman_armed = true;
  sample.command_deadline_ms = UINT32_MAX;
  sample.last_command_age_ms = UINT32_MAX;
  sample.heading_deg = 1234567.0F;
  sample.target_heading_deg = -1234567.0F;
  sample.angle_error_deg = -2469134.0F;
  sample.yaw_rate_dps = -500.0F;
  sample.rotation_command = -1.0F;
  for (std::size_t index = 0U;
       index < robot::kMotionDiagnosticSampleCapacity; ++index) {
    sample.timestamp_ms =
        static_cast<robot::Milliseconds>(310U + index);
    diagnostics.record(sample);
  }
  diagnostics.freeze(
      robot::MotionDiagnosticTrigger::DriveHeartbeatAfterStop, 320U);

  char json[16384]{};
  TEST_ASSERT_TRUE(
      robot::writeMotionDiagnosticsJson(diagnostics, json, sizeof(json)));
  TEST_ASSERT_NOT_NULL(
      std::strstr(json, "\"trigger\":\"DRIVE_HEARTBEAT_AFTER_STOP\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(json, "\"event\":\"WEB_DRIVE_HEARTBEAT_AFTER_STOP\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(json, "\"requested\":[250,-1000,1000,-1000]"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(
          json,
          "\"pwm_readback\":[255,4294967295,4294967295,4294967295]"));
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
  config.retry_forward_duty = -0.01F;
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config.retry_forward_duty = 1.01F;
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config.retry_forward_duty =
      std::numeric_limits<float>::quiet_NaN();
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config = solarContactConfig();
  config.post_contact_forward_duty = -0.01F;
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config.post_contact_forward_duty = 1.01F;
  TEST_ASSERT_FALSE(robot::solarPanelContactConfigValid(config));

  config.post_contact_forward_duty =
      std::numeric_limits<float>::quiet_NaN();
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
  TEST_ASSERT_EQUAL_STRING(
      "MOVE_FORWARD_AFTER_SOLAR_CONTACT",
      robot::solarPanelAutonomyStateName(
          robot::SolarPanelAutonomyState::MoveForwardAfterSolarContact));
  TEST_ASSERT_EQUAL_STRING(
      "STRAFE_LEFT_TO_REAR_LINE",
      robot::solarPanelAutonomyStateName(
          robot::SolarPanelAutonomyState::StrafeLeftToRearLine));
  TEST_ASSERT_EQUAL_STRING(
      "REAR_LINE_REACQUIRED",
      robot::solarPanelAutonomyStateName(
          robot::SolarPanelAutonomyState::RearLineReacquired));
  TEST_ASSERT_EQUAL_STRING(
      "BACKWARD_LINE_FOLLOW_AFTER_REAR_DETECTION",
      robot::solarPanelAutonomyStateName(
          robot::SolarPanelAutonomyState::
              BackwardLineFollowAfterRearDetection));
  TEST_ASSERT_EQUAL_STRING(
      "WAIT_BEFORE_STRAFE_LEFT_TO_REAR_LINE",
      robot::solarPanelAutonomyStateName(
          robot::SolarPanelAutonomyState::
              WaitBeforeStrafeLeftToRearLine));
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

void test_solar_post_contact_forward_stops_if_rear_line_is_already_detected() {
  const robot::SolarPanelContactConfig config = solarContactConfig();
  const robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::MoveForwardAfterSolarContact, true,
          true, true, config.post_contact_forward_duration_ms, config);

  assertSolarState(
      robot::SolarPanelAutonomyState::BackwardLineFollowAfterRearDetection,
      update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);
}

void test_solar_post_contact_delays_are_independently_adjustable() {
  robot::SolarPanelContactConfig config = solarContactConfig();
  config.post_contact_forward_start_delay_ms = 25U;
  config.line_reacquire_strafe_start_delay_ms = 35U;

  robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::SolarPanelContacted, true, true,
          false, 24U, config);
  assertSolarState(robot::SolarPanelAutonomyState::SolarPanelContacted,
                   update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::SolarPanelContacted, true, true, false,
      25U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::MoveForwardAfterSolarContact,
      update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      robot::SolarPanelAutonomyState::MoveForwardAfterSolarContact, true,
      true, false, config.post_contact_forward_duration_ms, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::WaitBeforeStrafeLeftToRearLine,
      update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, true, false, 34U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::WaitBeforeStrafeLeftToRearLine,
      update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, true, false, 35U, config);
  assertSolarState(robot::SolarPanelAutonomyState::StrafeLeftToRearLine,
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

void test_solar_contact_drives_forward_then_strafes_until_rear_line() {
  const robot::SolarPanelContactConfig config = solarContactConfig();

  robot::SolarPanelContactSequenceUpdate update =
      robot::updateSolarPanelContactSequence(
          robot::SolarPanelAutonomyState::SolarPanelContacted, true, true,
          false, 0U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::MoveForwardAfterSolarContact,
      update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, true, true,
      config.post_contact_forward_duration_ms - 1U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::MoveForwardAfterSolarContact,
      update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, true, false,
      config.post_contact_forward_duration_ms, config);
  assertSolarState(robot::SolarPanelAutonomyState::StrafeLeftToRearLine,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, true, false, 100000U, config);
  assertSolarState(robot::SolarPanelAutonomyState::StrafeLeftToRearLine,
                   update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, true, true, 100001U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::BackwardLineFollowAfterRearDetection,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, true, false,
      config.rear_line_follow_duration_ms - 1U, config);
  assertSolarState(
      robot::SolarPanelAutonomyState::BackwardLineFollowAfterRearDetection,
      update.next_state);
  TEST_ASSERT_FALSE(update.transitioned);

  update = robot::updateSolarPanelContactSequence(
      update.next_state, true, true, false,
      config.rear_line_follow_duration_ms, config);
  assertSolarState(robot::SolarPanelAutonomyState::RearLineReacquired,
                   update.next_state);
  TEST_ASSERT_TRUE(update.transitioned);
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
  snapshot.imu.configured = true;
  snapshot.imu.initialized = true;
  snapshot.imu.calibrated = true;
  snapshot.imu.healthy = true;
  snapshot.imu.data_fresh = true;
  snapshot.imu.acquisition_running = true;
  snapshot.imu.device_acknowledged = true;
  snapshot.imu.runtime_configuration_valid = true;
  snapshot.imu.register_reads_use_repeated_start = true;
  snapshot.imu.i2c_address = 0x68U;
  snapshot.imu.who_am_i = 0x74U;
  snapshot.imu.sda_gpio = 39;
  snapshot.imu.scl_gpio = 38;
  snapshot.imu.last_wire_status = 0;
  std::strcpy(snapshot.imu.initialization_error, "NONE");
  std::strcpy(snapshot.imu.last_read_failure_reason, "I2C_TIMEOUT");
  std::strcpy(snapshot.imu.disconnect_reason, "STALE_DATA");
  std::strcpy(snapshot.imu.last_disconnect_reason, "I2C_TIMEOUT");
  snapshot.imu.raw_gyro_z = -321;
  snapshot.imu.gyro_z_bias_dps = 1.25F;
  snapshot.imu.yaw_rate_dps = -4.5F;
  snapshot.imu.heading_deg = 92.75F;
  snapshot.imu.sample_age_ms = 3U;
  snapshot.imu.snapshot_age_ms = 2U;
  snapshot.imu.acquisition_duration_us = 1725U;
  snapshot.imu.maximum_completed_acquisition_duration_us = 12045U;
  snapshot.imu.total_acquisition_attempts = 702U;
  snapshot.imu.last_successful_read_us = 1234567U;
  snapshot.imu.last_sample_interval_us = 10025U;
  snapshot.imu.last_read_failure_us = 1200000U;
  snapshot.imu.successful_read_count = 700U;
  snapshot.imu.failed_read_count = 2U;
  snapshot.imu.consecutive_failed_reads = 1U;
  snapshot.imu.disconnect_count = 3U;
  snapshot.imu.last_disconnect_at_ms = 1200U;
  snapshot.imu.acquisition_loop_interval_us = 10010U;
  snapshot.imu.maximum_acquisition_loop_interval_us = 79990U;
  snapshot.imu.maximum_wire_lock_acquire_duration_us = 12U;
  snapshot.imu.maximum_measurement_read_duration_us = 3400U;
  snapshot.imu.maximum_successful_read_to_publication_us = 45U;
  snapshot.imu.successful_sample_publication_gap_us = 10050U;
  snapshot.imu.maximum_successful_sample_publication_gap_us = 218000U;
  snapshot.imu.current_observed_publication_gap_us = 217000U;
  snapshot.imu.maximum_observed_publication_gap_us = 217000U;
  snapshot.imu.publication_sequence = 800U;
  snapshot.imu.successful_sample_sequence = 790U;
  snapshot.imu_turn.configuration_valid = true;
  snapshot.imu_turn.active = true;
  snapshot.imu_turn.state = robot::ImuTurnState::Turning;
  snapshot.imu_turn.fault_reason = robot::ImuTurnFaultReason::None;
  snapshot.imu_turn.availability_fault_capture_valid = true;
  snapshot.imu_turn.availability_fault_latched = true;
  snapshot.imu_turn.imu_currently_available = true;
  snapshot.imu_turn.captured_configured = true;
  snapshot.imu_turn.captured_initialized = true;
  snapshot.imu_turn.captured_calibrated = true;
  snapshot.imu_turn.captured_healthy = true;
  snapshot.imu_turn.captured_sample_valid = true;
  snapshot.imu_turn.captured_data_fresh = false;
  snapshot.imu_turn.captured_acquisition_running = true;
  snapshot.imu_turn.captured_shared_snapshot_available = true;
  snapshot.imu_turn.captured_newest_snapshot_available = true;
  snapshot.imu_turn.captured_cached_snapshot_matches_newest = false;
  snapshot.imu_turn.captured_rear_link_configured = true;
  snapshot.imu_turn.captured_rear_status_available = true;
  snapshot.imu_turn.captured_rear_status_fresh = true;
  std::strcpy(snapshot.imu_turn.availability_fault_origin,
              "ACTIVE_TURN_RUNTIME_GATE");
  std::strcpy(snapshot.imu_turn.captured_availability_reason,
              "STALE_DATA");
  snapshot.imu_turn.availability_evaluated_at_us = 1300000U;
  snapshot.imu_turn.availability_evaluated_at_ms = 1300U;
  snapshot.imu_turn.captured_sample_age_us = 80000U;
  snapshot.imu_turn.captured_freshness_timeout_us = 75000U;
  snapshot.imu_turn.captured_cached_snapshot_sequence = 800U;
  snapshot.imu_turn.captured_newest_snapshot_sequence = 801U;
  snapshot.imu_turn.captured_cached_snapshot_fetch_to_gate_us = 5000U;
  snapshot.imu_turn.captured_current_observed_publication_gap_us = 217000U;
  snapshot.imu_turn.maximum_rotation_duty = 0.3F;
  snapshot.imu_turn.kp = 0.01F;
  snapshot.imu_turn.kd = 0.02F;
  snapshot.imu_turn.angle_tolerance_deg = 2.0F;
  snapshot.imu_turn.maximum_finishing_yaw_rate_dps = 3.0F;
  snapshot.imu_turn.settling_time_ms = 100U;
  snapshot.imu_turn.timeout_ms = 2000U;
  snapshot.imu_turn.yaw_command_polarity = -1;
  snapshot.imu_turn.start_heading_deg = 10.0F;
  snapshot.imu_turn.current_heading_deg = 42.0F;
  snapshot.imu_turn.target_heading_deg = 100.0F;
  snapshot.imu_turn.relative_angle_deg = 90.0F;
  snapshot.imu_turn.angle_error_deg = 58.0F;
  snapshot.imu_turn.yaw_rate_dps = 12.0F;
  snapshot.imu_turn.proportional_term = 0.58F;
  snapshot.imu_turn.damping_term = -0.24F;
  snapshot.imu_turn.rotation_command = 0.3F;
  snapshot.imu_turn.elapsed_ms = 250U;
  snapshot.imu_turn.settling_elapsed_ms = 0U;
  snapshot.imu_heading_hold.configuration_valid = true;
  snapshot.imu_heading_hold.active = true;
  snapshot.imu_heading_hold.state =
      robot::ImuHeadingHoldState::Active;
  snapshot.imu_heading_hold.fault_reason =
      robot::ImuHeadingHoldFaultReason::None;
  snapshot.imu_heading_hold.maximum_strafe_duty = 0.25F;
  snapshot.imu_heading_hold.kp = 0.01F;
  snapshot.imu_heading_hold.kd = 0.02F;
  snapshot.imu_heading_hold.maximum_yaw_correction_duty = 0.1F;
  snapshot.imu_heading_hold.yaw_command_polarity = 1;
  snapshot.imu_heading_hold.start_heading_deg = 20.0F;
  snapshot.imu_heading_hold.current_heading_deg = 22.0F;
  snapshot.imu_heading_hold.target_heading_deg = 20.0F;
  snapshot.imu_heading_hold.angle_error_deg = -2.0F;
  snapshot.imu_heading_hold.yaw_rate_dps = 3.0F;
  snapshot.imu_heading_hold.proportional_term = -0.02F;
  snapshot.imu_heading_hold.damping_term = -0.06F;
  snapshot.imu_heading_hold.yaw_correction_duty = -0.08F;
  snapshot.imu_heading_hold.lateral_direction = -1;
  snapshot.imu_heading_hold.elapsed_ms = 125U;
  snapshot.imu_recovery.turn_paused = true;
  snapshot.imu_recovery.turn_saved_heading_deg = 41.5F;
  snapshot.imu_recovery.turn_pause_elapsed_ms = 70U;
  snapshot.imu_recovery.maximum_pause_ms = 30000U;
  snapshot.imu_recovery.consecutive_fresh_samples_required = 3U;
  snapshot.imu_recovery.turn_consecutive_fresh_samples = 2U;
  snapshot.imu_recovery.turn_pause_count = 4U;
  snapshot.imu_recovery.strafe_pause_count = 5U;
  snapshot.imu_recovery.total_paused_ms = 600U;
  snapshot.lss_raw_level = 1;
  snapshot.lss2_raw_level = 0;
  snapshot.lss3_raw_level = 1;
  snapshot.lsfl_black = true;
  snapshot.lsfr_black = false;
  snapshot.lss_black = true;
  snapshot.lss_configured = true;
  snapshot.lss2_black = false;
  snapshot.lss2_configured = true;
  snapshot.lss3_black = true;
  snapshot.lss3_configured = true;
  snapshot.lsbl_raw_level = 1;
  snapshot.lsbr_raw_level = 0;
  snapshot.lsbl_black = true;
  snapshot.lsbr_black = false;
  snapshot.rear_line_configured = true;
  snapshot.rear_line_data_fresh = true;
  snapshot.rear_line_sequence = 77U;
  snapshot.rear_line_sample_age_ms = 4U;
  snapshot.rear_line_captured_at_ms = 456U;
  snapshot.rear_line_error = 1;
  snapshot.rear_line_visible = true;
  snapshot.rear_line_has_history = true;
  snapshot.rear_last_known_line_side = 1;
  snapshot.rear_line_follower_enabled = true;
  snapshot.rear_logical_left_black = false;
  snapshot.rear_logical_right_black = true;
  snapshot.rear_kp = 0.31F;
  snapshot.rear_ki = 0.04F;
  snapshot.rear_kd = 0.07F;
  snapshot.rear_base_duty = 0.2F;
  snapshot.rear_effective_base_duty = -0.2F;
  snapshot.rear_maximum_duty = 0.6F;
  snapshot.rear_maximum_correction = 0.3F;
  snapshot.rear_integral_limit = 1.5F;
  snapshot.rear_derivative_limit = 10.0F;
  snapshot.rear_derivative_filter_alpha = 0.25F;
  snapshot.rear_steering_polarity = -1;
  snapshot.rear_control_period_ms = 12U;
  snapshot.rear_remote_command_timeout_ms = 100U;
  snapshot.rear_line_telemetry_enabled = true;
  snapshot.rear_pid_correction = -0.125F;
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
  snapshot.esp1.ultrasonic_1_configured = true;
  snapshot.esp1.ultrasonic_1_echo_valid = true;
  snapshot.esp1.ultrasonic_1_distance_mm = 170U;
  snapshot.esp1.ultrasonic_1_echo_duration_us = 1000U;
  snapshot.ultrasonic_1.configured = true;
  snapshot.ultrasonic_1.data_fresh = true;
  snapshot.ultrasonic_1.echo_valid = true;
  snapshot.ultrasonic_1.distance_mm = 170U;
  snapshot.ultrasonic_1.echo_duration_us = 1000U;
  snapshot.ultrasonic_1.sample_age_ms = 12U;
  snapshot.ultrasonic_1_distance_mm = 170;
  snapshot.laser_distance.available = true;
  snapshot.laser_distance.configured = true;
  snapshot.laser_distance.initialized = true;
  snapshot.laser_distance.ranging = true;
  snapshot.laser_distance.data_fresh = true;
  snapshot.laser_distance.data_valid = true;
  snapshot.laser_distance.profile =
      robot::LaserDistanceProfile::HighAccuracy;
  snapshot.laser_distance.distance_mm = 287U;
  snapshot.laser_distance.measurement_sequence = 91U;
  snapshot.laser_distance.packet_sequence = 99U;
  snapshot.laser_distance.sensor_range_status = 0U;
  snapshot.laser_distance.driver_status = 0;
  snapshot.laser_distance.sda_gpio = 10;
  snapshot.laser_distance.scl_gpio = 9;
  snapshot.laser_distance.i2c_address = 0x29U;
  snapshot.laser_distance.sample_age_ms = 15U;
  snapshot.laser_distance.snapshot_age_ms = 5U;
  snapshot.laser_distance.intermeasurement_period_ms = 200U;
  snapshot.laser_distance.successful_measurement_count = 4000U;
  snapshot.laser_distance.failed_measurement_count = 7U;
  snapshot.laser_distance.consecutive_failed_measurements = 2U;
  snapshot.laser_distance.acquisition_duration_us = 830U;
  snapshot.laser_distance.maximum_acquisition_duration_us = 5100U;
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
  snapshot.solar_retry_forward_duty = 0.17F;
  snapshot.solar_retry_strafe_timeout_ms = 987U;
  snapshot.solar_post_contact_forward_duration_ms = 1000U;
  snapshot.solar_line_reacquire_strafe_duty = 0.18F;
  snapshot.solar_rear_line_follow_duration_ms = 750U;
  snapshot.solar_post_contact_forward_start_delay_ms = 125U;
  snapshot.solar_line_reacquire_strafe_start_delay_ms = 250U;
  snapshot.solar_post_contact_forward_duty = 0.19F;
  snapshot.solar_panel_limit_switches_configured = true;
  snapshot.solar_limit_back_right_high = true;
  snapshot.solar_limit_front_right_high = false;
  snapshot.solar_limit_back_right_hit = true;
  snapshot.solar_limit_front_right_hit = false;
  snapshot.solar_limit_all_hit = false;
  snapshot.habitat_pieces_state =
      robot::HabitatPiecesState::LineFollowing;
  snapshot.habitat_pieces_stop_reason =
      robot::HabitatPiecesStopReason::None;
  snapshot.habitat_pieces_time_in_state_ms = 345U;
  snapshot.habitat_pieces_line_follow_duty = 0.12F;
  snapshot.habitat_pieces_lss2_detection_delay_ms = 500U;
  snapshot.habitat_pieces_lss2_detection_remaining_ms = 155U;
  snapshot.habitat_pieces_run_timeout_ms = 5000U;
  snapshot.habitat_pieces_run_elapsed_ms = 345U;
  snapshot.habitat_pieces_timeout_remaining_ms = 4655U;
  snapshot.habitat_pieces_reverse_duty = 0.18F;
  snapshot.habitat_pieces_reverse_duration_ms = 250U;
  snapshot.habitat_pieces_reverse_elapsed_ms = 75U;
  snapshot.habitat_pieces_reverse_remaining_ms = 175U;
  snapshot.habitat_pieces_configuration_valid = true;
  snapshot.habitat_pieces_start_ready = true;
  snapshot.habitat_pieces_lss2_configured = true;
  snapshot.habitat_pieces_lss2_data_fresh = true;
  snapshot.habitat_pieces_lss2_detection_armed = false;
  snapshot.habitat_pieces_lss2_black = true;
  snapshot.habitat_pieces_should_stop = false;
  snapshot.habitat_pieces_target_reached = false;
  snapshot.habitat_pieces_line_following = true;
  snapshot.habitat_pieces_reversing = false;
  snapshot.habitat_pieces_timed_out = false;
  snapshot.habitat_placement_counter_clockwise_heading_captured = true;
  snapshot.habitat_placement_counter_clockwise_start_heading_deg = 12.5F;
  snapshot.habitat_placement_counter_clockwise_target_heading_deg = -77.5F;
  snapshot.habitat_placement_config.post_clockwise_reverse_duty = 0.21F;
  snapshot.habitat_placement_config
      .post_clockwise_reverse_duration_ms = 410U;
  snapshot.habitat_placement_config.post_clockwise_strafe_left_duty =
      0.22F;
  snapshot.habitat_placement_config
      .post_clockwise_strafe_left_duration_ms = 420U;
  snapshot.tower_pieces_state = robot::TowerPiecesState::ReverseLineFollow;
  snapshot.tower_pieces_fault_reason =
      robot::TowerPiecesFaultReason::None;
  snapshot.tower_pieces_time_in_state_ms = 1234U;
  snapshot.tower_pieces_reverse_line_duty = 0.22F;
  snapshot.tower_pieces_side_line_timeout_ms = 9000U;
  snapshot.tower_pieces_post_line_delay_ms = 250U;
  snapshot.tower_pieces_strafe_right_duty = 0.24F;
  snapshot.tower_pieces_strafe_right_duration_ms = 750U;
  snapshot.tower_pieces_post_strafe_pause_ms = 300U;
  snapshot.tower_pieces_clockwise_rotation_duty = 0.18F;
  snapshot.tower_pieces_clockwise_rotation_angle_deg = 80.0F;
  snapshot.tower_pieces_post_rotation_pause_ms = 200U;
  snapshot.tower_pieces_reverse_duty = 0.16F;
  snapshot.tower_pieces_reverse_duration_ms = 600U;
  snapshot.tower_pieces_shimmy_duty = 0.14F;
  snapshot.tower_pieces_shimmy_right_duration_ms = 250U;
  snapshot.tower_pieces_shimmy_left_duration_ms = 350U;
  snapshot.tower_pieces_shimmy_timeout_ms = 4000U;
  snapshot.tower_pieces_final_reverse_duty = 0.19F;
  snapshot.tower_pieces_final_reverse_duration_ms = 350U;
  snapshot.tower_pieces_post_final_reverse_delay_ms = 600U;
  snapshot.tower_pieces_post_winch_open_delay_ms = 610U;
  snapshot.tower_pieces_post_claws_open_delay_ms = 620U;
  snapshot.tower_pieces_stepper_down_speed_steps_per_second = 2000U;
  snapshot.tower_pieces_post_stepper_bottom_delay_ms = 630U;
  snapshot.tower_pieces_post_claws_closed_delay_ms = 640U;
  snapshot.tower_pieces_stepper_up_speed_steps_per_second = 1900U;
  snapshot.tower_pieces_side_line_count = 1U;
  snapshot.tower_pieces_target_side_line_count = 2U;
  snapshot.tower_pieces_side_line_sensor_configured = true;
  snapshot.tower_pieces_side_line_sensor_high = false;
  snapshot.tower_pieces_line_following = true;
  snapshot.tower_pieces_strafing_right = false;
  snapshot.tower_pieces_rotating_clockwise = false;
  snapshot.tower_pieces_driving_backward = false;
  snapshot.tower_pieces_shimmying_left = true;
  snapshot.tower_pieces_shimmying_right = false;
  snapshot.tower_pieces_back_line_detected = true;
  snapshot.tower_pieces_final_reverse_active = true;
  snapshot.tower_pieces_stepper_moving_down = true;
  snapshot.tower_pieces_stepper_moving_up = true;
  snapshot.peg_finder_state = robot::PegFinderState::FunnelForward;
  snapshot.peg_finder_fault_reason = robot::PegFinderFaultReason::None;
  snapshot.peg_finder_time_in_state_ms = 42U;
  snapshot.peg_finder_clockwise_duty = 0.3F;
  snapshot.peg_finder_clockwise_angle_deg = 90.0F;
  snapshot.peg_finder_post_rotation_pause_ms = 200U;
  snapshot.peg_finder_reverse_duty = 0.25F;
  snapshot.peg_finder_reverse_duration_ms = 700U;
  snapshot.peg_finder_post_reverse_pause_ms = 300U;
  snapshot.peg_finder_forward_duty = 0.23F;
  snapshot.peg_finder_forward_duration_ms = 450U;
  snapshot.peg_finder_funnel_forward_duty = 0.2F;
  snapshot.peg_finder_funnel_forward_timeout_ms = 500U;
  snapshot.peg_finder_post_funnel_limit_delay_ms = 600U;
  snapshot.peg_finder_claw_open_interval_ms = 700U;
  snapshot.peg_finder_claw_open_order_1 = 3U;
  snapshot.peg_finder_claw_open_order_2 = 1U;
  snapshot.peg_finder_claw_open_order_3 = 2U;
  snapshot.peg_finder_post_claws_open_delay_ms = 800U;
  snapshot.peg_finder_funnel_reverse_duty = 0.18F;
  snapshot.peg_finder_funnel_reverse_duration_ms = 900U;
  snapshot.peg_finder_funnel_limit_configured = true;
  snapshot.peg_finder_funnel_limit_high = true;
  snapshot.peg_finder_rotating_clockwise = true;
  snapshot.peg_finder_driving_backward = true;
  snapshot.peg_finder_driving_forward = true;
  snapshot.peg_finder_funnel_forward = true;
  snapshot.peg_finder_funnel_reverse = true;
  snapshot.peg_finder_opening_claw_1 = true;
  snapshot.peg_finder_opening_claw_2 = true;
  snapshot.peg_finder_opening_claw_3 = true;
  snapshot.time_trial_state =
      robot::TimeTrialState::SolarToTowerStrafeRight;
  snapshot.time_trial_time_in_state_ms = 75U;
  snapshot.time_trial_post_solar_delay_ms = 500U;
  snapshot.time_trial_strafe_right_duty = 0.15F;
  snapshot.time_trial_strafe_right_duration_ms = 250U;
  snapshot.time_trial_post_tower_delay_ms = 700U;
  snapshot.time_trial_strafing_right = true;
  snapshot.esp1.solar_panel_limit_switches_configured = true;
  snapshot.esp1.solar_limit_back_right_high = true;
  snapshot.esp1.solar_limit_front_right_high = false;
  snapshot.esp1.side_line_sensor_configured = true;
  snapshot.esp1.side_line_sensor_high = true;
  snapshot.claws.claw_1.hardware_configured = true;
  snapshot.claws.claw_1.open_configured = true;
  snapshot.claws.claw_1.closed_configured = true;
  snapshot.claws.claw_1.open_angle_deg = 120;
  snapshot.claws.claw_1.closed_angle_deg = 47;
  snapshot.claws.claw_1.commanded_angle_deg = 47;
  snapshot.claws.claw_1.commanded_open = false;
  snapshot.claws.winch.hardware_configured = true;
  snapshot.claws.winch.gpio = 6;
  snapshot.claws.winch.mcpwm_unit = 0;
  snapshot.claws.winch.mcpwm_timer = 0;
  snapshot.claws.winch.mcpwm_generator = 0;
  snapshot.claws.winch.pwm_frequency_hz = 50U;
  snapshot.claws.winch.mcpwm_timer_resolution_hz = 1000000U;
  snapshot.claws.winch.open_configured = true;
  snapshot.claws.winch.closed_configured = true;
  snapshot.claws.winch.open_angle_deg = 145;
  snapshot.claws.winch.closed_angle_deg = 35;
  snapshot.claws.winch.commanded_angle_deg = 145;
  snapshot.claws.winch.commanded_open = true;
  snapshot.servo_winch_position = 145;
  snapshot.esp1.solar_hook_configured = true;
  snapshot.esp1.solar_hook_output_enabled = true;
  snapshot.esp1.solar_hook_commanded_angle_deg = 137;
  snapshot.solar_hook.hardware_configured = true;
  snapshot.solar_hook.open_configured = true;
  snapshot.solar_hook.closed_configured = true;
  snapshot.solar_hook.output_enabled = true;
  snapshot.solar_hook.open_angle_deg = 137;
  snapshot.solar_hook.closed_angle_deg = 42;
  snapshot.solar_hook.commanded_angle_deg = 137;
  snapshot.solar_hook.commanded_open = true;

  char output[20480]{};
  TEST_ASSERT_TRUE(
      robot::writeTelemetryJson(snapshot, output, sizeof(output), false));

  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"current_mode\":\"LINE_SENSOR_TEST\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"enabled\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"habitat_placement\""));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"counter_clockwise_heading_captured\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"counter_clockwise_start_heading_deg\":12.50000"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"counter_clockwise_target_heading_deg\":-77.50000"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"post_clockwise_reverse_duration_ms\":410"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"post_clockwise_strafe_left_duration_ms\":420"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"habitat_pusher\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"pwmBackend\":\"MCPWM\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"mcpwmUnit\":0"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"mcpwmTimerResolutionHz\":1000000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"imu\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"configured\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"device_acknowledged\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"runtime_configuration_valid\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"acquisition_running\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"snapshot_age_ms\":2"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"acquisition_duration_us\":1725"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"maximum_completed_acquisition_duration_us\":12045"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"total_acquisition_attempts\":702"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"last_successful_read_us\":1234567"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"register_reads_use_repeated_start\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"who_am_i\":116"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"sda_gpio\":39"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"scl_gpio\":38"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"initialization_error\":\"NONE\""));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"last_read_failure_reason\":\"I2C_TIMEOUT\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"disconnect_reason\":\"STALE_DATA\""));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"last_disconnect_reason\":\"I2C_TIMEOUT\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"raw_gyro_z\":-321"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"gyro_z_bias_dps\":1.25000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"yaw_rate_dps\":-4.50000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"heading_deg\":92.75000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"last_sample_interval_us\":10025"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"last_read_failure_us\":1200000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"successful_read_count\":700"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"consecutive_failed_reads\":1"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"disconnect_count\":3"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"last_disconnect_at_ms\":1200"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"laser_distance\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"measurement_sequence\":91"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"intermeasurement_period_ms\":200"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"successful_measurement_count\":4000"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"maximum_acquisition_duration_us\":5100"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"acquisition_timing\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"maximum_loop_interval_us\":79990"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"maximum_wire_lock_acquire_duration_us\":12"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"maximum_measurement_read_duration_us\":3400"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"maximum_successful_sample_publication_gap_us\":218000"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"current_observed_publication_gap_us\":217000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"imu_turn\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"configuration_valid\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"state\":\"TURNING\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"fault_reason\":\"NONE\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"availability_fault\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"capture_valid\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"latched\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"origin\":\"ACTIVE_TURN_RUNTIME_GATE\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"reason\":\"STALE_DATA\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"sample_age_us\":80000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"freshness_timeout_us\":75000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"cached_snapshot_sequence\":800"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"newest_snapshot_sequence\":801"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"cached_snapshot_matches_newest\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"maximum_rotation_duty\":0.30000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"angle_tolerance_deg\":2.00000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"yaw_command_polarity\":-1"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"angle_error_deg\":58.00000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"damping_term\":-0.24000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"imu_heading_hold\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"maximum_strafe_duty\":0.25000"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"maximum_yaw_correction_duty\":0.10000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"yaw_correction_duty\":-0.08000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"lateral_direction\":-1"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"imu_recovery\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"turn_paused\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"consecutive_fresh_samples_required\":3"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"total_paused_ms\":600"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lsfl_black\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lsfl_level\":\"UNKNOWN\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_raw_level\":1"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_level\":\"HIGH\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss2_raw_level\":0"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss3_raw_level\":1"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss2_level\":\"LOW\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"rear_line\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lsbl_raw_level\":1"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lsbr_level\":\"LOW\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"data_fresh\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"sequence\":77"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"sample_age_ms\":4"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"line_follower_enabled\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"logical_left_source\":\"LSBR\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"logical_right_source\":\"LSBL\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"rear_pid\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"effectiveBaseDuty\":-0.20000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"kp\":0.31000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_black\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_configured\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss2_black\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss2_configured\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss3_configured\":true"));
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
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"ultrasonic_1\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"data_fresh\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"echo_valid\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"distance_mm\":170"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"echo_duration_us\":1000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"sample_age_ms\":12"));
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
      std::strstr(output, "\"retry_forward_duty\":0.17000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"retry_strafe_timeout_ms\":987"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"post_contact_forward_duration_ms\":1000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"line_reacquire_strafe_duty\":0.18000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"backward_pid_duration_ms\":750"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"post_contact_forward_start_delay_ms\":125"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"line_reacquire_strafe_start_delay_ms\":250"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_contact_forward_duty\":0.19000"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"solarLimitSwitches\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"solar_strafe_speeds\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"backRightHigh\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"frontRightHigh\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"backRightHit\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"frontRightHit\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"allHit\":false"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"tower_pieces\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"tower_line_control\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"rejected_detection_count\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"state\":\"REVERSE_LINE_FOLLOW\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"reverse_line_duty\":0.22000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"side_line_timeout_ms\":9000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_line_delay_ms\":250"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"strafe_right_duty\":0.24000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"strafe_right_duration_ms\":750"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_strafe_pause_ms\":300"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"clockwise_rotation_duty\":0.18000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"clockwise_rotation_angle_deg\":80.00000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_rotation_pause_ms\":200"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"reverse_duty\":0.16000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"reverse_duration_ms\":600"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"shimmy_duty\":0.14000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"shimmy_right_duration_ms\":250"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"shimmy_left_duration_ms\":350"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"shimmy_timeout_ms\":4000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"final_reverse_duty\":0.19000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"final_reverse_duration_ms\":350"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_final_reverse_delay_ms\":600"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_winch_open_delay_ms\":610"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_claws_open_delay_ms\":620"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"stepper_down_speed_steps_per_second\":2000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_stepper_bottom_delay_ms\":630"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_claws_closed_delay_ms\":640"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"stepper_up_speed_steps_per_second\":1900"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"side_line_count\":1"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"target_side_line_count\":2"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"line_following\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"strafing_right\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"rotating_clockwise\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"driving_backward\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"shimmying_left\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"shimmying_right\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"back_line_detected\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"final_reverse_active\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"stepper_moving_down\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"stepper_moving_up\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"peg_finder\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"state\":\"FUNNEL_FORWARD\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"clockwise_angle_deg\":90.00000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"forward_duty\":0.23000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"forward_duration_ms\":450"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_forward_duty\":0.20000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_forward_timeout_ms\":500"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_forward_duration_ms\":500"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_funnel_limit_delay_ms\":600"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"claw_open_interval_ms\":700"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"claw_open_order\":[3,1,2]"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_claws_open_delay_ms\":800"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_reverse_duty\":0.18000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_reverse_duration_ms\":900"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_limit_configured\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_limit_high\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_reverse\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"rotating_clockwise\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"driving_backward\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"driving_forward\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"funnel_forward\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"opening_claw_1\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"opening_claw_2\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"opening_claw_3\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"time_trial\""));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"state\":\"SOLAR_TO_TOWER_STRAFE_RIGHT\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_solar_delay_ms\":500"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"strafe_right_duty\":0.15000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"strafe_right_duration_ms\":250"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"post_tower_delay_ms\":700"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output,
                  "\"solar_panel_limit_switches_configured\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"confirmation_progress_ms\":300"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"claws\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"claw_1\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"openAngleDeg\":120"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"closedAngleDeg\":47"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"commandedAngleDeg\":47"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"winch\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"openAngleDeg\":145"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"closedAngleDeg\":35"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"commandedAngleDeg\":145"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"solar_hook\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"solar_hook_configured\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"solar_hook_output_enabled\":true"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"openAngleDeg\":137"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"closedAngleDeg\":42"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"commandedOpen\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"servo_winch_position\":145"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"habitat_pieces\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"profile\":\"HIGH_ACCURACY\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"state\":\"LINE_FOLLOWING\""));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"line_follow_duty\":0.12000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"lss2_detection_delay_ms\":500"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"lss2_detection_remaining_ms\":155"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"run_timeout_ms\":5000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"reverse_duty\":0.18000"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"reverse_duration_ms\":250"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"reverse_elapsed_ms\":75"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"reverse_remaining_ms\":175"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"lss2_detection_armed\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"lss2_black\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"start_ready\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"lss2_configured\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"lss2_data_fresh\":true"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"timed_out\":false"));

  snapshot.lss_raw_level = 0;
  snapshot.lss_black = false;
  snapshot.esp1.side_line_sensor_high = false;
  TEST_ASSERT_TRUE(
      robot::writeTelemetryJson(snapshot, output, sizeof(output), false));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_raw_level\":0"));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_level\":\"LOW\""));
  TEST_ASSERT_NOT_NULL(std::strstr(output, "\"lss_black\":false"));

  snapshot.imu.initialized = false;
  snapshot.imu.calibrated = false;
  snapshot.imu.healthy = false;
  snapshot.imu.data_fresh = false;
  snapshot.imu.acquisition_running = false;
  snapshot.imu.device_acknowledged = false;
  snapshot.imu.who_am_i = 0U;
  snapshot.imu.last_wire_status = 2;
  std::strcpy(snapshot.imu.initialization_error, "NO_DEVICE_ACK");
  TEST_ASSERT_TRUE(
      robot::writeTelemetryJson(snapshot, output, sizeof(output), false));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"device_acknowledged\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"acquisition_running\":false"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(output, "\"last_wire_status\":2"));
  TEST_ASSERT_NOT_NULL(std::strstr(
      output, "\"initialization_error\":\"NO_DEVICE_ACK\""));
}

void test_esp1_status_packet_round_trips() {
  robot::Esp1StatusReport report{
      1234U, robot::RobotTestMode::AutonomousTowerPieces, true,
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
  report.ultrasonic_1_configured = true;
  report.ultrasonic_1_echo_valid = true;
  report.ultrasonic_1_distance_mm = 170U;
  report.ultrasonic_1_echo_duration_us = 1000U;
  report.solar_hook_configured = true;
  report.solar_hook_output_enabled = true;
  report.solar_hook_commanded_angle_deg = 137;

  const robot::UartPacket packet = robot::makeEsp1StatusPacket(report, 42U);
  robot::Esp1StatusReport decoded{};

  TEST_ASSERT_EQUAL_UINT16(47U, packet.header.payload_size);
  TEST_ASSERT_TRUE(robot::decodeEsp1StatusPacket(packet, decoded));
  TEST_ASSERT_EQUAL_UINT32(report.uptime_ms, decoded.uptime_ms);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::RobotTestMode::AutonomousTowerPieces),
      static_cast<std::uint8_t>(decoded.mode));
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
  TEST_ASSERT_TRUE(decoded.ultrasonic_1_configured);
  TEST_ASSERT_TRUE(decoded.ultrasonic_1_echo_valid);
  TEST_ASSERT_EQUAL_UINT16(170U, decoded.ultrasonic_1_distance_mm);
  TEST_ASSERT_EQUAL_UINT32(1000U, decoded.ultrasonic_1_echo_duration_us);
  TEST_ASSERT_TRUE(decoded.solar_hook_configured);
  TEST_ASSERT_TRUE(decoded.solar_hook_output_enabled);
  TEST_ASSERT_EQUAL_INT16(137, decoded.solar_hook_commanded_angle_deg);

  report.mode = robot::RobotTestMode::HabitatPieces;
  report.side_line_sensor_high = false;
  const robot::UartPacket low_packet =
      robot::makeEsp1StatusPacket(report, 43U);
  decoded = {};
  TEST_ASSERT_TRUE(robot::decodeEsp1StatusPacket(low_packet, decoded));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::HabitatPieces),
      static_cast<std::uint8_t>(decoded.mode));
  TEST_ASSERT_TRUE(decoded.side_line_sensor_configured);
  TEST_ASSERT_FALSE(decoded.side_line_sensor_high);

  robot::UartPacket legacy_packet = low_packet;
  legacy_packet.header.payload_size =
      robot::kLegacyEsp1StatusPayloadSize;
  legacy_packet.header.integrity_crc16 =
      robot::calculatePacketIntegrity(legacy_packet);
  decoded = {};
  TEST_ASSERT_TRUE(
      robot::decodeEsp1StatusPacket(legacy_packet, decoded));
  TEST_ASSERT_FALSE(decoded.solar_hook_configured);
  TEST_ASSERT_FALSE(decoded.solar_hook_output_enabled);
  TEST_ASSERT_EQUAL_INT16(-1, decoded.solar_hook_commanded_angle_deg);
}

void test_hc_sr04_echo_conversion_and_valid_range() {
  TEST_ASSERT_EQUAL_UINT32(10U, robot::kHcSr04TriggerPulseUs);
  TEST_ASSERT_EQUAL_UINT32(23530U, robot::kHcSr04EchoTimeoutUs);
  TEST_ASSERT_EQUAL_UINT16(
      170U, robot::hcSr04DistanceMmFromEchoUs(1000U));
  TEST_ASSERT_EQUAL_UINT16(
      20U, robot::hcSr04DistanceMmFromEchoUs(118U));
  TEST_ASSERT_EQUAL_UINT16(
      4000U,
      robot::hcSr04DistanceMmFromEchoUs(robot::kHcSr04EchoTimeoutUs));
  TEST_ASSERT_FALSE(robot::hcSr04DistanceMmIsValid(0U));
  TEST_ASSERT_FALSE(robot::hcSr04DistanceMmIsValid(19U));
  TEST_ASSERT_TRUE(robot::hcSr04DistanceMmIsValid(20U));
  TEST_ASSERT_TRUE(robot::hcSr04DistanceMmIsValid(4000U));
  TEST_ASSERT_FALSE(robot::hcSr04DistanceMmIsValid(4001U));
}

void test_rear_line_sensor_packet_round_trips_and_rejects_corruption() {
  robot::RearLineSensorSnapshot expected{};
  expected.captured_at_ms = 12345U;
  expected.configured = true;
  expected.left_electrical_high = true;
  expected.right_electrical_high = false;
  expected.side_configured = true;
  expected.side_electrical_high = true;
  expected.side_2_configured = true;
  expected.side_2_electrical_high = true;
  expected.side_3_configured = true;
  expected.side_3_electrical_high = false;
  robot::UartPacket packet =
      robot::makeRearLineSensorPacket(expected, 88U);
  robot::RearLineSensorSnapshot decoded{};

  TEST_ASSERT_TRUE(robot::decodeRearLineSensorPacket(packet, decoded));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::UartMessageType::SensorSnapshot),
      static_cast<std::uint8_t>(packet.header.message_type));
  TEST_ASSERT_EQUAL_UINT16(88U, packet.header.sequence);
  TEST_ASSERT_EQUAL_UINT16(robot::kRearLineSensorPayloadSize,
                           packet.header.payload_size);
  TEST_ASSERT_EQUAL_UINT32(expected.captured_at_ms, decoded.captured_at_ms);
  TEST_ASSERT_TRUE(decoded.configured);
  TEST_ASSERT_TRUE(decoded.left_electrical_high);
  TEST_ASSERT_FALSE(decoded.right_electrical_high);
  TEST_ASSERT_TRUE(decoded.side_configured);
  TEST_ASSERT_TRUE(decoded.side_electrical_high);
  TEST_ASSERT_TRUE(decoded.side_2_configured);
  TEST_ASSERT_TRUE(decoded.side_2_electrical_high);
  TEST_ASSERT_TRUE(decoded.side_3_configured);
  TEST_ASSERT_FALSE(decoded.side_3_electrical_high);

  robot::UartPacket legacy_packet = packet;
  legacy_packet.header.payload_size =
      robot::kLegacyRearLineSensorPayloadSize;
  legacy_packet.header.integrity_crc16 =
      robot::calculatePacketIntegrity(legacy_packet);
  decoded.side_3_configured = true;
  decoded.side_3_electrical_high = true;
  TEST_ASSERT_TRUE(
      robot::decodeRearLineSensorPacket(legacy_packet, decoded));
  TEST_ASSERT_FALSE(decoded.side_3_configured);
  TEST_ASSERT_FALSE(decoded.side_3_electrical_high);

  packet.payload[4] ^= robot::kRearLineSensorRightHighFlag;
  TEST_ASSERT_FALSE(robot::decodeRearLineSensorPacket(packet, decoded));
  TEST_ASSERT_FALSE(decoded.configured);
}

void test_laser_distance_packet_round_trips_and_rejects_corruption() {
  robot::LaserDistanceSnapshot expected{};
  expected.captured_at_ms = 12345U;
  expected.distance_mm = 287U;
  expected.measurement_sequence = 91U;
  expected.configured = true;
  expected.initialized = true;
  expected.ranging = true;
  expected.data_valid = true;
  expected.profile = robot::LaserDistanceProfile::HighAccuracy;
  expected.sensor_range_status = 0U;
  expected.driver_status = -4;
  expected.successful_measurement_count = 4000U;
  expected.failed_measurement_count = 7U;
  expected.consecutive_failed_measurements = 2U;
  expected.acquisition_duration_us = 830U;
  expected.maximum_acquisition_duration_us = 5100U;
  expected.sda_gpio = 10;
  expected.scl_gpio = 9;
  expected.i2c_address = 0x29U;
  expected.intermeasurement_period_ms = 200U;

  robot::UartPacket packet =
      robot::makeLaserDistancePacket(expected, 99U);
  robot::LaserDistanceSnapshot decoded{};
  TEST_ASSERT_TRUE(robot::decodeLaserDistancePacket(packet, decoded));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::UartMessageType::LaserDistanceSnapshot),
      static_cast<std::uint8_t>(packet.header.message_type));
  TEST_ASSERT_EQUAL_UINT16(99U, packet.header.sequence);
  TEST_ASSERT_EQUAL_UINT32(expected.captured_at_ms,
                           decoded.captured_at_ms);
  TEST_ASSERT_EQUAL_UINT16(expected.distance_mm, decoded.distance_mm);
  TEST_ASSERT_EQUAL_UINT16(expected.measurement_sequence,
                           decoded.measurement_sequence);
  TEST_ASSERT_TRUE(decoded.configured);
  TEST_ASSERT_TRUE(decoded.initialized);
  TEST_ASSERT_TRUE(decoded.ranging);
  TEST_ASSERT_TRUE(decoded.data_valid);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::LaserDistanceProfile::HighAccuracy),
      static_cast<std::uint8_t>(decoded.profile));
  TEST_ASSERT_EQUAL_UINT8(0U, decoded.sensor_range_status);
  TEST_ASSERT_EQUAL_INT8(-4, decoded.driver_status);
  TEST_ASSERT_EQUAL_UINT32(expected.successful_measurement_count,
                           decoded.successful_measurement_count);
  TEST_ASSERT_EQUAL_UINT32(expected.failed_measurement_count,
                           decoded.failed_measurement_count);
  TEST_ASSERT_EQUAL_UINT16(expected.consecutive_failed_measurements,
                           decoded.consecutive_failed_measurements);
  TEST_ASSERT_EQUAL_UINT32(expected.acquisition_duration_us,
                           decoded.acquisition_duration_us);
  TEST_ASSERT_EQUAL_UINT32(expected.maximum_acquisition_duration_us,
                           decoded.maximum_acquisition_duration_us);
  TEST_ASSERT_EQUAL_INT8(10, decoded.sda_gpio);
  TEST_ASSERT_EQUAL_INT8(9, decoded.scl_gpio);
  TEST_ASSERT_EQUAL_UINT8(0x29U, decoded.i2c_address);
  TEST_ASSERT_EQUAL_UINT16(200U, decoded.intermeasurement_period_ms);

  packet.payload[4] ^= 0x01U;
  TEST_ASSERT_FALSE(robot::decodeLaserDistancePacket(packet, decoded));
  TEST_ASSERT_FALSE(decoded.configured);
}

robot::LaserDistanceSnapshot validLaserDistance(
    const std::uint16_t distance_mm) {
  robot::LaserDistanceSnapshot snapshot{};
  snapshot.configured = true;
  snapshot.initialized = true;
  snapshot.ranging = true;
  snapshot.data_valid = true;
  snapshot.sensor_range_status = robot::kVl53l0xValidRangeStatus;
  snapshot.distance_mm = distance_mm;
  return snapshot;
}

void test_habitat_distance_stop_is_locked_until_configured() {
  robot::HabitatDistanceStopState state{};
  const robot::LaserDistanceSnapshot snapshot =
      validLaserDistance(300U);
  const robot::HabitatDistanceStopUpdate update =
      robot::updateHabitatDistanceStop(
          state, {}, &snapshot, 100U, 110U);
  TEST_ASSERT_FALSE(update.configuration_valid);
  TEST_ASSERT_TRUE(update.should_stop);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::HabitatDistanceStopReason::ConfigurationIncomplete),
      static_cast<std::uint8_t>(update.reason));
}

void test_habitat_distance_stop_rejects_invalid_and_stale_data() {
  const robot::HabitatDistanceStopConfig config{200U, 100U};
  robot::HabitatDistanceStopState state{};
  robot::LaserDistanceSnapshot snapshot =
      validLaserDistance(300U);
  snapshot.data_valid = false;
  robot::HabitatDistanceStopUpdate update =
      robot::updateHabitatDistanceStop(
          state, config, &snapshot, 100U, 110U);
  TEST_ASSERT_TRUE(update.should_stop);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::HabitatDistanceStopReason::MeasurementInvalid),
      static_cast<std::uint8_t>(update.reason));

  update = robot::updateHabitatDistanceStop(
      state, config, &snapshot, 100U, 201U);
  TEST_ASSERT_TRUE(update.should_stop);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::HabitatDistanceStopReason::MeasurementStale),
      static_cast<std::uint8_t>(update.reason));

  snapshot = validLaserDistance(300U);
  update = robot::updateHabitatDistanceStop(
      state, config, &snapshot, 100U, 201U);
  TEST_ASSERT_TRUE(update.should_stop);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::HabitatDistanceStopReason::MeasurementStale),
      static_cast<std::uint8_t>(update.reason));
}

void test_habitat_distance_stop_latches_at_threshold() {
  const robot::HabitatDistanceStopConfig config{200U, 100U};
  robot::HabitatDistanceStopState state{};
  robot::LaserDistanceSnapshot snapshot =
      validLaserDistance(201U);
  robot::HabitatDistanceStopUpdate update =
      robot::updateHabitatDistanceStop(
          state, config, &snapshot, 100U, 110U);
  TEST_ASSERT_FALSE(update.should_stop);
  TEST_ASSERT_FALSE(update.target_reached);

  snapshot.distance_mm = 200U;
  update = robot::updateHabitatDistanceStop(
      state, config, &snapshot, 120U, 125U);
  TEST_ASSERT_TRUE(update.should_stop);
  TEST_ASSERT_TRUE(update.target_reached);

  snapshot.distance_mm = 500U;
  update = robot::updateHabitatDistanceStop(
      state, config, &snapshot, 130U, 135U);
  TEST_ASSERT_TRUE(update.should_stop);
  TEST_ASSERT_TRUE(update.target_reached);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::HabitatDistanceStopReason::TargetDistanceReached),
      static_cast<std::uint8_t>(update.reason));

  robot::resetHabitatDistanceStop(state);
  update = robot::updateHabitatDistanceStop(
      state, config, &snapshot, 140U, 145U);
  TEST_ASSERT_FALSE(update.should_stop);
  TEST_ASSERT_FALSE(update.target_reached);
}

void test_habitat_distance_stop_age_handles_millisecond_wrap() {
  const robot::HabitatDistanceStopConfig config{200U, 20U};
  robot::HabitatDistanceStopState state{};
  const robot::LaserDistanceSnapshot snapshot =
      validLaserDistance(300U);
  const robot::HabitatDistanceStopUpdate update =
      robot::updateHabitatDistanceStop(
          state, config, &snapshot, UINT32_MAX - 5U, 4U);

  TEST_ASSERT_FALSE(update.should_stop);
  TEST_ASSERT_TRUE(update.measurement_available);
  TEST_ASSERT_EQUAL_UINT32(10U, update.sample_age_ms);
}

void test_habitat_pieces_defaults_to_requested_duty_but_stays_locked() {
  robot::HabitatPiecesConfig config{};

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 0.12F, config.line_follow_duty);
  TEST_ASSERT_FALSE(robot::habitatPiecesConfigValid(config, 1.0F));
  config.lss2_detection_delay_ms = 100U;
  TEST_ASSERT_FALSE(robot::habitatPiecesConfigValid(config, 1.0F));
  config.run_timeout_ms = 1000U;
  TEST_ASSERT_FALSE(robot::habitatPiecesConfigValid(config, 1.0F));
  config.reverse_duty = 0.2F;
  TEST_ASSERT_FALSE(robot::habitatPiecesConfigValid(config, 1.0F));
  config.reverse_duration_ms = 200U;
  TEST_ASSERT_TRUE(robot::habitatPiecesConfigValid(config, 1.0F));
  config.run_timeout_ms = config.lss2_detection_delay_ms;
  TEST_ASSERT_FALSE(robot::habitatPiecesConfigValid(config, 1.0F));

  robot::HabitatPiecesAutonomy autonomy{};
  robot::startHabitatPiecesAutonomy(autonomy, 25U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::HabitatPiecesState::LineFollowing),
      static_cast<std::uint8_t>(autonomy.state));
  TEST_ASSERT_EQUAL_STRING(
      "LINE_FOLLOWING",
      robot::habitatPiecesStateName(autonomy.state));
}

void test_habitat_pieces_ignores_lss2_until_detection_delay() {
  robot::HabitatPiecesConfig config{};
  config.lss2_detection_delay_ms = 100U;
  config.run_timeout_ms = 1000U;
  config.reverse_duty = 0.2F;
  config.reverse_duration_ms = 100U;
  robot::HabitatPiecesAutonomy autonomy{};
  robot::startHabitatPiecesAutonomy(autonomy, 100U);

  robot::HabitatPiecesUpdate update =
      robot::updateHabitatPiecesAutonomy(autonomy, config, true, 199U);
  TEST_ASSERT_TRUE(update.should_line_follow);
  TEST_ASSERT_FALSE(update.should_stop);
  TEST_ASSERT_FALSE(update.lss2_detection_armed);
  TEST_ASSERT_TRUE(update.lss2_black);
  TEST_ASSERT_FALSE(update.target_reached);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::HabitatPiecesState::LineFollowing),
      static_cast<std::uint8_t>(update.state));

  update = robot::updateHabitatPiecesAutonomy(autonomy, config, true, 200U);
  TEST_ASSERT_FALSE(update.should_line_follow);
  TEST_ASSERT_FALSE(update.should_stop);
  TEST_ASSERT_TRUE(update.should_reverse);
  TEST_ASSERT_TRUE(update.lss2_detection_armed);
  TEST_ASSERT_TRUE(update.target_reached);
  TEST_ASSERT_TRUE(update.transitioned);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::HabitatPiecesStopReason::Lss2BlackDetected),
      static_cast<std::uint8_t>(update.stop_reason));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::HabitatPiecesState::Reversing),
      static_cast<std::uint8_t>(update.state));
}

void test_habitat_pieces_detects_black_after_delay() {
  robot::HabitatPiecesConfig config{};
  config.lss2_detection_delay_ms = 100U;
  config.run_timeout_ms = 1000U;
  config.reverse_duty = 0.2F;
  config.reverse_duration_ms = 100U;
  robot::HabitatPiecesAutonomy autonomy{};
  robot::startHabitatPiecesAutonomy(autonomy, 100U);

  robot::HabitatPiecesUpdate update =
      robot::updateHabitatPiecesAutonomy(autonomy, config, false, 200U);
  TEST_ASSERT_TRUE(update.should_line_follow);
  TEST_ASSERT_TRUE(update.lss2_detection_armed);
  TEST_ASSERT_FALSE(update.target_reached);

  update = robot::updateHabitatPiecesAutonomy(autonomy, config, true, 250U);
  TEST_ASSERT_FALSE(update.should_line_follow);
  TEST_ASSERT_FALSE(update.should_stop);
  TEST_ASSERT_TRUE(update.should_reverse);
  TEST_ASSERT_TRUE(update.target_reached);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::HabitatPiecesState::Reversing),
      static_cast<std::uint8_t>(update.state));

  update = robot::updateHabitatPiecesAutonomy(autonomy, config, false, 349U);
  TEST_ASSERT_FALSE(update.should_line_follow);
  TEST_ASSERT_FALSE(update.should_stop);
  TEST_ASSERT_TRUE(update.should_reverse);
  TEST_ASSERT_EQUAL_UINT32(99U, autonomy.reverse_elapsed_ms);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::HabitatPiecesState::Reversing),
      static_cast<std::uint8_t>(update.state));

  update = robot::updateHabitatPiecesAutonomy(autonomy, config, false, 350U);
  TEST_ASSERT_FALSE(update.should_line_follow);
  TEST_ASSERT_FALSE(update.should_reverse);
  TEST_ASSERT_TRUE(update.should_stop);
  TEST_ASSERT_TRUE(update.target_reached);
  TEST_ASSERT_TRUE(update.transitioned);
  TEST_ASSERT_EQUAL_UINT32(100U, autonomy.reverse_elapsed_ms);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::HabitatPiecesState::Complete),
      static_cast<std::uint8_t>(update.state));
}

void test_habitat_pieces_times_out_if_lss2_never_detects_black() {
  robot::HabitatPiecesConfig config{};
  config.lss2_detection_delay_ms = 10U;
  config.run_timeout_ms = 50U;
  config.reverse_duty = 0.2F;
  config.reverse_duration_ms = 100U;
  robot::HabitatPiecesAutonomy autonomy{};
  robot::startHabitatPiecesAutonomy(autonomy, 100U);

  robot::HabitatPiecesUpdate update =
      robot::updateHabitatPiecesAutonomy(autonomy, config, false, 149U);
  TEST_ASSERT_TRUE(update.should_line_follow);
  TEST_ASSERT_TRUE(update.lss2_detection_armed);
  TEST_ASSERT_FALSE(autonomy.timed_out);
  TEST_ASSERT_EQUAL_UINT32(49U, autonomy.run_elapsed_ms);

  update = robot::updateHabitatPiecesAutonomy(autonomy, config, false, 150U);
  TEST_ASSERT_FALSE(update.should_line_follow);
  TEST_ASSERT_TRUE(update.should_stop);
  TEST_ASSERT_TRUE(update.transitioned);
  TEST_ASSERT_TRUE(autonomy.timed_out);
  TEST_ASSERT_FALSE(update.target_reached);
  TEST_ASSERT_EQUAL_UINT32(50U, autonomy.run_elapsed_ms);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::HabitatPiecesStopReason::RunTimeout),
      static_cast<std::uint8_t>(update.stop_reason));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::HabitatPiecesState::Fault),
      static_cast<std::uint8_t>(update.state));
}

robot::HabitatPlacementConfig validHabitatPlacementConfig() {
  robot::HabitatPlacementConfig config{};
  config.reverse_line_follow_duty = 0.2F;
  config.lss1_timeout_ms = 100U;
  config.post_lss1_delay_ms = 10U;
  config.counter_clockwise_angle_deg = 90.0F;
  config.counter_clockwise_timeout_ms = 100U;
  config.forward_to_slide_duty = 0.2F;
  config.forward_to_slide_duration_ms = 10U;
  config.stepper_down_speed_steps_per_second = 100U;
  config.stepper_down_timeout_ms = 100U;
  config.pusher_open_settle_ms = 10U;
  config.push_forward_duty = 0.2F;
  config.push_forward_duration_ms = 10U;
  config.reverse_retreat_duty = 0.2F;
  config.reverse_retreat_duration_ms = 10U;
  config.clockwise_angle_deg = 90.0F;
  config.clockwise_timeout_ms = 100U;
  config.post_clockwise_reverse_duty = 0.2F;
  config.post_clockwise_reverse_duration_ms = 10U;
  config.post_clockwise_strafe_left_duty = 0.2F;
  config.post_clockwise_strafe_left_duration_ms = 10U;
  config.post_clockwise_delay_ms = 10U;
  config.exit_forward_duty = 0.2F;
  config.exit_forward_duration_ms = 10U;
  config.post_forward_delay_ms = 10U;
  config.strafe_right_duty = 0.2F;
  config.strafe_right_timeout_ms = 100U;
  return config;
}

void test_habitat_placement_mode_parses_and_allows_motion() {
  robot::RobotTestMode mode = robot::RobotTestMode::Disabled;
  TEST_ASSERT_TRUE(robot::parseRobotTestMode("habitat-placement", mode));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(robot::RobotTestMode::HabitatPlacement),
      static_cast<std::uint8_t>(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeAllowsMotion(mode));
  TEST_ASSERT_TRUE(robot::robotTestModeRequiresRearLink(mode));
}

void test_habitat_placement_config_starts_locked() {
  robot::HabitatPlacementConfig config{};
  TEST_ASSERT_FALSE(
      robot::habitatPlacementConfigValid(config, 0.8F, 2000U));
  config = validHabitatPlacementConfig();
  TEST_ASSERT_TRUE(
      robot::habitatPlacementConfigValid(config, 0.8F, 2000U));
  config.strafe_right_duty = 0.9F;
  TEST_ASSERT_FALSE(
      robot::habitatPlacementConfigValid(config, 0.8F, 2000U));
  config = validHabitatPlacementConfig();
  config.post_clockwise_strafe_left_duration_ms = 0U;
  TEST_ASSERT_FALSE(
      robot::habitatPlacementConfigValid(config, 0.8F, 2000U));
}

void test_habitat_placement_runs_requested_sequence() {
  const robot::HabitatPlacementConfig config =
      validHabitatPlacementConfig();
  robot::HabitatPlacementAutonomy autonomy{};
  robot::startHabitatPlacementAutonomy(autonomy, 0U);

  robot::HabitatPlacementInputs inputs{};
  robot::HabitatPlacementUpdate update =
      robot::updateHabitatPlacementAutonomy(autonomy, inputs, config, 0U);
  TEST_ASSERT_TRUE(update.should_reverse_line_follow);

  inputs.lss1_black = true;
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 1U);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::HabitatPlacementState::PostLss1Delay),
      static_cast<std::uint8_t>(update.state));
  TEST_ASSERT_TRUE(update.should_stop_drive);

  inputs = {};
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 11U);
  TEST_ASSERT_TRUE(update.should_turn_counter_clockwise);
  inputs.counter_clockwise_turn_complete = true;
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 12U);
  TEST_ASSERT_TRUE(update.should_drive_forward_to_slide);
  inputs = {};
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 22U);
  TEST_ASSERT_TRUE(update.should_lower_slide);
  inputs.bottom_limit_active = true;
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 23U);
  TEST_ASSERT_TRUE(update.should_open_pusher);
  inputs.pusher_open_commanded = true;
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 24U);
  TEST_ASSERT_TRUE(update.should_stop_drive);
  inputs = {};
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 34U);
  TEST_ASSERT_TRUE(update.should_drive_forward_push);
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 44U);
  TEST_ASSERT_TRUE(update.should_drive_reverse_retreat);
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 54U);
  TEST_ASSERT_TRUE(update.should_turn_clockwise);
  inputs.clockwise_turn_complete = true;
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 55U);
  TEST_ASSERT_TRUE(update.should_drive_reverse_after_clockwise);
  inputs = {};
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 65U);
  TEST_ASSERT_TRUE(update.should_strafe_left_after_clockwise);
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 75U);
  TEST_ASSERT_TRUE(update.should_stop_drive);
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 85U);
  TEST_ASSERT_TRUE(update.should_drive_forward_exit);
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 95U);
  TEST_ASSERT_TRUE(update.should_stop_drive);
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 105U);
  TEST_ASSERT_TRUE(update.should_strafe_right);
  inputs.front_line_black = true;
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 106U);
  TEST_ASSERT_TRUE(update.should_close_pusher);
  inputs.pusher_closed_commanded = true;
  update = robot::updateHabitatPlacementAutonomy(autonomy, inputs, config,
                                                 107U);
  TEST_ASSERT_TRUE(update.complete);
  TEST_ASSERT_TRUE(update.should_stop_drive);
}

void test_habitat_placement_lss1_timeout_faults_stopped() {
  const robot::HabitatPlacementConfig config =
      validHabitatPlacementConfig();
  robot::HabitatPlacementAutonomy autonomy{};
  robot::startHabitatPlacementAutonomy(autonomy, 10U);
  const robot::HabitatPlacementUpdate update =
      robot::updateHabitatPlacementAutonomy(
          autonomy, robot::HabitatPlacementInputs{}, config, 110U);
  TEST_ASSERT_TRUE(update.faulted);
  TEST_ASSERT_TRUE(update.should_stop_drive);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<std::uint8_t>(
          robot::HabitatPlacementFaultReason::Lss1Timeout),
      static_cast<std::uint8_t>(update.fault_reason));
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
  RUN_TEST(test_reverse_rear_observation_swaps_physical_sensor_sides);
  RUN_TEST(test_reverse_rear_config_preserves_pid_and_negates_base);
  RUN_TEST(
      test_reverse_rear_follow_drives_backward_and_steers_in_travel_frame);
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
  RUN_TEST(test_open_loop_clockwise_rotation_uses_mecanum_signs);
  RUN_TEST(test_valid_rear_command_is_accepted);
  RUN_TEST(test_rear_command_keeps_high_accuracy_when_stale);
  RUN_TEST(test_corrupt_rear_packet_is_rejected);
  RUN_TEST(test_stale_rear_command_stops_motors);
  RUN_TEST(test_explicit_stop_packet_stops_motors);
  RUN_TEST(test_valid_funnel_command_is_accepted);
  RUN_TEST(test_stale_funnel_command_stops_motor);
  RUN_TEST(test_stale_disabled_funnel_command_does_not_report_motion_fault);
  RUN_TEST(test_corrupt_funnel_packet_is_rejected);
  RUN_TEST(test_solar_hook_command_packet_round_trips);
  RUN_TEST(test_solar_hook_command_rejects_invalid_payload);
  RUN_TEST(test_mode_manager_starts_disabled);
  RUN_TEST(test_mode_manager_rejects_drive_while_disabled);
  RUN_TEST(test_mode_manager_accepts_sensor_mode_without_motors);
  RUN_TEST(test_mechanism_mode_is_not_sensor_only);
  RUN_TEST(test_autonomous_solar_mode_allows_motion_and_requires_rear_link);
  RUN_TEST(test_rear_line_modes_parse_and_enforce_motion_policy);
  RUN_TEST(test_tower_pieces_mode_parses_and_allows_distributed_motion);
  RUN_TEST(test_peg_finder_mode_parses_and_allows_distributed_motion);
  RUN_TEST(test_time_trial_mode_parses_and_allows_distributed_motion);
  RUN_TEST(test_habitat_pieces_mode_parses_and_allows_distributed_motion);
  RUN_TEST(test_imu_turn_mode_is_explicit_and_requires_rear_link);
  RUN_TEST(test_imu_strafe_mode_is_explicit_and_requires_rear_link);
  RUN_TEST(
      test_imu_heading_hold_config_requires_bounded_combined_duty);
  RUN_TEST(test_imu_heading_hold_captures_target_once);
  RUN_TEST(
      test_imu_heading_hold_pd_correction_clamps_and_damps_rate);
  RUN_TEST(
      test_imu_heading_hold_faults_on_nonfinite_measurement_and_stops);
  RUN_TEST(
      test_imu_turn_config_starts_locked_until_every_value_is_configured);
  RUN_TEST(test_imu_turn_start_captures_a_continuous_relative_target);
  RUN_TEST(test_imu_turn_ccw_target_stays_at_saved_start_minus_offset);
  RUN_TEST(test_clockwise_angle_uses_measured_imu_polarity);
  RUN_TEST(
      test_imu_turn_pd_output_clamps_and_rate_damping_opposes_motion);
  RUN_TEST(
      test_imu_turn_requires_low_angle_and_rate_for_full_settling_time);
  RUN_TEST(test_imu_turn_leaves_settling_if_either_condition_breaks);
  RUN_TEST(test_imu_turn_timeout_faults_and_stop_is_terminal);
  RUN_TEST(test_imu_turn_ignores_a_loop_timestamp_just_before_start);
  RUN_TEST(test_imu_turn_faults_instead_of_emitting_nonfinite_output);
  RUN_TEST(
      test_imu_recovery_pauses_saves_heading_and_requires_fresh_samples);
  RUN_TEST(test_imu_recovery_resets_confirmation_and_times_out);
  RUN_TEST(test_imu_recovery_defers_controller_timeouts);
  RUN_TEST(
      test_time_trial_config_allows_skipped_or_safe_transition_strafe);
  RUN_TEST(
      test_time_trial_runs_solar_strafe_tower_and_peg_finder_in_order);
  RUN_TEST(
      test_time_trial_hands_solar_line_follow_directly_to_tower);
  RUN_TEST(test_time_trial_propagates_included_mode_faults);
  RUN_TEST(test_peg_finder_config_requires_safe_duties_angle_and_timings);
  RUN_TEST(test_peg_finder_clockwise_turn_waits_for_imu_completion);
  RUN_TEST(
      test_peg_finder_opens_claws_in_configured_order_then_reverses_funnel);
  RUN_TEST(test_peg_finder_funnel_timeout_faults_without_limit);
  RUN_TEST(
      test_peg_finder_does_not_start_funnel_when_limit_already_pressed);
  RUN_TEST(test_tower_pieces_config_requires_duties_and_timings);
  RUN_TEST(test_tower_pieces_counts_distinct_side_line_rising_edges);
  RUN_TEST(test_tower_pieces_does_not_count_a_high_level_present_at_start);
  RUN_TEST(test_tower_pieces_timeout_stops_before_second_side_line);
  RUN_TEST(
      test_tower_side_crossing_requires_cooldown_and_offline_rearm);
  RUN_TEST(test_tower_pre_stepper_delay_holds_slide_stopped);
  RUN_TEST(test_tower_pieces_runs_full_sequence_in_order);
  RUN_TEST(test_tower_pieces_rotation_waits_for_imu_completion);
  RUN_TEST(test_tower_pieces_shimmy_hands_off_to_tail_and_can_skip_reverse);
  RUN_TEST(test_tower_pieces_conflicting_stepper_limits_fault);
  RUN_TEST(test_tower_pieces_shimmy_timeout_spans_direction_changes);
  RUN_TEST(test_emergency_stop_works_from_any_mode);
  RUN_TEST(test_command_validation_rejects_out_of_range_duty);
  RUN_TEST(test_command_validation_accepts_drive_test_duty_0_7);
  RUN_TEST(test_command_validation_rejects_overlong_duration);
  RUN_TEST(test_command_validation_rejects_malformed_motor_id);
  RUN_TEST(test_command_validation_rejects_invalid_pid_value);
  RUN_TEST(test_command_validation_rejects_mode_incompatible_motor_command);
  RUN_TEST(test_event_log_stores_newest_events_and_wraps);
  RUN_TEST(test_motion_diagnostics_retains_newest_samples_in_time_order);
  RUN_TEST(test_motion_diagnostics_freeze_preserves_trigger_and_samples);
  RUN_TEST(test_motion_diagnostics_tracks_loop_and_web_failure_evidence);
  RUN_TEST(test_motion_diagnostics_json_exposes_commands_pwm_and_timing);
  RUN_TEST(test_solar_contact_config_validation);
  RUN_TEST(test_solar_retry_state_names_are_exposed);
  RUN_TEST(test_solar_front_only_at_first_timeout_begins_adjustment);
  RUN_TEST(test_solar_first_timeout_faults_without_front_only_contact);
  RUN_TEST(test_solar_adjustment_runs_left_then_forward_then_right);
  RUN_TEST(test_solar_zero_adjustment_durations_transition_immediately);
  RUN_TEST(test_solar_second_strafe_times_out_without_another_adjustment);
  RUN_TEST(test_solar_second_strafe_timeout_is_independently_adjustable);
  RUN_TEST(test_solar_all_hit_has_priority_in_every_contact_motion_state);
  RUN_TEST(
      test_solar_contact_drives_forward_then_strafes_until_rear_line);
  RUN_TEST(
      test_solar_post_contact_forward_stops_if_rear_line_is_already_detected);
  RUN_TEST(test_solar_post_contact_delays_are_independently_adjustable);
  RUN_TEST(test_solar_non_contact_state_is_unchanged);
  RUN_TEST(test_solar_detector_no_beacon_does_not_confirm);
  RUN_TEST(test_solar_detector_brief_spike_does_not_confirm);
  RUN_TEST(test_solar_detector_sustained_beacon_confirms);
  RUN_TEST(test_solar_detector_hysteresis_holds_until_release_threshold);
  RUN_TEST(test_solar_detector_ignore_window_blocks_confirmation);
  RUN_TEST(test_solar_detector_reset_clears_state);
  RUN_TEST(test_telemetry_json_contains_required_fields_and_booleans);
  RUN_TEST(test_esp1_status_packet_round_trips);
  RUN_TEST(test_hc_sr04_echo_conversion_and_valid_range);
  RUN_TEST(
      test_rear_line_sensor_packet_round_trips_and_rejects_corruption);
  RUN_TEST(
      test_laser_distance_packet_round_trips_and_rejects_corruption);
  RUN_TEST(test_habitat_distance_stop_is_locked_until_configured);
  RUN_TEST(
      test_habitat_distance_stop_rejects_invalid_and_stale_data);
  RUN_TEST(test_habitat_distance_stop_latches_at_threshold);
  RUN_TEST(test_habitat_distance_stop_age_handles_millisecond_wrap);
  RUN_TEST(test_habitat_pieces_defaults_to_requested_duty_but_stays_locked);
  RUN_TEST(test_habitat_pieces_ignores_lss2_until_detection_delay);
  RUN_TEST(test_habitat_pieces_detects_black_after_delay);
  RUN_TEST(test_habitat_pieces_times_out_if_lss2_never_detects_black);
  RUN_TEST(test_habitat_placement_mode_parses_and_allows_motion);
  RUN_TEST(test_habitat_placement_config_starts_locked);
  RUN_TEST(test_habitat_placement_runs_requested_sequence);
  RUN_TEST(test_habitat_placement_lss1_timeout_faults_stopped);
  return UNITY_END();
}
