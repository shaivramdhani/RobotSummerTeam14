#include "common/RobotTestMode.h"

#include <cstring>

namespace robot {

namespace {

bool sameModeToken(const char* lhs, const char* rhs) {
  if (lhs == nullptr || rhs == nullptr) {
    return false;
  }

  while (*lhs != '\0' && *rhs != '\0') {
    char left = *lhs;
    char right = *rhs;
    if (left >= 'A' && left <= 'Z') {
      left = static_cast<char>(left - 'A' + 'a');
    }
    if (right >= 'A' && right <= 'Z') {
      right = static_cast<char>(right - 'A' + 'a');
    }
    if (left == '_') {
      left = '-';
    }
    if (right == '_') {
      right = '-';
    }
    if (left != right) {
      return false;
    }
    ++lhs;
    ++rhs;
  }

  return *lhs == '\0' && *rhs == '\0';
}

}  // namespace

const char* robotTestModeName(const RobotTestMode mode) {
  switch (mode) {
    case RobotTestMode::Disabled:
      return "DISABLED";
    case RobotTestMode::SensorMonitor:
      return "SENSOR_MONITOR";
    case RobotTestMode::SingleMotorTest:
      return "SINGLE_MOTOR_TEST";
    case RobotTestMode::ManualDriveTest:
      return "MANUAL_DRIVE_TEST";
    case RobotTestMode::DistributedDriveTest:
      return "DISTRIBUTED_DRIVE_TEST";
    case RobotTestMode::LineSensorTest:
      return "LINE_SENSOR_TEST";
    case RobotTestMode::LineFollowTest:
      return "LINE_FOLLOW_TEST";
    case RobotTestMode::MechanismTest:
      return "MECHANISM_TEST";
    case RobotTestMode::AutonomousDryRun:
      return "AUTONOMOUS_DRY_RUN";
    case RobotTestMode::AutonomousSolarPanel:
      return "AUTONOMOUS_SOLAR_PANEL";
  }

  return "DISABLED";
}

bool parseRobotTestMode(const char* text, RobotTestMode& mode) {
  if (sameModeToken(text, "disabled") || sameModeToken(text, "stop")) {
    mode = RobotTestMode::Disabled;
    return true;
  }
  if (sameModeToken(text, "sensors") ||
      sameModeToken(text, "sensor-monitor")) {
    mode = RobotTestMode::SensorMonitor;
    return true;
  }
  if (sameModeToken(text, "single-motor") ||
      sameModeToken(text, "single-motor-test")) {
    mode = RobotTestMode::SingleMotorTest;
    return true;
  }
  if (sameModeToken(text, "manual-drive") ||
      sameModeToken(text, "manual-drive-test")) {
    mode = RobotTestMode::ManualDriveTest;
    return true;
  }
  if (sameModeToken(text, "distributed-drive") ||
      sameModeToken(text, "distributed-drive-test")) {
    mode = RobotTestMode::DistributedDriveTest;
    return true;
  }
  if (sameModeToken(text, "line-sensor") ||
      sameModeToken(text, "line-sensor-test")) {
    mode = RobotTestMode::LineSensorTest;
    return true;
  }
  if (sameModeToken(text, "line-follow") ||
      sameModeToken(text, "line-follow-test")) {
    mode = RobotTestMode::LineFollowTest;
    return true;
  }
  if (sameModeToken(text, "mechanism") ||
      sameModeToken(text, "mechanism-test")) {
    mode = RobotTestMode::MechanismTest;
    return true;
  }
  if (sameModeToken(text, "autonomous-dry-run") ||
      sameModeToken(text, "dry-run")) {
    mode = RobotTestMode::AutonomousDryRun;
    return true;
  }
  if (sameModeToken(text, "autonomous-solar") ||
      sameModeToken(text, "autonomous-solar-panel") ||
      sameModeToken(text, "solar")) {
    mode = RobotTestMode::AutonomousSolarPanel;
    return true;
  }

  return false;
}

bool robotTestModeAllowsMotion(const RobotTestMode mode) {
  return mode == RobotTestMode::SingleMotorTest ||
         mode == RobotTestMode::ManualDriveTest ||
         mode == RobotTestMode::DistributedDriveTest ||
         mode == RobotTestMode::LineFollowTest ||
         mode == RobotTestMode::AutonomousSolarPanel;
}

bool robotTestModeRequiresRearLink(const RobotTestMode mode) {
  return mode == RobotTestMode::DistributedDriveTest ||
         mode == RobotTestMode::LineFollowTest ||
         mode == RobotTestMode::AutonomousSolarPanel;
}

bool robotTestModeIsSensorOnly(const RobotTestMode mode) {
  return mode == RobotTestMode::SensorMonitor ||
         mode == RobotTestMode::LineSensorTest ||
         mode == RobotTestMode::AutonomousDryRun ||
         mode == RobotTestMode::Disabled;
}

}  // namespace robot
